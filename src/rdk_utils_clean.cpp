/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

Copyright (c) 2024，WuChao D-Robotics.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "rdk_utils.h"

#define RDK_CHECK_SUCCESS(value, errmsg)                                         \
    do                                                                           \
    {                                                                            \
        auto ret_code = value;                                                   \
        if (ret_code != 0)                                                       \
        {                                                                        \
            spdlog::error("[ERROR] {}: {} error code: {}", __FILE__, errmsg, ret_code); \
            return ret_code;                                                     \
        }                                                                        \
    } while (0);

// COCO Names
std::vector<std::string> object_names = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"};

// RDKInference类实现
RDKInference::RDKInference(const std::string& model_path) : model_path_(model_path) {
    // 初始化BPU系统
    auto ret = hbSysInit();
    if (ret != 0) {
        spdlog::error("hbSysInit failed: {}", ret);
        throw std::runtime_error("BPU system initialization failed");
    }
    
    // 加载模型
    if (loadModel() != 0) {
        throw std::runtime_error("Failed to load model");
    }
    
    // 获取模型信息
    if (initializeModel() != 0) {
        throw std::runtime_error("Failed to initialize model");
    }
    
    spdlog::info("RDK Inference initialized successfully");
}

RDKInference::~RDKInference() {
    releaseResources();
    hbSysExit();
}

int RDKInference::loadModel() {
    auto begin_time = std::chrono::system_clock::now();
    const char *model_file_name = model_path_.c_str();
    
    auto ret = hbDNNInitializeFromFiles(&packed_dnn_handle_, &model_file_name, 1);
    if (ret != 0) {
        spdlog::error("hbDNNInitializeFromFiles failed: {}", ret);
        return ret;
    }
    
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now() - begin_time).count() / 1000.0;
    spdlog::info("Load D-Robotics model time = {:.2f} ms", duration);
    
    return 0;
}

int RDKInference::initializeModel() {
    // 获取模型句柄
    const char **model_name_list;
    int model_count = 0;
    auto ret = hbDNNGetModelNameList(&model_name_list, &model_count, packed_dnn_handle_);
    if (ret != 0) {
        spdlog::error("hbDNNGetModelNameList failed: {}", ret);
        return ret;
    }
    
    const char *model_name = model_name_list[0];
    spdlog::info("Model name: {}", model_name);
    
    ret = hbDNNGetModelHandle(&dnn_handle_, packed_dnn_handle_, model_name);
    if (ret != 0) {
        spdlog::error("hbDNNGetModelHandle failed: {}", ret);
        return ret;
    }
    
    // 验证输入
    if (validateModelInputs() != 0) {
        return -1;
    }
    
    // 验证输出
    if (validateModelOutputs() != 0) {
        return -1;
    }
    
    return 0;
}

int RDKInference::validateModelInputs() {
    int32_t input_count = 0;
    auto ret = hbDNNGetInputCount(&input_count, dnn_handle_);
    if (ret != 0) {
        spdlog::error("hbDNNGetInputCount failed: {}", ret);
        return ret;
    }
    
    if (input_count > 1) {
        spdlog::error("Model has more than 1 input");
        return -1;
    }
    
    hbDNNTensorProperties input_properties;
    ret = hbDNNGetInputTensorProperties(&input_properties, dnn_handle_, 0);
    if (ret != 0) {
        spdlog::error("hbDNNGetInputTensorProperties failed: {}", ret);
        return ret;
    }
    
    if (input_properties.tensorType != HB_DNN_IMG_TYPE_NV12) {
        spdlog::error("Input tensor type is not NV12");
        return -1;
    }
    
    if (input_properties.tensorLayout != HB_DNN_LAYOUT_NCHW) {
        spdlog::error("Input tensor layout is not NCHW");
        return -1;
    }
    
    if (input_properties.validShape.numDimensions == 4) {
        input_H_ = input_properties.validShape.dimensionSize[2];
        input_W_ = input_properties.validShape.dimensionSize[3];
        spdlog::info("Input shape: ({}, {}, {}, {})", 
                    input_properties.validShape.dimensionSize[0],
                    input_properties.validShape.dimensionSize[1],
                    input_H_, input_W_);
    } else {
        spdlog::error("Input shape is not 4D");
        return -1;
    }
    
    return 0;
}

int RDKInference::validateModelOutputs() {
    int32_t output_count = 0;
    auto ret = hbDNNGetOutputCount(&output_count, dnn_handle_);
    if (ret != 0) {
        spdlog::error("hbDNNGetOutputCount failed: {}", ret);
        return ret;
    }
    
    if (output_count != 6) {
        spdlog::error("Model output count is not 6, got: {}", output_count);
        return -1;
    }
    
    // 打印输出信息
    for (int i = 0; i < 6; i++) {
        hbDNNTensorProperties output_properties;
        ret = hbDNNGetOutputTensorProperties(&output_properties, dnn_handle_, i);
        if (ret != 0) {
            spdlog::error("hbDNNGetOutputTensorProperties failed for output {}: {}", i, ret);
            return ret;
        }
        
        spdlog::info("output[{}] shape: ({}, {}, {}, {})", i,
                    output_properties.validShape.dimensionSize[0],
                    output_properties.validShape.dimensionSize[1], 
                    output_properties.validShape.dimensionSize[2],
                    output_properties.validShape.dimensionSize[3]);
    }
    
    return 0;
}

cv::Mat RDKInference::preprocessImage(const cv::Mat& img, float& y_scale, float& x_scale, int& x_shift, int& y_shift) {
    auto begin_time = std::chrono::system_clock::now();
    cv::Mat resize_img;
    
    // 使用LetterBox方式预处理
    x_scale = std::min(1.0 * input_H_ / img.rows, 1.0 * input_W_ / img.cols);
    y_scale = x_scale;
    
    if (x_scale <= 0 || y_scale <= 0) {
        throw std::runtime_error("Invalid scale factor");
    }
    
    int new_w = img.cols * x_scale;
    x_shift = (input_W_ - new_w) / 2;
    int x_other = input_W_ - new_w - x_shift;
    
    int new_h = img.rows * y_scale;
    y_shift = (input_H_ - new_h) / 2;
    int y_other = input_H_ - new_h - y_shift;
    
    cv::Size targetSize(new_w, new_h);
    cv::resize(img, resize_img, targetSize);
    cv::copyMakeBorder(resize_img, resize_img, y_shift, y_other, x_shift, x_other,
                      cv::BORDER_CONSTANT, cv::Scalar(127, 127, 127));
    
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now() - begin_time).count() / 1000.0;
    spdlog::debug("Preprocess (LetterBox) time = {:.2f} ms", duration);
    
    return resize_img;
}

cv::Mat RDKInference::convertBGRToNV12(const cv::Mat& img) {
    auto begin_time = std::chrono::system_clock::now();
    
    cv::Mat img_nv12;
    cv::Mat yuv_mat;
    cv::cvtColor(img, yuv_mat, cv::COLOR_BGR2YUV_I420);
    uint8_t *yuv = yuv_mat.ptr<uint8_t>();
    img_nv12 = cv::Mat(input_H_ * 3 / 2, input_W_, CV_8UC1);
    uint8_t *ynv12 = img_nv12.ptr<uint8_t>();
    int uv_height = input_H_ / 2;
    int uv_width = input_W_ / 2;
    int y_size = input_H_ * input_W_;
    memcpy(ynv12, yuv, y_size);
    uint8_t *nv12 = ynv12 + y_size;
    uint8_t *u_data = yuv + y_size;
    uint8_t *v_data = u_data + uv_height * uv_width;
    for (int i = 0; i < uv_width * uv_height; i++) {
        *nv12++ = *u_data++;
        *nv12++ = *v_data++;
    }
    
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now() - begin_time).count() / 1000.0;
    spdlog::debug("BGR to NV12 time = {:.2f} ms", duration);
    
    return img_nv12;
}

int RDKInference::runInference(const cv::Mat& img, std::vector<DetectionResult>& results) {
    // 预处理
    float y_scale, x_scale;
    int x_shift, y_shift;
    cv::Mat resize_img = preprocessImage(img, y_scale, x_scale, x_shift, y_shift);
    cv::Mat img_nv12 = convertBGRToNV12(resize_img);
    
    // 准备输入tensor
    hbDNNTensorProperties input_properties;
    auto ret = hbDNNGetInputTensorProperties(&input_properties, dnn_handle_, 0);
    if (ret != 0) {
        spdlog::error("hbDNNGetInputTensorProperties failed: {}", ret);
        return ret;
    }
    
    hbDNNTensor input;
    input.properties = input_properties;
    hbSysAllocCachedMem(&input.sysMem[0], int(3 * input_H_ * input_W_ / 2));
    memcpy(input.sysMem[0].virAddr, img_nv12.ptr<uint8_t>(), int(3 * input_H_ * input_W_ / 2));
    hbSysFlushMem(&input.sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
    
    // 准备输出tensors
    int32_t output_count = 0;
    ret = hbDNNGetOutputCount(&output_count, dnn_handle_);
    if (ret != 0) {
        spdlog::error("hbDNNGetOutputCount failed: {}", ret);
        hbSysFreeMem(&input.sysMem[0]);
        return ret;
    }
    
    hbDNNTensor *output = new hbDNNTensor[output_count];
    for (int i = 0; i < output_count; i++) {
        hbDNNTensorProperties &output_properties = output[i].properties;
        hbDNNGetOutputTensorProperties(&output_properties, dnn_handle_, i);
        int out_aligned_size = output_properties.alignedByteSize;
        hbSysAllocCachedMem(&output[i].sysMem[0], out_aligned_size);
    }
    
    // 运行推理
    auto begin_time = std::chrono::system_clock::now();
    hbDNNTaskHandle_t task_handle = nullptr;
    hbDNNInferCtrlParam infer_ctrl_param;
    HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);
    ret = hbDNNInfer(&task_handle, &output, &input, dnn_handle_, &infer_ctrl_param);
    if (ret != 0) {
        spdlog::error("hbDNNInfer failed: {}", ret);
        // 清理资源
        hbSysFreeMem(&input.sysMem[0]);
        for (int i = 0; i < output_count; i++) {
            hbSysFreeMem(&output[i].sysMem[0]);
        }
        delete[] output;
        return ret;
    }
    
    hbDNNWaitTaskDone(task_handle, 0);
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now() - begin_time).count() / 1000.0;
    spdlog::debug("Forward time = {:.2f} ms", duration);
    
    // 后处理
    begin_time = std::chrono::system_clock::now();
    std::vector<std::vector<cv::Rect2d>> bboxes(CLASSES_NUM);
    std::vector<std::vector<float>> scores(CLASSES_NUM);
    
    // 处理三个尺度的特征图
    int order[6] = {0, 1, 2, 3, 4, 5};  // 简化处理，保持原有顺序
    postprocessFeatureMap(output, order[0], order[1], 8, input_H_/8, input_W_/8, bboxes, scores);
    postprocessFeatureMap(output, order[2], order[3], 16, input_H_/16, input_W_/16, bboxes, scores);
    postprocessFeatureMap(output, order[4], order[5], 32, input_H_/32, input_W_/32, bboxes, scores);
    
    // NMS
    std::vector<std::vector<int>> indices(CLASSES_NUM);
    performNMS(bboxes, scores, indices);
    
    duration = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now() - begin_time).count() / 1000.0;
    spdlog::debug("Post process time = {:.2f} ms", duration);
    
    // 转换结果格式
    convertToDetectionResults(bboxes, scores, indices, y_scale, x_scale, x_shift, y_shift, results);
    
    // 清理资源
    hbDNNReleaseTask(task_handle);
    hbSysFreeMem(&input.sysMem[0]);
    for (int i = 0; i < output_count; i++) {
        hbSysFreeMem(&output[i].sysMem[0]);
    }
    delete[] output;
    
    return 0;
}

void RDKInference::postprocessFeatureMap(hbDNNTensor* output, int order_cls, int order_bbox, int stride, 
                                       int H_div, int W_div, std::vector<std::vector<cv::Rect2d>>& bboxes, 
                                       std::vector<std::vector<float>>& scores) {
    float CONF_THRES_RAW = -log(1 / SCORE_THRESHOLD - 1);
    
    // 检查反量化类型
    if (output[order_cls].properties.quantiType != NONE) {
        spdlog::error("output[{}] QuantiType is not NONE", order_cls);
        return;
    }
    if (output[order_bbox].properties.quantiType != SCALE) {
        spdlog::error("output[{}] QuantiType is not SCALE", order_bbox);
        return;
    }
    
    // 刷新内存
    hbSysFlushMem(&(output[order_cls].sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
    hbSysFlushMem(&(output[order_bbox].sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
    
    // 获取指针
    auto *cls_raw = reinterpret_cast<float *>(output[order_cls].sysMem[0].virAddr);
    auto *bbox_raw = reinterpret_cast<int32_t *>(output[order_bbox].sysMem[0].virAddr);
    auto *bbox_scale = reinterpret_cast<float *>(output[order_bbox].properties.scale.scaleData);
    
    for (int h = 0; h < H_div; h++) {
        for (int w = 0; w < W_div; w++) {
            float *cur_cls_raw = cls_raw;
            int32_t *cur_bbox_raw = bbox_raw;
            
            // 找到分数的最大值索引
            int cls_id = 0;
            for (int i = 1; i < CLASSES_NUM; i++) {
                if (cur_cls_raw[i] > cur_cls_raw[cls_id]) {
                    cls_id = i;
                }
            }
            
            // 不合格则跳过
            if (cur_cls_raw[cls_id] < CONF_THRES_RAW) {
                cls_raw += CLASSES_NUM;
                bbox_raw += REG * 4;
                continue;
            }
            
            // 计算分数
            float score = 1 / (1 + std::exp(-cur_cls_raw[cls_id]));
            
            // DFL计算
            float ltrb[4], sum, dfl;
            for (int i = 0; i < 4; i++) {
                ltrb[i] = 0.;
                sum = 0.;
                for (int j = 0; j < REG; j++) {
                    int index_id = REG * i + j;
                    dfl = std::exp(float(cur_bbox_raw[index_id]) * bbox_scale[index_id]);
                    ltrb[i] += dfl * j;
                    sum += dfl;
                }
                ltrb[i] /= sum;
            }
            
            // 剔除不合格的框
            if (ltrb[2] + ltrb[0] <= 0 || ltrb[3] + ltrb[1] <= 0) {
                cls_raw += CLASSES_NUM;
                bbox_raw += REG * 4;
                continue;
            }
            
            // dist 2 bbox (ltrb 2 xyxy)
            float x1 = (w + 0.5 - ltrb[0]) * stride;
            float y1 = (h + 0.5 - ltrb[1]) * stride;
            float x2 = (w + 0.5 + ltrb[2]) * stride;
            float y2 = (h + 0.5 + ltrb[3]) * stride;
            
            // 添加到对应类别的vector中
            bboxes[cls_id].push_back(cv::Rect2d(x1, y1, x2 - x1, y2 - y1));
            scores[cls_id].push_back(score);
            
            cls_raw += CLASSES_NUM;
            bbox_raw += REG * 4;
        }
    }
}

void RDKInference::performNMS(std::vector<std::vector<cv::Rect2d>>& bboxes, 
                            std::vector<std::vector<float>>& scores, 
                            std::vector<std::vector<int>>& indices) {
    for (int i = 0; i < CLASSES_NUM; i++) {
        cv::dnn::NMSBoxes(bboxes[i], scores[i], SCORE_THRESHOLD, NMS_THRESHOLD, 
                         indices[i], 1.f, NMS_TOP_K);
    }
}

void RDKInference::convertToDetectionResults(const std::vector<std::vector<cv::Rect2d>>& bboxes,
                                           const std::vector<std::vector<float>>& scores,
                                           const std::vector<std::vector<int>>& indices,
                                           float y_scale, float x_scale, int x_shift, int y_shift,
                                           std::vector<DetectionResult>& results) {
    results.clear();
    
    for (int cls_id = 0; cls_id < CLASSES_NUM; cls_id++) {
        for (const auto& idx : indices[cls_id]) {
            DetectionResult result;
            
            // 坐标转换回原图
            result.x1 = (bboxes[cls_id][idx].x - x_shift) / x_scale;
            result.y1 = (bboxes[cls_id][idx].y - y_shift) / y_scale;
            result.x2 = result.x1 + (bboxes[cls_id][idx].width) / x_scale;
            result.y2 = result.y1 + (bboxes[cls_id][idx].height) / y_scale;
            result.confidence = scores[cls_id][idx];
            result.class_id = cls_id;
            result.class_name = object_names[cls_id % CLASSES_NUM];
            
            results.push_back(result);
        }
    }
}

cv::Mat RDKInference::renderResults(const cv::Mat& img, const std::vector<DetectionResult>& results) {
    cv::Mat result_img = img.clone();
    
    for (const auto& result : results) {
        // 绘制矩形
        cv::rectangle(result_img, cv::Point(result.x1, result.y1), cv::Point(result.x2, result.y2),
                     cv::Scalar(255, 0, 0), LINE_SIZE);
        
        // 绘制文字
        std::string text = result.class_name + ": " + std::to_string(static_cast<int>(result.confidence * 100)) + "%";
        cv::putText(result_img, text, cv::Point(result.x1, result.y1 - 5), cv::FONT_HERSHEY_SIMPLEX,
                   FONT_SIZE, cv::Scalar(0, 0, 255), FONT_THICKNESS, cv::LINE_AA);
        
        spdlog::info("Detected: {} ({:.1f}, {:.1f}, {:.1f}, {:.1f}) confidence: {:.2f}",
                    result.class_name, result.x1, result.y1, result.x2, result.y2, result.confidence);
    }
    
    return result_img;
}

void RDKInference::releaseResources() {
    if (packed_dnn_handle_ != nullptr) {
        hbDNNRelease(packed_dnn_handle_);
        packed_dnn_handle_ = nullptr;
    }
}
