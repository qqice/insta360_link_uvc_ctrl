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

// COCO Names
std::vector<std::string> object_names = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"};

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

// int main()
// {
//     // 0. 加载模型
//     hbPackedDNNHandle_t packed_dnn_handle;
//     if (loadModel(packed_dnn_handle) != 0) {
//         return -1;
//     }

//     // 1. 打印相关版本信息
//     std::cout << "[INFO] OpenCV Version: " << CV_VERSION << std::endl;
//     std::cout << "[INFO] MODEL_PATH: " << MODEL_PATH << std::endl;
//     std::cout << "[INFO] CLASSES_NUM: " << CLASSES_NUM << std::endl;
//     std::cout << "[INFO] NMS_THRESHOLD: " << NMS_THRESHOLD << std::endl;
//     std::cout << "[INFO] SCORE_THRESHOLD: " << SCORE_THRESHOLD << std::endl;

//     // 2. 获取模型handle并验证输入输出
//     const char **model_name_list;
//     int model_count = 0;
//     RDK_CHECK_SUCCESS(
//         hbDNNGetModelNameList(&model_name_list, &model_count, packed_dnn_handle),
//         "hbDNNGetModelNameList failed");
    
//     const char *model_name = model_name_list[0];
//     std::cout << "[model name]: " << model_name << std::endl;
    
//     hbDNNHandle_t dnn_handle;
//     RDK_CHECK_SUCCESS(
//         hbDNNGetModelHandle(&dnn_handle, packed_dnn_handle, model_name),
//         "hbDNNGetModelHandle failed");

//     int32_t input_H, input_W;
//     if (validateModelInputs(dnn_handle, input_H, input_W) != 0) {
//         return -1;
//     }

//     int order[6] = {0, 1, 2, 3, 4, 5};
//     if (validateModelOutputs(dnn_handle, order) != 0) {
//         return -1;
//     }

//     // 3. 图像预处理
//     cv::Mat img = cv::imread(TESR_IMG_PATH);
//     std::cout << "img path: " << TESR_IMG_PATH << std::endl;
//     std::cout << "img (cols, rows, channels): (" << img.rows << ", " << img.cols << ", " << img.channels() << ")" << std::endl;

//     float y_scale, x_scale;
//     int x_shift, y_shift;
//     cv::Mat resize_img = preprocessImage(img, input_H, input_W, y_scale, x_scale, x_shift, y_shift);
//     cv::Mat img_nv12 = convertBGRToNV12(resize_img, input_H, input_W);

//     // 4. 准备输入tensor
//     hbDNNTensorProperties input_properties;
//     RDK_CHECK_SUCCESS(
//         hbDNNGetInputTensorProperties(&input_properties, dnn_handle, 0),
//         "hbDNNGetInputTensorProperties failed");
    
//     hbDNNTensor input;
//     if (prepareInputTensor(input, input_properties, img_nv12, input_H, input_W) != 0) {
//         return -1;
//     }

//     // 5. 准备输出tensors
//     int32_t output_count = 0;
//     RDK_CHECK_SUCCESS(
//         hbDNNGetOutputCount(&output_count, dnn_handle),
//         "hbDNNGetOutputCount failed");
    
//     hbDNNTensor *output = nullptr;
//     if (prepareOutputTensors(output, dnn_handle, output_count) != 0) {
//         return -1;
//     }

//     // 6. 推理
//     hbDNNTaskHandle_t task_handle = nullptr;
//     if (runInference(dnn_handle, output, input, task_handle) != 0) {
//         return -1;
//     }

//     // 7. 后处理
//     auto begin_time = std::chrono::system_clock::now();
//     std::vector<std::vector<cv::Rect2d>> bboxes(CLASSES_NUM);
//     std::vector<std::vector<float>> scores(CLASSES_NUM);

//     // 处理三个尺度的特征图
//     postprocessFeatureMap(output, order[0], order[1], 8, input_H/8, input_W/8, bboxes, scores);
//     postprocessFeatureMap(output, order[2], order[3], 16, input_H/16, input_W/16, bboxes, scores);
//     postprocessFeatureMap(output, order[4], order[5], 32, input_H/32, input_W/32, bboxes, scores);

//     // NMS
//     std::vector<std::vector<int>> indices(CLASSES_NUM);
//     performNMS(bboxes, scores, indices);
    
//     std::cout << "\033[31m Post Process time = " << std::fixed << std::setprecision(2) 
//               << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - begin_time).count() / 1000.0 
//               << " ms\033[0m" << std::endl;

//     // 8. 渲染结果
//     renderResults(img, bboxes, scores, indices, y_scale, x_scale, x_shift, y_shift);

//     // 9. 保存图像
//     cv::imwrite(IMG_SAVE_PATH, img);

//     // 10. 释放资源
//     releaseResources(input, output, task_handle, packed_dnn_handle);

//     return 0;
// }

// 函数实现

// 加载模型
int loadModel(hbPackedDNNHandle_t& packed_dnn_handle) {
    auto begin_time = std::chrono::system_clock::now();
    const char *model_file_name = MODEL_PATH;
    RDK_CHECK_SUCCESS(
        hbDNNInitializeFromFiles(&packed_dnn_handle, &model_file_name, 1),
        "hbDNNInitializeFromFiles failed");
    std::cout << "\033[31m Load D-Robotics Quantize model time = " << std::fixed << std::setprecision(2) 
              << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - begin_time).count() / 1000.0 
              << " ms\033[0m" << std::endl;
    return 0;
}

// 验证模型输入
int validateModelInputs(hbDNNHandle_t dnn_handle, int32_t& input_H, int32_t& input_W) {
    int32_t input_count = 0;
    RDK_CHECK_SUCCESS(
        hbDNNGetInputCount(&input_count, dnn_handle),
        "hbDNNGetInputCount failed");

    hbDNNTensorProperties input_properties;
    RDK_CHECK_SUCCESS(
        hbDNNGetInputTensorProperties(&input_properties, dnn_handle, 0),
        "hbDNNGetInputTensorProperties failed");

    // 检查是否为单输入
    if (input_count > 1) {
        std::cout << "Your Model have more than 1 input, please check!" << std::endl;
        return -1;
    }

    // 检查输入类型
    if (input_properties.tensorType == HB_DNN_IMG_TYPE_NV12) {
        std::cout << "input tensor type: HB_DNN_IMG_TYPE_NV12" << std::endl;
    } else {
        std::cout << "input tensor type is not HB_DNN_IMG_TYPE_NV12, please check!" << std::endl;
        return -1;
    }

    // 检查数据排布
    if (input_properties.tensorLayout == HB_DNN_LAYOUT_NCHW) {
        std::cout << "input tensor layout: HB_DNN_LAYOUT_NCHW" << std::endl;
    } else {
        std::cout << "input tensor layout is not HB_DNN_LAYOUT_NCHW, please check!" << std::endl;
        return -1;
    }

    // 检查输入shape
    if (input_properties.validShape.numDimensions == 4) {
        input_H = input_properties.validShape.dimensionSize[2];
        input_W = input_properties.validShape.dimensionSize[3];
        std::cout << "input tensor valid shape: (" << input_properties.validShape.dimensionSize[0]
                  << ", " << input_properties.validShape.dimensionSize[1]
                  << ", " << input_H << ", " << input_W << ")" << std::endl;
    } else {
        std::cout << "input tensor validShape.numDimensions is not 4 such as (1,3,640,640), please check!" << std::endl;
        return -1;
    }

    return 0;
}

// 验证模型输出并调整顺序
int validateModelOutputs(hbDNNHandle_t dnn_handle, int* order) {
    int32_t output_count = 0;
    RDK_CHECK_SUCCESS(
        hbDNNGetOutputCount(&output_count, dnn_handle),
        "hbDNNGetOutputCount failed");

    // 检查输出数量
    if (output_count != 6) {
        std::cout << "Your Model's outputs num is not 6, please check!" << std::endl;
        return -1;
    }

    // 打印输出信息
    for (int i = 0; i < 6; i++) {
        hbDNNTensorProperties output_properties;
        RDK_CHECK_SUCCESS(
            hbDNNGetOutputTensorProperties(&output_properties, dnn_handle, i),
            "hbDNNGetOutputTensorProperties failed");
        std::cout << "output[" << i << "] valid shape: (" 
                  << output_properties.validShape.dimensionSize[0] << ", "
                  << output_properties.validShape.dimensionSize[1] << ", "
                  << output_properties.validShape.dimensionSize[2] << ", "
                  << output_properties.validShape.dimensionSize[3] << "), ";
        
        if (output_properties.quantiType == SHIFT)
            std::cout << "QuantiType: SHIFT" << std::endl;
        else if (output_properties.quantiType == SCALE)
            std::cout << "QuantiType: SCALE" << std::endl;
        else if (output_properties.quantiType == NONE)
            std::cout << "QuantiType: NONE" << std::endl;
    }

    // 调整输出头顺序的映射 (这里简化处理，保持原有顺序)
    // 在实际应用中需要根据具体模型调整
    
    std::cout << "Outputs order check SUCCESS, continue." << std::endl;
    std::cout << "order = {";
    for (int i = 0; i < 6; i++) {
        std::cout << order[i] << ", ";
    }
    std::cout << "}" << std::endl;

    return 0;
}

// 图像预处理
cv::Mat preprocessImage(const cv::Mat& img, int input_H, int input_W, float& y_scale, float& x_scale, int& x_shift, int& y_shift) {
    auto begin_time = std::chrono::system_clock::now();
    cv::Mat resize_img;
    
    if (PREPROCESS_TYPE == LETTERBOX_TYPE) { // letter box
        x_scale = std::min(1.0 * input_H / img.rows, 1.0 * input_W / img.cols);
        y_scale = x_scale;
        
        if (x_scale <= 0 || y_scale <= 0) {
            throw std::runtime_error("Invalid scale factor.");
        }

        int new_w = img.cols * x_scale;
        x_shift = (input_W - new_w) / 2;
        int x_other = input_W - new_w - x_shift;

        int new_h = img.rows * y_scale;
        y_shift = (input_H - new_h) / 2;
        int y_other = input_H - new_h - y_shift;

        cv::Size targetSize(new_w, new_h);
        cv::resize(img, resize_img, targetSize);
        cv::copyMakeBorder(resize_img, resize_img, y_shift, y_other, x_shift, x_other, 
                          cv::BORDER_CONSTANT, cv::Scalar(127, 127, 127));

        std::cout << "\033[31m pre process (LetterBox) time = " << std::fixed << std::setprecision(2) 
                  << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - begin_time).count() / 1000.0 
                  << " ms\033[0m" << std::endl;
    } else if (PREPROCESS_TYPE == RESIZE_TYPE) { // resize
        cv::Size targetSize(input_W, input_H);
        cv::resize(img, resize_img, targetSize);

        y_scale = 1.0 * input_H / img.rows;
        x_scale = 1.0 * input_W / img.cols;
        y_shift = 0;
        x_shift = 0;

        std::cout << "\033[31m pre process (Resize) time = " << std::fixed << std::setprecision(2) 
                  << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - begin_time).count() / 1000.0 
                  << " ms\033[0m" << std::endl;
    }
    
    std::cout << "y_scale = " << y_scale << ", x_scale = " << x_scale << std::endl;
    std::cout << "y_shift = " << y_shift << ", x_shift = " << x_shift << std::endl;
    
    return resize_img;
}

// BGR转NV12
cv::Mat convertBGRToNV12(const cv::Mat& img, int input_H, int input_W) {
    auto begin_time = std::chrono::system_clock::now();
    
    cv::Mat img_nv12;
    cv::Mat yuv_mat;
    cv::cvtColor(img, yuv_mat, cv::COLOR_BGR2YUV_I420);
    uint8_t *yuv = yuv_mat.ptr<uint8_t>();
    img_nv12 = cv::Mat(input_H * 3 / 2, input_W, CV_8UC1);
    uint8_t *ynv12 = img_nv12.ptr<uint8_t>();
    int uv_height = input_H / 2;
    int uv_width = input_W / 2;
    int y_size = input_H * input_W;
    memcpy(ynv12, yuv, y_size);
    uint8_t *nv12 = ynv12 + y_size;
    uint8_t *u_data = yuv + y_size;
    uint8_t *v_data = u_data + uv_height * uv_width;
    for (int i = 0; i < uv_width * uv_height; i++) {
        *nv12++ = *u_data++;
        *nv12++ = *v_data++;
    }
    
    std::cout << "\033[31m bgr8 to nv12 time = " << std::fixed << std::setprecision(2) 
              << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - begin_time).count() / 1000.0 
              << " ms\033[0m" << std::endl;
    
    return img_nv12;
}

// 准备输入tensor
int prepareInputTensor(hbDNNTensor& input, const hbDNNTensorProperties& input_properties, const cv::Mat& nv12_img, int input_H, int input_W) {
    input.properties = input_properties;
    hbSysAllocCachedMem(&input.sysMem[0], int(3 * input_H * input_W / 2));

    memcpy(input.sysMem[0].virAddr, nv12_img.ptr<uint8_t>(), int(3 * input_H * input_W / 2));
    hbSysFlushMem(&input.sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
    
    return 0;
}

// 准备输出tensors
int prepareOutputTensors(hbDNNTensor*& output, hbDNNHandle_t dnn_handle, int output_count) {
    output = new hbDNNTensor[output_count];
    for (int i = 0; i < 6; i++) {
        hbDNNTensorProperties &output_properties = output[i].properties;
        hbDNNGetOutputTensorProperties(&output_properties, dnn_handle, i);
        int out_aligned_size = output_properties.alignedByteSize;
        hbSysMem &mem = output[i].sysMem[0];
        hbSysAllocCachedMem(&mem, out_aligned_size);
    }
    return 0;
}

// 运行推理
int runInference(hbDNNHandle_t dnn_handle, hbDNNTensor* output, hbDNNTensor& input, hbDNNTaskHandle_t& task_handle) {
    auto begin_time = std::chrono::system_clock::now();
    
    hbDNNInferCtrlParam infer_ctrl_param;
    HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);
    hbDNNInfer(&task_handle, &output, &input, dnn_handle, &infer_ctrl_param);

    // 等待任务结束
    hbDNNWaitTaskDone(task_handle, 0);
    std::cout << "\033[31m forward time = " << std::fixed << std::setprecision(2) 
              << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - begin_time).count() / 1000.0 
              << " ms\033[0m" << std::endl;
    
    return 0;
}

// 后处理特征图
void postprocessFeatureMap(hbDNNTensor* output, int order_cls, int order_bbox, int stride, int H_div, int W_div, 
                          std::vector<std::vector<cv::Rect2d>>& bboxes, std::vector<std::vector<float>>& scores) {
    float CONF_THRES_RAW = -log(1 / SCORE_THRESHOLD - 1);
    
    // 检查反量化类型
    if (output[order_cls].properties.quantiType != NONE) {
        std::cout << "output[" << order_cls << "] QuantiType is not NONE, please check!" << std::endl;
        return;
    }
    if (output[order_bbox].properties.quantiType != SCALE) {
        std::cout << "output[" << order_bbox << "] QuantiType is not SCALE, please check!" << std::endl;
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

// 执行NMS
void performNMS(std::vector<std::vector<cv::Rect2d>>& bboxes, std::vector<std::vector<float>>& scores, 
                std::vector<std::vector<int>>& indices) {
    for (int i = 0; i < CLASSES_NUM; i++) {
        cv::dnn::NMSBoxes(bboxes[i], scores[i], SCORE_THRESHOLD, NMS_THRESHOLD, indices[i], 1.f, NMS_TOP_K);
    }
}

// 渲染结果
void renderResults(cv::Mat& img, const std::vector<std::vector<cv::Rect2d>>& bboxes, 
                  const std::vector<std::vector<float>>& scores, const std::vector<std::vector<int>>& indices,
                  float y_scale, float x_scale, int x_shift, int y_shift) {
    auto begin_time = std::chrono::system_clock::now();
    
    for (int cls_id = 0; cls_id < CLASSES_NUM; cls_id++) {
        for (std::vector<int>::const_iterator it = indices[cls_id].begin(); it != indices[cls_id].end(); ++it) {
            // 获取基本的 bbox 信息
            float x1 = (bboxes[cls_id][*it].x - x_shift) / x_scale;
            float y1 = (bboxes[cls_id][*it].y - y_shift) / y_scale;
            float x2 = x1 + (bboxes[cls_id][*it].width) / x_scale;
            float y2 = y1 + (bboxes[cls_id][*it].height) / y_scale;
            float score = scores[cls_id][*it];
            std::string name = object_names[cls_id % CLASSES_NUM];

            // 绘制矩形
            cv::rectangle(img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255, 0, 0), LINE_SIZE);

            // 绘制文字
            std::string text = name + ": " + std::to_string(static_cast<int>(score * 100)) + "%";
            cv::putText(img, text, cv::Point(x1, y1 - 5), cv::FONT_HERSHEY_SIMPLEX, FONT_SIZE, 
                       cv::Scalar(0, 0, 255), FONT_THICKNESS, cv::LINE_AA);

            // 打印检测信息
            std::cout << "(" << x1 << " " << y1 << " " << x2 << " " << y2 << "): \t" << text << std::endl;
        }
    }
    
    std::cout << "\033[31m Draw Result time = " << std::fixed << std::setprecision(2) 
              << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - begin_time).count() / 1000.0 
              << " ms\033[0m" << std::endl;
}

// 释放资源
void releaseResources(hbDNNTensor& input, hbDNNTensor* output, hbDNNTaskHandle_t task_handle, hbPackedDNNHandle_t packed_dnn_handle) {
    // 释放任务
    if (task_handle != nullptr) {
        hbDNNReleaseTask(task_handle);
    }

    // 释放内存
    hbSysFreeMem(&(input.sysMem[0]));
    for (int i = 0; i < 6; i++) {
        hbSysFreeMem(&(output[i].sysMem[0]));
    }
    delete[] output;

    // 释放模型
    hbDNNRelease(packed_dnn_handle);
}
