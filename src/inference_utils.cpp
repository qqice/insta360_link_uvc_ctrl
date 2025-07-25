//
// Created by qqice on 24-6-2.
//
#include "inference_utils.h"

#ifdef USE_RDK
#include "rdk_utils.h"
#else
#include "image_process.h"
#include "rknn_pool.h"
#endif

// RDK模型路径，使用.bin格式而不是.rknn
#ifdef USE_RDK
std::string model_path = "../model/yolo11x_detect_bayese_640x640_nv12_modified.bin";
#else
std::string model_path = "../model/yolov8n.rknn";
#endif
std::string label_path = "../model/coco80labels.txt";
int thread_num = 1;

#ifdef USE_RDK
// RDK推理实例
std::unique_ptr<RDKInference> rdk_inference;
#endif

void inference_thread() {
#ifdef USE_RDK
    try {
        // 初始化RDK推理引擎
        rdk_inference = std::make_unique<RDKInference>(model_path);
        spdlog::info("RDK Inference initialized successfully");
    } catch (const std::exception& e) {
        spdlog::error("Failed to initialize RDK Inference: {}", e.what());
        return;
    }
    
    spdlog::info("RDK inference thread started");
    
    while (true) {
        if (need_inference.load()) { // 收到"推理"请求
            spdlog::info("Inference_thread received inference request");
            std::unique_ptr<cv::Mat> image = std::make_unique<cv::Mat>();
            
            if (frame_available.load()) { // 检查帧可用标志
                {
                    std::lock_guard<std::mutex> lock(frame_mutex);
                    *image = current_frame.clone(); // 获取当前帧副本
                }
                frame_available.store(false); // 重置帧可用标志
                
                if (!image->empty()) {
                    spdlog::info("Start RDK inference");
                    
                    try {
                        // 使用RDK推理引擎进行推理
                        std::vector<DetectionResult> detection_results;
                        int ret = rdk_inference->runInference(*image, detection_results);
                        
                        if (ret == 0) {
                            spdlog::info("RDK Inference finished, detected {} objects", detection_results.size());
                            
                            // 渲染结果
                            cv::Mat result_img = rdk_inference->renderResults(*image, detection_results);
                            
                            //保存到本地
                            std::string result_path = "result_" + current_name;
                            cv::imwrite(result_path, result_img);
                            spdlog::info("Result saved to {}", result_path);
                        } else {
                            spdlog::error("RDK inference failed with code: {}", ret);
                        }
                    } catch (const std::exception& e) {
                        spdlog::error("Exception during RDK inference: {}", e.what());
                    }
                } else {
                    spdlog::warn("Current frame is empty, skipping inference");
                }
            } else {
                spdlog::warn("No frame available for inference");
            }
            
            need_inference.store(false); // 重置"推理"请求状态
        }
        
        // 若没有"推理"请求,则等待一段时间
        spdlog::debug("Waiting for inference request");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
#else
    // 原有的RKNN实现作为备选方案
    auto rknn_pool = std::make_unique<RknnPool>(
            model_path, thread_num, label_path);
    spdlog::info("RKNN pool initialized");
    while (true) {
        if (need_inference.load()) { // 收到"推理"请求
            spdlog::info("Inference_thread received inference request");
            std::unique_ptr<cv::Mat> image = std::make_unique<cv::Mat>();
            if (frame_available.load()) { // 检查帧可用标志
                {
                    std::lock_guard<std::mutex> lock(frame_mutex);
                    *image = current_frame.clone(); // 获取当前帧副本
                }
                frame_available.store(false); // 重置帧可用标志
                if (!image->empty()) {
                    spdlog::info("Start inference");
                    spdlog::debug("Preprocessing image");
                    ImageProcess image_process(image->cols, image->rows, 640);
                    std::shared_ptr<cv::Mat> image_res;
                    // 进行推理
                    rknn_pool->AddInferenceTask(std::move(image), image_process);
                    while (image_res == nullptr) {
                        image_res = rknn_pool->GetImageResultFromQueue();
                    }
                    spdlog::info("Inference finished");
                    upload_to_CF(current_name, *image_res);
                    spdlog::info("Result uploaded");
                }
            }
            need_inference.store(false); // 重置"推理"请求状态
        }
        // 若没有"推理"请求,则等待一段时间
        spdlog::debug("Waiting for inference request");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
#endif
}