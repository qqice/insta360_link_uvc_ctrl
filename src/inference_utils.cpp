//
// Created by qqice on 24-6-2.
//
#include "inference_utils.h"

std::string model_path = "../model/yolov8n.rknn";
std::string label_path = "../model/coco80labels.txt";
int thread_num = 1;

void inference_thread() {
    auto rknn_pool = std::make_unique<RknnPool>(
            model_path, thread_num, label_path);
    spdlog::info("RKNN pool initialized");
    std::unique_ptr<cv::Mat> image = std::make_unique<cv::Mat>();
    while (true) {
        if (need_inference.load()) { // 收到"推理"请求
            spdlog::info("Inference_thread received inference request");
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
                    cv::imwrite("result.jpg", *image_res);
                    spdlog::info("Result saved");
                }
            }
            need_inference.store(false); // 重置"推理"请求状态
        }
        // 若没有"推理"请求,则等待一段时间
        spdlog::debug("Waiting for inference request");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}