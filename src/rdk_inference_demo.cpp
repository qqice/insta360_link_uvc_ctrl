#include <iostream>
#include <opencv2/opencv.hpp>
#include "rdk_utils.h"
#include "spdlog/spdlog.h"

// 使用结构体来处理命令行参数
struct ProgramOptions {
    std::string model_path;
    std::string input_filename;
    std::string output_filename;
};

// 解析命令行参数
bool parseCommandLine(int argc, char *argv[], ProgramOptions &options) {
    if (argc < 4) {
        std::cout << "Usage: " << argv[0] << " <model_path> <input_image> <output_image>\n";
        std::cout << "Example: " << argv[0] << " model.bin input.jpg output.jpg\n";
        return false;
    }
    
    options.model_path = argv[1];
    options.input_filename = argv[2];
    options.output_filename = argv[3];
    
    return true;
}

int main(int argc, char *argv[]) {
    spdlog::info("YOLO11 RDK Inference Demo");
    
    ProgramOptions options;
    if (!parseCommandLine(argc, argv, options)) {
        return -1;
    }
    
    // 读取输入图像
    cv::Mat img = cv::imread(options.input_filename);
    if (img.empty()) {
        spdlog::error("Failed to read image: {}", options.input_filename);
        return -1;
    }
    
    spdlog::info("Input image: {} ({}x{})", options.input_filename, img.cols, img.rows);
    spdlog::info("Model path: {}", options.model_path);
    
    try {
        // 初始化RDK推理引擎
        auto begin_time = std::chrono::system_clock::now();
        RDKInference rdk_inference(options.model_path);
        auto init_duration = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now() - begin_time).count() / 1000.0;
        spdlog::info("RDK Inference initialization time: {:.2f} ms", init_duration);
        
        // 运行推理
        std::vector<DetectionResult> results;
        begin_time = std::chrono::system_clock::now();
        int ret = rdk_inference.runInference(img, results);
        auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now() - begin_time).count() / 1000.0;
        
        if (ret != 0) {
            spdlog::error("Inference failed with code: {}", ret);
            return -1;
        }
        
        spdlog::info("Total inference time: {:.2f} ms", total_duration);
        spdlog::info("Detected {} objects", results.size());
        
        // 渲染结果
        cv::Mat result_img = rdk_inference.renderResults(img, results);
        
        // 保存结果图像
        bool save_success = cv::imwrite(options.output_filename, result_img);
        if (save_success) {
            spdlog::info("Result saved to: {}", options.output_filename);
        } else {
            spdlog::error("Failed to save result to: {}", options.output_filename);
            return -1;
        }
        
        // 打印检测结果摘要
        if (!results.empty()) {
            spdlog::info("Detection results:");
            for (const auto& result : results) {
                spdlog::info("  {}: ({:.1f}, {:.1f}, {:.1f}, {:.1f}) confidence: {:.2f}",
                           result.class_name, result.x1, result.y1, result.x2, result.y2, result.confidence);
            }
        }
        
    } catch (const std::exception& e) {
        spdlog::error("Exception: {}", e.what());
        return -1;
    }
    
    spdlog::info("RDK inference demo completed successfully");
    return 0;
}
