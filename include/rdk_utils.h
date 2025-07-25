#ifndef RDK_UTILS_H
#define RDK_UTILS_H

// C/C++ Standard Libraries
#include <iostream>
#include <vector>
#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

// Third Party Libraries
#include <opencv2/opencv.hpp>
#include <opencv2/dnn/dnn.hpp>
#include "spdlog/spdlog.h"

// RDK BPU libDNN API
#include "dnn/hb_dnn.h"
#include "dnn/hb_dnn_ext.h"
#include "dnn/plugin/hb_dnn_layer.h"
#include "dnn/plugin/hb_dnn_plugin.h"
#include "dnn/hb_sys.h"

// 模型配置常量
#define CLASSES_NUM 80
#define NMS_THRESHOLD 0.7
#define SCORE_THRESHOLD 0.25
#define NMS_TOP_K 300
#define REG 16
#define FONT_SIZE 1.0
#define FONT_THICKNESS 1.0
#define LINE_SIZE 2.0

// 检测结果结构体
struct DetectionResult {
    float x1, y1, x2, y2;
    float confidence;
    int class_id;
    std::string class_name;
};

// RDK推理类
class RDKInference {
public:
    explicit RDKInference(const std::string& model_path);
    ~RDKInference();
    
    // 运行推理
    int runInference(const cv::Mat& img, std::vector<DetectionResult>& results);
    
    // 渲染结果
    cv::Mat renderResults(const cv::Mat& img, const std::vector<DetectionResult>& results);
    
    // 获取输入尺寸
    int getInputHeight() const { return input_H_; }
    int getInputWidth() const { return input_W_; }

private:
    std::string model_path_;
    hbPackedDNNHandle_t packed_dnn_handle_ = nullptr;
    hbDNNHandle_t dnn_handle_ = nullptr;
    int32_t input_H_ = 640;
    int32_t input_W_ = 640;
    
    // 私有方法
    int loadModel();
    int initializeModel();
    int validateModelInputs();
    int validateModelOutputs();
    
    cv::Mat preprocessImage(const cv::Mat& img, float& y_scale, float& x_scale, int& x_shift, int& y_shift);
    cv::Mat convertBGRToNV12(const cv::Mat& img);
    
    void postprocessFeatureMap(hbDNNTensor* output, int order_cls, int order_bbox, int stride,
                              int H_div, int W_div, std::vector<std::vector<cv::Rect2d>>& bboxes,
                              std::vector<std::vector<float>>& scores);
    
    void performNMS(std::vector<std::vector<cv::Rect2d>>& bboxes,
                   std::vector<std::vector<float>>& scores,
                   std::vector<std::vector<int>>& indices);
    
    void convertToDetectionResults(const std::vector<std::vector<cv::Rect2d>>& bboxes,
                                  const std::vector<std::vector<float>>& scores,
                                  const std::vector<std::vector<int>>& indices,
                                  float y_scale, float x_scale, int x_shift, int y_shift,
                                  std::vector<DetectionResult>& results);
    
    void releaseResources();
};

// 全局变量声明
extern std::vector<std::string> object_names;

#endif // RDK_UTILS_H
