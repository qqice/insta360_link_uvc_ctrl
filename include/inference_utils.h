//
// Created by qqice on 24-6-2.
//

#ifndef INFERENCE_UTILS_H
#define INFERENCE_UTILS_H

#include <thread>
#include <memory>
#include "uvc_utils.h"
#include "spdlog/spdlog.h"
#include "upload_utils.h"

// #ifdef USE_RDK
// 前向声明
class RDKInference;
// RDK推理实例的全局声明
extern std::unique_ptr<RDKInference> rdk_inference;
// #else
// #include "image_process.h"
// #include "rknn_pool.h"
// #endif

extern std::string model_path;
extern std::string label_path;
extern int thread_num;

void inference_thread();

#endif //INFERENCE_UTILS_H
