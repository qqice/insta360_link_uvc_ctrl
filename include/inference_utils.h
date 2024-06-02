//
// Created by qqice on 24-6-2.
//

#ifndef INFERENCE_UTILS_H
#define INFERENCE_UTILS_H

#include <thread>
#include "uvc_utils.h"
#include "image_process.h"
#include "spdlog/spdlog.h"
#include "rknn_pool.h"

extern std::string model_path;
extern std::string label_path;
extern int thread_num;

void inference_thread();

#endif //INFERENCE_UTILS_H
