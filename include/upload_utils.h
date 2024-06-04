//
// Created by qqice on 24-6-3.
//

#ifndef UPLOAD_UTILS_H
#define UPLOAD_UTILS_H

#include <iostream>
#include <string>
#include <curl/curl.h>
#include <opencv2/opencv.hpp>
#include "spdlog/spdlog.h"
void upload_to_CF(const std::string &file_path, const cv::Mat &img);
#endif //UPLOAD_UTILS_H
