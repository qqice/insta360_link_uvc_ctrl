#ifndef REALSENSE_UTILS_H
#define REALSENSE_UTILS_H

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include "spdlog/spdlog.h"

void realsense_loop();
extern int rs_frame_width;
extern int rs_frame_height;
extern int rs_frame_rate;

#endif //REALSENSE_UTILS_H
