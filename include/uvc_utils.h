#ifndef UVC_UTILS_H
#define UVC_UTILS_H
#include <cstdlib>
#include <ctime>
#include <sstream>
#include <iostream>
#include <atomic>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <libuvc/libuvc.h>
#include "spdlog/spdlog.h"
extern uvc_device_handle_t *devh;
void cb(uvc_frame_t *frame, void *ptr);
void video_capture_loop(const std::string& device_path);
void set_camera_gimbal_control(uvc_device_handle_t *deviceHandle, char horizontal_direction, char horizontal_speed,
                               char vertical_direction, char vertical_speed);
void stop_camera_gimbal_control(uvc_device_handle_t *deviceHandle);
void set_camera_gimbal_to_center(uvc_device_handle_t *deviceHandle);
void set_camera_zoom_absolute(uvc_device_handle_t *deviceHandle, int zoom);
void set_camera_gimbal_location(uvc_device_handle_t *deviceHandle, int horizontal_location, int vertical_location, int zoom);

extern std::atomic<bool> need_inference;
extern std::atomic<bool> frame_available;
extern std::mutex frame_mutex;
extern cv::Mat current_frame;
extern std::string current_name;

// 设置字体和颜色
extern int fontFace;
extern double fontScale;
extern int thickness;
extern cv::Scalar color; // 白色

// 计算文本宽度和高度，以便将其放置在右上角
extern int baseline;

extern int width;
extern int height;
extern int fps;
extern const int bitrate;
extern const char *rtsp_server;
extern const char *audio_device;
extern const char *audio_encoder;

extern cv::VideoWriter out;

#endif // UVC_UTILS_H