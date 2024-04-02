#include <opencv2/opencv.hpp>
#include <cstdio>
#include <unistd.h>
#include <iostream>
#include <cstring>

// 设置字体和颜色
int fontFace = cv::FONT_HERSHEY_SIMPLEX;
double fontScale = 1;
int thickness = 2;
cv::Scalar color(255, 255, 255); // 白色

// 计算文本宽度和高度，以便将其放置在右上角
int baseline = 0;

int width = 1920;
int height = 1080;
int fps = 30;

const int bitrate = 3000000;
const char* rtsp_server = "rtsp://127.0.0.1:8554/mystream";

cv::VideoWriter out("appsrc ! videoconvert ! video/x-raw,format=I420 ! nvvidconv ! nvv4l2h264enc preset-level=1 bitrate="+ std::to_string(bitrate) +" maxperf-enable=1 iframeinterval=" + std::to_string(fps * 2) +
              " ! video/x-h264,profile=baseline ! rtspclientsink location=" + rtsp_server,
              cv::CAP_GSTREAMER, 0, fps, cv::Size(width, height), true);