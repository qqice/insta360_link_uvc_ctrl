# insta360_link_uvc_ctrl_test
使用libuvc opencv mqtt gstreamer rkmpp以及rknn，实现基于Rockchip RK3588平台，使用insta360 link的实时视频传输、云台控制以及YOLOv8目标检测。

This project leverages the Rockchip RK3588 platform to implement real-time video transmission, gimbal control, and YOLOv8 object detection using the Insta360 Link camera. The implementation utilizes a combination of libraries and technologies including libuvc for USB video capture, OpenCV for image processing, MQTT for message queuing, GStreamer for media streaming, RKMpp for hardware-accelerated video decoding, and RKNN for running neural network models. Together, these components create a powerful and efficient system capable of delivering high-performance video analytics and control.

本项目的WebUI见另一个仓库：https://github.com/qqice/insta360_link_WebUI

The WebUI for this project is available in another repository: https://github.com/qqice/insta360_link_WebUI

使用到的部分库如下：

Some of the libraries used are listed below:

libuvc:https://github.com/libuvc/libuvc

opencv_for_arm64:https://github.com/Qengineering/Install-OpenCV-Raspberry-Pi-64-bits

opencv_for_jetson:https://github.com/Qengineering/Install-OpenCV-Jetson-Nano

sudo apt install gstreamer1.0-rtsp libmosquitto-dev nlohmann-json3-dev

