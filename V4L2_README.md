# V4L2视频捕获使用说明

本项目现在支持直接从Linux V4L2设备（如 `/dev/video0`）读取视频流，并输出到RTSP服务器。

## 编译

```bash
cd /root/insta360_link_uvc_ctrl/build
cmake ..
make V4L2Capture
```

## 使用方法

### 基本用法（使用默认设备 /dev/video0）
```bash
./V4L2Capture
```

### 指定视频设备
```bash
./V4L2Capture /dev/video1
```

## 功能特性

1. **视频捕获**: 直接从V4L2设备读取MJPEG格式的视频流
2. **实时时间戳**: 在视频帧上叠加当前时间戳
3. **RTSP输出**: 将视频流编码并推送到RTSP服务器 (rtsp://127.0.0.1:8554/mainstream)
4. **音频支持**: 同时捕获音频并添加到RTSP流中
5. **推理支持**: 保持原有的AI推理功能
6. **MQTT控制**: 保持原有的MQTT远程控制功能

## 配置参数

在 `uvc_utils.cpp` 中可以修改以下参数：

- `width`: 视频宽度 (默认: 1920)
- `height`: 视频高度 (默认: 1080) 
- `fps`: 帧率 (默认: 30)
- `rtsp_server`: RTSP服务器地址 (默认: "rtsp://127.0.0.1:8554/mainstream")
- `audio_device`: 音频设备 (默认: "hw:5,0")

## 依赖检查

确保系统中安装了以下组件：
- OpenCV (with GStreamer support)
- V4L2 设备驱动
- RTSP服务器 (如 GStreamer rtsp-server)

## 与原版差异

- **UVCCapture**: 使用UVC设备 (如Insta360 Link摄像头)
- **V4L2Capture**: 使用标准V4L2设备 (如 /dev/video0)

两个版本都支持相同的MQTT控制和AI推理功能。
