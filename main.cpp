#include <libuvc/libuvc.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <unistd.h>

void cb(uvc_frame_t *frame, void *ptr) {
    uvc_frame_t *bgr;
    uvc_error_t ret;

    // 尝试将UVC帧转换为BGR格式，这是OpenCV常用的格式
    bgr = uvc_allocate_frame(frame->width * frame->height * 3);
    if (!bgr) {
        std::cerr << "unable to allocate bgr frame!" << std::endl;
        return;
    }
    ret = uvc_any2bgr(frame, bgr);
    if (ret) {
        uvc_perror(ret, "uvc_any2bgr");
        uvc_free_frame(bgr);
        return;
    }

    cv::Mat mat(cv::Size(frame->width, frame->height), CV_8UC3, bgr->data, cv::Mat::AUTO_STEP);

    // 显示图像
    cv::imshow("UVC Test", mat);

    // 等待1ms，以便OpenCV可以处理事件
    cv::waitKey(1);

    uvc_free_frame(bgr);
}


int main() {
    uvc_context_t *ctx;
    uvc_device_t *dev;
    uvc_device_handle_t *devh;
    uvc_stream_ctrl_t ctrl;

    uvc_error_t res = uvc_init(&ctx, NULL);
    if (res < 0) {
        uvc_perror(res, "uvc_init");
        return res;
    }

    // 寻找并打开设备
    res = uvc_find_device(ctx, &dev, 0, 0, NULL); // 你可能需要指定vendor ID和product ID
    if (res < 0) {
        uvc_perror(res, "uvc_find_device");
    } else {
        res = uvc_open(dev, &devh);
        if (res < 0) {
            uvc_perror(res, "uvc_open");
        } else {
            // 打印设备描述
            uvc_print_diag(devh, stderr);

            // 获取流控制描述
            res = uvc_get_stream_ctrl_format_size(
                devh, &ctrl, UVC_FRAME_FORMAT_ANY, 640, 480, 30 /* width, height, fps */
            );

            cv::namedWindow("UVC Test", cv::WINDOW_AUTOSIZE);
            // 启动流
            res = uvc_start_streaming(devh, &ctrl, cb, (void *) 12345, 0);
            if (res < 0) {
                uvc_perror(res, "uvc_start_streaming");
            } else {
                puts("Streaming...");
                uvc_set_ae_mode(devh, 1); /* 自动曝光 */

                // 如果你需要控制云台，这里可能需要发送自定义命令
                // ...

                sleep(10); /* stream for 10 seconds */

                // 停止流
                uvc_stop_streaming(devh);
                puts("Done streaming.");
            }

            // 关闭设备句柄
            uvc_close(devh);
        }

        // 释放设备列表
        uvc_unref_device(dev);
    }

    // 关闭UVC上下文
    uvc_exit(ctx);
    puts("UVC exited");

    return 0;
}
