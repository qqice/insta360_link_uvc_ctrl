
#include <iostream>
#include <cstring>
#include <thread>
#include <csignal>
#include "mqtt_utils.h"
#include "uvc_utils.h"
#include "inference.h"


// 标志变量，用来控制循环
volatile sig_atomic_t loopFlag = 1;

// 信号处理函数
void signalHandler(int signal) {
    if (signal == SIGINT) {
        // 当接收到SIGINT信号时，修改循环控制变量
        loopFlag = 0;
    }
}


//void inference_thread() {
//    while (true) {
//        if (need_inference.load()) { // 收到"推理"请求
//            std::cout << "Inference_thread received inference request" << std::endl;
//            if (frame_available.load()) { // 检查帧可用标志
//              cv::Mat inf_frame;
//              {
//                  std::lock_guard<std::mutex> lock(frame_mutex);
//                  inf_frame = current_frame.clone(); // 获取当前帧副本
//              }
//              frame_available.store(false); // 重置帧可用标志
//              if (!inf_frame.empty()) {
//                std::cout << "Start inference" << std::endl;
//                //输出推理图像大小
//                std::cout << "Inference image size: " << inf_frame.size() << std::endl;
//                // cv::Mat resized_frame;
//                // cv::resize(inf_frame, resized_frame, cv::Size(640, 640), cv::INTER_LINEAR);
//                std::vector<Detection> output = leaf_disease_inf.runInference(inf_frame);
//
//                int detections = output.size();
//                std::cout << "Number of detections:" << detections << std::endl;
//
//                for (int i = 0; i < detections; ++i)
//                {
//                  Detection detection = output[i];
//
//                  cv::Rect box = detection.box;
//                  cv::Scalar color = detection.color;
//
//                  // Detection box
//                  cv::rectangle(inf_frame, box, color, 2);
//
//                  // Detection box text
//                  std::string classString = detection.className + ' ' + std::to_string(detection.confidence).substr(0, 4);
//                  std::cout << classString << std::endl;
//                  std::cout << "Detection box: " << box.x << " " <<box.y << std::endl;
//                  cv::Size textSize = cv::getTextSize(classString, cv::FONT_HERSHEY_DUPLEX, 1, 2, 0);
//                  cv::Rect textBox(box.x, box.y - 40, textSize.width + 10, textSize.height + 20);
//
//                  cv::rectangle(inf_frame, textBox, color, cv::FILLED);
//                  cv::putText(inf_frame, classString, cv::Point(box.x + 5, box.y - 10), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 0), 2, 0);
//                }
//                std::cout << "Inference finished" << std::endl;
//              }
//            }
//            need_inference.store(false); // 重置"推理"请求状态
//        }
//        // 若没有"推理"请求,则等待一段时间
//        // std::cout << "Waiting for inference request" << std::endl;
//        std::this_thread::sleep_for(std::chrono::milliseconds(10));
//    }
//}




int main(int argc, char **argv) {
    uvc_context_t *ctx;
    uvc_device_t *dev;
    uvc_stream_ctrl_t ctrl;
    uvc_error_t res;
    //setenv("GST_DEBUG", "3", 1);

    //std::cout << cv::getBuildInformation() << std::endl;

    // 注册信号处理函数
    signal(SIGINT, signalHandler);

    // 初始化mosquitto库
    mosquitto_lib_init();

    // 创建新的mosquitto客户端实例
    struct mosquitto *mosq = mosquitto_new(nullptr, true, nullptr);
    if (!mosq) {
        std::cerr << "Failed to create mosquitto instance." << std::endl;
        return -1;
    }

    // 设置消息回调
    mosquitto_message_callback_set(mosq, on_message_callback);

    // 连接到MQTT代理服务器
    if (mosquitto_connect(mosq, MQTT_HOST, MQTT_PORT, 60)) {
        std::cerr << "Could not connect to MQTT Broker." << std::endl;
        return -1;
    }

    // 订阅主题
    mosquitto_subscribe(mosq, nullptr, MQTT_TOPIC, 0);

    // 创建并启动处理MQTT消息的线程
    std::thread mqtt_thread(mqtt_loop, mosq);

    // 创建并启动推理线程
    //std::thread inf_thread(inference_thread);

    /* Initialize a UVC service context. Libuvc will set up its own libusb
     * context. Replace NULL with a libusb_context pointer to run libuvc
     * from an existing libusb context. */
    res = uvc_init(&ctx, nullptr);

    if (res < 0) {
        uvc_perror(res, "uvc_init");
        return res;
    }

    puts("UVC initialized");

    /* Locates the first attached UVC device, stores in dev */
    res = uvc_find_device(
            ctx, &dev,
            0x2e1a, 0x4c01, nullptr); /* filter devices: vendor_id, product_id, "serial_num" */

    if (res < 0) {
        uvc_perror(res, "uvc_find_device"); /* no devices found */
    } else {
        puts("Device found");

        /* Try to open the device: requires exclusive access */
        res = uvc_open(dev, &devh);

        if (res < 0) {
            uvc_perror(res, "uvc_open"); /* unable to open device */
        } else {
            puts("Device opened");

            /* Print out a message containing all the information that libuvc
             * knows about the device */
            uvc_print_diag(devh, stderr);

            const uvc_format_desc_t *format_desc = uvc_get_format_descs(devh);
            const uvc_frame_desc_t *frame_desc = format_desc->frame_descs;
            enum uvc_frame_format frame_format;

            switch (format_desc->bDescriptorSubtype) {
                case UVC_VS_FORMAT_MJPEG:
                    frame_format = UVC_COLOR_FORMAT_MJPEG;
                    break;
                case UVC_VS_FORMAT_FRAME_BASED:
                    frame_format = UVC_FRAME_FORMAT_H264;
                    break;
                default:
                    frame_format = UVC_FRAME_FORMAT_YUYV;
                    break;
            }

            if (frame_desc) {
                width = frame_desc->wWidth;
                height = frame_desc->wHeight;
                fps = 10000000 / frame_desc->dwDefaultFrameInterval;
            }

            printf("\nFirst format: (%4s) %dx%d %dfps\n", format_desc->fourccFormat, width, height, fps);

            /* Try to negotiate first stream profile */
            res = uvc_get_stream_ctrl_format_size(
                    devh, &ctrl, /* result stored in ctrl */
                    frame_format,
                    width, height, fps /* width, height, fps */
            );

            /* Print out the result */
            uvc_print_stream_ctrl(&ctrl, stderr);

            if (res < 0) {
                uvc_perror(res, "get_mode"); /* device doesn't provide a matching stream */
            } else {
                /* Start the video stream. The library will call user function cb:
                 *   cb(frame, (void *) 12345)
                 */
                res = uvc_start_streaming(devh, &ctrl, cb, (void *) 12345, 0);

                if (res < 0) {
                    uvc_perror(res, "start_streaming"); /* unable to start stream */
                } else {
                    puts("Streaming...");

                    // sleep(600); /* stream for 10 minutes */

                    while (loopFlag) {
                        sleep(1);
                    }
                    // 等待MQTT线程结束（在这个示例中，线程将无限循环，因此下面的join调用实际上会阻塞）
                    // mqtt_thread.join();

                    /* End the stream. Blocks until last callback is serviced */
                    uvc_stop_streaming(devh);
                    mosquitto_destroy(mosq);
                    puts("Done streaming.");
                }
            }

            /* Release our handle on the device */
            uvc_close(devh);
            puts("Device closed");
        }

        /* Release the device descriptor */
        uvc_unref_device(dev);
    }

    /* Close the UVC context. This closes and cleans up any existing device handles,
     * and it closes the libusb context if one was not provided. */
    uvc_exit(ctx);
    puts("UVC exited");
    mosquitto_lib_cleanup();
    return 0;
}
