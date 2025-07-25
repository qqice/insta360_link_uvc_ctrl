#include <iostream>
#include <cstring>
#include <thread>
#include <csignal>
#include <chrono>
#include "mqtt_utils.h"
#include "uvc_utils.h"
#include "spdlog/spdlog.h"
#include "inference_utils.h"

#ifdef USE_REALSENSE
#include "realsense_utils.h"
#endif

// 标志变量，用来控制循环
volatile sig_atomic_t loopFlag = 1;

// 信号处理函数
void signalHandler(int signal) {
    if (signal == SIGINT) {
        // 当接收到SIGINT信号时，修改循环控制变量
        loopFlag = 0;
    }
}

int main(int argc, char **argv) {
    // 默认视频设备路径
    std::string video_device = "/dev/video0";
    
    // 如果提供了命令行参数，使用指定的设备路径
    if (argc > 1) {
        video_device = argv[1];
    }
    
    spdlog::info("使用视频设备: {}", video_device);
    
    // 注册信号处理函数
    signal(SIGINT, signalHandler);

    // 初始化mosquitto库
    mosquitto_lib_init();

    // 创建新的mosquitto客户端实例
    struct mosquitto *mosq = mosquitto_new(nullptr, true, nullptr);
    if (!mosq) {
        spdlog::error("Failed to create mosquitto instance.");
        return -1;
    }

    // 设置消息回调
    mosquitto_message_callback_set(mosq, on_message_callback);

    // 连接到MQTT代理服务器
    if (mosquitto_connect(mosq, MQTT_HOST, MQTT_PORT, 60)) {
        spdlog::error("Could not connect to MQTT Broker.");
        return -1;
    }

    // 订阅主题
    mosquitto_subscribe(mosq, nullptr, MQTT_TOPIC, 0);

    // 创建并启动处理MQTT消息的线程
    std::thread mqtt_thread(mqtt_loop, mosq);

#ifdef USE_REALSENSE
    // 创建并启动Realsense线程
    std::thread realsense_thread(realsense_loop);
#endif

    // 创建并启动推理线程
    std::thread inf_thread(inference_thread);

    spdlog::info("开始从 {} 捕获视频流...", video_device);
    
    // 在单独线程中启动视频捕获
    std::thread video_thread([&video_device]() {
        video_capture_loop(video_device);
    });

    // 主循环
    while (loopFlag) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    spdlog::info("正在停止程序...");
    
    // 等待视频线程结束
    if (video_thread.joinable()) {
        video_thread.join();
    }
    
    // 等待推理线程结束
    if (inf_thread.joinable()) {
        inf_thread.join();
    }
    
    // 等待MQTT线程结束
    if (mqtt_thread.joinable()) {
        mqtt_thread.join();
    }

#ifdef USE_REALSENSE
    // 等待Realsense线程结束
    if (realsense_thread.joinable()) {
        realsense_thread.join();
    }
#endif

    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    
    spdlog::info("程序已退出");
    return 0;
}
