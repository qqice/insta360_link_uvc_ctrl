#include "mqtt_utils.h"
#include "uvc_utils.h"
#include <nlohmann/json.hpp>

// MQTT设置
const char *MQTT_HOST = "127.0.0.1"; // MQTT代理服务器地址
const int MQTT_PORT = 1883; // MQTT端口
const char *MQTT_TOPIC = "camera/control"; // 订阅的主题

// MQTT消息回调函数
void on_message_callback(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message) {
    if (message->payloadlen) {
        spdlog::info("Received message: {}", (char *) message->payload);
        // 这里处理接收到的控制数据
    } else {
        spdlog::error("Received message with null payload");
    }
    fflush(stdout);
    try {
        nlohmann::json jsonParsed = nlohmann::json::parse((char *) message->payload);

        // 访问解析后的JSON数据
        spdlog::debug("control: {}", jsonParsed["control"].get<int>());
        switch (jsonParsed["control"].get<int>()) {
            case CAMERA_GIMBAL_CONTROL:
                set_camera_gimbal_control(devh, (char) jsonParsed["horizontal_direction"].get<int>(),
                                          (char) jsonParsed["horizontal_speed"].get<int>(),
                                          (char) jsonParsed["vertical_direction"].get<int>(),
                                          (char) jsonParsed["vertical_speed"].get<int>());
                break;
            case CAMERA_GIMBAL_STOP:
                stop_camera_gimbal_control(devh);
                break;
            case CAMERA_GIMBAL_CENTER:
                set_camera_gimbal_to_center(devh);
                break;
            case CAMERA_ZOOM:
                set_camera_zoom_absolute(devh, jsonParsed["zoom"].get<int>());
                break;
            case CAMERA_GIMBAL_LOCATION:
                set_camera_gimbal_location(devh, jsonParsed["horizontal_location"].get<int>(),
                                           jsonParsed["vertical_location"].get<int>(), jsonParsed["zoom"].get<int>());
                break;
            case LEAF_DISEASE_INFERENCE:
              {
                spdlog::info("Received leaf disease inference request");
                need_inference.store(true);
                current_name = jsonParsed["name"];
                break;
              }
            default:
                spdlog::error("Unknown control command: {}", jsonParsed["control"].get<int>());
                break;
        }
    } catch (nlohmann::json::parse_error &e) {
        // 如果解析过程中发生错误，输出错误信息
        spdlog::error("JSON parse error: {}", e.what());
    }
}

// MQTT循环处理函数，将在独立线程中运行
void mqtt_loop(struct mosquitto *mosq) {
    mosquitto_loop_forever(mosq, -1, 1);
}