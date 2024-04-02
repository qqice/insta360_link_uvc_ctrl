#include "mqtt.h"

// MQTT消息回调函数
void on_message_callback(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message) {
    if(message->payloadlen){
        std::cout << "Received message: " << (char*)message->payload << std::endl;
        // 这里处理接收到的控制数据
    }else{
        std::cout << message->topic << " (null)\n";
    }
    fflush(stdout);
    try {
            nlohmann::json jsonParsed = nlohmann::json::parse((char*)message->payload);
    
            // 访问解析后的JSON数据
            std::cout << "control: " << jsonParsed["control"].get<int>() << std::endl;
            // std::cout << "zoom: " << jsonParsed["zoom"].get<int>() << std::endl;
            // std::cout << "horizontal_direction: " << jsonParsed["horizontal_direction"].get<int>() << std::endl;
            // std::cout << "horizontal_speed: " << jsonParsed["horizontal_speed"].get<int>() << std::endl;
            // std::cout << "vertical_direction: " << jsonParsed["vertical_direction"].get<int>() << std::endl;
            // std::cout << "vertical_speed: " << jsonParsed["vertical_speed"].get<int>() << std::endl;
            // std::cout << "horizontal_location: " << jsonParsed["horizontal_location"].get<int>() << std::endl;
            // std::cout << "vertical_location: " << jsonParsed["vertical_location"].get<int>() << std::endl;
            switch (jsonParsed["control"].get<int>())
            {
            case 0:
              set_camera_gimbal_control(devh,(char)jsonParsed["horizontal_direction"].get<int>(),(char)jsonParsed["horizontal_speed"].get<int>(),(char)jsonParsed["vertical_direction"].get<int>(),(char)jsonParsed["vertical_speed"].get<int>());
              break;
            case 1:
              stop_camera_gimbal_control(devh);
              break;
            case 2:
              set_camera_gimbal_to_center(devh);
              break;
            case 3:
              set_camera_zoom_absolute(devh,jsonParsed["zoom"].get<int>());
              break;
            case 4:
              set_camera_gimbal_location(devh,jsonParsed["horizontal_location"].get<int>(),jsonParsed["vertical_location"].get<int>(),jsonParsed["zoom"].get<int>());
              break;
            default:
              std::cerr << "Unknown control command: " << jsonParsed["control"] << std::endl;
              break;
            }
        } catch (nlohmann::json::parse_error& e) {
            // 如果解析过程中发生错误，输出错误信息
            std::cerr << "JSON parse error: " << e.what() << std::endl;
        }
}

// MQTT循环处理函数，将在独立线程中运行
void mqtt_loop(struct mosquitto *mosq) {
    mosquitto_loop_forever(mosq, -1, 1);
}