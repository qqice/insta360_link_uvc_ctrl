#include <nlohmann/json.hpp>
#include <mosquitto.h>

// MQTT设置
const char* MQTT_HOST = "127.0.0.1"; // MQTT代理服务器地址
const int MQTT_PORT = 1883; // MQTT端口
const char* MQTT_TOPIC = "camera/control"; // 订阅的主题

void on_message_callback(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message);

void mqtt_loop(struct mosquitto *mosq);