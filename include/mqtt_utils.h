#ifndef MQTT_UTILS_H
#define MQTT_UTILS_H

#include <mosquitto.h>
#include <cstdio>
#include <iostream>
#include "spdlog/spdlog.h"

void on_message_callback(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message);
void mqtt_loop(struct mosquitto *mosq);
enum mqtt_ctrl_command {
    CAMERA_GIMBAL_CONTROL = 0,
    CAMERA_GIMBAL_STOP = 1,
    CAMERA_GIMBAL_CENTER = 2,
    CAMERA_ZOOM = 3,
    CAMERA_GIMBAL_LOCATION = 4,
    LEAF_DISEASE_INFERENCE = 5
};
// MQTT设置
extern const char *MQTT_HOST; // MQTT代理服务器地址
extern const int MQTT_PORT; // MQTT端口
extern const char *MQTT_TOPIC; // 订阅的主题
#endif // MQTT_UTILS_H