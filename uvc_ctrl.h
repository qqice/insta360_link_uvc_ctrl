#include <libuvc/libuvc.h>
#include <ctime>
#include <sstream>
#include <cstdio>
#include <unistd.h>
#include <iostream>
#include <cstring>


uvc_device_handle_t *devh;

void set_camera_gimbal_control(uvc_device_handle_t *devh,const char horizontal_direction,const char horizontal_speed,const char vertical_direction,const char vertical_speed);

void stop_camera_gimbal_control(uvc_device_handle_t *devh);

void set_camera_gimbal_to_center(uvc_device_handle_t *devh);

void set_camera_zoom_absolute(uvc_device_handle_t *devh, int zoom);

void set_camera_gimbal_location(uvc_device_handle_t *devh,int horizontal_location,int vertical_location,int zoom);

void cb(uvc_frame_t *frame, void *ptr);