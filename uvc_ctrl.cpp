#include "uvc_ctrl.h"

void set_camera_gimbal_control(uvc_device_handle_t *devh,const char horizontal_direction,const char horizontal_speed,const char vertical_direction,const char vertical_speed) {
  int res;
  uint16_t Ctrl = 0x16;     //Control Selector (0x16)
  uint16_t Unit = 0x09;     //Entity (0x09)
  uint16_t Length = 4;      //数据帧长度
  unsigned char *data;
  data = (unsigned char *)malloc(Length);
  data[0] = horizontal_direction;
  data[1] = horizontal_speed;
  data[2] = vertical_direction;
  data[3] = vertical_speed;
  printf("horizontal_direction = %d, horizontal_speed = %d, vertical_direction = %d, vertical_speed = %d\n",horizontal_direction,horizontal_speed,vertical_direction,vertical_speed);
  // Send the control request
  res = uvc_set_ctrl(
      devh,
      Unit,
      Ctrl,
      data,
      Length
  );

  if (res != Length) {
    printf("Failed to set camera control\n");
    printf("res = %d\n", res);
  } else {
    printf("Control request sent successfully\n");
  }
}

void stop_camera_gimbal_control(uvc_device_handle_t *devh) {
  set_camera_gimbal_control(devh,0x00,0x01,0x00,0x01);
}

void set_camera_gimbal_to_center(uvc_device_handle_t *devh) {
  int res;
  uint16_t Ctrl = 0x1a;     //Control Selector (0x16)
  uint16_t Unit = 0x09;     //Entity (0x09)
  uint16_t Length = 8;      //数据帧长度
  unsigned char *data;
  data = (unsigned char *)calloc(Length,1);
  res = uvc_set_ctrl(
      devh,
      Unit,
      Ctrl,
      data,
      Length
  );
    if (res != Length) {
    printf("Failed to set camera gimbal to center\n");
    printf("res = %d\n", res);
  } else {
    printf("Set camera gimbal to center control request sent successfully\n");
  }
}

//zoom from 100 to 400
void set_camera_zoom_absolute(uvc_device_handle_t *devh, int zoom) {
  int res;
  uint16_t Ctrl = 0x0b;     //Control Selector (0x0b)
  uint16_t Unit = 0x01;     //Entity (0x01)
  uint16_t Length = 2;      //数据帧长度
  unsigned char *data;
  data = (unsigned char *)malloc(Length);
  data[0] = zoom & 0xff;
  data[1] = (zoom >> 8) & 0xff;
  // Send the control request
  res = uvc_set_ctrl(
      devh,
      Unit,
      Ctrl,
      data,
      Length
  );

  if (res != Length) {
    printf("Failed to set camera zoom\n");
    printf("res = %d\n", res);
  } else {
    printf("Control request sent successfully\n");
  }
}

//zoom from 100 to 400
//horizontal_location from -1397 to 1385
//vertical_location from -536 to 846
void set_camera_gimbal_location(uvc_device_handle_t *devh,int horizontal_location,int vertical_location,int zoom) {
  int res;
  uint16_t Ctrl = 0x02;     //Control Selector (0x02)
  uint16_t Unit = 0x09;     //Entity (0x09)
  uint16_t Length = 52;      //数据帧长度
  unsigned char *data;
  data = (unsigned char *)calloc(Length,1);
  
  data[50] = zoom & 0xff;
  data[51] = (zoom >> 8) & 0xff;

  data[42] = (vertical_location >= 0) ? vertical_location & 0xff : ~(-vertical_location & 0xff) + 1;
  data[43] = (vertical_location >= 0) ? (vertical_location >> 8) & 0xff : ~((-vertical_location >> 8) & 0xff) + 1;
  data[44] = (vertical_location >= 0) ? (vertical_location >> 16) & 0xff : ~((-vertical_location >> 16) & 0xff) + 1;
  data[45] = (vertical_location >= 0) ? (vertical_location >> 24) & 0xff : ~((-vertical_location >> 24) & 0xff) + 1;

  data[38] = (horizontal_location >= 0) ? horizontal_location & 0xff : ~(-horizontal_location & 0xff) + 1;
  data[39] = (horizontal_location >= 0) ? (horizontal_location >> 8) & 0xff : ~((-horizontal_location >> 8) & 0xff) + 1;
  data[40] = (horizontal_location >= 0) ? (horizontal_location >> 16) & 0xff : ~((-horizontal_location >> 16) & 0xff) + 1;
  data[41] = (horizontal_location >= 0) ? (horizontal_location >> 24) & 0xff : ~((-horizontal_location >> 24) & 0xff) + 1;

  // Send the control request
  res = uvc_set_ctrl(
      devh,
      Unit,
      Ctrl,
      data,
      Length
  );

  if (res != Length) {
    printf("Failed to set camera gimbal location\n");
    printf("res = %d\n", res);
  } else {
    printf("Control request sent successfully\n");
  }
}

/* This callback function runs once per frame. Use it to perform any
 * quick processing you need, or have it put the frame into your application's
 * input queue. If this function takes too long, you'll start losing frames. */
void cb(uvc_frame_t *frame, void *ptr) {
  uvc_frame_t *bgr;
  uvc_error_t ret;
  auto *frame_format = (enum uvc_frame_format *)ptr;

    // 获取当前时间
  std::time_t now = std::time(0);
  std::tm* ltm = std::localtime(&now);

  // 将时间转换为字符串
  std::stringstream ss;
  ss << 1900 + ltm->tm_year << "-"
      << 1 + ltm->tm_mon << "-"
      << ltm->tm_mday << " "
      << ltm->tm_hour << ":"
      << ltm->tm_min << ":"
      << ltm->tm_sec;
  std::string timestamp = ss.str();

  /* We'll convert the image from YUV/JPEG to BGR, so allocate space */
  bgr = uvc_allocate_frame(frame->width * frame->height * 3);
  if (!bgr) {
    printf("unable to allocate bgr frame!\n");
    return;
  }

  // printf("callback! frame_format = %d, width = %d, height = %d, length = %lu, ptr = %p\n",
  //   frame->frame_format, frame->width, frame->height, frame->data_bytes, ptr);

  cv::Mat mat;
  if (frame->frame_format == UVC_FRAME_FORMAT_MJPEG) {
    // 将UVC帧的数据转换为OpenCV的Mat
    std::vector<uchar> mjpegdata(static_cast<uchar*>(frame->data), static_cast<uchar*>(frame->data) + frame->data_bytes);
    mat = cv::imdecode(mjpegdata, cv::IMREAD_COLOR); // 解码MJPEG数据
    cv::Size textSize = cv::getTextSize(timestamp, fontFace, fontScale, thickness, &baseline);
    cv::Point textOrg(mat.cols - textSize.width - 10, textSize.height + 10);
    // 在图像上添加时间戳
    cv::putText(mat, timestamp, textOrg, fontFace, fontScale, color, thickness);

  } else {
    // 处理其他格式或错误
    std::cerr << "Frame format is not MJPEG!" << std::endl;
    return;
  }
  // 显示图像
  if (!mat.empty()) {
    // 显示图像
    out.write(mat);
    std::cout << "write frame to server" << std::endl;
    // cv::imshow("UVC Test", mat);
    // 等待1ms，以便OpenCV可以处理事件
    cv::waitKey(1);
  } else {
    std::cerr << "Could not decode MJPEG frame!" << std::endl;
  }

  uvc_free_frame(bgr);
}