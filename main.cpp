#include "libuvc/libuvc.h"
#include <cstdio>
#include <unistd.h>
#include <opencv2/opencv.hpp>

int width = 1920;
int height = 1080;
int fps = 30;
const int bitrate = 3000;
const char* rtsp_server = "rtsp:////10.0.0.137:8554/mystream";

cv::VideoWriter out("appsrc ! videoconvert ! video/x-raw,format=I420 ! x264enc speed-preset=ultrafast bitrate="+ std::to_string(bitrate) +" key-int-max=" + std::to_string(fps * 2) +
              " ! video/x-h264,profile=baseline ! rtspclientsink location=" + rtsp_server,
              cv::CAP_GSTREAMER, 0, fps, cv::Size(width, height), true);

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

  /* We'll convert the image from YUV/JPEG to BGR, so allocate space */
  bgr = uvc_allocate_frame(frame->width * frame->height * 3);
  if (!bgr) {
    printf("unable to allocate bgr frame!\n");
    return;
  }

  printf("callback! frame_format = %d, width = %d, height = %d, length = %lu, ptr = %p\n",
    frame->frame_format, frame->width, frame->height, frame->data_bytes, ptr);

  cv::Mat mat;
  if (frame->frame_format == UVC_FRAME_FORMAT_MJPEG) {
    // 将UVC帧的数据转换为OpenCV的Mat
    std::vector<uchar> mjpegdata(static_cast<uchar*>(frame->data), static_cast<uchar*>(frame->data) + frame->data_bytes);
    mat = cv::imdecode(mjpegdata, cv::IMREAD_COLOR); // 解码MJPEG数据
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
    cv::imshow("UVC Test", mat);
    // 等待1ms，以便OpenCV可以处理事件
    cv::waitKey(1);
  } else {
    std::cerr << "Could not decode MJPEG frame!" << std::endl;
  }

  uvc_free_frame(bgr);
}

int main(int argc, char **argv) {
  uvc_context_t *ctx;
  uvc_device_t *dev;
  uvc_device_handle_t *devh;
  uvc_stream_ctrl_t ctrl;
  uvc_error_t res;

  /* Initialize a UVC service context. Libuvc will set up its own libusb
   * context. Replace NULL with a libusb_context pointer to run libuvc
   * from an existing libusb context. */
  res = uvc_init(&ctx, NULL);

  if (res < 0) {
    uvc_perror(res, "uvc_init");
    return res;
  }

  puts("UVC initialized");

  /* Locates the first attached UVC device, stores in dev */
  res = uvc_find_device(
      ctx, &dev,
      0, 0, NULL); /* filter devices: vendor_id, product_id, "serial_num" */

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

          set_camera_gimbal_control(devh,0x00,0x01,0x00,0x01);//控制帧发送

          sleep(600); /* stream for 10 minutes */

          stop_camera_gimbal_control(devh);
          set_camera_gimbal_to_center(devh);
          /* End the stream. Blocks until last callback is serviced */
          uvc_stop_streaming(devh);
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

  return 0;
}