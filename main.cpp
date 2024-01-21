#include "libuvc/libuvc.h"
#include <cstdio>
#include <unistd.h>
#include <opencv2/opencv.hpp>

void set_camera_gimbal_control(uvc_device_handle_t *devh,const char horizontal_direction,const char vertical_direction,const char horizontal_speed,const char vertical_speed) {
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

/* This callback function runs once per frame. Use it to perform any
 * quick processing you need, or have it put the frame into your application's
 * input queue. If this function takes too long, you'll start losing frames. */
void cb(uvc_frame_t *frame, void *ptr) {
  uvc_frame_t *bgr;
  uvc_error_t ret;
  auto *frame_format = (enum uvc_frame_format *)ptr;
  // FILE *fp;
  // static int jpeg_count = 0;
  // static const char *H264_FILE = "iOSDevLog.h264";
  // static const char *MJPEG_FILE = ".jpeg";
  // char filename[16];

  /* We'll convert the image from YUV/JPEG to BGR, so allocate space */
  bgr = uvc_allocate_frame(frame->width * frame->height * 3);
  if (!bgr) {
    printf("unable to allocate bgr frame!\n");
    return;
  }

  printf("callback! frame_format = %d, width = %d, height = %d, length = %lu, ptr = %p\n",
    frame->frame_format, frame->width, frame->height, frame->data_bytes, ptr);

  switch (frame->frame_format) {
  case UVC_FRAME_FORMAT_H264:
    /* use `ffplay H264_FILE` to play */
    /* fp = fopen(H264_FILE, "a");
     * fwrite(frame->data, 1, frame->data_bytes, fp);
     * fclose(fp); */
    break;
  case UVC_COLOR_FORMAT_MJPEG:
    // sprintf(filename, "%d%s", jpeg_count++, MJPEG_FILE);
    // fp = fopen(filename, "w");
    // fwrite(frame->data, 1, frame->data_bytes, fp);
    // fclose(fp);
    break;
  case UVC_COLOR_FORMAT_YUYV:
    /* Do the BGR conversion */
    ret = uvc_any2bgr(frame, bgr);
    if (ret) {
      uvc_perror(ret, "uvc_any2bgr");
      uvc_free_frame(bgr);
      return;
    }
    break;
  default:
    break;
  }

  if (frame->sequence % 30 == 0) {
    printf(" * got image %u\n",  frame->sequence);
  }

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
      int width = 1920;
      int height = 1080;
      int fps = 30;

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

          //摄像头不支持自动曝光，所以这里注释掉
          // /* enable auto exposure - see uvc_set_ae_mode documentation */
          // puts("Enabling auto exposure ...");
          // const uint8_t UVC_AUTO_EXPOSURE_MODE_AUTO = 2;
          // res = uvc_set_ae_mode(devh, UVC_AUTO_EXPOSURE_MODE_AUTO);
          // if (res == UVC_SUCCESS) {
          //   puts(" ... enabled auto exposure");
          // } else if (res == UVC_ERROR_PIPE) {
          //   /* this error indicates that the camera does not support the full AE mode;
          //    * try again, using aperture priority mode (fixed aperture, variable exposure time) */
          //   puts(" ... full AE not supported, trying aperture priority mode");
          //   const uint8_t UVC_AUTO_EXPOSURE_MODE_APERTURE_PRIORITY = 8;
          //   res = uvc_set_ae_mode(devh, UVC_AUTO_EXPOSURE_MODE_APERTURE_PRIORITY);
          //   if (res < 0) {
          //     uvc_perror(res, " ... uvc_set_ae_mode failed to enable aperture priority mode");
          //   } else {
          //     puts(" ... enabled aperture priority auto exposure mode");
          //   }
          // } else {
          //   uvc_perror(res, " ... uvc_set_ae_mode failed to enable auto exposure mode");
          // }
          set_camera_gimbal_control(devh,0xff,0x04,0x00,0x01);//控制帧发送
          sleep(10); /* stream for 10 seconds */

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