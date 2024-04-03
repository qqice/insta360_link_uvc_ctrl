#include <libuvc/libuvc.h>
#include <cstdio>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <mosquitto.h>
#include <cstring>
#include <thread>
#include <csignal>
#include <ctime>
#include <sstream>
#include <nlohmann/json.hpp>

#include "inference.h"

#include "cos_api.h"
#include "cos_sys_config.h"
#include "cos_defines.h"

#define runOnGPU true
Inference leaf_disease_inf("/home/jetson/leaf_disease_detection.onnx", cv::Size(640, 360), "leaf_disease_classes.txt", runOnGPU);
Inference tomato_maturity_inf("/home/jetson/tomato_maturity_recognition.onnx", cv::Size(640, 360), "tomato_maturity_classes.txt", runOnGPU);

// 创建配置对象
qcloud_cos::CosConfig config("./cos_config.json");
qcloud_cos::CosAPI tcos(config);
// 存储桶名称
std::string bucket_name = "picgo-1253726783"; 

// 标志变量，用来控制循环
volatile sig_atomic_t loopFlag = 1;

// 信号处理函数
void signalHandler(int signal) {
    if (signal == SIGINT) {
        // 当接收到SIGINT信号时，修改循环控制变量
        loopFlag = 0;
    }
}

// 设置字体和颜色
int fontFace = cv::FONT_HERSHEY_SIMPLEX;
double fontScale = 1;
int thickness = 2;
cv::Scalar color(255, 255, 255); // 白色

// 计算文本宽度和高度，以便将其放置在右上角
int baseline = 0;

// MQTT设置
const char* MQTT_HOST = "127.0.0.1"; // MQTT代理服务器地址
const int MQTT_PORT = 1883; // MQTT端口
const char* MQTT_TOPIC = "camera/control"; // 订阅的主题

int width = 1920;
int height = 1080;
int fps = 30;
const int bitrate = 3000000;
const char* rtsp_server = "rtsp://127.0.0.1:8554/mystream";

cv::VideoWriter out("appsrc ! videoconvert ! video/x-raw,format=I420 ! nvvidconv ! nvv4l2h264enc preset-level=1 bitrate="+ std::to_string(bitrate) +" maxperf-enable=1 iframeinterval=" + std::to_string(fps * 2) +
              " ! video/x-h264,profile=baseline ! rtspclientsink location=" + rtsp_server,
              cv::CAP_GSTREAMER, 0, fps, cv::Size(width, height), true);

uvc_device_handle_t *devh;

void upload_pic_to_tencent_cos(cv::Mat pic, std::string pic_name) {
  // 将 cv::Mat 转换为 vector<uchar>
  std::vector<uchar> buffer;
  cv::imencode(".jpg", pic, buffer);

  // 创建内存流
  std::istringstream iss(std::string(buffer.begin(), buffer.end()));

  // 构造上传请求
  qcloud_cos::PutObjectByStreamReq req(bucket_name, pic_name, iss);

  // 关闭 MD5 校验(可选)
  req.TurnOffComputeConentMd5();

  // 执行上传
  qcloud_cos::PutObjectByStreamResp resp;
  qcloud_cos::CosResult result = tcos.PutObject(req, &resp);

  // 检查结果
  if (result.IsSucc()) {
    std::cout << "Upload success" << std::endl;
  } else {
    std::cerr << "Upload failed, error message: " << result.GetErrorMsg() << std::endl;
  }
}

void run_leaf_disease_inf(cv::Mat inf_frame) {
  std::vector<Detection> output = leaf_disease_inf.runInference(inf_frame);

  int detections = output.size();
  std::cout << "Number of detections:" << detections << std::endl;

  for (int i = 0; i < detections; ++i)
  {
    Detection detection = output[i];

    cv::Rect box = detection.box;
    cv::Scalar color = detection.color;

    // Detection box
    cv::rectangle(inf_frame, box, color, 2);

    // Detection box text
    std::string classString = detection.className + ' ' + std::to_string(detection.confidence).substr(0, 4);
    cv::Size textSize = cv::getTextSize(classString, cv::FONT_HERSHEY_DUPLEX, 1, 2, 0);
    cv::Rect textBox(box.x, box.y - 40, textSize.width + 10, textSize.height + 20);

    cv::rectangle(inf_frame, textBox, color, cv::FILLED);
    cv::putText(inf_frame, classString, cv::Point(box.x + 5, box.y - 10), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 0), 2, 0);
  }

  cv::imwrite("leaf_disease_detection.jpg", inf_frame);
}

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
            // case 5:
            //   run_leaf_disease_inf();
            //   break;
            // case 6:
            //   run_tomato_maturity_inf();
            //   break;
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
  uvc_stream_ctrl_t ctrl;
  uvc_error_t res;

  // 注册信号处理函数
  signal(SIGINT, signalHandler);
  
  // 初始化mosquitto库
  mosquitto_lib_init();

  // 创建新的mosquitto客户端实例
  struct mosquitto *mosq = mosquitto_new(nullptr, true, nullptr);
  if(!mosq){
      std::cerr << "Failed to create mosquitto instance." << std::endl;
      return -1;
  }

  // 设置消息回调
  mosquitto_message_callback_set(mosq, on_message_callback);

  // 连接到MQTT代理服务器
  if(mosquitto_connect(mosq, MQTT_HOST, MQTT_PORT, 60)){
      std::cerr << "Could not connect to MQTT Broker." << std::endl;
      return -1;
  }

  // 订阅主题
  mosquitto_subscribe(mosq, nullptr, MQTT_TOPIC, 0);

  // 创建并启动处理MQTT消息的线程
  std::thread mqtt_thread(mqtt_loop, mosq);

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
