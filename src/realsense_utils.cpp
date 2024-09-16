#include "realsense_utils.h"
int rs_frame_width = 1280;
int rs_frame_height = 720;
int rs_frame_rate = 6;

void realsense_loop() {
    try {
        rs2::colorizer color_map;
        rs2::pipeline pipe;
        rs2::config cfg;
        // 配置深度流的分辨率和帧率
        cfg.enable_stream(RS2_STREAM_DEPTH, rs_frame_width, rs_frame_height, RS2_FORMAT_Z16, rs_frame_rate);

        // 配置左右两侧的红外流的分辨率和帧率
        cfg.enable_stream(RS2_STREAM_INFRARED, 1, rs_frame_width, rs_frame_height, RS2_FORMAT_Y8, rs_frame_rate);
        cfg.enable_stream(RS2_STREAM_INFRARED, 2, rs_frame_width, rs_frame_height, RS2_FORMAT_Y8, rs_frame_rate);

        pipe.start(cfg);

        cv::VideoWriter depth_writer("appsrc ! videoconvert ! mpph264enc ! queue ! rtspclientsink location=rtsp://127.0.0.1:8554/depth", cv::CAP_GSTREAMER, 0, rs_frame_rate, cv::Size(rs_frame_width, rs_frame_height), true);
        cv::VideoWriter left_writer("appsrc ! videoconvert ! mpph264enc ! queue ! rtspclientsink location=rtsp://127.0.0.1:8554/ir_left", cv::CAP_GSTREAMER, 0, rs_frame_rate, cv::Size(rs_frame_width, rs_frame_height), false);
        cv::VideoWriter right_writer("appsrc ! videoconvert ! mpph264enc ! queue ! rtspclientsink location=rtsp://127.0.0.1:8554/ir_right", cv::CAP_GSTREAMER, 0, rs_frame_rate, cv::Size(rs_frame_width, rs_frame_height), false);

        while (true) {
            rs2::frameset frames = pipe.wait_for_frames();
            rs2::frame depth = frames.get_depth_frame().apply_filter(color_map);
            const int depth_width_img = depth.as<rs2::video_frame>().get_width();
            const int depth_height_img = depth.as<rs2::video_frame>().get_height();
            spdlog::debug("depth width: {} height: {}", depth_width_img, depth_height_img);
            cv::Mat depth_image(cv::Size(rs_frame_width, rs_frame_height), CV_8UC3, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
            depth_writer.write(depth_image);

            rs2::video_frame ir_frame_left = frames.get_infrared_frame(1);
            const int ir_left_width_img = ir_frame_left.as<rs2::video_frame>().get_width();
            const int ir_left_height_img = ir_frame_left.as<rs2::video_frame>().get_height();
            spdlog::debug("ir left width: {} height: {}", ir_left_width_img, ir_left_height_img);
            cv::Mat ir_left(cv::Size(rs_frame_width, rs_frame_height), CV_8UC1, (void*)ir_frame_left.get_data(), cv::Mat::AUTO_STEP);
            left_writer.write(ir_left);

            rs2::video_frame ir_frame_right = frames.get_infrared_frame(2);
            const int ir_right_width_img = ir_frame_right.as<rs2::video_frame>().get_width();
            const int ir_right_height_img = ir_frame_right.as<rs2::video_frame>().get_height();
            spdlog::debug("ir right width: {} height: {}", ir_right_width_img, ir_right_height_img);
            cv::Mat ir_right(cv::Size(rs_frame_width, rs_frame_height), CV_8UC1, (void*)ir_frame_right.get_data(), cv::Mat::AUTO_STEP);
            right_writer.write(ir_right);

            if (cv::waitKey(1) == 27) {
                break;
            }
        }
    } catch (const rs2::error & e) {
        spdlog::error("RealSense error calling {} ({}): {}", e.get_failed_function(), e.get_failed_args(), e.what());
    } catch (const std::exception & e) {
        spdlog::error("Exception: {}", e.what());
    }
}