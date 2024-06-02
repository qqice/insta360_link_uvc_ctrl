cmake_minimum_required(VERSION 3.21)
project(UVCCapture)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

include_directories(include)

add_subdirectory(utils)

# 寻找线程库
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# 寻找OpenCV库
find_package(OpenCV REQUIRED)

# 寻找libmosquitto库
find_package(PkgConfig REQUIRED)
pkg_check_modules(Mosquitto IMPORTED_TARGET libmosquitto REQUIRED)

# 寻找nlohmann_json库
find_package(nlohmann_json REQUIRED)

# 寻找realsense2库
find_package(realsense2 REQUIRED)

# 寻找spdlog库
find_package(spdlog REQUIRED)

# 寻找bytetrack库
find_package(bytetrack REQUIRED)

# 寻找curl库
find_package(CURL REQUIRED)

# 添加libuvc的头文件和库路径
set(libuvc_DIR /usr/local/lib/cmake/libuvc)
find_package(libuvc REQUIRED)


include_directories(${OpenCV_INCLUDE_DIRS} ${realsense_INCLUDE_DIR} ${CURL_INCLUDE_DIRS})
set(PROJECT_SOURCES
        src/main.cpp
        include/mqtt_utils.h
        src/mqtt_utils.cpp
        include/uvc_utils.h
        src/uvc_utils.cpp
        include/realsense_utils.h
        src/realsense_utils.cpp
        src/inference_utils.cpp
        include/inference_utils.h
        src/upload_utils.cpp
        include/upload_utils.h
)


add_executable(UVCCapture ${PROJECT_SOURCES})
target_link_libraries(UVCCapture
        ${OpenCV_LIBS}
        ${realsense2_LIBRARY}
        LibUVC::UVCShared
        PkgConfig::Mosquitto
        Threads::Threads
        nlohmann_json::nlohmann_json
        spdlog::spdlog
        yolov8_lib
        ${bytetrack_LIBS}
        ${CURL_LIBRARIES}
)

add_executable(inference_demo src/rknn_inference.cpp)
target_link_libraries(inference_demo
        ${OpenCV_LIBS}
        ${bytetrack_LIBS}
        spdlog::spdlog
        yolov8_lib
)

