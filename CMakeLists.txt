cmake_minimum_required(VERSION 3.7)
project(UVCCapture)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# 寻找线程库
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# 寻找OpenCV库
find_package(OpenCV REQUIRED)

# 寻找libmosquitto库
find_package(PkgConfig REQUIRED)
pkg_check_modules(Mosquitto IMPORTED_TARGET libmosquitto REQUIRED)

pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)
include_directories(${GSTREAMER_INCLUDE_DIRS})
link_directories(${GSTREAMER_LIBRARY_DIRS})
include_directories(${GSTREAMER_APP_INCLUDE_DIRS})
link_directories(${GSTREAMER_APP_LIBRARY_DIRS})
add_definitions(${GSTREAMER_CFLAGS_OTHER})

# 寻找nlohmann_json库
find_package(nlohmann_json REQUIRED)

find_package(ALSA REQUIRED)
if(ALSA_FOUND)
    include_directories(${ALSA_INCLUDE_DIRS})
endif()

# 添加libuvc的头文件和库路径
# 假设libuvc已经安装在默认的系统库路径中
# 如果不是这样，您可能需要指定include_directories和link_directories
#find_path(LIBUVC_INCLUDE_DIR NAMES libuvc/libuvc.h)
#find_library(LIBUVC_LIBRARY NAMES uvc)
set(libuvc_DIR /usr/local/lib/cmake/libuvc)
find_package(libuvc REQUIRED)
# 指定头文件路径
#include_directories(${LIBUVC_INCLUDE_DIR})

include_directories(${OpenCV_INCLUDE_DIRS})
set(PROJECT_SOURCES
    main.cpp
    inference.h
    inference.cpp
)

# 定义可执行文件
add_executable(UVCCapture ${PROJECT_SOURCES})
add_executable(inf_example inf_example.cpp inference.cpp inference.h)
# 链接OpenCV和libuvc库
target_link_libraries(UVCCapture ${OpenCV_LIBS} ${GSTREAMER_LIBRARIES} ${GSTREAMER_APP_LIBRARIES} ${ALSA_LIBRARIES} LibUVC::UVCShared PkgConfig::Mosquitto Threads::Threads nlohmann_json::nlohmann_json)
target_link_libraries(inf_example ${OpenCV_LIBS})
