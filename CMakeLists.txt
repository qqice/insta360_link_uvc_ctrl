cmake_minimum_required(VERSION 3.7)
project(UVCCapture)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# 寻找OpenCV库
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# 添加libuvc的头文件和库路径
# 假设libuvc已经安装在默认的系统库路径中
# 如果不是这样，您可能需要指定include_directories和link_directories
#find_path(LIBUVC_INCLUDE_DIR NAMES libuvc/libuvc.h)
#find_library(LIBUVC_LIBRARY NAMES uvc)
set(libuvc_DIR /usr/local/lib/cmake/libuvc)
find_package(libuvc REQUIRED)
# 指定头文件路径
#include_directories(${LIBUVC_INCLUDE_DIR})

# 定义可执行文件
add_executable(UVCCapture main.cpp)

# 链接OpenCV和libuvc库
target_link_libraries(UVCCapture ${OpenCV_LIBS} LibUVC::UVCShared)