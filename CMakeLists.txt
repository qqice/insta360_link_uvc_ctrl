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

# 寻找nlohmann_json库
find_package(nlohmann_json REQUIRED)

set(POCO_LIBS PocoNetSSL PocoNet PocoCrypto PocoUtil PocoJSON PocoXML PocoFoundation)
set(POCO_LINK_DIR ${CMAKE_SOURCE_DIR}/poco_lib/)
add_library(cossdk STATIC IMPORTED)
set_target_properties(cossdk PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/libcossdk.a)

# 添加libuvc的头文件和库路径
# 假设libuvc已经安装在默认的系统库路径中
# 如果不是这样，您可能需要指定include_directories和link_directories
#find_path(LIBUVC_INCLUDE_DIR NAMES libuvc/libuvc.h)
#find_library(LIBUVC_LIBRARY NAMES uvc)
set(libuvc_DIR /usr/local/lib/cmake/libuvc)
find_package(libuvc REQUIRED)
# 指定头文件路径
#include_directories(${LIBUVC_INCLUDE_DIR})

include_directories(${OpenCV_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/cos_include/ ${CMAKE_SOURCE_DIR}/poco_include/)
link_directories(${POCO_LINK_DIR})
set(PROJECT_SOURCES
    main.cpp

    inference.h
    inference.cpp
)

# 定义可执行文件
add_executable(UVCCapture ${PROJECT_SOURCES})

# 链接OpenCV和libuvc库
target_link_libraries(UVCCapture ${OpenCV_LIBS} cossdk ${POCO_LIBS} LibUVC::UVCShared PkgConfig::Mosquitto Threads::Threads nlohmann_json::nlohmann_json)
