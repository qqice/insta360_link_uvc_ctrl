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

# 寻找spdlog库
find_package(spdlog REQUIRED)

# 寻找bytetrack库
find_package(bytetrack REQUIRED)

# 寻找curl库
find_package(CURL REQUIRED)

# 添加libuvc的头文件和库路径
set(libuvc_DIR /usr/local/lib/cmake/libuvc)
find_package(libuvc REQUIRED)

# 添加选项来控制是否使用RDK推理
option(USE_RDK "Compile with RDK inference support" ON)

# 添加选项来控制是否编译 realsense 相关的代码
option(USE_REALSENSE "Compile with RealSense support" OFF)

if (USE_RDK)
    message(STATUS "Compile with RDK inference support")
    add_definitions(-DUSE_RDK)
    
    # 添加RDK BPU相关的库路径和头文件
    # 这些路径需要根据实际的RDK SDK安装位置调整
    set(RDK_SDK_PATH "/usr/include/dnn")

    set(DNN_LIB_PATH "/usr/lib/")
    
    if(EXISTS ${RDK_SDK_PATH})
        include_directories(${RDK_SDK_PATH}/include)
        link_directories(${RDK_SDK_PATH}/lib)
        list(APPEND PROJECT_LIBS
            dnn
            rt
            dl
        )
    else()
        message(WARNING "RDK SDK not found at ${RDK_SDK_PATH}, you may need to set the correct path")
    endif()
    
    list(APPEND PROJECT_SOURCES
        include/rdk_utils.h
        src/rdk_utils.cpp
    )
else ()
    message(STATUS "Compile without RDK inference support")
endif()

if (USE_REALSENSE)
    message(STATUS "Compile with RealSense support")
    add_definitions(-DUSE_REALSENSE)
    find_package(realsense2 REQUIRED)
    list(APPEND PROJECT_SOURCES
        include/realsense_utils.h
        src/realsense_utils.cpp
    )
    list(APPEND PROJECT_LIBS
        ${realsense2_LIBRARY}
    )
else ()
    message(STATUS "Compile without RealSense support")
endif()

include_directories(${OpenCV_INCLUDE_DIRS} ${CURL_INCLUDE_DIRS})
list(APPEND PROJECT_SOURCES
    src/main.cpp
    include/mqtt_utils.h
    src/mqtt_utils.cpp
    include/uvc_utils.h
    src/uvc_utils.cpp
    src/inference_utils.cpp
    include/inference_utils.h
    src/upload_utils.cpp
    include/upload_utils.h
)

add_executable(UVCCapture ${PROJECT_SOURCES})
target_link_libraries(UVCCapture
    ${OpenCV_LIBS}
    LibUVC::UVCShared
    PkgConfig::Mosquitto
    Threads::Threads
    nlohmann_json::nlohmann_json
    spdlog::spdlog
    ${bytetrack_LIBS}
    ${CURL_LIBRARIES}
    ${PROJECT_LIBS}
)

# 添加V4L2视频捕获程序
list(APPEND V4L2_PROJECT_SOURCES
    src/main_v4l2.cpp
    include/mqtt_utils.h
    src/mqtt_utils.cpp
    include/uvc_utils.h
    src/uvc_utils.cpp
    src/inference_utils.cpp
    include/inference_utils.h
    src/upload_utils.cpp
    include/upload_utils.h
    ${PROJECT_SOURCES}
)

# 去除重复的源文件
list(REMOVE_DUPLICATES V4L2_PROJECT_SOURCES)
# 移除原main.cpp，使用main_v4l2.cpp
list(REMOVE_ITEM V4L2_PROJECT_SOURCES src/main.cpp)

add_executable(V4L2Capture ${V4L2_PROJECT_SOURCES})
target_link_libraries(V4L2Capture
    ${OpenCV_LIBS}
    LibUVC::UVCShared
    PkgConfig::Mosquitto
    Threads::Threads
    nlohmann_json::nlohmann_json
    spdlog::spdlog
    ${bytetrack_LIBS}
    ${CURL_LIBRARIES}
    ${PROJECT_LIBS}
)

# 添加RDK推理演示程序
if (USE_RDK)
    add_executable(rdk_inference_demo src/rdk_inference_demo.cpp src/rdk_utils.cpp)
    target_link_libraries(rdk_inference_demo
        ${OpenCV_LIBS}
        spdlog::spdlog
        ${PROJECT_LIBS}
    )
endif()