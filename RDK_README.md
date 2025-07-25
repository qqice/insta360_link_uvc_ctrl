# RDK YOLO11 集成说明

本项目已集成RDK开发板的YOLO11推理功能，可以在D-Robotics RDK板端运行高效的目标检测。

## 功能特点

1. **原生RDK BPU支持**: 直接使用RDK的BPU(Brain Processing Unit)进行硬件加速推理
2. **YOLO11模型支持**: 支持最新的YOLO11检测模型
3. **高效图像预处理**: 支持LetterBox预处理，保持图像宽高比
4. **NV12格式优化**: 直接支持NV12格式输入，减少格式转换开销
5. **向后兼容**: 通过条件编译支持原有的RKNN实现

## 编译配置

### RDK模式编译
```bash
mkdir build && cd build
cmake -DUSE_RDK=ON ..
make
```

### 传统RKNN模式编译
```bash
mkdir build && cd build
cmake -DUSE_RDK=OFF ..
make
```

## 模型要求

RDK实现需要使用D-Robotics专用的`.bin`格式模型文件，而不是`.rknn`文件。

### 模型文件路径
- RDK模式: `../model/yolo11x_detect_bayese_640x640_nv12_modified.bin`
- RKNN模式: `../model/yolov8n.rknn`

### 模型规格要求
- 输入尺寸: 640x640
- 输入格式: NV12
- 输出数量: 6个(对应3个尺度的分类和边界框输出)
- 类别数量: 80(COCO数据集)

## 使用方式

### 1. 集成在主程序中
主程序会根据编译时的`USE_RDK`宏定义自动选择使用RDK或RKNN推理引擎。

### 2. 独立推理演示
```bash
# RDK推理演示
./rdk_inference_demo model.bin input.jpg output.jpg

# 传统RKNN推理演示  
./inference_demo -m model.rknn -i input.jpg
```

## RDK推理流程

1. **模型加载**: 使用`hbDNNInitializeFromFiles`加载.bin模型
2. **输入验证**: 验证模型输入为NV12格式，NCHW布局
3. **图像预处理**: BGR→YUV_I420→NV12格式转换
4. **BPU推理**: 使用`hbDNNInfer`进行硬件加速推理
5. **后处理**: DFL解码、NMS等后处理操作
6. **结果渲染**: 在原图上绘制检测框和标签

## 性能优势

相比传统RKNN实现，RDK实现具有以下优势：

1. **硬件加速**: 直接使用BPU硬件加速，性能更高
2. **内存优化**: 减少不必要的数据拷贝和格式转换
3. **延迟更低**: 优化的推理流水线，端到端延迟更低
4. **资源占用少**: 更高效的资源利用

## 依赖库

RDK实现需要以下额外的库：

- `libdnn`: RDK BPU推理库
- `libhbmem`: RDK内存管理库

这些库通常随RDK SDK一起提供。

## 注意事项

1. **平台兼容性**: RDK实现仅适用于D-Robotics RDK开发板
2. **模型格式**: 必须使用D-Robotics工具链转换的.bin格式模型
3. **SDK版本**: 确保使用兼容的RDK SDK版本
4. **内存管理**: BPU内存需要正确分配和释放，避免内存泄漏

## 故障排除

### 常见问题

1. **模型加载失败**: 检查模型文件路径和格式
2. **BPU初始化失败**: 确认RDK SDK正确安装
3. **推理结果异常**: 验证输入图像格式和模型匹配
4. **性能不佳**: 检查BPU资源占用和并发设置

### 调试建议

1. 启用详细日志输出
2. 检查模型输入输出规格
3. 验证图像预处理流程
4. 监控BPU资源使用情况
