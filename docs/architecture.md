# 架构说明

## 1. 分层

```
apps/                 CLI：list / capture / calib / lut / stitch
  ↓
src/                  hikquad_core（静态库）
  ├─ camera.cpp       MVS 句柄、参数、取流、像素转换
  ├─ camera_array.cpp 多路打开、并行取流、软件触发、帧集
  ├─ device_info.cpp  枚举、虚拟相机过滤、sysfs USB 拓扑
  ├─ config.cpp       JSON 配置
  ├─ calibration.cpp  棋盘检测、内参、逐视图位姿、外参图
  ├─ calib_io.cpp     quad-camera 兼容 YAML/JSON
  ├─ lut.cpp          圆柱投影 LUT / weight
  └─ stitcher.cpp     feather 加权拼接
  ↓
MVS SDK      OpenCV
```

`hikquad_core` 是普通 CMake 静态库；`apps/` 和 `tests/` 只通过公共头文件
`include/hikquad/*.hpp` 使用它。没有 ROS2 依赖。

## 2. 相机生命周期

```
enumerateUsbDevices()
  └─ MV_CC_EnumDevicesEx2(MV_USB_DEVICE, "Hikrobot", SortMethod_SerialNumber)
       └─ 过滤序列号以 "Vir" 开头的虚拟相机
Camera::open(config)
  ├─ 按 serial 或排序后的 index 找到 MV_CC_DEVICE_INFO
  ├─ MV_CC_CreateHandle / MV_CC_OpenDevice(MV_ACCESS_Exclusive)
  ├─ apply(config)：ROI → PixelFormat → ADC/帧率 → 触发 → 曝光/增益/白平衡
  │                 → 图像节点数 → grab strategy → USB 吞吐/TransferSize
  └─ Camera::start()：MV_CC_StartGrabbing
Camera::grab()
  ├─ MV_CC_GetImageBuffer(timeout)
  ├─ MV_CC_ConvertPixelTypeEx(BayerRG8 → BGR8)
  ├─ 复制元数据/时间戳；RAII 确保 MV_CC_FreeImageBuffer
  └─ 返回 Frame
```

控制接口（open/close/start/stop/参数写入）由 `Camera::Impl::mutex` 串行化；
`grab()` 故意不持有该锁，避免阻塞参数更新。调用方需保证**同一台相机同一时刻
只有一个 grab 线程**，且不要在 grab 期间 close/stop。

## 3. 多路取流

`CameraArray::captureOnce()`：

1. 若配置了软件触发，先对 4 台相机依次发送 `TriggerSoftware`；
2. 用 `std::async` 为每台相机启动一个 grab 任务；
3. 收集所有 future；任何一个失败都会在全部任务结束后重新抛出，避免悬空引用；
4. 用各帧 `host_time` 计算 `skew_ms`（仅主机接收时间差，不是曝光同步误差）。

`CameraArray::open()` 先打开全部句柄，再统一 start；任何一台失败都会关闭已打开
的相机，避免“只读到 2 台”时残留占用。

## 4. 标定数据流

```
discoverViews(root)
  ├─ root/cam0/*.png ... root/cam3/*.png      （按文件名排序配对）
  └─ root/view_000/cam0.png ... cam3.png      （按目录配对）
detectBoard()
  ├─ findChessboardCornersSB(EXHAUSTIVE | ACCURACY)
  └─ 失败时 findChessboardCorners + cornerSubPix
calibrateCamera() 每路：
  ├─ cv::fisheye::calibrate 或 cv::calibrateCamera
  ├─ 逐视图重投影误差
  └─ 逐视图 board->camera 位姿
estimateExtrinsics()：
  ├─ 每对相机取共同 view，计算 X_i = R_ij X_j + t_ij
  ├─ 四元数对齐后平均旋转，逐轴中值平均平移
  ├─ BFS 以相机 0 为根组合出 X_i = R_i0 X_0 + t_i0
  └─ 转成 quad-camera 约定 X_world = R_cw X_cam + T_cw
buildCylindricalLut()：
  └─ 输出像素 → 世界方向 → 每路鱼眼投影 → mapx/mapy + 角度权重
```

外参图的边要求至少 2 个共同视图；没有边的相机会回退到名义 yaw
（默认 0/90/180/270），并在 `report.json` 的 `warnings` 中记录。

## 5. 坐标约定

- 世界系 = 相机 0 系，OpenCV 相机约定：+X 右、+Y 下、+Z 前。
- `X_world = R_cw · X_cam + T_cw`，因此：
  - `R_cw` 的第三列是相机光轴在世界系中的方向；
  - `T_cw` 是相机光心在世界系中的坐标。
- 圆柱输出：`θ = (u + 0.5 - W/2) / f`，`f = W / fov`，
  方向 `(sinθ, y/f, cosθ)` 归一化。
- 鱼眼等距模型：`θ = atan2(r, z)`，`θ_d = θ(1 + k1θ² + k2θ⁴ + k3θ⁶ + k4θ⁸)`，
  `u = fx θ_d x/r + cx`。

## 6. 错误与资源

- 所有 MVS 返回值经 `checkMvs()` 转成 `SdkError`，包含十六进制错误码、
  错误名和可选 hint。
- `MV_CC_GetImageBuffer` 的 `MV_E_NODATA` 被视为超时，返回 `false`；
  其他错误抛出。
- SDK 使用进程级引用计数（`MV_CC_Initialize/Finalize`），支持多个
  `Camera` 对象。
- 帧缓冲通过 RAII guard 释放；异常路径不会泄漏 SDK buffer。

## 7. 扩展点

- 新增相机型号：在 `pixel_format.cpp` 增加格式表；在 `CameraConfig` 里
  加入需要的 GenICam 节点；参数写入用 `setEnum/setInt/setFloat/setBool` 的
  “required / optional” 语义。
- 新增标定模型：扩展 `CameraModel`，在 `calibrateCamera`、
  `buildCylindricalLut` 中加入投影分支。
- 新增加速度：`Camera::grab` 已把 BGR 转换放在主机侧；可改为零拷贝
  `cv::Mat` 包装 + 立即 remap，但要注意 SDK buffer 生命周期。
