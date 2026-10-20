# hik-quad

4 路 HIKROBOT（海康机器人）USB3 Vision 工业相机的**纯 CMake** 采集、标定与柱面拼接工程。

本仓库不依赖 ROS / ROS2 / ament / colcon，只使用：

- CMake ≥ 3.16 + C++17
- OpenCV 4.x（core / imgproc / imgcodecs / calib3d）
- HIKROBOT MVS Camera SDK（C API，`libMvCameraControl`）
- Linux（开发与验证环境为 Ubuntu 24.04，x86_64）

> ROS2 版本的 `mvs_camera` 只作为 MVS 取流、参数写入和异常处理的参考；
> 本工程没有引用任何 ROS2 头文件、消息、launch 或构建系统。

## 1. 已支持的硬件

| 项目 | 内容 |
|---|---|
| 相机 | HIKROBOT MV-CB016-10UC-S × 4 |
| 传感器 | IMX273，1/2.9"，全局快门，彩色 |
| 分辨率 | 1440 × 1080 |
| 像元 | 3.45 µm × 3.45 µm |
| 接口 | USB 3.0，USB3 Vision / GenICam |
| 像素格式 | BayerRG8（默认）、BayerRG10/12、BGR8、RGB8、Mono8 等 |
| 镜头 | M12，鱼眼/广角（标定模型默认 `fisheye`） |

本机实测：

- 4 台物理相机**同时**以 1440×1080 / BayerRG8 / 165 fps 取流 60 帧，
  4 路均无丢帧、无错误；
- MVS 还会枚举出一台虚拟相机（序列号以 `Vir` 开头），本工程用
  `MV_CC_EnumDevicesEx2()` 将其过滤，只保留 4 台物理相机；
- 4 路 30 fps 采集一帧的软件触发时间戳离散约 3~4 ms；静态标定场景足够，
  运动场景建议使用硬件触发（见 §7）。

## 2. 功能

- **设备发现**：序列号、型号、USB 端口、链路速率（读取 sysfs）、虚拟相机过滤。
- **驱动封装**：RAII 句柄、统一错误码、参数写入、Bayer→BGR 主机端转换、raw 缓存。
- **多路采集**：4 路并行取流、软件触发、帧集时间戳与 skew 统计、PNG/JPG/BMP 落盘。
- **USB 调优**：`DeviceLinkThroughputLimit` / `TransferSize` / `MV_CC_SetImageNodeNum`
  尽力写入；`usbfs_memory_mb`、udev、拓扑与带宽预算检查脚本。
- **标定**：8×11 内角点、100 mm 棋盘格；鱼眼/针孔模型；内参、逐视图外参、
  相机间相对位姿、以 0 号相机为世界系的 `R_cw/T_cw`。
- **quad-camera 兼容产物**：`intrinsics.yml`、`extrinsics.yml`、`lut.bin`、
  `weight.bin`、`calib_meta.json`，可直接交给 `quad-camera` 的 `panostitch`。
- **拼接验证**：内置圆柱投影 LUT 生成与 feather 加权拼接，不依赖 quad-camera 可执行文件。
- **测试**：像素格式/颜色映射、配置解析、标定 I/O、LUT 二进制、旋转平均、
  外参传递、合成内参标定等单元测试。

## 3. 目录结构

```
.
├─ CMakeLists.txt
├─ CMakePresets.json
├─ cmake/
│  ├─ CompilerWarnings.cmake
│  ├─ HikSDK.cmake
│  └─ HikQuadInstall.cmake
├─ config/
│  └─ default.json              # 默认配置（相对路径相对于本文件）
├─ include/hikquad/             # 公共头文件
│  ├─ camera.hpp
│  ├─ camera_array.hpp
│  ├─ calibration.hpp
│  ├─ calib_io.hpp
│  ├─ lut.hpp
│  ├─ stitcher.hpp
│  ├─ config.hpp
│  ├─ device_info.hpp
│  ├─ pixel_format.hpp
│  ├─ error.hpp
│  ├─ log.hpp
│  ├─ cli.hpp
│  └─ paths.hpp
├─ src/                         # 库实现
├─ apps/
│  ├─ hik_quad_list.cpp         # 枚举/拓扑
│  ├─ hik_quad_capture.cpp      # 多路采集
│  ├─ hik_quad_calib.cpp        # 标定 + 生成 quad-camera 产物
│  ├─ hik_quad_lut.cpp          # 由已有内外参重建 LUT
│  └─ hik_quad_stitch.cpp       # 单帧 4 路拼接
├─ tests/                       # CTest 单元测试
├─ scripts/
├─ docs/
├─ data/                        # 采集/标定数据（默认不纳入版本库）
└─ output/                      # 拼接结果（默认不纳入版本库）
```

## 4. 构建

### 4.1 依赖

```bash
sudo apt install build-essential cmake ninja-build libopencv-dev
# MVS SDK 由海康官网下载安装，默认路径 /opt/MVS
```

确认 SDK：

```bash
ls /opt/MVS/include/MvCameraControl.h
ls /opt/MVS/lib/64/libMvCameraControl.so
```

### 4.2 编译

```bash
cmake --preset release
cmake --build --preset release
ctest --preset release --output-on-failure
```

或传统方式：

```bash
cmake -S . -B build/release -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/release
ctest --test-dir build/release --output-on-failure
```

若 SDK 不在默认路径：

```bash
cmake --preset release -DHIK_SDK_ROOT=/path/to/MVS
```

产物在 `build/release/apps/`。CMake 会自动把 `/opt/MVS/lib/64` 写入
RUNPATH，因此通常**不需要**先 `source /opt/MVS/bin/set_env_path.sh`；
程序启动时也会补齐 `MVCAM_COMMON_RUNENV` 等环境变量。

可选安装：

```bash
cmake --install build/release --prefix /usr/local
hik_quad_list
```

安装后配置位于 `share/hik-quad/config/default.json`，程序会从可执行文件
的 `../share/hik-quad/config/` 自动找到它；也可以用 `--config` 指定。

## 5. 快速开始

### 5.1 查看相机

```bash
./build/release/apps/hik_quad_list
./build/release/apps/hik_quad_list --json
```

输出示例：

```
physical USB3 Vision cameras: 4

#   Serial        Model                   USB port      Speed
------------------------------------------------------------------
0   DB1505181     MV-CB016-10UC-S         2-1           5000M
1   DB1505182     MV-CB016-10UC-S         4-1           5000M
2   DB1505189     MV-CB016-10UC-S         4-2.2         5000M
3   DB1505196     MV-CB016-10UC-S         4-2.1         5000M
```

### 5.2 采集

```bash
# 四路各一帧，软件触发，20 ms 曝光
./build/release/apps/hik_quad_capture \
  --frames 1 --software-trigger --exposure 20000 --gain 10 \
  --out data/captures/demo

# 连续 10 组，每组间隔 0.5 s（输出 frame_000 ... frame_009）
./build/release/apps/hik_quad_capture --frames 10 --interval 0.5 --out data/captures/run1
```

输出：

```
data/captures/demo/
├─ cam0.png
├─ cam1.png
├─ cam2.png
├─ cam3.png
└─ meta.json          # 序列号、时间戳、曝光、增益、skew、像素格式
```

`--raw` 会额外保存传输原始缓存（Bayer 8 bit 存为 `camN_raw.png`，
packed 格式存为 `camN_raw.bin`）。默认在主机端用 MVS
`MV_CC_ConvertPixelTypeEx()` 把 BayerRG8 转成 BGR8，再交给 OpenCV。

### 5.3 标定

准备 8×11 内角点、100 mm 方格、带白色外边的棋盘格。推荐流程：

```bash
# 交互式采集 20 个视角；每次移动棋盘后回车
./scripts/capture_calibration.sh 20

# 检查检测率
./build/release/apps/hik_quad_calib --input data/calib_images --check

# 标定 + 生成 LUT/weight
./build/release/apps/hik_quad_calib \
  --input data/calib_images \
  --output data/calib \
  --pano 2048x512 --fov 360

# 查看产物
find data/calib -maxdepth 2 -type f
```

标定产物：

```
data/calib/
├─ camera_0/{intrinsics.yml,extrinsics.yml,lut.bin,weight.bin}
├─ camera_1/...
├─ camera_2/...
├─ camera_3/...
├─ calib_meta.json
└─ report.json
```

### 5.4 拼接

```bash
./build/release/apps/hik_quad_stitch \
  --calib data/calib \
  --frames data/captures/demo \
  --out output/panorama.jpg
```

### 5.5 由已有内外参重建 LUT

```bash
./build/release/apps/hik_quad_lut \
  --calib data/calib --output /tmp/new_lut \
  --pano 2048x512 --fov 360 --sigma 35
```

该命令也能直接读取 `quad-camera/data/calib`，用于验证 LUT 兼容性。

## 6. 配置

默认配置为 `config/default.json`。所有相对路径**相对于配置文件所在目录**。

```json
{
  "camera_count": 4,
  "usb": {
    "image_node_num": 6,
    "throughput_limit_mode": false,
    "grab_timeout_ms": 3000,
    "grab_strategy": "OldestFirst"
  },
  "defaults": {
    "width": 1440,
    "height": 1080,
    "pixel_format": "BayerRG8",
    "adc_bit_depth": "Bits_12",
    "frame_rate": 30.0,
    "exposure_us": 5000.0,
    "gain_db": 0.0,
    "white_balance_auto": "Off"
  },
  "cameras": [],
  "panorama": { "out_width": 2048, "out_height": 512, "fov_h_deg": 360.0 }
}
```

- `cameras` 为空时，按序列号排序自动选前 `camera_count` 台。
- 需要固定“物理位置 ↔ 序号”时，在 `cameras` 中写 4 个对象并用 `serial` 指定，
  例如 `{"serial": "DB1505181", "exposure_us": 15000}`。
- 命令行参数会覆盖配置：`--exposure`、`--gain`、`--fps`、`--pixel-format`、
  `--width/--height`、`--serial`（可重复，顺序即相机顺序）。

## 7. 同步与 USB

### 7.1 同步

- 默认自由运行（`trigger_mode=Off`），四路帧的采集时刻由各自曝光决定，
  帧集 `skew_ms` 只反映主机收到帧的时间差，不适合作为硬同步指标。
- `--software-trigger` 会把四台相机设为 `TriggerMode=On / TriggerSource=Software`，
  再依次发送 `TriggerSoftware`。实测 4 路时间戳离散约 3~4 ms，适合静态标定。
- 运动场景/高精度多视图：使用**硬件触发**。相机 Line2/Line5 支持双向 IO；
  可由主相机或外部信号源输出曝光开始脉冲，接到其余相机的 Line2，
  配置 `trigger_mode=On, trigger_source=Line2`。本项目已支持这些参数，
  但未在本机验证外部接线。

### 7.2 USB

本机 4 台相机分布在两个 USB3 控制器上：

- Bus 002：1 台（5 Gbps）
- Bus 004：3 台（1 台直连 + 2 台经 5 Gbps HUB）

1440×1080×1 B×165 fps ≈ **256.6 MB/s ≈ 2.05 Gbps/相机**。
Bus 004 上 3 台同时满帧约 6.2 Gbps，已经超过单条 5 Gbps 链路；
本机能跑通是因为控制器/端口分配和 SDK 缓冲共同作用，但这不是可移植的余量。

如果只能同时读到 2 台，按顺序检查：

1. `cat /sys/module/usbcore/parameters/usbfs_memory_mb`，应 ≥ 1000（本机 2000）；
2. udev 规则 `/etc/udev/rules.d/80-drivers-SDK-2bdf.rules` 是否存在；
3. 是否把 3 台相机挂在同一个 HUB/同一条 5 Gbps 链路上；
4. 降低 `--fps` 或使用 2×2 binning / 更小 ROI；
5. 检查供电，优先使用带外部供电的 USB3 HUB。

详细命令与持久化配置见 [docs/usb-tuning.md](docs/usb-tuning.md)。

## 8. 颜色与像素格式

- 设备端默认 **BayerRG8**：1 B/px，带宽最小，165 fps（ADC12）。
- 主机端用 MVS SDK 做 Bayer→BGR8 转换；SDK 不可用时回退到 OpenCV。
- **MVS 的 BayerRG8 是 RGGB，对应 OpenCV `COLOR_BayerBG2BGR`**（OpenCV 命名相反）。
  该映射有单元测试锁定。
- 标定/拼接时四路必须使用相同的曝光、增益、白平衡和 gamma 设置。
  默认关闭 `ExposureAuto/GainAuto/BalanceWhiteAuto`，避免帧间颜色漂移。
- 需要 249 fps 时把 `adc_bit_depth` 设为 `Bits_8`；代价是动态范围下降。
- 直接让相机输出 BGR8 会把 USB 负载提高到 3 倍，不建议在 4 路满帧时使用。

详见 [docs/color-and-format.md](docs/color-and-format.md)。

## 9. 与 quad-camera 的兼容性

本工程写出的标定产物与 `quad-camera` 的读取代码一致：

- `intrinsics.yml`：`model/K/D/image_width/image_height/pattern_cols/pattern_rows/square_mm/
  circle_center/circle_radius/fold_radius/rms/mean_error`
- `extrinsics.yml`：`R_cw/T_cw/center_angle_deg`
- `lut.bin`：magic `PLUT`，24 B 头，float32 mapx 后 mapy
- `weight.bin`：magic `PWGT`，24 B 头，float32
- 坐标约定：世界系 = 相机 0；`X_world = R_cw · X_cam + T_cw`；
  `T_cw` 为相机光心；`+Y` 向下，地面在 `Y=+H`。

可直接把 `data/calib` 复制到 `quad-camera/data/calib`，运行：

```bash
cd ../quad-camera
./build/src/panostitch --frames <你的四张图目录> --blend multiband --seam
```

更完整的字段说明见 [docs/quad-camera-compat.md](docs/quad-camera-compat.md)。

## 10. 实测记录

| 项目 | 结果 |
|---|---|
| 物理相机枚举 | 4 台，另过滤 1 台虚拟相机 |
| 4 路 165 fps / BayerRG8 / 60 帧 | 4 路全部成功，错误 0 |
| 4 路 30 fps 软件触发 | 帧集时间戳 skew 约 3.2~3.6 ms |
| 单帧 4 路拼接（2048×512，feather） | 约 68~73 ms（本机） |
| 用 quad-camera 标定数据 + 本工程 LUT 拼接 | 输出与 quad-camera 结果肉眼一致 |
| 单元测试 | 14/14 通过 |

## 11. 开发规范

- 提交信息使用 Conventional Commits（`feat(driver): ...`、`fix(calib): ...`、`docs: ...`）。
- 分支：`main` 保护；功能分支 `feat/<topic>`；修复分支 `fix/<topic>`。
- 格式化：`clang-format`（见 `.clang-format`）；静态检查：`.clang-tidy`。
- 不提交 `build/`、`data/captures/`、`data/calib/`、`output/` 和个人配置。
- 改动驱动/标定算法必须补单元测试；改动硬件相关路径需在真实 4 路相机上验证。

详见 [CONTRIBUTING.md](CONTRIBUTING.md) 和 [docs/architecture.md](docs/architecture.md)。

## 12. 已知限制

- 标定需要实际的棋盘格图像；仓库不附带针对本机相机的标定数据。
- 外参使用相邻相机共享棋盘视角构建生成树；若某对相机完全没有共同视野，
  该相机会回退到 90° 名义 yaw，并在 `report.json` 中给出 warning。
- LUT 默认按纯旋转（无穷远）生成；需要地面/焦点视差补偿时，
  可把 `lut.bin` 交给 `quad-camera --parallax inmem --ground-height ...`。
- 未在本仓库内实现多频段融合/找缝；需要更高质量时请直接用 `quad-camera`。
- 许可证未指定；发布前请由仓库所有者补充 `LICENSE`。
