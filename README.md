# hik-quad

4 路 HIKROBOT USB3 相机的标定与视频拼接。纯 CMake，依赖 OpenCV 4 和 HIK MVS SDK。

## 构建

```bash
sudo apt install build-essential cmake libopencv-dev
# MVS SDK 默认安装到 /opt/MVS
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

生成三个程序：`build/hik_quad_capture`、`build/hik_quad_calib`、`build/hik_quad_stitch`。

## 标定（只需一次）

准备 **8×11 内角点、100 mm 方格**的棋盘格。四路相机按 `config/default.json` 中
`cameras` 的顺序（顺时针）固定好序列号，然后同时拍摄约 20 个视角：

```bash
# 每 1 秒拍一组，移动棋盘后再拍下一组
./build/hik_quad_capture --frames 20 --interval 1 --out data/calib_images

./build/hik_quad_calib --input data/calib_images --output data/calib
```

标定结果在 `data/calib/camera_0..3/`，每个相机包含 `intrinsics.yml`、
`extrinsics.yml`、`lut.bin`、`weight.bin`。

## 启动拼接

### 视频拼接（4 路视频 → 1 路全景视频）

```bash
# 1) 录制四路同步视频，输出 cam0.mp4 ... cam3.mp4
./build/hik_quad_capture --video --frames 200 --fps 20 --out data/videos

# 2) 启动拼接
./build/hik_quad_stitch --calib data/calib --input data/videos --out output/panorama.mp4
```

### 单帧拼接（4 张图片 → 1 张全景图）

```bash
./build/hik_quad_stitch --calib data/calib --input data/frames --out output/panorama.jpg
```

### 输入输出规则

- 输入目录中每路相机一个文件，命名为 `cam0.*`、`cam1.*`、`cam2.*`、`cam3.*`；
  也接受 `camera_0.*`、`cam_0.*` 和 `0.*`。
- 图片支持 png/jpg/bmp/tif，视频支持 mp4/avi/mkv/mov。
- 输出类型由 `--out` 后缀决定：`.jpg`/`.png` 处理单帧，`.mp4`/`.avi` 逐帧处理整段视频。
- 四路输入的顺序必须与标定时的相机顺序一致；需要覆盖视频帧率时加 `--fps 20`。
- 配置文件和命令行中的相对路径都相对当前工作目录解析。

常用参数见 `./build/hik_quad_stitch --help`。

## 代码结构

| 路径 | 内容 |
| --- | --- |
| `include/hikquad/` | 公共头文件：相机、标定、LUT、拼接、配置、日志、媒体文件命名 |
| `src/camera*.cpp` | MVS SDK 封装：设备枚举、参数下发、同步抓帧 |
| `src/calibration*.cpp`、`src/lut.cpp` | 棋盘格标定、外参估计、柱面 LUT 与权重生成 |
| `src/stitcher.cpp` | 按 LUT 重映射并做加权融合 |
| `src/config.cpp`、`src/cli.cpp` | JSON 配置与命令行解析 |
| `apps/` | 三个可执行程序及其共享入口工具 |
