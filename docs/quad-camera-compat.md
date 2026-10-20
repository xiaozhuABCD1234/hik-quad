# quad-camera 兼容格式

本工程的目标之一是让标定产物可以直接被
[`quad-camera`](https://github.com/xiaozhuABCD1234/quad-camera) 的
`panostitch` 读取。字段与坐标约定严格对齐。

## 1. 坐标系

- 世界系 = 相机 0 系；OpenCV 相机约定（+X 右、+Y 下、+Z 前）。
- `X_world = R_cw · X_cam + T_cw`。
- `T_cw` 是相机光心在世界系中的位置。
- 相机光轴方向：`R_cw · (0,0,1)`；
  中心方位角：`atan2(axis.x, axis.z)`，范围 [0, 360)。

这与 `quad-camera` 的 `src/main.cpp`、`src/parallax.cpp` 一致。

## 2. intrinsics.yml

```yaml
%YAML:1.0
---
model: fisheye
K: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ fx, 0., cx, 0., fy, cy, 0., 0., 1. ]
D: !!opencv-matrix
   rows: 1
   cols: 4
   dt: d
   data: [ k1, k2, k3, k4 ]
image_width: 1440
image_height: 1080
pattern_cols: 8
pattern_rows: 11
square_mm: 100.
circle_center: !!opencv-matrix
   rows: 1
   cols: 2
   dt: d
   data: [ cx, cy ]
circle_radius: 602.
fold_radius: 1.357
rms: 0.42
mean_error: 0.39
```

- `model` 可以是 `fisheye` 或 `pinhole`。
- 鱼眼只取 D 的前 4 个系数。
- `fold_radius` 是 **θ_d** 上限（不是 θ），用于屏蔽像圈外像素。

## 3. extrinsics.yml

```yaml
%YAML:1.0
---
R_cw: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ ... ]
T_cw: !!opencv-matrix
   rows: 3
   cols: 1
   dt: d
   data: [ x, y, z ]
center_angle_deg: 90.
```

## 4. lut.bin

24 字节头，native little-endian：

```c
struct BinHeader {
  char     magic[4];   // "PLUT"
  uint16_t version;    // 1
  uint16_t dtype;      // 0 = float32
  int32_t  width;
  int32_t  height;
  int32_t  camera;
  int32_t  reserved;
};
```

负载：先 `width*height` 个 float32 的 `mapx`，再同样数量的 `mapy`。
无效像素为 `-1`。

## 5. weight.bin

同样的 24 字节头，magic `"PWGT"`，负载为 `width*height` 个 float32。
每个相机一份，值域 0~1，不跨相机归一化；拼接时按权重和归一化。

## 6. calib_meta.json

```json
{
  "version": 1,
  "created": "2026-10-20T16:00:00",
  "count": 4,
  "out_size": [2048, 512],
  "fov_h_deg": 360.0,
  "lut_dtype": "float32",
  "board": {"inner_cols": 8, "inner_rows": 11, "square_mm": 100.0},
  "cameras": [
    {"index": 0, "intrinsics_rms": 0.42, "has_lut": 1, "has_weight": 1}
  ]
}
```

`quad-camera` 不读取该文件，仅用于记录/复现。

## 7. 直接使用

```bash
cp -a data/calib/. ../quad-camera/data/calib/
cd ../quad-camera
./build/src/panostitch --frames /path/to/four/frames --blend multiband --seam
```

或者用本工程的 `hik_quad_stitch`（feather）：

```bash
./build/release/apps/hik_quad_stitch \
  --calib data/calib --frames /path/to/four/frames --out output/pano.jpg
```

## 8. 已知差异

- 本工程暂不生成 `gain.npy`；`quad-camera` 读取失败时会自动跳过。
- 本工程 LUT 默认纯旋转；`quad-camera --parallax inmem` 可覆盖视差。
- 本工程只做 feather 融合；多频段/找缝请用 `quad-camera`。
- `hik_quad_calib` 的 `has_lut/has_weight` 写为 1/0（quad-camera 不依赖它）。
