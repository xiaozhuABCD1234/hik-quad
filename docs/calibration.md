# 标定流程与算法

## 1. 标定板

默认参数（与 quad-camera 示例一致）：

- 内角点：**8 × 11**（列 × 行）
- 方格边长：**100 mm**
- 需要白色外边，棋盘区域平整、无反光
- 推荐打印在 A2/A3 硬板上，或使用陶瓷/铝板标定板

`config/default.json`：

```json
"calibration": {
  "pattern_cols": 8,
  "pattern_rows": 11,
  "square_mm": 100.0,
  "model": "fisheye",
  "min_views": 10,
  "max_rms": 1.0,
  "emit_lut": true
}
```

> 注意：`pattern_cols/rows` 是**内角点**数量，不是方格数。
> 8×11 内角点对应 9×12 个方格。

## 2. 采集

### 2.1 交互式脚本

```bash
./scripts/capture_calibration.sh 20
```

脚本对每个视角：

1. 提示移动棋盘；
2. 调用 `hik_quad_capture --frames 1 --software-trigger`；
3. 保存到 `data/calib_images/view_000/cam0.png ... cam3.png`；
4. 调用 `hik_quad_calib --check` 打印检测情况。

### 2.2 手动采集

```bash
./build/release/apps/hik_quad_capture \
  --frames 1 --software-trigger \
  --exposure 15000 --gain 0 \
  --out data/calib_images/view_000
```

重复并递增 `view_000`。建议 15~25 个视角。

### 2.3 采集要点

- 棋盘覆盖画面中心 + 四角 + 边缘；鱼眼镜头边缘畸变大，必须采到。
- 每个视角至少 2 台相机同时看到棋盘，才能建立外参边；
  相邻相机要有足够重叠。
- 棋盘倾斜 ±30° 以内，避免完全正对导致退化。
- 相机和棋盘保持静止；用软件触发时四路时间差约 3~4 ms。
- 四路曝光/增益/白平衡固定，关闭自动曝光/自动增益/自动白平衡。
- 避免强反光、运动模糊、过曝（黑白格都要有细节）。

## 3. 检测与内参

`calibrateCamera()`：

1. `cv::findChessboardCornersSB(..., EXHAUSTIVE | ACCURACY)`；
2. 失败时 `findChessboardCorners` + `cornerSubPix`；
3. 鱼眼模型：`cv::fisheye::calibrate`，
   flags = `CALIB_RECOMPUTE_EXTRINSIC | CALIB_FIX_SKEW`；
4. 针孔模型：`cv::calibrateCamera`；
5. 逐视图 `projectPoints` 计算重投影误差，写入 `intrinsics.yml` 的
   `rms` / `mean_error`。

质量建议：

| 指标 | 良好 | 可接受 | 需要重采 |
|---|---:|---:|---:|
| RMS (px) | < 0.5 | 0.5~1.0 | > 1.0 |
| 单相机可用视角 | ≥ 15 | 10~14 | < 10 |
| 边缘视角覆盖 | 四角都有 | 大部分 | 只拍中心 |

`--max-rms` 只告警，不强制失败；是否接受由使用者判断。

## 4. 外参

对每一对相机 `(a,b)`：

1. 找共同 `view_index`；
2. 每个共同视图有 `T_a_board`、`T_b_board`（board→camera）；
3. 计算 `X_a = R_ab X_b + t_ab`：
   `R_ab = R_a R_b^T`，`t_ab = t_a - R_ab t_b`；
4. 旋转用四元数对齐后平均（避免 180° 旋转的 rotvec 符号歧义），
   平移逐轴取中值，记录最大离散 `translation_spread_m`；
5. 共同视图 ≥ 2 才建边；
6. 以相机 0 为根 BFS：
   - 已知 a、未知 b：`R_b0 = R_ab^T R_a0`，
     `t_b0 = R_ab^T (t_a0 - t_ab)`；
   - 已知 b、未知 a：`R_a0 = R_ab R_b0`，
     `t_a0 = R_ab t_b0 + t_ab`；
7. 转成 quad-camera 约定：
   `R_cw = R_i0^T`，`T_cw = -R_i0^T t_i0`（即相机光心）。

没有连通的相机会回退到名义 yaw（`--nominal-step`，默认 90°），
并在 `report.json` 中记录 warning。此时 `T_cw=0`，只适合纯旋转预览，
不能用于有视差的拼接。

## 5. 图像圆与 fold

代码默认：

- `circle_center = (cx, cy)`；
- `circle_radius = min(cx, cy, W-cx, H-cy)`；
- `fold_radius = circle_radius / f`，`f=(fx+fy)/2`。

这三个量可以在 `intrinsics.yml` 中手工覆盖。对 M12 鱼眼镜头，
如果实际像圈更小，建议用实际像圈半径，避免 LUT 采到黑边。

## 6. 圆柱 LUT

对输出像素 `(u,v)`：

```
θ = (u + 0.5 - W/2) / f,   f = W / fov
d_world = normalize(sinθ, (v + 0.5 - H/2)/f, cosθ)
X_cam = R_cw^T (d_world - T_cw)      # 纯旋转时忽略 T_cw
```

鱼眼投影：

```
r = hypot(X, Y); θ = atan2(r, Z)
θ_d = θ (1 + k1θ² + k2θ⁴ + k3θ⁶ + k4θ⁸)
u_src = fx θ_d X/r + cx
v_src = fy θ_d Y/r + cy
```

有效性：

- 源像素在图像范围内；
- 在 `circle_radius` 内；
- `θ_d ≤ fold_radius`。

权重：

```
w = exp(-0.5 (angle(d_world, camera_axis) / sigma)^2)
```

`sigma` 由 `--sigma`（默认 35°）控制。权重不跨相机归一化，
`weight.bin` 保存单路权重；拼接时按 `Σ w_i I_i / Σ w_i` 归一化。

## 7. 产物

```
camera_i/intrinsics.yml
camera_i/extrinsics.yml
camera_i/lut.bin
camera_i/weight.bin
calib_meta.json
report.json
```

`report.json` 包含每路 K/D、RMS、中心角、T_cw，以及每条边的
共同视图数、baseline、平移离散；调参时优先看它。

## 8. 复标定

- 只换 LUT 尺寸/FOV：`hik_quad_lut --calib data/calib --pano ... --fov ...`
- 只重算外参：重新采集含共同视角的数据，重跑 `hik_quad_calib`。
- 相机/镜头/支架移动后必须重标；仅改曝光/增益不需要重标内参，
  但建议保持与拼接时一致的光度设置。
