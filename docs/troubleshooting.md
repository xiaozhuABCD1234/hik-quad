# 常见问题

## 1. 编译/运行

### `MvCameraControl.h was not found`

MVS SDK 未安装或路径不对：

```bash
cmake --preset release -DHIK_SDK_ROOT=/opt/MVS
```

### 运行时报 `MV_E_LOAD_LIBRARY`

SDK 的 `libMvUsb3vTL.so` / `libFormatConversion.so` 等运行时库找不到。
本工程已把 `/opt/MVS/lib/64` 写入 RUNPATH；若仍失败：

```bash
source /opt/MVS/bin/set_env_path.sh
ldd build/release/apps/hik_quad_list | grep Mv
```

### `MV_E_GC_ACCESS (0x80000106)`

某个 GenICam 节点不可访问。本机型常见：

- `GammaEnable`
- `TriggerActivation`
- `DeviceLinkThroughputLimit`

程序对这些**可选**节点只告警，不影响取流。若要区分“参数没写进去”，
加 `--verbose` 看 debug 日志。

## 2. 只能看到 2 台 / StartGrabbing 失败

按顺序：

1. `./scripts/check_usb.sh`
2. `cat /sys/module/usbcore/parameters/usbfs_memory_mb`，建议 2000
3. 是否 3 台挂在同一 HUB；拆到不同控制器
4. 降低 `--fps 20`
5. `dmesg -w` 看 USB reset / over-current
6. 确认没有 MVS 客户端或其他进程独占相机

## 3. 图像红蓝互换

- 确认相机 `PixelFormat` 是 `BayerRG8`；
- 如果不是，改成 `BayerRG8` 再试；
- 保存时使用 OpenCV `imwrite`（内部 BGR），不要手工把 BGR 当 RGB；
- 参考 [color-and-format.md](color-and-format.md) 的 Bayer 映射表。

## 4. 图像太暗/太亮

```bash
./build/release/apps/hik_quad_capture --frames 1 --exposure 20000 --gain 10 --out /tmp/x
```

- 曝光范围：正常模式 15 µs ~ 10 s；超短曝光 1~14 µs。
- 增益 0~17 dB。
- 四路要一致；不要只调其中一路，否则拼接会有亮度缝。

## 5. 标定检测不到棋盘

- `--check` 看每路检测数；
- 棋盘必须完整、平整、无遮挡；
- 内角点数是否写对（8×11，不是 9×12）；
- 黑白对比是否足够，是否过曝；
- 鱼眼边缘畸变很大时，棋盘不要贴边到完全出画；
- 用 `--debug-dir /tmp/calib_debug` 保存角点叠加图检查顺序。

## 6. RMS 很大 / 外参跳变

- 增加视角，覆盖画面四角和边缘；
- 棋盘倾斜不要超过 ±30°；
- 确保采集时相机/棋盘静止；
- 检查是否有某一路图像模糊、反光、曝光不一致；
- 看 `report.json` 的 `edges[].translation_spread_m`；
  某条边离散很大时，剔除该视角或重采。

## 7. 拼接有重影/错位

- 标定数据是否是当前相机/支架的？移动后必须重标；
- 外参是否回退到名义 yaw（看 `report.json.warnings`）；
- 近景视差：用 `quad-camera --parallax inmem --ground-height ...`；
- 接缝处颜色不一致：固定白平衡/曝光，或做光度补偿；
- LUT 尺寸/FOV 改过但 weight 没重建：用 `hik_quad_lut` 重建。

## 8. 4 路时间不同步

- 自由运行时 `skew_ms` 只是主机接收时间差；
- 静态标定可用 `--software-trigger`（约 3~4 ms）；
- 运动场景用硬件触发：主相机/外部信号 → 其余相机 Line2，
  `trigger_mode=On, trigger_source=Line2`；
- 确认所有相机曝光时间一致，触发线屏蔽/接地良好。
