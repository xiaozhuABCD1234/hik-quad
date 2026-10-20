# 颜色、像素格式与带宽

## 1. 默认链路

```
IMX273 (Bayer RGGB, ADC 12 bit)
  → 相机内部 PixelFormat = BayerRG8
  → USB3 Vision 传输 1 B/px
  → 主机 MV_CC_ConvertPixelTypeEx(BayerRG8 → BGR8)
  → cv::Mat CV_8UC3（OpenCV 约定 BGR）
  → 标定/拼接/保存
```

为什么不在相机端直接出 BGR8：

- BayerRG8 每像素 1 B；BGR8 每像素 3 B，USB 负载 ×3；
- 4 路 1440×1080×165 fps 时，BGR8 每路约 6.2 Gbps，完全不现实；
- 主机端转换可以利用 CPU，且 MVS 的转换是 SIMD 优化的。

## 2. Bayer 命名（重要）

MVS 的 `BayerRG8` 表示：

```
R G R G
G B G B
R G R G
```

即 **RGGB**。OpenCV 的常量命名与传感器 pattern 相反：

| MVS 格式 | pattern | OpenCV 回退代码 |
|---|---|---|
| `BayerRG8` | RGGB | `cv::COLOR_BayerBG2BGR` |
| `BayerBG8` | BGGR | `cv::COLOR_BayerRG2BGR` |
| `BayerGB8` | GBRG | `cv::COLOR_BayerGR2BGR` |
| `BayerGR8` | GRBG | `cv::COLOR_BayerGB2BGR` |

主路径使用 MVS SDK 转换，不依赖这张表；只有当 SDK 转换失败时才回退到
OpenCV。单元测试 `pixel_format_bayer_mapping` 锁定了这张表，防止“红蓝互换”。

> 如果发现画面红蓝互换，先检查相机 `PixelFormat` 是否真的是 BayerRG8，
> 以及是否有人手工把 BGR 当成 RGB 保存。

## 3. ADC 位深与帧率

| `adc_bit_depth` | 最大帧率（1440×1080 BayerRG8） | 说明 |
|---|---:|---|
| `Bits_12` | ~165 fps | 默认，动态范围好 |
| `Bits_8` | ~249 fps | 带宽/动态范围下降 |

相机节点 `ADCBitDepth` 支持 `Bits_8` / `Bits_12`。修改后
`ResultingFrameRate` 会变化；采集程序会打印实际值。

## 4. 曝光、增益、白平衡、Gamma

- `ExposureAuto=Off`、`GainAuto=Off`：避免帧间亮度漂移。
- `BalanceWhiteAuto=Off`：标定/拼接必须固定白平衡；需要时可写
  `balance_ratio_red/green/blue`（>0 才写入）。
- `GammaEnable`：本机型该节点不可访问（`0x80000106`），程序只告警。
  如需 gamma，请在主机端对 BGR 图统一处理。
- 四路相机建议使用相同曝光/增益；若场景亮度差异大，可在主机端做光度补偿
  （quad-camera 的 `gain.npy` 机制），但本工程暂未生成 gain。

## 5. 像素格式能力（MV-CB016-10UC-S）

```
Mono8 / Mono10 / Mono12
BayerRG8 / BayerRG10 / BayerRG10Packed / BayerRG12 / BayerRG12Packed
RGB8Packed / BGR8Packed
YUV422Packed / YUV422_YUYV_Packed
```

`hik_quad_capture --pixel-format BGR8Packed` 可切到相机端转换，但只建议在
低帧率/单路调试时使用。

## 6. 带宽估算

```
bytes/s = width × height × bytes_per_pixel × fps
Gbps    = bytes/s × 8 / 1e9
```

BayerRG8 1440×1080：

| fps | MB/s | Gbps |
|---:|---:|---:|
| 30 | 46.7 | 0.37 |
| 60 | 93.3 | 0.75 |
| 100 | 155.5 | 1.24 |
| 165 | 256.6 | 2.05 |

BGR8 同分辨率同帧率约为上表的 3 倍。
