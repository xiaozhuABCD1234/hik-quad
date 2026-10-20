# USB3 带宽与系统调优

## 1. 先看拓扑

```bash
lsusb -t
for d in /sys/bus/usb/devices/*; do
  [ -f "$d/idVendor" ] || continue
  [ "$(cat "$d/idVendor")" = "2bdf" ] || continue
  echo "$d bus=$(cat "$d/busnum") dev=$(cat "$d/devnum") speed=$(cat "$d/speed") serial=$(cat "$d/serial")"
done
```

本机实测：

```
Bus 002: 1 × MV-CB016-10UC-S (5000M)
Bus 004: 3 × MV-CB016-10UC-S
          ├─ 4-1    直连 root port
          ├─ 4-2.1  经 HUB
          └─ 4-2.2  经 HUB
```

## 2. 带宽预算

BayerRG8、1440×1080、165 fps：

```
1440 × 1080 × 1 B × 165 ≈ 256.6 MB/s ≈ 2.05 Gbps / 相机
```

| 每路帧率 | 单相机 | 2 台 | 3 台 | 4 台 |
|---:|---:|---:|---:|---:|
| 165 fps | 2.05 Gbps | 4.10 Gbps | 6.16 Gbps | 8.21 Gbps |
| 100 fps | 1.24 Gbps | 2.49 Gbps | 3.73 Gbps | 4.97 Gbps |
| 60 fps | 0.75 Gbps | 1.49 Gbps | 2.24 Gbps | 2.98 Gbps |
| 30 fps | 0.37 Gbps | 0.75 Gbps | 1.12 Gbps | 1.49 Gbps |

一条 USB3 Gen1 链路理论 5 Gbps，实际有效载荷通常 3.2~4.0 Gbps。
**同一 HUB/同一 root port 下挂 3 台满帧一定超预算**。

本机实测（1440×1080 BayerRG8，200~300 帧/路）：

| 配置 | 直连 2 路 | HUB 下 2 路 |
|---|---|---|
| 20 fps free-run | 完整 | 完整 |
| 30 fps free-run | 完整 | 间歇截断（DB1505196） |
| 60 fps free-run | 完整 | 截断 |
| 100/120/165 fps free-run | 完整 | 截断 |
| 20~60 fps 软件触发 | 完整 | 截断/超时 |

结论：**当前拓扑下 20 fps 是已验证的安全值**；30 fps 已经处于 HUB 的临界点。
需要 30 fps 以上时，必须把 HUB 下的相机（尤其 DB1505196）分散到不同 USB
控制器，或使用更高带宽的采集卡，而不是继续调 SDK 参数。

本工程 `CameraConfig` 的 `frame_rate` 会写 `AcquisitionFrameRate`
（并打开 `AcquisitionFrameRateEnable`），这是本机型实际可用的限速手段。

## 3. usbfs 内存

Linux 的 `usbcore` 默认 `usbfs_memory_mb=16`，4 路 1440×1080 很容易耗尽，
表现为“只能开 2 台”“StartGrabbing 失败”“取流一段时间后超时”。

查看：

```bash
cat /sys/module/usbcore/parameters/usbfs_memory_mb
```

本机为 **2000 MB**。临时提高（root）：

```bash
echo 4000 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```

持久化 + 关闭 USB autosuspend，直接运行：

```bash
sudo ./scripts/install_usb_tuning.sh
```

脚本会写 `/etc/modprobe.d/99-hik-quad-usb.conf`（`usbfs_memory_mb=4000`）
和 `/etc/udev/rules.d/99-hik-quad-usb.rules`（`2bdf` 设备 `power/control=on`）。

建议值：

- 2 台 1440×1080：≥ 512 MB
- 4 台 1440×1080：≥ 1024 MB，推荐 2000~4000 MB
- 更高分辨率/更高帧率：按 `width × height × bytes × 队列深度 × 相机数` 估算

## 4. udev 权限

MVS 安装包通常已安装：

```bash
ls -l /etc/udev/rules.d/80-drivers-SDK-2bdf.rules
```

没有时重新安装 MVS，或确认当前用户对 `/dev/bus/usb/*` 有读写权限。
不要把 `chmod 777` 当成长期方案。

## 5. MVS 侧参数

本机型（MV-CB016-10UC-S）实测：

| 节点/接口 | 是否存在 | 说明 |
|---|---|---|
| `DeviceLinkThroughputLimitMode` | 是 | On/Off |
| `DeviceLinkThroughputLimit` | **否**（0x80000106） | 不能按字节限速 |
| GenICam `TransferSize` | **否**（0x80000109） | 不是 FeatureTree 节点 |
| `MV_USB_SetTransferSize` | 是（SDK 接口） | 默认 1 MiB，Linux 范围 32 KiB~2 MiB |
| `MV_USB_SetTransferWays` | 是（SDK 接口） | 本机型上限 **8**，16/32 返回 0x80000025 |
| `AcquisitionFrameRate` | 是 | 实际限速入口 |
| `MV_CC_SetImageNodeNum` | 是（SDK 接口） | 建议 8 |

`config/default.json` 默认：

```json
"usb": {
  "image_node_num": 8,
  "usb_transfer_ways": 8,
  "usb_transfer_size": 1048576,
  "incomplete_frame_retries": 3,
  "throughput_limit_mode": false,
  "grab_timeout_ms": 3000,
  "grab_strategy": "OldestFirst"
}
```

注意：`MV_USB_SetTransferWays`/`SetTransferSize` **不能把 3 台 165 fps 的
HUB 链路变成可用**；它们只影响 URB 调度。实测 16/32 ways 直接失败，
2 MiB transfer size 也没有改善截断。真正的限制是 5 Gbps 上行带宽和帧率。

## 6. HUB 与供电

- 使用**带外部供电**的 USB3 HUB；相机典型 1.9 W@5 V，4 台同时启动有浪涌。
- 避免把 3 台相机挂在同一个无源 HUB 上。
- 优先把相机分散到主板不同 USB 控制器（本机 Bus 002 / Bus 004）。
- 线缆：USB3 认证线，尽量 ≤ 2 m；M12 板级相机的线序/屏蔽要可靠。
- 不要用 USB2 HUB/延长线；链路速率应显示 5000M。

## 7. 快速自检

```bash
sudo ./scripts/install_usb_tuning.sh
./scripts/check_usb.sh
./build/release/apps/hik_quad_list
./build/release/apps/hik_quad_capture --frames 1 --fps 20 --no-save
```

检查 `meta.json`：

```bash
grep -E 'transport_bytes|expected_bytes|lost_packets|complete' meta.json
```

正常应为 `1555200 / 1555200 / 0 / 1`。若出现 `complete: 0` 或驱动报
`incomplete frame ... after N attempts`：

1. 把 `--fps` 降到 30，确认四路完整；
2. 换端口/换线，确认 `lsusb` 和 sysfs 都能看到 4 台、5000M；
3. 提高 `usbfs_memory_mb` 并关闭 autosuspend；
4. 检查是否有 MVS 客户端、ROS2 节点或其他进程占用了相机（独占访问）；
5. 把 HUB 下的相机分散到不同控制器，或换带外部供电的 HUB；
6. 用 `dmesg -w` 观察 USB reset/over-current。
