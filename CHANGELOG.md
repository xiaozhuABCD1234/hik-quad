# Changelog

All notable changes to this project are documented here.
The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/).

## [0.2.0] - 2026-10-20

### Added

- USB3 Vision transport tuning: `MV_USB_SetTransferWays`,
  `MV_USB_SetTransferSize`, configurable node count and retry count.
- Frame integrity validation in `Camera::grab()`: checks `nFrameLen` against
  the expected payload, rejects `lost_packets > 0`, retries incomplete frames
  and reports an actionable error instead of returning a half image.
- Explicit camera order/role (`cam1..cam4`) in config; frame/output indices now
  follow the configured clockwise order rather than serial-number enumeration.
- `--horizontal` for `hik_quad_calib` / `hik_quad_lut`: yaw-only extrinsics
  and camera centres on the world XZ plane for a mechanically horizontal 360° ring.
- `--order` for `hik_quad_stitch`: explicit LUT camera -> source image mapping.
- `scripts/install_usb_tuning.sh` for `usbfs_memory_mb=4000` and USB autosuspend.
- Frame triage fields in capture metadata: `transport_bytes`, `expected_bytes`,
  `lost_packets`, `complete`.

### Fixed

- The two cameras behind the 5 Gbps HUB returned truncated frames at 165 fps;
  the driver now detects this. Current topology is stable at 20 fps.
- Output files `cam0..cam3` now match the configured physical order
  `cam1=DB1505189, cam2=DB1505182, cam3=DB1505181, cam4=DB1505196`.

### Changed

- Default capture is 20 fps, 15 ms exposure, 10 dB gain for the current
  Bus 004 HUB topology.
- Software trigger is marked as unreliable on the shared HUB; calibration uses
  free-run at 20 fps.

## [0.1.1] - 2026-10-20

### Fixed

- `discoverViews()` now only treats `view_*/shot_*` directories as calibration
  views, so a sibling `frames/` directory no longer shifts view ids or gets
  mistaken for a checkerboard view.
- Installed binaries locate `share/hik-quad/config/default.json` from the
  executable prefix.
- `hik_quad_lut` keeps rebuilt output directories self-contained by copying
  `intrinsics.yml` / `extrinsics.yml`; `hik_quad_stitch` validates frame
  resolution against the calibration.

### Added

- `synthetic_four_camera_pipeline` test: renders a 360-degree texture through
  four virtual fisheye cameras, builds the four LUTs and verifies the stitched
  panorama against the ground truth (PSNR > 22 dB).

## [0.1.0] - 2026-10-20

### Added

- Pure CMake/C++17 project with MVS SDK discovery, RUNPATH setup and install rules.
- USB3 Vision camera discovery with virtual-camera filtering and sysfs link details.
- RAII `Camera` driver: BayerRG8/BGR8/RGB8/Mono8, ROI, binning/decimation,
  exposure, gain, white balance, trigger, frame-rate and stream tuning.
- `CameraArray`: parallel 4-camera capture, software trigger, frame-set timestamps
  and skew reporting.
- `hik_quad_capture`: single/multi frame-set capture with PNG/JPG/BMP and raw output.
- `hik_quad_calib`: chessboard detection, fisheye/pinhole intrinsics, per-view poses,
  pairwise relative pose graph, quaternion rotation averaging and quad-camera
  compatible YAML/JSON/LUT/weight output.
- `hik_quad_lut`: rebuild cylindrical LUTs from existing intrinsics/extrinsics.
- `hik_quad_stitch`: feather-weighted 4-camera panorama.
- Unit tests for pixel formats, Bayer mapping, config parsing, calibration I/O,
  LUT binary format, rotation/extrinsics math and synthetic intrinsics calibration.
- USB diagnostics, interactive calibration capture, build and format scripts.
- Chinese README and docs: architecture, USB tuning, calibration, color pipeline,
  quad-camera compatibility, troubleshooting.
