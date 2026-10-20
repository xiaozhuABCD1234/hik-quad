# 贡献指南

## 1. 分支模型

- `main`：可发布分支，保持可编译、测试通过。
- `feat/<topic>`：新功能。
- `fix/<topic>`：缺陷修复。
- `docs/<topic>`：仅文档。
- `chore/<topic>`：构建、工具、依赖。

合并前请 rebase 到最新 `main`，保持线性历史。

## 2. 提交信息

使用 [Conventional Commits](https://www.conventionalcommits.org/)：

```
<type>(<scope>): <subject>

<body>

<footer>
```

常见类型：

| type | 用途 |
|---|---|
| `feat` | 新功能 |
| `fix` | 修复缺陷 |
| `docs` | 文档 |
| `test` | 测试 |
| `refactor` | 不改变行为的重构 |
| `perf` | 性能优化 |
| `build` | CMake/依赖/打包 |
| `ci` | CI |
| `chore` | 杂项 |

示例：

```
feat(driver): add DeviceLinkThroughputLimit fallback for USB3 cameras
fix(calib): average rotations with sign-aligned quaternions
docs(usb): document usbfs_memory_mb persistence
```

## 3. 代码规范

- C++17；4 空格缩进；行宽 100；`clang-format`（`.clang-format`）。
- 公共头文件放 `include/hikquad/`，实现放 `src/`；不要在下游包含 `src/` 头。
- 错误用 `SdkError` / `std::runtime_error` 表达，不要静默吞掉 MVS 错误。
- 日志用 `HIKQUAD_LOG(Info|Warn|Debug)`，终端输出保持英文/ASCII。
- 注释解释“为什么”，不要复述代码；坐标约定必须写清楚。
- 不引入 ROS2；如需参考 ROS2 实现，只借鉴逻辑，不复制依赖。

## 4. 测试

```bash
cmake --build --preset release
ctest --preset release --output-on-failure
```

- 改动像素格式/Bayer 映射：必须更新 `tests/test_pixel_format.cpp`。
- 改动标定数学：必须更新 `tests/test_calibration_math.cpp`。
- 改动 LUT 二进制格式：必须保持与 `quad-camera` 的 24 字节头兼容，
  并更新 `tests/test_calibration_io.cpp`。
- 改动驱动/取流：在 4 台真实相机上跑：

  ```bash
  ./build/release/apps/hik_quad_list
  ./build/release/apps/hik_quad_capture --frames 3 --software-trigger --no-save
  ```

## 5. 不要提交

- `build/`、`compile_commands.json`
- `data/captures/`、`data/calib_images/`、`data/calib/`
- `output/` 下的结果
- 个人 `config/local.json`、IDE 配置

## 6. Pull Request 检查表

- [ ] 能在干净目录 `cmake --preset release` + `cmake --build --preset release` 通过
- [ ] `ctest --preset release` 通过
- [ ] `./scripts/format.sh --check` 通过
- [ ] 新增/修改的行为有测试或真实硬件验证记录
- [ ] 文档同步更新（README/docs/CHANGELOG）
- [ ] 没有提交生成物、标定数据或大文件
