# build_perception_*.sh 使用文档

本文档说明以下脚本的使用方式：

- `build_perception_x64.sh`
- `build_perception_arm.sh`

## 1. 功能概览

两个脚本都会执行以下流程：

1. 解析参数，设置 `ROS_ENABLE/ROS2_ENABLE/LCM_ENABLE/DDS_ENABLE`
2. （可选）构建 ROS2 消息工作区：`modules/message/ros2`
3. 更新各子模块 `scripts/build.*.sh` 中的 CMake 开关（`-DXXX_ENABLE=ON/OFF`）
4. 检查并按需生成 `vehicle_param.proto` 的 C++ 文件
5. 调用已启用的感知子模块构建脚本

## 2. 参数说明

两个脚本参数一致：

- `--ros`：启用 ROS
- `--ros2`：启用 ROS2（并触发 ROS2 message 构建）
- `--lcm`：启用 LCM
- `--dds`：启用 DDS
- `--ros2-distro <name>`：指定 ROS2 发行版（如 `humble`、`jazzy`）
- `-h, --help`：查看帮助

ROS2 发行版自动检测顺序：

1. `ROS2_DISTRO` 环境变量
2. `ROS_DISTRO` 环境变量
3. `/opt/ros/` 下按顺序探测：`jazzy -> iron -> humble -> galactic -> foxy`

## 3. 使用示例

在仓库根目录执行：

```bash
# x64，启用 ROS2（推荐常用）
./build_perception_x64.sh --ros2

# arm64，启用 ROS2 并指定发行版
./build_perception_arm.sh --ros2 --ros2-distro humble

# 同时启用 ROS2 + LCM + DDS
./build_perception_x64.sh --ros2 --lcm --dds
```

查看帮助：

```bash
./build_perception_x64.sh --help
./build_perception_arm.sh --help
```

## 4. 当前默认构建范围（按脚本现状）

注意：以下是你当前脚本中“未注释”的模块，后续改脚本后可能变化。

- `build_perception_x64.sh`
  - 当前默认仅执行：`modules/perception/fusion/motion_manager/scripts/build.x64.sh`
- `build_perception_arm.sh`
  - 当前默认仅执行：`modules/perception/lidar/multi_lidar_splicing/scripts/build.arm64.sh`

如果要构建更多模块，取消对应 `run_script` 行的注释即可。

## 5. 依赖与环境要求

- 基础工具：`bash`、`cmake`、`make`、`colcon`、`protoc`
- ROS2 环境：`/opt/ros/<distro>/setup.bash`
- 工程内 ROS2 工作区：`modules/message/ros2`
- 第三方 `protoc`（优先）：
  - x64: `/usr/local/legion/third_party/x64/bin/protoc`
  - arm64: `/usr/local/legion/third_party/arm64/bin/protoc`

## 6. 常见问题

### Q1：报 `failed to detect ROS2 distro`

处理方式：

```bash
./build_perception_x64.sh --ros2 --ros2-distro humble
```

### Q2：报 `protoc not found`

- 安装系统 `protoc`，或
- 确认 `/usr/local/legion/third_party/<arch>/bin/protoc` 存在并可执行

### Q3：为什么子模块构建参数会“被改写”？

脚本会用 `sed -i` 更新子模块 `build.*.sh` 中的 `-DROS_ENABLE/-DROS2_ENABLE/-DLCM_ENABLE/-DDDS_ENABLE`，这是预期行为，用于统一开关。
