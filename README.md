# LegionClaw Merged

LegionClaw 是集成了感知（Perception）和规控（PNC - Planning and Control）功能的统一代码仓库。

## 项目结构

本仓库包含以下主要模块：

### 感知模块（Perception）
- **lidar_ground_segmentation**: 激光雷达地面分割
- **lidar_cluster_detect**: 激光雷达聚类检测
- **lidar_detect_voxel**: centerpoint障碍物检测
- **multi_lidar_splicing**: 多激光雷达点云拼接
- **fusion/motion_manager**: 障碍物跟踪

### 规控模块（PNC）
- **planning**: 路径规划模块
- **control**: 控制模块
- **prediction**: 预测模块
- **routing**: 路由规划模块

### 公共模块
- **common**: 通用工具库（数学、时间、状态机等）
- **message**: 消息接口定义（支持ROS、ROS2、DDS、LCM）

## 前置条件

1. **第三方库**: 确保所需的第三方库已经安装
2. **ROS2环境**: 确保ROS2环境已经配置（推荐使用ROS2 Galactic 或 Humble）

### 当前环境说明

- ROS2 建议使用 `humble`（示例命令可将 `galactic` 替换为 `humble`）。
- `multi_lidar_splicing` 需要 `ros-humble-pcl-ros`。
- `lidar_detect_voxel` 需要 CUDA + TensorRT 8.x + ONNX parser + `libprotobuf17`。
- `lidar_detect_voxel` 建议使用 `gcc-10/g++-10` 作为 host compiler。

## 编译

### 1. 编译消息接口

首先需要编译ROS2消息接口，这是所有模块的基础依赖：

```bash
cd modules/message/ros2
source /opt/ros/humble/setup.bash
colcon build
```

### 2. 编译感知模块

#### 2.1 lidar_ground_segmentation 编译

```bash
source modules/message/ros2/install/setup.bash
cd modules/perception/lidar/lidar_ground_segmentation
./scripts/build.x64.sh
```

#### 2.2 lidar_cluster_detect 编译

```bash
source modules/message/ros2/install/setup.bash
cd modules/perception/lidar/lidar_cluster_detect
./scripts/build.x64.sh
```

#### 2.3 multi_lidar_splicing 编译

```bash
source modules/message/ros2/install/setup.bash
cd modules/perception/lidar/multi_lidar_splicing
./scripts/build_release.sh
```

#### 2.4 lidar_detect_voxel 编译

```bash
source modules/message/ros2/install/setup.bash
cd modules/perception/lidar/lidar_detect_voxel
./scripts/build.x64.sh
```

推荐（x64）使用 gcc-10 以避免 CUDA 编译器兼容问题：

```bash
source modules/message/ros2/install/setup.bash
cd modules/perception/lidar/lidar_detect_voxel
rm -rf build
CC=/usr/bin/gcc-10 CXX=/usr/bin/g++-10 CUDAHOSTCXX=/usr/bin/g++-10 ./scripts/build.x64.sh
```

详细依赖/排障见：`modules/perception/lidar/lidar_detect_voxel/readme.md`

#### 2.5 motion_manager 编译

```bash
source modules/message/ros2/install/setup.bash
cd modules/perception/fusion/motion_manager
./scripts/build.x64.sh
```

### 3. 编译规控模块

#### 3.1 planning 编译

```bash
source modules/message/ros2/install/setup.bash
cd modules/planning/
./scripts/build.x64.sh
```

#### 3.2 control 编译

```bash
source modules/message/ros2/install/setup.bash
cd modules/control/
./scripts/build.x64.sh
```

#### 3.3 routing 编译

```bash
source modules/message/ros2/install/setup.bash
cd modules/routing/
./scripts/build.x64.sh
```

#### 3.4 prediction 编译

```bash
source modules/message/ros2/install/setup.bash
cd modules/prediction/
./scripts/build.x64.sh
```

### 4. 配置

每个子模块生成的 `bin/conf/<module>` 目录下都包含用于控制流程的 `params.json` 或 `config_*.json` 文件。

融合管理模块把卡尔曼滤波、目标跟踪等配置拆分为 `bin/conf/perception/fusion/motion_manager/config_kalman.json`、`config_centerpoint.json` 等多个文件。

## 运行

### 感知模块

#### lidar_ground_segmentation 运行

```bash
source modules/message/ros2/install/setup.bash
cd modules/perception/lidar/lidar_ground_segmentation/bin
./lidar_ground_segmentation
```

#### lidar_cluster_detect 运行

```bash
source modules/message/ros2/install/setup.bash
cd modules/perception/lidar/lidar_cluster_detect/bin
./lidar_cluster_detect
```

#### multi_lidar_splicing 运行

```bash
cd modules/perception/lidar/multi_lidar_splicing/bin
./multi_lidar_splicing
```

#### lidar_detect_voxel 运行

```bash
cd modules/perception/lidar/lidar_detect_voxel/bin
./lidar_detect
```

#### motion_manager 运行

```bash
source modules/message/ros2/install/setup.bash
cd modules/perception/fusion/motion_manager/bin
./motion_manager
```

### 规控模块

#### planning 运行

```bash
source modules/message/ros2/install/setup.bash
cd modules/planning/bin
./planning
```

#### control 运行

```bash
source modules/message/ros2/install/setup.bash
cd modules/control/bin
./control
```

#### routing 运行

```bash
source modules/message/ros2/install/setup.bash
cd modules/routing/bin
./routing
```

#### prediction 运行

```bash
source modules/message/ros2/install/setup.bash
cd modules/prediction/bin
./prediction
```

## 平台支持

所有模块都支持多种平台的编译：
- **x64**: 使用 `build.x64.sh` 脚本
- **arm64**: 使用 `build.arm64.sh` 脚本
- **mdc**: 使用 `build.mdc.sh` 脚本（部分模块）
- **pallas**: 使用 `build.pallas.sh` 脚本（部分模块）
- **x9u**: 使用 `build.x9u.sh` 脚本（部分模块）

## 接口说明

详细的接口说明文档请参考：`docs/legion_interface.xlsx`

## 常见问题

1. 报错找不到 ROS2 消息（`no definition for message ...`）通常是因为未完成消息接口编译，进入 `modules/message/ros2` 执行 `colcon build` 并 `source install/setup.bash` 后再编译感知模块即可解决。
2. 运行二进制找不到配置文件时，请确保当前工作目录为 `bin` 并且 `bin/conf` 与可执行文件在同一级。
3. `lidar_detect_voxel` 报 `NvInfer.h`、`NvOnnxConfig.h`、`libprotobuf.so.17` 相关错误时，请优先检查该模块的专属文档：`modules/perception/lidar/lidar_detect_voxel/readme.md`。

## 注意事项

1. 编译前请确保已正确设置ROS2环境变量
2. 所有模块的编译都依赖于消息接口的编译，请先完成消息接口的编译
3. 默认编译为Debug模式，如需Release模式，请修改对应的编译脚本
4. 不同平台的编译脚本可能略有不同，请根据目标平台选择合适的脚本

## Skills

本项目提供了一组 Cursor AI Skills，帮助你理解和使用 LegionClaw 的各个模块。

### 可用的 Skills

| Skill | 描述 |
|-------|------|
| `legionclaw-overview` | 系统架构总览，了解模块间消息流和算法流程 |
| `legionclaw-lidar-ground-segmentation` | 激光雷达地面分割模块 |
| `legionclaw-lidar-cluster-detect` | 激光雷达聚类检测模块 |
| `legionclaw-lidar-voxel-detect` | Voxel 障碍物检测模块 (CenterPoint) |
| `legionclaw-motion-manager-fusion` | 障碍物跟踪与融合模块 |
| `legionclaw-map` | FAST-LIO-SAM 建图模块 |
| `legionclaw-localization` | HDL NDT 定位模块 |
| `legionclaw-planning` | 路径规划模块 (Lattice + Parking) |
| `legionclaw-control` | 控制模块 (PID + LQR) |
| `legionclaw-prediction` | 障碍物预测模块 |
| `legionclaw-routing` | 路由规划模块 |

### 使用方法

在 Cursor 中输入 `@legionclaw-xxx` 即可调用对应的 skill，例如：
- `@legionclaw-overview` - 查看系统架构
- `@legionclaw-planning` - 了解规划模块的使用

每个 skill 包含：
- 消息流 (输入/输出 Topic)
- 核心算法流程
- 参数配置说明
- 编译和运行方法
- 常见问题排查
