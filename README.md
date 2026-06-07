# LegionClaw

LegionClaw 是一个自动驾驶感知与定位软件平台，包含激光雷达建图、定位、感知、预测、规划等模块。

## 目录结构

```
legionclaw/
├── scripts/                  ← 启动脚本
│   ├── slam/                 # 建图与定位
│   └── perception/           # 感知
├── modules/                  ← 功能模块
│   ├── map/                  # FAST-LIO-SAM 建图
│   ├── localization/         # NDT+UKF 定位 + hdl_localization
│   ├── driver/               # 传感器驱动（livox）
│   ├── message/              # ROS2 消息定义
│   ├── perception/           # 感知
│   ├── planning/             # 规划
│   ├── prediction/           # 预测
│   ├── control/              # 控制
│   ├── routing/              # 路由
│   └── common/               # 公共库
├── ros2_ws/                  ← ROS2 colcon workspace
├── data/                     ← bag 数据（不上传 git）
├── output/                   ← 建图输出（不上传 git）
└── test_result/              ← 定位结果（不上传 git）
```

## 快速开始

### 环境要求

- Ubuntu 22.04 + ROS2 Humble
- 依赖: PCL 1.12+, Eigen3, OpenCV, GeographicLib, OpenMP

### 建图测试

```bash
# 1. 编译
./scripts/slam/build_slam.sh

# 2. 建图（播 bag → 生成 PCD 地图）
./scripts/slam/run_mapping.sh

# 3. 定位（加载地图 → 播 bag → 记录 odom 轨迹）
./scripts/slam/run_localization.sh
```

详细说明见 [wiki/](wiki/)

## 模块说明

| 模块 | 说明 | ROS2包 |
|------|------|--------|
| **map** | FAST-LIO-SAM 激光雷达建图 | `legionclaw_map` |
| **localization** | NDT scan matching + UKF 位姿估计 | `hdl_localization` |
| **driver/livox** | Livox 激光雷达 ROS2 驱动 | `livox_ros_driver2` |

## 关键修复

在建图模块中修复了 esekf EKF 预测时的 segfault 问题（缺失 `init_dyn_runtime_share` 调用）。
