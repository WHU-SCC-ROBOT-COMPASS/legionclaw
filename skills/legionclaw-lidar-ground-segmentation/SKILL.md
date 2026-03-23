---
name: legionclaw-lidar-ground-segmentation
description: LiDAR ground segmentation using PatchWork++. Read point cloud data (rosbag/JSON), run ground/non-ground separation, output results and tune parameters.
argument-hint: [input_file_or_rosbag] [output_dir] [config_override]
allowed-tools: Bash(ssh *), Bash(cd *), Bash(python3 *), Bash(rostopic *), Bash(rosbag *), Bash(ros2 *), Read, Write, Glob, Grep
---

# LegionClaw LiDAR Ground Segmentation Module

Module: `modules/perception/lidar/lidar_ground_segmentation/`

## 1. Message Flow

```
INPUT:
  - PointCloud2 (ROS2/ROS/DDS/LCM): /sensor/lidar/front/PointCloud2 or configured topic
    → Eigen::MatrixXf (原始点云: x,y,z,intensity/ring/time)

OUTPUT:
  - GroundPoints: Eigen::MatrixX3f (地面点, z值较低的点)
  - NoGroundPoints: Eigen::MatrixX3f (非地面点/障碍物点)
  - Faults: legionclaw::interface::Faults (故障状态)
  - Topics: PublishGroundPoints, PublishNoGroundPoints
```

### 订阅/发布接口

| 消息类型 | 方向 | Topic | 格式 |
|---------|------|-------|------|
| PointCloud2 | 订阅 | /sensor/lidar/*/PointCloud2 | ROS2 sensor_msgs/PointCloud2 |
| GroundPoints | 发布 | ground_points | 自定义 Float32MultiArray 或 PointCloud2 |
| NoGroundPoints | 发布 | no_ground_points | 自定义 Float32MultiArray 或 PointCloud2 |
| Faults | 发布 | faults | legionclaw::interface::Faults |
| ObuCmdMsg | 订阅 | obu_cmd | 功能激活/去激活控制 |

## 2. Core Algorithm Flow

```
┌─────────────────────────────────────────────────────────────────────┐
│  Init():                                                           │
│    1. Load config.json → produce/publish_duration, use_system_ts   │
│    2. Load patchworkpp params from params_file                       │
│    3. Create PatchWorkpp instance                                   │
│    4. FaultMonitorInit()                                            │
│    5. Start spin thread                                             │
│    6. TaskActivate() → start 1000ms timer                          │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│  HandlePointCloudInput(Eigen::MatrixXf cloud):                      │
│    → patchworkpp_->estimateGround(cloud)                             │
│    → getGround() → ground_points                                    │
│    → getNonground() → nonground_points                              │
│    → PublishGroundPoints(ground_points)                              │
│    → PublishNoGroundPoints(nonground_points)                         │
└─────────────────────────────────────────────────────────────────────┘
```

### PatchWork++ Algorithm (patchworkpp)

PatchWork是一种基于区域生长的地面分割算法,核心流程:

1. **CZM (Concentric Zone Model)**: 将点云分为多个同心区域(近/中/远),不同区域使用不同参数
2. **种子点选择**: 找到每个区域中z值最低的点作为种子点(LPR - Lowest Point Representative)
3. **区域生长**: 从种子点开始,基于高度差阈值(th_seeds)扩展地面点
4. **平整度检查**: 检查区域内的平整度,平坦区域标记为地面
5. **RNR (Reverse Nearest neighbor Re-refinement)**: 去除反射点等噪声

### State Machine

```
┌──────────────────────────────────────────────────────────────────────┐
│  Module States:                                                       │
│                                                                      │
│   ┌──────────┐    Init()     ┌────────────┐   TaskActivate()        │
│   │  NONE    │ ──────────→  │   INIT     │ ─────────────────────→   │
│   └──────────┘              └────────────┘                           │
│                                │                                      │
│   ┌──────────────┐            │ TaskStop()         ┌─────────────┐  │
│   │  INACTIVE    │ ←─────────────────────────────── │  ACTIVE     │  │
│   │  (function_  │                                 │  (function_ │  │
│   │   activation_│                                 │   activation │ │
│   │   = false)   │────────────────────────────────→│   = true)   │  │
│   └──────────────┘           TaskStop()            └─────────────┘  │
│                                                                      │
│  OBU Command Codes:                                                  │
│    10086/DEACTIVATE_BOTH   → TaskStop()                             │
│    10086/ACTIVATE_DRIVING  → TaskActivate()                          │
│    10086/DEACTIVATE_DRIVING→ TaskStop()                             │
└──────────────────────────────────────────────────────────────────────┘
```

## 3. Parameter Configuration

### Main Config: `modules/perception/lidar/lidar_ground_segmentation/conf/`

```json
{
    "produce_lidar_ground_segmentation_command_duration": 100,
    "publish_lidar_ground_segmentation_command_duration": 100,
    "use_system_timestamp": false,
    "status": {
        "status_detect_duration": 10
    },
    "params_file": "./conf/patchworkpp_params.json",
    "message": {
        "active_message": [4],
        "message_info": [
            {"type": 0, "name": "LCM", "url": "udpm://239.255.76.21:7621?ttl=3"},
            {"type": 1, "name": "DDS", "url": "null"},
            {"type": 2, "name": "ROS", "url": "null"},
            {"type": 4, "name": "ROS2", "url": "null"}
        ]
    }
}
```

### PatchWork++ Params (from params_file):

| 参数 | 默认值 | 含义 | 调优建议 |
|------|--------|------|---------|
| sensor_height | 1.5 | LiDAR安装高度(m) | 校准安装位置 |
| num_iter | 3 | RANSAC迭代次数 | 复杂地形可增加 |
| num_lpr | 20 | LPR种子点数 | 增加提高稳定性 |
| num_min_pts | 3 | 最小点数 | 过小会产生噪声 |
| th_seeds | 0.15 | 种子点高度阈值(m) | 调整地面/非地面边界 |
| th_dist | 0.25 | 生长距离阈值(m) | 控制地面平滑度 |
| max_range | 80.0 | 最大感知距离(m) | 影响远距离分割 |
| min_range | 0.0 | 最小感知距离(m) | 去除车体附近点 |
| uprightness_thr | 0.5 | 法向量uprightness阈值 | 陡坡场景调整 |
| enable_RNR | true | 启用RNR去噪 | 复杂环境开启 |
| num_zones | 4 | CZM区域数 | 远距离可增加 |
| elevation_thr | [0.1, 0.2, ...] | 各区域高程阈值 | 分区域调优 |
| flatness_thr | [0.1, 0.1, ...] | 各区域平整度阈值 | 控制地面判定 |

### Vehicle Parameters: `modules/common/data/vehicle_param/vehicle_param.json`

| 参数 | 默认值 | 含义 |
|------|--------|------|
| length | 2.677 | 车辆长度(m) |
| width | 1.005 | 车辆宽度(m) |
| height | 1.41 | 车辆高度(m) |
| wheelbase | 1.5 | 轴距(m) |

## 4. Workflow - Run & Tune

### Step 1: Find Input Data

```bash
# Option A: ROS2 bag file
ros2 bag info <input_bag>  # 查看bag内容
ros2 bag play <input_bag> --remap /sensor/lidar/front/PointCloud2:=/input/lidar

# Option B: JSON/PCD file
ls <input_dir>/*.json
ls <input_dir>/*.pcd

# Option C: List all available bags
find /data -name "*.bag" | head -10
```

### Step 2: Configure Module

```bash
# 编辑配置文件
nano modules/perception/lidar/lidar_ground_segmentation/conf/config.json

# 关键调参:
# - th_seeds: 提高则更少点被分类为地面
# - th_dist: 提高则更大坡度被接受为地面
# - enable_RNR: 复杂环境开启
# - max_range: 根据LiDAR型号调整
```

### Step 3: Build & Run

```bash
# Build
cd modules/perception/lidar/lidar_ground_segmentation
mkdir -p build && cd build
cmake .. && make -j$(nproc)

# Run standalone
./bin/lidar_ground_segmentation \
    --config_file=./conf/config.json

# Or run via launch
cd launch
bash launch_all.sh
```

### Step 4: Verify Output

```bash
# 监听输出话题
ros2 topic echo /ground_points --once
ros2 topic echo /no_ground_points --once

# 可视化 (RViz2)
rviz2 -d <viz_config.rviz>

# 检查点云质量
python3 check_output.py --output_dir <output_dir>
```

### Step 5: Parameter Tuning

```bash
# 场景1: 城市道路 (平整地面)
# 降低 th_seeds (0.15 → 0.1)
# 降低 elevation_thr

# 场景2: 停车场/坡道
# 提高 th_seeds (0.15 → 0.2)
# 提高 th_dist (0.25 → 0.4)
# 降低 uprightness_thr (0.5 → 0.3)

# 场景3: 复杂地形(草地/沙地)
# 提高 num_min_pts (3 → 10)
# 关闭 enable_RNR (false)
# 提高 flatness_thr
```

### Step 6: Export Results

```bash
# 保存处理结果到JSON
python3 export_results.py \
    --input <input_bag_or_file> \
    --output <output_dir>/result.json \
    --module ground_segmentation \
    --params <used_params.json>
```

## 5. Troubleshooting

| 问题 | 原因 | 解决方案 |
|------|------|---------|
| 大量点被误分类 | th_seeds太高 | 降低阈值 |
| 地面点包含噪声 | th_dist太低 | 提高阈值 |
| 远处点全被过滤 | max_range太小 | 提高max_range |
| 启动失败 | params_file路径错误 | 检查config.json中路径 |
| 点云订阅无数据 | topic名称不匹配 | 检查ros2 topic list |

## 6. Integration Points

```
┌──────────────────┐
│ PointCloud Input │ ← 来自LiDAR驱动或rosbag
└────────┬─────────┘
         ↓
┌──────────────────────────────────────────────────────────────────────┐
│            lidar_ground_segmentation (PatchWork++)                  │
│                                                                      │
│  GroundPoints ──────────────────────────────────────────────────────→│
│         ↓                                                             │
│  lidar_cluster_detect (输入非地面点进行聚类)                           │
│                                                                      │
│  lidar_detect_voxel (CenterPoint网络进行深度学习检测)                  │
│                                                                      │
│  motion_manager (融合跟踪)                                            │
└──────────────────────────────────────────────────────────────────────┘
```
