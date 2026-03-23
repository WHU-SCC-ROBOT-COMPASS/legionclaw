---
name: legionclaw-lidar-cluster-detect
description: LiDAR obstacle clustering. Read point cloud data, run grid-based clustering, output obstacle list and tune parameters.
argument-hint: [input_file_or_rosbag] [output_dir] [config_override]
allowed-tools: Bash(ssh *), Bash(cd *), Bash(python3 *), Bash(rostopic *), Bash(rosbag *), Bash(ros2 *), Read, Write, Glob, Grep
---

# LegionClaw LiDAR Cluster Detection Module

Module: `modules/perception/lidar/lidar_cluster_detect/`

## 1. Message Flow

```
INPUT:
  - PointCloud (legionclaw::interface::PointCloud): 订阅点云数据
    → 支持 ROS2/ROS/DDS/LCM 多协议

OUTPUT:
  - ObstacleList (legionclaw::interface::ObstacleList): 障碍物列表
  - ClusterPointCloud (ROS2): 聚类后的点云 (可视化用)
  - Faults: 故障状态
```

### 订阅/发布接口

| 消息类型 | 方向 | Topic | 格式 |
|---------|------|-------|------|
| PointCloud | 订阅 | configurable | legionclaw::interface::PointCloud |
| ObstacleList | 发布 | obstacle_list | legionclaw::interface::ObstacleList |
| ClusterPointCloud | 发布 | cluster_points | ROS2 PointCloud2 |
| Faults | 发布 | faults | legionclaw::interface::Faults |
| ObuCmdMsg | 订阅 | obu_cmd | 功能激活/去激活控制 |

## 2. Core Algorithm Flow

```
┌─────────────────────────────────────────────────────────────────────┐
│  Init():                                                           │
│    1. Load config.json                                             │
│    2. Load clustering params from params_file (JSON)                │
│    3. Calculate grid dimensions (col_size, row_size)                │
│    4. FaultMonitorInit()                                            │
│    5. Start spin thread + 100ms timer                               │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│  ComputeLidarClusterDetectCommandOnTimer() [100ms]:                 │
│    1. BuildCellMap() → 建立网格地图                                  │
│    2. ClusterConnectedPoints() → 8连通域聚类                         │
│    3. BuildCluster() → 构建障碍物包围盒                              │
│    4. ConvertClustersToObstacleList() → 转换为标准障碍物格式         │
│    5. PublishObstacleList() → 发布结果                              │
└─────────────────────────────────────────────────────────────────────┘
```

### Grid-Based Clustering Algorithm

```
BuildCellMap():
  1. 将点云按 (x,y) 坐标分配到网格单元
  2. 过滤掉 z 值超出 [cell_map_min_z, cell_map_max_z] 的点
  3. 返回 CellMatrix (col_size × row_size 的网格)

ClusterConnectedPoints():
  1. 遍历所有网格,寻找未访问且点数 > point_min_size_for_cell 的单元
  2. 使用 Find8() 进行8连通域搜索(上下左右+4个对角)
  3. 合并连通区域形成障碍物簇
  4. 过滤掉过大的box (cell数 > cell_max_size_for_big_box)

BuildCluster():
  1. 遍历所有簇,过滤点数 < point_min_size_for_box 的
  2. 计算每个簇的包围盒 (min_x, max_x, min_y, max_y, min_z, max_z)
  3. 计算质心 (center_x, center_y, center_z)
  4. 过滤 height < obstacle_min_height 的
```

### State Machine

```
┌──────────────────────────────────────────────────────────────────────┐
│  States: NONE → INIT → INACTIVE/ACTIVE                              │
│                                                                      │
│  OBU Command (code=10086):                                         │
│    val=DEACTIVATE_BOTH   → TaskStop()                               │
│    val=ACTIVATE_DRIVING  → TaskActivate()                           │
│    val=DEACTIVATE_DRIVING→ TaskStop()                               │
└──────────────────────────────────────────────────────────────────────┘
```

## 3. Parameter Configuration

### Main Config: `modules/perception/lidar/lidar_cluster_detect/conf/`

```json
{
    "produce_lidar_cluster_detect_command_duration": 100,
    "publish_lidar_cluster_detect_command_duration": 100,
    "use_system_timestamp": false,
    "params_file": "./conf/cluster_params.json",
    "status": {
        "status_detect_duration": 10
    }
}
```

### Clustering Params (from params_file):

| 参数 | 默认值 | 含义 | 调优建议 |
|------|--------|------|---------|
| cell_map_min_x | -10.0 | 网格x范围最小值(m) | 根据感知范围调整 |
| cell_map_max_x | 10.0 | 网格x范围最大值(m) | 根据感知范围调整 |
| cell_map_min_y | -20.0 | 网格y范围最小值(m) | 前向距离大 |
| cell_map_max_y | 40.0 | 网格y范围最大值(m) | 前向40m |
| cell_map_min_z | -0.9 | 网格z范围最小值(m) | 过滤地面以下 |
| cell_map_max_z | 0.5 | 网格z范围最大值(m) | 过滤高处噪声 |
| cell_width | 0.4 | 网格宽度(m) | 影响聚类精度 |
| cell_length | 0.8 | 网格长度(m) | 影响聚类精度 |
| point_min_size_for_cell | 5 | 最小有效网格点数 | 过滤噪声 |
| point_min_size_for_box | 10 | 最小障碍物点数 | 过滤小目标 |
| cell_max_size_for_big_box | 100 | 最大允许网格数 | 过滤超大目标 |
| obstacle_min_height | 0.05 | 最小障碍物高度(m) | 过滤低矮目标 |
| delta_x | 0.0 | 位置修正X(m) | 安装偏差补偿 |
| delta_y | 0.0 | 位置修正Y(m) | 安装偏差补偿 |

## 4. Workflow - Run & Tune

### Step 1: Find Input Data

```bash
# 查看输入点云格式
ros2 topic info /input/pointcloud --verbose

# 如果是rosbag
ros2 bag play <input_bag> --remap /input/pointcloud:=/sensor/lidar/points
```

### Step 2: Configure Module

```bash
# 编辑聚类参数
nano modules/perception/lidar/lidar_cluster_detect/conf/cluster_params.json

# 调参要点:
# - 车辆周围区域: cell_map_min_x/max_x, cell_map_min_y/max_y
# - 聚类精度: cell_width, cell_length (越小精度越高但计算量大)
# - 噪声过滤: point_min_size_for_cell, point_min_size_for_box
# - 低矮障碍物: obstacle_min_height (根据场景调整)
```

### Step 3: Build & Run

```bash
cd modules/perception/lidar/lidar_cluster_detect
mkdir -p build && cd build
cmake .. && make -j$(nproc)

./bin/lidar_cluster_detect --config_file=./conf/config.json
```

### Step 4: Parameter Tuning Guide

```bash
# 场景1: 室内停车场
# → 减小 cell_width/length (0.4/0.8 → 0.2/0.4)
# → 提高 obstacle_min_height (0.05 → 0.1) 过滤地锁

# 场景2: 室外道路
# → 增大感知范围 cell_map_max_y (40 → 80)
# → 降低 point_min_size_for_box (10 → 5) 检出人

# 场景3: 拥挤交通
# → 提高 cell_max_size_for_big_box (100 → 200) 适应大型车辆
# → 降低 cell_width (0.4 → 0.3) 提高分辨率
```

## 5. Troubleshooting

| 问题 | 原因 | 解决方案 |
|------|------|---------|
| 障碍物过碎 | cell_width/length太大 | 减小网格尺寸 |
| 障碍物合并 | cell_width/length太小 | 增大网格尺寸 |
| 检测不到行人 | point_min_size_for_box太大 | 减小阈值 |
| 超大box噪声 | cell_max_size_for_big_box太小 | 增大阈值 |

## 6. Output Format

```protobuf
// ObstacleList 中的 Obstacle:
{
    "id": int,                    // 障碍物ID
    "existence_prob": float,      // 存在概率 (1.0)
    "type": int,                  // 类型 (0=unknown)
    "confidence": float,          // 置信度 (1.0)
    "center_pos_vehicle": Point3D, // 车体坐标系中心
    "center_pos_abs": Point3D,     // 世界坐标系中心
    "length": float,              // 长度(m)
    "width": float,               // 宽度(m)
    "height": float,              // 高度(m)
    "polygon_point_vehicle": [Point3D, ...],  // 车辆坐标系多边形
    "polygon_point_abs": [Point3D, ...],      // 世界坐标系多边形
    "timestamp": Time             // 时间戳
}
```
