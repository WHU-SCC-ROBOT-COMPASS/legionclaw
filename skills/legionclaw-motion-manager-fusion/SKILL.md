---
name: legionclaw-motion-manager-fusion
description: Multi-sensor fusion and obstacle tracking. Read obstacle lists, run Kalman filtering + IOU fusion, output tracked obstacle list and tune parameters.
argument-hint: [input_obstacle_bag] [output_dir] [fusion_config]
allowed-tools: Bash(ssh *), Bash(cd *), Bash(python3 *), Bash(rostopic *), Bash(rosbag *), Bash(ros2 *), Read, Write, Glob, Grep
---

# LegionClaw Motion Manager Fusion Module

Module: `modules/perception/fusion/motion_manager/`

## 1. Message Flow

```
INPUT:
  - ObstacleList (from lidar_cluster_detect): 聚类障碍物列表
  - ObstacleList (from lidar_detect_voxel): 深度学习检测结果
  - Location: 车辆定位信息 (heading, utm_position)

OUTPUT:
  - ObstacleListOutput: 融合跟踪后的障碍物列表
  - Faults: 故障状态
```

### 订阅/发布接口

| 消息类型 | 方向 | Topic | 格式 |
|---------|------|-------|------|
| ObstacleList (lidar_cluster) | 订阅 | configurable | legion::interface::ObstacleList |
| ObstacleList (voxel_detect) | 订阅 | configurable | legion::interface::ObstacleList |
| Location | 订阅 | configurable | legion::interface::Location |
| ObstacleListOutput | 发布 | obstacle_list_output | legion::interface::ObstacleList |
| Faults | 发布 | faults | legion::interface::Faults |
| ObuCmdMsg | 订阅 | obu_cmd | 功能激活/去激活控制 |

## 2. Core Algorithm Flow

```
┌─────────────────────────────────────────────────────────────────────┐
│  Init():                                                           │
│    1. Load config.json                                             │
│    2. Load tracker config from config_path                          │
│    3. tracker_.parseConfig() → 加载多目标跟踪器参数                   │
│    4. FaultMonitorInit()                                            │
│    5. Start spin thread + 定时器                                      │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│  MotionManagerRun() [on timer trigger]:                             │
│                                                                      │
│  ┌─ [Step 1] 坐标转换                                                │
│  │   VehicleToWorld(obstacle_list_input, location)                   │
│  │   → 障碍物从车体坐标系转换到世界坐标系                             │
│  │                                                                  │
│  ├─ [Step 2] 障碍物格式转换                                          │
│  │   TransObstacle(objects, obstacle_list_input)                     │
│  │   → legion::interface → motion_manager::interface                │
│  │                                                                  │
│  ├─ [Step 3] 多目标跟踪 (MultiObjectTracker)                         │
│  │   tracker_.onMeasurement(input_objects_msg, location)             │
│  │   → 返回 TrackedObjects (Kalman滤波 + 数据关联)                   │
│  │                                                                  │
│  ├─ [Step 4] 加速度计算                                              │
│  │   Getacc(obstacle_list_output_)                                  │
│  │   → 基于连续5帧速度差计算加速度                                    │
│  │                                                                  │
│  ├─ [Step 5] ID管理                                                  │
│  │   ResetId(obstacle_list_output_)                                 │
│  │   → 重新分配连续的目标ID                                         │
│  │                                                                  │
│  ├─ [Step 6] 速度过滤                                                │
│  │   FilterVel(obstacle_list_output_)                               │
│  │   → 过滤特定类型的无效速度                                         │
│  │                                                                  │
│  ├─ [Step 7] IOU融合                                                 │
│  │   FuseObstacleLists()                                           │
│  │   → 融合两个数据源的障碍物 (lidar_cluster + voxel_detect)         │
│  │                                                                  │
│  └─ [Step 8] ID稳定化                                                │
│      StabilizeFusionIds()                                          │
│      → 基于IOU和位置的历史匹配,稳定目标ID                             │
└─────────────────────────────────────────────────────────────────────┘
```

### MultiObjectTracker 核心流程

```
MultiObjectTracker::onMeasurement():
  1. 数据预处理: 从Location获取自车pose
  2. 预测(Predict): 所有tracker执行Kalman预测
  3. 关联(Associate): 
     → 计算 score_matrix (GNN匈牙利算法)
     → 执行 数据关联 (direct_assignment, reverse_assignment)
  4. 更新(Update): 
     → 已匹配tracker: 执行Kalman更新
     → 未匹配tracker: 标记为待删除
     → 未匹配measurement: 创建新tracker
  5. 格式转换: TrackedObjects → ObstacleList
```

### 数据关联矩阵 (可配置)

| Tracker Type | Car | Truck | Bus | Pedestrian | Bicycle | Motorcycle | Unknown |
|-------------|-----|-------|-----|------------|---------|------------|---------|
| Car | ✓ | | | | | | |
| Truck | | ✓ | | | | | |
| Bus | | | ✓ | | | | |
| Pedestrian | | | | ✓ | | | |
| Bicycle | | | | | ✓ | | |
| Motorcycle | | | | | | ✓ | |
| Unknown | | | | | | | ✓ |

## 3. State Machine

```
┌──────────────────────────────────────────────────────────────────────┐
│  Module States:                                                       │
│                                                                      │
│   ┌──────────┐                                                       │
│   │  NONE    │──Init()──→ ┌────────────┐                           │
│   └──────────┘          │   INIT     │                           │
│                         └─────┬──────┘                           │
│                               │                                    │
│              TaskActivate() ───┼──→ TaskStop()                     │
│                               ↓                                    │
│                        ┌────────────┐                              │
│                        │   ACTIVE   │ (function_activation_=true)  │
│                        │ INACTIVE   │ (function_activation_=false) │
│                        └────────────┘                              │
│                                                                      │
│  OBU Command (code=10086):                                         │
│    DEACTIVATE_BOTH   → TaskStop()                                  │
│    ACTIVATE_DRIVING  → TaskActivate()                              │
│    DEACTIVATE_DRIVING→ TaskStop()                                  │
└──────────────────────────────────────────────────────────────────────┘
```

## 4. Parameter Configuration

### Main Config: `modules/perception/fusion/motion_manager/conf/`

```json
{
    "produce_motion_manager_command_duration": 100,
    "publish_motion_manager_command_duration": 100,
    "use_system_timestamp": false,
    "id_num_max": 200,
    "use_location_time": false,
    "heading_error": 1.58,
    "use_sync": 1,
    "obj_rescale": 1.0,
    "fixed_obstacle": "./conf/fixed_obstacle.json",
    "config_path": "./conf/multi_object_tracker.json",

    "iou_threshold": 0.01,
    "data_timeout_ms": 3000.0,
    "grid_size": 0.1,
    "history_match_max_distance": 5.0,
    "close_match_distance": 2.0,
    "very_close_match_distance": 1.0,
    "lcd_id_offset": 10000,
    "duplicate_frame_threshold": 0.01,
    "time_sync_max_diff": 1.0,
    "match_quality_distance_offset": 0.1
}
```

### MultiObjectTracker Config: `modules/perception/fusion/motion_manager/conf/multi_object_tracker.json`

```json
{
    "can_assign_matrix": [7x7 关联矩阵],
    "max_dist_matrix": [距离阈值矩阵],
    "max_area_matrix": [面积阈值矩阵],
    "min_area_matrix": [最小面积矩阵],
    "max_rad_matrix": [角度阈值矩阵],
    "min_iou_matrix": [IOU阈值矩阵],
    "car_tracker": "normal_vehicle_tracker",
    "truck_tracker": "big_vehicle_tracker",
    "bus_tracker": "big_vehicle_tracker",
    "trailer_tracker": "big_vehicle_tracker",
    "pedestrian_tracker": "pedestrian_tracker",
    "bicycle_tracker": "bicycle_tracker",
    "motorcycle_tracker": "motorcycle_tracker"
}
```

### 关键融合参数说明

| 参数 | 默认值 | 含义 | 调优建议 |
|------|--------|------|---------|
| iou_threshold | 0.01 | IOU匹配阈值 | 太小→误匹配,太大→漏匹配 |
| data_timeout_ms | 3000 | 数据源超时阈值 | 根据数据频率调整 |
| grid_size | 0.1 | 位置网格精度(m) | 影响ID稳定化 |
| history_match_max_distance | 5.0 | 历史匹配最大距离(m) | 快速运动目标增大 |
| close_match_distance | 2.0 | 近距离匹配阈值(m) | 近距离场景减小 |
| lcd_id_offset | 10000 | LCD障碍物ID偏移 | 区分数据源 |
| time_sync_max_diff | 1.0 | 时间同步最大差(s) | 超时跳过同步 |
| heading_error | 1.58 | 航向角修正(度) | LiDAR-IMU偏差校准 |

## 5. Workflow - Run & Tune

### Step 1: 检查输入数据

```bash
# 检查障碍物话题
ros2 topic list | grep obstacle
ros2 topic echo /obstacle_list_input --once

# 检查定位话题
ros2 topic echo /location --once

# 播放rosbag
ros2 bag play <input_bag> --remap \
    /obstacle_list:=/obstacle_list_input \
    /location:=/location_input
```

### Step 2: 配置模块

```bash
# 编辑主配置
nano modules/perception/fusion/motion_manager/conf/config.json

# 编辑跟踪器配置
nano modules/perception/fusion/motion_manager/conf/multi_object_tracker.json
```

### Step 3: 参数调优指南

```bash
# 问题: ID跳变严重
# → 减小 grid_size (0.1 → 0.05)
# → 增大 history_match_max_distance (5.0 → 8.0)

# 问题: 障碍物误合并
# → 提高 iou_threshold (0.01 → 0.05)
# → 减小 close_match_distance (2.0 → 1.0)

# 问题: 障碍物漏检后ID丢失
# → 增大 data_timeout_ms (3000 → 5000)
# → 增大 history_match_max_distance (5.0 → 10.0)

# 问题: 远处目标跟踪不稳定
# → 调整 max_dist_matrix 中对应类型距离阈值
# → 提高 can_assign_matrix 中对应类型允许关联标志
```

### Step 4: 构建和运行

```bash
cd modules/perception/fusion/motion_manager
mkdir -p build && cd build
cmake .. && make -j$(nproc)

./bin/motion_manager --config_file=./conf/config.json
```

### Step 5: 验证输出

```bash
# 监听输出
ros2 topic echo /obstacle_list_output --once

# 检查ID稳定性
python3 check_fusion_ids.py --bag <input_bag> --output <output.json>

# 输出格式
{
    "frame_count": N,
    "obstacle_count": M,
    "id_transitions": [
        {"frame": i, "id_jumps": [{"from": a, "to": b, "distance": d}]}
    ],
    "avg_id_stability_score": 0.95
}
```

## 6. IOU Fusion 详解

```
FuseObstacleLists() 融合流程:

1. 超时检查: 检查数据源是否超时,超时则清空

2. 坐标转换: LCD障碍物从车体坐标→世界坐标

3. IOU匹配循环:
   - 遍历 obstacle_list_output (深度学习检测)
   - 计算与 lcd_obstacle_list 的多边形IOU
   - 计算矩形框IOU (对形状变化更鲁棒)
   - IOU > iou_threshold → 匹配成功 → 融合

4. 融合策略:
   - 面积大的保留位置
   - 深度学习的速度/朝向/类别优先
   - fusion_type = FUSED

5. 未匹配障碍物:
   - obstacle_list_output → fusion_type = CAMERA
   - lcd_obstacle_list → fusion_type = LIDAR, ID+10000

6. ID稳定化 (StabilizeFusionIds):
   - 多轮IOU匹配
   - 历史帧位置匹配
   - 网格位置匹配
   - 避免ID跳变
```

## 7. 输出格式

```protobuf
// ObstacleListOutput 中的 Obstacle:
{
    "id": int,                    // 稳定的目标ID
    "existence_prob": float,       // 存在概率
    "type": int,                  // 类型
    "sub_type": int,              // 子类型 (ST_CAR, ST_PEDESTRIAN, etc.)
    "confidence": float,          // 置信度
    "center_pos_vehicle": Point3D,
    "center_pos_abs": Point3D,
    "velocity_abs": Point3D,       // 世界坐标系速度
    "velocity_vehicle": Point3D,   // 车辆坐标系速度
    "acceleration_abs": Point3D,   // 世界坐标系加速度
    "theta_vehicle": float,        // 车辆坐标系朝向角
    "theta_abs": float,           // 世界坐标系朝向角
    "length/width/height": float,  // 尺寸
    "polygon_point_abs": [Point3D], // 凸包多边形
    "fusion_type": int,           // FUSED/CAMERA/LIDAR
    "lane_position": int,         // 车道位置 (-3~3)
    "timestamp": Time
}
```
