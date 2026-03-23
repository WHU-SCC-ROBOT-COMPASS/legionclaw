---
name: legionclaw-map
description: FAST-LIO-SAM based LiDAR-IMU odometry and mapping. Read IMU/PointCloud, run iESKF + iKD-Tree, output pose trajectory and point cloud map, save maps.
argument-hint: [config_file]
allowed-tools: Bash(ls *), Bash(find *), Bash(cd *), Bash(mkdir *), Bash(make *), Read, Glob, Grep
---

# LegionClaw Map Module (FAST-LIO-SAM)

Module: `modules/map/`

## 1. Message Flow

```
INPUT:
  - PointCloud2 (ROS2): /points
  - IMU: /imu

OUTPUT:
  - Odometry: /odom (TF: map → odom)
  - Path: /path (trajectory)
  - PointCloud: /cloud (world frame)
  - Services: /save_map, /save_pose
```

### 订阅/发布接口

| 消息类型 | 方向 | Topic | 格式 |
|---------|------|-------|------|
| PointCloud2 | 订阅 | /points | ROS2 sensor_msgs/PointCloud2 |
| IMU | 订阅 | /imu | ROS2 sensor_msgs/Imu |
| Odometry | 发布 | /odom | nav_msgs/Odometry |
| Path | 发布 | /path | nav_msgs/Path |
| PointCloud2 | 发布 | /cloud | sensor_msgs/PointCloud2 |
| PointCloud2 | 发布 | /cloud_body | sensor_msgs/PointCloud2 |
| Service | 提供 | /save_map | slam_msgs/SaveMap |
| Service | 提供 | /save_pose | slam_msgs/SavePose |

### TF

- `tf2_ros::TransformBroadcaster` → 发布 `map` → `odom` 变换
- `tf2_ros::TransformListener` → 接收 `odom` → `base_link` (来自 localization)

---

## 2. Core Algorithm Flow

```
┌─────────────────────────────────────────────────────────────────────┐
│  Init():                                                           │
│    1. Load config (acc_cov, gyr_cov, iESKF params)              │
│    2. Initialize IKFoM state (pos, vel, ori, ba, bg)            │
│    3. Initialize iKD-Tree for map storage                         │
│    4. MESSAGE_INIT()                                              │
│    5. Start spin thread                                           │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│  Main Loop [100Hz]:                                               │
│                                                                      │
│    1. IMU Processing:                                            │
│       → Pre-integration (propagate state)                         │
│       → iESKF predict                                            │
│                                                                      │
│    2. PointCloud Processing:                                      │
│       → Feature extraction (edge + surface points)                │
│       → Downsampling (voxel grid)                                 │
│       → Lidar Odometry (iKD-Tree scan-match)                    │
│       → iESKF correct                                            │
│       → Publish pose, path, cloud                                 │
│                                                                      │
│    3. KeyFrame Management:                                        │
│       → Check distance/angle threshold                            │
│       → Add to sliding window                                     │
│       → ISAM2 optimize (if loop closure enabled)                 │
└─────────────────────────────────────────────────────────────────────┘
```

### FAST-LIO-SAM Algorithm

```
1. LiDAR-IMU 紧耦合 (iESKF):
   - State: [pos, vel, rot, ba, bg] (18维)
   - Process: IMU 预积分传播状态
   - Measurement: LiDAR 特征点残差

2. 增量 KD-Tree (iKD-Tree):
   - 增量式最近邻搜索
   - O(log n) 平均复杂度
   - 支持动态插入点云

3. ISAM2 优化 (可选):
   - 滑动窗口优化
   - 回环检测 (scan-context)
   - 因子图后端
```

---

## 3. Parameter Configuration

### Main Config: `modules/map/src/apps/fast_lio_sam.json`

```json
{
    "app_name": "fast_lio_sam",
    "description": "FAST-LIO-SAM mapping module",
    "log_file_path": "./log",
    "log_level": 1,

    "acc_cov": 0.1,
    "gyr_cov": 0.1,
    "b_acc_cov": 0.0001,
    "b_gyr_cov": 0.0001,

    "lidar_to_imu_trans": [0.0, 0.0, 0.0],
    "lidar_to_imu_rot": [1, 0, 0, 0, 1, 0, 0, 0, 1],

    "filter_size_surf": 0.5,
    "filter_size_corner": 0.5,
    "max_iteration": 4,

    "cube_side_length": 200.0,
    "det_range": 300.0,

    "loop_closure_en": false,
    "loop_closure_interval": 100,
    "loop_closure_threshold": 0.3
}
```

### iESKF 参数

| 参数 | 默认值 | 含义 | 调优建议 |
|------|--------|------|---------|
| acc_cov | 0.1 | 加速度噪声 | 振动大提高 |
| gyr_cov | 0.1 | 角速度噪声 | 陀螺仪漂移提高 |
| b_acc_cov | 0.0001 | 加速度偏置噪声 | 长期漂移调整 |
| b_gyr_cov | 0.0001 | 角速度偏置噪声 | 偏置稳定性调整 |

### LiDAR-IMU 外参

| 参数 | 默认值 | 含义 |
|------|--------|------|
| lidar_to_imu_trans | [0,0,0] | LiDAR 到 IMU 平移 (xyz, 米) |
| lidar_to_imu_rot | 单位矩阵 | LiDAR 到 IMU 旋转矩阵 (行优先 3x3) |

### 点云滤波参数

| 参数 | 默认值 | 含义 | 调优建议 |
|------|--------|------|---------|
| filter_size_surf | 0.5m | 地面点体素尺寸 | 密提高精度, 疏提高速度 |
| filter_size_corner | 0.5m | 角点体素尺寸 | 同上 |
| max_iteration | 4 | iESKF 最大迭代 | 收敛快降低, 不收敛提高 |

---

## 4. Build & Run

### 构建

```bash
cd modules/map/make
cmake .. -DCMAKE_BUILD_TYPE=Release -DROS2_ENABLE=ON -DGLOG_ENABLE=ON -DNDT_OMP_ENABLE=ON
make -j$(nproc)
```

### 运行

```bash
# 启动建图节点
./bin/map --config_file=./conf/fast_lio_sam.json

# 或使用 launch
ros2 launch map fast_lio_sam.launch.py
```

### 保存地图

```bash
# 调用保存地图服务
ros2 service call /save_map slam_msgs/srv/SaveMap "{resolution: 0.5, destination: '/tmp/map.pcd'}"

# 保存轨迹
ros2 service call /save_pose slam_msgs/srv/SavePose "{resolution: 0.1, destination: '/tmp/pose.txt'}"
```

---

## 5. Output Formats

### Odometry Message

```yaml
header:
  stamp: 1234567890.123
  frame_id: "map"
child_frame_id: "odom"
pose:
  pose:
    position: {x: 10.5, y: 3.2, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}
```

### Path Message

```yaml
header:
  frame_id: "map"
poses:
  - pose:
      position: {x: 0.0, y: 0.0, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  - pose:
      position: {x: 1.0, y: 0.0, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.1, w: 0.995}
  # ... more poses
```

---

## 6. 集成到 Pipeline

```
LiDAR PointCloud
     ↓
[map] FAST-LIO-SAM (在线建图)
     ↓
/odom (TF: map→odom) → 供 localization 使用
     ↓
/cloud (点云地图) → 供 localization 加载
     ↓
Perception (detection, tracking)
     ↓
Prediction → Planning → Control
```

### 与 Localization 配合

1. **建图阶段**: 运行 `map` 模块，生成 `.pcd` 地图
2. **定位阶段**: 运行 `localization` 模块，加载 `.pcd` 作为 globalmap

---

## 7. 常见问题排查

| 问题 | 原因 | 解决方案 |
|------|------|---------|
| 点云漂移 | IMU 噪声参数太大 | 降低 acc_cov/gyr_cov |
| 建图不收敛 | 特征点太少 | 调整 filter_size_surf |
| 里程计抖动 | 外参不准 | 校准 lidar_to_imu |
| 实时性差 | 点云太多 | 增大 downsample 分辨率 |
