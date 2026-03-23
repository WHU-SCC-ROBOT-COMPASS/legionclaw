---
name: legionclaw-localization
description: NDT scan-matching based localization with UKF pose estimation. Read point cloud, load global map, output accurate pose, and tune parameters.
argument-hint: [config_file]
allowed-tools: Bash(ls *), Bash(find *), Bash(cd *), Bash(mkdir *), Bash(make *), Read, Glob, Grep
---

# LegionClaw Localization Module (HDL)

Module: `modules/localization/`

## 1. Message Flow

```
INPUT:
  - PointCloud2: /velodyne_points (实时点云)
  - IMU: /imu_data (IMU 数据，用于预测)
  - PointCloud2: /globalmap (全局地图，预先构建)
  - PoseWithCovarianceStamped: /initialpose (RViz 初始位姿)
  - Service: /relocalize (重定位服务)

OUTPUT:
  - Odometry: /odom (TF: map → odom)
  - PointCloud2: /aligned_points (对齐后的点云)
```

### 订阅/发布接口

| 消息类型 | 方向 | Topic | QoS | 格式 |
|---------|------|-------|-----|------|
| PointCloud2 | 订阅 | /velodyne_points | SensorDataQoS | ROS2 sensor_msgs/PointCloud2 |
| IMU | 订阅 | /imu_data | SensorDataQoS | ROS2 sensor_msgs/Imu |
| PointCloud2 | 订阅 | /globalmap | SensorDataQoS | 全局地图点云 |
| PoseWithCovarianceStamped | 订阅 | /initialpose | Reliable | 初始位姿 |
| Odometry | 发布 | /odom | 10 | nav_msgs/Odometry |
| PointCloud2 | 发布 | /aligned_points | SensorDataQoS | 对齐点云 |
| Service | 提供 | /relocalize | - | std_srvs/Empty |

### TF

- `tf2_ros::TransformBroadcaster` → 发布 `map` → `odom` 变换

---

## 2. Core Algorithm Flow

```
┌─────────────────────────────────────────────────────────────────────┐
│  Init():                                                           │
│    1. Load config.json                                            │
│    2. Create registration (NDT_OMP or NDT_CUDA)                 │
│    3. Initialize PoseEstimator with UKF                           │
│    4. Create voxel grid downsample filter                        │
│    5. Start relocalization thread (optional)                     │
│    6. Subscriptions & Publishers                                  │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│  PointsCallback [10-20Hz]:                                       │
│                                                                      │
│    1. Convert PointCloud2 → PCL                                    │
│    2. Transform to odom frame (TF)                                │
│    3. Downsample (VoxelGrid)                                       │
│                                                                      │
│    4. IMU Prediction (UKF):                                       │
│       → pose_estimator_->predict(stamp, acc, gyro)               │
│       → 基于 IMU 积分预测位姿                                      │
│                                                                      │
│    5. NDT Correction:                                             │
│       → registration_->align(aligned, init_guess)                  │
│       → Scan-matching 获取观测位姿                                 │
│                                                                      │
│    6. UKF Correction:                                            │
│       → pose_estimator_->correct(stamp, cloud)                   │
│       → 融合 IMU 预测 + NDT 观测                                  │
│                                                                      │
│    7. Publish:                                                    │
│       → TF (map→odom)                                             │
│       → Odometry message                                          │
│       → Aligned points                                            │
└─────────────────────────────────────────────────────────────────────┘
```

### UKF Pose Estimation Algorithm

```
1. 状态向量 (16维):
   [px, py, pz,           # 位置 (3)
    vx, vy, vz,           # 速度 (3)
    qw, qx, qy, qz,      # 四元数 (4)
    ax, ay, az,          # 加速度偏置 (3)
    gx, gy, gz]          # 陀螺仪偏置 (3)

2. IMU 预测 (f 函数):
   - 速度积分: v += (acc * g - gravity) * dt
   - 位置积分: p += v * dt
   - 四元数积分: q += q * dq(gyro * dt)

3. NDT 观测 (h 函数):
   - 返回 [pos_x, pos_y, pos_z, qw, qx, qy, qz]

4. 协方差融合:
   - Process noise: 位置 1.0, 速度 1.0, 四元数 0.5, 偏置 1e-6
   - Measurement noise: 位置 0.01, 四元数 0.001
```

---

## 3. Parameter Configuration

### Main Config: `modules/localization/conf/localization.json`

```json
{
    "points_topic": "/velodyne_points",
    "imu_topic": "/imu_data",
    "odom_child_frame_id": "base_link",
    "odom_frame_id": "odom",
    "map_frame_id": "map",

    "use_imu": true,
    "invert_acc": false,
    "invert_gyro": false,
    "cool_time_duration": 2.0,

    "enable_robot_odometry_prediction": false,
    "robot_odom_frame_id": "odom",

    "reg_method": "NDT_OMP",
    "ndt_neighbor_search_method": "DIRECT7",
    "ndt_neighbor_search_radius": 2.0,
    "ndt_resolution": 1.0,
    "downsample_resolution": 0.3,
    "converged_score": 4.0,

    "specify_init_pose": false,
    "init_pos_x": 0.0,
    "init_pos_y": 0.0,
    "init_pos_z": 0.0,
    "init_ori_x": 0.0,
    "init_ori_y": 0.0,
    "init_ori_z": 0.0,
    "init_ori_w": 1.0,

    "use_global_localization": true
}
```

### Registration 参数

| 参数 | 默认值 | 含义 | 调优建议 |
|------|--------|------|---------|
| reg_method | NDT_OMP | 注册方法 | NDT_OMP/CUDA |
| ndt_resolution | 1.0m | NDT 体素分辨率 | 小提高精度但慢 |
| ndt_neighbor_search_method | DIRECT7 | 搜索方法 | DIRECT7/DIRECT1/KDTREE |
| downsample_resolution | 0.3m | 下采样分辨率 | 平衡精度和速度 |
| converged_score | 4.0 | 收敛阈值 | 越小越严格 |

### IMU 参数

| 参数 | 默认值 | 含义 | 调优建议 |
|------|--------|------|---------|
| use_imu | true | 启用 IMU | 关闭只用里程计 |
| invert_acc | false | 加速度取反 | IMU 倒装时开启 |
| invert_gyro | false | 角速度取反 | IMU 倒装时开启 |
| cool_time_duration | 2.0s | 冷却时间 | 初始化期间不预测 |

---

## 4. Build & Run

### 构建

```bash
cd modules/localization/make
cmake .. -DCMAKE_BUILD_TYPE=Release -DROS2_ENABLE=ON -DGLOG_ENABLE=ON -DNDT_OMP_ENABLE=ON
make -j$(nproc)
```

### 运行

```bash
# 启动定位节点
./bin/localization --config_file=./conf/localization.json

# 或使用 launch
ros2 launch localization hdl_localization.launch.py
```

### 初始位姿设置

1. **RViz**: 点击 "2D Pose Estimate" 按钮设置初始位姿
2. **配置文件**: 设置 `specify_init_pose: true` 并填写 init_pos_* 参数

---

## 5. Output Formats

### Odometry Message

```yaml
header:
  stamp: 1234567890.123
  frame_id: "map"
child_frame_id: "base_link"
pose:
  pose:
    position: {x: 10.5, y: 3.2, z: 0.8}
    orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}
```

### TF

```
Transform: map → odom
  translation: [10.5, 3.2, 0.8]
  rotation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}
```

---

## 6. 集成到 Pipeline

```
LiDAR PointCloud
     ↓
[map] FAST-LIO-SAM (建图)
     ↓
保存 .pcd 地图
     ↓
[localization] HDL (定位)
     ↓
/odom (TF: map→odom) → 供其他模块使用
     ↓
Perception → Prediction → Planning → Control
```

### 工作流程

1. **建图阶段**:
   - 运行 `map` 模块
   - 采集点云数据
   - 调用 `/save_map` 服务保存 `.pcd` 文件

2. **定位阶段**:
   - 运行 `localization` 模块
   - 订阅 `/globalmap` 话题接收预先构建的地图
   - 通过 scan-matching 实时估计位姿

---

## 7. 常见问题排查

| 问题 | 原因 | 解决方案 |
|------|------|---------|
| 定位丢失 | 全局地图未加载 | 检查 /globalmap 话题 |
| 位姿抖动 | NDT 参数不当 | 调整 ndt_resolution |
| 预测漂移 | IMU 噪声大 | 校准 IMU 或关闭 IMU |
| 初始定位慢 | cool_time_duration 大 | 减小冷却时间 |
| 收敛慢 | downsample 太粗 | 减小 downsample_resolution |
