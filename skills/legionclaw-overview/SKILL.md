---
name: legionclaw-overview
description: Comprehensive overview of the LegionClaw autonomous driving stack. Understand message flows, algorithms, parameters, and how to run/tune each module.
argument-hint: [module_name or 'all']
allowed-tools: Bash(ls *), Bash(find *), Bash(python3 *), Read, Glob, Grep
---

# LegionClaw 自主驾驶系统全面总结

LegionClaw 是一个统一的自主驾驶代码仓库，集成 **感知 (Perception)** 和 **PNC (Planning and Control)** 功能。项目主要使用 **C++** 编写，支持 ROS2/ROS/DDS/LCM 多种通信中间件。

> **⚡ 新增模块：** 本框架现已集成 LiDAR SLAM 系统。
> - **`map`** — 基于 **FAST-LIO-SAM** 的实时建图模块（LiDAR-IMU 紧耦合 + iESKF + iKD-Tree）
> - **`localization`** — 基于 **HDL Localization** 的定位模块（NDT Scan Matching + UKF 姿态估计）
> - 详见下方新增模块章节或使用 `skill("legionclaw-map")` / `skill("legionclaw-localization")`

---

## 整体架构

```
┌─────────────────────────────────────────────────────────────────────────────────────┐
│                           LegionClaw 自主驾驶系统                                    │
├─────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                     │
│  ┌───────────────────────────────────────────────────────────────────────────────┐ │
│  │                           感知模块 (Perception)                                │ │
│  │  ┌──────────────────┐   ┌──────────────────────┐   ┌────────────────────────┐ │ │
│  │  │ LiDAR Sensors   │   │  lidar_detect_voxel │   │   motion_manager       │ │ │
│  │  │ (5个激光雷达)   │ → │  (CenterPoint网络)   │ → │   (融合跟踪)           │ │ │
│  │  │ 前/中/左/右/后  │   │  深度学习3D检测       │   │   IOU融合+Kalman滤波   │ │ │
│  │  └──────────────────┘   └──────────────────────┘   └───────────┬────────────┘ │ │
│  │          ↓                                                          ↓          │ │
│  │  ┌──────────────────┐                                    ┌──────────────────┐ │ │
│  │  │ lidar_ground_    │                                    │   prediction      │ │ │
│  │  │ segmentation     │                                    │   (轨迹预测)       │ │ │
│  │  │ (PatchWork++)    │                                    │   多模态意图预测   │ │ │
│  │  └────────┬─────────┘                                    └──────────────────┘ │ │
│  │           ↓                                                                  │ │
│  │  ┌──────────────────┐                                                          │ │
│  │  │ lidar_cluster_   │                                                          │ │
│  │  │ detect           │                                                          │ │
│  │  │ (网格聚类)       │                                                          │ │
│  │  └────────┬─────────┘                                                          │ │
│  └────────────┼────────────────────────────────────────────────────────────────────┘ │
│               ↓                                                                       │
│  ┌───────────────────────────────────────────────────────────────────────────────┐ │
│  │                           SLAM / Localization                                  │ │
│  └───────────────────────────────────────────────────────────────────────────────┘ │
│               ↓                                                                       │
│  ┌───────────────────────────────────────────────────────────────────────────────┐ │
│  │                           规划模块 (Planning)                                 │ │
│  │  ┌──────────────────┐   ┌──────────────────────┐                             │ │
│  │  │ ReferenceLine    │ → │  Lattice Planner     │                             │ │
│  │  │ Provider         │   │  (采样+优化)          │                             │ │
│  │  └──────────────────┘   └──────────┬───────────┘                             │ │
│  │                                   ↓                                           │ │
│  │                      ┌──────────────────────────┐                           │ │
│  │                      │ Parking Planner          │                           │ │
│  │                      │ (泊车路径规划)            │                           │ │
│  │                      └──────────────────────────┘                           │ │
│  └───────────────────────────────────────────────────────────────────────────────┘ │
│               ↓                                                                       │
│  ┌───────────────────────────────────────────────────────────────────────────────┐ │
│  │                           控制模块 (Control)                                  │ │
│  │  ┌──────────────────┐   ┌──────────────────────┐   ┌────────────────────────┐ │ │
│  │  │ Longitudinal     │   │  Lateral Controller  │   │   Controller Agent    │ │ │
│  │  │ Controller       │ + │  (LQR)               │ = │   (控制命令)            │ │ │
│  │  │ (PID/MRAC)       │   │                      │   │                        │ │ │
│  │  └──────────────────┘   └──────────────────────┘   └───────────┬────────────┘ │ │
│  └───────────────────────────────────────────────────────────────────────────────┘ │
│               ↓                                                                       │
│  ┌───────────────────────────────────────────────────────────────────────────────┐ │
│  │                           路由模块 (Routing)                                  │ │
│  │  ┌──────────────────┐   ┌──────────────────────┐                             │ │
│  │  │ Lanelet2 HD Map  │ → │  Routing Graph      │ → RoutingResponse            │ │
│  │  │ (高精地图)        │   │  (最短路径)          │                             │ │
│  │  └──────────────────┘   └──────────────────────┘                             │ │
│  └───────────────────────────────────────────────────────────────────────────────┘ │
│                                                                                     │
└─────────────────────────────────────────────────────────────────────────────────────┘
```

---

## 模块 1: LiDAR Ground Segmentation (地面分割)

**路径**: `modules/perception/lidar/lidar_ground_segmentation/`

### 消息流

```
输入:
  PointCloud → Eigen::MatrixXf (x,y,z,intensity)

输出:
  GroundPoints → Eigen::MatrixX3f (地面点)
  NoGroundPoints → Eigen::MatrixX3f (非地面点/障碍物点)
  Faults → 故障状态
```

### 核心算法

**PatchWork++** - 基于区域生长的地面分割:

1. **CZM (同心区域模型)**: 将点云分为近/中/远区域
2. **种子点选择**: 找 z 值最低的点作为种子 (LPR)
3. **区域生长**: 基于高度差阈值扩展地面点
4. **平整度检查**: 评估区域平坦度
5. **RNR 去噪**: 去除反射点噪声

### 关键参数

| 参数 | 默认值 | 含义 | 调优 |
|------|--------|------|------|
| th_seeds | 0.15m | 种子点高度阈值 | 调整地面/非地面边界 |
| th_dist | 0.25m | 生长距离阈值 | 控制地面平滑度 |
| sensor_height | 1.5m | LiDAR 安装高度 | 校准安装位置 |
| enable_RNR | true | 启用去噪 | 复杂环境开启 |

### State Machine

```
NONE → INIT → INACTIVE ←→ ACTIVE
                    ↓
              TaskActivate/TaskStop
```

---

## 模块 2: LiDAR Cluster Detection (聚类检测)

**路径**: `modules/perception/lidar/lidar_cluster_detect/`

### 消息流

```
输入:
  PointCloud → legionclaw::interface::PointCloud

输出:
  ObstacleList → {id, type, position, dimensions, polygon}
  ClusterPointCloud → 聚类点云(可视化)
```

### 核心算法

**Grid-Based Clustering**:

```
1. BuildCellMap: 建立 col_size × row_size 网格
2. ClusterConnectedPoints: 8连通域搜索
3. BuildCluster: 计算包围盒和质心
4. ConvertClustersToObstacleList: 转换为标准格式
```

### 关键参数

| 参数 | 默认值 | 含义 | 调优 |
|------|--------|------|------|
| cell_width | 0.4m | 网格宽度 | 影响聚类精度 |
| cell_length | 0.8m | 网格长度 | 影响聚类精度 |
| point_min_size_for_cell | 5 | 最小有效网格点数 | 过滤噪声 |
| point_min_size_for_box | 10 | 最小障碍物点数 | 过滤小目标 |
| obstacle_min_height | 0.05m | 最小障碍物高度 | 过滤低矮目标 |

---

## 模块 3: LiDAR Voxel Detection (深度学习检测)

**路径**: `modules/perception/lidar/lidar_detect_voxel/`

### 消息流

```
输入:
  PointCloud2 → ROS2 sensor_msgs

输出:
  ObstacleList → {id, type, position, velocity, dimensions}
```

### 核心算法

**CenterPoint (VoxelNet + RPN)**:

```
1. Voxelization: 点云 → (D,H,W) 稀疏网格
2. SCN Backbone: 稀疏卷积网络提取特征
3. RPN Head: 热力图 + 边界框回归
4. NMS: 非极大值抑制后处理
```

### 关键参数

| 参数 | 默认值 | 含义 | 调优 |
|------|--------|------|------|
| general_threshold | 0.5 | 置信度阈值 | 提高减少误检 |
| big_vehicle_threshold | 0.4 | 大车阈值 | 低于通用阈值 |
| unknow_threshold | 0.6 | 未知类型阈值 | 提高减少误分类 |

### 检测类别

| Type ID | 类别 |
|---------|------|
| 0 | UNKNOWN |
| 1 | CAR |
| 2 | TRUCK |
| 3 | BUS |
| 4 | PEDESTRIAN |
| 5 | BICYCLE |
| 6 | MOTORCYCLE |

---

## 模块 4: Motion Manager Fusion (融合跟踪)

**路径**: `modules/perception/fusion/motion_manager/`

### 消息流

```
输入:
  ObstacleList (lidar_cluster) → 聚类结果
  ObstacleList (voxel_detect) → 深度学习检测
  Location → 定位信息

输出:
  ObstacleListOutput → 融合跟踪后的障碍物列表
```

### 核心算法

```
1. VehicleToWorld: 坐标转换 (车体→世界)
2. TransObstacle: 格式转换
3. MultiObjectTracker:
   - Predict: Kalman 预测
   - Associate: GNN 匈牙利算法数据关联
   - Update: Kalman 更新
4. FuseObstacleLists: IOU 融合两个数据源
5. StabilizeFusionIds: ID 稳定化
```

### 关键融合参数

| 参数 | 默认值 | 含义 | 调优 |
|------|--------|------|------|
| iou_threshold | 0.01 | IOU 匹配阈值 | 太小误匹配,太大漏匹配 |
| data_timeout_ms | 3000 | 数据超时阈值 | 根据频率调整 |
| grid_size | 0.1m | 位置网格精度 | 影响 ID 稳定 |
| history_match_max_distance | 5.0m | 历史匹配距离 | 快速目标增大 |
| lcd_id_offset | 10000 | LCD ID 偏移 | 区分数据源 |

---

## 模块 5: Prediction (轨迹预测)

**路径**: `modules/prediction/`

### 消息流

```
输入:
  ObstacleList → 融合障碍物
  ADCTrajectory → 自车轨迹
  Location → 定位
  RoutingResponse → 路由
  LaneList → 车道

输出:
  PredictionObstacles → 多模态轨迹预测
```

### 核心算法

**Vector Map Prediction**:

```
1. 地图匹配: 障碍物位置 → 矢量地图
2. 意图识别: LANE_FOLLOW / LEFT_TURN / RIGHT_TURN / STOP
3. 轨迹生成: 每意图多条候选轨迹
4. 概率评估: 基于道路约束和历史运动
```

### 关键参数

| 参数 | 默认值 | 含义 |
|------|--------|------|
| prediction_period | 5.0s | 预测时长 |
| prediction_resolution | 0.1s | 时间分辨率 |
| num_trajectory_samples | 6 | 采样轨迹数 |
| lateral_extent | 1.0m | 横向搜索范围 |

---

## 模块 6: Planning (轨迹规划)

**路径**: `modules/planning/`

### 消息流

```
输入:
  ObstacleList → 感知障碍物
  PredictionObstacles → 预测轨迹
  RoutingResponse → 路由
  Location → 定位
  Chassis → 底盘

输出:
  ADCTrajectory → 规划轨迹
```

### 核心算法

**Lattice Planner**:

```
1. 目标点计算: 基于参考线和路由
2. 轨迹采样: 采样 (s, v, a) 组合
3. 代价评估: safety + comfort + efficiency
4. 最优选择: 代价最低轨迹
```

### State Machine

```
DRIVING_OUT ← DRIVING_IN ←→ DRIVING_CHANGE_LANE / DRIVING_EMERGENCY
     ↓
PARKING_IN → PARKING_OUT
```

### 关键参数

| 参数 | 默认值 | 含义 |
|------|--------|------|
| sample_distance | 60m | 规划距离 |
| safety_weight | 1.0 | 安全代价权重 |
| comfort_weight | 0.5 | 舒适代价权重 |
| obstacle_buffer | 0.5m | 安全缓冲 |
| max_curvature | 0.25 | 最大曲率 |

---

## 模块 7: Control (控制)

**路径**: `modules/control/`

### 消息流

```
输入:
  ADCTrajectory → 规划轨迹
  Chassis → 底盘状态
  Location → 定位

输出:
  ControlCommand → {throttle, steer, gear}
```

### 核心算法

**纵向控制 (PID/MRAC)**:
```
speed_error = v_ref - v_cur
throttle = Kp*error + Ki*∫error + Kd*d(error)/dt
```

**横向控制 (LQR)**:
```
状态: X = [横向误差, 航向误差, 积分误差]
控制: δ = -K*X (LQR 反馈增益)
```

### 关键参数

| 参数 | 默认值 | 含义 | 调优 |
|------|--------|------|------|
| lon.kp | 0.5 | 纵向 P 参数 | 响应速度 |
| lon.ki | 0.1 | 纵向 I 参数 | 消除稳态误差 |
| lqr.q0 | 1.0 | 横向误差权重 | 路径跟踪精度 |
| lqr.q1 | 2.0 | 航向误差权重 | 方向稳定性 |
| steer_ratio | 13.1 | 转向比 | 车型参数 |
| max_steer_angle | 6.85° | 最大转角 | 机械限制 |

---

## 模块 8: Routing (路由)

**路径**: `modules/routing/`

### 消息流

```
输入:
  RoutingRequest → 起点/终点/方式
  Location → 当前位置
  ParkingInfo → 泊车位 (AVP)

输出:
  RoutingResponse → 路径/车道序列
  LaneList → 车道信息
  GuideInfo → 导航信息
  NaviInfo → 导航状态
```

### 核心算法

**Lanelet2 Routing**:

```
1. 加载 OSM 格式地图
2. 构建 RoutingGraph
3. shortestPath() → 最短路径
4. 解析路由: 提取车道/限速/停车点
```

### 关键参数

| 参数 | 默认值 | 含义 |
|------|--------|------|
| loop_rate | 100ms | 主循环周期 |
| default_speed_limit | 30km/h | 默认限速 |
| intersection_speed_limit | 15km/h | 路口限速 |
| parking_space_search_radius | 50m | 泊车搜索半径 |

---

## 完整数据流

```
[传感器输入]
     │
     ├─→ LiDAR PointCloud ──────────────────────────┐
     │                                              ↓
     │                            ┌─────────────────────────────┐
     │                            │ lidar_ground_segmentation   │
     │                            │ (PatchWork++ 地面分割)       │
     │                            └────────────┬────────────────┘
     │                                         ↓
     │                            ┌─────────────────────────────┐
     │                            │ lidar_cluster_detect        │
     │                            │ (网格聚类)                   │
     │                            └────────────┬────────────────┘
     │                                         ↓
     ├─────────────────────────────────────────┼───────────────────┐
     │                                         ↓                   │
     │                            ┌─────────────────────────────┐   │
     │                            │ lidar_detect_voxel         │   │
     │                            │ (CenterPoint 深度学习)       │   │
     │                            └────────────┬────────────────┘   │
     │                                         ↓                   │
     │                            ┌─────────────────────────────┐   │
     │                            │ motion_manager              │   │
     │                            │ (融合 + 跟踪 + ID稳定化)     │   │
     │                            └────────────┬────────────────┘   │
     │                                         ↓                   │
     │                            ┌─────────────────────────────┐   │
     │                            │ prediction                 │   │
     │                            │ (多模态轨迹预测)            │   │
     │                            └────────────┬────────────────┘   │
     │                                         ↓                   │
     │  ┌──────────────────┐                   ↓                   │
     │  │ routing          │       ┌─────────────────────────────┐ │
     │  │ (Lanelet2)      │ ────→ │ planning                   │ │
     │  └──────────────────┘       │ (Lattice + Parking)         │ │
     │                             └────────────┬────────────────┘ │
     │                                          ↓                   │
     │                             ┌─────────────────────────────┐ │
     │                             │ control                    │ │
     │                             │ (纵向LQR + 横向PID)         │ │
     │                             └────────────┬────────────────┘ │
     │                                          ↓                   │
     │                             ┌─────────────────────────────┐ │
     │                             │ CAN Bus → 车辆执行           │ │
     │                             └─────────────────────────────┘ │
     │                                                             │
     └─────────────────────────────────────────────────────────────┘
```

---

## 配置管理系统

### 配置文件位置

```
modules/
├── perception/
│   ├── lidar/
│   │   ├── lidar_ground_segmentation/conf/
│   │   │   ├── config.json
│   │   │   └── patchworkpp_params.json
│   │   ├── lidar_cluster_detect/conf/
│   │   │   ├── config.json
│   │   │   └── cluster_params.json
│   │   └── lidar_detect_voxel/conf/
│   │       └── perception/lidar/lidar_detect/lidar_detect.json
│   └── fusion/
│       └── motion_manager/conf/
│           ├── config.json
│           └── multi_object_tracker.json
├── prediction/conf/
│   └── prediction_conf.json
├── planning/conf/
│   ├── planning_conf.json
│   ├── lattice_planner_conf.json
│   └── parking_conf.json
├── control/conf/
│   ├── control_conf.json
│   ├── lon_calibration_table.json
│   └── lqr_calibration_table.json
├── routing/conf/
│   └── routing_conf.json
└── common/
    ├── data/vehicle_param/
    │   ├── vehicle_param.json
    │   ├── vehicle_param_igv.json
    │   ├── vehicle_param_ehs9.json
    │   └── vehicle_param_sim.json
    └── integration_service/
        ├── integration_service.yaml
        ├── ros1_2_dds.yaml
        └── dds_2_ros1.yaml
```

### 统一状态机模式

所有模块遵循统一的初始化和状态管理流程:

```
Init():
  1. is_init_ = false
  2. VariableInit()
  3. 加载配置文件 (JSON → protobuf)
  4. LOGGING_INIT()
  5. MESSAGE_INIT()
  6. 算法初始化
  7. 定时器初始化
  8. is_init_ = true
  9. TaskActivate()

TaskActivate():
  - MessagesActivate()
  - 启动定时器
  - function_activation_ = true

TaskStop():
  - MessagesDeActivate()
  - 停止定时器
  - function_activation_ = false

OBU Command (code=10086):
  - DEACTIVATE_BOTH → TaskStop()
  - ACTIVATE_DRIVING → TaskActivate()
  - DEACTIVATE_DRIVING → TaskStop()
```

---

## 多中间件支持

所有模块支持多种通信中间件，通过配置切换:

```json
"message": {
    "active_message": [4],
    "message_info": [
        {"type": 0, "name": "LCM", "url": "udpm://239.255.76.21:7621?ttl=3"},
        {"type": 1, "name": "DDS", "url": "null"},
        {"type": 2, "name": "ROS", "url": "null"},
        {"type": 3, "name": "ADSFI", "url": "null"},
        {"type": 4, "name": "ROS2", "url": "null"}
    ]
}
```

---

## 运行和调优

### 快速启动

```bash
cd /home/lenovo/project/legionclaw

# 启动所有感知节点
bash launch/launch_all.sh

# 或者单独启动
./modules/perception/lidar/lidar_ground_segmentation/bin/lidar_ground_segmentation \
    --config_file=./modules/perception/lidar/lidar_ground_segmentation/conf/config.json
```

### 参数调优工作流

```bash
# 1. 编辑配置
nano <module>/conf/config.json

# 2. 重新构建
cd <module>/build && make -j$(nproc)

# 3. 重新运行
./bin/<module> --config_file=./conf/config.json

# 4. 验证输出
ros2 topic echo /<output_topic> --once

# 5. 如需要,导出结果
python3 export_results.py --output result.json --params <used_params.json>
```

### 常见问题排查

| 模块 | 问题 | 原因 | 解决方案 |
|------|------|------|---------|
| ground_segmentation | 点被误分类 | th_seeds 太高 | 降低阈值 |
| cluster_detect | 障碍物过碎 | cell 尺寸太大 | 减小 cell_width/length |
| voxel_detect | 漏检 | 阈值太高 | 降低 general_threshold |
| motion_manager | ID 跳变 | grid_size 太小 | 减小 grid_size |
| planning | 轨迹不平滑 | q1 权重太低 | 提高航向误差权重 |
| control | 转向振荡 | kp/LQR q0 太高 | 降低参数 |
| routing | 路径规划失败 | 地图缺失 | 检查 map_file 路径 |
