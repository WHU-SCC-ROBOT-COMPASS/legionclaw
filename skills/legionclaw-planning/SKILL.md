---
name: legionclaw-planning
description: Trajectory planning with lattice planner and parking planner. Read perception/routing/location, generate trajectories and tune parameters.
argument-hint: [input_bag] [output_dir] [planning_config]
allowed-tools: Bash(ssh *), Bash(cd *), Bash(python3 *), Bash(rostopic *), Bash(rosbag *), Bash(ros2 *), Read, Write, Glob, Grep
---

# LegionClaw Planning Module

Module: `modules/planning/`

## 1. Message Flow

```
INPUT:
  - ObstacleList: 感知障碍物 (来自 motion_manager)
  - PredictionObstacles: 预测轨迹 (来自 prediction)
  - RoutingResponse: 路由结果 (来自 routing)
  - Location: 车辆定位
  - Chassis: 车辆底盘状态
  - ADCTrajectory: 上一周期轨迹 (反馈)

OUTPUT:
  - ADCTrajectory: 规划轨迹
  - PlanningData: 内部规划数据 (调试)
  - Faults: 故障状态
```

### 订阅/发布接口

| 消息类型 | 方向 | Topic | 格式 |
|---------|------|-------|------|
| ObstacleList | 订阅 | obstacle_list | legionclaw::interface::ObstacleList |
| PredictionObstacles | 订阅 | prediction_obstacles | legionclaw::interface::PredictionObstacles |
| RoutingResponse | 订阅 | routing_response | legionclaw::interface::RoutingResponse |
| Location | 订阅 | location | legionclaw::interface::Location |
| Chassis | 订阅 | chassis | legionclaw::interface::Chassis |
| ADCTrajectory | 订阅/发布 | adc_trajectory | legionclaw::interface::ADCTrajectory |
| Faults | 发布 | faults | legionclaw::interface::Faults |
| ObuCmdMsg | 订阅 | obu_cmd | 功能激活/去激活控制 |

## 2. Core Algorithm Flow

```
┌─────────────────────────────────────────────────────────────────────┐
│  Init():                                                           │
│    1. Load PlanningConf from protobuf (JSON → pb)                   │
│    2. Load LatticePlannerConf                                       │
│    3. Load ParkingConf                                              │
│    4. Load VehicleParam                                             │
│    5. LOGGING_INIT()                                                │
│    6. MESSAGE_INIT()                                                │
│    7. ReferenceLineProvider::Start() → 参考线提供者                   │
│    8. frame_.Init(planning_conf_) → 规划框架初始化                    │
│    9. lattice_planner_.Init() → Lattice规划器初始化                   │
│   10. parking_planner_.Init() → 泊车规划器初始化                       │
│   11. TaskActivate()                                               │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│  ComputePlanningCommandOnTimer() [10Hz]:                          │
│                                                                      │
│  ┌─ [Step 1] 状态判断                                               │
│  │   driving_flag_: DRIVING/PARKING                               │
│  │   planning_flag_: PLANNING_VALID/INVALID                        │
│  │                                                                  │
│  ├─ [Step 2] PlanningStateMachine                                  │
│  │   → 根据 driving_flag_ 选择规划器                                │
│  │                                                                  │
│  ├─ [Step 3] Frame 处理                                            │
│  │   frame_.Update()                                              │
│  │   → 合并障碍物, 参考线, 路由等                                    │
│  │                                                                  │
│  ├─ [Step 4] Lattice Planning (行车)                               │
│  │   lattice_planner_.Plan()                                      │
│  │   → 基于采样+优化的轨迹规划                                        │
│  │                                                                  │
│  ├─ [Step 5] Parking Planning (泊车)                               │
│  │   parking_planner_.Plan()                                      │
│  │   → 泊车路径规划                                                  │
│  │                                                                  │
│  └─ [Step 6] 轨迹发布                                              │
│      PublishPlanningTrajectory()                                   │
└─────────────────────────────────────────────────────────────────────┘
```

### Lattice Planner 算法

```
LatticePlanner::Plan():
  1. 目标点计算
     → 基于参考线和Routing确定终点

  2. 轨迹采样 (Trajectory Sampling)
     → 采样不同 (s, v, a) 组合
     → 生成多候选轨迹
     → 满足车辆动力学约束

  3. 轨迹评估 (Trajectory Evaluation)
     → 安全性: 与障碍物/预测轨迹的距离
     → 舒适性: 曲率, 加速度, jerk
     → 效率性: 到达时间, 路径长度
     → 代价函数: cost = w1*safety + w2*comfort + w3*efficiency

  4. 最优轨迹选择
     → 选择cost最低的轨迹
     → 平滑度后处理

  5. 输出 TrajectoryPoint 序列
```

## 3. State Machine

```
┌──────────────────────────────────────────────────────────────────────┐
│  Driving State Machine (DrivingFlag):                                │
│                                                                      │
│   ┌──────────┐                                                      │
│   │   NONE   │──Init()──→ ┌─────────────┐                         │
│   └──────────┘          │ DRIVING_INIT │                         │
│                         └──────┬───────┘                         │
│                                │                                   │
│   ┌──────────────┐  TaskActivate()  ┌────────────────────┐        │
│   │ DRIVING_OUT  │ ←──────────────── │ DRIVING_IN         │        │
│   │ (车道巡航)    │                   │ (车道保持/跟车)      │        │
│   └──────┬───────┘                   └──────┬─────────────┘        │
│          │                                 │                       │
│          │           ┌─────────────────────┼─────────────────┐    │
│          │           ↓                     ↓                   │    │
│          │    ┌────────────┐        ┌─────────────┐           │    │
│          │    │ DRIVING_   │        │ DRIVING_   │           │    │
│          │    │ CHANGE_LANE│        │ EMERGENCY  │           │    │
│          │    └────────────┘        └─────────────┘           │    │
│          │                                 │                   │    │
│          └──────────────→ │ PARKING_IN │ ←───────────────────┘    │
│                           │ (泊车入位)  │                          │
│                           └──────┬─────┘                          │
│                                  │                                  │
│                           ┌──────▼─────┐                          │
│                           │ PARKING_OUT│                          │
│                           │ (泊出)     │                          │
│                           └────────────┘                          │
│                                                                      │
│  Planning State Machine (PlanningFlag):                             │
│   PLANNING_INVALID → PLANNING_OK → PLANNING_FALLBACK → ...        │
└──────────────────────────────────────────────────────────────────────┘
```

## 4. Parameter Configuration

### Main Config: `modules/planning/conf/planning_conf.json`

```protobuf
message PlanningConf {
    int produce_planning_command_duration = 100;  // ms
    int publish_planning_command_duration = 100;  // ms
    
    // Lattice Planner Config
    LatticePlannerConf lattice_planner_conf = 1;
    
    // Parking Planner Config
    ParkingConf parking_conf = 2;
    
    // Vehicle Parameters
    VehicleParam vehicle_param = 3;
}
```

### Lattice Planner Config: `modules/planning/conf/lattice_planner_conf.json`

```protobuf
message LatticePlannerConf {
    // 采样参数
    double sample_distance = 60.0;        // 采样距离(m)
    double lateral_sample_range = 1.5;    // 横向采样范围(m)
    double lateral_sample_resolution = 0.2;  // 横向分辨率(m)
    int sample_speed_num = 5;            // 速度采样数
    double min_sample_speed = 0.0;        // 最小速度(m/s)
    double max_sample_speed = 20.0;       // 最大速度(m/s)
    
    // 代价函数权重
    double safety_weight = 1.0;           // 安全代价权重
    double comfort_weight = 0.5;          // 舒适代价权重
    double efficiency_weight = 0.3;        // 效率代价权重
    double smoothness_weight = 0.2;       // 平滑代价权重
    
    // 约束参数
    double max_acceleration = 3.0;       // 最大加速度(m/s²)
    double max_deceleration = 6.0;       // 最大减速度(m/s²)
    double max_curvature = 0.25;          // 最大曲率(1/m)
    double max_curvature_rate = 0.5;      // 最大曲率变化率
    
    // 安全参数
    double obstacle_buffer = 0.5;         // 障碍物安全缓冲(m)
    double prediction_buffer = 1.0;       // 预测轨迹缓冲(m)
}
```

### Parking Planner Config: `modules/planning/conf/parking_conf.json`

```protobuf
message ParkingConf {
    double parking_speed = 0.5;           // 泊车速度(m/s)
    double parking_lookahead = 2.0;      // 前视距离(m)
    double min_parking_space_length = 6.0;  // 最小泊车空间长度(m)
    double min_parking_space_width = 3.0;   // 最小泊车空间宽度(m)
    double parking_success_threshold = 0.5;  // 成功阈值(m)
    
    // AVP 相关
    bool enable_avp = true;
    double avp_speed = 2.0;
}
```

### 关键参数说明

| 参数 | 默认值 | 含义 | 调优建议 |
|------|--------|------|---------|
| sample_distance | 60.0 | 规划距离(m) | 高速增大 |
| lateral_sample_range | 1.5 | 换道范围(m) | 车道宽度相关 |
| safety_weight | 1.0 | 安全权重 | 保守调高 |
| comfort_weight | 0.5 | 舒适权重 | 舒适调高 |
| obstacle_buffer | 0.5 | 安全缓冲(m) | 复杂环境增大 |
| max_curvature | 0.25 | 最大曲率 | 车辆极限限制 |
| parking_speed | 0.5 | 泊车速度 | 低速安全 |

## 5. Workflow - Run & Tune

### Step 1: 检查输入

```bash
# 检查规划话题
ros2 topic list | grep -E "(planning|trajectory|routing|location|obstacle)"

# 检查ROS bag
ros2 bag play <input_bag> --remap \
    /obstacle_list:=/planning/obstacle_list \
    /routing_response:=/planning/routing_response \
    /location:=/planning/location
```

### Step 2: 配置模块

```bash
# 编辑规划配置
nano modules/planning/conf/planning_conf.json
nano modules/planning/conf/lattice_planner_conf.json
nano modules/planning/conf/parking_conf.json
```

### Step 3: 构建和运行

```bash
cd modules/planning
mkdir -p build && cd build
cmake .. && make -j$(nproc)

./bin/planning --config_file=./conf/planning_conf.json
```

### Step 4: 参数调优

```bash
# 场景1: 高速公路 (高速+安全)
# → 增大 sample_distance (60 → 100)
# → 提高 safety_weight (1.0 → 2.0)
# → 降低 comfort_weight (0.5 → 0.3)
# → 减小 obstacle_buffer (0.5 → 0.3)

# 场景2: 城市道路 (低速+舒适)
# → 降低 max_curvature (0.25 → 0.2)
# → 提高 comfort_weight (0.5 → 1.0)
# → 提高 efficiency_weight (0.3 → 0.5)

# 场景3: 停车场泊车
# → 降低 parking_speed (0.5 → 0.3)
# → 增大 min_parking_space_length (6.0 → 6.5)
```

## 6. 输出格式

```protobuf
message ADCTrajectory {
    Header header = 1;
    string name = 2;
    int32 priority = 3;
    repeated TrajectoryPoint trajectory_point = 4;
    double total_length = 5;
    double total_time = 6;
    TrajectoryType type = 7;  // NORMAL, EMERGENCY, PARKING
}

message TrajectoryPoint {
    PathPoint path_point = 1;
    double v = 2;       // velocity (m/s)
    double a = 3;       // acceleration (m/s²)
    double relative_time = 4;  // relative time (s)
    double da = 5;      // jerk (m/s³)
}

message PathPoint {
    double x = 1;
    double y = 2;
    double z = 3;
    double theta = 4;
    double kappa = 5;    // curvature
    double s = 6;       // cumulative distance
}
```
