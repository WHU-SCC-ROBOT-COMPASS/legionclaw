---
name: legionclaw-prediction
description: Obstacle trajectory prediction. Read obstacle list and routing, predict multi-modal trajectories based on vector map and intent.
argument-hint: [input_bag] [output_dir] [prediction_config]
allowed-tools: Bash(ssh *), Bash(cd *), Bash(python3 *), Bash(rostopic *), Bash(rosbag *), Bash(ros2 *), Read, Write, Glob, Grep
---

# LegionClaw Prediction Module

Module: `modules/prediction/`

## 1. Message Flow

```
INPUT:
  - ObstacleList: 来自 motion_manager 的融合障碍物列表
  - ADCTrajectory: 自车轨迹规划
  - Location: 车辆定位
  - RoutingResponse: 路由结果
  - LaneList: 车道信息

OUTPUT:
  - PredictionObstacles: 障碍物预测结果 (多模态轨迹)
  - Faults: 故障状态
```

### 订阅/发布接口

| 消息类型 | 方向 | Topic | 格式 |
|---------|------|-------|------|
| ObstacleList | 订阅 | obstacle_list | legionclaw::interface::ObstacleList |
| ADCTrajectory | 订阅 | adc_trajectory | legionclaw::interface::ADCTrajectory |
| Location | 订阅 | location | legionclaw::interface::Location |
| RoutingResponse | 订阅 | routing_response | legionclaw::interface::RoutingResponse |
| LaneList | 订阅 | lane_list | legionclaw::interface::LaneList |
| PredictionObstacles | 发布 | prediction_obstacles | legionclaw::interface::PredictionObstacles |
| Faults | 发布 | faults | legionclaw::interface::Faults |
| ObuCmdMsg | 订阅 | obu_cmd | 功能激活/去激活控制 |

## 2. Core Algorithm Flow

```
┌─────────────────────────────────────────────────────────────────────┐
│  Init():                                                           │
│    1. Load PredictionConf from FLAGS_prediction_config_file         │
│    2. LOGGING_INIT()                                               │
│    3. MESSAGE_INIT()                                               │
│    4. vector_map_predictor_.Init(*prediction_conf_)               │
│    5. Start spin thread + 定时器                                      │
│    6. TaskActivate()                                               │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│  Task10ms() [on timer]:                                           │
│                                                                      │
│  1. 收集 LocalView:                                                │
│     - obstacle_list_ (障碍物)                                       │
│     - adc_trajectory_ (自车轨迹)                                    │
│     - location_ (定位)                                              │
│     - routing_response_ (路由)                                      │
│     - lane_list_ (车道)                                            │
│                                                                      │
│  2. vector_map_predictor_.Run():                                    │
│     → 基于矢量地图的轨迹预测                                          │
│                                                                      │
│  3. PublishPredictionObstacles()                                   │
└─────────────────────────────────────────────────────────────────────┘
```

### Vector Map Prediction 算法

```
VectorMapPredictor::Run():
  1. 地图加载与匹配
     → 基于障碍物位置匹配到矢量地图中的车道/道路元素

  2. 意图识别 (Intent Recognition)
     → LANE_FOLLOW: 沿车道行驶
     → LEFT_TURN: 左转
     → RIGHT_TURN: 右转
     → STOP: 停车
     → CROSS: 穿越

  3. 轨迹生成 (Multi-Modal)
     → 每个意图生成多条候选轨迹
     → 基于道路拓扑和交通规则约束
     → 轨迹点序列: (x, y, z, theta, velocity, acceleration, time)

  4. 概率评估
     → 计算每条轨迹的概率
     → 结合道路约束和历史运动模式

  5. 输出 PredictionObstacles
     → 每个障碍物多模态轨迹
     → 每条轨迹带概率
```

## 3. State Machine

```
┌──────────────────────────────────────────────────────────────────────┐
│  Module States:                                                       │
│                                                                      │
│  ┌──────────┐                                                       │
│  │  NONE    │──Init()──→ ┌────────────┐                           │
│  └──────────┘          │   INIT     │                           │
│                         └─────┬──────┘                           │
│                               │                                    │
│              TaskActivate() ───┼──→ TaskStop()                     │
│                               ↓                                    │
│                        ┌────────────┐                              │
│                        │   ACTIVE   │                              │
│                        │ INACTIVE   │                              │
│                        └────────────┘                              │
│                                                                      │
│  Inner State Flags:                                                 │
│    - location_init_: 定位是否初始化                                  │
│    - map_loaded_: 地图是否加载                                      │
│    - local_view_.localiztion_mode_: 0=INS, 1=Odometry              │
│    - local_view_.map_mode_: 0=HDMap, 1=LocalMap                    │
└──────────────────────────────────────────────────────────────────────┘
```

## 4. Parameter Configuration

### Config File: `modules/prediction/conf/prediction_conf.json`

```protobuf
message PredictionConf {
    int produce_prediction_command_duration = 100;  // 生产周期(ms)
    int publish_prediction_command_duration = 100;  // 发布周期(ms)
    StatusConf status = 1;
    
    // 预测相关参数
    double prediction_period = 5.0;        // 预测时长(s)
    double prediction_resolution = 0.1;    // 预测分辨率(s)
    int num_trajectory_samples = 6;         // 采样轨迹数
    double lateral_extent = 1.0;           // 横向范围
    double speed_scale_factor = 1.0;        // 速度缩放因子
    
    // 地图相关
    string vector_map_path = "./data/map/"; // 矢量地图路径
    bool enable_map_prediction = true;     // 启用地图预测
    
    // 障碍物类型相关
    repeated ObjectTypeConf object_type_conf = 1;
}

message ObjectTypeConf {
    int object_type = 0;
    double max_acceleration = 2.0;
    double max_deceleration = 4.0;
    double min_prediction_length = 10.0;
    double lane_change_probability = 0.2;
}
```

### 主要参数说明

| 参数 | 默认值 | 含义 | 调优建议 |
|------|--------|------|---------|
| prediction_period | 5.0 | 预测时长(s) | 高速场景增大 |
| prediction_resolution | 0.1 | 时间分辨率(s) | 计算精度权衡 |
| num_trajectory_samples | 6 | 每障碍物候选轨迹数 | 影响计算量 |
| lateral_extent | 1.0 | 横向搜索范围(m) | 车道变换场景 |
| speed_scale_factor | 1.0 | 速度缩放因子 | 保守/激进调整 |
| max_acceleration | 2.0 | 最大加速度(m/s²) | 车辆动力学约束 |
| max_deceleration | 4.0 | 最大减速度(m/s²) | 安全制动约束 |

## 5. Workflow - Run & Tune

### Step 1: 检查输入数据

```bash
# 检查所有预测相关话题
ros2 topic list | grep -E "(obstacle|trajectory|location|routing|prediction)"

# 检查地图数据
ls modules/prediction/data/map/

# 检查rosbag
ros2 bag play <input_bag> --remap \
    /obstacle_list:=/prediction/obstacle_list \
    /adc_trajectory:=/prediction/adc_trajectory \
    /location:=/prediction/location
```

### Step 2: 配置模块

```bash
# 编辑预测配置
nano modules/prediction/conf/prediction_conf.json

# 关键参数调整:
# - prediction_period: 城市低速5s, 高速10s
# - num_trajectory_samples: 复杂路口增加
# - lateral_extent: 匝道汇入场景增大
```

### Step 3: 构建和运行

```bash
cd modules/prediction
mkdir -p build && cd build
cmake .. && make -j$(nproc)

./bin/prediction --config_file=./conf/prediction_conf.json
```

### Step 4: 参数调优

```bash
# 场景1: 城市十字路口
# → 提高 num_trajectory_samples (6 → 10)
# → 增大 prediction_period (5.0 → 7.0)
# → 提高 lane_change_probability (0.2 → 0.4)

# 场景2: 高速公路
# → 增大 prediction_period (5.0 → 10.0)
# → 降低 lateral_extent (1.0 → 0.5)
# → 提高 speed_scale_factor (1.0 → 1.2)

# 场景3: 停车场
# → 减小 prediction_period (5.0 → 3.0)
# → 提高 num_trajectory_samples (6 → 12)
# → 启用 local_map 模式
```

### Step 5: 验证输出

```bash
# 监听预测结果
ros2 topic echo /prediction_obstacles --once

# 输出格式
{
    "prediction_obstacles": [
        {
            "obstacle_id": 123,
            "trajectories": [
                {
                    "probability": 0.7,
                    "trajectory_points": [
                        {"x": 10.5, "y": 3.2, "z": 0.0, "theta": 0.1, "velocity": 5.0, "time": 0.0},
                        {"x": 11.0, "y": 3.3, "z": 0.0, "theta": 0.1, "velocity": 5.2, "time": 0.1},
                        ...
                    ],
                    "intent": "LANE_FOLLOW"
                },
                ...
            ]
        },
        ...
    ]
}
```

## 6. 输出格式

```protobuf
message PredictionObstacles {
    Header header = 1;
    repeated PredictionObstacle prediction_obstacle = 2;
}

message PredictionObstacle {
    int32 obstacle_id = 1;
    repeated Trajectory trajectories = 2;
    double timestamp = 3;
}

message Trajectory {
    double probability = 1;
    repeated TrajectoryPoint trajectory_points = 2;
    TrajectoryType type = 3;  // LANE_FOLLOW, LEFT_TURN, RIGHT_TURN, etc.
}

message TrajectoryPoint {
    double x = 1;
    double y = 2;
    double z = 3;
    double theta = 4;
    double velocity = 5;
    double acceleration = 6;
    double relative_time = 7;
}
```
