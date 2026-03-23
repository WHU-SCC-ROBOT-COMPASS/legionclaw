---
name: legionclaw-control
description: Vehicle control with LQR lateral and PID/MRAC longitudinal controllers. Read trajectory/chassis/location, output control commands and tune parameters.
argument-hint: [input_bag] [output_dir] [control_config]
allowed-tools: Bash(ssh *), Bash(cd *), Bash(python3 *), Bash(rostopic *), Bash(rosbag *), Bash(ros2 *), Read, Write, Glob, Grep
---

# LegionClaw Control Module

Module: `modules/control/`

## 1. Message Flow

```
INPUT:
  - ADCTrajectory: 规划轨迹 (来自 planning)
  - Chassis: 底盘状态 (来自 CAN 总线)
  - Location: 定位信息

OUTPUT:
  - ControlCommand: 控制命令
    → throttle_cmd: 油门/刹车命令
    → steer_cmd: 转向命令
    → gear_cmd: 档位命令
  - Faults: 故障状态
```

### 订阅/发布接口

| 消息类型 | 方向 | Topic | 格式 |
|---------|------|-------|------|
| ADCTrajectory | 订阅 | adc_trajectory | legionclaw::interface::ADCTrajectory |
| Chassis | 订阅 | chassis | legionclaw::interface::Chassis |
| Location | 订阅 | location | legionclaw::interface::Location |
| ControlCommand | 发布 | control_command | legionclaw::interface::ControlCommand |
| Faults | 发布 | faults | legionclaw::interface::Faults |
| ObuCmdMsg | 订阅 | obu_cmd | 功能激活/去激活控制 |

## 2. Core Algorithm Flow

```
┌─────────────────────────────────────────────────────────────────────┐
│  Init():                                                           │
│    1. Load ControlConf from protobuf (JSON → pb)                   │
│    2. Load LonCalibrationTable (纵向标定表)                         │
│    3. Load StopDistanceCalibrationTable                             │
│    4. Load LqrCalibrationTable (LQR标定表)                        │
│    5. Load VehicleParam                                            │
│    6. LOGGING_INIT()                                               │
│    7. MESSAGE_INIT()                                                │
│    8. controller_agent_.Init(injector_, control_conf_)            │
│    9. Start spin thread + Timer                                    │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│  ComputeControlCommandOnTimer() [10Hz]:                           │
│                                                                      │
│  ┌─ [Step 1] 轨迹切换检查                                          │
│  │   轨迹更新 → 重置 trajectory_switch_index_                      │
│  │                                                                  │
│  ├─ [Step 2] 纵向控制 (LonController)                             │
│  │   ComputeLongitudinalControl()                                  │
│  │   → PID/MRAC 控制 + 标定表                                      │
│  │   → 输出 throttle_cmd (加速) / brake_cmd (制动)                │
│  │                                                                  │
│  ├─ [Step 3] 横向控制 (LatController)                             │
│  │   LQR Lateral Control                                          │
│  │   → 计算横向误差、航向误差                                        │
│  │   → LQR求解 → 前轮转角命令                                      │
│  │   → Pure Pursuit 备选                                           │
│  │                                                                  │
│  ├─ [Step 4] Shared Autonomy (可选)                               │
│  │   人类驾驶员介入检测                                             │
│  │   → 方向盘转角/扭矩检测                                          │
│  │   → 人类介入时降低控制权重                                        │
│  │                                                                  │
│  ├─ [Step 5] 控制命令组装                                          │
│  │   ControlCommand = {throttle, steer, gear, ...}                │
│  │                                                                  │
│  └─ [Step 6] 发布控制命令                                          │
│      PublishControlCommand()                                       │
└─────────────────────────────────────────────────────────────────────┘
```

### 纵向控制器 (Longitudinal Controller)

```
ComputeLongitudinalControl():
  1. 获取当前状态
     - 当前速度 v_cur (来自 Chassis)
     - 目标速度 v_ref (来自 TrajectoryPoint)
     - 目标加速度 a_ref

  2. 速度误差计算
     - speed_error = v_ref - v_cur

  3. PID 控制
     - throttle = Kp * error + Ki * ∫error + Kd * d(error)/dt
     - 或使用 Lead-Lag 控制器

  4. 标定表查询
     - 纵向标定表: (v_ref, a_ref) → throttle_value
     - 停车标定表: (v, distance) → brake_value

  5. 档位判断
     - 基于速度和加速度判断 D/R/N 档
```

### 横向控制器 (LQR Lateral Controller)

```
LQR Lateral Control():
  1. 误差计算 (在 Frenet 坐标系)
     - 横向误差 e1: 车辆到轨迹的横向距离
     - 航向误差 e2: 车辆航向与轨迹切向夹角

  2. LQR 求解
     - 状态向量 X = [e1, e2, ∫e1]ᵀ
     - 控制量 u = 前轮转角 δ
     - 系统矩阵 A, B (基于自行车模型)
     - LQR: min ∫(XᵀQX + uᵀRu)dt
     - 求解反馈增益 K → u = -K*X

  3. 前轮转角限幅
     - δ_cmd = clamp(δ_cmd, δ_min, δ_max)

  4. Pure Pursuit 备选
     - 当 LQR 失效时使用
     - 前视距离 lookahead
     - 计算转向角 α → δ = 2*L*sin(α)/la
```

## 3. State Machine

```
┌──────────────────────────────────────────────────────────────────────┐
│  Control States:                                                      │
│                                                                      │
│   ┌──────────┐                                                      │
│   │  INIT   │──Init()──→ Ready                                     │
│   └──────────┘           │                                          │
│                         │                                           │
│              ┌───────────┼───────────┐                             │
│              ↓           ↓           ↓                              │
│        ┌─────────┐ ┌─────────┐ ┌──────────┐                        │
│        │Driving  │ │Parking  │ │ Emergency│                        │
│        │(正常行驶)│ │(泊车模式)│ │(紧急制动) │                        │
│        └────┬────┘ └────┬────┘ └─────┬────┘                        │
│             └───────────┼───────────┘                              │
│                         ↓                                           │
│                    ┌────────┐                                      │
│                    │ Error  │ → 控制异常                           │
│                    └────────┘                                      │
│                                                                      │
│  Shared Autonomy:                                                    │
│    enable_shared_autonomy_ = true → 检测驾驶员介入                     │
│    steer_angle_tolerance_ → 介入检测阈值                              │
└──────────────────────────────────────────────────────────────────────┘
```

## 4. Parameter Configuration

### Main Config: `modules/control/conf/control_conf.json`

```protobuf
message ControlConf {
    int produce_control_command_duration = 100;  // ms
    int publish_control_command_duration = 100;  // ms
    bool use_system_timestamp = true;
    
    // Lon Controller
    LonControllerConf lon_controller_conf = 1;
    
    // LQR Controller
    LqrControllerConf lqr_controller_conf = 2;
    
    // Vehicle Parameters
    VehicleParam vehicle_param = 3;
    
    // Shared Autonomy
    SharedAutonomyConf shared_autonomy_conf = 4;
}
```

### Lon Controller Config:

```protobuf
message LonControllerConf {
    // PID 参数
    double kp = 0.5;
    double ki = 0.1;
    double kd = 0.0;
    
    // 标定表
    CalibrationTable calibration_table = 1;
    
    // 限幅
    double max_acceleration = 3.0;
    double max_deceleration = 6.0;
    double deadzone = 0.1;
}

message StopDistanceCalibrationTable {
    repeated StopDistanceCalibration data = 1;
}

message LonCalibrationTable {
    repeated LonCalibration data = 1;
}
```

### LQR Controller Config:

```protobuf
message LqrControllerConf {
    // LQR 权重
    double lqr_weight_matrix_q0 = 1.0;  // 横向误差权重
    double lqr_weight_matrix_q1 = 2.0;  // 航向误差权重
    double lqr_weight_matrix_q2 = 0.0;  // 积分误差权重
    double lqr_weight_matrix_r = 1.0;    // 控制量权重
    
    // 标定表
    LqrCalibrationTable lqr_calibration_table = 1;
    
    // 控制器参数
    double ts = 0.01;           // 控制周期(s)
    double cf = 100000.0;       // 前轮侧偏刚度(N/rad)
    double cr = 100000.0;       // 后轮侧偏刚度(N/rad)
    double steer_ratio = 13.1;   // 转向比
    
    // 限幅
    double max_steer_angle = 6.85;  // deg
    double min_steer_angle = -6.85; // deg
}
```

### Shared Autonomy Config:

```protobuf
message SharedAutonomyConf {
    bool enable_shared_autonomy = false;
    double steer_angle_rate = 0.1;      // 方向盘转动速率阈值
    double steer_angle_tolerance = 5.0; // 方向盘容忍角度
    repeated Scheduler steer_rate_speed_scheduler = 1;
}
```

### 关键参数说明

| 参数 | 默认值 | 含义 | 调优建议 |
|------|--------|------|---------|
| lon.kp | 0.5 | 纵向P参数 | 响应速度 |
| lon.ki | 0.1 | 纵向I参数 | 消除稳态误差 |
| lon.max_acceleration | 3.0 | 最大加速度 | 车辆性能限制 |
| lon.max_deceleration | 6.0 | 最大减速度 | 安全限制 |
| lqr.q0 | 1.0 | 横向误差权重 | 路径跟踪精度 |
| lqr.q1 | 2.0 | 航向误差权重 | 方向稳定性 |
| lqr.r | 1.0 | 控制量权重 | 控制平滑度 |
| steer_ratio | 13.1 | 转向比 | 车型参数 |
| max_steer_angle | 6.85 | 最大转角(°) | 机械限制 |

## 5. Workflow - Run & Tune

### Step 1: 检查输入

```bash
# 检查控制话题
ros2 topic list | grep -E "(control|trajectory|chassis|location)"

# 检查底盘状态
ros2 topic echo /chassis --once

# 检查轨迹
ros2 topic echo /adc_trajectory --once
```

### Step 2: 配置模块

```bash
# 编辑控制配置
nano modules/control/conf/control_conf.json

# 编辑纵向标定表
nano modules/control/conf/lon_calibration_table.json

# 编辑LQR标定表
nano modules/control/conf/lqr_calibration_table.json
```

### Step 3: 纵向标定表格式

```json
// lon_calibration_table.json
{
    "data": [
        {"velocity": 0.0, "acceleration": -2.0, "throttle": 0.0},
        {"velocity": 0.0, "acceleration": 0.0, "throttle": 0.3},
        {"velocity": 5.0, "acceleration": 0.0, "throttle": 0.2},
        {"velocity": 10.0, "acceleration": 0.0, "throttle": 0.15},
        ...
    ]
}
```

### Step 4: 构建和运行

```bash
cd modules/control
mkdir -p build && cd build
cmake .. && make -j$(nproc)

./bin/control --config_file=./conf/control_conf.json
```

### Step 5: 参数调优

```bash
# 纵向调优 (速度跟随):
# 问题: 速度超调
# → 降低 kp (0.5 → 0.3)
# → 提高 kd (0.0 → 0.1)

# 问题: 速度响应慢
# → 提高 kp (0.5 → 0.8)
# → 提高 ki (0.1 → 0.2)

# 问题: 停车抖动
# → 提高 lon_calibration_table 中低速段制动力

# 横向调优 (路径跟踪):
# 问题: 横向超调(画龙)
# → 提高 q0 (1.0 → 2.0) 横向误差权重
# → 提高 q1 (2.0 → 3.0) 航向误差权重

# 问题: 转向响应慢
# → 降低 q0 (1.0 → 0.5)
# → 降低 r (1.0 → 0.5) 控制量权重

# 问题: 高速转向过激
# → 提高 r (1.0 → 2.0)
# → 降低 max_steer_angle (6.85 → 5.0)
```

### Step 6: 验证输出

```bash
# 监听控制命令
ros2 topic echo /control_command --once

# 输出格式
{
    "throttle_cmd": 0.3,
    "brake_cmd": 0.0,
    "steer_cmd": 0.5,
    "gear_cmd": 1,  // 1=D, 2=R, 3=N
    "speed": 10.0,
    "acceleration": 0.5
}
```
