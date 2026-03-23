---
name: legionclaw-routing
description: Route planning using Lanelet2 HD map. Read routing request, compute shortest path, output routing response and guide info.
argument-hint: [input_bag] [output_dir] [routing_config]
allowed-tools: Bash(ssh *), Bash(cd *), Bash(python3 *), Bash(rostopic *), Bash(rosbag *), Bash(ros2 *), Read, Write, Glob, Grep
---

# LegionClaw Routing Module

Module: `modules/routing/`

## 1. Message Flow

```
INPUT:
  - RoutingRequest: 路由请求 (起点/终点/方式)
  - Location: 车辆当前位置 (用于地图匹配)
  - Odometry: 里程计信息
  - ParkingInfo: 泊车位信息 (AVP模式)
  - TrafficLightMsg: 交通灯状态
  - Chassis: 底盘状态

OUTPUT:
  - RoutingResponse: 路由结果 (路径/车道序列)
  - LaneList: 车道列表
  - GlobalRouteMsg: 全局路由消息
  - GuideInfo: 导航信息
  - StopInfo: 停车点信息
  - TrafficEvents: 交通事件
  - NaviInfo: 导航状态信息
```

### 订阅/发布接口

| 消息类型 | 方向 | Topic | 格式 |
|---------|------|-------|------|
| RoutingRequest | 订阅 | routing_request | legionclaw::interface::RoutingRequest |
| Location | 订阅 | location | legionclaw::interface::Location |
| Odometry | 订阅 | odometry | legionclaw::interface::Odometry |
| ParkingInfo | 订阅 | parking_info | legionclaw::interface::ParkingInfo |
| TrafficLightMsg | 订阅 | traffic_light | legionclaw::interface::TrafficLightMsg |
| Chassis | 订阅 | chassis | legionclaw::interface::Chassis |
| RoutingResponse | 发布 | routing_response | legionclaw::interface::RoutingResponse |
| LaneList | 发布 | lane_list | legionclaw::interface::LaneList |
| GlobalRouteMsg | 发布 | global_route_msg | legionclaw::interface::GlobalRouteMsg |
| GuideInfo | 发布 | guide_info | legionclaw::interface::GuideInfo |
| StopInfo | 发布 | stop_info | legionclaw::interface::StopInfo |
| TrafficEvents | 发布 | traffic_events | legionclaw::interface::TrafficEvents |
| NaviInfo | 发布 | navi_info | legionclaw::interface::NaviInfo |
| Faults | 发布 | faults | legionclaw::interface::Faults |
| ObuCmdMsg | 订阅 | obu_cmd | 功能激活/去激活控制 |

## 2. Core Algorithm Flow

```
┌─────────────────────────────────────────────────────────────────────┐
│  Init():                                                           │
│    1. Load RoutingConf from protobuf                               │
│    2. LOGGING_INIT()                                                │
│    3. MESSAGE_INIT()                                                │
│    4. Load Lanelet2 HD Map                                         │
│    5. Build Routing Graph                                           │
│    6. TrafficRules init                                            │
│    7. 创建多个定时器:                                                │
│       - task_mainloop_: 主循环 (loop_rate_)                         │
│       - task_check_station_: 站点检查                               │
│       - task_check_stopline_: 停止线检查                            │
│       - task_merge_polygons_: 多边形合并                             │
│       - task_global_msg_gen_: 全局消息生成                           │
│       - task_guide_info_gen_: 导航信息生成                           │
│       - task_lanelines_gen_: 车道线生成                              │
│       - task_navi_info_gen_: 导航状态生成                            │
│    8. TaskActivate()                                               │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│  MainLoop() [定时触发]:                                            │
│                                                                      │
│  ┌─ [Step 1] RoutingRequest 处理                                    │
│  │   接收新请求 → new_request_received_ = true                       │
│  │   解析起点/终点/路线类型                                          │
│  │                                                                  │
│  ├─ [Step 2] 地图匹配                                              │
│  │   基于Location匹配到地图元素                                       │
│  │   start_pose_inited_                                             │
│  │                                                                  │
│  ├─ [Step 3] 路径计算                                              │
│  │   Lanelet2 RoutingGraph 最短路径                                  │
│  │   → shortestPath(start_lanelet, end_lanelet)                    │
│  │                                                                  │
│  ├─ [Step 4] Route 解析                                            │
│  │   提取路径上的lanelet序列                                         │
│  │   计算限速信息                                                   │
│  │   标记停车点/交通灯/交叉口                                        │
│  │                                                                  │
│  └─ [Step 5] 发送 RoutingResponse                                 │
│      PublishRoutingResponse()                                       │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│  定时任务 (各独立Timer):                                            │
│                                                                      │
│  Check_Sation_Loop():                                               │
│    → 检查是否到达站点                                               │
│    → 触发站点事件                                                   │
│                                                                      │
│  CheckStopLine():                                                  │
│    → 检查前方停止线                                                 │
│    → 触发交通灯响应                                                 │
│                                                                      │
│  MergePolygons():                                                  │
│    → 合并车道多边形                                                 │
│    → 发布可行驶区域                                                 │
│                                                                      │
│  GlobalMsgGen():                                                   │
│    → 生成全局路由消息                                               │
│    → 包含所有路径点和属性                                           │
│                                                                      │
│  GuideInfoGen():                                                   │
│    → 生成导航信息                                                   │
│    → 转向提示/距离/路口信息                                         │
│                                                                      │
│  LanelinesGen():                                                   │
│    → 生成车道线信息                                                 │
│    → 用于车道保持/显示                                              │
│                                                                      │
│  NaviInfoPublisher():                                              │
│    → 发布导航状态                                                   │
│    → 当前车道/剩余距离/预计时间                                      │
└─────────────────────────────────────────────────────────────────────┘
```

### Lanelet2 Routing 算法

```
Lanelet2 routing graph:
  1. 地图加载
     → 读取OSM/XOSM格式地图
     → 构建 LaneletMap
     → 解析 lanelet, wayarea, polygon

  2. 路由图构建
     → RoutingGraphPtr routingGraph = RoutingGraph::build(map)
     → 基于 traffic_rules 配置可达性

  3. 最短路径计算
     → routingGraph->shortestPath(start_lanelet, end_lanelet)
     → 返回LaneletOrAreaSequence

  4. 路由结果解析
     → 遍历路径,提取每个lanelet属性
     → speed_limit: 限速值
     → traffic_rules: 交通规则
     → regulatory_elements: 交通标志/信号
```

## 3. State Machine

```
┌──────────────────────────────────────────────────────────────────────┐
│  Module States:                                                       │
│                                                                      │
│  Init Flags:                                                          │
│    map_loaded_ = false → true (地图加载成功)                         │
│    transformer_init_ = false → true (坐标变换初始化)                  │
│                                                                      │
│  Routing Flags:                                                       │
│    new_goal_received_ = false → true (新目标)                        │
│    new_parking_space_ = false → true (新泊车位)                      │
│    start_pose_inited_ = false → true (起点初始化)                     │
│    new_request_received_ = false → true (新请求)                     │
│    first_send_ = false → true (首次发送)                              │
│    send_flag_ = false → true (发送标志)                              │
│                                                                      │
│  Functional Flags:                                                    │
│    location_init_ = false → true (定位初始化)                        │
│    match_stop_points_ = false → true (停车点匹配)                     │
│    mileage_enable_ = false (里程计使能)                              │
│    stop_check_ = false (停车检查)                                    │
│                                                                      │
│  Traffic Flags:                                                       │
│    traffic_light_msg_.contain_lights() → 交通灯状态                   │
│    cur_event_.limit_speed_info() → 限速信息                          │
│    cur_speed_limit → 当前限速                                        │
└──────────────────────────────────────────────────────────────────────┘
```

## 4. Parameter Configuration

### Main Config: `modules/routing/conf/routing_conf.json`

```protobuf
message RoutingConf {
    int loop_rate = 100;                    // 主循环频率(ms)
    int routing_request_timeout = 5000;     // 路由请求超时(ms)
    
    // 地图相关
    string map_file = "./data/map/vector_map.osm";  // 地图文件路径
    string routing_graph_file = "";          // 预构建路由图(可选)
    
    // 路线计算
    bool allow_road_assignment = true;      // 允许道路分配
    double min_route_length = 10.0;          // 最小路由长度(m)
    
    // 交通规则
    bool enable_traffic_junction = true;     // 启用交通路口处理
    bool enable_polygon_send = true;        // 发送可行驶区域多边形
    bool enable_route_msg_send = true;      // 发送全局路由消息
    bool enable_guide_info_send = true;     // 发送导航信息
    bool enable_lanelines_send = true;      // 发送车道线信息
    bool enable_navi_info_send = true;      // 发送导航状态
    
    // 生成频率
    int check_junction_rate = 1000;         // 路口检查周期(ms)
    int polygon_gen_rate = 1000;           // 多边形生成周期(ms)
    int route_msg_gen_rate = 1000;         // 路由消息生成周期(ms)
    int guide_info_gen_rate = 1000;       // 导航信息生成周期(ms)
    int lanelines_gen_rate = 1000;         // 车道线生成周期(ms)
    int navi_info_gen_rate = 100;          // 导航状态生成周期(ms)
    
    // 限速相关
    double default_speed_limit = 30.0;      // 默认限速(km/h)
    double intersection_speed_limit = 15.0;  // 路口限速(km/h)
    
    // 停车相关
    double parking_space_search_radius = 50.0;  // 泊车位搜索半径(m)
    double stop_distance_threshold = 5.0;      // 停车距离阈值(m)
}
```

### 关键参数说明

| 参数 | 默认值 | 含义 | 调优建议 |
|------|--------|------|---------|
| loop_rate | 100 | 主循环周期(ms) | 计算资源权衡 |
| map_file | ./data/map/... | 地图文件路径 | 确保路径正确 |
| min_route_length | 10.0 | 最小路由长度(m) | 防止无效短途 |
| default_speed_limit | 30.0 | 默认限速(km/h) | 城市/高速切换 |
| intersection_speed_limit | 15.0 | 路口限速(km/h) | 安全调低 |
| parking_space_search_radius | 50.0 | 泊车搜索半径 | 室内增大 |
| stop_distance_threshold | 5.0 | 停车检测距离 | 精度要求 |

## 5. Workflow - Run & Tune

### Step 1: 准备地图数据

```bash
# 检查地图文件
ls modules/routing/data/map/
# 常见格式: .osm, .xosm (Lanelet2格式)

# 验证地图加载
python3 verify_map.py --map_file <map_path>
```

### Step 2: 发送路由请求

```bash
# ROS2 发送路由请求
ros2 topic pub /routing_request legionclaw_msgs/RoutingRequest "
header:
  stamp:
    sec: 0
    nsec: 0
  frame_id: 'map'
start_pose:
  position:
    x: 100.0
    y: 50.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.707
    w: 0.707
end_pose:
  position:
    x: 200.0
    y: 80.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.707
    w: 0.707
routing_type: 1  # 1=正常路线, 2=AVP
"
```

### Step 3: 配置模块

```bash
# 编辑路由配置
nano modules/routing/conf/routing_conf.json

# 关键参数:
# - map_file: 地图文件路径
# - enable_*_send: 各功能开关
# - *_rate: 各生成器周期
```

### Step 4: 构建和运行

```bash
cd modules/routing
mkdir -p build && cd build
cmake .. && make -j$(nproc)

./bin/routing --config_file=./conf/routing_conf.json
```

### Step 5: 验证输出

```bash
# 监听路由响应
ros2 topic echo /routing_response --once

# 监听导航信息
ros2 topic echo /navi_info --once

# 监听停车信息
ros2 topic echo /stop_info --once

# 输出格式 (RoutingResponse)
{
    "header": {...},
    "status": {
        "error_code": 0,
        "msg": "success"
    },
    "routing_type": 1,
    "road": [
        {
            "id": 1234,
            "lane_id": [1, 2, 3, ...],
            "speed_limit": 30.0,
            "stop_points": [
                {"x": 150.0, "y": 55.0, "type": 1}  // 1=站点, 2=停车位
            ],
            "lanelets": [...]
        }
    ]
}
```

### Step 6: 调优场景

```bash
# AVP泊车场景:
# → enable_route_msg_send = true
# → routing_type = 2 (AVP)
# → parking_space_search_radius 增大

# 高速巡航场景:
# → default_speed_limit = 60.0 (km/h)
# → intersection_speed_limit = 30.0 (km/h)
# → enable_traffic_junction = true

# 室内导航场景:
# → default_speed_limit = 15.0 (km/h)
# → parking_space_search_radius = 100.0 (m)
```

## 6. 地图格式说明

### Lanelet2 OSM 格式

```xml
<!-- 基本元素 -->
<node id="1" lat="31.0" lon="121.0" version="1"/>
<way id="101">
  <nd ref="1"/>
  <nd ref="2"/>
  <tag k="type" v="lanelet"/>
  <tag k="subtype" v="road"/>
  <tag k="speed_limit" v="30 km/h"/>
</way>

<!-- 路由关系 -->
<relation id="1001">
  <member type="way" ref="101" role="left"/>
  <member type="way" ref="102" role="right"/>
  <member type="way" ref="103" role="centerline"/>
  <tag k="type" v="lanelet"/>
</relation>
```

## 7. 输出格式汇总

### RoutingResponse
- `road[]`: 路线段列表
- `lane_id[]`: 车道ID序列
- `speed_limit`: 限速值
- `stop_points[]`: 停车点列表

### LaneList
- `lane[]`: 详细车道信息
- `lane_markings[]`: 车道线信息
- `polygon[]`: 可行驶区域

### NaviInfo
- `current_lane_id`: 当前车道ID
- `next_lane_id`: 下一车道ID
- `remaining_distance`: 剩余距离
- `eta`: 预计到达时间
- `current_speed_limit`: 当前限速
