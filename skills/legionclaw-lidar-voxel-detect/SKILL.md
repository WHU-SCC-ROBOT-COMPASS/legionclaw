---
name: legionclaw-lidar-voxel-detect
description: CenterPoint-based 3D object detection using VoxelNet + RPN. Read point cloud, run CNN inference (TensorRT), output 3D detections and tune parameters.
argument-hint: [input_bag] [output_dir] [detect_config]
allowed-tools: Bash(ssh *), Bash(cd *), Bash(python3 *), Bash(rostopic *), Bash(rosbag *), Bash(ros2 *), Read, Write, Glob, Grep
---

# LegionClaw LiDAR Voxel Detection Module (CenterPoint)

Module: `modules/perception/lidar/lidar_detect_voxel/`

## 1. Message Flow

```
INPUT:
  - PointCloud2 (ROS2): /sensor/lidar/front/PointCloud2
    → 支持多协议: ROS2/DDS/LCM/ROS

OUTPUT:
  - ObstacleList (深度学习3D检测结果)
  - LidarDetectCommand (可视化/调试)
  - Faults
```

### 订阅/发布接口

| 消息类型 | 方向 | Topic | 格式 |
|---------|------|-------|------|
| PointCloud2 | 订阅 | configurable | ROS2 sensor_msgs/PointCloud2 |
| ObstacleList | 发布 | obstacle_list | legionclaw::interface::ObstacleList |
| LidarDetectCommand | 发布 | lidar_detect_command | 调试可视化 |
| Faults | 发布 | faults | legionclaw::interface::Faults |

## 2. Core Algorithm Flow

```
┌─────────────────────────────────────────────────────────────────────┐
│  Init():                                                           │
│    1. Load config.json                                             │
│    2. 加载 SCN backbone 模型 (scn.centerpoint.onnx)                │
│    3. 加载 RPN head 模型 (rpn_centerhead_sim_x64.plan)             │
│    4. TensorRT 引擎初始化                                           │
│    5. MESSAGE_INIT()                                               │
│    6. Start spin thread                                             │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│  主循环 [10Hz]:                                                    │
│    1. 订阅点云数据                                                  │
│    2. VoxelNet 编码                                                │
│       → 体素化: (D, H, W) grid                                    │
│       → 特征编码: 每个体素计算特征                                   │
│                                                                      │
│    3. SCN (Sparse Convolutional Network) Backbone                   │
│       → SparseConv3d 操作                                          │
│       → 提取点云特征                                                │
│                                                                      │
│    4. RPN (Region Proposal Network) Head                           │
│       → 检测头: 类别分类 + 边界框回归                                │
│       → NMS 后处理                                                  │
│                                                                      │
│    5. 输出检测结果                                                  │
│       → (x, y, z, l, w, h, theta, score, class)                   │
│       → 转换为 ObstacleList                                        │
└─────────────────────────────────────────────────────────────────────┘
```

### CenterPoint 算法

```
CenterPoint 检测流程:
  1. Voxelization (体素化)
     → 点云 → (D, H, W) 稀疏网格
     → 每个非空体素计算均值特征

  2. Backbone (SCN)
     → SparseConv3d → 稀疏3D卷积
     → 逐步下采样提取特征

  3. Detection Head (RPN)
     → 中心点热力图: H × W × Num_Classes
     → 边界框回归: H × W × 6 (x, y, z, l, w, h)
     → 角度回归: H × W × 2 (sin, cos)

  4. 后处理
     → 从热力图提取局部最大值 → 检测框
     → NMS 去除重叠框
     → 过滤低置信度框
```

## 3. Parameter Configuration

### Main Config: `modules/perception/lidar/lidar_detect_voxel/conf/perception/lidar/lidar_detect/lidar_detect.json`

```json
{
    "app_name": "lidar_detect",
    "description": "lidar_detect module",
    "log_file_path": "./log",
    "log_level": 1,
    "logging_data_enable": false,
    "use_system_timestamp": false,
    "produce_lidar_detect_command_duration": 100,
    "publish_lidar_detect_command_duration": 100,
    
    "scn_engine_file_path": "../model/scn.centerpoint.onnx",
    "rpn_engine_file_path": "../model/rpn_centerhead_sim_x64.plan.8415",
    
    "general_threshold": 0.5,
    "big_vehicle_threshold": 0.4,
    "unknow_threshold": 0.6,
    
    "message": {
        "active_message": [4],
        "message_info": [
            {"type": 0, "name": "LCM", "url": "udpm://239.255.76.21:7621?ttl=3"},
            {"type": 4, "name": "ROS2", "url": "null"}
        ]
    }
}
```

### 检测阈值参数

| 参数 | 默认值 | 含义 | 调优建议 |
|------|--------|------|---------|
| general_threshold | 0.5 | 通用障碍物置信度阈值 | 提高减少误检 |
| big_vehicle_threshold | 0.4 | 大型车辆置信度阈值 | 低于通用阈值 |
| unknow_threshold | 0.6 | 未知类型阈值 | 提高减少误分类 |

### 模型文件

| 文件 | 格式 | 作用 |
|------|------|------|
| scn.centerpoint.onnx | ONNX | SCN Backbone (稀疏卷积网络) |
| rpn_centerhead_sim_x64.plan | TensorRT Plan | RPN检测头 |

## 4. Workflow - Run & Tune

### Step 1: 检查模型文件

```bash
# 检查模型文件存在
ls modules/perception/lidar/lidar_detect_voxel/model/

# 验证模型格式
python3 -c "import onnx; onnx.load('scn.centerpoint.onnx')"
```

### Step 2: 配置模块

```bash
# 编辑检测配置
nano modules/perception/lidar/lidar_detect_voxel/conf/perception/lidar/lidar_detect/lidar_detect.json

# 关键参数:
# - general_threshold: 提高减少误检, 降低增加召回
# - big_vehicle_threshold: 针对卡车/大巴
# - unknow_threshold: 未知类别检测阈值
```

### Step 3: 构建和运行

```bash
cd modules/perception/lidar/lidar_detect_voxel
mkdir -p build && cd build
cmake .. && make -j$(nproc)

./bin/lidar_detect --config_file=./conf/lidar_detect.json
```

### Step 4: 参数调优

```bash
# 场景1: 减少误检(误识别为障碍物)
# → 提高 general_threshold (0.5 → 0.6)
# → 提高 unknow_threshold (0.6 → 0.7)

# 场景2: 提高召回(漏检障碍物)
# → 降低 general_threshold (0.5 → 0.4)
# → 降低 unknow_threshold (0.6 → 0.5)

# 场景3: 大型车辆检测
# → 调整 big_vehicle_threshold
# → 确保训练数据包含足够的大型车辆样本
```

### Step 5: 输出验证

```bash
# 监听检测结果
ros2 topic echo /obstacle_list --once

# 输出格式 (ObstacleList)
{
    "header": {...},
    "obstacle": [
        {
            "id": 0,
            "type": 1,  // 0=unknown, 1=car, 2=pedestrian, etc.
            "confidence": 0.85,
            "center_pos": {"x": 10.5, "y": 3.2, "z": 0.8},
            "length": 4.5,
            "width": 1.8,
            "height": 1.6,
            "theta": 0.1,
            "velocity": {"x": 2.0, "y": 0.0, "z": 0.0}
        },
        ...
    ]
}
```

## 5. CenterPoint 检测类别

| Type ID | 类别 | 说明 |
|---------|------|------|
| 0 | UNKNOWN | 未知类型 |
| 1 | CAR | 乘用车 |
| 2 | TRUCK | 卡车 |
| 3 | BUS | 公交车 |
| 4 | PEDESTRIAN | 行人 |
| 5 | BICYCLE | 自行车 |
| 6 | MOTORCYCLE | 摩托车 |

## 6. 集成到 Pipeline

```
PointCloud (LiDAR)
     ↓
lidar_ground_segmentation (可选: PatchWork++)
     ↓
lidar_detect_voxel (CenterPoint 深度学习检测)
     ↓
ObstacleList (带类别/朝向/速度)
     ↓
motion_manager (多目标跟踪 + 融合)
```
