# 多激光雷达拼接系统 (Multi-Lidar Splicing)

## 项目简介

本项目是一个基于ROS2的多激光雷达点云拼接系统，能够同步订阅5个激光雷达的点云数据，通过外参标定矩阵进行坐标变换，最终合并为统一的点云数据并发布。

### 主要功能

- 同步订阅5个激光雷达的点云话题（前、中、左、右、后）
- 使用 `message_filters` 进行时间同步（ApproximateTime策略）
- 顺序处理各激光雷达的点云转换和坐标变换，避免频繁创建/销毁线程导致抖动
- 将变换后的点云合并为统一坐标系下的点云并发布
- 可靠QoS配置，减少点云丢帧

### 技术特点

- 使用ROS2框架，支持可靠的消息传输
- 可靠QoS与较大队列深度，降低同步丢帧风险
- 支持仿真时间（`use_sim_time`）和系统时间
- 优化的内存管理，减少点云合并时的内存分配开销

## 系统要求

### 依赖项

- **ROS2** (Foxy或更高版本)
  - `rclcpp`
  - `sensor_msgs`
  - `message_filters`
  - `pcl_ros` (可选，用于PCL与ROS消息转换)

- **第三方库**
  - **PCL** (Point Cloud Library) - 点云处理
  - **Eigen3** - 矩阵运算
  - **jsoncpp** - JSON配置文件解析

### 安装依赖

#### Ubuntu/Debian系统

```bash
# 安装ROS2（如果未安装）
# 参考：https://docs.ros.org/en/foxy/Installation.html

# 安装PCL
sudo apt-get install libpcl-dev

# 安装Eigen3
sudo apt-get install libeigen3-dev

# 安装jsoncpp
sudo apt-get install libjsoncpp-dev

# 安装ROS2相关包
sudo apt-get install ros-foxy-rclcpp ros-foxy-sensor-msgs ros-foxy-message-filters

# 安装pcl_ros（可选，但推荐）
sudo apt-get install ros-foxy-pcl-ros
```

## 编译指南

### 方法一：使用提供的编译脚本

```bash
# 使用默认编译脚本（Release模式）
./scripts/build.sh

# 或使用Debug模式
./scripts/build_debug.sh

# 或使用Release模式
./scripts/build_release.sh
```

### 方法二：手动编译

```bash
# 创建工作目录
mkdir -p build
cd build

# 配置CMake（Release模式）
cmake .. -DCMAKE_BUILD_TYPE=Release

# 编译
make -j$(nproc)

# 清理编译文件（可选）
cd ..
./scripts/clean.sh
```

### 编译输出

编译成功后，可执行文件位于：
```
bin/multi_lidar_splicing
```

## 使用方法

### 1. 准备配置文件

#### 主配置文件 (`config.json`)

在主配置文件中指定5个激光雷达的标定参数路径、输出坐标系和发布话题：

```json
{
    "calibration_params_path": {
        "lidar_front": "./config/sensors/legion/lidar_front.json",
        "lidar_mid": "./config/sensors/legion/lidar_mid.json",
        "lidar_left": "./config/sensors/legion/lidar_left.json",
        "lidar_right": "./config/sensors/legion/lidar_right.json",
        "lidar_back": "./config/sensors/legion/lidar_back.json"
    },
    "frame_id": "map",
    "publish_topic": "/rslidar_points"
}
```

**参数说明：**
- `calibration_params_path`: 包含5个激光雷达标定文件路径的对象
  - `lidar_front`: 前激光雷达标定文件路径
  - `lidar_mid`: 中间激光雷达标定文件路径
  - `lidar_left`: 左激光雷达标定文件路径
  - `lidar_right`: 右激光雷达标定文件路径
  - `lidar_back`: 后激光雷达标定文件路径
- `frame_id`: 合并后点云的坐标系名称（如 "map", "base_link" 等）
- `publish_topic`: 发布合并点云的话题名称

#### 激光雷达标定文件格式

每个激光雷达的标定文件（JSON格式）包含以下参数：

```json
{
    "channel": "/sensor/lidar/front/PointCloud2",
    "rotation": [
        0.999931, -0.006289, 0.003173,
        0.006355,  0.999861, -0.015226,
       -0.003076,  0.015245, 0.999876
    ],
    "translation": [2.02388, 0.017979, 0.44323]
}
```

**参数说明：**
- `channel`: 该激光雷达的ROS2话题名称（PointCloud2类型）
- `rotation`: 旋转矩阵，支持两种格式：
  - **3x3矩阵格式**：`[[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]]`
  - **9元素数组格式**：`[r11, r12, r13, r21, r22, r23, r31, r32, r33]`（按行展开）
- `translation`: 平移向量，支持两种格式：
  - **数组格式**：`[x, y, z]`
  - **对象格式**：`{"x": 2.02388, "y": 0.017979, "z": 0.44323}`

**注意：** 外参矩阵表示从激光雷达坐标系到目标坐标系（`frame_id`）的变换。

### 2. 运行程序

#### 基本运行

```bash
# 确保已source ROS2环境
source /opt/ros/foxy/setup.bash  # 根据你的ROS2版本调整

# 注意：当前代码使用硬编码配置文件路径 "./conf/config.json"
# 请确保在运行目录下存在该配置文件，或修改代码中的路径

# 方法1：创建符号链接或复制配置文件到运行目录
mkdir -p conf
cp bin/config/config.json conf/config.json

# 方法2：在bin目录下运行（如果配置文件路径是相对于可执行文件的）
cd bin
./multi_lidar_splicing

# 方法3：使用绝对路径（需要修改代码中的硬编码路径）
```

**注意：** 当前版本代码中配置文件路径硬编码为 `"./conf/config.json"`。建议：
- 在运行目录下创建 `conf` 目录并放置配置文件
- 或修改 `src/app/main.cpp` 中的路径为实际配置文件位置
- 或修改代码支持命令行参数传递配置文件路径

#### 使用ROS2 launch（推荐）

如果使用ROS2 launch文件，可以通过参数传递配置文件路径（需要先修改代码支持命令行参数）。

#### 使用仿真时间

如果需要在Gazebo等仿真环境中使用，需要启用仿真时间：

```bash
# 在运行目录下执行
./bin/multi_lidar_splicing --ros-args -p use_sim_time:=true
```

### 3. 验证运行

运行后，程序会：
1. 订阅5个激光雷达的点云话题
2. 等待所有话题的消息同步
3. 对每个激光雷达的点云进行坐标变换
4. 合并所有点云
5. 发布到指定的输出话题

可以使用以下命令查看发布的话题：

```bash
# 查看话题列表
ros2 topic list

# 查看点云数据
ros2 topic echo /rslidar_points --no-arr

# 使用rviz2可视化
rviz2
# 在rviz2中添加PointCloud2显示，话题选择配置的publish_topic
```

## 配置文件参数详解

### 主配置文件参数

| 参数 | 类型 | 说明 | 必填 |
|------|------|------|------|
| `calibration_params_path` | object | 包含5个激光雷达标定文件路径的对象 | 是 |
| `calibration_params_path.lidar_front` | string | 前激光雷达标定文件路径（相对或绝对路径） | 是 |
| `calibration_params_path.lidar_mid` | string | 中间激光雷达标定文件路径 | 是 |
| `calibration_params_path.lidar_left` | string | 左激光雷达标定文件路径 | 是 |
| `calibration_params_path.lidar_right` | string | 右激光雷达标定文件路径 | 是 |
| `calibration_params_path.lidar_back` | string | 后激光雷达标定文件路径 | 是 |
| `frame_id` | string | 输出点云的坐标系名称 | 是 |
| `publish_topic` | string | 发布合并点云的话题名称 | 是 |

### 激光雷达标定文件参数

| 参数 | 类型 | 说明 | 必填 |
|------|------|------|------|
| `channel` | string | 该激光雷达的ROS2话题名称（PointCloud2类型） | 是 |
| `rotation` | array | 旋转矩阵（3x3），支持3x3嵌套数组或9元素一维数组 | 是 |
| `translation` | array/object | 平移向量 [x, y, z] 或 {"x": ..., "y": ..., "z": ...} | 是 |

## 性能优化

程序已实现以下性能优化：

1. **顺序处理**：避免每帧创建/销毁线程导致的抖动
2. **内存优化**：预先计算总点数，一次性分配内存，使用 `insert` 而非 `+=` 减少内存重分配
3. **消息同步**：使用 `ApproximateTime` 策略，队列大小50（可按需要调整）
4. **QoS配置**：使用 `Reliable` 策略，队列深度100，降低消息丢帧概率
5. **性能告警**：处理耗时超过10Hz周期时输出拼接耗时告警

## 代码修改建议

### 支持命令行参数传递配置文件路径

当前代码使用硬编码路径 `"./conf/config.json"`。如果需要支持命令行参数，可以修改 `src/app/main.cpp`：

```cpp
int main(int argc, char *argv[])
{
    // 支持命令行参数
    std::string file_path = "./conf/config.json";  // 默认路径
    if (argc >= 2) {
        file_path = argv[1];  // 使用命令行参数
    }
    
    std::string lidar_front, lidar_mid, lidar_left, lidar_right, lidar_back, frame_id, publish_topic;
    if (!loadConfig(file_path, lidar_front, lidar_mid, lidar_left, lidar_right, lidar_back, frame_id, publish_topic))
        return 1;
    
    // ... 其余代码保持不变
}
```

修改后，可以这样运行：
```bash
./bin/multi_lidar_splicing ./bin/config/config.json
```

## 故障排除

### 常见问题

1. **找不到依赖库**
   - 确保已安装所有依赖项（见"系统要求"部分）
   - 检查CMake是否能找到ROS2：`echo $ROS_DISTRO`

2. **消息同步失败**
   - 检查5个激光雷达话题是否都在发布数据
   - 确认话题名称与标定文件中的 `channel` 一致
   - 检查时间戳是否同步（如果使用仿真时间，确保所有节点都启用 `use_sim_time`）

3. **点云数据异常**
   - 检查外参标定矩阵是否正确
   - 确认 `frame_id` 设置合理
   - 使用 `rviz2` 可视化检查各激光雷达原始数据

4. **编译错误**
   - 确保CMake版本 >= 3.10
   - 检查C++标准（需要C++17）
   - 查看完整错误信息：`make VERBOSE=1`

## 项目结构

```
multi_lidar_splicing/
├── bin/                    # 编译输出目录
│   ├── multi_lidar_splicing  # 可执行文件
│   └── config/             # 配置文件目录
│       ├── config.json     # 主配置文件
│       └── sensors/        # 激光雷达标定文件目录
│           └── legion/    # 示例配置（legion车型）
├── build/                  # CMake构建目录
├── scripts/                # 编译脚本
│   ├── build.sh
│   ├── build_debug.sh
│   ├── build_release.sh
│   └── clean.sh
├── src/                    # 源代码目录
│   ├── app/                # 主程序入口与节点逻辑
│   │   ├── main.cpp
│   │   ├── multi_lidar_splicing.h
│   │   └── multi_lidar_splicing.cpp
│   └── sensors/
│       ├── lidar.h
│       └── lidar.cpp
├── CMakeLists.txt         # CMake配置文件
└── README.md              # 本文档
```
