# 环境搭建

## 系统要求

- **OS**: Ubuntu 22.04 LTS
- **ROS2**: Humble Hawksbill
- **编译器**: GCC 11.4+

## 安装依赖

```bash
# ROS2 Humble（如未安装）
sudo apt install ros-humble-desktop

# 建图/定位依赖
sudo apt install -y \
  libpcl-dev \
  libeigen3-dev \
  libgeographic-dev \
  libboost-all-dev \
  libopencv-dev

# ROS2 依赖包
sudo apt install -y \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-tf2-eigen \
  ros-humble-tf2-geometry-msgs \
  ros-humble-rclcpp-components
```

## 验证

```bash
source /opt/ros/humble/setup.bash
ros2 pkg list | grep -E "pcl|tf2|geographic"
```

## 编译

```bash
cd /path/to/legionclaw
./scripts/slam/build_slam.sh
```

成功输出：
```
✅ 全部编译完成：消息包(3) + 定位包(7) + 建图包(1)
```
