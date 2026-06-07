# LegionClaw Wiki

## 目录

- [环境搭建](env-setup.md)
- [建图流程](mapping.md)
- [定位流程](localization.md)
- [数据说明](data.md)
- [常见问题](faq.md)

---

## 环境搭建

**依赖**

```bash
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-pcl-ros \
  ros-humble-tf2-eigen \
  ros-humble-tf2-geometry-msgs \
  libpcl-dev \
  libeigen3-dev \
  libgeographic-dev \
  libboost-all-dev \
  libopencv-dev
```

**编译（一键）**
```bash
cd /path/to/legionclaw
./scripts/slam/build_slam.sh
```

编译内容包括：
1. `modules/message/ros2/` — 消息包（ros2_interface, ros2_slam_msgs）
2. `ros2_ws/` — 定位包（hdl_localization, ndt_omp, fast_gicp, global_localization）
3. `ros2_ws/src/legionclaw_map/` — 建图包

---

## 建图流程

使用 FAST-LIO-SAM 进行激光雷达建图。输入 livox 点云 + IMU，输出 PCD 格式点云地图。

**启动**

```bash
./scripts/slam/build_slam.sh       # 首次需要编译
./scripts/slam/run_mapping.sh       # 播完整建图 bag
```

**参数**

```bash
./scripts/slam/run_mapping.sh \
  --rate 1.0          # 播放速率（默认 1.0）
  --bag <path>        # 指定 bag 路径
  --duration 60       # 只播前 60 秒
  --res 0.3           # 地图保存分辨率（默认 0.3）
```

**输出**

- `output/slam_map.pcd` — 保存的地图（注意：当前 `getFullMap()` 返回 body-frame 点云，此功能待完善）
- 可使用预先建好的地图 `output/mid360_map_filtered.pcd`

---

## 定位流程

使用 hdl_localization 进行 NDT 扫描匹配定位，加载预建 PCD 地图，在 bag 点云中实时定位。

**启动**

```bash
./scripts/slam/build_slam.sh
./scripts/slam/run_localization.sh
```

**参数**

```bash
./scripts/slam/run_localization.sh \
  --map output/mid360_map_filtered.pcd   # 指定地图
  --bag data/bag_files/lidar_xxx.db3     # 指定定位 bag
  --rate 3.0           # 播放速率（加速测试）
  --duration 60        # 只播前 60 秒
  --odom-child "livox_frame"  # odom child frame（匹配点云 frame_id）
```

**输出**

- `test_result/localization_yyyyMMdd_HHmmss.txt` — odom 轨迹数据
- `test_result/localization_yyyyMMdd_HHmmss_2d.png` — 二维轨迹图
- `test_result/localization_yyyyMMdd_HHmmss_3d.png` — 三维轨迹图
- `test_result/localization_yyyyMMdd_HHmmss_stats.png` — 统计图

---

## 数据说明

bag 文件存放在 `data/bag_files/`（不上传 git）。

| 文件 | 大小 | 时长 | 用途 |
|------|------|------|------|
| `lidar_20260326_013648_0.db3` | 1.5G | ~5min | 建图 |
| `lidar_20260326_014243_0.db3` | 1.2G | ~4min | 定位 |
| `lidar_20260327_173914_0.mcap` | 2.1G | ~28s | 多雷达测试 |

---

## 常见问题

### Q1: 建图 segfault

已修复。`fast_lio_sam.cpp` 构造函数中增加了 `state_->init_dyn_runtime_share()` 调用。
如果仍出现，检查 `esekfom.hpp` 中 `predict()` 是否有 `build_*_state()` 保护。

### Q2: "Failed to find match for field 'normal_x'"

livox 点云只有 xyz+intensity，但 map 内部使用 PointXYZINormal。
已在 `laserMappingNode.cpp` 中通过 `fromROSMsgWithFallback()` 自动补零解决。

### Q3: 定位显示 "No input target dataset was given"

地图文件路径不正确或 PCD 文件损坏。检查 `--map` 参数指向的文件是否存在。

### Q4: TF 错误 "livox_frame does not exist"

hdl_localization launch 中 `odom_child_frame_id` 默认是 `lidar`，但点云 frame_id 是 `livox_frame`。
需要用 `--odom-child livox_frame` 或直接在 launch 参数中设置。

### Q5: 编译时 GeographicLib 找不到

```bash
sudo apt install libgeographic-dev
```

或在 CMake 中手动指定模块路径：
```cmake
set(CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
```
