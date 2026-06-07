# 数据说明

## bag 数据

bag 文件存放于 `data/bag_files/` 目录（不上传 git，需手动放置）。

### 建图 bag

| 属性 | 值 |
|------|------|
| 文件 | `lidar_20260326_013648_0.db3` |
| 大小 | 1.5 GiB |
| 时长 | 299.8s (~5min) |
| 点云帧 | 2998 帧 |
| IMU帧 | 59967 帧 |
| 话题 | `/livox/lidar` (sensor_msgs/PointCloud2), `/livox/imu` (sensor_msgs/Imu) |

### 定位 bag

| 属性 | 值 |
|------|------|
| 文件 | `lidar_20260326_014243_0.db3` |
| 大小 | 1.2 GiB |
| 时长 | 240.6s (~4min) |
| 点云帧 | 2399 帧 |
| IMU帧 | 2440 帧 |
| 话题 | `/livox/lidar`, `/livox/imu` |

### 多雷达 bag

| 属性 | 值 |
|------|------|
| 文件 | `lidar_20260327_173914_0.mcap` |
| 大小 | 2.1 GiB |
| 时长 | 27.7s |
| 话题 | `/d1/front_left/point_cloud`, `/d1/front_right/point_cloud`, `/d1/rear_left/point_cloud`, `/d1/rear_right/point_cloud`, `/livox/lidar` |

## 地图文件

`output/` 目录下的 PCD 文件：

| 文件 | 大小 | 点数 | 说明 |
|------|------|------|------|
| `mid360_map_test.pcd` | 157 MB | 513万 | 原始建图输出 |
| `mid360_map_filtered.pcd` | 157 MB | 513万 | 下采样后（0.5m分辨率），供定位使用 |
| `mid360_map_bev.png` | 380 KB | — | BEV 鸟瞰图 |

## 传感器配置

- 雷达：Livox Mid-360
- LiDAR 话题：`/livox/lidar`，frame_id: `livox_frame`
- IMU 话题：`/livox/imu`
- LiDAR-IMU 外参：已标定，集成在配置中
