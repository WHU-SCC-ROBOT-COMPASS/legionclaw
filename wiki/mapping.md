# 建图流程

## 流程

```
bag 点云 + IMU  →  FAST-LIO-SAM  →  PCD 点云地图
                    (legionclaw_map)
```

## 启动

```bash
cd /path/to/legionclaw

# 首次：编译
./scripts/slam/build_slam.sh

# 建图（播完整建图 bag）
./scripts/slam/run_mapping.sh
```

## 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--rate` | 1.0 | bag 播放速率（1.0=正常速度） |
| `--bag` | `data/bag_files/lidar_20260326_013648_0.db3` | 建图 bag 路径 |
| `--duration` | 0 | 限定时长（秒），0=播完整 bag |
| `--res` | 0.3 | 地图保存体素分辨率 |

## 输出

- `output/slam_map.pcd` — 地图文件

## 已知问题

`getFullMap()` 当前返回 body-frame 点云而非 world-frame 点云，PCD 地图保存功能需进一步完善。
目前使用预先建好的 `output/mid360_map_filtered.pcd` 供定位使用。
如需重新建图，可使用原工程的 fast_lio ROS2 包或等待此功能修复。
