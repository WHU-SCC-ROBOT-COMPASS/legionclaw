# 定位流程

## 流程

```
预建 PCD 地图 + bag 点云  →  hdl_localization(NDT)  →  odom 轨迹
```

## 启动

```bash
cd /path/to/legionclaw

# 首次：编译
./scripts/slam/build_slam.sh

# 定位（播完整定位 bag）
./scripts/slam/run_localization.sh
```

## 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--map` | `output/mid360_map_filtered.pcd` | 预建地图路径 |
| `--bag` | `data/bag_files/lidar_20260326_014243_0.db3` | 定位 bag 路径 |
| `--rate` | 3.0 | bag 播放速率 |
| `--duration` | 0 | 限定时长，0=完整 |
| `--odom-child` | `livox_frame` | odom 子坐标系 |

## 输出

```
test_result/
├── localization_20260607_093000.txt         — odom 轨迹（时间 x y z qx qy qz qw）
├── localization_20260607_093000_2d.png      — 二维轨迹图
├── localization_20260607_093000_3d.png      — 三维轨迹图
├── localization_20260607_093000_stats.png   — 统计图（位置/速度/高度）
└── localization_log.txt                     — 定位日志
```

## 输出格式

odom 文本格式（空格分隔）：
```
# timestamp sec x y z qx qy qz qw
1774460563.939338 0.035 0.437 0.011 0.001 -0.004 -0.005 0.999
1774460564.039291 0.052 0.794 0.024 0.001 -0.005 -0.004 0.999
...
```

## 关键参数

- `odom_child_frame_id="livox_frame"` — 必须与 livox 点云的 frame_id 匹配
- NDT 分辨率：1.0m
- 下采样分辨率：0.5m
