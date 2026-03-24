# Motion Manager

基于卡尔曼的障碍物跟踪与融合模块，负责接收 `LidarObstacleList`、`LCDObstacleList` 和 `Location`，完成目标跟踪、坐标转换、结果融合，并输出 `MMObstacleList`。

## 本次更新

### 1. Lidar 障碍物绝对坐标转换改为使用同步后的定位姿态

此前主流程虽然会先调用 `SyncStamp()`，但在 `VehicleToWorld()` 中仍然使用当前 `location` 做车体坐标到绝对坐标系的转换。  
现已修改为使用同步后的 `location_current`，保证：

- 障碍物时间戳与用于转换的 GPS/Location pose 对齐
- 大转弯场景下，`center_pos_abs`、`theta_abs` 与地图系更一致

对应修改点：

- `MotionManagerRun()` 中 `VehicleToWorld(obstacle_list_input, location_current)`

### 2. 输出消息头时间和坐标系改为与同步后的 pose 保持一致

`VehicleToWorld()` 中已调整为：

- `header.stamp` 使用参与坐标转换的 `location.header().stamp()`
- `header.frame_id` 使用参与坐标转换的 `location.header().frame_id()`
- 每个障碍物的 `timestamp` 同步写回到同一时刻

这样可以避免出现“内容已经是绝对坐标，但 `frame_id` 仍是旧坐标系”的语义不一致问题。

### 3. LCDObstacleList 新增独立的时间同步姿态选择

此前 `FuseObstacleLists()` 中对 `LCDObstacleList` 的绝对坐标转换直接使用成员变量 `location_`：

- `ConvertPoint(center_pos, location_)`
- `ConvertPoint(pt, location_)`
- `theta_abs = theta_vehicle + location_.heading()`

这会导致 LCD 障碍物在大转弯时使用“当前最新 pose”而不是“与 LCD 时间最接近的 pose”。

现已新增辅助函数：

- `FindSynchronizedLocation(const ObstacleList&, const Location&, Location*)`

用于从 `location_list_` 中查找与目标障碍物时间最接近的 `Location`，并在 `FuseObstacleLists()` 中：

- 为 `LCDObstacleList` 单独选择 `lcd_location`
- 使用 `lcd_location` 完成 `center_pos_abs`、`polygon_point_abs`、`theta_abs` 的转换
- 将 `lcd_obstacle_list_input_` 的 `header.stamp`、`header.frame_id` 和每个障碍物 `timestamp` 对齐到 `lcd_location`

## 背景问题

在问题排查中，发现以下现象：

- `/localization/global_fusion/Location` 时间连续性基本正常
- `/rslidar_points` 在 bag 中存在明显秒级 gap，且 `record time` 与 `header time` 同时不连续
- `MMObstacleList` 曾出现 `header.stamp` 回退
- `LCDObstacleList` 原路径未做与自身时间对应的 pose 同步

这些问题在大转弯场景下会表现为：

- 障碍物在绝对坐标系下与地图对不齐
- 障碍物轮廓随车抖动
- RViz 中世界坐标障碍物显示不稳定

