# 常见问题

## Q1: 建图 segfault

**现象**：建图启动后约 15s 崩溃

**原因**：`FastLIOSAM` 构造函数中未调用 `esekf::init()`，导致 `predict()` 中函数指针为空

**修复**：已在 `fast_lio_sam.cpp` 构造函数末尾增加：
```cpp
state_->init_dyn_runtime_share(get_f, df_dx, df_dw, max_iter, limit);
```
同时 `esekfom.hpp` 的 `predict()` 中增加了 `build_*_state()` 保护。

## Q2: "Failed to find match for field 'normal_x'"

**现象**：建图启动时输出大量字段不匹配 warning

**原因**：livox 点云只有 `x,y,z,intensity` 四个字段，但 map 内部定义 `PointType` 为 `pcl::PointXYZINormal`（8字段）

**修复**：`laserMappingNode.cpp` 中新增 `fromROSMsgWithFallback()` 模板函数，通过 `PointXYZI` 中转并补零。

## Q3: 定位 "No input target dataset was given"

**现象**：hdl_localization 启动后持续报此错误

**原因**：地图文件路径不正确，或 PCD 地图为空/损坏

**解决**：
```bash
ls -lh output/mid360_map_filtered.pcd
# 确认文件存在且 > 1MB
```

## Q4: TF "Could not find a connection between 'lidar' and 'livox_frame'"

**现象**：hdl_localization 启动后 TF 树冲突崩溃

**原因**：launch 中 `lidar_tf` 发布 `odom → lidar`，但点云 frame_id 为 `livox_frame`，两个 frame 不在同一 TF 树

**解决**：将 `odom_child_frame_id` 设为 `livox_frame`
```bash
ros2 launch hdl_localization hdl_localization_livox_test.launch.py \
  odom_child_frame_id:="livox_frame" ...
```

## Q5: GeographicLib cmake 找不到

**现象**：`find_package(GeographicLib) failed`

**解决**：
```bash
sudo apt install libgeographic-dev
```
然后在 CMake 中：
```cmake
set(CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
```

## Q6: colcon 编译 global_localization 失败

**现象**：`nav_msgs/msg/occupancy_grid.hpp` not found

**原因**：CMake 配置缺少 nav_msgs 依赖

**修复**：在 `global_localization/CMakeLists.txt` 中增加：
```cmake
find_package(nav_msgs REQUIRED)
ament_target_dependencies(${PROJECT_NAME}_lib nav_msgs)
```
并删除 `COLCON_IGNORE`。

## Q7: 建图保存地图不成功

**服务调用返回成功但文件未生成**。

当前 `getFullMap()` 返回 body-frame 点云，仅包含最新一帧。完整地图保存需要从 KD-tree 中提取。
临时方案：使用预先建好的 `output/mid360_map_filtered.pcd`。
