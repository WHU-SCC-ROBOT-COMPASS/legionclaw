# params.json 参数文件字段说明
## 1. 功能开关类参数（Boolean Flags）

| 参数名 | 类型 | 默认值 | 说明 |
|--------|--------|---------|--------|
| verbose | bool | false | 是否输出详细调试日志 |
| enable_RNR | bool | true | 是否启用反射噪声去除（Reflected Noise Removal） |
| enable_RVPF | bool | true | 是否启用区域垂直平面拟合（Region-wise Vertical Plane Fitting） |
| enable_TGR | bool | true | 是否启用时间地面恢复（Temporal Ground Revert） |

## 2. 迭代与数量参数（Iteration & Count）

| 参数名 | 类型 | 默认值 | 说明 |
|--------|--------|---------|--------|
| num_iter | int | 3 | PCA 地面拟合的迭代次数 |
| num_lpr | int | 20 | 初始种子选择中最低点代表（LPR）最大数量 |
| num_min_pts | int | 10 | 每个 patch 最小用于地面估计的点数 |
| num_zones | int | 4 | 同心区域模型（CZM）的 zone 数量 |
| num_rings_of_interest | int | 4 | 用于检查高程和平坦度的 ring 数量 |

## 3. 反射噪声去除（RNR）参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|--------|---------|--------|
| RNR_ver_angle_thr | double | -15.0 | 垂直角度阈值（°），向下激光更容易生成噪声点 |
| RNR_intensity_thr | double | 0.2 | 强度阈值，反射噪声点通常强度较低 |

## 4. 传感器与距离参数（Sensor & Range）

| 参数名 | 类型 | 默认值 | 说明 |
|--------|--------|---------|--------|
| sensor_height | double | 1.723 | 激光传感器高度（米） |
| max_range | double | 80.0 | 地面估计处理的最大距离 |
| min_range | double | 2.7 | 地面估计处理的最小距离 |

## 5. 阈值类参数（Thresholds）

| 参数名 | 类型 | 默认值 | 说明 |
|--------|--------|---------|--------|
| th_seeds | double | 0.125 | 初始 LPR 种子的阈值 |
| th_dist | double | 0.125 | 地面厚度阈值 |
| th_seeds_v | double | 0.25 | 垂直结构种子选择阈值 |
| th_dist_v | double | 0.1 | 垂直结构厚度阈值 |
| uprightness_thr | double | 0.707 | GLE 中直立度判断阈值 |
| adaptive_seed_selection_margin | double | -1.2 | 自适应初始种子选择的 margin |
| intensity_thr | double | 未初始化 | 代码中定义但构造函数未设置 |

## 6. 同心区域模型（CZM）配置

| 参数名 | 类型 | 默认值 | 说明 |
|--------|--------|---------|--------|
| num_sectors_each_zone | vector<int> | {16, 32, 54, 32} | 每个 zone 的扇区数量 |
| num_rings_each_zone | vector<int> | {2, 4, 4, 4} | 每个 zone 的 ring 数量 |

## 7. 自适应阈值与存储（Adaptive Thresholds & Buffers）

| 参数名 | 类型 | 默认值 | 说明 |
|--------|:-------|---------|--------|
| max_flatness_storage | int | 1000 | 平坦度存储最大数量 |
| max_elevation_storage | int | 1000 | 高程存储最大数量 |
| elevation_thr | vector<double> | {0,0,0,0} | 每个 ring 的高程阈值（可自适应更新） |
| flatness_thr | vector<double> | {0,0,0,0} | 每个 ring 的平坦度阈值（可自适应更新） |

## 参数分类总结

| 分类 | 含义 |
|--------|--------|
| 功能开关 | 控制是否启用 RNR、RVPF、TGR |
| 迭代与数量 | 控制 PCA / patch 区域处理规模 |
| 反射噪声 | 删除低强度、角度异常的噪声点 |
| 传感器/距离 | 控制 min/max range 的处理范围 |
| 阈值参数 | 控制地面分类的严格程度 |
| CZM 配置 | 控制同心区域模型的分区方式 |
| 自适应阈值 | 动态更新 GLE 相关判断阈值 |
