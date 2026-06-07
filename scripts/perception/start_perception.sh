#!/bin/bash

PROJECT_ROOT="/home/BEACON/project/legionclaw"

echo "=========================================="
echo "启动 Perception 模块"
echo "=========================================="

# 后台启动 multi_lidar_splicing
echo "[1/4] 启动 multi_lidar_splicing..."
(
    cd "${PROJECT_ROOT}/modules/perception/lidar/multi_lidar_splicing"
    bash scripts/start.sh
) &
PID_MLS=$!

# 后台启动 lidar_ground_segmentation
echo "[2/4] 启动 lidar_ground_segmentation..."
(
    cd "${PROJECT_ROOT}/modules/perception/lidar/lidar_ground_segmentation"
    bash scripts/start.sh
) &
PID_GS=$!

# 等待一下确保前两个模块启动
sleep 2

# 后台启动 lidar_cluster_detect
echo "[3/4] 启动 lidar_cluster_detect..."
(
    cd "${PROJECT_ROOT}/modules/perception/lidar/lidar_cluster_detect"
    bash scripts/start.sh
) &
PID_CD=$!

# 等待一下
sleep 2

# 后台启动 motion_manager
echo "[4/4] 启动 motion_manager..."
(
    cd "${PROJECT_ROOT}/modules/perception/fusion/motion_manager"
    bash scripts/start.sh
) &
PID_MM=$!

echo ""
echo "=========================================="
echo "所有模块已启动"
echo "=========================================="
echo "multi_lidar_splicing:        PID=$PID_MLS"
echo "lidar_ground_segmentation:   PID=$PID_GS"
echo "lidar_cluster_detect:        PID=$PID_CD"
echo "motion_manager:              PID=$PID_MM"
echo ""
echo "停止命令: kill $PID_MLS $PID_GS $PID_CD $PID_MM"
echo "=========================================="

# 等待所有后台进程
wait