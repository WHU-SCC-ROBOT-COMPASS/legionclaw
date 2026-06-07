#!/usr/bin/env bash
# ============================================================
# build_slam.sh — 编译 LegionClaw 建图和定位 ROS2 包
# ============================================================
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MSG_DIR="${ROOT_DIR}/modules/message/ros2"
ROS2_WS_DIR="${ROOT_DIR}/ros2_ws"

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
log() { echo -e "${GREEN}[build]${NC} $*"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }
err() { echo -e "${RED}[ERROR]${NC} $*"; }

# 检查 ROS2 环境
if [ ! -f /opt/ros/humble/setup.bash ]; then
  err "ROS2 Humble 未安装: /opt/ros/humble/setup.bash 不存在"
  exit 1
fi

source /opt/ros/humble/setup.bash

# Step 1: 编译消息包
log "Step 1/3: 编译消息包 (ros2_interface, ros2_slam_msgs) ..."
cd "${MSG_DIR}"
colcon build --allow-overriding rosbridge_msgs 2>&1 | sed 's/^/  [colcon] /'
source install/setup.bash
log "✅ 消息包编译完成"

# Step 2: 编译 ROS2 定位包
log "Step 2/3: 编译定位包 (hdl_localization, ndt_omp, fast_gicp, global_localization) ..."
cd "${ROS2_WS_DIR}"

# 删除 COLCON_IGNORE（原工程默认忽略 global_localization）
if [ -f src/global_localization/COLCON_IGNORE ]; then
  rm src/global_localization/COLCON_IGNORE
  log "  已删除 global_localization/COLCON_IGNORE"
fi

colcon build --symlink-install 2>&1 | sed 's/^/  [colcon] /'
source install/setup.bash
log "✅ 定位包编译完成"

# Step 3: 编译 map colcon 包
log "Step 3/3: 编译建图包 (legionclaw_map) ..."
source "${MSG_DIR}/install/setup.bash"
colcon build --packages-select legionclaw_map --symlink-install 2>&1 | sed 's/^/  [colcon] /'
log "✅ 建图包编译完成"

log "=========================================="
log "全部编译完成：消息包(3) + 定位包(7) + 建图包(1)"
log "共 11 个 ROS2 包"
log "=========================================="
