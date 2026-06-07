#!/usr/bin/env bash
# ============================================================
# run_mapping.sh — 建图（播完整 bag → 输出 PCD 地图）
#
# 用法:
#   ./run_mapping.sh                          # 默认建图，rate=1.0
#   ./run_mapping.sh --rate 2.0               # 加速播放
#   ./run_mapping.sh --bag <path>              # 指定 bag
#   ./run_mapping.sh --duration 60             # 只播前 60 秒
# ============================================================
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DATA_DIR="${ROOT_DIR}/data/bag_files"
OUTPUT_DIR="${ROOT_DIR}/output"
MSG_DIR="${ROOT_DIR}/modules/message/ros2"
ROS2_WS_DIR="${ROOT_DIR}/ros2_ws"

# 默认参数
MAP_BAG="${DATA_DIR}/lidar_20260326_013648_0.db3"
RATE=1.0
DURATION=0   # 0 = 播完整 bag
SAVE_RES=0.3

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; CYAN='\033[0;36m'; NC='\033[0m'
log() { echo -e "${GREEN}[mapping]${NC} $*"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }
err() { echo -e "${RED}[ERROR]${NC} $*"; }
info() { echo -e "${CYAN}[INFO]${NC} $*"; }

# 解析参数
while [ $# -gt 0 ]; do
  case "$1" in
    --rate) RATE="$2"; shift 2 ;;
    --bag) MAP_BAG="$2"; shift 2 ;;
    --duration) DURATION="$2"; shift 2 ;;
    --res) SAVE_RES="$2"; shift 2 ;;
    -h|--help)
      echo "用法: $0 [--rate <rate>] [--bag <path>] [--duration <sec>] [--res <resolution>]"
      exit 0 ;;
    *) err "未知参数: $1"; exit 1 ;;
  esac
done

# 检查环境
source /opt/ros/humble/setup.bash
if [ -f "${MSG_DIR}/install/setup.bash" ]; then source "${MSG_DIR}/install/setup.bash"; fi
if [ -f "${ROS2_WS_DIR}/install/setup.bash" ]; then source "${ROS2_WS_DIR}/install/setup.bash"; fi

if [ ! -f "${MAP_BAG}" ]; then
  err "建图 bag 不存在: ${MAP_BAG}"
  exit 1
fi
if ! command -v ros2 &>/dev/null; then
  err "ros2 命令不可用，请先编译 (./build_slam.sh)"
  exit 1
fi

mkdir -p "${OUTPUT_DIR}"

# Bag 信息
BAG_DUR=$(ros2 bag info "${MAP_BAG}" 2>/dev/null | grep "Duration" | awk '{print $2}' | sed 's/s//')
log "Bag: ${MAP_BAG}"
log "时长: ${BAG_DUR}s, 播放速率: ${RATE}x"
if [ "${DURATION}" -gt 0 ]; then
  log "限定时长: ${DURATION}s（实际播放 $(echo "scale=1; ${DURATION}/${RATE}" | bc)s）"
fi

# 清理残留
for p in legionclaw_map component_container; do
  kill $(ps aux | grep "$p" | grep -v grep | awk '{print $2}') 2>/dev/null || true
done
sleep 2

# 启动建图节点
log "启动建图节点 ..."
ros2 run legionclaw_map legionclaw_map &
MAP_PID=$!
sleep 5

# 播放 bag
log "播放 bag ..."
if [ "${DURATION}" -gt 0 ]; then
  timeout $(echo "scale=0; ${DURATION}/${RATE} + 5" | bc) \
    ros2 bag play "${MAP_BAG}" --clock 100 --rate "${RATE}" || true
else
  ros2 bag play "${MAP_BAG}" --clock 100 --rate "${RATE}"
fi

log "Bag 播放结束，等待建图收尾 ..."
sleep 3

# 保存地图
log "保存地图到 ${OUTPUT_DIR}/ ..."
timeout 8 ros2 service call /save_map ros2_slam_msgs/srv/SaveMap \
  "{destination: \"${OUTPUT_DIR}\", resolution: ${SAVE_RES}}" 2>&1 || \
  warn "地图保存服务调用超时"

sleep 2

# 停止建图
kill $MAP_PID 2>/dev/null || true
sleep 1

# 检查输出
if [ -f "${OUTPUT_DIR}/slam_map.pcd" ]; then
  PTS=$(sed -n '/^POINTS/s/.* //p' "${OUTPUT_DIR}/slam_map.pcd")
  log "✅ 建图完成: ${OUTPUT_DIR}/slam_map.pcd  (${PTS} 点, $(du -h "${OUTPUT_DIR}/slam_map.pcd" | cut -f1))"
else
  warn "地图文件未生成（slam_map.pcd 不存在）"
  warn "提示: getFullMap() 当前返回 body-frame 点云，保存逻辑需后续完善"
  warn "可继续使用已有地图: ${OUTPUT_DIR}/mid360_map_filtered.pcd"
fi
