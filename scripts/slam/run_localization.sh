#!/usr/bin/env bash
# ============================================================
# run_localization.sh — 定位（加载地图 → 播 bag → 记录 odom）
#
# 用法:
#   ./run_localization.sh                              # 默认定位
#   ./run_localization.sh --map <path>                  # 指定地图 PCD
#   ./run_localization.sh --bag <path>                  # 指定 bag
#   ./run_localization.sh --rate 3.0                    # 播放速率
#   ./run_localization.sh --duration 60                 # 只播前 60 秒
#   ./run_localization.sh --odom-child "livox_frame"    # odom child frame
# ============================================================
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DATA_DIR="${ROOT_DIR}/data/bag_files"
OUTPUT_DIR="${ROOT_DIR}/output"
TEST_DIR="${ROOT_DIR}/test_result"
MSG_DIR="${ROOT_DIR}/modules/message/ros2"
ROS2_WS_DIR="${ROOT_DIR}/ros2_ws"

# 默认参数
LOC_BAG="${DATA_DIR}/lidar_20260326_014243_0.db3"
GLOBALMAP="${OUTPUT_DIR}/mid360_map_filtered.pcd"
RATE=3.0
DURATION=0
ODOM_CHILD_FRAME="livox_frame"

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; CYAN='\033[0;36m'; NC='\033[0m'
log() { echo -e "${GREEN}[localization]${NC} $*"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }
err() { echo -e "${RED}[ERROR]${NC} $*"; }
info() { echo -e "${CYAN}[INFO]${NC} $*"; }

# 解析参数
while [ $# -gt 0 ]; do
  case "$1" in
    --map) GLOBALMAP="$2"; shift 2 ;;
    --bag) LOC_BAG="$2"; shift 2 ;;
    --rate) RATE="$2"; shift 2 ;;
    --duration) DURATION="$2"; shift 2 ;;
    --odom-child) ODOM_CHILD_FRAME="$2"; shift 2 ;;
    -h|--help)
      echo "用法: $0 [--map <path>] [--bag <path>] [--rate <rate>] [--duration <sec>] [--odom-child <frame>]"
      exit 0 ;;
    *) err "未知参数: $1"; exit 1 ;;
  esac
done

# 环境
source /opt/ros/humble/setup.bash
if [ -f "${ROS2_WS_DIR}/install/setup.bash" ]; then source "${ROS2_WS_DIR}/install/setup.bash"; fi

# 检查依赖
if [ ! -f "${GLOBALMAP}" ]; then
  err "地图文件不存在: ${GLOBALMAP}"
  warn "请先建图或指定地图路径: --map <path>"
  exit 1
fi
if [ ! -f "${LOC_BAG}" ]; then
  err "定位 bag 不存在: ${LOC_BAG}"
  exit 1
fi

mkdir -p "${TEST_DIR}"

# Bag 信息
BAG_DUR=$(ros2 bag info "${LOC_BAG}" 2>/dev/null | grep "Duration" | awk '{print $2}' | sed 's/s//')
log "Bag: ${LOC_BAG}"
log "时长: ${BAG_DUR}s, 播放速率: ${RATE}x"
if [ "${DURATION}" -gt 0 ]; then
  log "限定时长: ${DURATION}s"
fi
log "地图: ${GLOBALMAP}"
log "odom_child_frame_id: ${ODOM_CHILD_FRAME}"

# 清理残留
for p in hdl_localization record_odom component_container static_transform; do
  kill $(ps aux | grep "$p" | grep -v grep | awk '{print $2}') 2>/dev/null || true
done
sleep 3

ODOM_FILE="${TEST_DIR}/localization_$(date +%Y%m%d_%H%M%S).txt"

# 启动 hdl_localization
log "启动 hdl_localization ..."
ros2 launch hdl_localization hdl_localization_livox_test.launch.py \
  globalmap_pcd:="${GLOBALMAP}" \
  use_sim_time:=true \
  odom_child_frame_id:="${ODOM_CHILD_FRAME}" > "${TEST_DIR}/localization_log.txt" 2>&1 &
sleep 8

# 启动 odom 记录器
log "启动 odom 记录器 -> ${ODOM_FILE} ..."
python3 "${SCRIPTS_DIR}/record_odom.py" "${ODOM_FILE}" &
REC_PID=$!
sleep 2

# 播放定位 bag
log "播放定位 bag ..."
if [ "${DURATION}" -gt 0 ]; then
  timeout $(echo "scale=0; ${DURATION}/${RATE} + 5" | bc) \
    ros2 bag play "${LOC_BAG}" --clock 100 --rate "${RATE}" --topics /livox/lidar /livox/imu || true
else
  ros2 bag play "${LOC_BAG}" --clock 100 --rate "${RATE}" --topics /livox/lidar /livox/imu
fi

log "Bag 播放结束，等待数据刷入 ..."
sleep 5

# 停止
kill $REC_PID 2>/dev/null || true
for p in hdl_localization component_container; do
  kill $(ps aux | grep "$p" | grep -v grep | awk '{print $2}') 2>/dev/null || true
done
sleep 2

# 结果统计
echo ""
info "=========================================="
info "定位结果"
info "=========================================="

if [ -f "${ODOM_FILE}" ]; then
  COUNT=$(wc -l < "${ODOM_FILE}")
  if [ "${COUNT}" -gt 1 ]; then
    log "✅ odom 记录: ${COUNT} 条"
    log "文件: ${ODOM_FILE}"

    # 绘图
    log "生成轨迹图 ..."
    python3 "${SCRIPTS_DIR}/plot_trajectory.py" "${ODOM_FILE}" "Legionclaw hdl_localization"
    log "轨迹图: ${ODOM_FILE%.txt}_2d.png"
    log "       ${ODOM_FILE%.txt}_3d.png"
    log "       ${ODOM_FILE%.txt}_stats.png"

    # 统计
    python3 -c "
import numpy as np
d = np.loadtxt('${ODOM_FILE}', skiprows=1)
if len(d) > 1:
    x, y, z = d[:,1], d[:,2], d[:,3]
    print(f'  时间跨度: {d[-1,0]-d[0,0]:.1f}s')
    print(f'  位移: {np.linalg.norm([x[-1]-x[0], y[-1]-y[0], z[-1]-z[0]]):.2f}m')
    print(f'  X: [{x.min():.1f}, {x.max():.1f}] = {x.max()-x.min():.1f}m')
    print(f'  Y: [{y.min():.1f}, {y.max():.1f}] = {y.max()-y.min():.1f}m')
    print(f'  起点: ({x[0]:.2f}, {y[0]:.2f}, {z[0]:.2f})')
    print(f'  终点: ({x[-1]:.2f}, {y[-1]:.2f}, {z[-1]:.2f})')
"
  else
    warn "odom 记录不足（仅 ${COUNT} 行）"
    warn "请检查:"
    warn "  1. hdl_localization 是否启动成功 (查看 ${TEST_DIR}/localization_log.txt)"
    warn "  2. 地图路径是否正确: ${GLOBALMAP}"
    warn "  3. odom_child_frame_id 是否匹配点云 frame_id: ${ODOM_CHILD_FRAME}"
  fi
else
  err "odom 文件未生成"
fi
