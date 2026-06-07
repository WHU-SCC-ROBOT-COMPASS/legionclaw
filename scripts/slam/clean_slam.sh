#!/usr/bin/env bash
# ============================================================
# clean_slam.sh — 清理构建产物
# ============================================================
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MSG_DIR="${ROOT_DIR}/modules/message/ros2"
ROS2_WS_DIR="${ROOT_DIR}/ros2_ws"

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
log() { echo -e "${GREEN}[clean]${NC} $*"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }

confirm() {
  if [ "${1:-}" != "--yes" ]; then
    echo -n "确定清理所有构建产物？[y/N] "
    read -r ans
    [ "${ans}" != "y" ] && [ "${ans}" != "Y" ] && log "已取消" && exit 0
  fi
}

confirm "${1:-}"

log "清理消息包编译产物 ..."
cd "${MSG_DIR}"
rm -rf build install log 2>/dev/null || true
log "  已删除: ${MSG_DIR}/{build,install,log}"

log "清理 ROS2 包编译产物 ..."
cd "${ROS2_WS_DIR}"
rm -rf build install log 2>/dev/null || true
log "  已删除: ${ROS2_WS_DIR}/{build,install,log}"

# 清理残留进程
log "清理残留进程 ..."
for p in legionclaw_map hdl_localization record_odom component_container static_transform; do
  kill $(ps aux | grep "$p" | grep -v grep | awk '{print $2}') 2>/dev/null || true
done
log "  进程已清理"

log "✅ 清理完成"
log "注意: output/ 和 test_result/ 中的数据文件未被删除"
