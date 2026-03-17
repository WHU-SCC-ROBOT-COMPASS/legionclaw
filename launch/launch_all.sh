#!/usr/bin/env bash
set -euo pipefail

# One-stop launcher for perception binaries.
# Starts lidar_ground_segmentation, lidar_cluster_detect, and motion_manager with logging.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
MESSAGE_WS_SETUP="${ROOT_DIR}/modules/message/ros2/install/setup.bash"

# Optional: force Cyclone DDS when shared memory/UDP sockets are restricted.
if [[ "${USE_CYCLONEDDS:-0}" == "1" ]]; then
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
fi

export ROS_LOG_DIR="${ROS_LOG_DIR:-${ROOT_DIR}/log/ros}"
LOG_DIR="${LOG_DIR:-${ROOT_DIR}/log/run}"
mkdir -p "${ROS_LOG_DIR}" "${LOG_DIR}"

# Helper to source setup files without being killed by set -e/-u.
safe_source() {
  local script_path="$1"
  if [[ ! -f "${script_path}" ]]; then
    echo "WARN: ${script_path} not found."
    return
  fi
  local prev_e=$-
  set +eu
  # shellcheck source=/dev/null
  source "${script_path}"
  # restore -e/-u if they were set
  [[ "${prev_e}" == *e* ]] && set -e
  [[ "${prev_e}" == *u* ]] && set -u
}

safe_source "${ROS_SETUP}"
safe_source "${MESSAGE_WS_SETUP}"

declare -a PIDS=()

start_node() {
  local name="$1"; shift
  local cmd="$1"; shift
  local logfile="${LOG_DIR}/${name}.log"

  if [[ ! -x "${cmd}" ]]; then
    echo "ERROR: ${cmd} not found or not executable. Please build first."
    exit 1
  fi

  echo "Starting ${name} -> ${logfile}"
  (cd "$(dirname "${cmd}")" && "./$(basename "${cmd}")" "$@") > "${logfile}" 2>&1 &
  local pid=$!
  PIDS+=("${pid}")
  echo "${name} PID ${pid}"
}

trap 'echo "Stopping nodes: ${PIDS[*]}"; kill "${PIDS[@]}" 2>/dev/null || true' EXIT

start_node "lidar_ground_segmentation" "${ROOT_DIR}/modules/perception/lidar/lidar_ground_segmentation/bin/lidar_ground_segmentation" --ros-args -r __node:=lidar_ground_segmentation
start_node "lidar_cluster_detect" "${ROOT_DIR}/modules/perception/lidar/lidar_cluster_detect/bin/lidar_cluster_detect" --ros-args -r __node:=lidar_cluster_detect
start_node "motion_manager" "${ROOT_DIR}/modules/perception/fusion/motion_manager/bin/motion_manager" --ros-args -r __node:=motion_manager

echo "All nodes launched. Logs under ${LOG_DIR}; ROS logs under ${ROS_LOG_DIR}."
wait
