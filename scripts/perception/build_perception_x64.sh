#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS_DIR="${ROOT_DIR}/modules/message/ros2"
VEHICLE_PROTO="modules/common/data/vehicle_param/proto/vehicle_param.proto"
VEHICLE_PB_H="modules/common/data/vehicle_param/proto/vehicle_param.pb.h"
VEHICLE_PB_CC="modules/common/data/vehicle_param/proto/vehicle_param.pb.cc"

ROS_ENABLE_MODE="OFF"
ROS2_ENABLE_MODE="OFF"
LCM_ENABLE_MODE="OFF"
DDS_ENABLE_MODE="OFF"

usage() {
  cat <<'EOF'
Usage: ./build_perception_x64.sh [options]

Options:
  --ros                  Enable ROS integration
  --ros2                 Enable ROS2 integration
  --lcm                  Enable LCM integration
  --dds                  Enable DDS integration
  --ros2-distro <name>  ROS2 distro to source (optional, auto-detect if omitted)
  -h, --help             Show this help
EOF
}

log() {
  echo "[build_perception_x64] $*"
}

safe_source() {
  local script_path="$1"
  set +u
  # shellcheck disable=SC1090
  source "${script_path}"
  set -u
}

detect_ros2_distro() {
  if [ -n "${ROS2_DISTRO:-}" ]; then
    return 0
  fi

  if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    ROS2_DISTRO="${ROS_DISTRO}"
    return 0
  fi

  local distro
  for distro in jazzy iron humble galactic foxy; do
    if [ -f "/opt/ros/${distro}/setup.bash" ]; then
      ROS2_DISTRO="${distro}"
      return 0
    fi
  done

  return 1
}

update_module_build_flags() {
  local script_path="$1"
  if [ ! -f "${script_path}" ]; then
    log "skip flags update: script not found -> ${script_path}"
    return 0
  fi

  sed -E -i "s/-DROS_ENABLE=(ON|OFF)/-DROS_ENABLE=${ROS_ENABLE_MODE}/g" "${script_path}"
  sed -E -i "s/-DROS2_ENABLE=(ON|OFF)/-DROS2_ENABLE=${ROS2_ENABLE_MODE}/g" "${script_path}"
  sed -E -i "s/-DLCM_ENABLE=(ON|OFF)/-DLCM_ENABLE=${LCM_ENABLE_MODE}/g" "${script_path}"
  sed -E -i "s/-DDDS_ENABLE=(ON|OFF)/-DDDS_ENABLE=${DDS_ENABLE_MODE}/g" "${script_path}"

  log "updated flags in ${script_path}"
}

apply_module_build_flags() {
  update_module_build_flags "${ROOT_DIR}/modules/perception/lidar/lidar_ground_segmentation/scripts/build.x64.sh"
  update_module_build_flags "${ROOT_DIR}/modules/perception/lidar/lidar_cluster_detect/scripts/build.x64.sh"
  update_module_build_flags "${ROOT_DIR}/modules/perception/lidar/multi_lidar_splicing/scripts/build.x64.sh"
  update_module_build_flags "${ROOT_DIR}/modules/perception/fusion/motion_manager/scripts/build.x64.sh"
}

run_script() {
  local script_path="$1"
  if [ ! -f "${script_path}" ]; then
    log "skip: script not found -> ${script_path}"
    return 0
  fi

  local script_dir
  script_dir="$(dirname "${script_path}")"
  local module_dir
  module_dir="${script_dir}"
  local invoke_path
  invoke_path="./$(basename "${script_path}")"
  if [ "$(basename "${script_dir}")" = "scripts" ]; then
    module_dir="$(dirname "${script_dir}")"
    invoke_path="./scripts/$(basename "${script_path}")"
  fi

  log "run: ${script_path}"
  (
    cd "${module_dir}"
    bash "${invoke_path}"
  )
}

generate_vehicle_proto() {
  local proto_abs="${ROOT_DIR}/${VEHICLE_PROTO}"
  local pb_h_abs="${ROOT_DIR}/${VEHICLE_PB_H}"
  local pb_cc_abs="${ROOT_DIR}/${VEHICLE_PB_CC}"

  if [ ! -f "${proto_abs}" ]; then
    log "error: proto file not found -> ${proto_abs}"
    return 1
  fi

  if [ -f "${pb_h_abs}" ] && [ -f "${pb_cc_abs}" ] \
    && [ "${pb_h_abs}" -nt "${proto_abs}" ] && [ "${pb_cc_abs}" -nt "${proto_abs}" ]; then
    log "protobuf already up to date: ${VEHICLE_PROTO}"
    return 0
  fi

  local protoc_bin=""
  if [ -x /usr/local/legion/third_party/x64/bin/protoc ]; then
    protoc_bin="/usr/local/legion/third_party/x64/bin/protoc"
    export LD_LIBRARY_PATH="/usr/local/legion/third_party/x64/lib/protobuf:${LD_LIBRARY_PATH:-}"
  elif command -v protoc >/dev/null 2>&1; then
    protoc_bin="$(command -v protoc)"
  else
    log "error: protoc not found"
    return 1
  fi

  log "generate protobuf: ${VEHICLE_PROTO}"
  (
    cd "${ROOT_DIR}"
    "${protoc_bin}" -I . --cpp_out=. "${VEHICLE_PROTO}"
  )
}

while [ $# -gt 0 ]; do
  case "$1" in
    --ros)
      ROS_ENABLE_MODE="ON"
      shift
      ;;
    --ros2)
      ROS2_ENABLE_MODE="ON"
      shift
      ;;
    --lcm)
      LCM_ENABLE_MODE="ON"
      shift
      ;;
    --dds)
      DDS_ENABLE_MODE="ON"
      shift
      ;;
    --ros2-distro)
      ROS2_DISTRO="${2:-}"
      if [ -z "${ROS2_DISTRO}" ]; then
        log "error: --ros2-distro requires a value"
        exit 1
      fi
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      log "error: unknown arg -> $1"
      usage
      exit 1
      ;;
  esac
done

export ROS_ENABLE="${ROS_ENABLE_MODE}"
export ROS2_ENABLE="${ROS2_ENABLE_MODE}"
export LCM_ENABLE="${LCM_ENABLE_MODE}"
export DDS_ENABLE="${DDS_ENABLE_MODE}"
export GLOG_ENABLE="ON"

log "cmake flags: ROS_ENABLE=${ROS_ENABLE} ROS2_ENABLE=${ROS2_ENABLE} LCM_ENABLE=${LCM_ENABLE} DDS_ENABLE=${DDS_ENABLE} GLOG_ENABLE=${GLOG_ENABLE}"

if [ "${ROS2_ENABLE}" = "ON" ]; then
  if ! detect_ros2_distro; then
    log "error: failed to detect ROS2 distro, use --ros2-distro <name>"
    exit 1
  fi
  log "build ros2 messages: ${ROS2_WS_DIR} (ROS2_DISTRO=${ROS2_DISTRO})"
  safe_source "/opt/ros/${ROS2_DISTRO}/setup.bash"
  (
    cd "${ROS2_WS_DIR}"
    colcon build \
      --allow-overriding rosbridge_msgs \
      --cmake-args \
        -DPYTHON_EXECUTABLE=/usr/bin/python3 \
        -DPython3_EXECUTABLE=/usr/bin/python3
  )
  safe_source "${ROS2_WS_DIR}/install/setup.bash"
else
  log "skip ros2 messages build (ROS2_ENABLE=OFF)"
fi

apply_module_build_flags

generate_vehicle_proto

# run_script "${ROOT_DIR}/modules/perception/lidar/lidar_ground_segmentation/scripts/build.x64.sh"
# run_script "${ROOT_DIR}/modules/perception/lidar/lidar_cluster_detect/scripts/build.x64.sh"
run_script "${ROOT_DIR}/modules/perception/lidar/multi_lidar_splicing/scripts/build.x64.sh"

# run_script "${ROOT_DIR}/modules/perception/lidar/lidar_detect_voxel/scripts/build.x64.sh"

# run_script "${ROOT_DIR}/modules/perception/fusion/motion_manager/scripts/build.x64.sh"

log "all done"
