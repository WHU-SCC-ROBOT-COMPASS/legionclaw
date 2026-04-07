#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${SCRIPT_DIR}"

# ROS2 workspace
ROS2_WS_DIR="${PROJECT_ROOT}/modules/message/ros2"

# Module paths
MULTI_LIDAR_SPLICING_DIR="${PROJECT_ROOT}/modules/perception/lidar/multi_lidar_splicing"
LIDAR_GROUND_SEGMENTATION_DIR="${PROJECT_ROOT}/modules/perception/lidar/lidar_ground_segmentation"
LIDAR_CLUSTER_DETECT_DIR="${PROJECT_ROOT}/modules/perception/lidar/lidar_cluster_detect"
MOTION_MANAGER_DIR="${PROJECT_ROOT}/modules/perception/fusion/motion_manager"

# Executables
MULTI_LIDAR_SPLICING_BIN="${MULTI_LIDAR_SPLICING_DIR}/bin/multi_lidar_splicing"
LIDAR_GROUND_SEGMENTATION_BIN="${LIDAR_GROUND_SEGMENTATION_DIR}/bin/lidar_ground_segmentation"
LIDAR_CLUSTER_DETECT_BIN="${LIDAR_CLUSTER_DETECT_DIR}/bin/lidar_cluster_detect"
MOTION_MANAGER_BIN="${MOTION_MANAGER_DIR}/bin/motion_manager"

# Log directory
LOG_DIR="${PROJECT_ROOT}/logs"
mkdir -p "${LOG_DIR}"

# Default: start all modules
START_ALL=true
START_MULTI_LIDAR=false
START_GROUND_SEG=false
START_CLUSTER=false
START_MOTION=false

# Process IDs for cleanup
PIDS=()

log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*"
}

log_error() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] ERROR: $*" >&2
}

cleanup() {
    log "Shutting down..."
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -SIGTERM "$pid" 2>/dev/null || true
            wait "$pid" 2>/dev/null || true
        fi
    done
    log "All processes stopped."
    exit 0
}

trap cleanup SIGINT SIGTERM

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

source_ros2_environment() {
    log "Detecting ROS2 environment..."

    if ! detect_ros2_distro; then
        log_error "Failed to detect ROS2 distro. Please source ROS2 manually or set ROS2_DISTRO."
        exit 1
    fi

    log "Using ROS2 distro: ${ROS2_DISTRO}"

    # Temporarily disable nounset for ROS2 setup scripts
    set +u
    # shellcheck disable=SC1091
    source "/opt/ros/${ROS2_DISTRO}/setup.bash"

    # Source project's ROS2 message workspace if exists
    if [ -f "${ROS2_WS_DIR}/install/setup.bash" ]; then
        # shellcheck disable=SC1091
        source "${ROS2_WS_DIR}/install/setup.bash"
        log "Sourced project ROS2 workspace"
    fi
    set -u

    # Explicitly set RMW implementation to FastRTPS to match the built-in typesupport
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    log "Set RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
}

check_binary() {
    local binary="$1"
    local name="$2"
    if [ ! -f "$binary" ]; then
        log_error "${name} binary not found: ${binary}"
        log_error "Please build the project first: ./build_perception_arm.sh --ros2"
        return 1
    fi

    # Check architecture compatibility
    local binary_arch
    binary_arch=$(file -b "$binary" 2>/dev/null | grep -o 'ARM\|x86-64\|aarch64' || echo "unknown")
    local system_arch
    system_arch=$(uname -m)

    if [[ "$binary" == *"arm64"* ]] || [[ "$binary_arch" == *"ARM"* ]] || [[ "$binary_arch" == *"aarch64"* ]]; then
        if [[ "$system_arch" == *"aarch64"* ]]; then
            : # ARM binary on ARM system - OK
        fi
    elif [[ "$binary_arch" == *"x86-64"* ]]; then
        if [[ "$system_arch" == *"x86_64"* ]] || [[ "$system_arch" == *"x64"* ]]; then
            : # x86-64 binary on x86-64 system - OK
        else
            log_error "${name} binary is compiled for x86-64 but system is ${system_arch}"
            log_error "Please rebuild: cd ${PROJECT_ROOT} && ./build_perception_arm.sh --ros2"
            return 1
        fi
    fi

    if [ ! -x "$binary" ]; then
        chmod +x "$binary"
    fi
    return 0
}

start_module() {
    local name="$1"
    local binary="$2"
    local dir="$3"
    local log_file="${LOG_DIR}/${name}.log"

    log "Starting ${name}..."
    log "  Binary: ${binary}"
    log "  Working dir: ${dir}"
    log "  Log: ${log_file}"

    # Create symlink for config if needed (some programs look for configs relative to project root)
    if [ -d "${PROJECT_ROOT}/${name}/bin/conf" ] && [ ! -L "${PROJECT_ROOT}/conf" ]; then
        ln -sf "${PROJECT_ROOT}/${name}/bin/conf" "${PROJECT_ROOT}/conf"
    fi

    (
        cd "${dir}"
        "${binary}" >> "${log_file}" 2>&1
    ) &

    local pid=$!
    PIDS+=("$pid")
    log "${name} started (PID: ${pid})"
}

start_multi_lidar_splicing() {
    if ! check_binary "${MULTI_LIDAR_SPLICING_BIN}" "multi_lidar_splicing"; then
        return 1
    fi
    start_module "multi_lidar_splicing" "${MULTI_LIDAR_SPLICING_BIN}" "${MULTI_LIDAR_SPLICING_DIR}/bin"
}

start_lidar_ground_segmentation() {
    if ! check_binary "${LIDAR_GROUND_SEGMENTATION_BIN}" "lidar_ground_segmentation"; then
        return 1
    fi
    start_module "lidar_ground_segmentation" "${LIDAR_GROUND_SEGMENTATION_BIN}" "${LIDAR_GROUND_SEGMENTATION_DIR}/bin"
}

start_lidar_cluster_detect() {
    if ! check_binary "${LIDAR_CLUSTER_DETECT_BIN}" "lidar_cluster_detect"; then
        return 1
    fi
    start_module "lidar_cluster_detect" "${LIDAR_CLUSTER_DETECT_BIN}" "${LIDAR_CLUSTER_DETECT_DIR}/bin"
}

start_motion_manager() {
    if ! check_binary "${MOTION_MANAGER_BIN}" "motion_manager"; then
        return 1
    fi
    start_module "motion_manager" "${MOTION_MANAGER_BIN}" "${MOTION_MANAGER_DIR}/bin"
}

usage() {
    cat <<EOF
Usage: $(basename "$0") [options]

Start perception modules for legionclaw project.

Options:
  --all              Start all modules (default)
  --multi-lidar      Start multi_lidar_splicing only
  --ground           Start lidar_ground_segmentation only
  --cluster          Start lidar_cluster_detect only
  --motion           Start motion_manager only
  --ros2-distro      ROS2 distro to use (auto-detect if not specified)
  -h, --help         Show this help message

Examples:
  $(basename "$0")                 # Start all modules
  $(basename "$0") --multi-lidar  # Start only multi_lidar_splicing
  $(basename "$0") --ground --cluster  # Start ground segmentation and cluster detect

EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --all)
            START_ALL=true
            START_MULTI_LIDAR=true
            START_GROUND_SEG=true
            START_CLUSTER=true
            START_MOTION=true
            shift
            ;;
        --multi-lidar)
            START_MULTI_LIDAR=true
            START_ALL=false
            shift
            ;;
        --ground)
            START_GROUND_SEG=true
            START_ALL=false
            shift
            ;;
        --cluster)
            START_CLUSTER=true
            START_ALL=false
            shift
            ;;
        --motion)
            START_MOTION=true
            START_ALL=false
            shift
            ;;
        --ros2-distro)
            ROS2_DISTRO="${2:-}"
            if [ -z "${ROS2_DISTRO}" ]; then
                log_error "--ros2-distro requires a value"
                exit 1
            fi
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            log_error "Unknown option: $1"
            usage
            exit 1
            ;;
    esac
done

# Main
log "========================================="
log "LegionClaw Perception Launch Script"
log "========================================="

source_ros2_environment

# Set library paths for custom OpenCV 4.1 (compiled against this version)
export LD_LIBRARY_PATH="${PROJECT_ROOT}/modules/perception/lidar/lidar_cluster_detect/bin:${PROJECT_ROOT}/modules/perception/lidar/lidar_ground_segmentation/bin:${PROJECT_ROOT}/modules/perception/lidar/multi_lidar_splicing/bin:${PROJECT_ROOT}/modules/perception/fusion/motion_manager/bin:${PROJECT_ROOT}/third_party/arm64/lib/opencv4:${PROJECT_ROOT}/third_party/arm64/lib:/usr/local/legionclaw/third_party/arm64/lib/opencv4:/usr/local/legionclaw/third_party/arm64/lib:${LD_LIBRARY_PATH:-}"

log "LD_LIBRARY_PATH set for OpenCV 4.1"

if [ "${START_ALL}" = true ]; then
    log "Starting all modules..."
    start_multi_lidar_splicing
    start_lidar_ground_segmentation
    start_lidar_cluster_detect
    start_motion_manager
else
    if [ "${START_MULTI_LIDAR}" = true ]; then
        start_multi_lidar_splicing
    fi
    if [ "${START_GROUND_SEG}" = true ]; then
        start_lidar_ground_segmentation
    fi
    if [ "${START_CLUSTER}" = true ]; then
        start_lidar_cluster_detect
    fi
    if [ "${START_MOTION}" = true ]; then
        start_motion_manager
    fi
fi

if [ ${#PIDS[@]} -eq 0 ]; then
    log_error "No modules started. Use --help for usage."
    exit 1
fi

log "========================================="
log "All modules started successfully"
log "Log files: ${LOG_DIR}/"
log "Press Ctrl+C to stop all modules"
log "========================================="

# Wait for all processes
while true; do
    any_running=false
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            any_running=true
            break
        fi
    done

    if [ "$any_running" = false ]; then
        log "All modules have exited."
        exit 0
    fi

    sleep 1
done
