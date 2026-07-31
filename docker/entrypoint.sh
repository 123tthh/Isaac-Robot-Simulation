#!/usr/bin/env bash

set -e

export WORKSPACE="${WORKSPACE:-/workspace}"
export ROS2_WS="${ROS2_WS:-$WORKSPACE/projects/ros2_ws}"
export ROS2_INSTALL_DIR="${ROS2_INSTALL_DIR:-$ROS2_WS/install_docker}"
export TRAJECTORY_DIR="${TRAJECTORY_DIR:-$WORKSPACE/projects/Trajectory}"
export SIM1_DIR="${SIM1_DIR:-$TRAJECTORY_DIR/SIM1}"
export ROS2_LOG_DIR="${ROS2_LOG_DIR:-$WORKSPACE/data/ros2_log}"
export ISAAC_USD_PATH="${ISAAC_USD_PATH:-$WORKSPACE/assets/scenes/scene.usd}"
export OUTPUT_DIR="${OUTPUT_DIR:-$WORKSPACE/outputs}"
export LEROBOT_DATASET_PATH="${LEROBOT_DATASET_PATH:-}"
export LEROBOT_NPZ_PATH="${LEROBOT_NPZ_PATH:-}"
export SIM1_GENERATE_LEROBOT_V21="${SIM1_GENERATE_LEROBOT_V21:-0}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DISTRO_TARGET="${ROS_DISTRO_TARGET:-jazzy}"
export NVIDIA_VISIBLE_DEVICES="${NVIDIA_VISIBLE_DEVICES:-all}"
export NVIDIA_DRIVER_CAPABILITIES="${NVIDIA_DRIVER_CAPABILITIES:-compute,utility,graphics,display}"
ros_underlay="/opt/ros/$ROS_DISTRO_TARGET/setup.bash"
if [[ -f "$ros_underlay" ]]; then
  # shellcheck source=/dev/null
  source "$ros_underlay"
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  printf 'WARNING: requested ROS_DISTRO_TARGET=%s is unavailable; using Humble.\n' "$ROS_DISTRO_TARGET" >&2
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi

if [[ -f "$ROS2_INSTALL_DIR/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "$ROS2_INSTALL_DIR/setup.bash"
fi

if [[ -f "$WORKSPACE/docker/shell_aliases.sh" ]]; then
  # Prefer the mounted project version so a restored workspace does not inherit
  # stale absolute paths from an older image.
  # shellcheck source=/dev/null
  source "$WORKSPACE/docker/shell_aliases.sh"
elif [[ -f /etc/profile.d/isaac_ocs_aliases.sh ]]; then
  # shellcheck source=/dev/null
  source /etc/profile.d/isaac_ocs_aliases.sh
fi

if [[ -n "${XDG_RUNTIME_DIR:-}" ]]; then
  mkdir -p "$XDG_RUNTIME_DIR"
  chmod 0700 "$XDG_RUNTIME_DIR"
fi
mkdir -p "$OUTPUT_DIR" "$ROS2_LOG_DIR"
cd "$WORKSPACE"
exec "$@"
