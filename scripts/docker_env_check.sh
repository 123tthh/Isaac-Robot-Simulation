#!/usr/bin/env bash

set -u

WORKSPACE="${WORKSPACE:-/workspace}"
ROS2_WS="${ROS2_WS:-$WORKSPACE/projects/ros2_ws}"
ROS2_INSTALL_DIR="${ROS2_INSTALL_DIR:-$ROS2_WS/install_docker}"
SIM1_DIR="${SIM1_DIR:-$WORKSPACE/projects/teleoperation/sim1}"
ISAAC_USD_PATH="${ISAAC_USD_PATH:-$WORKSPACE/assets/scenes/scene.usd}"
ROS_DISTRO_TARGET="${ROS_DISTRO_TARGET:-jazzy}"
failures=0

check_command() {
  local label="$1"
  shift
  printf '\n[%s]\n' "$label"
  if timeout 15 "$@"; then
    printf 'PASS: %s\n' "$label"
  else
    printf 'WARN: %s failed or timed out\n' "$label"
    ((failures += 1))
  fi
}

check_file() {
  local path="$1"
  if [[ -e "$path" ]]; then
    printf 'PASS: %s\n' "$path"
  else
    printf 'WARN: missing %s\n' "$path"
    ((failures += 1))
  fi
}

check_command "NVIDIA GPU" nvidia-smi
check_command "Python" python3 --version
check_command "Isaac Sim Python" /isaac-sim/python.sh --version
check_command "ROS 2 CLI" ros2 --help
check_file "/opt/ros/$ROS_DISTRO_TARGET/setup.bash"
check_file "$ROS2_INSTALL_DIR/setup.bash"
check_file "$SIM1_DIR/game_control.sh"
check_file "$ISAAC_USD_PATH"
check_command "ROS 2 topics" ros2 topic list
check_command "ROS 2 services" ros2 service list
check_command "SIM1 status" "$SIM1_DIR/game_control.sh" check

printf '\nEnvironment check completed with %d warning(s).\n' "$failures"
exit 0
