#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "$script_dir/.." && pwd)"
host_project_root="${HOST_PROJECT_ROOT:-$project_root}"
image_name="${IMAGE_NAME:-issac_ocs_docker}"
container_name="${CONTAINER_NAME:-demo_vla}"
host_uid="${HOST_UID:-${SUDO_UID:-$(id -u)}}"
host_gid="${HOST_GID:-${SUDO_GID:-$(id -g)}}"
isaac_sim_gid="${ISAAC_SIM_GID:-1234}"
container_home="${CONTAINER_HOME:-/tmp/isaac-ocs-home-$host_uid}"
replace=false

if [[ "${1:-}" == "--replace" ]]; then
  replace=true
elif [[ $# -ne 0 ]]; then
  printf 'Usage: %s [--replace]\n' "$0" >&2
  exit 2
fi

mkdir -p \
  "$host_project_root/projects" \
  "$host_project_root/data" \
  "$host_project_root/assets" \
  "$host_project_root/outputs"

[[ -d "$host_project_root/projects/ros2_ws" ]] \
  || { printf 'ERROR: missing %s\n' "$host_project_root/projects/ros2_ws" >&2; exit 1; }
[[ -d "$host_project_root/projects/Trajectory/SIM1" ]] \
  || { printf 'ERROR: missing %s\n' "$host_project_root/projects/Trajectory/SIM1" >&2; exit 1; }

host_usd="$host_project_root/assets/scenes/scene.usd"
if [[ ! -f "$host_usd" ]]; then
  printf 'WARNING: USD does not exist yet: %s\n' "$host_usd" >&2
fi

docker_cmd=(docker)
if ! docker info >/dev/null 2>&1; then
  docker_cmd=(sudo docker)
fi

if "${docker_cmd[@]}" container inspect "$container_name" >/dev/null 2>&1; then
  if [[ "$replace" != true ]]; then
    printf 'Container %s already exists. Re-run with --replace.\n' "$container_name" >&2
    exit 1
  fi
  "${docker_cmd[@]}" rm -f "$container_name"
fi

x11_args=(
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw
  -e "DISPLAY=${DISPLAY:-:1}"
  -e QT_X11_NO_MITSHM=1
)
if [[ -n "${HOME:-}" && -f "$HOME/.Xauthority" ]]; then
  x11_args+=(
    -v "$HOME/.Xauthority:/tmp/.docker.xauthority:ro"
    -e XAUTHORITY=/tmp/.docker.xauthority
  )
fi

exec "${docker_cmd[@]}" run -itd \
  --gpus all \
  --privileged=true \
  --net=host \
  --ipc=host \
  --pid=host \
  --user "$host_uid:$host_gid" \
  --group-add "$isaac_sim_gid" \
  -m 500G \
  --shm-size=100G \
  --tmpfs /tmp:rw,exec,nosuid,size=4g \
  -v "$host_project_root:/workspace" \
  "${x11_args[@]}" \
  -e WORKSPACE=/workspace \
  -e ROS2_WS=/workspace/projects/ros2_ws \
  -e ROS2_INSTALL_DIR=/workspace/projects/ros2_ws/install_docker \
  -e TRAJECTORY_DIR=/workspace/projects/Trajectory \
  -e SIM1_DIR=/workspace/projects/Trajectory/SIM1 \
  -e ROS2_LOG_DIR=/workspace/data/ros2_log \
  -e ISAAC_USD_PATH=/workspace/assets/scenes/scene.usd \
  -e OUTPUT_DIR=/workspace/outputs \
  -e LEROBOT_DATASET_PATH="${LEROBOT_DATASET_PATH:-}" \
  -e LEROBOT_NPZ_PATH="${LEROBOT_NPZ_PATH:-}" \
  -e SIM1_GENERATE_LEROBOT_V21="${SIM1_GENERATE_LEROBOT_V21:-0}" \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" \
  -e ROS_DISTRO_TARGET="${ROS_DISTRO_TARGET:-jazzy}" \
  -e HOME="$container_home" \
  -e XDG_RUNTIME_DIR="/tmp/runtime-$host_uid" \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display \
  -e ACCEPT_EULA="${ACCEPT_EULA:-Y}" \
  --name "$container_name" \
  "$image_name" \
  /bin/bash
