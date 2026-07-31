#!/usr/bin/env bash
#
# Unified host launcher for the normalized Isaac OCS project.
# Local documentation references:
#   /home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.md
#   /home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/overview/release_notes.md
#   /home/gtk/ai_docs/docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.md

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS2_WS="${ROS2_WS:-$PROJECT_ROOT/projects/ros2_ws}"
SIM1_DIR="${SIM1_DIR:-$PROJECT_ROOT/projects/Trajectory/SIM1}"
ROS2_LOG_DIR="${ROS2_LOG_DIR:-$PROJECT_ROOT/data/ros2_log}"
ISAAC_SIM_ROOT="${ISAAC_SIM_ROOT:-/home/gtk/isaac-sim-5.1}"
ISAAC_SIM_LAUNCHER="${ISAAC_SIM_LAUNCHER:-$ISAAC_SIM_ROOT/isaac-sim.sh}"
ISAAC_USD_PATH="${ISAAC_USD_PATH:-$PROJECT_ROOT/assets/scenes/scene.usd}"
CAMERA_REBUILD_SCRIPT="${CAMERA_REBUILD_SCRIPT:-$SCRIPT_DIR/rebuild_six_camera_render_products_20260730.py}"
ISAAC_PYTHON_VENDOR="${ISAAC_PYTHON_VENDOR:-$PROJECT_ROOT/vendor/isaac_sim_5_1_python}"
ROS_DISTRO_TARGET="${ROS_DISTRO_TARGET:-humble}"
ROS_UNDERLAY="${ROS_UNDERLAY:-/opt/ros/$ROS_DISTRO_TARGET/setup.bash}"

usage() {
  cat <<EOF
Usage:
  $0 preflight
  $0 build [extra colcon arguments...]
  $0 build --all [extra colcon arguments...]
  $0 start-isaac [Isaac Sim arguments...]
  $0 inspect-usd [USD path]
  $0 migrate-scene-paths [--apply]
  $0 launch-ocs2 [ROS launch arguments...]
  $0 sim1-check
  $0 sim1-reset
  $0 sim1-teach [SIM1 teach arguments...]
  $0 sim1-demo [SIM1 replay arguments...]

Defaults:
  PROJECT_ROOT=$PROJECT_ROOT
  ROS2_WS=$ROS2_WS
  SIM1_DIR=$SIM1_DIR
  ROS2_LOG_DIR=$ROS2_LOG_DIR
  ISAAC_SIM_ROOT=$ISAAC_SIM_ROOT
  ISAAC_USD_PATH=$ISAAC_USD_PATH
  CAMERA_REBUILD_SCRIPT=$CAMERA_REBUILD_SCRIPT
  ISAAC_PYTHON_VENDOR=$ISAAC_PYTHON_VENDOR
  ROS_DISTRO_TARGET=$ROS_DISTRO_TARGET
EOF
}

require_file() {
  local path="$1"
  local label="$2"
  if [[ ! -f "$path" ]]; then
    printf 'ERROR: missing %s: %s\n' "$label" "$path" >&2
    return 1
  fi
}

source_ros() {
  require_file "$ROS_UNDERLAY" "ROS 2 underlay"
  # shellcheck source=/dev/null
  set +u
  source "$ROS_UNDERLAY"
  set -u

  require_file "$ROS2_WS/install/setup.bash" "normalized ROS 2 overlay; run '$0 build' first"
  # shellcheck source=/dev/null
  set +u
  source "$ROS2_WS/install/setup.bash"
  set -u
}

preflight() {
  local failures=0
  local path=""

  printf '[project]\nroot=%s\nros2_ws=%s\nsim1=%s\n\n' \
    "$PROJECT_ROOT" "$ROS2_WS" "$SIM1_DIR"

  for path in \
    "$ROS_UNDERLAY" \
    "$ROS2_WS/src/robot/package.xml" \
    "$SIM1_DIR/game_control.sh" \
    "$ISAAC_SIM_LAUNCHER" \
    "$ISAAC_USD_PATH" \
    "$CAMERA_REBUILD_SCRIPT" \
    "$ISAAC_PYTHON_VENDOR/psutil/__init__.py" \
    "$ISAAC_PYTHON_VENDOR/click/__init__.py" \
    "$ISAAC_PYTHON_VENDOR/typing_extensions.py"; do
    if [[ -e "$path" ]]; then
      printf 'PASS %s\n' "$path"
    else
      printf 'FAIL missing %s\n' "$path"
      ((failures += 1))
    fi
  done

  if command -v nvidia-smi >/dev/null 2>&1; then
    nvidia-smi --query-gpu=name,driver_version,memory.total \
      --format=csv,noheader || ((failures += 1))
  else
    printf 'FAIL nvidia-smi is unavailable\n'
    ((failures += 1))
  fi

  if [[ -f "$ROS_UNDERLAY" ]]; then
    # shellcheck source=/dev/null
    set +u
    source "$ROS_UNDERLAY"
    set -u
  fi
  if [[ -f "$ROS2_WS/install/setup.bash" ]]; then
    # shellcheck source=/dev/null
    set +u
    source "$ROS2_WS/install/setup.bash"
    set -u
    for path in r1_description ocs2_arm_controller simulation_interfaces; do
      if ros2 pkg prefix "$path" >/dev/null 2>&1; then
        printf 'PASS ROS package %s\n' "$path"
      else
        printf 'FAIL ROS package %s\n' "$path"
        ((failures += 1))
      fi
    done
  else
    printf 'FAIL normalized ROS overlay is not built: %s/install/setup.bash\n' "$ROS2_WS"
    ((failures += 1))
  fi

  if bash -n "$SIM1_DIR/game_control.sh"; then
    printf 'PASS SIM1 shell syntax\n'
  else
    printf 'FAIL SIM1 shell syntax\n'
    ((failures += 1))
  fi

  printf '\nPreflight completed with %d failure(s).\n' "$failures"
  return "$failures"
}

mode="${1:-help}"
shift || true

case "$mode" in
  help|-h|--help)
    usage
    ;;
  preflight)
    preflight
    ;;
  build)
    require_file "$ROS_UNDERLAY" "ROS 2 underlay"
    # shellcheck source=/dev/null
    set +u
    source "$ROS_UNDERLAY"
    set -u
    cd "$ROS2_WS"
    if [[ "${1:-}" == "--all" ]]; then
      shift
      exec colcon build --symlink-install "$@"
    fi
    exec colcon build --symlink-install \
      --packages-up-to \
      r1_description \
      ocs2_arm_controller \
      arms_target_manager \
      topic_based_ros2_control \
      r1_lerobot_sim \
      "$@"
    ;;
  start-isaac)
    require_file "$ISAAC_SIM_LAUNCHER" "Isaac Sim launcher"
    # Named Replicator RenderProducts are session-layer objects in Isaac Sim
    # 5.1.  With no explicit arguments, open the validated scene through the
    # rebuild script and bind every CameraHelper to the path actually returned
    # by rep.create.render_product(). The root USD is not given session paths.
    if (($# == 0)); then
      require_file "$CAMERA_REBUILD_SCRIPT" "six-camera RenderProduct rebuild script"
      set -- --exec "$CAMERA_REBUILD_SCRIPT"
    fi
    # Isaac Sim embeds Python 3.11. Do not inherit the host ROS Humble
    # Python 3.10 environment or a different Isaac installation.
    export ISAAC_PATH="$ISAAC_SIM_ROOT"
    export EXP_PATH="$ISAAC_SIM_ROOT/apps"
    unset \
      AMENT_PREFIX_PATH \
      CMAKE_PREFIX_PATH \
      COLCON_PREFIX_PATH \
      LD_LIBRARY_PATH \
      PYTHONPATH \
      ROS_DISTRO \
      ROS_PACKAGE_PATH \
      ROS_PYTHON_VERSION \
      ROS_VERSION \
      RMW_IMPLEMENTATION
    export PYTHONPATH="$ISAAC_PYTHON_VENDOR"
    export ROS2_LOG_DIR
    exec "$ISAAC_SIM_LAUNCHER" \
      --/app/python/extraPaths/0="$ISAAC_PYTHON_VENDOR" \
      --isaac/startup/ros_sim_control_extension=True \
      "$@"
    ;;
  inspect-usd)
    usd_path="${1:-$ISAAC_USD_PATH}"
    require_file "$usd_path" "USD scene"
    require_file "$ISAAC_SIM_ROOT/python.sh" "Isaac Sim Python launcher"
    export ISAAC_PATH="$ISAAC_SIM_ROOT"
    export EXP_PATH="$ISAAC_SIM_ROOT/apps"
    unset \
      AMENT_PREFIX_PATH \
      CMAKE_PREFIX_PATH \
      COLCON_PREFIX_PATH \
      LD_LIBRARY_PATH \
      PYTHONPATH \
      ROS_DISTRO \
      ROS_PACKAGE_PATH \
      ROS_PYTHON_VERSION \
      ROS_VERSION \
      RMW_IMPLEMENTATION
    export PYTHONPATH="$ISAAC_PYTHON_VENDOR"
    shift || true
    exec "$ISAAC_SIM_ROOT/python.sh" "$SCRIPT_DIR/inspect_usd_scene_20260723.py" "$usd_path" "$@"
    ;;
  migrate-scene-paths)
    require_file "$ISAAC_USD_PATH" "USD scene"
    require_file "$ISAAC_SIM_ROOT/python.sh" "Isaac Sim Python launcher"
    export ISAAC_PATH="$ISAAC_SIM_ROOT"
    export EXP_PATH="$ISAAC_SIM_ROOT/apps"
    unset \
      AMENT_PREFIX_PATH \
      CMAKE_PREFIX_PATH \
      COLCON_PREFIX_PATH \
      LD_LIBRARY_PATH \
      PYTHONPATH \
      ROS_DISTRO \
      ROS_PACKAGE_PATH \
      ROS_PYTHON_VERSION \
      ROS_VERSION \
      RMW_IMPLEMENTATION
    export PYTHONPATH="$ISAAC_PYTHON_VENDOR"
    exec "$ISAAC_SIM_ROOT/python.sh" \
      "$SCRIPT_DIR/migrate_scene_runtime_paths_20260724.py" \
      --scene "$ISAAC_USD_PATH" "$@"
    ;;
  launch-ocs2)
    source_ros
    exec ros2 launch r1_description ocs2_isaac.launch.py "$@"
    ;;
  sim1-check)
    source_ros
    exec "$SIM1_DIR/game_control.sh" check "$@"
    ;;
  sim1-reset)
    source_ros
    exec "$SIM1_DIR/game_control.sh" reset "$@"
    ;;
  sim1-teach)
    source_ros
    exec "$SIM1_DIR/game_control.sh" teach "$@"
    ;;
  sim1-demo)
    source_ros
    exec "$SIM1_DIR/game_control.sh" demo "$@"
    ;;
  *)
    printf 'ERROR: unknown mode: %s\n\n' "$mode" >&2
    usage >&2
    exit 2
    ;;
esac
