#!/usr/bin/env bash
#
# Unified host launcher for the normalized Isaac OCS project.
# Current operator guide: docs/archive/startup-guide-alias.zh-CN.md

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS2_WS="${ROS2_WS:-$PROJECT_ROOT/projects/ros2_ws}"
SIM1_DIR="${SIM1_DIR:-$PROJECT_ROOT/projects/teleoperation/sim1}"
ROS2_LOG_DIR="${ROS2_LOG_DIR:-$PROJECT_ROOT/data/ros2_log}"
ISAAC_SIM_ROOT="${ISAAC_SIM_ROOT:-}"
ISAAC_SIM_LAUNCHER="${ISAAC_SIM_LAUNCHER:-$ISAAC_SIM_ROOT/isaac-sim.sh}"
if [[ -f "$PROJECT_ROOT/assets/scenes/scene_portable.usda" ]]; then
  DEFAULT_ISAAC_USD_PATH="$PROJECT_ROOT/assets/scenes/scene_portable.usda"
else
  DEFAULT_ISAAC_USD_PATH="$PROJECT_ROOT/assets/scenes/scene.usd"
fi
ISAAC_USD_PATH="${ISAAC_USD_PATH:-$DEFAULT_ISAAC_USD_PATH}"
export ROS2_WS SIM1_DIR ROS2_LOG_DIR ISAAC_USD_PATH ISAAC_SIM_ROOT
export ISAAC_OCS_PROJECT_ROOT="$PROJECT_ROOT"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$PROJECT_ROOT/data/ros2_log}"
mkdir -p "$ROS_LOG_DIR"
CAMERA_REBUILD_SCRIPT="${CAMERA_REBUILD_SCRIPT:-$SCRIPT_DIR/rebuild_camera_render_products.py}"
ISAAC_PYTHON_VENDOR="${ISAAC_PYTHON_VENDOR:-$PROJECT_ROOT/vendor/isaac_sim_5_1_python}"
if [[ -z "${ROS_DISTRO_TARGET:-}" ]]; then
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    ROS_DISTRO_TARGET=jazzy
  elif [[ -f /opt/ros/humble/setup.bash ]]; then
    ROS_DISTRO_TARGET=humble
  else
    ROS_DISTRO_TARGET=jazzy
  fi
fi
ROS_UNDERLAY="${ROS_UNDERLAY:-/opt/ros/$ROS_DISTRO_TARGET/setup.bash}"

usage() {
  local command_name="${PROJECT_COMMAND_NAME:-$0}"
  cat <<EOF
Usage:
  $command_name preflight
  $command_name build [extra colcon arguments...]
  $command_name build --all [extra colcon arguments...]
  $command_name start-isaac [--headless] [--duration SECONDS]
  $command_name sim1-session [--name NAME] [--no-camera-grid]
  $command_name inspect-usd [USD path]
  $command_name migrate-scene-paths [--apply]
  $command_name launch-ocs2 [ROS launch arguments...]
  $command_name sim1-check
  $command_name sim1-reset
  $command_name sim1-teach [SIM1 teach arguments...]
  $command_name sim1-demo [SIM1 replay arguments...]
  $command_name sim1-camera-grid [viewer arguments...]
  $command_name sim1-base-record [record arguments...]
  $command_name sim1-base-path-demo [drive arguments...]
  $command_name sim1-direct-demo [replay arguments...]
  $command_name sim1-process TRACE.csv [--with-v21]

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
    export PYTHONNOUSERSITE=1
    # A host may stage missing ROS development debs outside the repository.
    # The overlay is opt-in; a normal system installation needs no override.
    if [[ -n "${ROS_DEPENDENCY_OVERLAY:-}" ]]; then
      if [[ ! -d "$ROS_DEPENDENCY_OVERLAY/opt/ros/$ROS_DISTRO_TARGET" ]]; then
        printf 'ERROR: ROS_DEPENDENCY_OVERLAY has no ROS %s tree: %s\n' \
          "$ROS_DISTRO_TARGET" "$ROS_DEPENDENCY_OVERLAY" >&2
        exit 2
      fi
      overlay_ros="$ROS_DEPENDENCY_OVERLAY/opt/ros/$ROS_DISTRO_TARGET"
      overlay_system="$ROS_DEPENDENCY_OVERLAY/usr/lib/x86_64-linux-gnu"
      export CMAKE_PREFIX_PATH="$overlay_ros${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
      export AMENT_PREFIX_PATH="$overlay_ros${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
      export CPATH="$overlay_ros/include${CPATH:+:$CPATH}"
      export LIBRARY_PATH="$overlay_ros/lib:$overlay_ros/lib/x86_64-linux-gnu:$overlay_system${LIBRARY_PATH:+:$LIBRARY_PATH}"
      export LD_LIBRARY_PATH="$overlay_ros/lib:$overlay_ros/lib/x86_64-linux-gnu:$overlay_system${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
      export PKG_CONFIG_PATH="$overlay_ros/lib/pkgconfig:$overlay_ros/lib/x86_64-linux-gnu/pkgconfig:$overlay_system/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
    fi
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
      "$@" \
      --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
    ;;
  sim1-session)
    exec python "$SCRIPT_DIR/sim1_session.py" "$@"
    ;;
  start-isaac)
    if [[ -z "$ISAAC_SIM_ROOT" ]]; then
      printf 'ERROR: set ISAAC_SIM_ROOT to the Isaac Sim 5.1 installation directory\n' >&2
      exit 1
    fi
    require_file "$ISAAC_SIM_ROOT/python.sh" "Isaac Sim Python launcher"
    require_file "$SCRIPT_DIR/isaac_sim1_runtime.py" "SIM1 runtime"
    # Isaac embeds Python 3.11; external Jazzy/Conda uses Python 3.12.
    export ISAAC_PATH="$ISAAC_SIM_ROOT"
    export EXP_PATH="$ISAAC_SIM_ROOT/apps"
    unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH \
      PYTHONPATH ROS_PACKAGE_PATH ROS_PYTHON_VERSION ROS_VERSION \
      CONDA_PREFIX CONDA_DEFAULT_ENV
    export ROS_DISTRO=jazzy
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export LD_LIBRARY_PATH="$ISAAC_SIM_ROOT/exts/isaacsim.ros2.bridge/jazzy/lib"
    export PYTHONNOUSERSITE=1
    export PYTHONUNBUFFERED=1
    export ROS2_LOG_DIR
    exec "$ISAAC_SIM_ROOT/python.sh" "$SCRIPT_DIR/isaac_sim1_runtime.py" "$@"
    ;;
  inspect-usd)
    if [[ -z "$ISAAC_SIM_ROOT" ]]; then
      printf 'ERROR: set ISAAC_SIM_ROOT to the Isaac Sim 5.1 installation directory\n' >&2
      exit 1
    fi
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
    exec "$ISAAC_SIM_ROOT/python.sh" "$SCRIPT_DIR/inspect_usd_scene.py" "$usd_path" "$@"
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
      "$SCRIPT_DIR/migrate_scene_runtime_paths.py" \
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
  sim1-camera-grid)
    source_ros
    exec "$SIM1_DIR/game_control.sh" camera-grid "$@"
    ;;
  sim1-base-record)
    source_ros
    exec "$SIM1_DIR/game_control.sh" base-record "$@"
    ;;
  sim1-base-path-demo)
    source_ros
    exec "$SIM1_DIR/game_control.sh" base-path-demo "$@"
    ;;
  sim1-direct-demo)
    source_ros
    exec "$SIM1_DIR/game_control.sh" direct-demo "$@"
    ;;
  sim1-process)
    if [[ $# -eq 0 || ! -f "$1" ]]; then
      printf 'ERROR: pass an existing SIM1 trace CSV to sim1-process\n' >&2
      exit 2
    fi
    exec "$SIM1_DIR/trace_cleaning/scripts/process_sim1_episode.py" "$@"
    ;;
  *)
    printf 'ERROR: unknown mode: %s\n\n' "$mode" >&2
    usage >&2
    exit 2
    ;;
esac
