#!/usr/bin/env bash
# Source this file in Bash to enter the project's ROS Jazzy SIM1 Conda runtime.

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  printf 'Source this file: source scripts/activate_sim1_conda.sh\n' >&2
  exit 2
fi

_sim1_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_sim1_project_root="$(cd "$_sim1_script_dir/.." && pwd)"
# Re-sourcing from an active ROS terminal must not feed system dist-packages
# into Conda's own Python (which may require newer typing_extensions).
_sim1_saved_pythonpath="${PYTHONPATH-}"
_sim1_saved_no_user_site="${PYTHONNOUSERSITE-}"
unset PYTHONPATH PYTHONNOUSERSITE
_sim1_conda_base="$(conda info --base)"
# shellcheck source=/dev/null
source "$_sim1_conda_base/etc/profile.d/conda.sh"
conda activate "${SIM1_CONDA_ENV:-ros2-jazzy}"
if [[ -n "$_sim1_saved_pythonpath" ]]; then
  export PYTHONPATH="$_sim1_saved_pythonpath"
fi
if [[ -n "$_sim1_saved_no_user_site" ]]; then
  export PYTHONNOUSERSITE="$_sim1_saved_no_user_site"
fi

if [[ "${ROS_DISTRO:-}" != jazzy ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/jazzy/setup.bash
fi
# Ubuntu ROS debs use dist-packages, which Conda's Python does not search by default.
case ":${PYTHONPATH:-}:" in
  *:/usr/lib/python3/dist-packages:*) ;;
  *) export PYTHONPATH="/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}" ;;
esac

if [[ -z "${ROS_DEPENDENCY_OVERLAY:-}" && -d "$_sim1_project_root/.runtime/ros-jazzy/opt/ros/jazzy" ]]; then
  export ROS_DEPENDENCY_OVERLAY="$_sim1_project_root/.runtime/ros-jazzy"
fi
export PYTHONNOUSERSITE=1
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
if [[ -n "${ROS_DEPENDENCY_OVERLAY:-}" ]]; then
  _sim1_overlay_ros="$ROS_DEPENDENCY_OVERLAY/opt/ros/jazzy"
  _sim1_overlay_system="$ROS_DEPENDENCY_OVERLAY/usr/lib/x86_64-linux-gnu"
  _sim1_overlay_python="$_sim1_overlay_ros/lib/python3.12/site-packages"
  case ":${PYTHONPATH:-}:" in
    *:"$_sim1_overlay_python":*) ;;
    *) export PYTHONPATH="$_sim1_overlay_python${PYTHONPATH:+:$PYTHONPATH}" ;;
  esac
  case ":${PYTHONPATH:-}:" in
    *:"$ROS_DEPENDENCY_OVERLAY/usr/lib/python3/dist-packages":*) ;;
    *) export PYTHONPATH="$ROS_DEPENDENCY_OVERLAY/usr/lib/python3/dist-packages:$PYTHONPATH" ;;
  esac
  if [[ ! -d "$_sim1_overlay_ros/lib" ]]; then
    printf 'Missing ROS dependency overlay: %s\n' "$_sim1_overlay_ros/lib" >&2
    return 2
  fi
  case ":${LD_LIBRARY_PATH:-}:" in
    *:"$_sim1_overlay_ros/lib/x86_64-linux-gnu":*) ;;
    *) export LD_LIBRARY_PATH="$_sim1_overlay_ros/lib:$_sim1_overlay_ros/lib/x86_64-linux-gnu:$_sim1_overlay_system${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" ;;
  esac
  case ":${AMENT_PREFIX_PATH:-}:" in
    *:"$_sim1_overlay_ros":*) ;;
    *) export AMENT_PREFIX_PATH="$_sim1_overlay_ros${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}" ;;
  esac
fi
export ROS2_WS="$_sim1_project_root/projects/ros2_ws"
export SIM1_DIR="$_sim1_project_root/projects/teleoperation/sim1"
export ROS_LOG_DIR="$_sim1_project_root/data/ros2_log"
mkdir -p "$ROS_LOG_DIR"
if [[ -f "$ROS2_WS/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "$ROS2_WS/install/setup.bash"
fi

unset _sim1_script_dir _sim1_project_root _sim1_conda_base _sim1_saved_pythonpath _sim1_saved_no_user_site _sim1_overlay_ros _sim1_overlay_system _sim1_overlay_python
