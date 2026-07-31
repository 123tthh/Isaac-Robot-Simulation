#!/usr/bin/env bash
#
# Build the complete Docker-supported ROS 2 workspace.
#
# Local documentation references:
#   /home/gtk/ai_docs/docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.md
#   /home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_container.md
#   /home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.md
#
# The three skipped packages form the licensed Raisim advanced example.  The
# repository's own README requires a separately obtained Raisim SDK/license:
#   projects/ros2_ws/src/ocs2_ros2/advance examples/ocs2_raisim/README.md

set -euo pipefail

workspace="${WORKSPACE:-/workspace}"
ros2_ws="${ROS2_WS:-$workspace/projects/ros2_ws}"
ros_distro_target="${ROS_DISTRO_TARGET:-jazzy}"
ros_underlay="/opt/ros/$ros_distro_target/setup.bash"
build_base="${ROS2_BUILD_BASE:-build_docker}"
install_base="${ROS2_INSTALL_BASE:-install_docker}"
log_base="${ROS2_LOG_BASE:-log_docker}"

if [[ ! -f "$ros_underlay" ]]; then
  printf 'ERROR: ROS 2 underlay is missing: %s\n' "$ros_underlay" >&2
  exit 1
fi

if [[ ! -d "$ros2_ws/src" ]]; then
  printf 'ERROR: ROS 2 workspace source directory is missing: %s/src\n' "$ros2_ws" >&2
  exit 1
fi

# ROS setup files probe optional variables that are legitimately unset.
# shellcheck source=/dev/null
set +u
source "$ros_underlay"
set -u
cd "$ros2_ws"

exec colcon \
  --log-base "$log_base" \
  build \
  --symlink-install \
  --build-base "$build_base" \
  --install-base "$install_base" \
  --cmake-clean-cache \
  --packages-skip \
  ocs2_raisim_core \
  ocs2_legged_robot_raisim \
  ocs2_legged_robot_mpcnet \
  "$@" \
  --cmake-args \
  -DBUILD_TESTING=OFF
