#!/usr/bin/env bash

alias ocs_check='"${SIM1_DIR:-/workspace/projects/teleoperation/sim1}/game_control.sh" check'
alias ocs_launch='ros2 launch r1_description ocs2_isaac.launch.py'
alias ocs_build_all='bash "${WORKSPACE:-/workspace}/scripts/build_ros_workspace_docker.sh"'
alias sim1_check='"${SIM1_DIR:-/workspace/projects/teleoperation/sim1}/game_control.sh" check'
alias sim1_teach='"${SIM1_DIR:-/workspace/projects/teleoperation/sim1}/game_control.sh" teach'
alias sim1_demo='"${SIM1_DIR:-/workspace/projects/teleoperation/sim1}/game_control.sh" demo'
alias sim1_reset='"${SIM1_DIR:-/workspace/projects/teleoperation/sim1}/game_control.sh" reset'
alias sim1_clean='python3 "${SIM1_DIR:-/workspace/projects/teleoperation/sim1}/trace_cleaning/scripts/clean_sim1_episode.py"'
alias sim1_convert='python3 "${SIM1_DIR:-/workspace/projects/teleoperation/sim1}/trace_cleaning/scripts/convert_ocs2_trace_to_lerobot_v30.py"'
alias sim1_convert_v21='python3 "${SIM1_DIR:-/workspace/projects/teleoperation/sim1}/trace_cleaning/scripts/convert_ocs2_trace_to_lerobot_v21.py"'
