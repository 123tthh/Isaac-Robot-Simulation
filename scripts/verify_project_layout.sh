#!/usr/bin/env bash

set -u

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="${PROJECT_ROOT:-$(cd "$script_dir/.." && pwd)}"
trace_root="$project_root/projects/teleoperation/sim1/trace_data"
failures=0

check_path() {
  local path="$1"
  if [[ -e "$path" || -L "$path" ]]; then
    printf 'PASS %s\n' "$path"
  else
    printf 'FAIL missing %s\n' "$path"
    ((failures += 1))
  fi
}

for path in \
  "$project_root/projects/ros2_ws/src/r1_lerobot_sim" \
  "$project_root/projects/ros2_ws/src/robot" \
  "$project_root/projects/teleoperation/sim1/game_control.sh" \
  "$project_root/data/ros2_log" \
  "$project_root/assets/scenes/scene.usd" \
  "$project_root/docker/Dockerfile" \
  "$project_root/docker/entrypoint.sh" \
  "$project_root/outputs"; do
  check_path "$path"
done

if find "$project_root/assets/datasets/sac-m" -mindepth 1 -print -quit 2>/dev/null | grep -q .; then
  printf 'FAIL legacy sac-m assets remain after workcell migration\n'
  ((failures += 1))
else
  printf 'PASS legacy sac-m asset directory is empty\n'
fi

# Both LeRobot versions are supported local outputs, not layout violations.
check_path "$project_root/run.sh"
check_path "$project_root/docs/overview.md"
check_path "$project_root/docs/overview.zh-CN.md"
check_path "$project_root/projects/physics_parameters"

if [[ -d "$trace_root/raw" ]]; then
  mapfile -t actual_traces < <(
    find "$trace_root/raw" -maxdepth 1 -type f \
      -name 'manual_ocs2_keyboard_trace_20*.csv' -printf '%f\n' | sort -u
  )
  printf 'INFO raw SIM1 traces available: %d\n' "${#actual_traces[@]}"
else
  printf 'INFO raw SIM1 traces were not supplied; collection can create %s/raw\n' "$trace_root"
fi

printf 'Layout verification completed with %d failure(s).\n' "$failures"
exit "$failures"
