#!/usr/bin/env bash

set -u

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="${PROJECT_ROOT:-$(cd "$script_dir/.." && pwd)}"
trace_root="$project_root/projects/Trajectory/SIM1/trace_data"
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
  "$project_root/projects/Trajectory/SIM1/game_control.sh" \
  "$project_root/data/ros2_log" \
  "$project_root/assets/scenes/scene.usd" \
  "$project_root/docker/Dockerfile" \
  "$project_root/docker/entrypoint.sh" \
  "$project_root/outputs"; do
  check_path "$path"
done

if find "$project_root/assets/datasets/sac-m" -mindepth 1 -print -quit 2>/dev/null | grep -q .; then
  printf 'FAIL legacy sac-m assets remain after 615scene migration\n'
  ((failures += 1))
else
  printf 'PASS legacy sac-m asset directory is empty\n'
fi

if find \
  "$project_root/projects/Trajectory" \
  "$project_root/data/ros2_log" \
  -type d -iname 'lerobot_v21' -print -quit 2>/dev/null | grep -q .; then
  printf 'FAIL LeRobot v2.1 user data remains\n'
  ((failures += 1))
else
  printf 'PASS LeRobot v2.1 user data absent\n'
fi

expected_traces=(
  manual_ocs2_keyboard_trace_20260715_133454
)

mapfile -t actual_traces < <(
  find "$trace_root/raw" -maxdepth 1 \( -type f -o -type d \) \
    -name 'manual_ocs2_keyboard_trace_20*' -printf '%f\n' 2>/dev/null \
    | sed -E 's/\.(diagnostics\.csv|metadata\.json|label\.md|csv)$//' \
    | sort -u
)

if [[ "${actual_traces[*]}" == "${expected_traces[*]}" ]]; then
  printf 'PASS retained trace family list\n'
else
  printf 'FAIL retained trace family list\n'
  printf '  expected: %s\n' "${expected_traces[*]}"
  printf '  actual:   %s\n' "${actual_traces[*]}"
  ((failures += 1))
fi

printf 'Layout verification completed with %d failure(s).\n' "$failures"
exit "$failures"
