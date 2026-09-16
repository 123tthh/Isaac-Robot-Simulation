#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_ROS2_WS="$ROOT_DIR/../../ros2_ws"
ROS_SETUP="${ROS_SETUP:-${ROS2_WS:-$DEFAULT_ROS2_WS}/install/setup.bash}"
TRACE_DIR="${TRACE_DIR:-$ROOT_DIR/trace_data/raw}"
TRACE_LATEST="${TRACE_LATEST:-$TRACE_DIR/manual_ocs2_keyboard_trace_latest.csv}"
TRACE_DIAGNOSTICS_LATEST="${TRACE_DIAGNOSTICS_LATEST:-$TRACE_DIR/manual_ocs2_keyboard_trace_latest.diagnostics.csv}"
TRACE="${TRACE:-}"
DIRECT_TRACE_DIR="${DIRECT_TRACE_DIR:-$ROOT_DIR/trace_data/direct}"
BASE_TRACE_DIR="${BASE_TRACE_DIR:-$ROOT_DIR/trace_data/base_raw}"
BASE_SPEED="${BASE_SPEED:-0.5}"
BASE_TURN_WAIT="${BASE_TURN_WAIT:-6.0}"
BASE_RECORD_RATE="${BASE_RECORD_RATE:-50}"
LE_ROBOT_EPISODE="${LE_ROBOT_EPISODE:-${LEROBOT_NPZ_PATH:-}}"
SIM1_PROCESS_AFTER_TEACH="${SIM1_PROCESS_AFTER_TEACH:-1}"
SIM1_GENERATE_LEROBOT_V21="${SIM1_GENERATE_LEROBOT_V21:-0}"

if [[ -f "$ROS_SETUP" ]]; then
  # shellcheck source=/dev/null
  source "$ROS_SETUP"
fi

set -u

usage() {
  cat <<EOF
Usage:
  $0 check
  $0 reset
  $0 teach [extra keyboard_ocs2_gripper_teleop.py args...]
  $0 teach --open-camera-grid [extra keyboard_ocs2_gripper_teleop.py args...]
  $0 teach-with-cameras [extra keyboard_ocs2_gripper_teleop.py args...]
  $0 teach --input-mode pygame [--linear-speed 0.025 --angular-speed-deg 4.0 --control-rate 50]
  $0 demo [extra replay_ocs2_target_trace.py args...]
  $0 camera-grid [extra tools/camera_grid_viewer.py args...]
  $0 base-record [--base-record-rate 50]
  $0 base-demo [--base-speed 0.5 --base-turn-wait 6.0]
  $0 base-path-demo [--base-speed 0.5 --base-turn-wait 6.0]
  $0 direct-demo [extra replay_lerobot_episode.py args...]

Modes:
  reset       Reset Isaac Sim if available, then publish HOME/FSM=1.
  teach       OCS2 target teleop. Default terminal mode reads one key at a time;
              --input-mode pygame opens a small focused window for held-key multi-axis velocity control.
              --open-camera-grid also opens the 2x3 live camera grid and closes it after teach exits.
  teach-with-cameras
              Alias for teach --open-camera-grid.
  demo        OCS2 target replay. Replays the CSV recorded by teach.
  camera-grid Open one window with a 2x3 live camera grid: top RGB, bottom depth.
  base-record Record /sim1/base_diff_cmd, /isaac_joint_states, /joint_states and /clock to trace_data/base_raw.
  base-demo   Alias for base-path-demo.
  base-path-demo Publish a timed forward/turn/forward/turn/forward base path through /sim1/base_diff_cmd.
  direct-demo Direct Isaac joint replay from the converted LeRobot npz. Do not run with OCS2 publishing /arm_joint_cmd.

Environment:
  ROS_SETUP=$ROS_SETUP
  TRACE=${TRACE:-auto timestamped under $TRACE_DIR}
  TRACE_DIR=$TRACE_DIR
  TRACE_LATEST=$TRACE_LATEST
  TRACE_DIAGNOSTICS_LATEST=$TRACE_DIAGNOSTICS_LATEST
  BASE_TRACE_DIR=$BASE_TRACE_DIR
  BASE_SPEED=$BASE_SPEED
  BASE_TURN_WAIT=$BASE_TURN_WAIT
  BASE_RECORD_RATE=$BASE_RECORD_RATE
  LE_ROBOT_EPISODE=$LE_ROBOT_EPISODE
  SIM1_PROCESS_AFTER_TEACH=$SIM1_PROCESS_AFTER_TEACH
  SIM1_GENERATE_LEROBOT_V21=$SIM1_GENERATE_LEROBOT_V21
EOF
}

parse_base_args() {
  BASE_EXTRA_ARGS=()
  while [[ "$#" -gt 0 ]]; do
    case "$1" in
      --base-speed)
        BASE_SPEED="$2"
        shift 2
        ;;
      --base-turn-wait)
        BASE_TURN_WAIT="$2"
        shift 2
        ;;
      --base-record-rate)
        BASE_RECORD_RATE="$2"
        shift 2
        ;;
      *)
        BASE_EXTRA_ARGS+=("$1")
        shift
        ;;
    esac
  done
}

parse_teach_args() {
  TEACH_EXTRA_ARGS=()
  CAMERA_GRID_EXTRA_ARGS=()
  OPEN_CAMERA_GRID="${SIM1_OPEN_CAMERA_GRID:-0}"
  RECORD_CAMERAS_ARG=0

  while [[ "$#" -gt 0 ]]; do
    case "$1" in
      --open-camera-grid)
        OPEN_CAMERA_GRID=1
        shift
        ;;
      --camera-grid-cell-width)
        if [[ "$#" -lt 2 ]]; then
          echo "[teach] missing value for $1" >&2
          exit 2
        fi
        CAMERA_GRID_EXTRA_ARGS+=("--cell-width" "$2")
        shift 2
        ;;
      --camera-grid-cell-height)
        if [[ "$#" -lt 2 ]]; then
          echo "[teach] missing value for $1" >&2
          exit 2
        fi
        CAMERA_GRID_EXTRA_ARGS+=("--cell-height" "$2")
        shift 2
        ;;
      --camera-grid-display-fps)
        if [[ "$#" -lt 2 ]]; then
          echo "[teach] missing value for $1" >&2
          exit 2
        fi
        CAMERA_GRID_EXTRA_ARGS+=("--display-fps" "$2")
        shift 2
        ;;
      --camera-grid-depth-percentile)
        if [[ "$#" -lt 2 ]]; then
          echo "[teach] missing value for $1" >&2
          exit 2
        fi
        CAMERA_GRID_EXTRA_ARGS+=("--depth-percentile" "$2")
        shift 2
        ;;
      --camera-grid-reliability)
        if [[ "$#" -lt 2 ]]; then
          echo "[teach] missing value for $1" >&2
          exit 2
        fi
        CAMERA_GRID_EXTRA_ARGS+=("--reliability" "$2")
        shift 2
        ;;
      --camera-grid-window-title)
        if [[ "$#" -lt 2 ]]; then
          echo "[teach] missing value for $1" >&2
          exit 2
        fi
        CAMERA_GRID_EXTRA_ARGS+=("--window-title" "$2")
        shift 2
        ;;
      --record-cameras)
        RECORD_CAMERAS_ARG=1
        TEACH_EXTRA_ARGS+=("$1")
        shift
        ;;
      *)
        TEACH_EXTRA_ARGS+=("$1")
        shift
        ;;
    esac
  done
}

start_camera_grid_for_teach() {
  CAMERA_GRID_PID=""
  if [[ "${OPEN_CAMERA_GRID:-0}" != "1" ]]; then
    return
  fi

  echo "[teach] opening 6-camera grid viewer"
  python3 "$ROOT_DIR/tools/camera_grid_viewer.py" "${CAMERA_GRID_EXTRA_ARGS[@]}" &
  CAMERA_GRID_PID="$!"
}

stop_camera_grid_for_teach() {
  if [[ -n "${CAMERA_GRID_PID:-}" ]]; then
    if kill -0 "$CAMERA_GRID_PID" 2>/dev/null; then
      echo "[teach] closing 6-camera grid viewer"
      kill "$CAMERA_GRID_PID" 2>/dev/null || true
    fi
    wait "$CAMERA_GRID_PID" 2>/dev/null || true
  fi
  CAMERA_GRID_PID=""
}

make_trace_path() {
  if [[ -n "$TRACE" ]]; then
    printf '%s\n' "$TRACE"
    return
  fi
  mkdir -p "$TRACE_DIR"
  printf '%s/manual_ocs2_keyboard_trace_%s.csv\n' "$TRACE_DIR" "$(date +%Y%m%d_%H%M%S)"
}

resolve_demo_trace() {
  if [[ -n "$TRACE" ]]; then
    printf '%s\n' "$TRACE"
  elif [[ -e "$TRACE_LATEST" ]]; then
    printf '%s\n' "$TRACE_LATEST"
  else
    printf '%s/manual_ocs2_keyboard_trace.csv\n' "$TRACE_DIR"
  fi
}

check() {
  echo "[check] ROS environment"
  echo "ROS_DISTRO=${ROS_DISTRO:-unset}"
  ros2 pkg prefix simulation_interfaces || true

  echo
  echo "[check] Isaac simulation control services"
  ros2 service type /reset_simulation || true
  ros2 service type /set_simulation_state || true

  echo "[check] ROS nodes"
  ros2 node list || true

  echo
  echo "[check] Key topics"
  ros2 topic info -v /left_current_pose || true
  ros2 topic info -v /right_current_pose || true
  ros2 topic info -v /left_target/stamped || true
  ros2 topic info -v /right_target/stamped || true
  ros2 topic info -v /arm_joint_cmd || true
  ros2 topic info -v /sim1/base_diff_cmd || true
  ros2 topic info -v /isaac_joint_states || true
  ros2 topic info -v /left_gripper_controller/commands || true
  ros2 topic info -v /right_gripper_controller/commands || true

  echo
  echo "[check] Controllers"
  timeout 5 ros2 control list_controllers || true
}

reset_recover() {
  local reset_type=""
  local set_state_type=""

  echo "[reset] Checking simulation_interfaces package"
  if ! ros2 pkg prefix simulation_interfaces >/dev/null 2>&1; then
    cat <<EOF
[reset] simulation_interfaces is not visible in this terminal.
Install/source it, then retry:
  sudo apt-get install -y ros-jazzy-simulation-interfaces
  source "$ROS_SETUP"
EOF
  else
    ros2 pkg prefix simulation_interfaces || true
  fi

  echo "[reset] Checking Isaac Sim control services"
  reset_type="$(ros2 service type /reset_simulation 2>/dev/null || true)"
  set_state_type="$(ros2 service type /set_simulation_state 2>/dev/null || true)"

  if [[ "$set_state_type" == "simulation_interfaces/srv/SetSimulationState" ]]; then
    echo "[reset] Ensure simulation is playing"
    timeout 5 ros2 service call /set_simulation_state simulation_interfaces/srv/SetSimulationState "{state: {state: 1}}" || true
  else
    echo "[reset] /set_simulation_state unavailable; enable isaacsim.ros2.sim_control in Isaac Sim."
  fi

  if [[ "$reset_type" == "simulation_interfaces/srv/ResetSimulation" ]]; then
    echo "[reset] Calling /reset_simulation"
    timeout 8 ros2 service call /reset_simulation simulation_interfaces/srv/ResetSimulation "{scope: 255}" || true
  else
    cat <<EOF
[reset] /reset_simulation unavailable.
Start Isaac Sim with:
  ./scripts/project.sh start-isaac
or enable Extension Manager item:
  isaacsim.ros2.sim_control
EOF
  fi

  echo "[reset] Publish HOME/FSM=1 repeatedly"
  for _ in 1 2 3 4 5; do
    ros2 topic pub --once /fsm_command std_msgs/msg/Int32 "{data: 1}" || true
    sleep 0.2
  done

  if [[ "${RESET_ACTIVATE_OCS2:-0}" == "1" ]]; then
    echo "[reset] Publish OCS2/FSM=3 because RESET_ACTIVATE_OCS2=1"
    ros2 topic pub --once /fsm_command std_msgs/msg/Int32 "{data: 3}" || true
  fi

  echo "[reset] If target/current pose is still stale, start teach and press Z to force target reinitialization from current pose."
}

write_teach_label() {
  local trace_path="$1"
  local label_path="${trace_path%.csv}.label.md"
  local metadata_path="${trace_path%.csv}.metadata.json"
  local camera_dir="${trace_path%.csv}/camera"
  local choice=""
  local outcome=""
  local note=""

  if [[ ! -t 0 ]]; then
    cat > "$label_path" <<EOF
# Teach Trace Label

- trace: \`$trace_path\`
- metadata: \`$metadata_path\`
- camera: \`$camera_dir\`
- outcome: \`unlabeled_noninteractive\`
- note: \`stdin was not interactive after teach\`
EOF
    echo "[teach] label -> $label_path"
    return
  fi

  echo
  echo "[teach] Recording finished. Mark this trace:"
  echo "  0) success / no failure observed"
  echo "  1) failure: joint torque disappeared"
  echo "  2) failure: singularity or near-singularity suspected"
  echo "  3) failure: other"
  read -r -p "Select [0-3]: " choice
  case "$choice" in
    0) outcome="success" ;;
    1) outcome="joint_torque_disappeared" ;;
    2) outcome="singularity_suspected" ;;
    3) outcome="other_failure" ;;
    *) outcome="unlabeled_invalid_choice" ;;
  esac
  read -r -p "Optional note: " note

  cat > "$label_path" <<EOF
# Teach Trace Label

- trace: \`$trace_path\`
- metadata: \`$metadata_path\`
- camera: \`$camera_dir\`
- outcome: \`$outcome\`
- note: \`$note\`
- reset_note: \`Q/Esc triggers script reset_after_task. If Isaac/OCS2 did not reset, run manual reset commands in GAME_GUIDE.md.\`
EOF
  echo "[teach] label -> $label_path"
}

run_teach() {
  local trace_path=""
  local status=0
  local diagnostics_path=""
  local process_status=0
  local -a process_args=()

  parse_teach_args "$@"
  trace_path="$(make_trace_path)"
  if [[ "${RECORD_CAMERAS_ARG:-0}" == "1" && "${OPEN_CAMERA_GRID:-0}" != "1" ]]; then
    echo "[teach] --record-cameras records RGB+D only; add --open-camera-grid to view the 2x3 live camera window."
  fi
  start_camera_grid_for_teach

  set +e
  "$ROOT_DIR/keyboard_ocs2_gripper_teleop.py" \
    --reset-targets \
    --log-csv "$trace_path" \
    "${TEACH_EXTRA_ARGS[@]}"
  status=$?
  set -e

  stop_camera_grid_for_teach

  if [[ "$status" -eq 0 && -f "$trace_path" ]]; then
    mkdir -p "$(dirname "$TRACE_LATEST")"
    ln -sfn "$(realpath --relative-to="$(dirname "$TRACE_LATEST")" "$trace_path")" "$TRACE_LATEST"
    echo "[teach] latest trace -> $TRACE_LATEST"
    diagnostics_path="${trace_path%.csv}.diagnostics.csv"
    if [[ -f "$diagnostics_path" ]]; then
      ln -sfn "$(realpath --relative-to="$(dirname "$TRACE_DIAGNOSTICS_LATEST")" "$diagnostics_path")" "$TRACE_DIAGNOSTICS_LATEST"
      echo "[teach] latest diagnostics -> $TRACE_DIAGNOSTICS_LATEST"
    fi
    write_teach_label "$trace_path"
    if [[ "$SIM1_PROCESS_AFTER_TEACH" == "1" ]]; then
      process_args=("$trace_path")
      if [[ "$SIM1_GENERATE_LEROBOT_V21" == "1" ]]; then
        process_args+=("--with-v21")
        echo "[teach] process raw -> cleaned -> lerobot_v30 + lerobot_v21"
      else
        echo "[teach] process raw -> cleaned -> lerobot_v30"
      fi
      set +e
      "$ROOT_DIR/trace_cleaning/scripts/process_sim1_episode.py" "${process_args[@]}"
      process_status=$?
      set -e
      if [[ "$process_status" -ne 0 ]]; then
        echo "[teach] WARNING: post-process failed with status $process_status; raw trace is kept at $trace_path" >&2
      fi
    fi
  fi

  return "$status"
}

mode="${1:-}"
if [[ -z "$mode" ]]; then
  usage
  exit 2
fi
shift || true

case "$mode" in
  help|-h|--help)
    usage
    ;;
  check)
    check
    ;;
  reset)
    reset_recover
    ;;
  teach)
    set +e
    run_teach "$@"
    status=$?
    set -e
    exit "$status"
    ;;
  teach-with-cameras)
    set +e
    run_teach --open-camera-grid "$@"
    status=$?
    set -e
    exit "$status"
    ;;
  demo)
    exec "$ROOT_DIR/replay_ocs2_target_trace.py" \
      --trace "$(resolve_demo_trace)" \
      "$@"
    ;;
  camera-grid)
    exec python3 "$ROOT_DIR/tools/camera_grid_viewer.py" "$@"
    ;;
  base-record)
    parse_base_args "$@"
    exec "$ROOT_DIR/base_drive_recorder.py" \
      --rate "$BASE_RECORD_RATE" \
      --out-dir "$BASE_TRACE_DIR" \
      "${BASE_EXTRA_ARGS[@]}"
    ;;
  base-demo|base-path-demo)
    parse_base_args "$@"
    exec "$ROOT_DIR/base_drive_path_runner.py" \
      path-demo \
      --speed "$BASE_SPEED" \
      --turn-wait "$BASE_TURN_WAIT" \
      "${BASE_EXTRA_ARGS[@]}"
    ;;
  direct-demo)
    if [[ -z "$LE_ROBOT_EPISODE" || ! -f "$LE_ROBOT_EPISODE" ]]; then
      echo "[direct-demo] Set LEROBOT_NPZ_PATH or LE_ROBOT_EPISODE to an existing v3.0-compatible NPZ file." >&2
      exit 2
    fi
    mkdir -p "$DIRECT_TRACE_DIR"
    exec "$ROOT_DIR/replay_lerobot_episode.py" \
      --episode "$LE_ROBOT_EPISODE" \
      --record-csv "$DIRECT_TRACE_DIR/lerobot_direct_replay_trace.csv" \
      "$@"
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    usage
    exit 2
    ;;
esac
