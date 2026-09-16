# Path Audit

Date: 2026-07-23

Scanned roots:

- `projects/ros2_ws`
- `projects/teleoperation`
- `data/ros2_log`

Excluded generated/build/cache trees (`.git`, `build`, `install`, `log`,
`__pycache__`) and trace payloads. The scan matched 222 files. Raw occurrence
counts were: `/home/gtk` 3,603, `/home/bt` 10, `Desktop` 2,389, `522.usd`
767, and `.usd` 2,539. Most large counts come from historical graph exports,
URDF snapshots, and diagnostics; rewriting those records would destroy their
provenance.

## Replacement map

| Host path | Runtime replacement |
| --- | --- |
| `/home/gtk/ros2_ws` | `${ROS2_WS:-/workspace/projects/ros2_ws}` |
| `/home/gtk/teleoperation` | `${TRAJECTORY_DIR:-/workspace/projects/teleoperation}` |
| `/home/gtk/teleoperation/sim1` | `${SIM1_DIR:-/workspace/projects/teleoperation/sim1}` |
| `/home/gtk/ros2_log` | `${ROS2_LOG_DIR:-/workspace/data/ros2_log}` |
| `.../scenes/r1_workcell/workcell.usd` | `${ISAAC_USD_PATH:-/workspace/assets/scenes/scene.usd}` |

## Current runtime files

| File | Use | Disposition |
| --- | --- | --- |
| `projects/teleoperation/sim1/game_control.sh` | Main SIM1 launcher | Already environment-driven |
| `projects/teleoperation/sim1/script_gripper_v4.2.py` | Gripper diagnostic log output | Replaced with `ROS2_LOG_DIR` |
| `projects/teleoperation/sim1/isaac_script_editor_diagnostics/inspect_arm_control_graph_attrs.py` | Report output | Replaced with `SIM1_DIR` |
| `projects/teleoperation/sim1/isaac_script_editor_diagnostics/inspect_arm_joint1_7_drive_attrs.py` | Report output | Replaced with `SIM1_DIR` |
| `projects/teleoperation/sim1/isaac_save_arm_initial_pose_to_usd.py` | Pose snapshot input | Replaced with `SIM1_DIR` |
| `projects/teleoperation/sim1/trace_cleaning/scripts/clean_ocs2_trace.py` | Cleaned trace output | Replaced with `SIM1_DIR` |
| `projects/teleoperation/sim1/trace_cleaning/scripts/clean_sim1_episode.py` | Cleaned episode output | Replaced with `SIM1_DIR` |
| `projects/teleoperation/sim1/trace_cleaning/scripts/convert_ocs2_trace_to_lerobot_v21.py` | LeRobot v2.1 output | Replaced with `SIM1_DIR` |
| `projects/teleoperation/sim1/trace_cleaning/scripts/convert_ocs2_trace_to_lerobot_v30.py` | LeRobot v3.0 output | Replaced with `SIM1_DIR` |
| `projects/ros2_ws/src/r1_lerobot_sim/launch/r1_lerobot_rviz_replay.launch.py` | Dataset, URDF and mesh defaults | Uses `LEROBOT_DATASET_PATH`/`TRAJECTORY_DIR` |
| `projects/ros2_ws/src/r1_lerobot_sim/r1_lerobot_sim/joint_state_replay_node.py` | Dataset and URDF defaults | Uses `LEROBOT_DATASET_PATH`/`ROS2_WS` |
| `projects/ros2_ws/src/r1_lerobot_sim/r1_lerobot_sim/pose_path_replay_node.py` | Dataset default | Uses `LEROBOT_DATASET_PATH` |
| `projects/ros2_ws/src/r1_lerobot_sim/tools/mujoco_smoke_test.py` | URDF and episode defaults | Uses `ROS2_WS`/`LEROBOT_NPZ_PATH` |
| `projects/IsaacLab-Arena/docker/Dockerfile.isaaclab_arena` | Isaac Sim base image | Updated default from 5.0.0 to 5.1.0 |

Every modified runtime file has a `.bak_dockerize` backup under
`backups/dockerize/`, preserving its project-relative path. Centralizing
backups keeps them out of ROS package installation output. Exact local
documentation paths at the top of Python files are retained intentionally to
satisfy project provenance requirements.

## Intentionally unchanged

- `data/ros2_log/**`: historical logs, generated graphs, diagnostics and prior
  experiment reports.
- `projects/teleoperation/sim1/*.md` and Python header reference lists: evidence
  of original sources and local documentation.
- abandoned URDF snapshots under `projects/ros2_ws/src/robot/urdf`: historical
  source locations embedded in mesh declarations.
- Isaac Lab and Arena upstream sources: managed independently by their
  upstream repositories.

Reproduce the scan:

```bash
rg -n --hidden \
  -g '!**/.git/**' -g '!**/trace_data/**' -g '!**/build/**' \
  -g '!**/install/**' -g '!**/log/**' -g '!**/__pycache__/**' \
  '/home/(gtk|bt)|/workspace|Desktop|522\.usd|\.usd' \
  projects/ros2_ws projects/teleoperation data/ros2_log
```
