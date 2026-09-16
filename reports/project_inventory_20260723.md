> **Historical record / 历史记录**：本页描述其记录日期的状态；当前运行以[中文指南](../docs/getting-started.zh-CN.md) / [English guide](../docs/getting-started.md)为准。Historical failures and paths are not current release claims.

# Project Inventory (2026-07-23)

Date: 2026-07-23

Normalized root: `${PROJECT_ROOT}`

## Main content

| Path | Approximate size | Purpose |
| --- | ---: | --- |
| `projects/teleoperation` | 247 GB | Canonical SIM1 code, retained traces, robot and trajectory tools |
| `data/ros2_log` | 17 GB | Historical logs and older independent trace archive |
| `projects/ros2_ws` | 2.4 GB | ROS 2/OCS2 sources plus integrated `r1_lerobot_sim` |
| `projects/IsaacLab-Arena` | 2.1 GB | Full Arena repository and Isaac Lab submodule |
| `assets/datasets/sac-m` | 313 MB | USD and related assets |
| `backups` | 276 KB | 13 Docker path backups and 7 v2.1-removal transition backups |

The canonical SIM1 trace tree contains 234,494 regular files in 90
directories and occupies approximately 247 GB. The old path
`/home/gtk/teleoperation/sim1/trace_data` is now a compatibility symlink to this
tree. No `.partial_migration` files remain.

## Final data policy

- Retained four June camera trajectories longer than 30 seconds.
- Retained the only explicitly successful July trajectory.
- Removed two short June camera trajectories and all non-success July
  trajectories.
- Removed all user LeRobot v2.1 datasets. Generation remains available
  explicitly, while automatic post-teach v2.1 output is disabled by default.
- Preserved LeRobot v2.1 conversion/migration source code and upstream Arena
  test/config fixtures.

## Validation summary

- Project layout verifier: 0 failures.
- Current Python source: 28 files parsed successfully.
- Shell scripts: Bash syntax checks passed.
- Docker Compose v1 configuration: passed.
- Integrated `r1_lerobot_sim`: clean Humble `colcon build` passed.
- LeRobot v2.1 user data audit: 0 directories, 0 metadata declarations.
- Trace migration: compatibility link resolves correctly, 0 partial files.

The host GPU/driver and NVIDIA Docker runtime are verified outside the
sandbox. The pre-existing `issac_ocs_docker:latest` image can access the GPU;
the normalized source image has not been rebuilt because its default
`isaac-lab-ros2:latest` base image is not present locally. See
`environment_compatibility_report_20260723.md`.
