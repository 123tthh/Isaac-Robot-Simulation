> **Historical record / 历史记录**：本页描述其记录日期的状态；当前运行以[中文指南](../docs/getting-started.zh-CN.md) / [English guide](../docs/getting-started.md)为准。Historical failures and paths are not current release claims.

# LeRobot v2.1 Data Deletion Report (2026-07-23)

Date: 2026-07-23

Status: applied successfully. Post-deletion scans found zero user
`lerobot_v21` directories and zero user `info.json` files declaring
`codebase_version: v2.1`.

## User data selected for deletion

- `/home/gtk/teleoperation/sim1/trace_data/lerobot_v21`
- `${PROJECT_ROOT}/projects/teleoperation/sim1/trace_data/lerobot_v21`
- `/home/gtk/ros2_log/SIM1/trace_data/lerobot_v21`
- `${PROJECT_ROOT}/data/ros2_log/SIM1/trace_data/lerobot_v21`
- `/home/gtk/ros2_ws/teleoperation/place_tray_middle`
- `${PROJECT_ROOT}/projects/teleoperation/converted_lerobot/local/place_tray_middle`

The last two directories do not contain `v21` in their names, but their
`meta/info.json` files explicitly declare `"codebase_version": "v2.1"`.

## Preserved code

- v2.1 conversion scripts and the v2.1-to-v3.0 migration helper remain as
  source code; they are not datasets.
- Isaac Lab Arena test fixtures and GR00T configuration metadata remain because
  they are upstream source/test dependencies, not user trajectory datasets.

## Regeneration prevention

`process_sim1_episode.py` and the automatic post-teach pipeline generate
LeRobot v3.0 by default. LeRobot v2.1 generation remains available with
`--with-v21`, `sim1_convert_v21`, or
`SIM1_GENERATE_LEROBOT_V21=1`. Replay defaults no longer point at the deleted
dataset; callers must provide `LEROBOT_DATASET_PATH` or
`LEROBOT_NPZ_PATH`.

Deletion is destructive and the listed data is not recoverable from this
project after removal.
