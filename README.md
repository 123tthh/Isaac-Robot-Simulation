# Isaac OCS Project

Portable working copy for Isaac Sim 5.1.0, ROS 2/OCS2 and SIM1 trace
handling. IsaacLab-Arena is an optional experiment source, not a core runtime
dependency.

## Simulation demos

Two animated excerpts show the Isaac Sim robot working in the simulation scene.
Click either preview to watch its full recording.

### Part 1

[![Part 1: robot interacting with the simulation scene](docs/demo/test_part1.gif)](docs/demo/test_part1.mp4)

### Part 2

[![Part 2: close-up of the robot handling an object](docs/demo/test_part2.gif)](docs/demo/test_part2.mp4)

## Start here

Canonical host startup:

```bash
cd /path/to/isaac_ocs_project
./scripts/project_control_20260723.sh build
./scripts/project_control_20260723.sh preflight
```

Then follow [the Chinese startup guide](docs/startup_guide_20260723_zh.md). The
[document index](docs/document_index_20260723_zh.md) separates current instructions
from historical handoff and diagnosis notes.

## Layout

| Directory | Content | Original source |
| --- | --- | --- |
| `projects/ros2_ws` | ROS 2 and OCS2 source workspace | `/home/gtk/ros2_ws` |
| `projects/Trajectory` | SIM1 and trajectory tools | `/home/gtk/Trajectory` |
| `projects/IsaacLab-Arena` | Optional Arena experiments; excluded from core Git | `/home/gtk/Desktop/isaaclab_arena/IsaacLab-Arena` |
| `data/ros2_log` | Historical runtime and diagnosis logs | `/home/gtk/ros2_log` |
| `assets/datasets/sac-m` | USD/dataset assets | `/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m` |
| `docker` | Container configuration and usage | consolidated |
| `backups/dockerize` | Pre-change runtime files with `.bak_dockerize` suffix | generated |
| `reports` | Cleanup and audit reports | generated |
| `scripts` | Project maintenance and environment checks | generated |
| `outputs` | New runtime/training outputs | generated |

The canonical R1 robot package is `projects/ros2_ws/src/robot`; see
`docs/ROBOT_RESOURCE_POLICY.md`. External repositories and exact commits are
recorded in `dependencies/repositories.lock.yaml`.

The old standalone `Trajectory/SIM` package was useful and was integrated as
`projects/ros2_ws/src/r1_lerobot_sim`; generated build/install/log files and
the now-redundant copied `SIM` directory were removed only from this normalized
copy.

All user LeRobot v2.1 datasets, including the former
`ros2_ws/Trajectory/place_tray_middle` dataset, were removed on 2026-07-23.
Replay now requires an explicit v3.0 dataset through
`LEROBOT_DATASET_PATH` or `LEROBOT_NPZ_PATH`. LeRobot v2.1 generation remains
available on demand with `--with-v21`, `sim1_convert_v21`, or
`SIM1_GENERATE_LEROBOT_V21=1`.

Trace data is migrated per file with byte comparison before the source file is
unlinked. See `reports/trace_data_cleanup_report_20260723.md` for the applied retention
rules and `docker/README_DOCKER.md` for container use.
