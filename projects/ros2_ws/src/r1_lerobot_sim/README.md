# R1 LeRobot Offline Simulation

This workspace replays a user-supplied LeRobot v3.0 dataset without Isaac
Sim/NVIDIA. All user LeRobot v2.1 datasets were removed on 2026-07-23. It
publishes LeRobot joint trajectories to
`/joint_states`, runs `robot_state_publisher` using the installed
`r1_description` URDF and meshes, and opens RViz2 with left/right target paths from
`action.pose`.

The 2026-09-15 local check verified launch arguments and the RViz2 GUI, but no episode was supplied for replay. See [the local report](../../../../reports/LOCAL_VALIDATION_20260915.md).

## Build

```bash
cd /path/to/Isaac-Robot-Simulation-main
source scripts/activate_sim1_conda.sh
./scripts/project.sh build
```

## RViz2 Replay

```bash
cd /path/to/Isaac-Robot-Simulation-main
source scripts/activate_sim1_conda.sh
export LEROBOT_DATASET_PATH=/path/to/lerobot_v30_dataset
ros2 launch r1_lerobot_sim r1_lerobot_rviz_replay.launch.py
```

Useful arguments:

```bash
ros2 launch r1_lerobot_sim r1_lerobot_rviz_replay.launch.py episode_index:=0 speed:=0.5
ros2 launch r1_lerobot_sim r1_lerobot_rviz_replay.launch.py field:=observation.state enable_rviz:=false
ros2 launch r1_lerobot_sim r1_lerobot_rviz_replay.launch.py --show-args
```

## MuJoCo Smoke Test

```bash
export LEROBOT_NPZ_PATH=/path/to/episode.npz
python3 projects/ros2_ws/src/r1_lerobot_sim/tools/mujoco_smoke_test.py
```

This verifies that MuJoCo can load the URDF after resolving package mesh paths
and that first-frame arm joint names map into the MuJoCo model.
