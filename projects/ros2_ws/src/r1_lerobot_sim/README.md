# R1 LeRobot Offline Simulation

This workspace replays a user-supplied LeRobot v3.0 dataset without Isaac
Sim/NVIDIA. All user LeRobot v2.1 datasets were removed on 2026-07-23. It
publishes LeRobot joint trajectories to
`/joint_states`, runs `robot_state_publisher` on
`~/Trajectory/robot/r1_fixed.urdf`, rewrites mesh URIs to
`~/Trajectory/robot/meshes`, and opens RViz2 with left/right target paths from
`action.pose`.

References used for generated Python files:

- `/home/gtk/ai_docs/docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Creating-Launch-Files.md`
- `/home/gtk/ai_docs/docs.ros.org/en/rolling/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher-py.md`
- `/home/gtk/ai_docs/docs.ros.org/en/rolling/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.md`
- `/home/gtk/ai_docs/docs.ros.org/en/rolling/p/tf2_ros_py/tf2_ros.transform_broadcaster.md`

## Build

```bash
cd /workspace/projects/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## RViz2 Replay

```bash
export LEROBOT_DATASET_PATH=/workspace/path/to/lerobot_v30_dataset
ros2 launch r1_lerobot_sim r1_lerobot_rviz_replay.launch.py
```

Useful arguments:

```bash
ros2 launch r1_lerobot_sim r1_lerobot_rviz_replay.launch.py episode_index:=0 speed:=0.5
ros2 launch r1_lerobot_sim r1_lerobot_rviz_replay.launch.py field:=observation.state enable_rviz:=false
ros2 launch r1_lerobot_sim r1_lerobot_rviz_replay.launch.py mesh_root:=${TRAJECTORY_DIR}/robot/meshes
```

## MuJoCo Smoke Test

```bash
export LEROBOT_NPZ_PATH=/workspace/path/to/episode.npz
python3 /workspace/projects/ros2_ws/src/r1_lerobot_sim/tools/mujoco_smoke_test.py
```

This verifies that MuJoCo can load the URDF after resolving package mesh paths
and that first-frame arm joint names map into the MuJoCo model.
