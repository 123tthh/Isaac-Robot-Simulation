# Isaac Sim arm joint torque diagnostics

SIM1 project overview: [`../PROJECT_OVERVIEW.md`](../PROJECT_OVERVIEW.md).
Operation guide: [`../GAME_GUIDE.md`](../GAME_GUIDE.md).

These scripts are intended for Isaac Sim Script Editor and are read-only.
They do not print to the Script Editor console. Each run writes one Markdown
report into this same folder.

Local documentation referenced:

- `/home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_simulation/articulation_controller.md`
- `/home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/physics/joint_inspector.md`
- `/home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_setup_tutorials/joint_tuning.md`
- `/home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_manipulation.md`

## Scripts

- `inspect_arm_joint1_7_drive_attrs.py`
  - Scans `/World/Robot` for prims whose paths include `left_joint1` to `left_joint7` and `right_joint1` to `right_joint7`.
  - Reports drive, stiffness, damping, maxForce/maxTorque, target, limit, break, enabled, and axis-related attributes.
  - Writes `arm_joint1_7_drive_attrs_latest.md`.

- `inspect_arm_control_graph_attrs.py`
  - Scans the whole stage for graph attributes mentioning `/arm_joint_cmd`, joint names, command inputs, topic names, and target prims.
  - Reports values and USD connections so graph wiring and joint name order can be checked from the Markdown file.
  - Writes `arm_control_graph_attrs_latest.md`.

## ROS-side checks to run in a terminal

```bash
source ~/ros2_ws/install/setup.bash
ros2 control list_controllers
ros2 topic hz /arm_joint_cmd
ros2 topic echo --once /arm_joint_cmd
ros2 topic echo --once /joint_states
ros2 topic echo --once /fsm_command
```

If `/arm_joint_cmd` is still publishing but some arm joints stop receiving meaningful commands, compare the vector length and joint order with the graph `jointNames`.

## Likely fault classes

1. Command stream problem: OCS2 stops commanding some arm joints, publishes NaN/zero/short arrays, or switches FSM/controller state.
2. Graph mapping problem: `jointNames` order is wrong, arm joints are missing, or the Articulation Action input uses the wrong command channel.
3. Drive saturation/disable problem: maxForce/maxTorque becomes zero or too small, stiffness/damping changes, or limits/break settings are hit.
4. Multiple controller problem: two publishers or two Isaac graph paths drive the same articulation with conflicting command modes.
5. Physics instability/contact problem: a collision/contact spike or joint limit impact causes the joint to clamp, sleep, or appear torque-free.
