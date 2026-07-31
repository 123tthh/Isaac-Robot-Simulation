# PGIA Gripper Full Diagnosis

- SIM1 工程总纲: [`../PROJECT_OVERVIEW.md`](../PROJECT_OVERVIEW.md)
- 相关操作指导: [`../GAME_GUIDE.md`](../GAME_GUIDE.md)
- time: `2026-05-28 11:26:20.731978`
- stage root layer: `/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd`
- edit target: `/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd`
- output dir: `/home/gtk/ros2_log/gripper_full_diagnosis`
- json: `/home/gtk/ros2_log/gripper_full_diagnosis/gripper_full_diagnosis.json`
- runtime trace csv: `/home/gtk/ros2_log/gripper_full_diagnosis/gripper_runtime_trace.csv`
- test commands: `/home/gtk/ros2_log/gripper_full_diagnosis/gripper_test_commands.sh`

# 0. Background / Debug Context

本报告用于集中排查 Isaac Sim PGIA 夹爪问题，包括：

- USD joint / drive / link / xform / layer stack
- Action Graph 连线
- ScriptNode pin 与脚本内容摘要
- ArticulationController 的 positionCommand / effortCommand 状态
- DynamicControl runtime DOF 状态
- 左右夹爪行程、gap、重力轴投影
- ROS2 测试命令

重要背景：

- v3.8 纯 effort 同号施力不能严格保证 joint1/joint2 等行程。
- right_PGIA_joint2 如果 prismatic 轴接近重力方向，stiffness=0 时容易下坠。
- 直接写 DOF position 已验证右侧模型本体可以对称运动。
- 严格等行程最干净的路线仍是 positionCommand + [pos, pos]。

# 1. Auto Analysis

## Critical

- None

## Warnings

- PGIA Drive stiffness is high. If effortCommand is connected, Drive may fight effort control.
- left_j1_upper axis has strong gravity projection: dot_gravity_down=0.9990927912196993. Gravity compensation or Drive holding is required.
- left_j2_lower axis has strong gravity projection: dot_gravity_down=0.9990927912196993. Gravity compensation or Drive holding is required.
- right_j1_upper axis has strong gravity projection: dot_gravity_down=0.9989973870601369. Gravity compensation or Drive holding is required.
- right_j2_lower axis has strong gravity projection: dot_gravity_down=0.9989973870601369. Gravity compensation or Drive holding is required.
- Graph appears to use effortCommand. Equal travel of joint1/joint2 is not guaranteed unless software PD/sync loop is used.

## Recommendations

- Joint limits appear symmetric from gap calculation.
- For strict equal travel, prefer positionCommand with identical [pos, pos] for joint1/joint2.
- For effort-based position control, use software PD: effort = Kp*(target-current) - Kd*velocity + gravity_comp.
- Do not judge control failure from repeated same command. Test close-open-close sequence.
- If FastDDS SHM errors appear, run: sudo rm -f /dev/shm/fastrtps_port* and consider export RMW_FASTRTPS_USE_SHM=0.

## Mode Guess

- `usd_drive`: `strong_position_drive`
- `graph_control`: `effortCommand`

# 2. Gap / Travel Calculation

- `left_min_gap_m`: `0.02770000037997961`
- `left_max_gap_m`: `0.25969999463558197`
- `right_min_gap_m`: `0.02770000037997961`
- `right_max_gap_m`: `0.25969999463558197`
- `left_min_gap_mm`: `27.70000037997961`
- `left_max_gap_mm`: `259.69999463558196`
- `right_min_gap_mm`: `27.70000037997961`
- `right_max_gap_mm`: `259.69999463558196`
- `limit_gap_symmetric`: `True`

Expected relation:

```text
real_gap = BASE_GAP + joint1_position + joint2_position
strict equal travel requires: joint1_position == joint2_position
```

# 3. USD Joint Details

## left_j1_upper
- path: `/World/Robot/joints/left_PGIA_joint1`
- valid: `True`
- typeName: `PhysicsPrismaticJoint`
- schemas: `['PhysicsJointStateAPI:linear', 'PhysxJointAPI', 'PhysicsDriveAPI:linear', 'IsaacJointAPI']`

### joint
- `axis`: `Y`
- `lowerLimit`: `{'value': 0.004000000189989805, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/left_PGIA_joint1.physics:lowerLimit', 'value': 0.004000000189989805}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint1.physics:lowerLimit', 'value': 0.009999999776482582}]}`
- `upperLimit`: `{'value': 0.11999999731779099, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/left_PGIA_joint1.physics:upperLimit', 'value': 0.11999999731779099}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint1.physics:upperLimit', 'value': 0.11999999731779099}]}`
- `jointEnabled`: `{'value': True, 'stack': []}`
- `excludeFromArticulation`: `{'value': False, 'stack': []}`
- `collisionEnabled`: `{'value': False, 'stack': []}`
- `body0`: `['/World/Robot/left_PGIA_base_link']`
- `body1`: `['/World/Robot/left_PGIA_link1']`
- `localPos0`: `{'value': [0.0, 0.0, 0.14100000262260437], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint1.physics:localPos0', 'value': [0.0, 0.0, 0.14100000262260437]}]}`
- `localRot0`: `{'value': '(0, 1, 0, 0)', 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint1.physics:localRot0', 'value': '(0, 1, 0, 0)'}]}`
- `localPos1`: `{'value': [0.0, 0.0, 0.0], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint1.physics:localPos1', 'value': [0.0, 0.0, 0.0]}]}`
- `localRot1`: `{'value': '(0, 1, 0, 0)', 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint1.physics:localRot1', 'value': '(0, 1, 0, 0)'}]}`

### drive_linear
- `exists`: `True`
- `type`: `{'value': 'force', 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/left_PGIA_joint1.drive:linear:physics:type', 'value': 'force'}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint1.drive:linear:physics:type', 'value': 'force'}]}`
- `stiffness`: `{'value': 2000.0, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/left_PGIA_joint1.drive:linear:physics:stiffness', 'value': 2000.0}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint1.drive:linear:physics:stiffness', 'value': 7.092898368835449}]}`
- `damping`: `{'value': 200.0, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/left_PGIA_joint1.drive:linear:physics:damping', 'value': 200.0}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint1.drive:linear:physics:damping', 'value': 0.0028371592052280903}]}`
- `maxForce`: `{'value': 500.0, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/left_PGIA_joint1.drive:linear:physics:maxForce', 'value': 500.0}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint1.drive:linear:physics:maxForce', 'value': 140.0}]}`
- `targetPosition`: `{'value': 0.019999999552965164, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/left_PGIA_joint1.drive:linear:physics:targetPosition', 'value': 0.019999999552965164}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint1.drive:linear:physics:targetPosition', 'value': 0.019999999552965164}]}`
- `targetVelocity`: `{'value': 0.0, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/left_PGIA_joint1.drive:linear:physics:targetVelocity', 'value': 0.0}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint1.drive:linear:physics:targetVelocity', 'value': 0.0}]}`

### world_axis_from_body0
`{'axis': [-0.04248149540795254, -0.002986148143475198, -0.9990927912196993], 'dot_gravity_down': 0.9990927912196993}`

### suspicious_attrs
`{'drive:linear:physics:damping': {'value': 200.0, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/left_PGIA_joint1.drive:linear:physics:damping', 'value': 200.0}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint1.drive:linear:physics:damping', 'value': 0.0028371592052280903}]}, 'drive:linear:physics:maxForce': {'value': 500.0, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/left_PGIA_joint1.drive:linear:physics:maxForce', 'value': 500.0}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint1.drive:linear:physics:maxForce', 'value': 140.0}]}, 'drive:linear:physics:stiffness': {'value': 2000.0, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/left_PGIA_joint1.drive:linear:physics:stiffness', 'value': 2000.0}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint1.drive:linear:physics:stiffness', 'value': 7.092898368835449}]}, 'drive:linear:physics:targetPosition': {'value': 0.019999999552965164, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/left_PGIA_joint1.drive:linear:physics:targetPosition', 'value': 0.019999999552965164}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint1.drive:linear:physics:targetPosition', 'value': 0.019999999552965164}]}, 'drive:linear:physics:targetVelocity': {'value': 0.0, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/left_PGIA_joint1.drive:linear:physics:targetVelocity', 'value': 0.0}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint1.drive:linear:physics:targetVelocity', 'value': 0.0}]}, 'drive:linear:physics:type': {'value': 'force', 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/left_PGIA_joint1.drive:linear:physics:type', 'value': 'force'}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint1.drive:linear:physics:type', 'value': 'force'}]}, 'isaac:physics:AccelerationLimit': {'value': [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_robot.usd', 'path': '/r1/joints/left_PGIA_joint1.isaac:physics:AccelerationLimit', 'value': None}]}, 'isaac:physics:JerkLimit': {'value': [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_robot.usd', 'path': '/r1/joints/left_PGIA_joint1.isaac:physics:JerkLimit', 'value': None}]}, 'physics:lowerLimit': {'value': 0.004000000189989805, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/left_PGIA_joint1.physics:lowerLimit', 'value': 0.004000000189989805}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint1.physics:lowerLimit', 'value': 0.009999999776482582}]}, 'physics:upperLimit': {'value': 0.11999999731779099, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/left_PGIA_joint1.physics:upperLimit', 'value': 0.11999999731779099}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint1.physics:upperLimit', 'value': 0.11999999731779099}]}}`

## left_j2_lower
- path: `/World/Robot/joints/left_PGIA_joint2`
- valid: `True`
- typeName: `PhysicsPrismaticJoint`
- schemas: `['PhysicsJointStateAPI:linear', 'PhysxJointAPI', 'PhysicsDriveAPI:linear', 'IsaacJointAPI']`

### joint
- `axis`: `Y`
- `lowerLimit`: `{'value': 0.004000000189989805, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/left_PGIA_joint2.physics:lowerLimit', 'value': 0.004000000189989805}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint2.physics:lowerLimit', 'value': 0.009999999776482582}]}`
- `upperLimit`: `{'value': 0.11999999731779099, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/left_PGIA_joint2.physics:upperLimit', 'value': 0.11999999731779099}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint2.physics:upperLimit', 'value': 0.11999999731779099}]}`
- `jointEnabled`: `{'value': True, 'stack': []}`
- `excludeFromArticulation`: `{'value': False, 'stack': []}`
- `collisionEnabled`: `{'value': False, 'stack': []}`
- `body0`: `['/World/Robot/left_PGIA_base_link']`
- `body1`: `['/World/Robot/left_PGIA_link2']`
- `localPos0`: `{'value': [0.0, 0.0, 0.14100000262260437], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint2.physics:localPos0', 'value': [0.0, 0.0, 0.14100000262260437]}]}`
- `localRot0`: `{'value': '(1, 0, 0, 0)', 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint2.physics:localRot0', 'value': '(1, 0, 0, 0)'}]}`
- `localPos1`: `{'value': [0.0, 0.0, 0.0], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint2.physics:localPos1', 'value': [0.0, 0.0, 0.0]}]}`
- `localRot1`: `{'value': '(1, 0, 0, 0)', 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint2.physics:localRot1', 'value': '(1, 0, 0, 0)'}]}`

### drive_linear
- `exists`: `True`
- `type`: `{'value': 'force', 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/left_PGIA_joint2.drive:linear:physics:type', 'value': 'force'}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint2.drive:linear:physics:type', 'value': 'force'}]}`
- `stiffness`: `{'value': 2000.0, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/left_PGIA_joint2.drive:linear:physics:stiffness', 'value': 2000.0}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint2.drive:linear:physics:stiffness', 'value': 5.679325103759766}]}`
- `damping`: `{'value': 200.0, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/left_PGIA_joint2.drive:linear:physics:damping', 'value': 200.0}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint2.drive:linear:physics:damping', 'value': 0.002271729987114668}]}`
- `maxForce`: `{'value': 500.0, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/left_PGIA_joint2.drive:linear:physics:maxForce', 'value': 500.0}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint2.drive:linear:physics:maxForce', 'value': 140.0}]}`
- `targetPosition`: `{'value': 0.019999999552965164, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/left_PGIA_joint2.drive:linear:physics:targetPosition', 'value': 0.019999999552965164}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint2.drive:linear:physics:targetPosition', 'value': 0.019999999552965164}]}`
- `targetVelocity`: `{'value': 0.0, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/left_PGIA_joint2.drive:linear:physics:targetVelocity', 'value': 0.0}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint2.drive:linear:physics:targetVelocity', 'value': 0.0}]}`

### world_axis_from_body0
`{'axis': [-0.04248149540795254, -0.002986148143475198, -0.9990927912196993], 'dot_gravity_down': 0.9990927912196993}`

### suspicious_attrs
`{'drive:linear:physics:damping': {'value': 200.0, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/left_PGIA_joint2.drive:linear:physics:damping', 'value': 200.0}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint2.drive:linear:physics:damping', 'value': 0.002271729987114668}]}, 'drive:linear:physics:maxForce': {'value': 500.0, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/left_PGIA_joint2.drive:linear:physics:maxForce', 'value': 500.0}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint2.drive:linear:physics:maxForce', 'value': 140.0}]}, 'drive:linear:physics:stiffness': {'value': 2000.0, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/left_PGIA_joint2.drive:linear:physics:stiffness', 'value': 2000.0}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint2.drive:linear:physics:stiffness', 'value': 5.679325103759766}]}, 'drive:linear:physics:targetPosition': {'value': 0.019999999552965164, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/left_PGIA_joint2.drive:linear:physics:targetPosition', 'value': 0.019999999552965164}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint2.drive:linear:physics:targetPosition', 'value': 0.019999999552965164}]}, 'drive:linear:physics:targetVelocity': {'value': 0.0, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/left_PGIA_joint2.drive:linear:physics:targetVelocity', 'value': 0.0}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint2.drive:linear:physics:targetVelocity', 'value': 0.0}]}, 'drive:linear:physics:type': {'value': 'force', 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/left_PGIA_joint2.drive:linear:physics:type', 'value': 'force'}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint2.drive:linear:physics:type', 'value': 'force'}]}, 'isaac:physics:AccelerationLimit': {'value': [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_robot.usd', 'path': '/r1/joints/left_PGIA_joint2.isaac:physics:AccelerationLimit', 'value': None}]}, 'isaac:physics:JerkLimit': {'value': [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_robot.usd', 'path': '/r1/joints/left_PGIA_joint2.isaac:physics:JerkLimit', 'value': None}]}, 'physics:lowerLimit': {'value': 0.004000000189989805, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/left_PGIA_joint2.physics:lowerLimit', 'value': 0.004000000189989805}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint2.physics:lowerLimit', 'value': 0.009999999776482582}]}, 'physics:upperLimit': {'value': 0.11999999731779099, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/left_PGIA_joint2.physics:upperLimit', 'value': 0.11999999731779099}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/left_PGIA_joint2.physics:upperLimit', 'value': 0.11999999731779099}]}}`

## right_j1_upper
- path: `/World/Robot/joints/right_PGIA_joint1`
- valid: `True`
- typeName: `PhysicsPrismaticJoint`
- schemas: `['PhysicsJointStateAPI:linear', 'PhysxJointAPI', 'PhysicsDriveAPI:linear', 'IsaacJointAPI']`

### joint
- `axis`: `Y`
- `lowerLimit`: `{'value': 0.004000000189989805, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/right_PGIA_joint1.physics:lowerLimit', 'value': 0.004000000189989805}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint1.physics:lowerLimit', 'value': 0.009999999776482582}]}`
- `upperLimit`: `{'value': 0.11999999731779099, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/right_PGIA_joint1.physics:upperLimit', 'value': 0.11999999731779099}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint1.physics:upperLimit', 'value': 0.11999999731779099}]}`
- `jointEnabled`: `{'value': True, 'stack': []}`
- `excludeFromArticulation`: `{'value': False, 'stack': []}`
- `collisionEnabled`: `{'value': False, 'stack': []}`
- `body0`: `['/World/Robot/right_PGIA_base_link']`
- `body1`: `['/World/Robot/right_PGIA_link1']`
- `localPos0`: `{'value': [0.0, 0.0, 0.14100000262260437], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint1.physics:localPos0', 'value': [0.0, 0.0, 0.14100000262260437]}]}`
- `localRot0`: `{'value': '(0, 1, 0, 0)', 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint1.physics:localRot0', 'value': '(0, 1, 0, 0)'}]}`
- `localPos1`: `{'value': [0.0, 0.0, 0.0], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint1.physics:localPos1', 'value': [0.0, 0.0, 0.0]}]}`
- `localRot1`: `{'value': '(0, 1, 0, 0)', 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint1.physics:localRot1', 'value': '(0, 1, 0, 0)'}]}`

### drive_linear
- `exists`: `True`
- `type`: `{'value': 'force', 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/right_PGIA_joint1.drive:linear:physics:type', 'value': 'force'}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint1.drive:linear:physics:type', 'value': 'force'}]}`
- `stiffness`: `{'value': 2000.0, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/right_PGIA_joint1.drive:linear:physics:stiffness', 'value': 2000.0}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint1.drive:linear:physics:stiffness', 'value': 7.092749118804932}]}`
- `damping`: `{'value': 200.0, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/right_PGIA_joint1.drive:linear:physics:damping', 'value': 200.0}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint1.drive:linear:physics:damping', 'value': 0.0028370998334139585}]}`
- `maxForce`: `{'value': 500.0, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/right_PGIA_joint1.drive:linear:physics:maxForce', 'value': 500.0}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint1.drive:linear:physics:maxForce', 'value': 140.0}]}`
- `targetPosition`: `{'value': 0.019999999552965164, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/right_PGIA_joint1.drive:linear:physics:targetPosition', 'value': 0.019999999552965164}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint1.drive:linear:physics:targetPosition', 'value': 0.019999999552965164}]}`
- `targetVelocity`: `{'value': 0.0, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/right_PGIA_joint1.drive:linear:physics:targetVelocity', 'value': 0.0}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint1.drive:linear:physics:targetVelocity', 'value': 0.0}]}`

### world_axis_from_body0
`{'axis': [-0.0446227295994317, 0.0036100761924177105, -0.9989973870601369], 'dot_gravity_down': 0.9989973870601369}`

### suspicious_attrs
`{'drive:linear:physics:damping': {'value': 200.0, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/right_PGIA_joint1.drive:linear:physics:damping', 'value': 200.0}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint1.drive:linear:physics:damping', 'value': 0.0028370998334139585}]}, 'drive:linear:physics:maxForce': {'value': 500.0, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/right_PGIA_joint1.drive:linear:physics:maxForce', 'value': 500.0}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint1.drive:linear:physics:maxForce', 'value': 140.0}]}, 'drive:linear:physics:stiffness': {'value': 2000.0, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/right_PGIA_joint1.drive:linear:physics:stiffness', 'value': 2000.0}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint1.drive:linear:physics:stiffness', 'value': 7.092749118804932}]}, 'drive:linear:physics:targetPosition': {'value': 0.019999999552965164, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/right_PGIA_joint1.drive:linear:physics:targetPosition', 'value': 0.019999999552965164}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint1.drive:linear:physics:targetPosition', 'value': 0.019999999552965164}]}, 'drive:linear:physics:targetVelocity': {'value': 0.0, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/right_PGIA_joint1.drive:linear:physics:targetVelocity', 'value': 0.0}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint1.drive:linear:physics:targetVelocity', 'value': 0.0}]}, 'drive:linear:physics:type': {'value': 'force', 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/right_PGIA_joint1.drive:linear:physics:type', 'value': 'force'}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint1.drive:linear:physics:type', 'value': 'force'}]}, 'isaac:physics:AccelerationLimit': {'value': [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_robot.usd', 'path': '/r1/joints/right_PGIA_joint1.isaac:physics:AccelerationLimit', 'value': None}]}, 'isaac:physics:JerkLimit': {'value': [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_robot.usd', 'path': '/r1/joints/right_PGIA_joint1.isaac:physics:JerkLimit', 'value': None}]}, 'physics:lowerLimit': {'value': 0.004000000189989805, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/right_PGIA_joint1.physics:lowerLimit', 'value': 0.004000000189989805}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint1.physics:lowerLimit', 'value': 0.009999999776482582}]}, 'physics:upperLimit': {'value': 0.11999999731779099, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/right_PGIA_joint1.physics:upperLimit', 'value': 0.11999999731779099}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint1.physics:upperLimit', 'value': 0.11999999731779099}]}}`

## right_j2_lower
- path: `/World/Robot/joints/right_PGIA_joint2`
- valid: `True`
- typeName: `PhysicsPrismaticJoint`
- schemas: `['PhysicsJointStateAPI:linear', 'PhysxJointAPI', 'PhysicsDriveAPI:linear', 'IsaacJointAPI']`

### joint
- `axis`: `Y`
- `lowerLimit`: `{'value': 0.004000000189989805, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/right_PGIA_joint2.physics:lowerLimit', 'value': 0.004000000189989805}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint2.physics:lowerLimit', 'value': 0.009999999776482582}]}`
- `upperLimit`: `{'value': 0.11999999731779099, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/right_PGIA_joint2.physics:upperLimit', 'value': 0.11999999731779099}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint2.physics:upperLimit', 'value': 0.11999999731779099}]}`
- `jointEnabled`: `{'value': True, 'stack': []}`
- `excludeFromArticulation`: `{'value': False, 'stack': []}`
- `collisionEnabled`: `{'value': False, 'stack': []}`
- `body0`: `['/World/Robot/right_PGIA_base_link']`
- `body1`: `['/World/Robot/right_PGIA_link2']`
- `localPos0`: `{'value': [0.0, 0.0, 0.14100000262260437], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint2.physics:localPos0', 'value': [0.0, 0.0, 0.14100000262260437]}]}`
- `localRot0`: `{'value': '(1, 0, 0, 0)', 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint2.physics:localRot0', 'value': '(1, 0, 0, 0)'}]}`
- `localPos1`: `{'value': [0.0, 0.0, 0.0], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint2.physics:localPos1', 'value': [0.0, 0.0, 0.0]}]}`
- `localRot1`: `{'value': '(1, 0, 0, 0)', 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint2.physics:localRot1', 'value': '(1, 0, 0, 0)'}]}`

### drive_linear
- `exists`: `True`
- `type`: `{'value': 'force', 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/right_PGIA_joint2.drive:linear:physics:type', 'value': 'force'}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint2.drive:linear:physics:type', 'value': 'force'}]}`
- `stiffness`: `{'value': 2000.0, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/right_PGIA_joint2.drive:linear:physics:stiffness', 'value': 2000.0}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint2.drive:linear:physics:stiffness', 'value': 5.679229736328125}]}`
- `damping`: `{'value': 200.0, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/right_PGIA_joint2.drive:linear:physics:damping', 'value': 200.0}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint2.drive:linear:physics:damping', 'value': 0.0022716918028891087}]}`
- `maxForce`: `{'value': 500.0, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/right_PGIA_joint2.drive:linear:physics:maxForce', 'value': 500.0}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint2.drive:linear:physics:maxForce', 'value': 140.0}]}`
- `targetPosition`: `{'value': 0.019999999552965164, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/right_PGIA_joint2.drive:linear:physics:targetPosition', 'value': 0.019999999552965164}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint2.drive:linear:physics:targetPosition', 'value': 0.019999999552965164}]}`
- `targetVelocity`: `{'value': 0.0, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/right_PGIA_joint2.drive:linear:physics:targetVelocity', 'value': 0.0}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint2.drive:linear:physics:targetVelocity', 'value': 0.0}]}`

### world_axis_from_body0
`{'axis': [-0.0446227295994317, 0.0036100761924177105, -0.9989973870601369], 'dot_gravity_down': 0.9989973870601369}`

### suspicious_attrs
`{'drive:linear:physics:damping': {'value': 200.0, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/right_PGIA_joint2.drive:linear:physics:damping', 'value': 200.0}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint2.drive:linear:physics:damping', 'value': 0.0022716918028891087}]}, 'drive:linear:physics:maxForce': {'value': 500.0, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/right_PGIA_joint2.drive:linear:physics:maxForce', 'value': 500.0}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint2.drive:linear:physics:maxForce', 'value': 140.0}]}, 'drive:linear:physics:stiffness': {'value': 2000.0, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/right_PGIA_joint2.drive:linear:physics:stiffness', 'value': 2000.0}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint2.drive:linear:physics:stiffness', 'value': 5.679229736328125}]}, 'drive:linear:physics:targetPosition': {'value': 0.019999999552965164, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/right_PGIA_joint2.drive:linear:physics:targetPosition', 'value': 0.019999999552965164}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint2.drive:linear:physics:targetPosition', 'value': 0.019999999552965164}]}, 'drive:linear:physics:targetVelocity': {'value': 0.0, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/right_PGIA_joint2.drive:linear:physics:targetVelocity', 'value': 0.0}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint2.drive:linear:physics:targetVelocity', 'value': 0.0}]}, 'drive:linear:physics:type': {'value': 'force', 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/right_PGIA_joint2.drive:linear:physics:type', 'value': 'force'}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint2.drive:linear:physics:type', 'value': 'force'}]}, 'isaac:physics:AccelerationLimit': {'value': [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_robot.usd', 'path': '/r1/joints/right_PGIA_joint2.isaac:physics:AccelerationLimit', 'value': None}]}, 'isaac:physics:JerkLimit': {'value': [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_robot.usd', 'path': '/r1/joints/right_PGIA_joint2.isaac:physics:JerkLimit', 'value': None}]}, 'physics:lowerLimit': {'value': 0.004000000189989805, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/right_PGIA_joint2.physics:lowerLimit', 'value': 0.004000000189989805}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint2.physics:lowerLimit', 'value': 0.009999999776482582}]}, 'physics:upperLimit': {'value': 0.11999999731779099, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/joints/right_PGIA_joint2.physics:upperLimit', 'value': 0.11999999731779099}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/joints/right_PGIA_joint2.physics:upperLimit', 'value': 0.11999999731779099}]}}`

# 4. USD Link Details

## left_base
- path: `/World/Robot/left_PGIA_base_link`
- valid: `True`
- typeName: `Xform`
- schemas: `['PhysicsRigidBodyAPI', 'PhysicsMassAPI', 'IsaacLinkAPI']`
- world_xform: `{'translation': [-0.8026303052902222, 0.9172664284706116, 1.1713085174560547], 'matrix': [[0.9990972522301119, -7.093689837470857e-05, -0.042481473069993364, 0.0], [-0.042481495407952576, -0.002986148143475198, -0.9990927912196993, 0.0], [-5.598342814239557e-05, 0.9999955389336604, -0.0029864659104206215, 0.0], [-0.8026303052902222, 0.9172664284706116, 1.1713085174560547, 1.0]]}`
- bbox: `{'min': [-0.8355195323981215, 0.9381193179125044, 1.1211832876267245], 'max': [-0.7691542365690222, 1.037554714369442, 1.234999654597088], 'size': [0.06636529582909934, 0.09943539645693755, 0.11381636697036357], 'center': [-0.8023368844835719, 0.9878370161409731, 1.1780914711119062]}`

### attrs
- `physics:rigidBodyEnabled`: `{'value': True, 'stack': []}`
- `physics:kinematicEnabled`: `{'value': False, 'stack': []}`
- `physics:mass`: `{'value': 1.163100004196167, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/left_PGIA_base_link.physics:mass', 'value': 1.163100004196167}]}`
- `physics:centerOfMass`: `{'value': [-2.4360000679735094e-05, -0.0013886999804526567, 0.0655980035662651], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/left_PGIA_base_link.physics:centerOfMass', 'value': [-2.4360000679735094e-05, -0.0013886999804526567, 0.0655980035662651]}]}`
- `physics:diagonalInertia`: `{'value': [0.0016413721023127437, 0.0010600349633023143, 0.001246853033080697], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/left_PGIA_base_link.physics:diagonalInertia', 'value': [0.0016413721023127437, 0.0010600349633023143, 0.001246853033080697]}]}`
- `physxRigidBody:disableGravity`: `{'value': None, 'stack': []}`
- `physxRigidBody:linearDamping`: `{'value': None, 'stack': []}`
- `physxRigidBody:angularDamping`: `{'value': None, 'stack': []}`
- `physxRigidBody:solverPositionIterationCount`: `{'value': None, 'stack': []}`
- `physxRigidBody:solverVelocityIterationCount`: `{'value': None, 'stack': []}`
- `physxRigidBody:maxDepenetrationVelocity`: `{'value': None, 'stack': []}`
- `physics:collisionEnabled`: `{'value': None, 'stack': []}`
- `xformOp:translate`: `{'value': [0.19736969470977783, 0.9172664284706116, 1.0913085174560546], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/left_PGIA_base_link.xformOp:translate', 'value': [0.19736969470977783, 0.9172664284706116, 1.0913085174560546]}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_base.usd', 'path': '/r1/left_PGIA_base_link.xformOp:translate', 'value': [0.1971452236175537, 0.9172012209892273, 1.1134798526763916]}]}`
- `xformOp:orient`: `{'value': '(-0.7058903311025404, 0.7080024481391305, -0.015025524423739443, -0.015020236374160346)', 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/left_PGIA_base_link.xformOp:orient', 'value': '(-0.7058903311025404, 0.7080024481391305, -0.015025524423739443, -0.015020236374160346)'}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_base.usd', 'path': '/r1/left_PGIA_base_link.xformOp:orient', 'value': '(0.7069445848464966, -0.7069446444511414, 0.01514384150505066, 0.015143781900405884)'}]}`
- `xformOp:rotateXYZ`: `{'value': None, 'stack': []}`
- `xformOp:scale`: `{'value': [1.0, 1.0, 1.0], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_base.usd', 'path': '/r1/left_PGIA_base_link.xformOp:scale', 'value': [1.0, 1.0, 1.0]}]}`
- `xformOpOrder`: `{'value': ['xformOp:translate', 'xformOp:orient', 'xformOp:scale'], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_base.usd', 'path': '/r1/left_PGIA_base_link.xformOpOrder', 'value': ['xformOp:translate', 'xformOp:orient', 'xformOp:scale']}]}`

### children
- `/World/Robot/left_PGIA_base_link/visuals` type=`Xform`
  - schemas: `[]`
  - bbox: `{'min': [-0.8355195323981213, 0.9381193179125044, 1.1211832876267243], 'max': [-0.7691542365690219, 1.037554714369442, 1.2349996545970878], 'size': [0.06636529582909934, 0.09943539645693755, 0.11381636697036357], 'center': [-0.8023368844835717, 0.9878370161409731, 1.1780914711119062]}`
  - attrs: `{'physics:collisionEnabled': {'value': None, 'stack': []}, 'physxCollision:contactOffset': {'value': None, 'stack': []}, 'physxCollision:restOffset': {'value': None, 'stack': []}, 'xformOp:translate': {'value': None, 'stack': []}, 'xformOp:orient': {'value': None, 'stack': []}, 'xformOp:rotateXYZ': {'value': None, 'stack': []}, 'xformOp:scale': {'value': None, 'stack': []}, 'xformOpOrder': {'value': None, 'stack': []}}`
- `/World/Robot/left_PGIA_base_link/collisions` type=`Xform`
  - schemas: `[]`
  - bbox: `{'min': [3.4028234663852886e+38, 3.4028234663852886e+38, 3.4028234663852886e+38], 'max': [-3.4028234663852886e+38, -3.4028234663852886e+38, -3.4028234663852886e+38], 'size': [-6.805646932770577e+38, -6.805646932770577e+38, -6.805646932770577e+38], 'center': [0.0, 0.0, 0.0]}`
  - attrs: `{'physics:collisionEnabled': {'value': None, 'stack': []}, 'physxCollision:contactOffset': {'value': None, 'stack': []}, 'physxCollision:restOffset': {'value': None, 'stack': []}, 'xformOp:translate': {'value': None, 'stack': []}, 'xformOp:orient': {'value': None, 'stack': []}, 'xformOp:rotateXYZ': {'value': None, 'stack': []}, 'xformOp:scale': {'value': None, 'stack': []}, 'xformOpOrder': {'value': None, 'stack': []}}`

## left_link1_upper
- path: `/World/Robot/left_PGIA_link1`
- valid: `True`
- typeName: `Xform`
- schemas: `['MaterialBindingAPI', 'PhysicsCollisionAPI', 'PhysicsMeshCollisionAPI', 'PhysicsRigidBodyAPI', 'PhysicsMassAPI', 'IsaacLinkAPI']`
- world_xform: `{'translation': [-0.801792562007904, 1.0583252906799316, 1.190774917602539], 'matrix': [[0.9990972531260119, -7.098256324257826e-05, -0.04248145192355576, 0.0], [-0.0424814743886668, -0.00298631473172728, -0.9990927916155178, 0.0], [-5.5944818438426885e-05, 0.9999955384329468, -0.0029866342893809517, 0.0], [-0.801792562007904, 1.0583252906799316, 1.190774917602539, 1.0]]}`
- bbox: `{'min': [-0.903629165049492, 1.0217833718871483, 1.141009376890154], 'max': [-0.7003514658028992, 1.1784393581918324, 1.2310991153006412], 'size': [0.20327769924659278, 0.15665598630468414, 0.09008973841048729], 'center': [-0.8019903154261956, 1.1001113650394903, 1.1860542460953976]}`

### attrs
- `physics:rigidBodyEnabled`: `{'value': True, 'stack': []}`
- `physics:kinematicEnabled`: `{'value': False, 'stack': []}`
- `physics:mass`: `{'value': 0.33761999011039734, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/left_PGIA_link1.physics:mass', 'value': 0.33761999011039734}]}`
- `physics:centerOfMass`: `{'value': [0.009168500080704689, -0.011239999905228615, 0.0019898000173270702], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/left_PGIA_link1.physics:centerOfMass', 'value': [0.009168500080704689, -0.011239999905228615, 0.0019898000173270702]}]}`
- `physics:diagonalInertia`: `{'value': [0.0006257590721361339, 0.001445254310965538, 0.0010444067884236574], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/left_PGIA_link1.physics:diagonalInertia', 'value': [0.0006257590721361339, 0.001445254310965538, 0.0010444067884236574]}]}`
- `physxRigidBody:disableGravity`: `{'value': None, 'stack': []}`
- `physxRigidBody:linearDamping`: `{'value': None, 'stack': []}`
- `physxRigidBody:angularDamping`: `{'value': None, 'stack': []}`
- `physxRigidBody:solverPositionIterationCount`: `{'value': None, 'stack': []}`
- `physxRigidBody:solverVelocityIterationCount`: `{'value': None, 'stack': []}`
- `physxRigidBody:maxDepenetrationVelocity`: `{'value': None, 'stack': []}`
- `physics:collisionEnabled`: `{'value': True, 'stack': []}`
- `xformOp:translate`: `{'value': [0.19820743799209595, 1.0583252906799316, 1.110774917602539], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/left_PGIA_link1.xformOp:translate', 'value': [0.19820743799209595, 1.0583252906799316, 1.110774917602539]}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_base.usd', 'path': '/r1/left_PGIA_link1.xformOp:translate', 'value': [0.1971452385187149, 1.0582011938095093, 1.1134798526763916]}]}`
- `xformOp:orient`: `{'value': '(-0.705890271944745, 0.7080025074367887, -0.015025531867805041, -0.015020214015905859)', 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/left_PGIA_link1.xformOp:orient', 'value': '(-0.705890271944745, 0.7080025074367887, -0.015025531867805041, -0.015020214015905859)'}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_base.usd', 'path': '/r1/left_PGIA_link1.xformOp:orient', 'value': '(0.7069445848464966, -0.7069446444511414, 0.01514384150505066, 0.015143781900405884)'}]}`
- `xformOp:rotateXYZ`: `{'value': None, 'stack': []}`
- `xformOp:scale`: `{'value': [1.0, 1.0, 1.0], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_base.usd', 'path': '/r1/left_PGIA_link1.xformOp:scale', 'value': [1.0, 1.0, 1.0]}]}`
- `xformOpOrder`: `{'value': ['xformOp:translate', 'xformOp:orient', 'xformOp:scale'], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_base.usd', 'path': '/r1/left_PGIA_link1.xformOpOrder', 'value': ['xformOp:translate', 'xformOp:orient', 'xformOp:scale']}]}`

### children
- `/World/Robot/left_PGIA_link1/visuals` type=`Xform`
  - schemas: `[]`
  - bbox: `{'min': [-0.9036291650494921, 1.0217833718871483, 1.1410093768901546], 'max': [-0.7003514658028993, 1.1784393581918324, 1.2310991153006419], 'size': [0.20327769924659278, 0.15665598630468414, 0.09008973841048729], 'center': [-0.8019903154261957, 1.1001113650394903, 1.1860542460953982]}`
  - attrs: `{'physics:collisionEnabled': {'value': None, 'stack': []}, 'physxCollision:contactOffset': {'value': None, 'stack': []}, 'physxCollision:restOffset': {'value': None, 'stack': []}, 'xformOp:translate': {'value': None, 'stack': []}, 'xformOp:orient': {'value': None, 'stack': []}, 'xformOp:rotateXYZ': {'value': None, 'stack': []}, 'xformOp:scale': {'value': None, 'stack': []}, 'xformOpOrder': {'value': None, 'stack': []}}`
- `/World/Robot/left_PGIA_link1/collisions` type=`Xform`
  - schemas: `[]`
  - bbox: `{'min': [3.4028234663852886e+38, 3.4028234663852886e+38, 3.4028234663852886e+38], 'max': [-3.4028234663852886e+38, -3.4028234663852886e+38, -3.4028234663852886e+38], 'size': [-6.805646932770577e+38, -6.805646932770577e+38, -6.805646932770577e+38], 'center': [0.0, 0.0, 0.0]}`
  - attrs: `{'physics:collisionEnabled': {'value': None, 'stack': []}, 'physxCollision:contactOffset': {'value': None, 'stack': []}, 'physxCollision:restOffset': {'value': None, 'stack': []}, 'xformOp:translate': {'value': None, 'stack': []}, 'xformOp:orient': {'value': None, 'stack': []}, 'xformOp:rotateXYZ': {'value': None, 'stack': []}, 'xformOp:scale': {'value': None, 'stack': []}, 'xformOpOrder': {'value': None, 'stack': []}}`

## left_link2_lower
- path: `/World/Robot/left_PGIA_link2`
- valid: `True`
- typeName: `Xform`
- schemas: `['MaterialBindingAPI', 'PhysicsCollisionAPI', 'PhysicsMeshCollisionAPI', 'PhysicsRigidBodyAPI', 'PhysicsMassAPI', 'IsaacLinkAPI']`
- world_xform: `{'translation': [-0.8037368655204773, 1.058188557624817, 1.1450488567352295], 'matrix': [[0.9990972577157696, -7.093062896850258e-05, -0.042481344066248206, 0.0], [-0.0424813663673579, -0.0029864821304359523, -0.9990927957082101, 0.0], [-5.600349453326742e-05, 0.9999955379367105, -0.002986799336771595, 0.0], [-0.8037368655204773, 1.058188557624817, 1.1450488567352295, 1.0]]}`
- bbox: `{'min': [-0.832348451277161, 1.021679403557829, 1.1076440755994046], 'max': [-0.7764375989743085, 1.1783249307228543, 1.1914678202172875], 'size': [0.05591085230285242, 0.15664552716502533, 0.08382374461788289], 'center': [-0.8043930251257347, 1.1000021671403415, 1.149555947908346]}`

### attrs
- `physics:rigidBodyEnabled`: `{'value': True, 'stack': []}`
- `physics:kinematicEnabled`: `{'value': False, 'stack': []}`
- `physics:mass`: `{'value': 0.26010000705718994, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/left_PGIA_link2.physics:mass', 'value': 0.26010000705718994}]}`
- `physics:centerOfMass`: `{'value': [-0.009510800242424011, 0.006729300133883953, 0.007861699908971786], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/left_PGIA_link2.physics:centerOfMass', 'value': [-0.009510800242424011, 0.006729300133883953, 0.007861699908971786]}]}`
- `physics:diagonalInertia`: `{'value': [0.0007054557208903134, 0.000655503710731864, 0.00011099052790086716], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/left_PGIA_link2.physics:diagonalInertia', 'value': [0.0007054557208903134, 0.000655503710731864, 0.00011099052790086716]}]}`
- `physxRigidBody:disableGravity`: `{'value': None, 'stack': []}`
- `physxRigidBody:linearDamping`: `{'value': None, 'stack': []}`
- `physxRigidBody:angularDamping`: `{'value': None, 'stack': []}`
- `physxRigidBody:solverPositionIterationCount`: `{'value': None, 'stack': []}`
- `physxRigidBody:solverVelocityIterationCount`: `{'value': None, 'stack': []}`
- `physxRigidBody:maxDepenetrationVelocity`: `{'value': None, 'stack': []}`
- `physics:collisionEnabled`: `{'value': True, 'stack': []}`
- `xformOp:translate`: `{'value': [0.1962631344795227, 1.058188557624817, 1.0650488567352294], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/left_PGIA_link2.xformOp:translate', 'value': [0.1962631344795227, 1.058188557624817, 1.0650488567352294]}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_base.usd', 'path': '/r1/left_PGIA_link2.xformOp:translate', 'value': [0.1971452385187149, 1.0582011938095093, 1.1134798526763916]}]}`
- `xformOp:orient`: `{'value': '(-0.7058902138875001, 0.7080025669414938, -0.015025474123684478, -0.015020195387334158)', 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/left_PGIA_link2.xformOp:orient', 'value': '(-0.7058902138875001, 0.7080025669414938, -0.015025474123684478, -0.015020195387334158)'}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_base.usd', 'path': '/r1/left_PGIA_link2.xformOp:orient', 'value': '(0.7069445848464966, -0.7069446444511414, 0.01514384150505066, 0.015143781900405884)'}]}`
- `xformOp:rotateXYZ`: `{'value': None, 'stack': []}`
- `xformOp:scale`: `{'value': [1.0, 1.0, 1.0], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_base.usd', 'path': '/r1/left_PGIA_link2.xformOp:scale', 'value': [1.0, 1.0, 1.0]}]}`
- `xformOpOrder`: `{'value': ['xformOp:translate', 'xformOp:orient', 'xformOp:scale'], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_base.usd', 'path': '/r1/left_PGIA_link2.xformOpOrder', 'value': ['xformOp:translate', 'xformOp:orient', 'xformOp:scale']}]}`

### children
- `/World/Robot/left_PGIA_link2/visuals` type=`Xform`
  - schemas: `[]`
  - bbox: `{'min': [-0.8323484512771613, 1.0216794035578292, 1.1076440755994048], 'max': [-0.7764375989743089, 1.1783249307228545, 1.1914678202172877], 'size': [0.05591085230285242, 0.15664552716502533, 0.08382374461788289], 'center': [-0.8043930251257351, 1.100002167140342, 1.1495559479083464]}`
  - attrs: `{'physics:collisionEnabled': {'value': None, 'stack': []}, 'physxCollision:contactOffset': {'value': None, 'stack': []}, 'physxCollision:restOffset': {'value': None, 'stack': []}, 'xformOp:translate': {'value': None, 'stack': []}, 'xformOp:orient': {'value': None, 'stack': []}, 'xformOp:rotateXYZ': {'value': None, 'stack': []}, 'xformOp:scale': {'value': None, 'stack': []}, 'xformOpOrder': {'value': None, 'stack': []}}`
- `/World/Robot/left_PGIA_link2/collisions` type=`Xform`
  - schemas: `[]`
  - bbox: `{'min': [3.4028234663852886e+38, 3.4028234663852886e+38, 3.4028234663852886e+38], 'max': [-3.4028234663852886e+38, -3.4028234663852886e+38, -3.4028234663852886e+38], 'size': [-6.805646932770577e+38, -6.805646932770577e+38, -6.805646932770577e+38], 'center': [0.0, 0.0, 0.0]}`
  - attrs: `{'physics:collisionEnabled': {'value': None, 'stack': []}, 'physxCollision:contactOffset': {'value': None, 'stack': []}, 'physxCollision:restOffset': {'value': None, 'stack': []}, 'xformOp:translate': {'value': None, 'stack': []}, 'xformOp:orient': {'value': None, 'stack': []}, 'xformOp:rotateXYZ': {'value': None, 'stack': []}, 'xformOp:scale': {'value': None, 'stack': []}, 'xformOpOrder': {'value': None, 'stack': []}}`

## right_base
- path: `/World/Robot/right_PGIA_base_link`
- valid: `True`
- typeName: `Xform`
- schemas: `['PhysicsRigidBodyAPI', 'PhysicsMassAPI', 'IsaacLinkAPI']`
- world_xform: `{'translation': [-0.8027700781822205, -0.9181316494941711, 1.1711441278457642], 'matrix': [[-0.9990038829875529, 7.088039420615616e-05, 0.044623275896792734, 0.0], [-0.04462272959943171, 0.0036100761924177105, -0.9989973870601369, 0.0], [-0.00023190275454844367, -0.9999934811416796, -0.003603317257441674, 0.0], [-0.8027700781822205, -0.9181316494941711, 1.1711441278457642, 1.0]]}`
- bbox: `{'min': [-0.8357814026211999, -1.0384586820120918, 1.1208833666534799], 'max': [-0.7691667570710534, -0.9389542369543202, 1.2348824229673743], 'size': [0.06661464555014651, 0.09950444505777156, 0.1139990563138944], 'center': [-0.8024740798461267, -0.988706459483206, 1.177882894810427]}`

### attrs
- `physics:rigidBodyEnabled`: `{'value': True, 'stack': []}`
- `physics:kinematicEnabled`: `{'value': False, 'stack': []}`
- `physics:mass`: `{'value': 1.163100004196167, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/right_PGIA_base_link.physics:mass', 'value': 1.163100004196167}]}`
- `physics:centerOfMass`: `{'value': [-2.4360000679735094e-05, -0.0013886999804526567, 0.0655980035662651], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/right_PGIA_base_link.physics:centerOfMass', 'value': [-2.4360000679735094e-05, -0.0013886999804526567, 0.0655980035662651]}]}`
- `physics:diagonalInertia`: `{'value': [0.0016413721023127437, 0.0010600349633023143, 0.001246853033080697], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/right_PGIA_base_link.physics:diagonalInertia', 'value': [0.0016413721023127437, 0.0010600349633023143, 0.001246853033080697]}]}`
- `physxRigidBody:disableGravity`: `{'value': None, 'stack': []}`
- `physxRigidBody:linearDamping`: `{'value': None, 'stack': []}`
- `physxRigidBody:angularDamping`: `{'value': None, 'stack': []}`
- `physxRigidBody:solverPositionIterationCount`: `{'value': None, 'stack': []}`
- `physxRigidBody:solverVelocityIterationCount`: `{'value': None, 'stack': []}`
- `physxRigidBody:maxDepenetrationVelocity`: `{'value': None, 'stack': []}`
- `physics:collisionEnabled`: `{'value': None, 'stack': []}`
- `xformOp:translate`: `{'value': [0.19722992181777954, -0.9181316494941711, 1.091144127845764], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/right_PGIA_base_link.xformOp:translate', 'value': [0.19722992181777954, -0.9181316494941711, 1.091144127845764]}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_base.usd', 'path': '/r1/right_PGIA_base_link.xformOp:translate', 'value': [0.19714513421058655, -0.9181990623474121, 1.1134800910949707]}]}`
- `xformOp:orient`: `{'value': '(-0.0158341083378816, -0.015727031486194136, 0.7082049979415235, -0.7056540387359966)', 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/right_PGIA_base_link.xformOp:orient', 'value': '(-0.0158341083378816, -0.015727031486194136, 0.7082049979415235, -0.7056540387359966)'}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_base.usd', 'path': '/r1/right_PGIA_base_link.xformOp:orient', 'value': '(0.015706762671470642, 0.015706762671470642, -0.7069323062896729, 0.7069324254989624)'}]}`
- `xformOp:rotateXYZ`: `{'value': None, 'stack': []}`
- `xformOp:scale`: `{'value': [1.0, 1.0, 1.0], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_base.usd', 'path': '/r1/right_PGIA_base_link.xformOp:scale', 'value': [1.0, 1.0, 1.0]}]}`
- `xformOpOrder`: `{'value': ['xformOp:translate', 'xformOp:orient', 'xformOp:scale'], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_base.usd', 'path': '/r1/right_PGIA_base_link.xformOpOrder', 'value': ['xformOp:translate', 'xformOp:orient', 'xformOp:scale']}]}`

### children
- `/World/Robot/right_PGIA_base_link/visuals` type=`Xform`
  - schemas: `[]`
  - bbox: `{'min': [-0.8357814026212, -1.0384586820120918, 1.1208833666534797], 'max': [-0.7691667570710535, -0.9389542369543202, 1.234882422967374], 'size': [0.06661464555014651, 0.09950444505777156, 0.1139990563138944], 'center': [-0.8024740798461267, -0.988706459483206, 1.177882894810427]}`
  - attrs: `{'physics:collisionEnabled': {'value': None, 'stack': []}, 'physxCollision:contactOffset': {'value': None, 'stack': []}, 'physxCollision:restOffset': {'value': None, 'stack': []}, 'xformOp:translate': {'value': None, 'stack': []}, 'xformOp:orient': {'value': None, 'stack': []}, 'xformOp:rotateXYZ': {'value': None, 'stack': []}, 'xformOp:scale': {'value': None, 'stack': []}, 'xformOpOrder': {'value': None, 'stack': []}}`
- `/World/Robot/right_PGIA_base_link/collisions` type=`Xform`
  - schemas: `[]`
  - bbox: `{'min': [3.4028234663852886e+38, 3.4028234663852886e+38, 3.4028234663852886e+38], 'max': [-3.4028234663852886e+38, -3.4028234663852886e+38, -3.4028234663852886e+38], 'size': [-6.805646932770577e+38, -6.805646932770577e+38, -6.805646932770577e+38], 'center': [0.0, 0.0, 0.0]}`
  - attrs: `{'physics:collisionEnabled': {'value': None, 'stack': []}, 'physxCollision:contactOffset': {'value': None, 'stack': []}, 'physxCollision:restOffset': {'value': None, 'stack': []}, 'xformOp:translate': {'value': None, 'stack': []}, 'xformOp:orient': {'value': None, 'stack': []}, 'xformOp:rotateXYZ': {'value': None, 'stack': []}, 'xformOp:scale': {'value': None, 'stack': []}, 'xformOpOrder': {'value': None, 'stack': []}}`

## right_link1_upper
- path: `/World/Robot/right_PGIA_link1`
- valid: `True`
- typeName: `Xform`
- schemas: `['MaterialBindingAPI', 'PhysicsCollisionAPI', 'PhysicsMeshCollisionAPI', 'PhysicsRigidBodyAPI', 'PhysicsMassAPI', 'IsaacLinkAPI']`
- world_xform: `{'translation': [-0.8019266128540039, -1.059201717376709, 1.1902503967285156], 'matrix': [[-0.9990038820596561, 7.090932891979859e-05, 0.04462329662412178, 0.0], [-0.044622750183033424, 0.0036102457713007574, -0.998997385527897, 0.0], [-0.00023193930213915442, -0.9999934805274172, -0.0036034853714490556, 0.0], [-0.8019266128540039, -1.059201717376709, 1.1902503967285156, 1.0]]}`
- bbox: `{'min': [-0.9038717838033729, -1.179337992132977, 1.1402009617703435], 'max': [-0.7004113616310252, -1.0226316791289054, 1.2308077976611704], 'size': [0.2034604221723476, 0.15670631300407156, 0.09060683589082696], 'center': [-0.802141572717199, -1.1009848356309413, 1.185504379715757]}`

### attrs
- `physics:rigidBodyEnabled`: `{'value': True, 'stack': []}`
- `physics:kinematicEnabled`: `{'value': False, 'stack': []}`
- `physics:mass`: `{'value': 0.33761999011039734, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/right_PGIA_link1.physics:mass', 'value': 0.33761999011039734}]}`
- `physics:centerOfMass`: `{'value': [0.009168500080704689, -0.011239999905228615, 0.0019898000173270702], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/right_PGIA_link1.physics:centerOfMass', 'value': [0.009168500080704689, -0.011239999905228615, 0.0019898000173270702]}]}`
- `physics:diagonalInertia`: `{'value': [0.0006257590721361339, 0.001445254310965538, 0.0010444067884236574], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/right_PGIA_link1.physics:diagonalInertia', 'value': [0.0006257590721361339, 0.001445254310965538, 0.0010444067884236574]}]}`
- `physxRigidBody:disableGravity`: `{'value': None, 'stack': []}`
- `physxRigidBody:linearDamping`: `{'value': None, 'stack': []}`
- `physxRigidBody:angularDamping`: `{'value': None, 'stack': []}`
- `physxRigidBody:solverPositionIterationCount`: `{'value': None, 'stack': []}`
- `physxRigidBody:solverVelocityIterationCount`: `{'value': None, 'stack': []}`
- `physxRigidBody:maxDepenetrationVelocity`: `{'value': None, 'stack': []}`
- `physics:collisionEnabled`: `{'value': True, 'stack': []}`
- `xformOp:translate`: `{'value': [0.1980733871459961, -1.059201717376709, 1.1102503967285156], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/right_PGIA_link1.xformOp:translate', 'value': [0.1980733871459961, -1.059201717376709, 1.1102503967285156]}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_base.usd', 'path': '/r1/right_PGIA_link1.xformOp:translate', 'value': [0.19714513421058655, -1.0591990947723389, 1.1134800910949707]}]}`
- `xformOp:orient`: `{'value': '(-0.015834127227258687, -0.015727027218234074, 0.7082050573814065, -0.7056539787524949)', 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/right_PGIA_link1.xformOp:orient', 'value': '(-0.015834127227258687, -0.015727027218234074, 0.7082050573814065, -0.7056539787524949)'}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_base.usd', 'path': '/r1/right_PGIA_link1.xformOp:orient', 'value': '(0.015706762671470642, 0.015706762671470642, -0.7069323062896729, 0.7069324254989624)'}]}`
- `xformOp:rotateXYZ`: `{'value': None, 'stack': []}`
- `xformOp:scale`: `{'value': [1.0, 1.0, 1.0], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_base.usd', 'path': '/r1/right_PGIA_link1.xformOp:scale', 'value': [1.0, 1.0, 1.0]}]}`
- `xformOpOrder`: `{'value': ['xformOp:translate', 'xformOp:orient', 'xformOp:scale'], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_base.usd', 'path': '/r1/right_PGIA_link1.xformOpOrder', 'value': ['xformOp:translate', 'xformOp:orient', 'xformOp:scale']}]}`

### children
- `/World/Robot/right_PGIA_link1/visuals` type=`Xform`
  - schemas: `[]`
  - bbox: `{'min': [-0.9038717838033729, -1.179337992132977, 1.1402009617703437], 'max': [-0.7004113616310252, -1.0226316791289054, 1.2308077976611707], 'size': [0.2034604221723476, 0.15670631300407156, 0.09060683589082696], 'center': [-0.802141572717199, -1.1009848356309413, 1.1855043797157572]}`
  - attrs: `{'physics:collisionEnabled': {'value': None, 'stack': []}, 'physxCollision:contactOffset': {'value': None, 'stack': []}, 'physxCollision:restOffset': {'value': None, 'stack': []}, 'xformOp:translate': {'value': None, 'stack': []}, 'xformOp:orient': {'value': None, 'stack': []}, 'xformOp:rotateXYZ': {'value': None, 'stack': []}, 'xformOp:scale': {'value': None, 'stack': []}, 'xformOpOrder': {'value': None, 'stack': []}}`
- `/World/Robot/right_PGIA_link1/collisions` type=`Xform`
  - schemas: `[]`
  - bbox: `{'min': [3.4028234663852886e+38, 3.4028234663852886e+38, 3.4028234663852886e+38], 'max': [-3.4028234663852886e+38, -3.4028234663852886e+38, -3.4028234663852886e+38], 'size': [-6.805646932770577e+38, -6.805646932770577e+38, -6.805646932770577e+38], 'center': [0.0, 0.0, 0.0]}`
  - attrs: `{'physics:collisionEnabled': {'value': None, 'stack': []}, 'physxCollision:contactOffset': {'value': None, 'stack': []}, 'physxCollision:restOffset': {'value': None, 'stack': []}, 'xformOp:translate': {'value': None, 'stack': []}, 'xformOp:orient': {'value': None, 'stack': []}, 'xformOp:rotateXYZ': {'value': None, 'stack': []}, 'xformOp:scale': {'value': None, 'stack': []}, 'xformOpOrder': {'value': None, 'stack': []}}`

## right_link2_lower
- path: `/World/Robot/right_PGIA_link2`
- valid: `True`
- typeName: `Xform`
- schemas: `['MaterialBindingAPI', 'PhysicsCollisionAPI', 'PhysicsMeshCollisionAPI', 'PhysicsRigidBodyAPI', 'PhysicsMassAPI', 'IsaacLinkAPI']`
- world_xform: `{'translation': [-0.8041641712188721, -1.0590206384658813, 1.140156865119934], 'matrix': [[-0.9990038774848071, 7.086958950477604e-05, 0.04462339910654767, 0.0], [-0.04462285280819818, 0.0036102448878601034, -0.9989973809470711, 0.0], [-0.00023189993280742227, -0.9999934805334239, -0.003603486238380915, 0.0], [-0.8041641712188721, -1.0590206384658813, 1.140156865119934, 1.0]]}`
- bbox: `{'min': [-0.8311731200235856, -1.179184957101223, 1.1025512484756068], 'max': [-0.7750657712733852, -1.0224891053804777, 1.1865761531353143], 'size': [0.0561073487502004, 0.1566958517207453, 0.08402490465970747], 'center': [-0.8031194456484854, -1.1008370312408502, 1.1445637008054605]}`

### attrs
- `physics:rigidBodyEnabled`: `{'value': True, 'stack': []}`
- `physics:kinematicEnabled`: `{'value': False, 'stack': []}`
- `physics:mass`: `{'value': 0.26010000705718994, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/right_PGIA_link2.physics:mass', 'value': 0.26010000705718994}]}`
- `physics:centerOfMass`: `{'value': [-0.009510800242424011, 0.006729300133883953, 0.007861699908971786], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/right_PGIA_link2.physics:centerOfMass', 'value': [-0.009510800242424011, 0.006729300133883953, 0.007861699908971786]}]}`
- `physics:diagonalInertia`: `{'value': [0.0007054557208903134, 0.000655503710731864, 0.00011099052790086716], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_physics.usd', 'path': '/r1/right_PGIA_link2.physics:diagonalInertia', 'value': [0.0007054557208903134, 0.000655503710731864, 0.00011099052790086716]}]}`
- `physxRigidBody:disableGravity`: `{'value': None, 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/right_PGIA_link2.physxRigidBody:disableGravity', 'value': None}]}`
- `physxRigidBody:linearDamping`: `{'value': None, 'stack': []}`
- `physxRigidBody:angularDamping`: `{'value': None, 'stack': []}`
- `physxRigidBody:solverPositionIterationCount`: `{'value': None, 'stack': []}`
- `physxRigidBody:solverVelocityIterationCount`: `{'value': None, 'stack': []}`
- `physxRigidBody:maxDepenetrationVelocity`: `{'value': None, 'stack': []}`
- `physics:collisionEnabled`: `{'value': True, 'stack': []}`
- `xformOp:translate`: `{'value': [0.19583582878112793, -1.0590206384658813, 1.060156865119934], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/right_PGIA_link2.xformOp:translate', 'value': [0.19583582878112793, -1.0590206384658813, 1.060156865119934]}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_base.usd', 'path': '/r1/right_PGIA_link2.xformOp:translate', 'value': [0.19714513421058655, -1.0591990947723389, 1.1134800910949707]}]}`
- `xformOp:orient`: `{'value': '(-0.015834149524621796, -0.01572707749165359, 0.7082050565710202, -0.7056539779450276)', 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd', 'path': '/World/Robot/right_PGIA_link2.xformOp:orient', 'value': '(-0.015834149524621796, -0.01572707749165359, 0.7082050565710202, -0.7056539779450276)'}, {'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_base.usd', 'path': '/r1/right_PGIA_link2.xformOp:orient', 'value': '(0.015706762671470642, 0.015706762671470642, -0.7069323062896729, 0.7069324254989624)'}]}`
- `xformOp:rotateXYZ`: `{'value': None, 'stack': []}`
- `xformOp:scale`: `{'value': [1.0, 1.0, 1.0], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_base.usd', 'path': '/r1/right_PGIA_link2.xformOp:scale', 'value': [1.0, 1.0, 1.0]}]}`
- `xformOpOrder`: `{'value': ['xformOp:translate', 'xformOp:orient', 'xformOp:scale'], 'stack': [{'layer': '/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/configuration/513_base.usd', 'path': '/r1/right_PGIA_link2.xformOpOrder', 'value': ['xformOp:translate', 'xformOp:orient', 'xformOp:scale']}]}`

### children
- `/World/Robot/right_PGIA_link2/visuals` type=`Xform`
  - schemas: `[]`
  - bbox: `{'min': [-0.8311731200235853, -1.1791849571012232, 1.1025512484756068], 'max': [-0.7750657712733849, -1.0224891053804779, 1.1865761531353143], 'size': [0.0561073487502004, 0.1566958517207453, 0.08402490465970747], 'center': [-0.8031194456484851, -1.1008370312408506, 1.1445637008054605]}`
  - attrs: `{'physics:collisionEnabled': {'value': None, 'stack': []}, 'physxCollision:contactOffset': {'value': None, 'stack': []}, 'physxCollision:restOffset': {'value': None, 'stack': []}, 'xformOp:translate': {'value': None, 'stack': []}, 'xformOp:orient': {'value': None, 'stack': []}, 'xformOp:rotateXYZ': {'value': None, 'stack': []}, 'xformOp:scale': {'value': None, 'stack': []}, 'xformOpOrder': {'value': None, 'stack': []}}`
- `/World/Robot/right_PGIA_link2/collisions` type=`Xform`
  - schemas: `[]`
  - bbox: `{'min': [3.4028234663852886e+38, 3.4028234663852886e+38, 3.4028234663852886e+38], 'max': [-3.4028234663852886e+38, -3.4028234663852886e+38, -3.4028234663852886e+38], 'size': [-6.805646932770577e+38, -6.805646932770577e+38, -6.805646932770577e+38], 'center': [0.0, 0.0, 0.0]}`
  - attrs: `{'physics:collisionEnabled': {'value': None, 'stack': []}, 'physxCollision:contactOffset': {'value': None, 'stack': []}, 'physxCollision:restOffset': {'value': None, 'stack': []}, 'xformOp:translate': {'value': None, 'stack': []}, 'xformOp:orient': {'value': None, 'stack': []}, 'xformOp:rotateXYZ': {'value': None, 'stack': []}, 'xformOp:scale': {'value': None, 'stack': []}, 'xformOpOrder': {'value': None, 'stack': []}}`

# 5. Action Graph / Control Chain

- graph path: `/World/ActionGraphs/Gripper_Control_Graph`
- graph exists: `True`

## Connection Summary

| label | src | dst | connected | dst upstream |
|---|---|---|---|---|
| left jointNames | `/World/ActionGraphs/Gripper_Control_Graph/script_left_gripper.outputs:joint_names` | `/World/ActionGraphs/Gripper_Control_Graph/artic_left_gripper.inputs:jointNames` | `True` | `['Attribute("/World/ActionGraphs/Gripper_Control_Graph/script_left_gripper.outputs:joint_names")']` |
| left positionCommand | `/World/ActionGraphs/Gripper_Control_Graph/script_left_gripper.outputs:position_cmds` | `/World/ActionGraphs/Gripper_Control_Graph/artic_left_gripper.inputs:positionCommand` | `False` | `[]` |
| left effortCommand | `/World/ActionGraphs/Gripper_Control_Graph/script_left_gripper.outputs:effort_cmds` | `/World/ActionGraphs/Gripper_Control_Graph/artic_left_gripper.inputs:effortCommand` | `True` | `['Attribute("/World/ActionGraphs/Gripper_Control_Graph/script_left_gripper.outputs:effort_cmds")']` |
| left exec | `/World/ActionGraphs/Gripper_Control_Graph/script_left_gripper.outputs:execOut` | `/World/ActionGraphs/Gripper_Control_Graph/artic_left_gripper.inputs:execIn` | `True` | `['Attribute("/World/ActionGraphs/Gripper_Control_Graph/script_left_gripper.outputs:execOut")']` |
| left sub data | `/World/ActionGraphs/Gripper_Control_Graph/sub_left_gripper.outputs:data` | `/World/ActionGraphs/Gripper_Control_Graph/script_left_gripper.inputs:input_double_array` | `True` | `['Attribute("/World/ActionGraphs/Gripper_Control_Graph/sub_left_gripper.outputs:data")']` |
| right jointNames | `/World/ActionGraphs/Gripper_Control_Graph/script_right_gripper.outputs:joint_names` | `/World/ActionGraphs/Gripper_Control_Graph/artic_right_gripper.inputs:jointNames` | `True` | `['Attribute("/World/ActionGraphs/Gripper_Control_Graph/script_right_gripper.outputs:joint_names")']` |
| right positionCommand | `/World/ActionGraphs/Gripper_Control_Graph/script_right_gripper.outputs:position_cmds` | `/World/ActionGraphs/Gripper_Control_Graph/artic_right_gripper.inputs:positionCommand` | `False` | `[]` |
| right effortCommand | `/World/ActionGraphs/Gripper_Control_Graph/script_right_gripper.outputs:effort_cmds` | `/World/ActionGraphs/Gripper_Control_Graph/artic_right_gripper.inputs:effortCommand` | `True` | `['Attribute("/World/ActionGraphs/Gripper_Control_Graph/script_right_gripper.outputs:effort_cmds")']` |
| right exec | `/World/ActionGraphs/Gripper_Control_Graph/script_right_gripper.outputs:execOut` | `/World/ActionGraphs/Gripper_Control_Graph/artic_right_gripper.inputs:execIn` | `True` | `['Attribute("/World/ActionGraphs/Gripper_Control_Graph/script_right_gripper.outputs:execOut")']` |
| right sub data | `/World/ActionGraphs/Gripper_Control_Graph/sub_right_gripper.outputs:data` | `/World/ActionGraphs/Gripper_Control_Graph/script_right_gripper.inputs:input_double_array` | `True` | `['Attribute("/World/ActionGraphs/Gripper_Control_Graph/sub_right_gripper.outputs:data")']` |

## Graph Nodes

### sub_left
- path: `/World/ActionGraphs/Gripper_Control_Graph/sub_left_gripper`
- valid: `True`
- type_name: `isaacsim.ros2.bridge.ROS2Subscriber`

- `inputs:execIn`
  - value: `1`
  - upstream: `['Attribute("/World/ActionGraphs/Gripper_Control_Graph/on_playback_tick.outputs:tick")']`
  - downstream: `[]`
- `inputs:messageName`
  - value: `Float64MultiArray`
  - upstream: `[]`
  - downstream: `[]`
- `inputs:topicName`
  - value: `left_gripper_controller/commands`
  - upstream: `[]`
  - downstream: `[]`
- `outputs:data`
  - value: `[0.0]`
  - upstream: `[]`
  - downstream: `['Attribute("/World/ActionGraphs/Gripper_Control_Graph/script_left_gripper.inputs:input_double_array")']`
- `outputs:execOut`
  - value: `0`
  - upstream: `[]`
  - downstream: `[]`
- `outputs:layout:data_offset`
  - value: `0`
  - upstream: `[]`
  - downstream: `[]`

### sub_right
- path: `/World/ActionGraphs/Gripper_Control_Graph/sub_right_gripper`
- valid: `True`
- type_name: `isaacsim.ros2.bridge.ROS2Subscriber`

- `inputs:execIn`
  - value: `1`
  - upstream: `['Attribute("/World/ActionGraphs/Gripper_Control_Graph/on_playback_tick.outputs:tick")']`
  - downstream: `[]`
- `inputs:messageName`
  - value: `Float64MultiArray`
  - upstream: `[]`
  - downstream: `[]`
- `inputs:topicName`
  - value: `right_gripper_controller/commands`
  - upstream: `[]`
  - downstream: `[]`
- `outputs:data`
  - value: `[1.0]`
  - upstream: `[]`
  - downstream: `['Attribute("/World/ActionGraphs/Gripper_Control_Graph/script_right_gripper.inputs:input_double_array")']`
- `outputs:execOut`
  - value: `0`
  - upstream: `[]`
  - downstream: `[]`
- `outputs:layout:data_offset`
  - value: `0`
  - upstream: `[]`
  - downstream: `[]`

### script_left
- path: `/World/ActionGraphs/Gripper_Control_Graph/script_left_gripper`
- valid: `True`
- type_name: `omni.graph.scriptnode.ScriptNode`

- `inputs:execIn`
  - value: `1`
  - upstream: `['Attribute("/World/ActionGraphs/Gripper_Control_Graph/on_playback_tick.outputs:tick")']`
  - downstream: `[]`
- `inputs:input_double_array`
  - value: `[0.0]`
  - upstream: `['Attribute("/World/ActionGraphs/Gripper_Control_Graph/sub_left_gripper.outputs:data")']`
  - downstream: `[]`
- `inputs:script`
  - value: `"""
PGIA 夹爪 Effort-PD 混合控制 v4.1 — 统一版 (左右共用)
================================================================
基于 v3.8 架构 (effort + builtins 隔离), 加入软件 PD 位控约束.

控制策略:
  每帧计算: effort = Kp * (target - current) - Kd * vel + gravity_comp
  不同状态设置不同的 target_pos, PD 自动跟踪

状态机:
  INIT  → target=托盘夹持位置, PD 闭合到接触 (初始含托盘)
  IDLE  → target=MAX_TRAVEL, PD 保持在张开位置 (无托盘)
  OPEN  → target=MAX_TRAVEL, PD 张开
  CLOSE → target 逐帧递减, PD 闭合, 接触检测
  HOLD  → target=锁定位置, PD 保持不漂移

5 项需求:
  1. 闭合最小行程 (含 gap) = 0.04m
     → MIN_GAP = 0.04, 即 j1=j2 ≈ (0.04-0.0197)/2 ≈ 0.0102
  2. MAX_TRAVEL = 0.24m (注: 超过 USD_UPPER=0.12, 会被钳位)
  3. HOLD 锁定接触位置不漂移 → PD 跟踪锁定位置
  4. 有托盘: 力控(PD 闭合+接触检测); 无托盘: 位控(PD 到 MAX)
  5. 初始含托盘 → INIT 直接进 CLOSE 逻辑, PD 闭合到接触

USD Drive 要求:
  stiffness = 0 (或很小, ≤50)
  damping = 0 (或很小, ≤20)
  maxForce = 500
  → 主要控制力来自 ScriptNode 的 PD effort

Graph 连线: effort_cmds → effortCommand (不是 positionCommand)
================================================================
"""

import os
import math
import builtins
from datetime import datetime

import numpy as np
import omni.graph.core as og
import carb

try:
    from omni.isaac.dynamic_control import _dynamic_control
except Exception:
    _dynamic_
...<TRUNCATED>`
  - upstream: `[]`
  - downstream: `[]`
- `inputs:scriptPath`
  - value: ``
  - upstream: `[]`
  - downstream: `[]`
- `outputs:effort_cmds`
  - value: `[1.2902519411159044, -10.429496902099963]`
  - upstream: `[]`
  - downstream: `['Attribute("/World/ActionGraphs/Gripper_Control_Graph/artic_left_gripper.inputs:effortCommand")']`
- `outputs:execOut`
  - value: `1`
  - upstream: `[]`
  - downstream: `['Attribute("/World/ActionGraphs/Gripper_Control_Graph/artic_left_gripper.inputs:execIn")']`
- `outputs:joint_names`
  - value: `['left_PGIA_joint1', 'left_PGIA_joint2']`
  - upstream: `[]`
  - downstream: `['Attribute("/World/ActionGraphs/Gripper_Control_Graph/artic_left_gripper.inputs:jointNames")']`

### script_right
- path: `/World/ActionGraphs/Gripper_Control_Graph/script_right_gripper`
- valid: `True`
- type_name: `omni.graph.scriptnode.ScriptNode`

- `inputs:execIn`
  - value: `1`
  - upstream: `['Attribute("/World/ActionGraphs/Gripper_Control_Graph/on_playback_tick.outputs:tick")']`
  - downstream: `[]`
- `inputs:input_double_array`
  - value: `[1.0]`
  - upstream: `['Attribute("/World/ActionGraphs/Gripper_Control_Graph/sub_right_gripper.outputs:data")']`
  - downstream: `[]`
- `inputs:script`
  - value: `"""
PGIA 夹爪 Effort-PD 混合控制 v4.1 — 统一版 (左右共用)
================================================================
基于 v3.8 架构 (effort + builtins 隔离), 加入软件 PD 位控约束.

控制策略:
  每帧计算: effort = Kp * (target - current) - Kd * vel + gravity_comp
  不同状态设置不同的 target_pos, PD 自动跟踪

状态机:
  INIT  → target=托盘夹持位置, PD 闭合到接触 (初始含托盘)
  IDLE  → target=MAX_TRAVEL, PD 保持在张开位置 (无托盘)
  OPEN  → target=MAX_TRAVEL, PD 张开
  CLOSE → target 逐帧递减, PD 闭合, 接触检测
  HOLD  → target=锁定位置, PD 保持不漂移

5 项需求:
  1. 闭合最小行程 (含 gap) = 0.04m
     → MIN_GAP = 0.04, 即 j1=j2 ≈ (0.04-0.0197)/2 ≈ 0.0102
  2. MAX_TRAVEL = 0.24m (注: 超过 USD_UPPER=0.12, 会被钳位)
  3. HOLD 锁定接触位置不漂移 → PD 跟踪锁定位置
  4. 有托盘: 力控(PD 闭合+接触检测); 无托盘: 位控(PD 到 MAX)
  5. 初始含托盘 → INIT 直接进 CLOSE 逻辑, PD 闭合到接触

USD Drive 要求:
  stiffness = 0 (或很小, ≤50)
  damping = 0 (或很小, ≤20)
  maxForce = 500
  → 主要控制力来自 ScriptNode 的 PD effort

Graph 连线: effort_cmds → effortCommand (不是 positionCommand)
================================================================
"""

import os
import math
import builtins
from datetime import datetime

import numpy as np
import omni.graph.core as og
import carb

try:
    from omni.isaac.dynamic_control import _dynamic_control
except Exception:
    _dynamic_
...<TRUNCATED>`
  - upstream: `[]`
  - downstream: `[]`
- `inputs:scriptPath`
  - value: ``
  - upstream: `[]`
  - downstream: `[]`
- `outputs:effort_cmds`
  - value: `[60.0, 48.56764623153845]`
  - upstream: `[]`
  - downstream: `['Attribute("/World/ActionGraphs/Gripper_Control_Graph/artic_right_gripper.inputs:effortCommand")']`
- `outputs:execOut`
  - value: `1`
  - upstream: `[]`
  - downstream: `['Attribute("/World/ActionGraphs/Gripper_Control_Graph/artic_right_gripper.inputs:execIn")']`
- `outputs:joint_names`
  - value: `['right_PGIA_joint1', 'right_PGIA_joint2']`
  - upstream: `[]`
  - downstream: `['Attribute("/World/ActionGraphs/Gripper_Control_Graph/artic_right_gripper.inputs:jointNames")']`

### artic_left
- path: `/World/ActionGraphs/Gripper_Control_Graph/artic_left_gripper`
- valid: `True`
- type_name: `isaacsim.core.nodes.IsaacArticulationController`

- `inputs:effortCommand`
  - value: `[1.2902519411159044, -10.429496902099963]`
  - upstream: `['Attribute("/World/ActionGraphs/Gripper_Control_Graph/script_left_gripper.outputs:effort_cmds")']`
  - downstream: `[]`
- `inputs:execIn`
  - value: `1`
  - upstream: `['Attribute("/World/ActionGraphs/Gripper_Control_Graph/script_left_gripper.outputs:execOut")']`
  - downstream: `[]`
- `inputs:jointNames`
  - value: `['left_PGIA_joint1', 'left_PGIA_joint2']`
  - upstream: `['Attribute("/World/ActionGraphs/Gripper_Control_Graph/script_left_gripper.outputs:joint_names")']`
  - downstream: `[]`
- `inputs:positionCommand`
  - value: `[]`
  - upstream: `[]`
  - downstream: `[]`

### artic_right
- path: `/World/ActionGraphs/Gripper_Control_Graph/artic_right_gripper`
- valid: `True`
- type_name: `isaacsim.core.nodes.IsaacArticulationController`

- `inputs:effortCommand`
  - value: `[60.0, 48.56764623153845]`
  - upstream: `['Attribute("/World/ActionGraphs/Gripper_Control_Graph/script_right_gripper.outputs:effort_cmds")']`
  - downstream: `[]`
- `inputs:execIn`
  - value: `1`
  - upstream: `['Attribute("/World/ActionGraphs/Gripper_Control_Graph/script_right_gripper.outputs:execOut")']`
  - downstream: `[]`
- `inputs:jointNames`
  - value: `['right_PGIA_joint1', 'right_PGIA_joint2']`
  - upstream: `['Attribute("/World/ActionGraphs/Gripper_Control_Graph/script_right_gripper.outputs:joint_names")']`
  - downstream: `[]`
- `inputs:positionCommand`
  - value: `[]`
  - upstream: `[]`
  - downstream: `[]`

# 6. Dynamic Control Runtime

- available: `True`
- articulation_path: `/World/Robot/base_link`
- error: `None`
- note: `Safe version: no app.update() is called.`

## DOF States

### left_j1_upper
- `dof_name`: `left_PGIA_joint1`
- `exists`: `True`
- `path`: `/World/Robot/joints/left_PGIA_joint1`
- `type`: `DofType.DOF_TRANSLATION`
- `parent_body`: `2203318222884`
- `child_body`: `2203318222887`
- `joint`: `2207613190182`
- `position`: `0.019905533641576767`
- `velocity`: `-0.0005870074965059757`
- `effort`: `-9.555386668580468e-07`
- `position_target`: `0.019999999552965164`
- `velocity_target`: `0.0`
- `properties`: `{'damping': 200.0, 'drive_mode': 'DriveMode.DRIVE_FORCE', 'has_limits': True, 'lower': 0.004000000189989805, 'max_effort': 500.0, 'max_velocity': 100.0, 'stiffness': 2000.0, 'type': 'DofType.DOF_TRANSLATION', 'upper': 0.11999999731779099}`

### left_j2_lower
- `dof_name`: `left_PGIA_joint2`
- `exists`: `True`
- `path`: `/World/Robot/joints/left_PGIA_joint2`
- `type`: `DofType.DOF_TRANSLATION`
- `parent_body`: `2203318222884`
- `child_body`: `2203318222888`
- `joint`: `2207613190183`
- `position`: `0.025862008333206177`
- `velocity`: `0.039717890322208405`
- `effort`: `7.167007538555481e-07`
- `position_target`: `0.019999999552965164`
- `velocity_target`: `0.0`
- `properties`: `{'damping': 200.0, 'drive_mode': 'DriveMode.DRIVE_FORCE', 'has_limits': True, 'lower': 0.004000000189989805, 'max_effort': 500.0, 'max_velocity': 100.0, 'stiffness': 2000.0, 'type': 'DofType.DOF_TRANSLATION', 'upper': 0.11999999731779099}`

### right_j1_upper
- `dof_name`: `right_PGIA_joint1`
- `exists`: `True`
- `path`: `/World/Robot/joints/right_PGIA_joint1`
- `type`: `DofType.DOF_TRANSLATION`
- `parent_body`: `2203318222886`
- `child_body`: `2203318222889`
- `joint`: `2207613190184`
- `position`: `0.019633974879980087`
- `velocity`: `0.0003236536867916584`
- `effort`: `-9.534218179396703e-07`
- `position_target`: `0.019999999552965164`
- `velocity_target`: `0.0`
- `properties`: `{'damping': 200.0, 'drive_mode': 'DriveMode.DRIVE_FORCE', 'has_limits': True, 'lower': 0.004000000189989805, 'max_effort': 500.0, 'max_velocity': 100.0, 'stiffness': 2000.0, 'type': 'DofType.DOF_TRANSLATION', 'upper': 0.11999999731779099}`

### right_j2_lower
- `dof_name`: `right_PGIA_joint2`
- `exists`: `True`
- `path`: `/World/Robot/joints/right_PGIA_joint2`
- `type`: `DofType.DOF_TRANSLATION`
- `parent_body`: `2203318222886`
- `child_body`: `2203318222890`
- `joint`: `2207613190185`
- `position`: `0.030509883537888527`
- `velocity`: `0.002587784081697464`
- `effort`: `7.150955525503377e-07`
- `position_target`: `0.019999999552965164`
- `velocity_target`: `0.0`
- `properties`: `{'damping': 200.0, 'drive_mode': 'DriveMode.DRIVE_FORCE', 'has_limits': True, 'lower': 0.004000000189989805, 'max_effort': 500.0, 'max_velocity': 100.0, 'stiffness': 2000.0, 'type': 'DofType.DOF_TRANSLATION', 'upper': 0.11999999731779099}`

# 7. ROS2 Commands

Generated command file:

```bash
/home/gtk/ros2_log/gripper_full_diagnosis/gripper_test_commands.sh
```

Examples:

```bash
/home/gtk/ros2_log/gripper_full_diagnosis/gripper_test_commands.sh left_close
/home/gtk/ros2_log/gripper_full_diagnosis/gripper_test_commands.sh left_open
/home/gtk/ros2_log/gripper_full_diagnosis/gripper_test_commands.sh right_close
/home/gtk/ros2_log/gripper_full_diagnosis/gripper_test_commands.sh right_open
/home/gtk/ros2_log/gripper_full_diagnosis/gripper_test_commands.sh both_close
/home/gtk/ros2_log/gripper_full_diagnosis/gripper_test_commands.sh both_open
/home/gtk/ros2_log/gripper_full_diagnosis/gripper_test_commands.sh cycle
```

# 8. PD / Control Parameter Guidance

## PositionCommand route

Recommended for strict equal travel:

```text
Graph:
  position_cmds -> positionCommand
  effortCommand disconnected

ScriptNode:
  position_cmds = [pos, pos]

USD Drive:
  stiffness = 1000~3000
  damping   = 100~300
  maxForce  = 300~500
```

## Effort-PD route

Only use if force-level control is required:

```text
Graph:
  effort_cmds -> effortCommand
  positionCommand disconnected

USD Drive:
  stiffness = 0 or very small
  damping   = 0 or very small

Software PD:
  effort = Kp*(target-current) - Kd*filtered_velocity + gravity_comp

Suggested initial parameters:
  KP = 250~500
  KD = 45~80
  EFFORT_MAX = 30~50
  EMA_ALPHA = 0.10~0.20
  J2_GRAV_COMP = -5.0~-10.0
  CLOSE_STEP = 0.0001~0.0003
  VEL_THRESH = 0.003
  CONTACT_CONFIRM = 8~15
```

# 9. Checklist

- Only one command mode should be active: positionCommand OR effortCommand, not both.
- If using effortCommand, same effort does not guarantee equal travel.
- If strict equal travel is required, prefer positionCommand or Mimic/gear coupling.
- If lower finger falls with no tray, check world_axis dot_gravity_down and Drive stiffness/damping.
- If first ROS command works but later commands fail, check FastDDS SHM and resource pressure.
- If GUI shows grey jointNames, check upstream connection instead of editing grey field directly.
- If left/right behavior differs, compare layer stack for xformOp, drive, mass, disableGravity.
- Do not run app.update() inside this diagnosis script in Isaac Sim 5.1.
