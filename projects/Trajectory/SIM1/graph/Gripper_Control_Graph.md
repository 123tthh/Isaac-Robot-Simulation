# Action Graph 参数导出报告: Gripper_Control_Graph
> SIM1 工程总纲: [`../PROJECT_OVERVIEW.md`](../PROJECT_OVERVIEW.md)
> 相关操作指导: [`../GAME_GUIDE.md`](../GAME_GUIDE.md)
> 生成时间: 2026-05-28 11:28:57
> Graph 路径: `/World/ActionGraphs/Gripper_Control_Graph`
> 当前更新: 2026-05-28，夹爪 Script Node 改为 PGIA v4.2 位置控制。

---

## 0. 当前目标配置

Script Node 源文件：

```text
/home/gtk/Trajectory/SIM1/script_gripper_v4.2.py
/home/gtk/ros2_log/scripts/夹爪调试/00整理版/POS控制/SCRIPTS NODE/script_gripper_v4.2.py
```

`script_gripper_v4.2.py` 输出 `position_cmds`，因此 Articulation Controller 必须使用 `positionCommand` 输入。旧版 `effort_cmds -> effortCommand` 是 v4.1 Effort-PD 链路，切到 v4.2 后需要断开。

## 1. 节点清单与核心属性

| 节点名称 | 节点类型 | 关键参数 (Topic/Message/Path) |
| :--- | :--- | :--- |
| `on_playback_tick` | `omni.graph.action.OnPlaybackTick` | - |
| `ros2_context` | `isaacsim.ros2.bridge.ROS2Context` | - |
| `sub_left_gripper` | `isaacsim.ros2.bridge.ROS2Subscriber` | **topicName**: `left_gripper_controller/commands`<br>**messageName**: `Float64MultiArray` |
| `sub_right_gripper` | `isaacsim.ros2.bridge.ROS2Subscriber` | **topicName**: `right_gripper_controller/commands`<br>**messageName**: `Float64MultiArray` |
| `script_right_gripper` | `omni.graph.scriptnode.ScriptNode` | - |
| `script_left_gripper` | `omni.graph.scriptnode.ScriptNode` | - |
| `artic_left_gripper` | `isaacsim.core.nodes.IsaacArticulationController` | **robotPath**: `/World/Robot` |
| `artic_right_gripper` | `isaacsim.core.nodes.IsaacArticulationController` | **robotPath**: `/World/Robot` |

---

## 2. 拓扑连线 (Connections)

| 源节点 (Source) | 输出端口 (Output) | ➔ | 目标节点 (Target) | 输入端口 (Input) |
| :--- | :--- | :---: | :--- | :--- |
| `ros2_context` | `outputs:context` | ➔ | `sub_left_gripper` | `context` |
| `on_playback_tick` | `outputs:tick` | ➔ | `sub_left_gripper` | `execIn` |
| `ros2_context` | `outputs:context` | ➔ | `sub_right_gripper` | `context` |
| `on_playback_tick` | `outputs:tick` | ➔ | `sub_right_gripper` | `execIn` |
| `on_playback_tick` | `outputs:tick` | ➔ | `script_right_gripper` | `execIn` |
| `sub_right_gripper` | `outputs:data` | ➔ | `script_right_gripper` | `input_double_array` |
| `on_playback_tick` | `outputs:tick` | ➔ | `script_left_gripper` | `execIn` |
| `sub_left_gripper` | `outputs:data` | ➔ | `script_left_gripper` | `input_double_array` |
| `script_left_gripper` | `outputs:position_cmds` | ➔ | `artic_left_gripper` | `positionCommand` |
| `script_left_gripper` | `outputs:execOut` | ➔ | `artic_left_gripper` | `execIn` |
| `script_left_gripper` | `outputs:joint_names` | ➔ | `artic_left_gripper` | `jointNames` |
| `script_right_gripper` | `outputs:position_cmds` | ➔ | `artic_right_gripper` | `positionCommand` |
| `script_right_gripper` | `outputs:execOut` | ➔ | `artic_right_gripper` | `execIn` |
| `script_right_gripper` | `outputs:joint_names` | ➔ | `artic_right_gripper` | `jointNames` |
