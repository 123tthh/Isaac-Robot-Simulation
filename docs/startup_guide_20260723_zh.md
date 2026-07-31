# Isaac OCS 项目启动指南

适用环境：Isaac Sim 5.1.0、Ubuntu 22.04、ROS 2 Humble 运行环境，以及 ROS 2 Rolling 本地参考文档。

项目实际运行默认使用 Humble。Isaac Sim 5.1.0 本地安装文档只将 Humble/Jazzy 列为 ROS Bridge 支持版本，因此 Rolling 只用于文档和代码核对，不作为当前运行时。

本指南依据：

- `/home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.md`
- `/home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/overview/release_notes.md`
- `/home/gtk/ai_docs/docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.md`

## 1. 首次构建与检查

```bash
cd /home/gtk/isaac_ocs_project
./scripts/project_control_20260723.sh build
./scripts/project_control_20260723.sh preflight
```

默认构建当前 SIM1/OCS2 运行链所需包，而不是工作区内所有第三方机器人包。需要验证整个工作区时使用：

```bash
./scripts/project_control_20260723.sh build --all
```

`preflight` 检查宿主机 GPU、Isaac Sim 安装、USD、ROS 2 overlay 和关键 ROS 包。它不会启动 Isaac Sim，也不会让机器人运动。

## 2. 正式启动顺序

终端 A：启动 Isaac Sim 5.1.0。

```bash
cd /home/gtk/isaac_ocs_project
./scripts/project_control_20260723.sh start-isaac \
  /home/gtk/isaac_ocs_project/assets/scenes/scene.usd
```

启动参数会启用 `isaacsim.ros2.sim_control`。默认候选场景
`assets/scenes/scene.usd` 已完成只读结构验证；它包含机器人、双臂、
夹爪和三路 RGB+D 相机，并包含：

- `Arm_Control_Graph`
- `Gripper_Control_Graph`
- `State_Telemetry_Graph`
- `Camera_Publish_Graph`

统一入口会清除外部 ROS 2 Python 3.10 和其他 Isaac 安装遗留的环境变量，
让 Isaac Sim 5.1.0 使用其自带 Python 3.11/ROS Bridge 环境。不要绕过该入口
在已经 source Humble 的终端直接执行 `isaac-sim.sh`。

该场景还包含 `Base_Drive_Graph`，可用于底盘差速命令订阅和驱动。场景还存在一条
`base_link/visuals` 未解析引用警告，不影响已识别的控制图、关节与相机 Prim，
但可能影响底盘基础视觉显示。

先只读检查场景结构：

```bash
./scripts/project_control_20260723.sh inspect-usd
```

正式启动时显式传入已验证场景：

```bash
./scripts/project_control_20260723.sh start-isaac \
  /home/gtk/isaac_ocs_project/assets/scenes/scene.usd
```

终端 B：启动 OCS2/ros2_control。该入口会启动 OCS2、ros2_control、TF 和 RViz2；不需要另开一个“看不到 RViz2”的手工命令。

```bash
cd /home/gtk/isaac_ocs_project
./scripts/project_control_20260723.sh launch-ocs2
```

终端 C：检查运行链路并进入 SIM1。需要键盘示教和单窗口六路相机监看时，使用带 `--open-camera-grid` 的入口：

```bash
cd /home/gtk/isaac_ocs_project
./scripts/project_control_20260723.sh sim1-check
./scripts/project_control_20260723.sh sim1-reset
./scripts/project_control_20260723.sh sim1-teach \
  --open-camera-grid --input-mode pygame \
  --record-cameras --camera-depth-save-every 1
```

`camera-grid` 是只读监看器，不会启动 Isaac Sim、OCS2、RViz2 或键盘控制；如果单独运行它，六格显示 `pub=0/no publisher` 不能作为场景故障结论。必须先让 Isaac Sim 加载 `615scene.usd` 并按 Play，再启动 OCS2 和示教器。

本次场景修复还把三个 RealSense D455 的远程 S3 引用改为项目内
`assets/scenes/615scene_20260723/isaac_assets/rsd455.usd`。若旧 Isaac Sim 进程已经打开过场景，请关闭并按上述命令重新打开，避免继续使用旧的 USD 组合结果。

正式相机示教参数见 [SIM1 操作手册](../projects/Trajectory/SIM1/GAME_GUIDE.md)。

## 3. 常用命令

```bash
# 回放最新 OCS2 轨迹
./scripts/project_control_20260723.sh sim1-demo

# pygame 多键示教
./scripts/project_control_20260723.sh sim1-teach \
  --input-mode pygame \
  --record-cameras

# 显式额外生成 LeRobot v2.1
SIM1_GENERATE_LEROBOT_V21=1 \
  ./scripts/project_control_20260723.sh sim1-teach
```

LeRobot v2.1 默认关闭，v3.0 默认保留。

## 4. 运行检查标准

`sim1-check` 输出中应重点确认：

- `/isaac_joint_states` 有 Isaac Sim publisher。
- `/arm_joint_cmd` 有 Isaac Sim subscriber。
- 左右 current pose 与 target topic 均存在。
- `joint_state_broadcaster` 和 `ocs2_arm_controller` 为 `active`。
- 使用 reset 时存在 `/reset_simulation`。

`game_control.sh check` 为运行态诊断，会把缺失项打印出来，但不会因为某个 topic 尚未出现就自动修改场景。

## 5. 停止

- 示教窗口：按 `Q` 或 `Esc`。
- OCS2：在终端 B 按 `Ctrl+C`。
- Isaac Sim：先停止 Play，再正常关闭窗口。

不要在 OCS2 仍发布 `/arm_joint_cmd` 时使用 `direct-demo`，否则两个控制源会冲突。
