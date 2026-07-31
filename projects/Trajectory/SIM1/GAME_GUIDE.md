# OCS2 双臂夹爪游戏指导书

工程总纲见 [PROJECT_OVERVIEW.md](PROJECT_OVERVIEW.md)。数据字段和转换格式见 [TRACE_DATA_FORMAT.md](TRACE_DATA_FORMAT.md)。

本工程面向 Isaac Sim 5.1 + ROS 2 的两种操作模式。AGENTS.md 要求优先参考本地 Rolling 文档；当前机器实际运行环境是 ROS 2 Humble（`ROS_DISTRO=humble`）。

- `teach`：轨迹操作示教。像玩游戏一样用键盘控制双臂末端目标，OCS2 负责求解关节，脚本同步记录可回放 CSV。
- `demo`：轨迹演示。回放示教 CSV 到 OCS2 目标话题，双臂仍然走 OCS2，夹爪走 PGIA v4.5 位置控制 Script Node。

本文件依据以下本地文档和工程文件整理：

- `/home/gtk/ai_docs/docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.md`
- `/home/gtk/ai_docs/docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.md`
- `/home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_manipulation.md`
- `/home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_tf.md`
- `/home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_simulation_control.md`
- `/home/gtk/isaac_ocs_project/projects/Trajectory/SIM1/graph/ALL_Graph.md`
- `/home/gtk/isaac_ocs_project/projects/Trajectory/SIM1/graph/Gripper_Control_Graph.md`
- `/home/gtk/isaac_ocs_project/data/ros2_log/scripts/夹爪调试/00整理版/POS控制/SCRIPTS NODE/script_gripper_v4.5.py`

## 1. 控制链路

OCS2 示教/演示链路：

```text
键盘/轨迹CSV
  -> /left_target/stamped, /right_target/stamped
  -> ocs2_arm_controller
  -> /arm_joint_cmd
  -> Isaac Sim Arm_Control_Graph
  -> /World/Robot 双臂关节
```

夹爪链路：

```text
键盘/轨迹CSV
  -> /left_gripper_controller/commands
  -> /right_gripper_controller/commands
  -> Gripper_Control_Graph
  -> script_gripper_v4.5.py
  -> positionCommand
```

`script_gripper_v4.5.py` 的命令语义：

```text
cmd > 0   打开夹爪
cmd <= 0  闭合/保持夹爪
```

Script Node 输出必须连接为：

```text
script_left_gripper.outputs:position_cmds  -> artic_left_gripper.inputs:positionCommand
script_right_gripper.outputs:position_cmds -> artic_right_gripper.inputs:positionCommand
script_*.outputs:joint_names               -> artic_*.inputs:jointNames
script_*.outputs:execOut                   -> artic_*.inputs:execIn
```

旧版 `effort_cmds -> effortCommand` 是 v4.1 Effort-PD 链路，切到 v4.5 后不要再使用。

## 2. 启动顺序（2026-07-23，必须按此顺序）

终端 A：启动 Isaac Sim 5.1.0，并明确加载整理后的 `615scene.usd`。启动后在 Isaac Sim GUI 中按 **Play**；未按 Play 时，ROS 2 相机 topic 和仿真控制服务都不会工作。

```bash
cd /home/gtk/isaac_ocs_project
./scripts/project_control_20260723.sh start-isaac \
  /home/gtk/isaac_ocs_project/assets/scenes/615scene_20260723/615scene.usd
```

确认场景中已有：

- `Arm_Control_Graph` 订阅 `/arm_joint_cmd`
- `Gripper_Control_Graph` 订阅 `/left_gripper_controller/commands` 和 `/right_gripper_controller/commands`
- `State_Telemetry_Graph` 发布 `/isaac_joint_states`、`/clock`
- `isaacsim.ros2.sim_control` 扩展已启用，用于 `/reset_simulation`

当前系统已安装 Isaac Sim Simulation Control 依赖：

```bash
sudo apt-get install -y ros-humble-simulation-interfaces
source /home/gtk/isaac_ocs_project/projects/ros2_ws/install/setup.bash
ros2 pkg prefix simulation_interfaces
ros2 service type /reset_simulation
```

期望输出：

```text
/opt/ros/humble
simulation_interfaces/srv/ResetSimulation
```

终端 B：启动 OCS2。该 launch 文件会同时启动 `robot_state_publisher`、控制器、OCS2 和 RViz2（默认 `enable_rviz:=true`）；它不会替代 Isaac Sim，也不会自动按 Play。

```bash
cd /home/gtk/isaac_ocs_project
./scripts/project_control_20260723.sh launch-ocs2
```

等待控制器 active。

```bash
ros2 control list_controllers
```

预期至少看到：

```text
joint_state_broadcaster active
ocs2_arm_controller active
```

终端 C：先检查 ROS 2 链路，再启动键盘示教。`teach` 才是键盘/pygame 控制入口。

```bash
cd /home/gtk/isaac_ocs_project
./scripts/project_control_20260723.sh sim1-check
./scripts/project_control_20260723.sh sim1-teach \
  --open-camera-grid --input-mode pygame \
  --record-cameras --camera-depth-save-every 1
```

注意：`./game_control.sh camera-grid` **只打开 6 路相机监看器**，不会启动 Isaac Sim、OCS2、RViz2 或键盘控制；因此单独运行它时显示 `pub=0 / no publisher` 是正常的。若只想监看，必须先完成终端 A 并按 Play。

启动后可在任意已 source ROS 2 的终端验证：

```bash
ros2 topic list | rg 'head_cam|left_cam|right_cam|arm_joint_cmd|isaac_joint_states'
ros2 control list_controllers
```

相机窗口应逐步显示 `pub=1`，控制器至少应显示 `joint_state_broadcaster active` 和 `ocs2_arm_controller active`。

### Isaac Sim Script Editor 相机检查/修复

如需直接在 Isaac Sim 内读取或修复相机图，打开 **Window → Script Editor**，执行：

```python
exec(open('/home/gtk/isaac_ocs_project/scripts/inspect_repair_615scene_camera_script_editor_20260724.py', encoding='utf-8').read())
```

脚本默认只读，报告写入
`/home/gtk/isaac_ocs_project/reports/615scene_camera_script_editor_20260724.md`。
报告确认存在缺失 `NodeGraphNodeAPI` 节点后，才将脚本顶部
`APPLY_REPAIR = False` 改为 `True`，再次执行；脚本会先备份 USD，再写回
`615scene.usd`。写回后重新打开场景并按 Play。

## 3. 模式切换

### 示教模式

示教模式用于人工操作并录制轨迹。

默认示教步长已经按右臂失败现象收紧为 `--step 0.003 --rot-step-deg 0.5`。默认 `./game_control.sh teach` 不启用双臂移动空间位置/范围限制，不启用 target-current lag gate，也不启用姿态 gate；这些保护都需要通过命令行显式传参启用。watchdog 只告警和记录诊断，不冻结右臂 target，不拦截 `1/2` 选臂后的手臂控制键。

正式示教前先确认以下链路都在线：

```bash
ros2 topic info /arm_joint_cmd -v
ros2 topic info /sim1/base_diff_cmd -v
ros2 topic info /sim1/base_drive_debug -v
ros2 topic echo /joint_states --once
ros2 topic echo /isaac_joint_states --once
```

`/sim1/base_drive_debug` 应显示 `sim1_base_drive_debug_scriptnode` 作为 publisher。`/sim1/base_diff_cmd` 应显示 Isaac `Base_Drive_Graph` 作为 subscriber。圆弧转弯触发不要用 `ros2 topic pub --once` 做验收；一次性消息可能被 Isaac/ROS2 Bridge 时序错过。用 `base_drive_path_runner.py left90`，或用 `ros2 topic pub --times 5 --rate 10` 连续发数帧。

#### A. 完整示教采集：双臂 + 夹爪 + 相机 + 底盘

正式采集优先使用这个命令。它同时启用 pygame 多键 OCS2 target 示教、夹爪控制、RGB+D 相机记录、底盘片选控制和底盘字段记录。

注意：`--record-cameras` 只负责保存 RGB+D 数据，不会打开相机画面窗口。

```bash
./game_control.sh teach --input-mode pygame \
  --linear-speed 0.025 \
  --angular-speed-deg 4.0 \
  --control-rate 50 \
  --base-speed 0.25 \
  --base-yaw-rate 1.0 \
  --base-turn-mode arc90 \
  --max-position-error 0.20 \
  --hard-position-error 0.35 \
  --max-orientation-error-deg 15.0 \
  --watchdog-position-error 0.30 \
  --watchdog-orientation-error-deg 30.0 \
  --watchdog-severe-position-error 0.50 \
  --watchdog-severe-orientation-error-deg 60.0 \
  --right-joint1-lower-safety -2.2 \
  --right-joint1-upper-safety 2.2 \
  --record-cameras \
  --camera-depth-save-every 1
```

如果示教时要同步打开 6 宫格相机监看窗口，必须在同一条命令里加 `--open-camera-grid`：

```bash
./game_control.sh teach --open-camera-grid --input-mode pygame \
  --linear-speed 0.025 \
  --angular-speed-deg 4.0 \
  --control-rate 50 \
  --base-speed 0.25 \
  --base-yaw-rate 1.0 \
  --base-turn-mode arc90 \
  --max-position-error 0.20 \
  --hard-position-error 0.35 \
  --max-orientation-error-deg 15.0 \
  --watchdog-position-error 0.30 \
  --watchdog-orientation-error-deg 30.0 \
  --watchdog-severe-position-error 0.50 \
  --watchdog-severe-orientation-error-deg 60.0 \
  --right-joint1-lower-safety -2.2 \
  --right-joint1-upper-safety 2.2 \
  --record-cameras \
  --camera-depth-save-every 1
```

`teach-with-cameras` 只是 `teach --open-camera-grid` 的别名。正式采集如果使用这个别名，也必须带同一组完整正式参数，避免漏掉底盘、误差门限、watchdog 和安全范围参数。

pygame 窗口必须保持焦点。键位总览：

```text
片选：1 左臂，2 右臂，3 底盘；任一时刻只控制一个对象，可随时切换
手臂移动：选 1/2 后，W/S 控 X，A/D 控 Y，R/F 控 Z
手臂旋转：选 1/2 后，U/O、I/K、J/L
夹爪操控：V/B 左闭/开，N/M 右闭/开，空格切换当前选中臂夹爪
底盘运动：选 3 后，↑ 前进，↓ 后退，← 左转圆弧 90°，→ 右转圆弧 90°
恢复/退出：Z 左臂慢速恢复初始姿态，X 右臂慢速恢复初始姿态，Q/Esc 结束记录
```

#### B. 默认终端示教：调试/SSH/无图形环境

这是最小命令，一次处理一个终端按键事件，适合检查 OCS2、夹爪和记录链路是否正常。

```bash
./game_control.sh teach
```

等价于：

```bash
./game_control.sh teach --input-mode terminal
```

#### C. pygame 多键示教：基础调试模式

`pygame` 模式会打开一个窗口并读取键盘按住状态，支持 `W+A`、`W+A+R` 这种多键同时输入，并按固定频率输出连续速度；窗口必须保持焦点。这个基础命令只用于调试控制手感，正式采集使用 A 的完整示教采集命令。

```bash
./game_control.sh teach --input-mode pygame \
  --linear-speed 0.025 \
  --angular-speed-deg 4.0 \
  --control-rate 50
```

`pygame` 模式中手臂平移和旋转方向会分别归一化，所以 `W+A` 斜向移动不会比单独 `W` 快 `sqrt(2)` 倍。`Q` 或 `Esc` 结束记录，`Z` 慢速恢复左臂初始姿态，`X` 慢速恢复右臂初始姿态。

#### D. pygame + 误差门限/watchdog + RGB+D：推荐正式录制模式

这个命令同步采集轨迹、诊断、夹爪、关节、3 路 RGB 视频和 3 路 depth npy，并默认启用底盘字段记录。它不启用 workspace 位置边界，但会启用 target-current lag gate、姿态 gate、右臂运行时 watchdog 和 right_joint1 安全范围。正式采集默认使用 RGB+D，不使用 `--camera-rgb-only`。如果要同时采集底盘操作，优先使用上面的完整命令 A。

```bash
./game_control.sh teach --input-mode pygame \
  --linear-speed 0.025 \
  --angular-speed-deg 4.0 \
  --control-rate 50 \
  --base-speed 0.25 \
  --base-yaw-rate 1.0 \
  --max-position-error 0.20 \
  --hard-position-error 0.35 \
  --max-orientation-error-deg 15.0 \
  --watchdog-position-error 0.30 \
  --watchdog-orientation-error-deg 30.0 \
  --watchdog-severe-position-error 0.50 \
  --watchdog-severe-orientation-error-deg 60.0 \
  --right-joint1-lower-safety -2.2 \
  --right-joint1-upper-safety 2.2 \
  --record-cameras \
  --camera-depth-save-every 1
```

#### E. pygame + workspace clamp：限制 target 累计推出可达域

这个命令在 D 的基础上增加左右臂 workspace clamp。左右臂参数数值一致；涉及 Y 方向时按左右镜像解释。

```bash
./game_control.sh teach --input-mode pygame \
  --linear-speed 0.025 \
  --angular-speed-deg 4.0 \
  --control-rate 50 \
  --base-speed 0.25 \
  --base-yaw-rate 1.0 \
  --max-position-error 0.20 \
  --hard-position-error 0.35 \
  --max-orientation-error-deg 15.0 \
  --watchdog-position-error 0.30 \
  --watchdog-orientation-error-deg 30.0 \
  --watchdog-severe-position-error 0.50 \
  --watchdog-severe-orientation-error-deg 60.0 \
  --right-joint1-lower-safety -2.2 \
  --right-joint1-upper-safety 2.2 \
  --left-max-target-offset 0.45 \
  --left-max-axis-offset 0.40 0.40 0.25 \
  --right-max-target-offset 0.45 \
  --right-max-axis-offset 0.40 0.40 0.25 \
  --record-cameras \
  --camera-depth-save-every 1
```

#### F. 终端模式 + 安全门限：无图形环境下的保守示教

如果不能打开 pygame 窗口，但仍希望启用安全保护，用这个命令：

```bash
./game_control.sh teach --input-mode terminal \
  --max-position-error 0.20 \
  --hard-position-error 0.35 \
  --max-orientation-error-deg 15.0 \
  --watchdog-position-error 0.30 \
  --watchdog-orientation-error-deg 30.0 \
  --watchdog-severe-position-error 0.50 \
  --watchdog-severe-orientation-error-deg 60.0 \
  --right-joint1-lower-safety -2.2 \
  --right-joint1-upper-safety 2.2
```

默认输出：

```text
trace_data/raw/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS.csv
trace_data/raw/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS.diagnostics.csv
trace_data/raw/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS.metadata.json
trace_data/raw/manual_ocs2_keyboard_trace_latest.csv -> 最近一次成功示教
trace_data/raw/manual_ocs2_keyboard_trace_latest.diagnostics.csv -> 最近一次成功示教诊断数据
```

这个 CSV 记录了：

- wall time
- ROS time
- 左右末端 target position/quaternion
- 左右末端 current position/quaternion
- 左右夹爪命令
- 左右夹爪闭合状态：`1` 表示闭合/保持，`0` 表示打开，空值表示未知
- `/joint_states` 的关节名和关节位置
- `/isaac_joint_states` 的关节力矩/effort；如果该话题没有 effort，则回退记录 `/joint_states.effort`，仍为空则表示当前链路没有发布力矩数据

同名 `.diagnostics.csv` 额外记录数值化诊断数据：

- `left/right_joint1_position` 到 `left/right_joint7_position`
- `left/right_joint1_effort` 到 `left/right_joint7_effort`
- `left/right_target_current_dx/dy/dz/norm`
- `left/right_orientation_error_deg`
- `blocked_translation_count` 和 `blocked_rotation_count`
- `workspace_clamp_count` 和 `hard_lag_block_count`
- `watchdog_freeze_right`、`watchdog_freeze_count`、`watchdog_severe_count`、`watchdog_joint_limit_count`；当前保留字段名用于兼容旧 CSV，watchdog 只告警，不再冻结右臂 target。
- `/fsm_command` 最近值、是否为 OCS2 模式 `3`、非 OCS2 累计次数
- OCS2 日志中 `SQP did not converge` 累计次数
- OCS2 日志中 cost 为 NaN/Inf 或超过阈值的累计次数
- 相机 recorder 状态、输出目录和 WARNING

默认扫描 `$ROS_LOG_DIR/latest` 或 `~/.ros/log` 下的 OCS2 日志。可手动指定：

```bash
./game_control.sh teach --ocs2-log-dir ~/.ros/log/latest --ocs2-cost-explosion-threshold 1000000
```

默认 `./game_control.sh teach` 不启用 workspace 可达域边界，也不启用 target-current lag gate。需要按命令行显式传参才启用这些限制：

```bash
./game_control.sh teach \
  --max-position-error 0.20 \
  --hard-position-error 0.35 \
  --max-orientation-error-deg 15.0
```

左右臂 workspace clamp 推荐使用同一组数值；涉及 Y 方向时按左右镜像解释：

```bash
./game_control.sh teach \
  --step 0.003 \
  --rot-step-deg 0.5 \
  --left-max-target-offset 0.45 \
  --left-max-axis-offset 0.40 0.40 0.25 \
  --right-max-target-offset 0.45 \
  --right-max-axis-offset 0.40 0.40 0.25
```

镜像方向约定：左臂 `Y+` 是外侧，右臂 `Y-` 是外侧；左臂 `Y-` 是内侧，右臂 `Y+` 是内侧。日志会同时给出 base 坐标 `hit_axis` 和按左右臂镜像后的 `hit_axis_arm`。

运行时 watchdog 默认不启用。需要显式传参后，它会在 tick 中持续监控右臂 current 是否突然崩：

```bash
./game_control.sh teach \
  --watchdog-position-error 0.30 \
  --watchdog-orientation-error-deg 30.0 \
  --watchdog-severe-position-error 0.50 \
  --watchdog-severe-orientation-error-deg 60.0 \
  --right-joint1-lower-safety -2.2 \
  --right-joint1-upper-safety 2.2
```

```text
right_target_current_norm > 0.30 m 或 right_orientation_error_deg > 30 deg:
  打印 watchdog 警告并记录诊断，不冻结右臂 target，不拒绝右臂平移旋转按键

right_target_current_norm > 0.50 m 或 right_orientation_error_deg > 60 deg:
  打印严重发散警告，不自动 HOME

right_joint1 超出 [-2.2, +2.2] rad:
  打印安全范围警告并记录诊断，不冻结右臂 target
```

watchdog 不自动 HOME，也不锁定右臂，避免打断完整工作循环。需要右臂回初始姿态时按 `X`，脚本使用 MoveJ 慢速恢复右臂最佳初始关节姿态，再用 current pose 重建 target。左臂回初始姿态按 `Z`。

按 `Q` 或 `Esc` 结束记录。脚本退出后，`game_control.sh teach` 会询问本次轨迹是否有失败或力矩消失，并生成同名标注文件：

```text
trace_data/raw/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS.label.md
```

标注选项：

```text
0  success / no failure observed
1  failure: joint torque disappeared
2  failure: singularity or near-singularity suspected
3  failure: other
```

### 示教阶段同步观看 6 路相机画面

本工程的 6 路图像流是 `3 路 RGB + 3 路 depth`：

```text
/head_cam/color/image_raw
/head_cam/depth/image_rect_raw
/left_cam/color/image_raw
/left_cam/depth/image_rect_raw
/right_cam/color/image_raw
/right_cam/depth/image_rect_raw
```

优先使用集成示教入口自动打开单窗口 6 宫格 viewer。它会把 6 路 ROS 图像流拼成一个总窗口：

```text
第一行：head_rgb   left_rgb   right_rgb
第二行：head_depth left_depth right_depth
```

先确认 Isaac Sim 已按 Play，且 6 个 topic 都存在：

```bash
source /home/gtk/isaac_ocs_project/projects/ros2_ws/install/setup.bash
ros2 topic list | rg '(/head_cam|/left_cam|/right_cam)/(color/image_raw|depth/image_rect_raw)$'
```

预期应看到这 6 路 topic 全部列出。正式示教时直接执行：

```bash
cd /home/gtk/isaac_ocs_project/projects/Trajectory/SIM1
source /home/gtk/isaac_ocs_project/projects/ros2_ws/install/setup.bash
./game_control.sh teach --open-camera-grid --input-mode pygame \
  --record-cameras \
  --camera-depth-save-every 1
```

`./game_control.sh teach-with-cameras ...` 只是 `./game_control.sh teach --open-camera-grid ...` 的别名。正式采集时不要省略 A 命令中的底盘、误差门限、watchdog 和安全范围参数。

只想单独检查 6 路相机画面时，执行：

```bash
cd /home/gtk/isaac_ocs_project/projects/Trajectory/SIM1
source /home/gtk/isaac_ocs_project/projects/ros2_ws/install/setup.bash
./game_control.sh camera-grid
```

集成到 `teach` 后，示教开始前会自动打开相机窗口；`Q` 或 `Esc` 结束示教后，`game_control.sh` 会自动关闭相机窗口。

可调整每个宫格尺寸：

```bash
cd /home/gtk/isaac_ocs_project/projects/Trajectory/SIM1
source /home/gtk/isaac_ocs_project/projects/ros2_ws/install/setup.bash
./game_control.sh teach --open-camera-grid \
  --camera-grid-cell-width 520 \
  --camera-grid-cell-height 300 \
  --input-mode pygame
```

可调整 depth 画面的伪彩显示范围：

```bash
cd /home/gtk/isaac_ocs_project/projects/Trajectory/SIM1
source /home/gtk/isaac_ocs_project/projects/ros2_ws/install/setup.bash
./game_control.sh camera-grid --depth-percentile 90
```

示教时使用：

```bash
./game_control.sh teach --open-camera-grid \
  --camera-grid-depth-percentile 90 \
  --input-mode pygame
```

`--camera-grid-depth-percentile` 只改变 viewer 里的深度伪彩归一化上限，不改变 ROS topic 中的真实 depth 数值，也不改变录制到 `.npy` 的深度数据。默认值是 `98`；如果近处目标细节不明显，可以试 `95`、`90`；如果远处过早变黑，可以调回 `98` 或 `99`。

说明：

- RGB 窗口直接看彩色画面。
- depth 画面会显示为伪彩深度图，这是 viewer 的正常显示方式。
- 相机录制和单窗口监看默认使用 `best_effort` QoS，匹配 Isaac Sim 5.1.0
  传感器图像 publisher；如现场桥接明确配置为可靠传输，再显式加
  `--camera-reliability reliable` 或 `--reliability reliable`。
- 录制启动会等待最多 5 秒让 Isaac Sim 进入 Play 并完成 6 路 publisher 发现；
  仍显示 `pub=0` 时，记录会保留诊断元数据，不会把空相机目录误判为有效数据。
- 如果格子里显示 `pub=0` 或 `no publisher`，说明该 ROS topic 当前没有 Isaac publisher；先确认 Isaac Sim 已按 Play，且 `/World/ActionGraphs/Camera_Publish_Graph` 的 `ROS2CameraHelper` 已配置 `topicName/type/renderProductPath`。
- viewer 窗口里按 `Q` 或 `Esc` 退出。
- `pygame` 示教时要保持示教窗口焦点；相机窗口只用于监看，不要频繁点击。
- 如果机器显卡压力明显增大，先把 viewer 缩小，或把 `--camera-grid-cell-width/--camera-grid-cell-height` 调低。

如果单窗口 viewer 因桌面环境或 OpenCV GUI 后端异常无法启动，再退回到 6 个 `rqt_image_view` 独立窗口的方案：

```bash
source /home/gtk/isaac_ocs_project/projects/ros2_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view /head_cam/color/image_raw &
ros2 run rqt_image_view rqt_image_view /head_cam/depth/image_rect_raw &
ros2 run rqt_image_view rqt_image_view /left_cam/color/image_raw &
ros2 run rqt_image_view rqt_image_view /left_cam/depth/image_rect_raw &
ros2 run rqt_image_view rqt_image_view /right_cam/color/image_raw &
ros2 run rqt_image_view rqt_image_view /right_cam/depth/image_rect_raw &
```

如果你确实想在 Isaac Sim GUI 内看机载相机画面，可参考本地 Isaac Sim 文档 `/home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_setup_tutorials/tutorial_gui_camera_sensors.md`：

- `Window > Viewports > Viewport 2` 继续加更多 viewport。
- 在每个 viewport 顶部的 `Camera` 菜单切到对应相机。
- `Tools > Robotics > Camera Inspector` 可检查单个相机画面。
- 单独查看 depth/disparity 时，可选中相机 render product，然后在 `Render Settings > Post Processing > Depth Sensor` 中查看或调整 Depth Sensor 显示/后处理选项。本地 Isaac Sim 文档说明这些 Post Processing 设置会作用到场景中的 render products，包括 viewport；这属于 Isaac GUI 显示/后处理设置，不等同于 ROS 侧 `camera-grid --depth-percentile`。

但对当前工程的示教阶段，ROS 侧单窗口 `camera-grid` 更适合边示教边看 6 路图像，也更接近最终录制 topic。

### 相机同步录制

Isaac Sim Play 且 3 个 D455 的 ROS2 图像 topic 已发布时，可以在示教时同时启动独立相机 recorder：

```bash
./game_control.sh teach --input-mode pygame \
  --linear-speed 0.025 \
  --angular-speed-deg 4.0 \
  --control-rate 50 \
  --max-position-error 0.20 \
  --hard-position-error 0.35 \
  --max-orientation-error-deg 15.0 \
  --watchdog-position-error 0.30 \
  --watchdog-orientation-error-deg 30.0 \
  --watchdog-severe-position-error 0.50 \
  --watchdog-severe-orientation-error-deg 60.0 \
  --right-joint1-lower-safety -2.2 \
  --right-joint1-upper-safety 2.2 \
  --record-cameras \
  --camera-depth-save-every 1
```

直接运行键盘入口也支持同样参数：

```bash
python3 keyboard_ocs2_gripper_teleop.py --input-mode pygame \
  --linear-speed 0.025 \
  --angular-speed-deg 4.0 \
  --control-rate 50 \
  --max-position-error 0.20 \
  --hard-position-error 0.35 \
  --max-orientation-error-deg 15.0 \
  --watchdog-position-error 0.30 \
  --watchdog-orientation-error-deg 30.0 \
  --watchdog-severe-position-error 0.50 \
  --watchdog-severe-orientation-error-deg 60.0 \
  --right-joint1-lower-safety -2.2 \
  --right-joint1-upper-safety 2.2 \
  --record-cameras \
  --camera-depth-save-every 1
```

输出沿用本次示教 run_id：

```text
trace_data/raw/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS/
  camera/
    videos/
      head_rgb.mp4
      left_rgb.mp4
      right_rgb.mp4
    depth/
      head_depth/*.npy
      left_depth/*.npy
      right_depth/*.npy
    camera_timestamps.csv
    summary.json
    camera_recorder.log
```

如需降低深度数据体积，可以在上面的完整正式命令中把：

```bash
--camera-depth-save-every 1
```

替换为：

```bash
--camera-depth-save-every 5
```

正式采集不使用 `--camera-rgb-only`，避免只得到“轨迹 + RGB”而缺少 depth。

如果某个相机 topic 不存在或 publisher count 为 0，示教仍继续，WARNING 写入 `.diagnostics.csv` 和 `.metadata.json`。后续 LeRobot 转换应使用 `camera_timestamps.csv` 中的 ROS timestamp 或 wall timestamp 与轨迹对齐，不要按帧序号对齐。

示教成功结束后，`game_control.sh teach` 默认会执行后处理：

```text
raw trace/camera 保留在 trace_data/raw/
cleaned trace/camera 写入 trace_data/cleaned/
LeRobotDataset v3.0 写入 trace_data/lerobot_v30/
LeRobotDataset v2.1 写入 trace_data/lerobot_v21/
```

如需采集完整 raw RGB+D 但跳过自动清洗/转换，关闭自动后处理：

```bash
SIM1_PROCESS_AFTER_TEACH=0 ./game_control.sh teach --input-mode pygame \
  --linear-speed 0.025 \
  --angular-speed-deg 4.0 \
  --control-rate 50 \
  --max-position-error 0.20 \
  --hard-position-error 0.35 \
  --max-orientation-error-deg 15.0 \
  --watchdog-position-error 0.30 \
  --watchdog-orientation-error-deg 30.0 \
  --watchdog-severe-position-error 0.50 \
  --watchdog-severe-orientation-error-deg 60.0 \
  --right-joint1-lower-safety -2.2 \
  --right-joint1-upper-safety 2.2 \
  --record-cameras \
  --camera-depth-save-every 1
```

### 演示模式

演示模式回放上一步示教得到的 CSV。这个模式是 OCS2 target 回放，不是 Isaac 关节直连回放，因此必须先启动 Isaac Sim Play 和 OCS2。

```bash
./game_control.sh demo
```

慢速演示：

```bash
./game_control.sh demo --speed 0.5
```

循环演示：

```bash
./game_control.sh demo --loop
```

指定轨迹：

```bash
TRACE=/tmp/my_trace.csv ./game_control.sh demo
```

指定已清洗轨迹：

```bash
TRACE=trace_data/cleaned/manual_ocs2_keyboard_trace_20260529_154535_cleaned.csv ./game_control.sh demo
```

也可以直接调用回放脚本：

```bash
./replay_ocs2_target_trace.py --trace trace_data/cleaned/manual_ocs2_keyboard_trace_20260529_154535_cleaned.csv
```

轨迹文件名可以自己改，只要 `TRACE=` 或 `--trace` 指向实际 CSV 文件即可：

```bash
mv trace_data/cleaned/manual_ocs2_keyboard_trace_20260529_154535_cleaned.csv trace_data/cleaned/pick_test_cleaned.csv
TRACE=trace_data/cleaned/pick_test_cleaned.csv ./game_control.sh demo
```

清洗示教轨迹：

```bash
./trace_cleaning/scripts/clean_ocs2_trace.py trace_data/raw/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS.csv
TRACE=trace_data/cleaned/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS_cleaned.csv ./game_control.sh demo
```

带相机数据的完整 episode 清洗与转换：

```bash
./trace_cleaning/scripts/process_sim1_episode.py \
  trace_data/raw/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS.csv
```

将清洗后 CSV 转成 LeRobot dataset v3.0 本地目录：

```bash
./trace_cleaning/scripts/convert_ocs2_trace_to_lerobot_v30.py \
  trace_data/cleaned/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS_cleaned.csv \
  --output trace_data/lerobot_v30/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS
```

将清洗后 CSV 转成 LeRobot dataset v2.1 本地目录：

```bash
./trace_cleaning/scripts/convert_ocs2_trace_to_lerobot_v21.py \
  trace_data/cleaned/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS_cleaned.csv \
  --output trace_data/lerobot_v21/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS
```

v3.0 输出结构：

```text
trace_data/lerobot_v30/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS/
  data/chunk-000/file-000.parquet
  videos/observation.images.head/chunk-000/file-000.mp4
  videos/observation.images.left_wrist/chunk-000/file-000.mp4
  videos/observation.images.right_wrist/chunk-000/file-000.mp4
  meta/info.json
  meta/stats.json
  meta/tasks.parquet
  meta/episodes/chunk-000/file-000.parquet
```

v2.1 输出结构：

```text
trace_data/lerobot_v21/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS/
  data/chunk-000/episode_000000.parquet
  videos/chunk-000/observation.images.head/episode_000000.mp4
  videos/chunk-000/observation.images.left_wrist/episode_000000.mp4
  videos/chunk-000/observation.images.right_wrist/episode_000000.mp4
  meta/info.json
  meta/tasks.jsonl
  meta/episodes.jsonl
  meta/episodes_stats.jsonl
```

转换脚本要求新轨迹包含：

- `joint_names` 和 `joint_positions`，且能找到 `left_joint1` 到 `left_joint7`、`right_joint1` 到 `right_joint7`
- `left_gripper_closed`、`right_gripper_closed`
- `effort_joint_names` 和 `joint_efforts`

老轨迹没有力矩和夹爪闭合状态时只能做格式兼容检查，不能恢复真实力矩：

```bash
./trace_cleaning/scripts/convert_ocs2_trace_to_lerobot_v30.py \
  trace_data/cleaned/manual_ocs2_keyboard_trace_20260529_154535_cleaned.csv \
  --output trace_data/lerobot_v30/manual_ocs2_keyboard_trace_20260529_154535_cleaned_lerobot_v30 \
  --allow-missing-effort \
  --missing-gripper-closed 1.0
```

### LeRobot 直连演示

这个模式直接向 Isaac 的 `/arm_joint_cmd` 发布 14 维关节轨迹，不经过 OCS2。只用于检查 converted LeRobot 数据和 Isaac Action Graph。

```bash
./game_control.sh direct-demo
```

注意：不要让 OCS2 同时持续发布 `/arm_joint_cmd`，否则两个 publisher 会抢同一个 Arm_Control_Graph 输入。

## 4. 游戏键位

选择控制对象：

```text
1  左臂
2  右臂
3  底盘
```

示教过程中 `1/2/3` 是互斥片选：任一时刻只控制左臂、右臂、底盘中的一个对象，但可以随时切换。

平移选中的 joint7/末端目标：

```text
W/S  +X / -X
A/D  +Y / -Y
R/F  +Z / -Z
```

只有选中 `1` 或 `2` 时，W/S/A/D/R/F 和 U/O/I/K/J/L 才会控制手臂；选中 `3` 时这些键不会移动手臂。

旋转选中的 joint7/末端目标：

```text
I/K  pitch +/-
J/L  yaw +/-
U/O  roll +/-
```

夹爪：

```text
Space       切换当前选中手臂夹爪开/合；选中底盘时不动作
V/B         左夹爪闭/开
N/M         右夹爪闭/开
```

底盘：

```text
先按 3 选中底盘
↑           前进，发布 linear.x = +base_speed
↓           后退，发布 linear.x = -base_speed
←           触发左转圆弧 90°，发布 linear.x = 0, angular.z = +base_yaw_rate
→           触发右转圆弧 90°，发布 linear.x = 0, angular.z = -base_yaw_rate
```

运行控制：

```text
+/-  调整末端平移步长
P    立即发布当前 target
H    打印帮助
Z    左臂慢速恢复到最佳初始关节姿态，然后重建 OCS2 target
X    右臂慢速恢复到最佳初始关节姿态，然后重建 OCS2 target
Q/Esc  结束记录并执行退出复位
```

`Q` 的复位请求使用：

```bash
ros2 service call /reset_simulation simulation_interfaces/srv/ResetSimulation "{scope: 255}"
```

OCS2 HOME 姿态来自：

```text
/home/gtk/isaac_ocs_project/projects/ros2_ws/src/robot/config/ros2_control/ocs2_controllers.yaml
```

当前 `home_1/home_2` 都已改为双臂最佳初始姿态，避免任何 HOME 切换回到 14 关节全零打开姿态。示教中 `Z/X` 使用 MoveJ 目标关节位置慢速恢复左/右臂，不再把 `Z` 作为双臂 HOME 快捷键。

若 `Q/Esc` 退出后没有复位，先运行统一恢复入口：

```bash
./game_control.sh reset
```

该入口会检查当前终端是否能看到 `simulation_interfaces`，尝试将 Isaac Sim 切到 playing，调用 `/reset_simulation`，并连续发布 HOME/FSM=1。默认不立刻切回 FSM=3，避免 Isaac reset 后 target/current pose 尚未同步时又被旧 target 覆盖。若确认需要 reset 后自动回 OCS2，可用：

```bash
RESET_ACTIVATE_OCS2=1 ./game_control.sh reset
```

若 reset 后 target/current pose 仍不同步，重新进入 `teach` 后按 `Z` 或 `X` 慢速恢复对应手臂，让示教器用 current pose 重建 target。

## 5. 推荐工作流

先做一次小范围示教，确认 OCS2 目标和夹爪都通。

```bash
./game_control.sh teach
```

正式录制、需要同时按住多方向键时，优先使用“示教模式”小节里的 A 命令；如果需要 workspace clamp，使用 E 命令。

操作 10 到 20 秒后按 `Esc` 退出，然后立即回放：

```bash
./game_control.sh demo
```

如果回放正确，也不建议回到旧的 `0.01 m / 2 deg` 连续输入；右臂高姿态约束任务容易在 target-current 误差堆积后进入坏条件区。确实需要加速时，优先只小幅增加平移步长：

```bash
./game_control.sh teach --step 0.005 --rot-step-deg 0.5
```

## 6. 切换原则

`teach` 和 `demo` 可以在 OCS2 启动后切换，因为二者都只发布 OCS2 target 和夹爪命令。退出一个脚本后启动另一个脚本即可。

`demo` 默认会发布 `/fsm_command=3` 激活 OCS2，但不会替你启动 OCS2 launch。回放前仍需保持 `ros2 launch r1_description ocs2_isaac.launch.py` 正在运行。

`direct-demo` 是直连 Isaac 的关节回放，不是 OCS2 target 回放。它用于数据检查，不建议和 OCS2 闭环同时控制。

## 7. 常见问题

没有收到 `/left_current_pose` 或 `/right_current_pose`：

```bash
ros2 topic echo --once /left_current_pose
ros2 topic echo --once /right_current_pose
```

如果没有输出，先检查 OCS2 launch 和 RViz/TF 链路。

机器人不动但 `/arm_joint_cmd` 有数据：

```bash
ros2 topic echo --once /fsm_command
ros2 topic hz /arm_joint_cmd
ros2 topic info -v /arm_joint_cmd
```

确认 OCS2 不在 HOLD，或者重新执行：

```bash
ros2 topic pub --once /fsm_command std_msgs/msg/Int32 "{data: 3}"
```

夹爪方向反了：

当前工程按 `script_gripper_v4.5.py`：`1.0` 打开，`0.0` 闭合/保持。若 Isaac 中实际方向不一致，应先检查 Script Node 是否加载的是 `/home/gtk/isaac_ocs_project/data/ros2_log/scripts/夹爪调试/00整理版/POS控制/SCRIPTS NODE/script_gripper_v4.5.py`，并确认 Graph 连线是 `position_cmds -> positionCommand`。

夹爪按键有记录但 Isaac 中偶发不动作：

`keyboard_ocs2_gripper_teleop.py` 默认会把每次夹爪命令重复发布 3 次、间隔 0.03 s，以降低 Isaac ActionGraph 某一帧未消费到 ROS 消息的概率。需要更强重复发布时可加：

```bash
--gripper-command-repeats 5 --gripper-repeat-interval 0.02
```

注意：CSV/diagnostics 里的 `left_gripper_closed`、`right_gripper_closed` 是脚本发布的命令状态，不是 Isaac 关节实际反馈。若命令状态变化但夹爪不动，应检查 `/right_gripper_controller/commands` 是否有 Isaac 订阅者，以及 `Gripper_Control_Graph` 的 Script Node/Articulation Controller 连线。

演示回放姿态突然跳：

优先使用默认小步长重录，并观察 diagnostics 中的 `right_target_current_norm`、`right_orientation_error_deg` 和 `blocked_rotation_count`：

```bash
./game_control.sh teach
```

示教时出现 `[gate] blocked translation`：

这是安全门限在阻止 target 继续远离 current。终端会同时打印中文说明、当前 `target-current` 误差和推荐按键方向。优先暂停等待 current 追上；如果需要手动修正，按提示沿反方向小步移动；如果误差持续不收敛，按 `Z` 慢速恢复左臂、按 `X` 慢速恢复右臂。若出现 `[reach] right workspace clamp`，日志会列出 `offset_from_center`、`axis_limit`、`sphere_norm` 和 `hit_axis`，用来判断是 X/Y/Z 哪个方向碰到边界。

OCS2 零空间漂移抑制：

`/home/gtk/isaac_ocs_project/projects/ros2_ws/src/robot/config/ocs2/task.info` 当前关闭固定 `stateCost`，改用 controller 内的动态 null-space 正则。`q_ref` 在进入 OCS2 / reset MPC 时取当前关节姿态，之后默认跟随每帧有效 MPC policy 输出；`dynamicNullspaceCost.updateReferenceFromPolicy true` 用于保持示教操作的响应性。若需要临时强抑制漂移，可改为 `false`，但会明显降低可操作性。权重含义、当前配置和调参步骤见 [OCS2_DYNAMIC_NULLSPACE_REGULARIZATION.md](OCS2_DYNAMIC_NULLSPACE_REGULARIZATION.md)。

`Q` 后 Isaac Sim 没有自动复位：

```bash
ros2 service type /reset_simulation
ros2 service call /reset_simulation simulation_interfaces/srv/ResetSimulation "{scope: 255}"
./game_control.sh reset
```

如果服务不存在，确认 Isaac Sim 已启用 `isaacsim.ros2.sim_control`，并确认当前终端已 source `/home/gtk/isaac_ocs_project/projects/ros2_ws/install/setup.bash`。

`Q` 或 `Esc` 后没有完成复位的常见原因：

- Isaac Sim 未启用 `isaacsim.ros2.sim_control`，`/reset_simulation` 服务不存在。
- `simulation_interfaces` 没有安装或当前终端没有 source 正确 ROS 环境。
- Isaac reset 完成后 OCS2 target/current pose 尚未重新同步，导致 HOME 或 target 复位被后续控制覆盖。
- OCS2 控制器处于非 active 状态，或者 `/fsm_command` 没有被控制器接收。

手动复位：

```bash
source /home/gtk/isaac_ocs_project/projects/ros2_ws/install/setup.bash
ros2 service call /set_simulation_state simulation_interfaces/srv/SetSimulationState "{state: {state: 1}}"
ros2 service call /reset_simulation simulation_interfaces/srv/ResetSimulation "{scope: 255}"
ros2 topic pub --once /fsm_command std_msgs/msg/Int32 "{data: 1}"
```

如果手动 reset 后还停在错误 target，重新启动一次示教器并按 `Z` 或 `X` 慢速恢复对应手臂，让 target 从 current pose 重新初始化。

## 8. 文件说明

- `PROJECT_OVERVIEW.md`：SIM1 工程总纲和文档地图，建议作为入口阅读。
- `GAME_GUIDE.md`：本文件，说明启动、示教、回放、键位和常见问题。
- `TRACE_DATA_FORMAT.md`：原始示教、诊断 CSV、相机数据、清洗、失败标注、LeRobot v3.0/v2.1 转换格式。
- `keyboard_ocs2_gripper_teleop.py`：游戏式示教器，发布 OCS2 target，记录 CSV。
- `tools/camera_recorder.py`：独立 ROS2 相机 recorder，可由示教器 subprocess 启动。
- `replay_ocs2_target_trace.py`：OCS2 target CSV 回放器，用于轨迹演示。
- `replay_lerobot_episode.py`：LeRobot 关节轨迹直连回放器，用于数据/Action Graph 检查。
- `game_control.sh`：统一入口。
- `trace_data/raw/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS.csv`：默认示教轨迹文件。
- `trace_data/raw/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS/camera/`：原始相机视频、深度帧、timestamp 和 summary。
- `trace_data/raw/manual_ocs2_keyboard_trace_latest.csv`：最近一次成功示教轨迹链接，`demo` 默认优先使用它。
- `trace_data/cleaned/`：清洗后可回放轨迹和裁剪后相机数据。
- `trace_data/lerobot_v30/`：由清洗后数据转出的 LeRobotDataset v3.0 本地目录。
- `trace_data/lerobot_v21/`：由清洗后数据转出的 LeRobotDataset v2.1 本地目录。
- `trace_cleaning/scripts/clean_sim1_episode.py`：清洗 trace，并按同一保留区间裁剪相机数据。
- `trace_cleaning/scripts/process_sim1_episode.py`：raw episode 到 cleaned、v3.0、v2.1 的一键后处理。
- `trace_cleaning/scripts/convert_ocs2_trace_to_lerobot_v30.py`：清洗后数据到 LeRobotDataset v3.0 的转换脚本。
- `trace_cleaning/scripts/convert_ocs2_trace_to_lerobot_v21.py`：清洗后数据到 LeRobotDataset v2.1 的转换脚本。
- `simulation_interfaces`：ROS 2 Humble apt 包 `ros-humble-simulation-interfaces`，用于 Isaac Sim `/reset_simulation`。

## 9. 底盘运动与记录

底盘控制 topic：

```text
/sim1/base_diff_cmd
geometry_msgs/msg/Twist
```

Isaac Sim `Base_Drive_Graph` 当前实际使用的底盘 ScriptNode 源文件是：

```text
底盘差速驱动/script_node.py
```

若场景中换成调试版 ScriptNode：

```text
/home/gtk/isaac_ocs_project/data/ros2_log/scripts/底盘差速驱动/script_node_base_arc_debug_v2.py
```

则会额外发布：

```text
/sim1/base_drive_debug
std_msgs/msg/String
```

该消息是 JSON 字符串，只用于诊断和记录，不参与控制。

当前只使用 `linear.x` 和 `angular.z`：

```text
linear.x > 0                 前进
linear.x < 0                 后退
linear.x = 0, angular.z > 0  触发左转圆弧 90°
linear.x = 0, angular.z < 0  触发右转圆弧 90°
linear.x = 0, angular.z = 0  停止 / 解锁下一次转向
```

其中 `left90/right90` 的 `angular.z` 是触发信号，不是持续角速度命令。触发后 Isaac Sim `script_node.py` 在内部继续执行圆弧闭环；ROS2 侧记录的 `base_cmd_vx/base_cmd_wz` 只表示外部输入，实际轮子执行结果以 `/isaac_joint_states` 中的 `wheel_L/R_velocity` 和 `wheel_L/R_effort` 为准。

不要改成 `/cmd_vel`，不要使用 `linear.y`，不要恢复原地转弯。当前 R1 是 2 个主动轮 + 4 个 caster，使用圆弧转弯更稳定。

独立底盘运动脚本：

```bash
python3 base_drive_path_runner.py forward --speed 0.5 --duration 2.0
python3 base_drive_path_runner.py backward --speed 0.5 --duration 2.0
python3 base_drive_path_runner.py stop
python3 base_drive_path_runner.py left90 --trigger-wz 1.0 --wait 6.0
python3 base_drive_path_runner.py right90 --trigger-wz -1.0 --wait 6.0
python3 base_drive_path_runner.py path-demo --speed 0.5 --turn-wait 6.0
```

`path-demo` 当前按时间近似执行：前进 1m、左转 90°、前进 2m、右转 90°、前进 1m。脚本已保留 `--use-odom`、`--use-tf`、`--base-frame base_link` 参数给后续闭环扩展，第一版不依赖闭环里程计。

独立底盘记录脚本：

```bash
python3 base_drive_recorder.py --rate 50 --out-dir trace_data/base_raw
```

输出：

```text
trace_data/base_raw/base_drive_trace_YYYYmmdd_HHMMSS.csv
trace_data/base_raw/base_drive_trace_latest.csv
```

底盘记录字段语义：

```text
base_cmd_vx / base_cmd_wz
  记录 ROS2 外部输入，也就是 /sim1/base_diff_cmd 的原始 linear.x / angular.z。
  对 left_arc_90 / right_arc_90 来说，它们只表示触发命令，不表示 ScriptNode 内部闭环期间每帧的真实速度命令。

base_cmd_mode
  根据外部输入分类为 forward / backward / left_arc_90 / right_arc_90 / stop / manual。

base_left_wheel_cmd / base_right_wheel_cmd
  只在普通差速命令下按公式估算。
  left  = (vx - wz * L / 2) / r
  right = (vx + wz * L / 2) / r
  然后应用 script_node.py 中的 LEFT_SIGN / RIGHT_SIGN。
  如果 /sim1/base_drive_debug 有数据，则记录 ScriptNode JSON 中的 left_wheel_cmd / right_wheel_cmd。
  如果没有 debug 数据，对 left_arc_90 / right_arc_90 写空，因为真实 wheel velocityCommand 由 Isaac ScriptNode 内部闭环生成。

wheel_L_velocity / wheel_R_velocity / wheel_L_effort / wheel_R_effort
  来自 /isaac_joint_states，是底盘实际执行结果，分析运动质量时优先看这些字段。

base_arc_active / base_arc_target_yaw_deg / base_arc_error_deg
  如果 /sim1/base_drive_debug 有数据，则来自调试 JSON 的 arc_active / target_yaw_deg / error_yaw_deg。
  没有 debug 数据时保持为空。
```

不要为了填充 `base_arc_*` 或 `base_wheel_slip_hint` 修改当前已调通的底盘控制逻辑；只在 Isaac 端已经换成 debug 版 ScriptNode 时记录这些字段。

统一入口：

```bash
./game_control.sh base-record
./game_control.sh base-path-demo --base-speed 0.5 --base-turn-wait 6.0
./game_control.sh base-demo --base-speed 0.5 --base-turn-wait 6.0
```

`base-demo` 是 `base-path-demo` 的别名。记录频率可用：

```bash
./game_control.sh base-record --base-record-rate 50
```

与 teach 联合示教时，pygame 模式使用 `1/2/3` 互斥片选：`1` 控左臂，`2` 控右臂，`3` 控底盘。选中 `3` 后，方向键控制底盘前进、后退和左/右圆弧 90°；选中 `1/2` 后，W/A/S/D/R/F 与 U/O/I/K/J/L 控制对应 OCS2 target。

```bash
./game_control.sh teach --input-mode pygame --base-speed 0.25 --base-yaw-rate 1.0
```

底盘键位语义：

```text
先按 3 选中底盘
↑  linear.x = +base_speed
↓  linear.x = -base_speed
←  linear.x = 0, angular.z = +base_yaw_rate，触发左转圆弧 90°
→  linear.x = 0, angular.z = -base_yaw_rate，触发右转圆弧 90°
无方向键  linear.x = 0, angular.z = 0，停止 / 解锁
```

如果想临时关闭底盘方向键：

```bash
./game_control.sh teach --input-mode pygame --disable-base-motion
```
