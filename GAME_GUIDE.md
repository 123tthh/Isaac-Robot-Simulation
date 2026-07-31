# Isaac OCS 双臂夹爪操作指南

更新日期：2026-07-24

本文件是 `/home/gtk/isaac_ocs_project` 的当前操作入口。启动、构建和数据路径
以本文件为准；不要再从旧的 `/home/gtk/Trajectory`、`/home/gtk/ros2_ws`
或 `/home/gtk/ros2_log` 实体目录启动任务。

## 1. 当前运行基线

- Isaac Sim：5.1.0
- 宿主机 ROS 运行环境：ROS 2 Humble
- API/设计参考：本地 ROS 2 Rolling 文档
- 唯一运行场景：
  `/home/gtk/isaac_ocs_project/assets/scenes/scene.usd`
- ROS 2 工作区：
  `/home/gtk/isaac_ocs_project/projects/ros2_ws`
- SIM1 控制工程：
  `/home/gtk/isaac_ocs_project/projects/Trajectory/SIM1`
- ROS 日志：
  `/home/gtk/isaac_ocs_project/data/ros2_log`

以下旧路径仅作为观察期兼容链接保留：

```text
/home/gtk/Trajectory -> /home/gtk/isaac_ocs_project/projects/Trajectory
/home/gtk/ros2_ws    -> /home/gtk/isaac_ocs_project/projects/ros2_ws
/home/gtk/ros2_log   -> /home/gtk/isaac_ocs_project/data/ros2_log
```

新脚本、配置和说明文档必须使用工程内路径或环境变量，不应继续写死上述旧路径。

兼容性说明：Isaac Sim 5.1 本地文档列出的 ROS Bridge 支持版本是 Humble 和
Jazzy，因此本机运行时固定使用 Humble。Rolling 文档用于知识检索和接口设计
参考，不代表当前 Isaac ROS Bridge 运行在 Rolling。

## 2. 控制链路

双臂示教和回放：

```text
键盘或轨迹 CSV
  -> /left_target/stamped、/right_target/stamped
  -> ocs2_arm_controller
  -> /arm_joint_cmd
  -> Isaac Sim Arm_Control_Graph
  -> /World/Robot 双臂关节
```

夹爪：

```text
键盘或轨迹 CSV
  -> /left_gripper_controller/commands
  -> /right_gripper_controller/commands
  -> Gripper_Control_Graph
  -> script_gripper_v4.5.py
  -> positionCommand
```

PGIA v4.5 命令语义：

```text
cmd > 0   打开夹爪
cmd <= 0  闭合或保持夹爪
```

Script Node 应使用 `position_cmds -> positionCommand`。旧版
`effort_cmds -> effortCommand` 是 v4.1 Effort-PD 链路，不要和 v4.5 混用。

底盘：

```text
键盘或底盘脚本
  -> /sim1/base_diff_cmd
  -> Base_Drive_Graph
  -> 左右主动轮
```

## 3. 首次使用或代码变更后的检查

进入工程：

```bash
cd /home/gtk/isaac_ocs_project
```

检查场景、Isaac、GPU、ROS 包和 SIM1 入口：

```bash
./scripts/project_control_20260723.sh preflight
```

重新构建宿主机核心 ROS 包：

```bash
./scripts/project_control_20260723.sh build
```

构建整个工作区：

```bash
./scripts/project_control_20260723.sh build --all
```

宿主机和 Docker 必须使用各自的构建产物：

```text
宿主机：projects/ros2_ws/{build,install,log}
Docker：projects/ros2_ws/{build_docker,install_docker,log_docker}
```

不要在容器中 source 宿主机 `install`，也不要在宿主机 source
`install_docker`。

## 4. 宿主机标准启动顺序

必须按 A、B、C 的顺序启动。

### A. 启动 Isaac Sim

终端 A：

```bash
cd /home/gtk/isaac_ocs_project
./scripts/project_control_20260723.sh start-isaac
```

不带额外参数时，统一入口会自动加载：

```text
/home/gtk/isaac_ocs_project/assets/scenes/scene.usd
```

打开后在 Isaac Sim GUI 中按 **Play**。未按 Play 时，相机 topic、关节状态和
仿真控制服务不会正常工作。

场景应包含：

- `Arm_Control_Graph`
- `Gripper_Control_Graph`
- `State_Telemetry_Graph`
- `Camera_Publish_Graph`
- `Base_Drive_Graph`

### B. 启动 OCS2

终端 B：

```bash
cd /home/gtk/isaac_ocs_project
./scripts/project_control_20260723.sh launch-ocs2
```

该命令会启动 robot state publisher、ros2_control、OCS2 和默认 RViz2；
它不会替代 Isaac Sim，也不会替用户在 Isaac Sim 中按 Play。

等待控制器 active：

```bash
source /opt/ros/humble/setup.bash
source /home/gtk/isaac_ocs_project/projects/ros2_ws/install/setup.bash
ros2 control list_controllers
```

预期至少包括：

```text
joint_state_broadcaster active
ocs2_arm_controller active
```

### C. 检查并启动示教

终端 C：

```bash
cd /home/gtk/isaac_ocs_project
./scripts/project_control_20260723.sh sim1-check
```

最小终端示教：

```bash
./scripts/project_control_20260723.sh sim1-teach
```

推荐的完整采集命令：

```bash
./scripts/project_control_20260723.sh sim1-teach \
  --open-camera-grid \
  --input-mode pygame \
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

`--record-cameras` 负责保存 RGB+D 数据；`--open-camera-grid` 负责打开
2×3 相机监看窗口，两者相互独立。pygame 窗口必须保持焦点。

## 5. 游戏键位

选择控制对象：

```text
1  左臂
2  右臂
3  底盘
```

三者为互斥片选，可以随时切换。

选中左臂或右臂时：

```text
W/S  末端目标 +X / -X
A/D  末端目标 +Y / -Y
R/F  末端目标 +Z / -Z
I/K  pitch +/-
J/L  yaw +/-
U/O  roll +/-
```

夹爪：

```text
Space  切换当前选中手臂的夹爪开/合
V/B    左夹爪闭/开
N/M    右夹爪闭/开
```

选中底盘后：

```text
↑  前进
↓  后退
←  触发左转圆弧 90°
→  触发右转圆弧 90°
```

运行控制：

```text
+/-    调整末端平移步长
P      立即发布当前 target
H      显示帮助
Z      左臂慢速恢复初始姿态并重建 target
X      右臂慢速恢复初始姿态并重建 target
Q/Esc  结束记录并请求复位
```

默认示教步长已经收紧为 `0.003 m / 0.5°`。不要直接恢复旧的
`0.01 m / 2°` 连续输入；需要加速时先尝试：

```bash
./scripts/project_control_20260723.sh sim1-teach \
  --step 0.005 --rot-step-deg 0.5
```

## 6. 回放、相机和底盘工具

回放最近一次成功示教：

```bash
cd /home/gtk/isaac_ocs_project
./scripts/project_control_20260723.sh sim1-demo
```

慢速或循环回放：

```bash
./scripts/project_control_20260723.sh sim1-demo --speed 0.5
./scripts/project_control_20260723.sh sim1-demo --loop
```

指定轨迹：

```bash
TRACE=/absolute/path/to/trace.csv \
  ./scripts/project_control_20260723.sh sim1-demo
```

只打开六路相机监看器：

```bash
cd /home/gtk/isaac_ocs_project/projects/Trajectory/SIM1
./game_control.sh camera-grid
```

该命令不会启动 Isaac Sim、OCS2、RViz2 或键盘示教。看到
`pub=0 / no publisher` 时，先检查 Isaac Sim 是否已经打开当前场景并按 Play。

独立底盘记录和路径演示：

```bash
cd /home/gtk/isaac_ocs_project/projects/Trajectory/SIM1
./game_control.sh base-record --base-record-rate 50
./game_control.sh base-path-demo --base-speed 0.5 --base-turn-wait 6.0
```

`base-demo` 是 `base-path-demo` 的别名。底盘只使用 `linear.x` 和
`angular.z`，不要改成 `/cmd_vel` 或 `linear.y`。

LeRobot 关节直连回放：

```bash
cd /home/gtk/isaac_ocs_project/projects/Trajectory/SIM1
LEROBOT_NPZ_PATH=/absolute/path/to/episode.npz ./game_control.sh direct-demo
```

`direct-demo` 会直接发布 `/arm_joint_cmd`，不经过 OCS2。运行时不要让 OCS2
同时发布该 topic，否则两个 publisher 会争用同一 Action Graph 输入。

## 7. 数据输出

每次成功示教的原始数据位于：

```text
projects/Trajectory/SIM1/trace_data/raw/
```

主要文件：

```text
manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS.csv
manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS.diagnostics.csv
manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS.metadata.json
manual_ocs2_keyboard_trace_latest.csv
manual_ocs2_keyboard_trace_latest.diagnostics.csv
```

成功退出后，`game_control.sh` 默认执行：

```text
raw -> cleaned -> lerobot_v30
```

工程默认保留并生成 LeRobot v3.0：

```text
projects/Trajectory/SIM1/trace_data/lerobot_v30/
```

LeRobot v2.1 默认不生成，避免重新产生已经清理的冗余数据。确实需要时：

```bash
SIM1_GENERATE_LEROBOT_V21=1 \
  ./scripts/project_control_20260723.sh sim1-teach
```

手动处理已有 episode：

```bash
cd /home/gtk/isaac_ocs_project/projects/Trajectory/SIM1
./trace_cleaning/scripts/process_sim1_episode.py \
  trace_data/raw/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS.csv
```

ROS 与夹爪运行日志统一写入：

```text
/home/gtk/isaac_ocs_project/data/ros2_log
```

活动脚本使用 `ROS2_LOG_DIR`。临时改写日志位置时应设置该变量，不要修改脚本
为 `/home/gtk/ros2_log` 绝对路径。

## 8. Docker 使用

当前完整构建镜像：

```text
issac_ocs_docker:latest
issac_ocs_docker:20260724-full
image ID: 3fc6e462bfc8c269e77818342698b477b060adc18c041dde89067d4bf182582e
```

该历史镜像经容器内 `/isaac-sim/VERSION` 复核，实际继承
`5.0.0-rc.45`。它可用于重放原验证环境，但不是严格的 Isaac Sim 5.1
复现镜像。严格复现应按 `docker/README_DOCKER.md` 从
`nvcr.io/nvidia/isaac-sim:5.1.0` 构建新标签。

镜像内置 `/entrypoint.sh`；标准启动不再从宿主机覆盖 entrypoint。工程目录会
挂载到容器 `/workspace`。

启动或替换容器：

```bash
cd /home/gtk/isaac_ocs_project
bash docker/run_isaac_ocs_docker.sh --replace
docker exec -it demo_vla bash
```

容器内常用命令：

```bash
docker_env_check.sh
ocs_launch
sim1_check
sim1_teach
sim1_demo
sim1_reset
nvidia-smi
```

Docker launcher 已包含 GPU、host network、host IPC/PID、宿主 UID/GID、
4 GiB `/tmp` tmpfs 和 X11 配置。容器内规范路径为：

```text
/workspace/assets/scenes/scene.usd
/workspace/projects/ros2_ws/install_docker
/workspace/projects/Trajectory/SIM1
/workspace/data/ros2_log
```

完整重建：

```bash
cd /home/gtk/isaac_ocs_project
docker build --no-cache \
  -t issac_ocs_docker:20260724-full \
  -f docker/Dockerfile .
docker tag issac_ocs_docker:20260724-full issac_ocs_docker:latest
```

Docker 镜像不包含宿主机 `/home/gtk/isaac-sim-5.1` 安装目录。通常由宿主机
运行 Isaac Sim GUI，容器运行 ROS/OCS2 和数据工具，并通过 host
network/IPC 通信。

## 9. 复位和停止

优先使用统一复位入口：

```bash
cd /home/gtk/isaac_ocs_project
./scripts/project_control_20260723.sh sim1-reset
```

若退出后仍未恢复：

```bash
source /opt/ros/humble/setup.bash
source /home/gtk/isaac_ocs_project/projects/ros2_ws/install/setup.bash
ros2 service type /reset_simulation
ros2 service call /reset_simulation \
  simulation_interfaces/srv/ResetSimulation "{scope: 255}"
ros2 topic pub --once /fsm_command std_msgs/msg/Int32 "{data: 1}"
```

停止顺序：

1. 在示教窗口按 `Q` 或 `Esc`，等待 CSV 和后处理完成。
2. `Ctrl+C` 停止 OCS2 launch。
3. 在 Isaac Sim 中停止 Play，再关闭 Isaac Sim。
4. 如需停止并删除 Docker 容器，执行 `docker rm -f demo_vla`。

不要在轨迹、相机或夹爪日志仍在写入时强制结束进程。

## 10. 常见问题

### OCS2 当前位姿没有数据

```bash
ros2 topic echo --once /left_current_pose
ros2 topic echo --once /right_current_pose
ros2 control list_controllers
```

先确认 Isaac Sim 已按 Play、OCS2 launch 仍在运行、控制器为 active。

### `/arm_joint_cmd` 有数据但机器人不动

```bash
ros2 topic echo --once /fsm_command
ros2 topic hz /arm_joint_cmd
ros2 topic info -v /arm_joint_cmd
```

必要时重新激活：

```bash
ros2 topic pub --once /fsm_command std_msgs/msg/Int32 "{data: 3}"
```

### 夹爪方向错误或偶发不动作

确认场景 Script Node 使用工程内 v4.5 脚本，并检查：

```bash
ros2 topic info -v /left_gripper_controller/commands
ros2 topic info -v /right_gripper_controller/commands
```

命令状态不是夹爪关节实际反馈。topic 有消息但夹爪不动时，检查
`Gripper_Control_Graph` 的 Script Node、Articulation Controller 和
`positionCommand` 连线。

### 示教出现 gate/watchdog 告警

先松开移动键，等待 current 追上 target。误差持续不收敛时，按 `Z` 恢复
左臂或按 `X` 恢复右臂。不要直接提高 hard gate 或 watchdog 阈值掩盖问题。

### 当前场景的已知资产警告

`/World/Robot/base_link/visuals` 对
`615scene_20260723/configuration/513_physics.usd</visuals/base_link>` 的引用
仍可能报告无法解析。因此 `assets/scenes/615scene_20260723` 依赖目录暂时
不能删除。运行入口仍然只使用顶层 `assets/scenes/scene.usd`。

### Isaac Sim Script Editor 相机检查

打开 **Window → Script Editor**，执行：

```python
exec(open('/home/gtk/isaac_ocs_project/scripts/inspect_repair_615scene_camera_script_editor_20260724.py', encoding='utf-8').read())
```

脚本默认只读，报告写入：

```text
/home/gtk/isaac_ocs_project/reports/615scene_camera_script_editor_20260724.md
```

只有报告确认需要修复时，才按脚本说明启用写入模式；脚本会先备份 USD。

## 11. 关键文件与参考

工程文件：

- `project_manifest.yaml`：规范路径和运行基线
- `scripts/project_control_20260723.sh`：宿主机统一入口
- `projects/Trajectory/SIM1/game_control.sh`：SIM1 模式入口
- `projects/Trajectory/SIM1/TRACE_DATA_FORMAT.md`：轨迹字段和转换格式
- `docker/README_DOCKER.md`：Docker 构建与运行说明
- `docker/run_isaac_ocs_docker.sh`：Docker 标准启动器
- `reports/migration_finalization_20260724.md`：目录迁移、构建和验证结论

本指南依据以下本地文档：

- `/home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.md`
- `/home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_manipulation.md`
- `/home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_tf.md`
- `/home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_simulation_control.md`
- `/home/gtk/ai_docs/docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.md`
- `/home/gtk/ai_docs/docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.md`
