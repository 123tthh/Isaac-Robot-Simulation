> **Historical record / 历史记录**：本页描述其记录日期的状态；当前运行以[中文指南](../../../docs/getting-started.zh-CN.md) / [English guide](../../../docs/getting-started.md)为准。Historical failures and paths are not current release claims.

# Claude 接手说明：Trajectory / Isaac Sim / OCS2 / PGIA 夹爪控制

SIM1 工程总纲见 [PROJECT_OVERVIEW.md](PROJECT_OVERVIEW.md)。本文件是历史接手说明，优先用于理解夹爪控制背景；当前操作命令以 [GAME_GUIDE.md](GAME_GUIDE.md) 为准。

生成时间：2026-05-28  
最近更新：2026-05-29  
当前工作目录：`/home/gtk/teleoperation/sim1`

## 0. 给 Claude 的首要任务

当前夹爪控制已切到 **PGIA v4.5 位置控制模式**。请优先确认 Isaac Sim Action Graph 中左右 PGIA 夹爪 Script Node 均加载 v4.5，并且输出端口连接到 Articulation Controller 的 `positionCommand`。

当前明确的夹爪控制源文件是：

```text
/home/gtk/ros2_log/scripts/夹爪调试/00整理版/POS控制/SCRIPTS NODE/script_gripper_v4.5.py
```

该脚本被用于 Isaac Sim 的左右夹爪 Script Node，当前设计是 positionCommand 位置控制。不要把问题误判为 OCS2 手臂控制问题；目前用户要确认的是 **PGIA 夹爪开合/夹持控制图是否按 v4.5 接线**。

当前示教入口已改为终端键盘直接控制：终端 C 进入 `/home/gtk/teleoperation/sim1` 后执行 `./game_control.sh check`、`./game_control.sh teach`。鼠标切换夹爪选择和夹爪开合功能已从代码中移除，只使用键盘。输入 `q` 结束一次示教任务时，脚本会关闭双夹爪，先调用 Isaac Sim ROS2 Simulation Control 的 `/reset_simulation`，再发送 HOME FSM 并复位 OCS2 target。OCS2 `home_1` 已在 `/home/gtk/ros2_ws/src/robot/config/ros2_control/ocs2_controllers.yaml` 中改为双臂胸前折叠姿态，`home_2` 保留 14 关节全零姿态作为备用。

## 1. 本项目约束

本目录有 AGENTS.md 约束，开发 Isaac Sim 5.1.0 / ROS 2 Rolling 相关内容时必须先查本地文档，严禁猜 API。

注意：项目约束和本地文档路径使用 Rolling 文档，但当前机器实际 ROS 运行环境是 `ROS_DISTRO=humble`，`simulation_interfaces` 已安装为 Debian 包 `ros-humble-simulation-interfaces`，安装位置 `/opt/ros/humble`。

已参考的本地文档：

```text
https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.md
https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.md
https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_manipulation.md
https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_tf.md
https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_simulation_control.md
```

如果生成或修改 Python 脚本，文件顶部需要注释写明参考了哪些本地文档路径。

## 2. 目录概览

### 2.1 当前诊断/控制工程

目录：

```text
/home/gtk/teleoperation/sim1
```

关键文件：

```text
ALL_Graph.md                         Isaac Sim Action Graph 全量导出
graph/Gripper_Control_Graph.md       夹爪控制图导出/当前目标接线
scene_structure.md                    Isaac Sim Stage/Robot/Joint 结构导出
gripper_full_diagnosis.md/json        夹爪诊断导出
gripper_runtime_trace.csv             运行时 trace
diagnosis.py                          Isaac Sim 内诊断脚本
keyboard_ocs2_gripper_teleop.py       游戏式 OCS2 双臂末端示教
replay_ocs2_target_trace.py           OCS2 target 轨迹回放
replay_lerobot_episode.py             LeRobot 关节轨迹直连 Isaac 回放
script_gripper_v4.2.py                历史 PGIA 夹爪位置控制 Script Node 备份
game_control.sh                       teach/demo/direct-demo 统一入口
GAME_GUIDE.md                         游戏操作说明
manual_ocs2_keyboard_trace_*.csv      时间戳示教 CSV
manual_ocs2_keyboard_trace_latest.csv 最近一次成功示教软链接，demo 默认优先使用
```

### 2.2 Trajectory 数据与仿真工程

顶层目录：

```text
/home/gtk/teleoperation
```

关键子目录：

```text
/home/gtk/teleoperation/converted_lerobot/local/place_tray_middle
/home/gtk/teleoperation/SIM
/home/gtk/teleoperation/place_tray_middle
```

`converted_lerobot/local/place_tray_middle` 是已转换的 LeRobot 数据集，关键文件：

```text
data/chunk-000/file-000.parquet
meta/info.json
meta/stats.json
meta/tasks.parquet
meta/episodes/chunk-000/file-000.parquet
episode_000_rm_dual_joint_traj.npz
export_rm_dual_episode.py
videos/observation.image.camera_H/...
videos/observation.image.camera_L/...
videos/observation.image.camera_R/...
```

注意：上面是历史外部 converted_lerobot 数据集结构，不是当前 `~/teleoperation/sim1` 新采集流水线。当前 SIM1 示教数据先保留 raw，再写入 `trace_data/cleaned/`，并分别转换到 `trace_data/lerobot_v30/` 和 `trace_data/lerobot_v21/`；细节见 `PROJECT_OVERVIEW.md` 和 `TRACE_DATA_FORMAT.md`。

`episode_000_rm_dual_joint_traj.npz` 字段：

```text
t           (N,)      轨迹时间，50 Hz 重采样
q           (N, 14)   左右双臂 14 维关节角
gripper     (N, 2)    左右夹爪原始动作
joint_names (14,)     left_joint1..7, right_joint1..7
```

`/home/gtk/teleoperation/SIM` 是一个 ROS 2 包工程，包含 `r1_lerobot_sim`，用于 RViz/轨迹回放相关实验。当前夹爪问题主要不在这个目录。

## 3. Isaac Sim 场景与 Action Graph 链路

场景文件来自：

```text
/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd
```

Stage 机器人根路径：

```text
/World/Robot
```

Articulation Root 在导出中显示为：

```text
/World/Robot/base_link
```

双臂关节：

```text
left_joint1 .. left_joint7
right_joint1 .. right_joint7
```

PGIA 夹爪主动关节：

```text
left_PGIA_joint1
left_PGIA_joint2
right_PGIA_joint1
right_PGIA_joint2
```

PGIA 夹爪 link：

```text
left_PGIA_base_link
left_PGIA_link1
left_PGIA_link2
right_PGIA_base_link
right_PGIA_link1
right_PGIA_link2
```

### 3.1 Arm_Control_Graph

来自 `ALL_Graph.md`：

```text
Graph: /World/ActionGraphs/Arm_Control_Graph
ROS2SubscribeJointState topicName: arm_joint_cmd
ArticulationController robotPath/targetPrim: /World/Robot
```

数据流：

```text
OCS2 / replay_lerobot_episode.py
  -> /arm_joint_cmd  (sensor_msgs/JointState)
  -> ROS2SubscribeJointState
  -> IsaacArticulationController
  -> /World/Robot
```

注意：OCS2 闭环启动时也会发布 `/arm_joint_cmd`。直连回放 LeRobot 关节轨迹时不要同时让 OCS2 抢这个 topic。

### 3.2 Gripper_Control_Graph

来自 `Gripper_Control_Graph.md`：

```text
Graph: /World/ActionGraphs/Gripper_Control_Graph
sub_left_gripper topicName: left_gripper_controller/commands
sub_right_gripper topicName: right_gripper_controller/commands
messageName: Float64MultiArray
script_left_gripper / script_right_gripper: omni.graph.scriptnode.ScriptNode
artic_left_gripper / artic_right_gripper: isaacsim.core.nodes.IsaacArticulationController
robotPath: /World/Robot
```

数据流：

```text
/left_gripper_controller/commands  std_msgs/msg/Float64MultiArray
  -> sub_left_gripper.outputs:data
  -> script_left_gripper.inputs:input_double_array
  -> script_left_gripper.outputs:joint_names
  -> script_left_gripper.outputs:position_cmds
  -> artic_left_gripper.inputs:jointNames
  -> artic_left_gripper.inputs:positionCommand

/right_gripper_controller/commands 同理
```

核心点：**v4.5 必须连接 `position_cmds -> positionCommand`。旧的 `effort_cmds -> effortCommand` 是 v4.1 链路，需要断开。**

## 4. OCS2 / RViz / 游戏式示教链路

OCS2 启动命令通常是：

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch r1_description ocs2_isaac.launch.py
```

启动后检查：

```bash
ros2 control list_controllers
```

期望：

```text
joint_state_broadcaster active
ocs2_arm_controller active
```

OCS2 控制链路：

```text
RViz Interactive Marker 或 keyboard_ocs2_gripper_teleop.py
  -> /left_target/stamped, /right_target/stamped
  -> ocs2_arm_controller
  -> /arm_joint_cmd
  -> Isaac Arm_Control_Graph
```

FSM 激活 OCS2：

```bash
ros2 topic pub --once /fsm_command std_msgs/msg/Int32 "{data: 3}"
```

工程入口：

```bash
cd ~/teleoperation/sim1
source ~/ros2_ws/install/setup.bash
./game_control.sh check
./game_control.sh teach
./game_control.sh demo
```

`teach` 是键盘示教，发布 OCS2 target 并记录时间戳 CSV。  
`demo` 是回放最近一次成功示教 CSV 到 OCS2 target，仍走 OCS2。  
`direct-demo` 是 LeRobot 关节轨迹直连 Isaac，不走 OCS2。

`teach` 退出行为：

```text
按 q
  -> 记录 quit
  -> 发布双夹爪 close/hold
  -> 尝试调用 /reset_simulation simulation_interfaces/srv/ResetSimulation "{scope: 255}"
  -> /fsm_command 发布 HOME 指令，默认 data=1；HOME 使用 ocs2_controllers.yaml 的 home_1 折叠姿态
  -> 将 OCS2 target 重设为 current pose
```

当前已安装并验证：

```bash
sudo apt-get install -y ros-humble-simulation-interfaces
source ~/ros2_ws/install/setup.bash
ros2 pkg prefix simulation_interfaces
ros2 service type /reset_simulation
```

确认输出：

```text
/opt/ros/humble
simulation_interfaces/srv/ResetSimulation
```

## 5. 当前夹爪 Script Node v4.5 设计

真实源文件：

```text
/home/gtk/ros2_log/scripts/夹爪调试/00整理版/POS控制/SCRIPTS NODE/script_gripper_v4.5.py
```

命令语义：

```text
input_double_array[0] > 0   -> OPEN
input_double_array[0] <= 0  -> CLOSE / HOLD
```

因此外部发送：

```bash
ros2 topic pub --once /left_gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.0]}"
ros2 topic pub --once /left_gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0]}"
```

预期分别为打开、闭合。

v4.5 当前关键参数：

```text
BASE_GAP          0.0197
USD_LOWER         0.004
USD_UPPER         0.12
MIN_POS           0.005
MAX_POS           0.118
CLOSE_STEP        0.0005
OPEN_STEP         0.0008
VEL_THRESH        0.003
POS_STABLE        0.0006
CONTACT_CONFIRM   8
HOLD_PRELOAD      0.015
SLIP_VEL          0.02
INIT_CLOSE        True
```

状态机：

```text
INIT  -> 初始含托盘时自动闭合到 HOLD
OPEN  -> target 每帧增加到 MAX_POS
CLOSE -> target 每帧减少 CLOSE_STEP，直到接触或 MIN_POS 后进 HOLD
HOLD  -> target = hold_pos，USD Drive 保持
IDLE  -> target 保持不变
```

输出：

```python
db.outputs.joint_names = [c["joint1"], c["joint2"]]
db.outputs.position_cmds = [float(target), float(target)]
db.outputs.execOut = og.ExecutionAttributeState.ENABLED
```

## 6. 当前注意事项

用户已指定改为：

```text
/home/gtk/ros2_log/scripts/夹爪调试/00整理版/POS控制/SCRIPTS NODE/script_gripper_v4.5.py
```

已知关键点：

1. v4.5 输出 `position_cmds`，必须接到 `IsaacArticulationController.inputs:positionCommand`。
2. USD Drive 建议保持强位控：`stiffness=2000`、`damping=200`、`maxForce=500`。
3. `MAX_POS=0.118`，略低于 USD upper limit `0.12`；`MIN_POS=0.005`，略高于 lower limit `0.004`。
4. 接触检测基于速度趋零和位置稳定，不再依赖 ScriptNode 输出 effort。
5. 当前命令仍保持 `0/1` 兼容：`1.0=open`，`0.0=close/hold`。
6. v4.5 删除了 HOLD 状态下自动脱落检测，避免翻转/快速运动时速度瞬变误触发重新闭合。

## 7. 建议 Claude 优先排查/修改

### 7.1 先做最小可复现实验

在 Isaac Sim Play 状态下，直接发夹爪命令：

```bash
source ~/ros2_ws/install/setup.bash

ros2 topic pub --once /left_gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.0]}"
ros2 topic pub --once /right_gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.0]}"

ros2 topic pub --once /left_gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0]}"
ros2 topic pub --once /right_gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0]}"
```

同时看日志：

```bash
tail -f /home/gtk/ros2_log/left_gripper_v45.log
tail -f /home/gtk/ros2_log/right_gripper_v45.log
```

重点观察：

```text
state: OPEN/CLOSE/HOLD
target
j1/j2 pos
v1/v2
gap
cc contact counter
是否频繁 re-CLOSE slip
```

### 7.2 不要先改 OCS2

OCS2 只管手臂末端 target 到 `/arm_joint_cmd`。夹爪问题在 `Gripper_Control_Graph + script_gripper_v4.5.py + USD/PhysX joint config`。

### 7.3 当前修法方向

v4.5 是位置控制分层状态：

```text
INIT: 启动预热，按当前位置防跳；INIT_CLOSE=True 时自动闭合
OPEN: 位置目标逐帧增大到 MAX_POS
CLOSE: 位置目标逐帧减小到接触或 MIN_POS
HOLD: 接触后锁定 hold_pos，Drive 保持
```

ROS 输入协议：

```text
[1.0]      OPEN
[0.0]      CLOSE/HOLD
```

但要保持已有 `0/1` 兼容，因为 `keyboard_ocs2_gripper_teleop.py`、`replay_ocs2_target_trace.py` 和 `gripper_test_commands.sh` 已按该协议发送。

### 7.4 建议核对 Graph 与诊断输出

在 Isaac Sim 中核对：

```text
script_left_gripper.outputs:position_cmds  -> artic_left_gripper.inputs:positionCommand
script_right_gripper.outputs:position_cmds -> artic_right_gripper.inputs:positionCommand
script_*.outputs:joint_names               -> artic_*.inputs:jointNames
script_*.outputs:execOut                   -> artic_*.inputs:execIn
```

建议观察日志：

```text
raw command / state transition
target
j1/j2 pos
v1/v2
gap
contact confirm counter
```

### 7.5 建议核对方向

单独对每个 DOF 施加正/负小 effort，确认：

```text
left_PGIA_joint1: 正 effort 是打开还是闭合
left_PGIA_joint2: 正 effort 是打开还是闭合
right_PGIA_joint1: 正 effort 是打开还是闭合
right_PGIA_joint2: 正 effort 是打开还是闭合
```

如果方向不是脚本假设，需要对每个 joint 引入 `sign`：

```python
effort = sign * (Kp * (target - current) - Kd * velocity) + gravity_comp
```

不要假设左右夹爪和上下指节同号。

## 8. 当前交互/示教脚本与夹爪命令关系

`keyboard_ocs2_gripper_teleop.py`：

```text
Space：切换当前选中夹爪开合
Z/X：左夹爪闭/开
N/M：右夹爪闭/开
C/V：双夹爪闭/开
q：退出一次示教，触发夹爪/OCS2/Isaac reset 流程
```

鼠标切换功能已从 `keyboard_ocs2_gripper_teleop.py` 移除，不再启用 terminal mouse reporting，避免影响终端复制粘贴和键盘控制焦点。

发布 topic：

```text
/left_gripper_controller/commands
/right_gripper_controller/commands
```

消息：

```text
std_msgs/msg/Float64MultiArray
data: [1.0]  打开
data: [0.0]  闭合
```

`replay_ocs2_target_trace.py` 回放 CSV 时也会按记录发布上述夹爪命令。

## 9. 验证标准

修复后至少要通过以下验证：

1. Isaac Sim Play 后，直接命令 `[1.0]` 能稳定打开左右夹爪。
2. 直接命令 `[0.0]` 能稳定闭合，空夹不抖动。
3. 有托盘时，闭合后进入稳定 HOLD，托盘不明显滑落。
4. HOLD 后再发 `[1.0]` 能可靠释放并展开。
5. 左右夹爪表现一致，允许单独参数但不能一边反向。
6. `./game_control.sh teach` 中 Space / Z/X / N/M / C/V 操作夹爪与直接 ros2 topic pub 一致。
7. `./game_control.sh teach` 第二次及以后录制不会覆盖前一次文件，会生成新的时间戳 CSV。
8. 输入 `q` 后模型/仿真能通过 `/reset_simulation` 回到初始状态，RViz/OCS2 target 回到 current pose。
9. `./game_control.sh demo` 回放最近一次成功示教 CSV 时夹爪开合时序正确。

## 10. 可能需要读取的文件

优先读取：

```text
/home/gtk/ros2_log/scripts/夹爪调试/00整理版/POS控制/SCRIPTS NODE/script_gripper_v4.5.py
/home/gtk/teleoperation/sim1/graph/Gripper_Control_Graph.md
/home/gtk/teleoperation/sim1/graph/ALL_Graph.md
/home/gtk/teleoperation/sim1/graph/scene_structure.md
/home/gtk/teleoperation/sim1/graph/gripper_full_diagnosis.md
/home/gtk/teleoperation/sim1/graph/gripper_full_diagnosis.json
/home/gtk/teleoperation/sim1/graph/gripper_runtime_trace.csv
/home/gtk/teleoperation/sim1/keyboard_ocs2_gripper_teleop.py
/home/gtk/teleoperation/sim1/replay_ocs2_target_trace.py
/home/gtk/teleoperation/sim1/GAME_GUIDE.md
```

历史可参考：

```text
/home/gtk/ros2_log/scripts/夹爪调试/effort control/
/home/gtk/ros2_log/scripts/夹爪调试/effort+pos/
/home/gtk/ros2_log/scripts/夹爪调试/USD左右夹爪一致性检查/
/home/gtk/ros2_log/scripts/夹爪调试/右夹爪运行时排查/
```

## 11. 不要破坏的内容

1. 不要把 `Gripper_Control_Graph` 改成订阅别的 topic，除非同时更新所有外部脚本。
2. 不要把 `0/1` 命令语义反过来；当前工程约定 `1.0=open`、`0.0=close/hold`。
3. 不要修改 OCS2 手臂 topic `/arm_joint_cmd` 来解决夹爪问题。
4. 不要删除 `manual_ocs2_keyboard_trace_*.csv` 和 `manual_ocs2_keyboard_trace_latest.csv`，它们是当前示教记录和 demo 默认入口。
5. 如果生成新的 Python 脚本，顶部必须写本地文档参考路径。

## 12. 最短接手命令

```bash
cd /home/gtk/teleoperation/sim1
source ~/ros2_ws/install/setup.bash

# 检查 OCS2/Isaac topic 状态
./game_control.sh check

# 验证 Isaac Sim ROS2 Simulation Control
ros2 pkg prefix simulation_interfaces
ros2 service type /reset_simulation

# 单独测夹爪
./gripper_test_commands.sh both_open
./gripper_test_commands.sh both_close

# 或直接发命令
ros2 topic pub --once /left_gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.0]}"
ros2 topic pub --once /left_gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0]}"

# 看脚本日志
tail -f /home/gtk/ros2_log/left_gripper_v45.log
tail -f /home/gtk/ros2_log/right_gripper_v45.log
```

下面按**结构参数、几何开合、控制方向、当前 USD/脚本状态**总结。先说结论：你的 PGIA 夹爪本质是**双主动对称 prismatic 夹爪**，理想关系必须满足：

```text
joint1 = 上夹爪
joint2 = 下夹爪
j1 ≈ j2
real_gap = BASE_GAP + j1 + j2
```

其中 `BASE_GAP = 19.7 mm`，这是后续所有 gap 计算的基准。该几何模型在原位控脚本中已经明确记录。

------































## 1. 左右夹爪 USD 结构参数

| 项目              | 左夹爪                  | 右夹爪                  | 结论               |
| ----------------- | ----------------------- | ----------------------- | ------------------ |
| 上夹爪 joint      | `left_PGIA_joint1`      | `right_PGIA_joint1`     | joint1 = 上夹爪    |
| 下夹爪 joint      | `left_PGIA_joint2`      | `right_PGIA_joint2`     | joint2 = 下夹爪    |
| joint 类型        | `PhysicsPrismaticJoint` | `PhysicsPrismaticJoint` | 两侧一致           |
| joint axis        | `Y`                     | `Y`                     | 两侧一致           |
| joint1 lowerLimit | `0.004 m`               | `0.004 m`               | 一致               |
| joint1 upperLimit | `0.120 m`               | `0.120 m`               | 一致               |
| joint2 lowerLimit | `0.004 m`               | `0.004 m`               | 一致               |
| joint2 upperLimit | `0.120 m`               | `0.120 m`               | 一致               |
| base link mass    | `1.1631 kg`             | `1.1631 kg`             | 一致               |
| link1 mass        | `0.33762 kg`            | `0.33762 kg`            | 一致               |
| link2 mass        | `0.26010 kg`            | `0.26010 kg`            | 一致               |
| gravity           | 未禁用                  | 未禁用                  | 两侧 link 都受重力 |

USD 对比显示：左右 joint limit、axis、mass、rigid body、kinematic 状态基本一致；差异主要是左右镜像导致的 world translation / orient / bbox 差异，不是 joint limit 不一致。

------

## 2. 几何开合参数

| 参数                        | 数值                   | 含义                                | 计算结果                |
| --------------------------- | ---------------------- | ----------------------------------- | ----------------------- |
| `BASE_GAP`                  | `0.0197 m` / `19.7 mm` | j1=j2=0 时上下夹爪基准间距          | 19.7 mm                 |
| `USD_LOWER` / `CLOSE_LIMIT` | `0.004 m` / `4 mm`     | 单个 prismatic joint 最小行程       | 最小机械 gap = 27.7 mm  |
| `USD_UPPER`                 | `0.120 m` / `120 mm`   | USD 单 joint 最大行程               | 最大理论 gap = 259.7 mm |
| 旧工作 `OPEN_POS`           | `0.055 m` / `55 mm`    | 原位控/早期 effort 版本常用张开位置 | gap = 129.7 mm          |
| v3.8 `MAX_TRAVEL`           | `0.080 m` / `80 mm`    | v3.8 允许的单 joint 最大工作行程    | gap = 179.7 mm          |
| v3.8 `INIT_POS`             | `0.020 m` / `20 mm`    | 启动预定位                          | gap = 59.7 mm           |
| v3.8 `TRAY_MIN_HEIGHT`      | `0.060 m` / `60 mm`    | 托盘最小高度判据，不是命令          | 接近 60 mm 才允许 HOLD  |
| v3.8 `MIN_REAL_GAP`         | `0.052 m` / `52 mm`    | 空夹/薄托盘安全保护                 | 防止过度闭合            |

原始几何模型里给过典型值：`j1=j2=55mm` 时 gap≈129.7mm，`j1=j2=30.5mm` 时 gap≈80.7mm，`j1=j2=22mm` 时 gap≈63.7mm，`j1=j2=4mm` 时 gap≈27.7mm。

------

## 3. gap 公式表

| j1 行程 | j2 行程 | real_gap 计算      | real_gap |
| ------- | ------- | ------------------ | -------- |
| 4 mm    | 4 mm    | 19.7 + 4 + 4       | 27.7 mm  |
| 10 mm   | 10 mm   | 19.7 + 10 + 10     | 39.7 mm  |
| 20 mm   | 20 mm   | 19.7 + 20 + 20     | 59.7 mm  |
| 22 mm   | 22 mm   | 19.7 + 22 + 22     | 63.7 mm  |
| 30.5 mm | 30.5 mm | 19.7 + 30.5 + 30.5 | 80.7 mm  |
| 40 mm   | 40 mm   | 19.7 + 40 + 40     | 99.7 mm  |
| 55 mm   | 55 mm   | 19.7 + 55 + 55     | 129.7 mm |
| 80 mm   | 80 mm   | 19.7 + 80 + 80     | 179.7 mm |
| 120 mm  | 120 mm  | 19.7 + 120 + 120   | 259.7 mm |

所以：**托盘高度 h 要能被夹住，必须满足**

```text
BASE_GAP < h <= BASE_GAP + 2x
```

其中 `x` 是单个 joint 当前允许最大开口行程。

------

## 4. 实测右夹爪对称运动结果

你最后的 runtime probe 证明：**右夹爪模型本体可以对称运动**。直接写入相同 target 时，j1/j2 基本同步，link1/link2 的世界 z 坐标也随之反向变化。

| 目标 target | 实际 j1   | 实际 j2   | j1-j2    | 实测 gap  | 结论           |
| ----------- | --------- | --------- | -------- | --------- | -------------- |
| 4 mm        | 6.01 mm   | 6.49 mm   | -0.48 mm | 32.20 mm  | 可对称         |
| 10 mm       | 11.18 mm  | 11.62 mm  | -0.45 mm | 42.50 mm  | 可对称         |
| 20 mm       | 19.78 mm  | 20.19 mm  | -0.41 mm | 59.67 mm  | 接近 60mm 托盘 |
| 40 mm       | 36.99 mm  | 37.32 mm  | -0.32 mm | 94.01 mm  | 可对称         |
| 55 mm       | 49.90 mm  | 50.17 mm  | -0.26 mm | 119.77 mm | 可对称         |
| 80 mm       | 71.42 mm  | 71.57 mm  | -0.16 mm | 162.69 mm | 可对称         |
| 120 mm      | 105.83 mm | 105.83 mm | 0.00 mm  | 231.36 mm | 可对称         |

注意：实测值小于目标值，是因为 runtime 里还有物理约束、接触、限位、solver 误差；但对称性是成立的。

------

## 5. 控制方向与命令含义

| 项目             | 当前实测结论                                                 |
| ---------------- | ------------------------------------------------------------ |
| `joint1`         | 上夹爪                                                       |
| `joint2`         | 下夹爪                                                       |
| `effort > 0`     | 张开                                                         |
| `effort < 0`     | 闭合                                                         |
| `data[0] > 0`    | 张开命令                                                     |
| `data[0] <= 0`   | 闭合 / 夹紧命令                                              |
| `data[1]`        | 不是控制 joint1；在某些测试脚本里被临时用作单 joint 测试或 step 参数 |
| 正常同步要求     | `j1 ≈ j2`                                                    |
| 正常位置输出     | `position_cmds = [pos, pos]`                                 |
| 正常 effort 输出 | 同号输出，但不保证严格等行程                                 |

v3.7/v3.8 脚本已经明确记录：`joint1 = 上夹爪`，`joint2 = 下夹爪`，并且实测方向是 `effort < 0 = 闭合，effort > 0 = 张开`。

------

## 6. 当前 Drive / 控制模式特征

| 版本                  | 控制输出        | Drive stiffness | Drive damping | 特征                                                   | 问题                             |
| --------------------- | --------------- | --------------- | ------------- | ------------------------------------------------------ | -------------------------------- |
| v2.2 伪力控           | `position_cmds` | `20000`         | `800`         | 用 position target + Drive 力估算实现夹紧              | 不是纯力控，接触力是估算         |
| v3.x / v3.8 纯 effort | `effort_cmds`   | `0`             | `0`           | 直接施加 effort，试图靠同步力保证 j1/j2                | 竖直轴会下坠，且同力不等于同行程 |
| v4.0 混合位控         | `position_cmds` | `2000`          | `200`         | Drive 负责重力补偿和位置同步，逐帧减小 target 模拟力控 | 更适合现在的机械结构             |

v4.0 文件里也写得很清楚：PGIA prismatic 轴几乎竖直，纯 effort 的 `stiffness=0, damping=0` 会变成零刚度竖直滑块，容易下坠；更合理的是恢复 stiffness/damping，用 positionCommand 控制，同时通过逐帧移动 target 实现夹紧。

------

## 7. 最关键的工程判断

| 问题                                | 判断                                                         |
| ----------------------------------- | ------------------------------------------------------------ |
| 右侧 USD joint limit 是否比左侧大？ | 不是。左右 lower/upper limit 一致。                          |
| 右侧 link 是否完全坏掉？            | 不是。直接写 j1/j2 position 可以对称运动。                   |
| 右侧为什么 v3.8 下不同步？          | 因为 v3.8 是 effort 控制，同样的力不保证同样的位移。右侧重力/接触/初始姿态更容易放大非对称。 |
| 真正要保证上下等行程应该用什么？    | `positionCommand = [pos, pos]` 或 Mimic/机械耦合。           |
| effort 还能不能用？                 | 可以作为接触/夹持力调节，但不应作为唯一主控来保证几何同步。  |

最终建议参数基线：

| 参数                  | 推荐值                               |
| --------------------- | ------------------------------------ |
| `BASE_GAP`            | 19.7 mm                              |
| `USD lowerLimit`      | 4 mm                                 |
| `USD upperLimit`      | 120 mm                               |
| 常用闭合最小 gap      | 52–60 mm，取决于托盘高度             |
| 60mm 托盘预夹持 `pos` | 20 mm                                |
| 常用张开 `pos`        | 55 mm                                |
| 大开口 `pos`          | 80 mm                                |
| 最稳控制方式          | `positionCommand [pos, pos]`         |
| 力控辅助              | 读取 effort/velocity 判断接触后 HOLD |
