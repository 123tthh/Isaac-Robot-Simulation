# R1 机器人 OCS2 控制 + HIL-SERL 强化学习训练 完整技术指导

> 适用平台：R1 双臂轮式机器人 (RM-75 × 2 + PGIA 夹爪) · Isaac Sim 5.1 Docker · Isaac Lab 0.47.2  
> 任务：传送带托盘放置/抓取 (位置1 放置 → 位置2 抓取)

---

## 目录

1. [整体架构概览](#1-整体架构概览)
2. [任务一：OCS2 控制 Isaac Sim 中的机器人](#2-任务一ocs2-控制-isaac-sim-中的机器人)
3. [任务二：HIL-SERL 强化学习训练](#3-任务二hil-serl-强化学习训练)
4. [实施路线图与时间规划](#4-实施路线图与时间规划)

---

## 1. 整体架构概览

```
┌─────────────────────────────────────────────────────────┐
│                     系统全景                             │
│                                                         │
│  ┌──────────────┐    ROS2 Topics    ┌────────────────┐  │
│  │  OCS2 MPC    │ ←───────────────→ │  Isaac Sim 5.1 │  │
│  │  控制器      │   /joint_states   │  (Docker)      │  │
│  │  (任务一)    │   /joint_commands │  物理仿真引擎  │  │
│  └──────────────┘                   └───────┬────────┘  │
│                                             │           │
│  ┌──────────────┐    Gym Env API    ┌───────┴────────┐  │
│  │  HIL-SERL    │ ←───────────────→ │  Isaac Lab     │  │
│  │  RL 训练     │   obs/act/reward  │  Env Wrapper   │  │
│  │  (任务二)    │                   │  (RL 训练环境) │  │
│  └──────────────┘                   └────────────────┘  │
└─────────────────────────────────────────────────────────┘
```

两个任务的关系：任务一先通过 OCS2 MPC 控制器确定机器人的**工作站位** (位置1/位置2)、**可达工作空间**、和**避碰约束**，这些参数随后作为任务二 RL 训练环境的**workspace bounds**和**初始化配置**。

## 项目概述

轮式双臂机器人在 Isaac Sim 5.1 中进行物理仿真，执行托盘抓放任务。轮式双臂机器人（共享躯干 + 2×RM75-6FB）在 Isaac Sim 5.1 中进行物理仿真抓放托盘任务 ；机器人在驻车模式下使机械臂操作（机器人左右两个夹爪拿着托盘，

1、机器人基座运行到放置托盘位置1,左右手各自拿着一个托盘。放到传送带上（可行域内）（输入传送带只有一个入口，意味着左右机械臂留有足够空间，避免两个机械臂交叉/奇异）；

2、机器人轮式基座向左平移到输出传送带最佳位置，左右夹爪抓取新托盘（也是只有一个出口））；

3、夹爪电动（仅有开/关两个离散状态）：

完整工作循环： 

1. 阶段A：放置循环（输入传送带）  机器人驻车在输入传送带前  → 右臂先放托盘1到传送带（左臂等待）  → 左臂夹爪将托盘2给到右臂夹爪  → 左臂退出避让  → 右臂放托盘2到传送带  → 两个托盘都放完，左右手臂回到初始姿态 
2. 阶段B：平移到输出传送带  轮式底盘向左平移  → 到达输出传送带最佳取托盘位置 
3. 阶段C：抓取循环（输出传送带）  → 右臂夹爪抓取托盘1  → 左臂抓取右臂夹爪的托盘1  → 右臂抓取托盘2  → 返回阶段A 

**三阶段流水线：**

1. **阶段1 (当前)**: Isaac Sim + OCS2 仿真闭环 → 生成可行轨迹
2. **阶段2**: 轨迹数据 → 强化学习训练
3. **阶段3**: Sim2Real → 实机部署

**控制架构**: 修改版 OCS2 (基于 Optimal Control Toolbox for Switched Systems) 运行在 ROS 2 中，Isaac Sim 作为物理仿真器通过 ROS 2 bridge 通信。

**任务背景：**

1. 机器人基座运行到传送带入口(位置1),右夹爪拿着一个托盘,放到传送带上(可行域内),然后左夹爪将托盘给右夹爪,右夹爪将该托盘也放入传送带(输入传送带只有一个入口,需要该规划好空间,避免两个机械臂交叉/奇异 
2. 机器人轮式基座向左运动到输出传送带最佳位置(位置2),右夹爪抓取新托盘给左夹爪,然后右夹爪再抓取一个托盘,然后任务结束;
3. 夹爪具有开/关两个离散状态,机器人在驻车模式下使机械臂操作。

---

## 2. 任务一：OCS2 控制 Isaac Sim 中的机器人

### 2.1 你的代码库结构分析

从目录树看，你的 OCS2 控制栈的核心模块为：

| 模块路径 | 功能 |
|---------|------|
| `ocs2_arm_controller/` | ros2_control 插件，内含 FSM (Hold/Home/MoveJ/OCS2 四个状态) |
| `arms_target_manager/` | 管理末端目标位姿，支持 Interactive Marker 和 VR 输入 |
| `r1_robot_description/config/ocs2/` | OCS2 配置文件 (`task.info`, `fixed_base.info` 等) |
| `r1_moveit_config/` | MoveIt2 运动规划配置 (kinematics.yaml, joint_limits.yaml) |
| `topic_based_ros2_control/` | 通过 ROS2 topic 实现的 hardware_interface (仿真桥接关键) |

### 2.2 OCS2 ↔ Isaac Sim 桥接方案

OCS2 是一个基于模型预测控制 (MPC) 的优化控制框架。你已有的 `ocs2_arm_controller` 是一个 ros2_control 控制器插件。桥接的核心问题是：**如何让 OCS2 控制器的输出驱动 Isaac Sim 中的关节？**

推荐架构（基于你已有的 `topic_based_ros2_control`）：

```
Isaac Sim 5.1 (Docker)
  ├── ROS2 Bridge (OmniGraph)
  │     ├── 发布: /joint_states (sensor_msgs/JointState)
  │     └── 订阅: /joint_commands (各控制器输出)
  │
ROS2 (宿主机 or 同一容器)
  ├── topic_based_ros2_control (hardware_interface)
  │     ├── 从 /joint_states 读取当前关节状态
  │     └── 将控制器输出写入 /joint_commands
  ├── ocs2_arm_controller (controller plugin)
  │     └── FSM: StateOCS2 → 调用 OCS2 MPC 求解
  └── arms_target_manager
        └── 发布末端目标位姿给 OCS2 reference
```

**具体步骤：**

**Step 1 — Isaac Sim 端 ROS2 Bridge 配置**

在 Isaac Sim 中使用 OmniGraph 创建 ROS2 Bridge 节点：

```python
# 在 Isaac Sim Script Editor 中运行
import omni.graph.core as og

# 创建 Action Graph
keys = og.Controller.Keys
(graph, nodes, _, _) = og.Controller.edit(
    {"graph_path": "/World/ActionGraph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
            # 发布关节状态
            ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
            # 订阅关节指令
            ("SubscribeJointState", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),
            # 将指令写入 Articulation
            ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
            ("SubscribeJointState.outputs:execOut", "ArticulationController.inputs:execIn"),
            # 连接关节名、位置等
            ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
            ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
        ],
    },
)
```

关键参数配置：
- `PublishJointState` 的 `targetPrim` → 指向 R1 的 Articulation root (`/World/R1`)
- `ArticulationController` 的 `targetPrim` → 同上
- Topic 名称与 OCS2 控制栈匹配（检查你的 `ros2_controllers.yaml`）

------

### 第一张图：双臂连续控制核心 (`Arm_Control_Graph`)

**目标：** 接收 ROS 2 侧发送的 `/arm_cmd`（包含 14 个关节的目标位置），驱动物理引擎。

1. **新建节点：**
   - `On Playback Tick` (Trigger)
   - `ROS2 Context` (提供 ROS 环境)
   - `ROS2 Subscribe Joint State`
   - `Articulation Controller`
2. **核心参数配置：**
   - 点击 `ROS2 Subscribe Joint State` 节点，在属性面板设置 `Topic Name` 为 `/arm_cmd`。**取消勾选** `Enable Velocities` 和 `Enable Efforts`（我们走纯位置控制闭环）。
   - 点击 `Articulation Controller` 节点，在属性面板将 `Target Type` 设为 `Position`。勾选 `usePath`，并在 `robotPath` 填入 `/World/Robot`。
3. **执行连线 (Wiring)：**
   - `On Playback Tick` [Tick] $\rightarrow$ `ROS2 Subscribe Joint State` [Exec In]
   - `ROS2 Context` [Context] $\rightarrow$ `ROS2 Subscribe Joint State` [Context]
   - `ROS2 Subscribe Joint State` [Exec Out] $\rightarrow$ `Articulation Controller` [Exec In]
   - `ROS2 Subscribe Joint State` [Joint Names] $\rightarrow$ `Articulation Controller` [Joint Names]
   - `ROS2 Subscribe Joint State` [Position Commands] $\rightarrow$ `Articulation Controller` [Position Commands]

------

### 第二张图：离散电爪控制映射 (`Gripper_Control_Graph`)

**目标：** 拦截 ROS 2 发来的 Bool 信号，转换为物理关节位移。这是防止 Type Mismatch 报错的关键。（以左臂为例，右臂照做即可）

1. **新建节点：**
   - `On Playback Tick`
   - `ROS2 Subscribe Bool`
   - `OgnScript` (Python 脚本节点)
   - `Articulation Controller`
2. **OgnScript 核心转换层编写：**
   - 选中 `OgnScript`，在属性面板中点击 `Edit` 编辑引脚 (Pins)。
   - **添加 Input：** 命名为 `cmd_in`，类型设为 `bool`。
   - **添加 Outputs：**
     - `joint_names`，类型设为 `token[]` (字符串数组)
     - `position_cmds`，类型设为 `double[]` (双精度数组)
   - **填入底层转换代码（在节点的 Script 编辑框中粘贴）：**

Python

```
def compute(db):
    cmd = db.inputs.cmd_in
    
    # ⚠️ 必须核对你的 URDF，确认这两个物理关节的真实名称和行程范围！
    open_pos = 0.04  # 假设张开行程为 4cm
    close_pos = 0.00 # 假设闭合为 0cm
    
    # PGIA 夹爪的左右指节名称
    db.outputs.joint_names = ["left_PGIA_link1", "left_PGIA_link2"]
    
    # 布尔值转连续张量
    pos = open_pos if cmd else close_pos
    db.outputs.position_cmds = [pos, pos]
    
    return True
```

1. **执行连线 (Wiring)：**
   - `On Playback Tick` [Tick] $\rightarrow$ `ROS2 Subscribe Bool` [Exec In]
   - `ROS2 Subscribe Bool` [Exec Out] $\rightarrow$ `OgnScript` [Exec In]
   - `OgnScript` [Exec Out] $\rightarrow$ `Articulation Controller` [Exec In]
   - `ROS2 Subscribe Bool` [Data] $\rightarrow$ `OgnScript` [cmd_in]
   - `OgnScript` [joint_names] $\rightarrow$ `Articulation Controller` [Joint Names]
   - `OgnScript` [position_cmds] $\rightarrow$ `Articulation Controller` [Position Commands]
   - *(注意 `Articulation Controller` 同样要指向 `/World/Robot`，并设为 Position 模式)*

------

### 第三张图：机器人状态回传 (`State_Telemetry_Graph`)

**目标：** 实时向 ROS 2 状态机回传真实的物理状态，支撑 RL 的末端误差计算。

1. **新建节点：**
   - `On Playback Tick`
   - `Isaac Read Simulation Time` (获取精准仿真时间戳)
   - `Isaac Read Articulation State` (Target 指向 `/World/Robot`)
   - `ROS2 Publish Joint State`
   - `ROS2 Publish Transform Tree` (发布 TF，状态机查末端坐标全靠它)
2. **执行连线 (Wiring)：**
   - `On Playback Tick` [Tick] $\rightarrow$ `ROS2 Publish Joint State` [Exec In] 以及 `ROS2 Publish Transform Tree` [Exec In] (一托二)
   - `Isaac Read Simulation Time` [System Time] $\rightarrow$ 两者的 `TimeStamp` 引脚
   - `Isaac Read Articulation State` [Joint Names] $\rightarrow$ `ROS2 Publish Joint State` [Joint Names]
   - `Isaac Read Articulation State` [Joint Positions] $\rightarrow$ `ROS2 Publish Joint State` [Joint Positions]
   - `Isaac Read Articulation State` [Joint Velocities] $\rightarrow$ `ROS2 Publish Joint State` [Joint Velocities]
   - 对于 `ROS2 Publish Transform Tree`，在属性面板中配置 `targetPrims`，勾选整个 `/World/Robot`（或者只勾选你需要追踪坐标的 `base_link` 和左右手的 `flange_link` 以节省通信带宽）。

------

**Step 2 — topic_based_ros2_control 配置**

你的 `topic_based_ros2_control` 已经实现了通过 ROS2 topic 收发关节状态的 hardware_interface。需要确认 `ros2_controllers.yaml` 中的配置：

```yaml
# r1_robot_description/config/ros2_control/ros2_controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz，与 Isaac Sim physics step 匹配

    # OCS2 控制器 - 右臂
    right_arm_ocs2_controller:
      type: ocs2_arm_controller/Ocs2ArmController
    
    # OCS2 控制器 - 左臂  
    left_arm_ocs2_controller:
      type: ocs2_arm_controller/Ocs2ArmController

    # 夹爪控制器
    right_gripper_controller:
      type: adaptive_gripper_controller/AdaptiveGripperController
    left_gripper_controller:
      type: adaptive_gripper_controller/AdaptiveGripperController
```

**Step 3 — 启动顺序**

```bash
# Terminal 1: 启动 Isaac Sim (Docker)
docker exec -it isaac-sim-container bash
cd /isaac-sim && ./isaac-sim.sh --allow-root

# Terminal 2: 启动 ros2_control + OCS2
ros2 launch r1_robot_description demo.launch.py \
    use_sim:=true \
    use_ocs2:=true

# Terminal 3: 启动 target manager (交互式目标设置)
ros2 launch arms_target_manager ocs2_arm_target_manager.launch.py
```

### 2.3 确定工作站位与工作空间

这是任务一的**核心产出**，也是后续 RL 训练的基础。

**2.3.1 位置1（传送带入口 — 放置站位）**

目标：找到基座位置使得右臂能够稳定地将托盘放置到传送带入口。

方法：利用 OCS2 的 MPC 求解器，在 Isaac Sim 中系统性地扫描可行空间。

```python
"""
workspace_exploration.py
在 Isaac Sim standalone 模式下运行，扫描工作空间
"""
import numpy as np
from scipy.spatial.transform import Rotation as R

# ---- 机器人参数 (来自你的 background 文档) ----
# 相机到基座变换矩阵
T_cam_to_base = np.array([
    [0.02611044, -0.29522803, 0.95506998, 0.30146924],
    [-0.99671237, -0.08099104, 0.00221321, 0.01238448],
    [0.07669871, -0.95198785, -0.29637214, 1.40424101],
    [0, 0, 0, 1]
])

# RM-75 臂参数
ARM_DOF = 7
JOINT_LIMITS = {  # 从 joint_limits.yaml 获取，此处为示例
    'joint1': (-3.1, 3.1),
    'joint2': (-2.268, 2.268),
    'joint3': (-3.1, 3.1),
    'joint4': (-2.268, 2.268),
    'joint5': (-3.1, 3.1),
    'joint6': (-2.268, 2.268),
    'joint7': (-3.1, 3.1),
}

# 传送带入口位置 (在世界坐标系下，需要你在 Isaac Sim 中测量)
CONVEYOR_INLET_POS = np.array([1.0, 0.0, 0.85])  # 示例值，需替换

def compute_workspace_reachability(base_pose_candidates, target_pose):
    """
    对每个基座候选位置，检查末端是否能到达目标位姿
    返回可行的基座位置列表
    """
    feasible = []
    for base_xy_theta in base_pose_candidates:
        x, y, theta = base_xy_theta
        # 将目标从世界坐标转换到基座坐标
        T_world_to_base = np.eye(4)
        T_world_to_base[:2, :2] = R.from_euler('z', theta).as_matrix()[:2, :2]
        T_world_to_base[:3, 3] = [x, y, 0]
        T_base_inv = np.linalg.inv(T_world_to_base)
        
        target_in_base = T_base_inv @ np.append(target_pose, 1.0)
        
        # 检查是否在臂的可达范围内
        # RM-75 的最大臂展约 0.75m
        reach = np.linalg.norm(target_in_base[:3])
        if 0.2 < reach < 0.70:  # 留余量，避免奇异
            feasible.append({
                'base_pose': base_xy_theta,
                'target_in_base': target_in_base[:3],
                'reach_ratio': reach / 0.75,
            })
    return feasible

# 生成基座候选位置网格
candidates = []
for x in np.arange(0.2, 1.5, 0.05):
    for y in np.arange(-0.5, 0.5, 0.05):
        for theta in np.arange(-np.pi/4, np.pi/4, np.pi/12):
            candidates.append((x, y, theta))

results = compute_workspace_reachability(candidates, CONVEYOR_INLET_POS)
print(f"可行基座位置数: {len(results)}/{len(candidates)}")

# 按 reach_ratio 排序，选择最优 (reach_ratio 在 0.5 附近最佳)
results.sort(key=lambda r: abs(r['reach_ratio'] - 0.5))
print(f"最优基座位置: {results[0]}")
```

**2.3.2 双臂交叉/奇异避免策略**

你的任务描述中提到"左夹爪将托盘给右夹爪"这一交接动作，需要特别注意：

```
时序规划 (位置1 - 放置):
  Phase 1: 右臂持托盘 → 放到传送带       左臂持托盘，waiting
  Phase 2: 右臂退回到 safe pose          左臂不动
  Phase 3: 左臂将托盘递到交接区           右臂移到交接区
  Phase 4: 右臂接过托盘                  左臂退回 safe pose
  Phase 5: 右臂将第二个托盘放到传送带     左臂 idle

关键约束:
  - 交接区 (handover zone) 应设在机器人正前方, 两臂都容易到达
  - 永远只有一个臂在运动，另一个保持 safe pose (简化避碰)
  - 或: 使用 OCS2 的约束功能设置双臂最小距离约束
```

在 OCS2 的 `task.info` 中可以设置自碰撞约束：

```
; r1_robot_description/config/ocs2/task.info (关键段)
selfCollision
{
  ; 设置双臂最小安全距离
  minimumDistance  0.10  ; 10cm

  ; 碰撞对 (左臂 link vs 右臂 link)
  collisionObjectPairs
  {
    [0] "left_Link4, right_Link4"
    [1] "left_Link6, right_Link6"
    [2] "left_gripper, right_gripper"
  }
}
```

**2.3.3 workspace bounds 确定流程**

这一步的输出将直接用于任务二的 RL 环境配置：

```python
# 最终产出: workspace_config.yaml
workspace_config = {
    'position_1': {  # 传送带入口 - 放置站位
        'base_pose': [x1, y1, theta1],  # 基座位置
        'right_arm_workspace': {
            'x_range': [x_min, x_max],
            'y_range': [y_min, y_max], 
            'z_range': [z_min, z_max],
        },
        'left_arm_workspace': {
            'x_range': [x_min, x_max],
            'y_range': [y_min, y_max],
            'z_range': [z_min, z_max],
        },
        'handover_zone': {
            'center': [hx, hy, hz],
            'radius': 0.15,
        },
    },
    'position_2': {  # 传送带出口 - 抓取站位
        'base_pose': [x2, y2, theta2],
        'right_arm_workspace': { ... },
        'left_arm_workspace': { ... },
    },
    'conveyor_height': 0.85,
    'tray_dimensions': [0.3, 0.2, 0.05],  # L, W, H
}
```

### 2.4 OCS2 控制验证检查清单

在进入任务二之前，你应该能够做到：

- [ ] Isaac Sim 中的 R1 机器人能通过 ROS2 Bridge 接收关节指令
- [ ] OCS2 控制器能驱动右臂到达传送带入口上方
- [ ] OCS2 控制器能驱动左臂到达交接区
- [ ] 两条臂单独运动时没有自碰撞
- [ ] 确定了位置1和位置2的基座坐标
- [ ] 记录了两个位置下各臂的可达工作空间边界
- [ ] 夹爪开/关能通过 gripper_controller 正常触发

---

## 3. 任务二：HIL-SERL 强化学习训练

### 3.1 HIL-SERL 方法核心原理

HIL-SERL 的训练流程可以分为 4 个阶段：

```
阶段 0: 数据准备
  ├── 采集成功/失败样本 → 训练二分类 Reward Classifier
  └── 采集 20~30 条人工 demo 轨迹 → 存入 Demo Buffer

阶段 1: 在线 RL 训练 (Human-in-the-Loop)
  ├── Actor 进程: 执行当前策略, 人类可随时通过遥操作介入纠正
  ├── Learner 进程: 从 Demo Buffer + RL Buffer 均匀采样, 用 RLPD 更新策略
  └── 人类介入率随训练进行逐渐降低

阶段 2: 收敛
  ├── 成功率 → 100%, 介入率 → 0%
  └── 通常 1~2.5 小时 (实物) / 仿真更快

阶段 3: 评估
  └── 无人类介入, 连续 100 episode 测试成功率
```

**核心算法组件：**

| 组件 | 说明 |
|------|------|
| **RLPD** (RL with Prior Data) | 基于 SAC 的 off-policy 算法，从 demo buffer + online buffer 各 50% 采样 |
| **Pretrained Visual Backbone** | ResNet-10 (ImageNet 预训练)，冻结参数，提取图像特征 |
| **Reward Classifier** | 二分类器 (成功/失败)，用少量标注数据训练，作为稀疏奖励信号 |
| **Grasp Critic (DQN)** | 离散夹爪控制的专用 Q 网络，与连续控制分离训练 |
| **Human Intervention** | 通过 SpaceMouse/手柄遥操作，随时覆盖策略输出 |

### 3.2 Observation 空间设计

结合你的机器人配置（双腕部相机 + 头部相机）和背景文档中的观测定义：

```python
observation_space = {
    # ---- 视觉观测 ----
    'wrist_cam_right':  (128, 128, 3),  # 右腕部 RGB
    'wrist_cam_left':   (128, 128, 3),  # 左腕部 RGB
    'head_cam':         (128, 128, 3),  # 头部全局 RGB
    
    # ---- 本体感受 (14 维) ----
    'joint_positions':  (7,),   # 激活臂 7 关节位置 q
    'joint_velocities': (7,),   # 激活臂 7 关节速度 dq
    
    # ---- 末端追踪误差 (6 维) ----  
    'ee_pos_error':     (3,),   # 末端位置误差 ΔP
    'ee_rot_error':     (3,),   # 末端姿态误差 ΔR (axis-angle)
    
    # ---- 动作历史 (21 维) ----
    'action_history':   (3, 7), # 过去 3 帧的 7 维关节速度增量
    
    # ---- 夹爪状态 ----
    'gripper_state':    (1,),   # 0=关, 1=开 (离散)
}
```

**HIL-SERL 中图像处理流程：**
```
Raw Image (480×640×3)
  → Crop (聚焦工作区域)
  → Resize (128×128)
  → Normalize ([0,1])
  → ResNet-10 Backbone (冻结)
  → Feature Vector (512-d)
  → 与 proprioception 拼接
  → Actor/Critic Network
```

### 3.3 Action 空间设计

```python
action_space = {
    # 连续动作: 末端执行器的增量控制
    'ee_delta': (6,),  # [Δx, Δy, Δz, Δroll, Δpitch, Δyaw]
    
    # 离散动作: 夹爪控制 (由独立的 Grasp Critic DQN 控制)
    'gripper': Discrete(2),  # 0=关, 1=开
}
# 动作范围
ee_delta_bounds = {
    'position': [-0.05, 0.05],   # 米, 每步最大位移
    'rotation': [-0.10, 0.10],   # rad, 每步最大旋转
}
```

### 3.4 Reward 设计

HIL-SERL 使用**学习到的 binary reward classifier** 作为稀疏奖励，你需要自定义训练该分类器。

```python
"""
reward_classifier.py
训练二分类 reward classifier
"""
import torch
import torch.nn as nn
from torchvision import models, transforms

class RewardClassifier(nn.Module):
    """
    输入: 末端相机图像 (128×128×3)
    输出: P(success) ∈ [0, 1]
    """
    def __init__(self):
        super().__init__()
        # 使用与策略相同的视觉 backbone
        resnet = models.resnet18(pretrained=True)
        self.features = nn.Sequential(*list(resnet.children())[:-1])
        self.classifier = nn.Sequential(
            nn.Linear(512, 128),
            nn.ReLU(),
            nn.Dropout(0.3),
            nn.Linear(128, 1),
            nn.Sigmoid(),
        )
    
    def forward(self, img):
        feat = self.features(img).squeeze(-1).squeeze(-1)
        return self.classifier(feat)

# ---- 数据采集方案 ----
# 成功样本: 托盘正确放置在传送带上的最后一帧画面
# 失败样本: 托盘掉落 / 放偏 / 碰撞 等画面
# 每类采集约 50~100 张图像即可

# ---- 判定逻辑 ----
def compute_reward(classifier, observation):
    """
    稀疏奖励: 仅在 episode 末尾给出
    """
    img = observation['wrist_cam_right']
    with torch.no_grad():
        success_prob = classifier(preprocess(img))
    
    if success_prob > 0.8:  # 阈值
        return 1.0   # 成功
    else:
        return 0.0   # 失败
```

**针对你的托盘放置/抓取任务的补充奖励 (可选，加速训练)：**

```python
def shaped_reward(obs, action, next_obs, done):
    """
    可选的 shaped reward, 与 sparse classifier reward 配合使用
    """
    reward = 0.0
    
    # 1. 稀疏成功奖励 (来自 classifier)
    if done:
        reward += classifier_reward(next_obs) * 10.0
    
    # 2. 距离奖励 (鼓励末端靠近目标)
    ee_pos = next_obs['ee_position']
    target_pos = next_obs['target_position']  # 传送带放置点
    dist = np.linalg.norm(ee_pos - target_pos)
    reward += -0.1 * dist  # 距离惩罚
    
    # 3. 动作平滑惩罚
    reward += -0.01 * np.linalg.norm(action['ee_delta'])
    
    # 4. 夹爪不当使用惩罚
    # 如果在空中松开夹爪 → 惩罚
    if action['gripper'] == 1 and ee_pos[2] > conveyor_height + 0.1:
        reward += -1.0
    
    return reward
```

### 3.5 Isaac Lab 环境搭建

在 Isaac Lab 中创建你的自定义任务环境：

```
my_tray_task/
├── __init__.py
├── tray_place_env_cfg.py      # 环境配置
├── tray_place_env.py           # 环境主类
├── reward_classifier/
│   ├── model.py
│   └── weights/
│       └── classifier.pth
└── config/
    └── workspace_config.yaml   # 来自任务一的产出
```

**环境配置文件核心结构：**

```python
"""
tray_place_env_cfg.py
Isaac Lab 环境配置
"""
import isaaclab.envs.mdp as mdp
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import (
    EventTermCfg,
    ObservationGroupCfg,
    ObservationTermCfg,
    RewardTermCfg,
    TerminationTermCfg,
    SceneEntityCfg,
)
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.assets import ArticulationCfg, RigidObjectCfg
from isaaclab.sensors import CameraCfg

class TrayPlaceSceneCfg(InteractiveSceneCfg):
    """场景配置: 机器人 + 传送带 + 托盘"""
    
    # 机器人
    robot = ArticulationCfg(
        prim_path="/World/R1",
        spawn=None,  # 使用已有的 USD 场景
        actuators={
            "right_arm": ImplicitActuatorCfg(
                joint_names_expr=["right_joint[1-7]"],
                stiffness=100.0,
                damping=10.0,
            ),
            "left_arm": ImplicitActuatorCfg(
                joint_names_expr=["left_joint[1-7]"],
                stiffness=100.0,
                damping=10.0,
            ),
        },
    )
    
    # 托盘 (刚体)
    tray = RigidObjectCfg(
        prim_path="/World/Tray",
        spawn=UsdFileCfg(usd_path="path/to/tray.usd"),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.5, 0.0, 1.0),  # 初始在夹爪中
        ),
    )
    
    # 相机
    wrist_cam_right = CameraCfg(
        prim_path="/World/R1/right_wrist_camera",
        update_period=0.1,  # 10 Hz
        height=128,
        width=128,
    )
    wrist_cam_left = CameraCfg(
        prim_path="/World/R1/left_wrist_camera",
        update_period=0.1,
        height=128,
        width=128,
    )
    head_cam = CameraCfg(
        prim_path="/World/R1/head_camera",
        update_period=0.1,
        height=128,
        width=128,
    )


class TrayPlaceEnvCfg(ManagerBasedRLEnvCfg):
    """RL 环境配置"""
    
    scene = TrayPlaceSceneCfg(num_envs=1, env_spacing=5.0)
    
    # 观测
    observations = {
        "policy": ObservationGroupCfg(
            concatenate_terms=False,  # HIL-SERL 需要分别处理图像和状态
            terms={
                "joint_pos": ObservationTermCfg(func=mdp.joint_pos_rel),
                "joint_vel": ObservationTermCfg(func=mdp.joint_vel_rel),
                "ee_pos_error": ObservationTermCfg(func=ee_tracking_error_pos),
                "ee_rot_error": ObservationTermCfg(func=ee_tracking_error_rot),
                "wrist_cam_right": ObservationTermCfg(
                    func=mdp.image,
                    params={"sensor_cfg": SceneEntityCfg("wrist_cam_right")},
                ),
            },
        ),
    }
    
    # 动作: 末端增量控制
    actions = mdp.DifferentialInverseKinematicsActionCfg(
        asset_name="robot",
        joint_names=["right_joint[1-7]"],
        body_name="right_ee_link",
        controller=DifferentialIKControllerCfg(
            command_type="pose",
            use_relative_mode=True,
            ik_method="dls",  # 与你之前的 DLS 方法一致
        ),
    )
    
    # 奖励
    rewards = {
        "success": RewardTermCfg(
            func=tray_placed_successfully,
            weight=10.0,
        ),
        "distance": RewardTermCfg(
            func=ee_to_target_distance,
            weight=-0.1,
        ),
        "action_penalty": RewardTermCfg(
            func=mdp.action_rate_l2,
            weight=-0.01,
        ),
    }
    
    # 终止条件
    terminations = {
        "time_out": TerminationTermCfg(
            func=mdp.time_out,
            time_out=10.0,  # 每个 episode 最长 10 秒
        ),
        "tray_dropped": TerminationTermCfg(
            func=tray_fell_off,
        ),
    }
    
    # 事件 (reset 时随机化)
    events = {
        "reset_tray": EventTermCfg(
            func=randomize_tray_initial_pose,
            mode="reset",
        ),
    }
```

### 3.6 HIL-SERL 训练实施 — 两种路径选择

你有两种实施路径，根据你的仿真环境选择：

**路径 A: 使用原始 hil-serl 仓库 + Isaac Sim Gym Wrapper (推荐用于仿真)**

基于 `hil-serl-sim` (GitHub: ggggfff1/hil-serl-sim) 的方法，将你的 Isaac Sim 环境包装为 Gym 接口。

```python
"""
isaac_sim_gym_env.py
将 Isaac Lab 环境包装为 hil-serl 兼容的 Gym 接口
"""
import gymnasium as gym
import numpy as np
from hil_serl.envs.wrappers import GripperCloseEnv

class IsaacTrayPlaceEnv(gym.Env):
    """
    Isaac Sim 环境的 Gym 包装器
    适配 hil-serl 的接口要求
    """
    
    def __init__(self, env_cfg):
        super().__init__()
        
        # 创建 Isaac Lab 环境
        from isaaclab.envs import ManagerBasedRLEnv
        self._isaac_env = ManagerBasedRLEnv(cfg=env_cfg)
        
        # 动作空间: 6D 末端增量 (连续)
        self.action_space = gym.spaces.Box(
            low=-0.05, high=0.05, shape=(6,), dtype=np.float32
        )
        
        # 观测空间: 字典形式 (hil-serl 要求)
        self.observation_space = gym.spaces.Dict({
            'state': gym.spaces.Box(
                low=-np.inf, high=np.inf, shape=(41,),  # 14+6+21
                dtype=np.float32
            ),
            'wrist_cam_right': gym.spaces.Box(
                low=0, high=255, shape=(128, 128, 3), dtype=np.uint8
            ),
            'wrist_cam_left': gym.spaces.Box(
                low=0, high=255, shape=(128, 128, 3), dtype=np.uint8
            ),
            'head_cam': gym.spaces.Box(
                low=0, high=255, shape=(128, 128, 3), dtype=np.uint8
            ),
        })
        
        # 来自任务一的 workspace bounds
        self.workspace_bounds = {
            'x': [0.25, 0.65],
            'y': [-0.25, 0.25],
            'z': [0.80, 1.10],
        }
    
    def reset(self, seed=None, **kwargs):
        obs_dict, info = self._isaac_env.reset()
        return self._convert_obs(obs_dict), info
    
    def step(self, action):
        # 裁剪到 workspace bounds
        action = self._clip_to_workspace(action)
        
        obs_dict, reward, terminated, truncated, info = self._isaac_env.step(action)
        return self._convert_obs(obs_dict), reward, terminated, truncated, info
    
    def _clip_to_workspace(self, action):
        """确保末端不超出工作空间边界"""
        current_ee = self._isaac_env.unwrapped.scene['robot'].data.body_state_w[
            :, self._ee_body_idx, :3
        ]
        new_pos = current_ee + action[:3]
        for i, (dim, bounds) in enumerate(
            zip(['x', 'y', 'z'], [
                self.workspace_bounds['x'],
                self.workspace_bounds['y'],
                self.workspace_bounds['z'],
            ])
        ):
            new_pos[:, i] = np.clip(new_pos[:, i], bounds[0], bounds[1])
        action[:3] = (new_pos - current_ee).squeeze()
        return action
    
    def _convert_obs(self, obs_dict):
        """将 Isaac Lab 观测转为 hil-serl 格式"""
        state = np.concatenate([
            obs_dict['joint_pos'].cpu().numpy().flatten(),
            obs_dict['joint_vel'].cpu().numpy().flatten(),
            obs_dict['ee_pos_error'].cpu().numpy().flatten(),
            obs_dict['ee_rot_error'].cpu().numpy().flatten(),
            self._action_history.flatten(),
        ])
        return {
            'state': state,
            'wrist_cam_right': obs_dict['wrist_cam_right'].cpu().numpy(),
            'wrist_cam_left': obs_dict['wrist_cam_left'].cpu().numpy(),
            'head_cam': obs_dict['head_cam'].cpu().numpy(),
        }
```

**路径 B: 使用 LeRobot + gym_hil (适合后续 Sim2Real)**

HuggingFace LeRobot 已集成 HIL-SERL，使用 MuJoCo 环境。你可以将 Isaac Sim 数据导出为 LeRobot 格式 (你之前已经做过类似工作)。

### 3.7 完整训练流程 (以路径 A 为例)

#### Step 0: 安装依赖

```bash
# 在你的云服务器上 (RTX 4090)
git clone https://github.com/ggggfff1/hil-serl-sim.git
cd hil-serl-sim

# 安装 JAX (CUDA 12)
pip install torch==2.3.0 torchvision==0.18.0 --index-url \
    https://download.pytorch.org/whl/cu121 \
    -i https://pypi.tuna.tsinghua.edu.cn/simple

pip install "jax[cuda12_pip]==0.4.35" -f \
    https://storage.googleapis.com/jax-releases/jax_cuda_releases.html

# 安装 hil-serl 核心
pip install -e .
pip install -e serl_robot_infra/

# 注意: 你的服务器 GPU 0 故障, 确保
export CUDA_VISIBLE_DEVICES=1
```

#### Step 1: 训练 Reward Classifier

```bash
# 1. 采集正负样本
#    在 Isaac Sim 中手动将托盘放到正确/错误位置,截图
#    正样本: ~50 张托盘在传送带上的图片
#    负样本: ~50 张托盘偏离/掉落/碰撞的图片

# 2. 训练分类器
python train_reward_classifier.py \
    --positive_dir ./data/positive/ \
    --negative_dir ./data/negative/ \
    --output ./reward_classifier/classifier.pth \
    --epochs 50
```

#### Step 2: 采集 Demo 轨迹

```bash
# 启动 Isaac Sim 环境的 actor (带键盘/手柄控制)
python examples/experiments/tray_place/run_demo_collection.py \
    --num_demos 20 \
    --save_dir ./demo_data/

# 控制键 (参考 hil-serl-sim 的控制方案):
#   W/S: 前/后 (X轴)
#   A/D: 左/右 (Y轴)
#   J/K: 上/下 (Z轴)
#   L: 开/关夹爪
#   ;: 切换人类介入模式
```

#### Step 3: 启动 RL 训练

HIL-SERL 使用 Actor-Learner 异步架构，需要启动两个进程：

```bash
# Terminal 1: Learner 进程 (GPU 密集)
export CUDA_VISIBLE_DEVICES=1
python examples/experiments/tray_place/run_learner.py \
    --env tray_place_isaac \
    --demo_path ./demo_data/ \
    --classifier_path ./reward_classifier/classifier.pth \
    --batch_size 256 \
    --utd_ratio 4 \
    --learning_rate 3e-4

# Terminal 2: Actor 进程 (环境交互 + 人类介入)
python examples/experiments/tray_place/run_actor.py \
    --env tray_place_isaac \
    --enable_human_intervention \
    --checkpoint_dir ./checkpoints/

# Terminal 3: 监控训练
tensorboard --logdir ./logs/
```

#### Step 4: 训练过程中的人类介入策略

```
训练阶段          介入策略
─────────────────────────────────────────────
0~500 步          频繁介入 (~70% 的 episode)
                  目的: 示范正确的放置动作路径
                  
500~2000 步       适度介入 (~30%)
                  目的: 仅在快要碰撞或掉落时纠正
                  
2000~5000 步      偶尔介入 (~10%)  
                  目的: 仅在极端失败时纠正

5000+ 步          不再介入
                  目的: 评估策略的独立表现
```

#### Step 5: 评估

```bash
python examples/experiments/tray_place/run_actor.py \
    --eval_checkpoint_step 30000 \
    --eval_n_trajs 100 \
    --checkpoint_path ./checkpoints/best/ \
    --no_intervention  # 禁用人类介入
```

### 3.8 关键超参数参考

```python
hil_serl_config = {
    # RLPD (SAC 变体)
    'algorithm': 'RLPD',
    'discount': 0.99,
    'tau': 0.005,           # target network 软更新系数
    'learning_rate': 3e-4,
    'batch_size': 256,
    'utd_ratio': 4,         # update-to-data ratio (每个环境步更新4次)
    'demo_ratio': 0.5,      # demo buffer 采样比例 (50%)
    
    # 视觉 backbone
    'encoder': 'resnet10',
    'encoder_pretrained': True,
    'encoder_frozen': True,  # 冻结 backbone
    'image_size': (128, 128),
    
    # Actor-Critic 网络
    'actor_hidden': [256, 256],
    'critic_hidden': [256, 256],
    'num_critics': 10,       # ensemble critics for better exploration
    'num_min_critics': 2,    # 取最小的 2 个 critic 值 (保守估计)
    
    # Grasp Critic (DQN for discrete gripper)
    'grasp_critic_lr': 1e-4,
    'grasp_critic_hidden': [256, 256],
    
    # 训练
    'total_steps': 50000,
    'eval_interval': 5000,
    'save_interval': 5000,
    'warmup_steps': 1000,    # 前 1000 步只用 demo 数据
}
```

### 3.9 任务分解建议

你的完整任务（放置2个托盘 + 移动 + 抓取2个托盘）是一个长horizon任务。HIL-SERL 最适合 5~10 秒的短任务。建议将整个流程拆解为独立的子技能分别训练：

```
子技能 1: right_place_tray
  - 右臂将托盘从当前位置放到传送带上
  - 约 5 秒, 最适合 HIL-SERL

子技能 2: left_to_right_handover  
  - 左臂将托盘递给右臂
  - 约 5 秒, 双臂协调任务

子技能 3: right_grasp_tray
  - 右臂从传送带出口抓取托盘
  - 约 5 秒

子技能 4: right_to_left_handover
  - 右臂将抓取的托盘递给左臂
  - 约 5 秒

整体编排 (由上层 FSM 控制):
  位置1: 子技能1 → 子技能2 → 子技能1
  基座移动 (OCS2 导航)
  位置2: 子技能3 → 子技能4 → 子技能3
```

---

## 4. 实施路线图与时间规划

```
Week 1-2: OCS2 控制桥接
  ├── Day 1-3: Isaac Sim ROS2 Bridge 配置, 验证关节指令收发
  ├── Day 4-7: OCS2 控制器接入, 单臂运动验证
  ├── Day 8-10: 双臂协调, 工作空间扫描
  └── Day 11-14: 确定位置1/2, 记录 workspace bounds

Week 3: HIL-SERL 环境搭建
  ├── Day 1-2: 安装 hil-serl 依赖, 跑通 pick_cube_sim 示例
  ├── Day 3-5: 创建 Isaac Sim Gym Wrapper
  ├── Day 5-7: 采集 reward classifier 数据并训练

Week 4-5: RL 训练
  ├── Day 1-3: 采集 20 条 demo 轨迹 (子技能1: right_place_tray)
  ├── Day 4-7: HIL-SERL 训练子技能1 (预计 ~30000 steps)
  ├── Day 8-10: 训练子技能2 (left_to_right_handover)
  └── Day 11-14: 训练子技能3/4, 整合测试

Week 6: 集成与评估
  ├── 上层 FSM 编排
  ├── 端到端流程测试
  └── 结果整理, 论文/报告素材准备
```

---

## 附录 A: 常见问题排查

| 问题 | 可能原因 | 解决方案 |
|------|---------|---------|
| OCS2 solver 不收敛 | 初始猜测离目标太远 | 使用 warm-start, 先 MoveJ 到附近 |
| Isaac Sim 关节抖动 | PD 增益不匹配 | 调整 stiffness/damping, 降低控制频率 |
| RL 训练奖励始终为 0 | Reward classifier 阈值太高 | 降低阈值到 0.5, 检查分类器准确率 |
| 训练不收敛 | workspace bounds 太大 | 缩小边界, 增加 demo 数量 |
| 图像特征无用 | 相机角度看不到关键区域 | 调整相机位姿, 确保能看到托盘和传送带 |
| 夹爪控制混乱 | 连续/离散动作混淆 | 确认使用独立 DQN 控制夹爪 |

## 附录 B: 关键参考资料

- HIL-SERL 论文: [arXiv:2410.21845](https://arxiv.org/abs/2410.21845)
- HIL-SERL 代码: [rail-berkeley/hil-serl](https://github.com/rail-berkeley/hil-serl)
- HIL-SERL 仿真版: [ggggfff1/hil-serl-sim](https://github.com/ggggfff1/hil-serl-sim)
- LeRobot HIL-SERL 教程: [HuggingFace Docs](https://hugging-face.cn/docs/lerobot/hilserl)
- Isaac Lab RL 训练教程: [Isaac Lab Docs](https://isaac-sim.github.io/IsaacLab/main/source/tutorials/03_envs/run_rl_training.html)
- RLPD 论文: [arXiv:2302.02948](https://arxiv.org/abs/2302.02948)
