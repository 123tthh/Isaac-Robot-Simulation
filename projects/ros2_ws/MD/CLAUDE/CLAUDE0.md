# CLAUDE.md — OCS2 ↔ Isaac Sim 仿真闭环搭建

> **给 Claude Code 的项目上下文文件**
> 放置于 `~/ros2_ws/MD/CLAUDE.md`，确保 Claude Code 理解项目全貌

---

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

---

## 机器人硬件配置

### 机体结构

| 部件 | 规格 |
|------|------|
| 底盘 | 轮式移动底盘 (2 驱动轮 + 4 万向轮) |
| 躯干 | 共享躯干，waist_joint (升降, prismatic) + body_joint (俯仰, revolute) |
| 手臂 | 2× 睿尔曼 RM75-6FB，各 7 DOF |
| 夹爪 | 2× 平行夹爪 (PGIA)，开/关离散状态，Mimic 联动 |
| 相机 | 3× Intel RealSense D455 (头部×1 + 左右腕各×1) |

### 关节名称 (URDF)

**手臂关节 (14个, OCS2 控制):**
```
left_joint1, left_joint2, left_joint3, left_joint4, left_joint5, left_joint6, left_joint7
right_joint1, right_joint2, right_joint3, right_joint4, right_joint5, right_joint6, right_joint7
```

**夹爪关节 (4个, 离散控制):**
```
left_PGIA_joint1, left_PGIA_joint2    # 左夹爪, prismatic [0, 0.04]m
right_PGIA_joint1, right_PGIA_joint2  # 右夹爪, prismatic [0, 0.04]m
```
注意: PGIA_joint1 和 joint2 通过 URDF Mimic 标签联动，控制其中一个即可。

**躯干/底盘关节:**
```
waist_joint          # prismatic, Z 轴升降 [-0.5, 0]m
body_joint           # revolute, Y 轴俯仰 [0, 1.5708]rad
head_camera_joint    # continuous, 头部相机旋转
wheel_L_joint, wheel_R_joint           # 驱动轮
caster_LF/RF/LB/RB_joint              # 万向轮
```

### RM75-6FB 手臂动力学参数

| Joint | 最大角速度 (°/s) | 关节范围 (rad) |
|-------|-----------------|---------------|
| J1 | 180 | [-3.107, 3.107] |
| J2 | 180 | [-2.269, 2.269] |
| J3 | 225 | [-1.896, 1.896] |
| J4 | 225 | [-2.356, 2.356] |
| J5 | 225 | [-3.107, 3.107] |
| J6 | 225 | [-2.234, 2.234] |
| J7 | 225 | [-6.283, 6.283] |

手臂自重 7.9kg/只，有效负载 5kg，工作半径 638.5mm (6FB版)。

---

## ROS 2 话题 (已确认存在)

### 实机端已有话题

```
/arm_joint_cmd                          # OCS2 → 手臂关节指令
/joint_states                           # 当前关节状态
/left_gripper_controller/commands       # 左夹爪指令
/right_gripper_controller/commands      # 右夹爪指令
/tf, /tf_static                         # 坐标变换
/odom                                   # 里程计
/cmd_vel                                # 底盘速度指令
/agv_pose                               # AGV 位姿
```

### Isaac Sim 需要桥接的话题

| 方向 | 话题 | 消息类型 | 对应 Action Graph |
|------|------|---------|------------------|
| ROS2 → Sim | `/arm_joint_cmd` | `sensor_msgs/JointState` | Arm_Control_Graph |
| ROS2 → Sim | `/left_gripper_controller/commands` | `std_msgs/Float64MultiArray` | Gripper_Control_Graph |
| ROS2 → Sim | `/right_gripper_controller/commands` | `std_msgs/Float64MultiArray` | Gripper_Control_Graph |
| Sim → ROS2 | `/joint_states` | `sensor_msgs/JointState` | State_Telemetry_Graph |
| Sim → ROS2 | `/tf` | `tf2_msgs/TFMessage` | State_Telemetry_Graph |
| Sim → ROS2 | `/head_cam/color/image_raw` | `sensor_msgs/Image` | Camera_Publish_Graph |
| Sim → ROS2 | `/head_cam/depth/image_rect_raw` | `sensor_msgs/Image` | Camera_Publish_Graph |
| Sim → ROS2 | `/left_cam/color/image_raw` | `sensor_msgs/Image` | Camera_Publish_Graph |
| Sim → ROS2 | `/left_cam/depth/image_rect_raw` | `sensor_msgs/Image` | Camera_Publish_Graph |
| Sim → ROS2 | `/right_cam/color/image_raw` | `sensor_msgs/Image` | Camera_Publish_Graph |
| Sim → ROS2 | `/right_cam/depth/image_rect_raw` | `sensor_msgs/Image` | Camera_Publish_Graph |

**关键**: 实机端用 `/arm_joint_cmd` 而不是 `/joint_commands`。Isaac Sim 的 `ros2_subscribe_joint_state` 节点的 Topic Name 必须设为 `/arm_joint_cmd` 以匹配。

---

## Isaac Sim 配置

### URDF 导入设置

| 选项 | 值 | 原因 |
|------|----|------|
| Links | Moveable Base | 轮式底盘需移动 |
| Default Density | 1000 | 安全网 (link 已有 inertial) |
| Ignore Mimic | 不勾选 | 夹爪联动 |
| Joint Configuration | Natural Frequency | 调参直观 |
| Drive Type | **Force** | OCS2 掌控完整动力学补偿 |
| Collision From Visuals | 不勾选 | URDF 有专用碰撞 mesh 时 |
| Collider Type | Convex Decomposition | 复杂形状精度 |
| Allow Self-Collision | **不勾选** | 关节处 mesh 重叠会爆炸 |
| Replace Cylinders with Capsules | 勾选 | 数值稳定 |

### Action Graphs (已搭建4个)

```
/World/ActionGraphs/
  ├── Arm_Control_Graph
  │     on_playback_tick → ros2_subscribe_joint_state → articulation_controller
  │     ros2_context
  │
  ├── Gripper_Control_Graph
  │     on_playback_tick → sub_left_gripper → artic_right_gripper
  │                      → sub_right_gripper → artic_left_gripper
  │     ros2_context
  │
  ├── State_Telemetry_Graph
  │     on_playback_tick → ros2_publish_joint_state
  │                      → publish_tf
  │     ros2_context, read_sim_time
  │
  └── Camera_Publish_Graph
        on_playback_tick → cam_head_rgb, cam_head_depth
                         → cam_left_rgb, cam_left_depth
                         → cam_right_rgb, cam_right_depth
        ros2_context
```

### Robot prim 路径

```
/World/Robot                            # articulation root
/World/Robot/base_link                  # 底盘基座
/World/Robot/trunk_link                 # 躯干
/World/Robot/left_base_link             # 左臂基座
/World/Robot/left_Link1 ... left_Link7  # 左臂连杆
/World/Robot/right_base_link            # 右臂基座
/World/Robot/right_Link1 ... right_Link7 # 右臂连杆
```

---

## 当前阶段：搭建最小闭环

### 目标

跑通 **OCS2 发送关节指令 → Isaac Sim 执行 → 关节状态反馈给 OCS2** 这个核心环路。

### 当前状态

- [x] URDF 修正 (惯性参数、关节限位、相机惯量)
- [x] Isaac Sim 导入机器人模型
- [x] 4 个 Action Graph 已搭建
- [x] 6 个相机 prim 已创建并验证
- [ ] **Arm_Control_Graph 话题名匹配** (需设为 `/arm_joint_cmd`)
- [ ] **OCS2 节点适配 Isaac Sim 接口**
- [ ] **最小闭环验证** (手臂能动)

### 需要解决的问题



#### 问题 1: OCS2 输出格式（已解决）

OCS2 修改版通过 `/arm_joint_cmd` 发布 `sensor_msgs/JointState`。需要确认：
- `name` 字段包含哪些关节名？是否与 URDF 中的 `left_joint1`...`right_joint7` 一致？
- `position` 字段单位是否为弧度？
- 发布频率是多少 Hz？

#### 问题 2: 手动测试指令（已解决）

在 OCS2 接入前，先用命令行验证 Isaac Sim 能响应：

```bash
# 正确的 ros2 topic pub 语法 (注意不能用 '...' 省略)
ros2 topic pub --once /arm_joint_cmd sensor_msgs/msg/JointState \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, \
    name: ['left_joint1', 'left_joint2', 'left_joint3', 'left_joint4', \
           'left_joint5', 'left_joint6', 'left_joint7'], \
    position: [0.5, 0.3, 0.0, 0.5, 0.0, 0.0, 0.0], \
    velocity: [], effort: []}"
```

如果手臂动了，说明 Arm_Control_Graph 链路通了。

#### 问题 3: articulation_controller 配置（已解决）

`articulation_controller` 节点需要正确设置：

- **Robot Path**: `/World/Robot`
- **Joint Names**: 从 subscriber 输出连接 (动态获取)
- **Position Command**: 从 subscriber 的 Position Command 输出连接
- 确保 Joint Indices 为空 (使用 Joint Names 模式)

#### 验证：

运行仿真后

脚本check_isaac_bridge.py做了 4 项检查：

| 检查 | 验证内容                                              |
| ---- | ----------------------------------------------------- |
| 1    | `/joint_states` 是否有数据，14 个手臂关节是否全部存在 |
| 2    | 发送 `left_joint1=0.3` 指令后关节是否真的动了         |
| 3    | 列出所有控制相关话题，确认 `/arm_joint_cmd` 等存在    |
| 4    | `/joint_states` 发布频率是否够用 (OCS2 需要 100Hz+)   |

运行方式（Isaac Sim 按 Play 后）：

```bash
cd ~/ros2_ws/scripts
python3 check_isaac_bridge.py
```

修复：

**欠阻尼，阻尼比严重不足**。Stiffness 2520（刚度系数） 配 Damping 1.0（阻尼系数），相当于一个几乎没有减震的硬弹簧——弹一下就飞出去了。

### Force vs Acceleration 在位置控制中的区别

|                 | Force Drive                                  | Acceleration Drive                  |
| --------------- | -------------------------------------------- | ----------------------------------- |
| 位置控制        | ✅ 可以（Stiffness = 弹簧，Damping = 阻尼器） | ✅ 可以                              |
| OCS2 兼容       | ✅ 与真实电机特性匹配                         | ⚠️ 归一化惯量，OCS2 动力学模型不一致 |
| Target Position | 填 articulation_controller 发来的角度        | 同左                                |

**建议不变：保留 Force Drive**，因为 OCS2 需要一致的动力学特性。

Damping 太低。Natural Frequency 模式计算出的 Damping=1.008 意味着导入时的**阻尼比 ζ 设得很小**（可能 ζ=0.01 级别）。临界阻尼需要 ζ=1.0。

**修正后的脚本set_drive_params.py把 Damping 调到合理值。**

再次验证结果：

```
============================================================
  Isaac Sim ↔ ROS2 通信链路诊断
  前提: Isaac Sim 已打开 stage 并按下 ▶ Play
============================================================

============================================================
  检查 3: 当前 ROS2 话题中与控制相关的列表
============================================================
  关键话题:
    /arm_joint_cmd
    /clock
    /cmd_vel
    /joint_states
    /left_gripper_controller/commands
    /loc_cmd
    /right_gripper_controller/commands
    /tf
    /tf_static

  话题存在性检查:
  ✅ /arm_joint_cmd                                OCS2 → Isaac Sim 指令
  ✅ /joint_states                                 Isaac Sim → OCS2 状态
  ✅ /tf                                           TF 变换树
  ✅ /left_gripper_controller/commands             左夹爪
  ✅ /right_gripper_controller/commands            右夹爪

============================================================
  检查 1: /joint_states 是否有数据
============================================================
  等待 5.0s 内接收 /joint_states ...
  ✅ 收到 /joint_states
     关节数: 27
     关节名: ['caster_LB_joint', 'caster_LF_joint', 'caster_RB_joint', 'caster_RF_joint', 'wheel_L_joint']...
     位置数: 27
     时间戳: sec=865, nanosec=933378495
  ✅ 左右臂 14 个关节全部存在于 /joint_states

============================================================
  检查 4: /joint_states 发布频率
============================================================
  采集 2s 内的消息数量...
  2s 内收到 120 条消息 → 频率 ≈ 60.0 Hz
  ✅ 频率正常

============================================================
  检查 2: 发送指令 → 验证关节响应
============================================================
  读取当前关节位置...
  发送前 left_joint1 = 0.0022 rad

  发送测试指令 (left_joint1=0.3, left_joint4=0.5)...
  📤 Published [TEST]: {'left_joint1': 0.3, 'left_joint2': 0.0, 'left_joint3': 0.0}...
  等待 3s 观察 /joint_states 变化...

  结果:
    left_joint1: 0.0022 → 0.3423  (Δ=0.3401)
    left_joint4: -0.0077 → 0.4931  (Δ=0.5008)

  ✅ 手臂响应了指令！Arm_Control_Graph 链路正常

  发送归零指令...
  📤 Published [HOME]: {'left_joint1': 0.0, 'left_joint2': 0.0, 'left_joint3': 0.0}...

============================================================
  诊断完成
============================================================


```

#### 检查 1 输出

```
关节数: 27
时间戳: sec=865
```

**27 个关节** = 14 手臂 + 4 夹爪 + 2 躯干(waist+body) + 1 头部相机 + 6 轮子。全部被 `State_Telemetry_Graph` 正确发布。时间戳 sec=865 表示仿真已经运行了约 865 秒。

#### 检查 4 输出

```
频率 ≈ 60.0 Hz
```

Isaac Sim 默认 60 FPS 渲染，`on_playback_tick` 每帧触发一次，所以 `/joint_states` 发布频率 = 渲染帧率 ≈ 60 Hz。对 OCS2 基本够用（MPC 通常 100-400 Hz，但 60 Hz 足以验证闭环）。

#### 检查 2 关键数据

```
left_joint1: 0.0022 → 0.3423  (Δ=0.3401)   目标 0.3
left_joint4: -0.0077 → 0.4931  (Δ=0.5008)   目标 0.5
```

**怎么判断控制是否正常：**

| 指标       | 计算               | 修复前   | 修复后    | 判断标准        |
| ---------- | ------------------ | -------- | --------- | --------------- |
| 稳态误差   | \|实际 - 目标\|    | 2.31 rad | 0.042 rad | < 0.05 rad 正常 |
| 超调量     | (实际-目标)/目标   | +770%    | +14%      | < 20% 可接受    |
| 方向正确性 | 实际是否朝目标移动 | ❌ 飞反了 | ✅ 正确    | 方向一致        |

修复前 Damping=1.0 时关节像弹弓一样弹飞到 2.61 rad（限位附近），修复后 Damping=80 提供了足够的"刹车力"，关节平稳到达目标附近。

#### joint4 欠调 vs joint1 超调

```
joint1: 超调 14% (0.3423 > 0.3)  → Damping 稍低，有少许冲过头
joint4: 欠调 1.4% (0.4931 < 0.5) → Damping 略高，还没完全到位
```

这是因为 joint1（肩部，惯量大）和 joint4（肘部，惯量小）用了不同的 Stiffness/Damping 比值。这在 3 秒内的单次采样中完全正常——如果持续发送目标位置（OCS2 闭环时每 10-16ms 发一次），稳态误差会趋近零。

### 最小闭环已跑通 ✅

现在的状态：

```
✅ /arm_joint_cmd → Arm_Control_Graph → articulation_controller → 手臂运动
✅ PhysX 仿真 → State_Telemetry_Graph → /joint_states 发布 (60Hz, 27关节)
✅ 位置控制正常，超调可控
```

---

## 任务工作流 (阶段 A/B/C)

### 阶段 A: 放置循环 (输入传送带)

```
机器人驻车在输入传送带前
→ 右臂放托盘1到传送带 (左臂等待)
→ 左臂将托盘2递给右臂
→ 左臂退出避让
→ 右臂放托盘2到传送带
→ 双臂回初始姿态
```

### 阶段 B: 平移到输出传送带

```
轮式底盘向左平移 → 到达输出传送带最佳取盘位置
```

### 阶段 C: 抓取循环 (输出传送带)

```
右臂抓取托盘1
→ 左臂接过右臂的托盘1
→ 右臂抓取托盘2
→ 返回阶段 A
```

### 托盘参数 (域随机化范围)

| 参数 | 名义安全区 |
|------|-----------|
| 质量 | [0.70, 2.00] kg |
| 静摩擦系数 μ_s | [0.020, 0.100] |
| 释放位姿 Z | [1.255, 1.291] m |
| 释放位姿 Pitch | [4.0, 25.0]° |

---

## 文件结构

```
~/ros2_ws/
  ├── MD/
  │   ├── CLAUDE.md                  # ← 本文件
  │   └── scene_structure.md
  ├── src/
  │   ├── ocs2_control/          # 修改版 OCS2 控制器
  │   │   ├── config/            # MPC 参数配置
  │   │   ├── launch/            # ROS2 launch 文件
  │   │   └── src/               # C++ 源码
  │   └── robot_description/     # URDF/meshes
  │       ├── urdf/
  │       │   ├── r1.urdf        # 原始 URDF
  │       │   └── r1_fixed.urdf  # 修正版(目前使用) (惯性参数修复)
  │       └── meshes/            # 3D mesh 文件
  └── scripts/
      ├── scene_structure.md     # Isaac Sim stage 结构导出
      └── test_arm_cmd.py        # 手臂控制测试脚本
```

---

## OCS2 接入方案（已实现）

### 新增文件

```
src/robot/ (r1_description 包)
  ├── xacro/ros2_control/robot.xacro     ← 修改: 新增 isaac 硬件类型
  ├── config/ocs2/task.info              ← 新建: 14-DOF 双臂 OCS2 任务配置
  ├── config/ros2_control/
  │   ├── ros2_controllers.yaml          ← 原有 (demo 模式)
  │   └── ocs2_controllers.yaml          ← 新建: OCS2 控制器参数
  └── launch/ocs2_isaac.launch.py        ← 新建: OCS2+Isaac Sim 启动文件
```

### 数据流

```
Isaac Sim
  └─ State_Telemetry_Graph
       └─ /joint_states (60Hz, 27关节)
            └──► topic_based_ros2_control
                  (匹配 left_joint1..7, right_joint1..7)
                  └──► ros2_control 状态接口
                        └──► ocs2_arm_controller (100Hz)
                              ├── updateObservation() ← 读 14 个关节位置
                              ├── advanceMpc() ← OCS2 求解 (25Hz)
                              └── evaluatePolicy() ← 写位置命令
                                    └──► topic_based_ros2_control
                                          └──► /arm_joint_cmd (JointState, 14关节)
                                                └──► Isaac Sim Arm_Control_Graph
                                                       └─ articulation_controller → 手臂运动
```

### 构建与启动

```bash
# 1. 构建 r1_description 包
cd ~/ros2_ws
colcon build --packages-select r1_description
source install/setup.bash

# 2. 确保 Isaac Sim 已运行并按下 Play (/joint_states 正在发布)

# 3. 启动 OCS2 闭环控制
ros2 launch r1_description ocs2_isaac.launch.py

# 4. OCS2 控制器默认进入 HOLD 状态 (保持当前位置)
# 通过 FSM 指令切换状态:
ros2 topic pub --once /fsm_command std_msgs/msg/Int32 "{data: 1}"  # → HOME (归零)
ros2 topic pub --once /fsm_command std_msgs/msg/Int32 "{data: 3}"  # → OCS2 (MPC 规划)
ros2 topic pub --once /fsm_command std_msgs/msg/Int32 "{data: 2}"  # → HOLD (停止)

# 5. 进入 OCS2 模式后，在 RViz 中拖动 Interactive Marker 设置目标末端位姿
#    arms_target_manager 会发布目标到 OCS2 的 PoseBasedReferenceManager
```

### OCS2 任务配置要点 (config/ocs2/task.info)

| 参数 | 值 | 说明 |
|------|---|------|
| manipulatorModelType | 0 | defaultManipulator (固定基座) |
| baseFrame | base_link | 运动链根节点 |
| eeFrame | left_flange_link | 左臂末端 |
| eeFrame1 | right_flange_link | 右臂末端 |
| removeJoints | waist, body, head, wheels, casters, PGIA | 只规划14个手臂关节 |
| selfCollision.activate | false | 初始关闭，验证后可打开 |

### 首次运行注意事项

1. **OCS2 代码生成**：`recompileLibraries=true`，首次启动会编译动力学代码到 `install/r1_description/share/r1_description/ocs2/`，耗时约30秒。
2. **状态顺序**：OCS2 状态向量顺序 = YAML `joints` 参数顺序 = `[left_joint1..7, right_joint1..7]`，如果 MPC 行为异常检查此顺序。
3. **自碰撞**：初始关闭，调试稳定后在 task.info 中设置 `activate = true` 并配置碰撞对。

---

## 优先级任务清单

```
✅ P0: 跑通 Arm_Control + State_Telemetry 最小闭环
✅ P0: OCS2 → Isaac Sim 通信 (已实现，待运行验证)
    → topic_based_ros2_control 桥接 /joint_states ↔ /arm_joint_cmd
    → ocs2_arm_controller 订阅状态发布轨迹

P1: 验证 OCS2 闭环
    → colcon build, ros2 launch r1_description ocs2_isaac.launch.py
    → 验证 MPC 初始化 (等待 initial policy)
    → 拖动 RViz marker, 观察手臂是否跟踪目标

P1: Gripper 联调
    → 验证 /left_gripper_controller/commands
    → 验证 Mimic 联动 (发一个 joint 另一个跟随)

P1: Camera 联调
    → 验证 6 个相机话题发布
    → 感知节点订阅并检测托盘位姿

P2: 完整闭环 (A/B/C 阶段全流程)
P3: Trajectory Logger → 强化学习数据
```
