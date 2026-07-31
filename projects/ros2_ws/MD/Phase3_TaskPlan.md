# 第三阶段任务规划：夹爪联调 + 碰撞检测 + 轨迹示教录制

> 用于开启新对话，使 LLM 理解任务背景、当前状态和目标
> 搭配 ~/ros2_ws/CLAUDE.md 和 代码变更清单.md 使用

---

## 一、项目背景概要

### 1.1 项目目标

R1 轮式双臂机器人在 Isaac Sim 5.1 中完成传送带托盘放置/抓取任务。整体分三阶段：

| 阶段 | 目标 | 状态 |
|------|------|------|
| 阶段1 | OCS2 MPC ↔ Isaac Sim 控制闭环 | ✅ 已完成 |
| 阶段2 | 夹爪联调 + 碰撞检测 + 轨迹示教录制 | 🔴 **当前阶段** |
| 阶段3 | HIL-SERL 强化学习训练 + Sim2Real | ⏳ 待开始 |

### 1.2 阶段1 已完成的成果

OCS2 → ROS2 → Isaac Sim 控制闭环已打通，具体包括：

- ✅ 在 RViz2 中拖动 Interactive Marker → Isaac Sim 中双臂手臂跟随运动
- ✅ 14 个手臂关节（left_joint1-7 + right_joint1-7）受 OCS2 MPC 实时控制
- ✅ `topic_based_ros2_control/TopicBasedSystem` 作为 Isaac Sim ↔ ros2_control 桥接
- ✅ TF 树完整（world → odom → base_link → 所有子 frame）
- ✅ 6 个相机（3×RGB + 3×Depth）话题正常发布
- ✅ /joint_states 60Hz 反馈、/clock 仿真时钟、/odom 里程计 均正常

控制链路数据流：
```
RViz Interactive Marker → arms_target_manager
  → /left_target/stamped, /right_target/stamped
    → ocs2_arm_controller (MPC 求解, 25Hz)
      → topic_based_ros2_control
        → /arm_joint_cmd (sensor_msgs/JointState, 14 关节)
          → Isaac Sim Arm_Control_Graph → 手臂运动
            → /joint_states (60Hz, 27 关节) → 反馈给 OCS2
```

### 1.3 机器人关节配置

| 关节组 | 关节名 | 数量 | 控制方式 | 当前状态 |
|--------|--------|------|---------|---------|
| 左臂 | left_joint1-7 | 7 | OCS2 MPC 位置控制 | ✅ 已通 |
| 右臂 | right_joint1-7 | 7 | OCS2 MPC 位置控制 | ✅ 已通 |
| 左夹爪 | left_PGIA_joint1 (主动) + left_PGIA_joint2 (Mimic 从动) | 2 | 离散位置控制 | 🔴 待联调 |
| 右夹爪 | right_PGIA_joint1 (主动) + right_PGIA_joint2 (Mimic 从动) | 2 | 离散位置控制 | 🔴 待联调 |
| 躯干 | waist_joint (prismatic) + body_joint (revolute) | 2 | 暂不控制 | — |
| 轮子 | wheel_L/R, caster_LB/LF/RB/RF | 6 | 阶段B底盘移动 | ⏳ |

### 1.4 Isaac Sim Action Graph 架构

```
Arm_Control_Graph:       订阅 /arm_joint_cmd → 驱动 14 个手臂关节     ✅
Gripper_Control_Graph:   订阅夹爪指令 → 驱动 PGIA_joint1              🔴 待验证
State_Telemetry_Graph:   发布 /joint_states + /clock                  ✅
Camera_Publish_Graph:    发布 6 个 RGB-D 话题                         ✅
```

### 1.5 关键文件位置

```
~/ros2_ws/src/robot/                    ← 包名: r1_description
  urdf/r1_fixed.urdf                    ← 当前使用的 URDF
  xacro/ros2_control/robot.xacro        ← 硬件接口定义
  config/ocs2/task.info                 ← OCS2 MPC 配置
  config/ros2_control/ocs2_controllers.yaml
  launch/ocs2_isaac.launch.py           ← 一键启动
  scripts/odom_to_tf.py                 ← /odom → TF

~/ros2_ws/src/arms_ros2_control/
  command/arms_target_manager/          ← RViz Interactive Marker
  controller/ocs2_arm_controller/       ← OCS2 MPC 控制器
  hardwares/topic_based_ros2_control/   ← Isaac Sim 桥接插件

~/ros2_ws/scripts/
  set_drive_params.py                   ← Isaac Sim 关节 Drive 参数
  check_isaac_bridge.py                 ← 通信链路诊断
  robot_teach.py                        ← 关节示教工具（已有基础版）
```

---

## 二、当前阶段三大任务

### 任务 A：夹爪联调

### 任务 B：碰撞检测配置

### 任务 C：轨迹示教录制脚本

---

## 三、任务 A：夹爪联调

### 3.1 夹爪硬件参数

| 参数 | 值 |
|------|---|
| 类型 | 平行电爪 (PGIA) |
| 关节类型 | prismatic (直线移动) |
| 主动关节 | left_PGIA_joint1 / right_PGIA_joint1 |
| 从动关节 | left_PGIA_joint2 / right_PGIA_joint2 (Mimic, 自动跟随) |
| 闭合位置 | 0.04 m |
| 张开位置 | 0.1 m（最大张开） |
| 控制方式 | 离散二值：闭合(0.04) 或 张开(0.10) |

注意：在这个机器人中，0.04m 是闭合（夹住），0.0m 是张开（松开）。

### 3.2 Isaac Sim Gripper_Control_Graph 当前配置

```
on_playback_tick → sub_left_gripper → artic_right_gripper
                 → sub_right_gripper → artic_left_gripper
ros2_context
```

话题：
- `/left_gripper_controller/commands` (std_msgs/Float64MultiArray)
- `/right_gripper_controller/commands` (std_msgs/Float64MultiArray)

⚠️ 注意连线交叉：`sub_left_gripper` 连到了 `artic_right_gripper`，`sub_right_gripper` 连到了 `artic_left_gripper`。需要验证这是否是 bug（可能只是命名问题）。

### 3.3 验证步骤

```bash
# Step 1: 验证话题存在
ros2 topic list | grep gripper

# Step 2: 发送闭合指令（左夹爪）
ros2 topic pub --once /left_gripper_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.04]}"

# Step 3: 发送张开指令（左夹爪）
ros2 topic pub --once /left_gripper_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.0]}"

# Step 4: 验证 Mimic 联动
ros2 topic echo /joint_states --once | grep -A1 "PGIA"
# 应看到 PGIA_joint1 和 PGIA_joint2 位置一致

# Step 5: 右夹爪同理
ros2 topic pub --once /right_gripper_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.04]}"
```

### 3.4 可能遇到的问题

| 问题 | 排查方法 | 解决方案 |
|------|---------|---------|
| 夹爪不响应 | 检查 Gripper_Control_Graph 连线，确认 sub 和 artic 节点的 Topic/RobotPath | 修正连线或属性 |
| Mimic 不联动 | 检查 Isaac Sim 中 PGIA_joint2 是否有 Mimic API | 确认 URDF 导入时未勾选 Ignore Mimic |
| 左右夹爪反了 | sub_left 连到 artic_right 可能是 bug | 交换连线 |
| 夹爪 Stiffness 不足 | 夹不住托盘 | 在 Isaac Sim 中调整 PGIA_joint1 的 Drive Stiffness (推荐 1000) |

---

## 四、任务 B：碰撞检测配置

### 4.1 需要碰撞检测的对象

在完整工作循环中，以下碰撞对需要物理引擎正确处理：

| 碰撞对 | 场景 | 必要性 |
|--------|------|--------|
| 夹爪尖端 ↔ 托盘 | 抓取/释放时夹爪与托盘接触 | 🔴 必须 |
| 夹爪尖端 ↔ 传送带滚筒/导轨 | 放置/抓取时避免穿模 | 🔴 必须 |
| 托盘 ↔ 传送带表面 | 托盘放在传送带上 | 🔴 必须 |
| 左臂 ↔ 右臂 | 双臂递交时避免穿模 | 🟡 建议 |
| 托盘 ↔ 地面 | 托盘掉落检测 | 🟡 建议 |

### 4.2 Isaac Sim 中的碰撞体现状

从 scene_structure.md 看：

```
/World/Pallet/CollisionProxy:      Collision ✅ + RigidBody ✅
/World/Pallet/Back_CollisionProxy: Collision ✅ + RigidBody ✅
/World/ConveyorGroup/Roller_00_L:  Collision ✅ + RigidBody ❌ (静态)
/World/ConveyorGroup/Rail_*:       Collision ✅ + RigidBody ❌ (静态)
Robot arm links:                   Collision ✅ + RigidBody ✅ (关节驱动)
```

### 4.3 需要在 Isaac Sim 中添加的碰撞配置

#### 4.3.1 夹爪尖端碰撞

检查 PGIA_link1 和 PGIA_link2 是否有 Collision API：
```
/World/Robot/left_PGIA_link1   → 需要 CollisionAPI
/World/Robot/left_PGIA_link2   → 需要 CollisionAPI
/World/Robot/right_PGIA_link1  → 需要 CollisionAPI
/World/Robot/right_PGIA_link2  → 需要 CollisionAPI
```

如果没有，在 Isaac Sim Property 面板中为每个 link 添加 Physics > Collision > Collider。

#### 4.3.2 Physics Material（摩擦系数）

托盘和夹爪之间的摩擦系数直接影响抓取稳定性：

```
托盘 Physics Material:
  静摩擦系数: 0.5 (初始值，后续域随机化 [0.02, 0.1])
  动摩擦系数: 0.3

夹爪 Physics Material:
  静摩擦系数: 0.8 (橡胶垫)
  动摩擦系数: 0.6
```

#### 4.3.3 Collision Filter（可选）

如果开启了 Allow Self-Collision（导入时未勾选），可以通过 Collision Group 精细控制：
- 同一只手臂的相邻 link 之间：禁止碰撞（避免关节处爆炸）
- 左臂 ↔ 右臂：允许碰撞（防止递交时穿模）

---

## 五、任务 C：轨迹示教录制脚本

### 5.1 示教方式

通过 OCS2 在 RViz2 中拖动 Interactive Marker（末端目标位姿），手臂跟随运动。同时录制整个过程中所有关节角度、夹爪状态、时间戳。

### 5.2 录制内容

每个时刻记录：

```python
record = {
    "timestamp": float,           # 仿真时间 (秒)
    "phase": str,                 # "A1_right_place" / "A2_handover" / "B_move" / "C1_right_grab" ...
    "left_arm": [7 floats],       # left_joint1-7 位置 (rad)
    "right_arm": [7 floats],      # right_joint1-7 位置 (rad)
    "left_gripper": float,        # left_PGIA_joint1 位置 (m)
    "right_gripper": float,       # right_PGIA_joint1 位置 (m)
    "left_ee_pose": {             # 左末端位姿 (来自 /left_current_pose)
        "position": [x, y, z],
        "orientation": [qx, qy, qz, qw]
    },
    "right_ee_pose": {            # 右末端位姿
        "position": [x, y, z],
        "orientation": [qx, qy, qz, qw]
    },
    "base_pose": {                # 底盘位姿 (来自 /odom)
        "position": [x, y, z],
        "orientation": [qx, qy, qz, qw]
    },
}
```

### 5.3 脚本功能需求

```
核心功能：
  [1] 实时录制: 订阅 /joint_states + /left_current_pose + /right_current_pose + /odom
  [2] 暂停/继续: 按键暂停录制（调整 Marker 位置时不录入无效数据）
  [3] 阶段标记: 手动标记当前处于哪个阶段（A1/A2/A3/B/C1/C2/C3）
  [4] 夹爪触发: 手动发送夹爪开/闭指令并记录
  [5] 关键帧标记: 标记当前帧为关键帧（如"放置点"、"递交点"）
  [6] 导出: 保存为 YAML 和 CSV 两种格式
  [7] 回放: 读取录制文件，按时间戳回放关节轨迹到 Isaac Sim
  [8] 可视化: 录制结束后生成关节角度-时间曲线图

交互命令（键盘）：
  空格     — 暂停/继续录制
  1-7      — 标记阶段 (A1/A2/A3/A4/B/C1/C2/C3)
  g        — 切换当前活跃臂的夹爪（开↔闭）
  k        — 标记当前帧为关键帧
  s        — 保存当前录制
  p        — 回放已保存的轨迹
  q        — 退出
```

### 5.4 完整工作循环的阶段定义

```
阶段 A: 放置循环（输入传送带）
  A1: 右臂持托盘1 → 传送带入口放置
      右臂运动 + 右夹爪闭合→张开
  A2: 左臂持托盘2 → 递交区
      右臂 → 递交区（张开等待）
      左臂 → 递交区（持托盘）
  A3: 双臂递交
      右夹爪张开→闭合（接住托盘2）
      左夹爪闭合→张开（松手）
  A4: 右臂放托盘2 → 传送带
      左臂 → 初始姿态
      右臂 → 传送带入口 + 右夹爪闭合→张开
  A5: 双臂回初始姿态

阶段 B: 底盘平移
  B1: 底盘 Y 方向平移到输出传送带位置

阶段 C: 抓取循环（输出传送带）
  C1: 右臂 → 输出传送带抓取托盘1
      右夹爪张开→闭合
  C2: 双臂递交（右→左）
      左夹爪张开→闭合
      右夹爪闭合→张开
  C3: 右臂 → 输出传送带抓取托盘2
      右夹爪张开→闭合
```

### 5.5 录制数据用途

```
1. 轨迹数据 → OCS2 目标序列 (TargetTrajectories)
   提取各阶段关键帧的末端位姿作为 MPC 的参考目标

2. 轨迹数据 → 强化学习 demo 轨迹
   作为 HIL-SERL 的 demo buffer 初始数据

3. 轨迹数据 → 碰撞检测验证
   回放轨迹，确认无穿模/碰撞异常

4. 轨迹数据 → Sim2Real 对比基准
   仿真轨迹 vs 实机轨迹的偏差分析
```

---

## 六、实施顺序

```
Week 1: 夹爪联调
  Day 1: 验证 Gripper_Control_Graph 连线和话题
  Day 2: 验证 Mimic 联动 + 夹爪 Stiffness 调参
  Day 3: 手动抓取/释放测试（pub 指令 → 观察 Isaac Sim）

Week 1-2: 碰撞检测
  Day 4: 检查并补全夹爪 link 的 Collision API
  Day 5: 设置 Physics Material（摩擦系数）
  Day 6: 测试夹爪-托盘抓取物理效果

Week 2-3: 轨迹示教
  Day 7-8: 开发录制脚本（核心功能）
  Day 9: 录制阶段 A 完整轨迹
  Day 10: 录制阶段 B+C 轨迹
  Day 11: 回放验证 + 轨迹优化
  Day 12-14: 导出关键帧 → OCS2 TargetTrajectories
```

---

## 七、启动流程（供参考）

```bash
# 1. Isaac Sim → 打开 Stage → Play
# 2. 确认话题
ros2 topic hz /joint_states
ros2 topic hz /clock

# 3. 启动 OCS2 闭环
source ~/ros2_ws/install/setup.bash
ros2 launch r1_description ocs2_isaac.launch.py

# 4. 等待 OCS2 初始化完成
# 日志应显示: "activate hardware 'r1_arm_isaac'"
# 日志应显示: "ocs2_arm_controller active"

# 5. 初始化目标位姿（零误差启动）
ros2 topic echo --once /left_current_pose
ros2 topic echo --once /right_current_pose
# 将当前位姿发到 /left_target/stamped 和 /right_target/stamped

# 6. 切入 OCS2 模式
ros2 topic pub --once /fsm_command std_msgs/msg/Int32 "{data: 3}"

# 7. 小幅拖动 RViz Marker 确认手臂跟随
```

---

## 八、阶段1已解决的问题清单（供背景参考）

共 18 个问题，详见 `OCS2_IsaacSim_Complete_Debug_Report.md`，此处仅列标题：

1. URDF 手臂 Link 无惯性数据
2. trunk_link 惯量单位错误
3. D455 相机惯量偏大
4. waist/body_joint effort=0
5. 关节位置控制飞出（Damping 不足）
6. left/right_flange_joint 重复 origin
7. head_camera/lidar joint → fixed
8. use_sim_time 导致 spawner 超时
9. libarms_target_manager.so 找不到
10. URDF mesh 路径硬编码
11. RViz TF 重映射导致全断
12. TF 双源冲突（Isaac Sim + robot_state_publisher）
13. odom_to_tf 时间戳 sec=0
14. world→base_link 断裂
15. robot.xacro 加载 mock 而非 isaac
16. FSM 停在 HOLD（/arm_joint_cmd 有数据但不动）
17. RViz Marker → target 断链
18. task.info 非法语法导致控制器加载失败
