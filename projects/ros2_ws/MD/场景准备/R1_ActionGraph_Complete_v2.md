# R1 Isaac Sim 5.1 — Action Graph 完整重构指南 v2

> 前置问题修复 + 4 个 Action Graph 完整节点 I/O 规格  
> 机器人: R1 双臂 (RM-75 ×2 + PGIA 夹爪) + 3 个 RealSense D455 相机  
> 环境: Isaac Sim 5.1 Docker, ROS2 Humble

---

## 目录

0. [前置问题: STL Mesh 灰色修复](#0-前置问题-stl-mesh-灰色修复)
1. [Graph 1 — Arm_Control_Graph (手臂控制)](#1-graph-1--arm_control_graph)
2. [Graph 2 — Gripper_Control_Graph (夹爪控制)](#2-graph-2--gripper_control_graph)
3. [Graph 3 — State_Telemetry_Graph (状态回传+TF)](#3-graph-3--state_telemetry_graph)
4. [Graph 4 — Camera_Publish_Graph (3 相机 RGB/Depth)](#4-graph-4--camera_publish_graph)
5. [Python 一键生成脚本](#5-python-一键生成脚本)
6. [验证方法](#6-验证方法)

---

## 0. 前置问题: STL Mesh 灰色修复

你截图里看到的灰色 mesh (`node_STL_BINARY_` → `Looks` → `mesh`) 是 URDF 导入时 STL 文件的引用路径断裂。

### 原因

Isaac Sim 5.1 的 URDF Importer 在导入 `Referenced Model` 模式下，会将 STL 转换为 USD 并保存在与 URDF 同目录下。但你的 URDF 里的 mesh 路径可能使用了 `package://` 前缀或相对路径，导入后 USD 中的 reference 指向了一个不存在的位置。

### 修复方法

**方法 A — 重新导入 (推荐)**

1. 打开 Isaac Sim → File → Import
2. 选择 `r1_pgia_ros2_control.urdf` (带 ros2_control tag 的版本)
3. 在导入面板中配置:
   - **Model**: `Create in Stage` (而不是 Referenced Model)
   - **Links**: `Moveable Base` (你的 R1 有轮式底座!)
   - **Default Density**: `1000.0` kg/m³
   - **Joint Configuration**: `Natural Frequency` → Drive Type: `Force`
   - **Colliders**: 勾选 `Collision From Visuals` → `Convex Decomposition`
4. **关键**: 在导入面板底部的 **ROS Package List** 中，添加一行:
   - Package Name: `r1_robot_description` (或你 URDF 中 package:// 后面的名字)
   - Package Path: `/home/gtk/Desktop/Robot` (你的机器人描述包所在路径)
5. 点击 Import

**方法 B — 手动修复 USD 引用 (如果不想重新导入)**

在 Isaac Sim Script Editor 中运行:

```python
import omni.usd
from pxr import Usd, UsdGeom, Sdf

stage = omni.usd.get_context().get_stage()

# 遍历所有 mesh prim, 检查 reference
for prim in stage.Traverse():
    if prim.GetTypeName() == "Mesh":
        refs = prim.GetReferences()
        # 如果引用无效, 打印路径
        print(f"Prim: {prim.GetPath()}")
```

然后手动在属性面板中更新断裂的引用路径, 指向 `/home/gtk/Desktop/Robot/meshes/` 下的对应 STL 文件。

**方法 C — 使用已导出的 USD (如果你之前导出过)**

你的 tree 中已经有 `urdf/r1_pgia/r1_pgia.usd` 和 `urdf/r1_pgia_ros2_control/r1_pgia_ros2_control.usd`。这些是导入器之前生成的 USD 文件。你可以直接:

1. File → Open → 选择 `r1_pgia_ros2_control.usd`
2. 检查 mesh 是否正常显示
3. 如果仍然灰色, 说明 USD 内部的 STL reference 也断了, 需要用方法 A 重新导入

### 导入后验证

- Stage 树中每个 link 下的 `visuals` → `mesh` 应该显示彩色 (如果 STL 有颜色) 或白色 (STL 无颜色, 正常)
- STL 格式本身不携带纹理, 只有几何体。如果你的 `.dae` 文件有颜色, 用 `.dae` mesh 版本的 URDF 导入效果更好

```
# 在 Isaac Sim Script Editor 中运行
# 作用: 将所有 instanceable mesh 解除实例化，让 STL 直接嵌入 Stage

import omni.usd
from pxr import Usd

stage = omni.usd.get_context().get_stage()
count = 0
for prim in stage.Traverse():
    if prim.IsInstanceable():
        prim.SetInstanceable(False)
        count += 1
        print(f"解除实例化: {prim.GetPath()}")

print(f"\n共解除 {count} 个 instanceable prim")
print("完成。请检查 viewport 中 mesh 是否变为白色实体。")
```



---

## 1. Graph 1 — Arm_Control_Graph

**功能**: 接收 ROS2 关节指令, 驱动左右两条 RM-75 机械臂

### 节点清单 (4 个节点)

| # | 节点名 (Stage 中显示) | 节点类型 (Isaac Sim 5.1) | 功能 |
|---|---|---|---|
| 1 | `on_playback_tick` | `omni.graph.action.OnPlaybackTick` | 仿真每帧触发 |
| 2 | `ros2_context` | `isaacsim.ros2.bridge.ROS2Context` | ROS2 域配置 |
| 3 | `subscribe_joint_state` | `isaacsim.ros2.bridge.ROS2SubscribeJointState` | 订阅关节指令 |
| 4 | `articulation_controller` | `isaacsim.core.nodes.IsaacArticulationController` | 驱动关节 |

### 各节点输入/输出规格

#### 节点 1: `on_playback_tick`

| 引脚方向 | 引脚名 | 类型 | 说明 |
|---------|--------|------|------|
| Output | `Tick` | execution | 每仿真帧触发一次 |
| Output | `Delta Seconds` | double | 距上一帧的时间差 |
| Output | `Frame` | int64 | 当前帧号 |
| Output | `Time` | double | 仿真时间 |

#### 节点 2: `ros2_context`

| 引脚方向 | 引脚名 | 类型 | 值 |
|---------|--------|------|-----|
| Input | `Domain Id` | uint8 | `0` (默认) |
| Input | `Use Domain ID Env Var` | bool | `false` |
| Output | `Context` | target | ROS2 上下文句柄 |

#### 节点 3: `subscribe_joint_state`

| 引脚方向 | 引脚名 | 类型 | 值/来源 |
|---------|--------|------|---------|
| Input | `Exec In` | execution | ← `on_playback_tick.Tick` |
| Input | `Context` | target | ← `ros2_context.Context` |
| Input | `Node Namespace` | string | `""` |
| Input | `Topic Name` | string | **`/arm_joint_cmd`** |
| Input | `Qos Profile` | string | `""` (默认) |
| Input | `Queue Size` | int | `10` |
| Output | `Exec Out` | execution | → `articulation_controller.Exec In` |
| Output | `Joint Names` | token[] | → `articulation_controller.Joint Names` |
| Output | `Position Command` | double[] | → `articulation_controller.Position Command` |
| Output | `Velocity Command` | double[] | (不连接) |
| Output | `Effort Command` | double[] | (不连接) |
| Output | `Timestamp` | double | (不连接) |

#### 节点 4: `articulation_controller`

| 引脚方向 | 引脚名 | 类型 | 值/来源 |
|---------|--------|------|---------|
| Input | `Exec In` | execution | ← `subscribe_joint_state.Exec Out` |
| Input | `Joint Names` | token[] | ← `subscribe_joint_state.Joint Names` |
| Input | `Position Command` | double[] | ← `subscribe_joint_state.Position Command` |
| Input | `Velocity Command` | double[] | (不连接) |
| Input | `Effort Command` | double[] | (不连接) |
| Input | `Joint Indices` | int[] | (不连接, 用 Joint Names 替代) |
| Input | `Robot Path` | string | **`/World/Robot`** (在属性面板设置) |
| Input | `Target Prim` | target | (用 Robot Path 替代, 二选一) |

### 连线清单

```
连线 1: on_playback_tick.Tick        → subscribe_joint_state.Exec In
连线 2: ros2_context.Context         → subscribe_joint_state.Context
连线 3: subscribe_joint_state.Exec Out       → articulation_controller.Exec In
连线 4: subscribe_joint_state.Joint Names    → articulation_controller.Joint Names
连线 5: subscribe_joint_state.Position Command → articulation_controller.Position Command
```

### ⚠️ 重要说明: Topic 名称

你的 `ros2_controllers.yaml` 使用 `JointTrajectoryController`, 它的 topic 是 action server (`/left_arm_controller/follow_joint_trajectory`), 不是普通 topic。Isaac Sim 的 `SubscribeJointState` 节点只能接收 `sensor_msgs/JointState` 格式的普通 topic。

**解决方案**: 在 ROS2 侧运行一个 bridge node, 将 OCS2 的输出转发为 `sensor_msgs/JointState` 到 `/arm_joint_cmd`:

```python
#!/usr/bin/env python3
"""
ocs2_to_isaac_bridge.py
将 OCS2 arm controller 的输出转发为 Isaac Sim 可接收的 JointState
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

class OCS2ToIsaacBridge(Node):
    def __init__(self):
        super().__init__('ocs2_isaac_bridge')
        
        # 订阅 OCS2 输出的轨迹
        self.sub_left = self.create_subscription(
            JointTrajectory,
            '/left_arm_controller/joint_trajectory',
            lambda msg: self.traj_cb(msg, 'left'),
            10
        )
        self.sub_right = self.create_subscription(
            JointTrajectory,
            '/right_arm_controller/joint_trajectory',
            lambda msg: self.traj_cb(msg, 'right'),
            10
        )
        
        # 发布合并后的 JointState 给 Isaac Sim
        self.pub = self.create_publisher(JointState, '/arm_joint_cmd', 10)
        
        self.left_cmd = None
        self.right_cmd = None
        
        # 腰部控制
        self.sub_torso = self.create_subscription(
            JointTrajectory,
            '/torso_controller/joint_trajectory',
            self.torso_cb,
            10
        )
        self.torso_cmd = None
        
        # 定时发布 (100Hz, 匹配 update_rate)
        self.timer = self.create_timer(0.01, self.publish_cmd)
    
    def traj_cb(self, msg, side):
        if msg.points:
            point = msg.points[0]  # 取第一个轨迹点
            if side == 'left':
                self.left_cmd = (msg.joint_names, point.positions)
            else:
                self.right_cmd = (msg.joint_names, point.positions)
    
    def torso_cb(self, msg):
        if msg.points:
            point = msg.points[0]
            self.torso_cmd = (msg.joint_names, point.positions)
    
    def publish_cmd(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        
        for cmd in [self.left_cmd, self.right_cmd, self.torso_cmd]:
            if cmd:
                js.name.extend(cmd[0])
                js.position.extend(cmd[1])
        
        if js.name:
            self.pub.publish(js)

def main():
    rclpy.init()
    node = OCS2ToIsaacBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 2. Graph 2 — Gripper_Control_Graph

**功能**: 接收夹爪位置指令, 左右并联在同一个 Graph 中

连线：

```
on_playback_tick.Tick           → ros2_subscriber.Exec In
ros2_context.Context            → ros2_subscriber.Context
ros2_subscriber.Exec Out        → articulation_controller.Exec In
ros2_subscriber.data            → articulation_controller.Position Command
```

`articulation_controller` 属性面板里手动填：

- `Robot Path`: `/World/Robot`
- `Joint Names`: `left_PGIA_joint1`（左）/ `right_PGIA_joint1`（右）

从你的截图看到 `right_PGIA_joint2 = Mimic`。这意味着:
- `PGIA_joint1` = 主动关节 (你控制这个)
- `PGIA_joint2` = 从动关节 (PhysX 自动让它跟随 joint1)
- 你只需要向 `joint1` 发指令, `joint2` 会自动联动
- 夹爪行程: `[0.00, 0.04]` m (闭合到张开)

---

## 3. Graph 3 — State_Telemetry_Graph

**功能**: 发布关节状态、TF 树、仿真时钟给 ROS2 侧

### 节点清单 (5 个节点)

| # | 节点名 | 节点类型 | 功能 |
|---|--------|---------|------|
| 1 | `on_playback_tick` | `omni.graph.action.OnPlaybackTick` | 触发 |
| 2 | `ros2_context` | `isaacsim.ros2.bridge.ROS2Context` | ROS2 域 |
| 3 | `read_sim_time` | `isaacsim.core.nodes.IsaacReadSimulationTime` | 读仿真时间 |
| 4 | `publish_joint_state` | `isaacsim.ros2.bridge.ROS2PublishJointState` | 发布 /joint_states |
| 5 | `publish_tf` | `isaacsim.ros2.bridge.ROS2PublishTransformTree` | 发布 /tf |

### 各节点输入/输出规格

#### 节点 3: `read_sim_time`

| 引脚方向 | 引脚名 | 类型 | 说明 |
|---------|--------|------|------|
| Input | `Reference Time Denominator` | int | `1` (默认) |
| Input | `Reference Time Numerator` | int | `0` (默认) |
| Input | `Reset On Stop` | bool | `false` |
| Output | `Simulation Time` | double | 当前仿真时间戳 |

#### 节点 4: `publish_joint_state`

| 引脚方向 | 引脚名 | 类型 | 值/来源 |
|---------|--------|------|---------|
| Input | `Exec In` | execution | ← `on_playback_tick.Tick` |
| Input | `Context` | target | ← `ros2_context.Context` |
| Input | `Timestamp` | double | ← `read_sim_time.Simulation Time` |
| Input | `Topic Name` | string | **`/joint_states`** |
| Input | `Target Prim` | target | **`/World/Robot`** (属性面板选择) |
| Input | `Queue Size` | int | `10` |

#### 节点 5: `publish_tf`

| 引脚方向 | 引脚名 | 类型 | 值/来源 |
|---------|--------|------|---------|
| Input | `Exec In` | execution | ← `on_playback_tick.Tick` |
| Input | `Context` | target | ← `ros2_context.Context` |
| Input | `Timestamp` | double | ← `read_sim_time.Simulation Time` |
| Input | `Topic Name` | string | **`/tf`** |
| Input | `Target Prims` | target[] | 属性面板添加以下 prim: |
| | | | `/World/Robot/base_link` |
| | | | `/World/Robot/left_flange_link` |
| | | | `/World/Robot/right_flange_link` |
| | | | `/World/Robot/camera_H_link` |
| | | | `/World/Robot/camera_L_link` |
| | | | `/World/Robot/camera_R_link` |
| Input | `Static Publisher` | bool | `false` (动态 TF) |
| Input | `Parent Prim` | target | (留空, 自动推断) |

### 连线清单

```
连线 1: on_playback_tick.Tick            → publish_joint_state.Exec In
连线 2: on_playback_tick.Tick            → publish_tf.Exec In
连线 3: ros2_context.Context             → publish_joint_state.Context
连线 4: ros2_context.Context             → publish_tf.Context
连线 5: read_sim_time.Simulation Time    → publish_joint_state.Timestamp
连线 6: read_sim_time.Simulation Time    → publish_tf.Timestamp
```

---

## 4. Graph 4 — Camera_Publish_Graph

先确认一个关键问题：你的文档方案用的是**同一个CameraSensor prim**同时发布RGB和Depth，这是**错误的**。

原因：Isaac Sim的`ROS2CameraHelper`的`type`参数决定渲染管线，一个prim只能对应一种输出。Depth需要挂在`depth_frame`下的独立prim，RGB挂在`color_frame`下。

------

## 修订后的完整方案

### Prim结构（每个相机建两个prim）

```
camera_H_link/
├── camera_H_color_frame/
│   └── RGBCamera          ← RGB参数
└── camera_H_depth_frame/
    └── DepthCamera         ← Depth参数
```

------

### 第一步：创建所有Camera Prim（脚本）

```python
import omni.usd
import omni.kit.commands
from pxr import UsdGeom, Gf, Sdf

def create_d455_camera_prims(robot_base: str, camera_prefix: str):
    stage = omni.usd.get_context().get_stage()
    assert stage, "Stage not found"

    specs = {
        "RGBCamera": {
            # 挂在 color_frame 下
            "frame":              f"{robot_base}/{camera_prefix}_link/{camera_prefix}_color_frame",
            "focalLength":        1.93,
            "horizontalAperture": 3.68,   # HFOV≈90°
            "verticalAperture":   2.30,   # VFOV≈65°
            "clippingRange":      Gf.Vec2f(0.3, 10.0),
        },
        "DepthCamera": {
            # 挂在 depth_frame 下（D455深度光学中心不同于color中心）
            "frame":              f"{robot_base}/{camera_prefix}_link/{camera_prefix}_depth_frame",
            "focalLength":        1.88,
            "horizontalAperture": 3.44,   # HFOV≈87°
            "verticalAperture":   2.06,   # VFOV≈58°
            "clippingRange":      Gf.Vec2f(0.52, 6.0),  # D455 Min-Z=52cm!
        },
    }

    results = {}
    for prim_name, cfg in specs.items():
        target_path = f"{cfg['frame']}/{prim_name}"
        prim_path = Sdf.Path(target_path)

        if stage.GetPrimAtPath(prim_path).IsValid():
            omni.kit.commands.execute('DeletePrims', paths=[str(prim_path)])

        cam = UsdGeom.Camera.Define(stage, prim_path)
        assert cam, f"Failed: {target_path}"

        cam.GetFocalLengthAttr().Set(cfg["focalLength"])
        cam.GetHorizontalApertureAttr().Set(cfg["horizontalAperture"])
        cam.GetVerticalApertureAttr().Set(cfg["verticalAperture"])
        cam.GetClippingRangeAttr().Set(cfg["clippingRange"])
        cam.GetProjectionAttr().Set(UsdGeom.Tokens.perspective)

        xform = UsdGeom.Xformable(cam.GetPrim())
        xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0, 0, 0))
        xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))

        results[prim_name] = target_path
        print(f"[OK] {camera_prefix}/{prim_name} → {target_path}")

    return results

# 对三个相机执行（robot_base改为你的实际路径）
ROBOT_BASE = "/World/Robot"
for cam in ["camera_H", "camera_L", "camera_R"]:
    create_d455_camera_prims(ROBOT_BASE, cam)
```

------

### 第二步：OmniGraph — Camera_Publish_Graph

RGB和Depth必须**分开prim、分开节点**，共**14个节点**（1 Tick + 1 Context + 6×2 CameraHelper）。

#### 节点清单

| #    | 节点名             | 类型                                    | 说明      |
| ---- | ------------------ | --------------------------------------- | --------- |
| 1    | `on_playback_tick` | `omni.graph.action.OnPlaybackTick`      | 触发源    |
| 2    | `ros2_context`     | `isaacsim.ros2.bridge.ROS2Context`      | ROS2域    |
| 3    | `cam_head_rgb`     | `isaacsim.ros2.bridge.ROS2CameraHelper` | 头部RGB   |
| 4    | `cam_head_depth`   | `isaacsim.ros2.bridge.ROS2CameraHelper` | 头部Depth |
| 5    | `cam_left_rgb`     | `isaacsim.ros2.bridge.ROS2CameraHelper` | 左腕RGB   |
| 6    | `cam_left_depth`   | `isaacsim.ros2.bridge.ROS2CameraHelper` | 左腕Depth |
| 7    | `cam_right_rgb`    | `isaacsim.ros2.bridge.ROS2CameraHelper` | 右腕RGB   |
| 8    | `cam_right_depth`  | `isaacsim.ros2.bridge.ROS2CameraHelper` | 右腕Depth |

#### 6个CameraHelper完整参数表

| 节点              | cameraPrim                             | topicName                         | type    | frameId                        | width | height |
| ----------------- | -------------------------------------- | --------------------------------- | ------- | ------------------------------ | ----- | ------ |
| `cam_head_rgb`    | `.../camera_H_color_frame/RGBCamera`   | `/head_cam/color/image_raw`       | `rgb`   | `camera_H_color_optical_frame` | 1280  | 800    |
| `cam_head_depth`  | `.../camera_H_depth_frame/DepthCamera` | `/head_cam/depth/image_rect_raw`  | `depth` | `camera_H_depth_optical_frame` | 1280  | 720    |
| `cam_left_rgb`    | `.../camera_L_color_frame/RGBCamera`   | `/left_cam/color/image_raw`       | `rgb`   | `camera_L_color_optical_frame` | 1280  | 800    |
| `cam_left_depth`  | `.../camera_L_depth_frame/DepthCamera` | `/left_cam/depth/image_rect_raw`  | `depth` | `camera_L_depth_optical_frame` | 1280  | 720    |
| `cam_right_rgb`   | `.../camera_R_color_frame/RGBCamera`   | `/right_cam/color/image_raw`      | `rgb`   | `camera_R_color_optical_frame` | 1280  | 800    |
| `cam_right_depth` | `.../camera_R_depth_frame/DepthCamera` | `/right_cam/depth/image_rect_raw` | `depth` | `camera_R_depth_optical_frame` | 1280  | 720    |

> topic命名遵循RealSense ROS2 wrapper惯例，方便后续与真实机器人对齐。

#### 连线

```
on_playback_tick.Tick → cam_head_rgb.Exec In
on_playback_tick.Tick → cam_head_depth.Exec In
on_playback_tick.Tick → cam_left_rgb.Exec In
on_playback_tick.Tick → cam_left_depth.Exec In
on_playback_tick.Tick → cam_right_rgb.Exec In
on_playback_tick.Tick → cam_right_depth.Exec In

ros2_context.Context → cam_head_rgb.Context
ros2_context.Context → cam_head_depth.Context
ros2_context.Context → cam_left_rgb.Context
ros2_context.Context → cam_left_depth.Context
ros2_context.Context → cam_right_rgb.Context
ros2_context.Context → cam_right_depth.Context
```

------

### 关于`IsaacSensorCreateRenderProduct`的结论

**不需要手动添加**。`ROS2CameraHelper`在Play时会自动创建RenderProduct，你之前看到的那个`omni.isaac.ros2_bridge`的Python API在Isaac Sim 5.1中路径已变更，且功能已被OmniGraph节点完全替代。直接用OmniGraph节点即可，不要用那段Python API代码。

------

### 验证命令

运行仿真后在服务器执行：

```bash
sudo docker exec -u root isaac-sim bash -c "
  source /opt/ros/humble/setup.bash &&
  ros2 topic list | grep cam
"
```

预期看到6条topic：

```
/head_cam/color/image_raw
/head_cam/depth/image_rect_raw
/left_cam/color/image_raw
/left_cam/depth/image_rect_raw
/right_cam/color/image_raw
/right_cam/depth/image_rect_raw
```

## 5. Python 一键生成脚本

将以下脚本粘贴到 Isaac Sim Script Editor 中运行, 一次性创建所有 4 个 Graph:

```python
import omni.graph.core as og

keys = og.Controller.Keys

# ============================
# Graph 1: Arm Control
# ============================
og.Controller.edit(
    {"graph_path": "/World/Arm_Control_Graph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("tick", "omni.graph.action.OnPlaybackTick"),
            ("context", "isaacsim.ros2.bridge.ROS2Context"),
            ("sub_arm", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            ("artic_arm", "isaacsim.core.nodes.IsaacArticulationController"),
        ],
        keys.SET_VALUES: [
            ("context.inputs:domain_id", 0),
            ("sub_arm.inputs:topicName", "/arm_joint_cmd"),
            ("artic_arm.inputs:usePath", True),
            ("artic_arm.inputs:robotPath", "/World/Robot"),
        ],
        keys.CONNECT: [
            ("tick.outputs:tick", "sub_arm.inputs:execIn"),
            ("context.outputs:context", "sub_arm.inputs:context"),
            ("sub_arm.outputs:execOut", "artic_arm.inputs:execIn"),
            ("sub_arm.outputs:jointNames", "artic_arm.inputs:jointNames"),
            ("sub_arm.outputs:positionCommand", "artic_arm.inputs:positionCommand"),
        ],
    },
)

# ============================
# Graph 2: Gripper Control
# ============================
og.Controller.edit(
    {"graph_path": "/World/Gripper_Control_Graph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("tick", "omni.graph.action.OnPlaybackTick"),
            ("context", "isaacsim.ros2.bridge.ROS2Context"),
            # Left gripper
            ("sub_left", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            ("artic_left", "isaacsim.core.nodes.IsaacArticulationController"),
            # Right gripper
            ("sub_right", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            ("artic_right", "isaacsim.core.nodes.IsaacArticulationController"),
        ],
        keys.SET_VALUES: [
            ("context.inputs:domain_id", 0),
            # Left
            ("sub_left.inputs:topicName", "/left_gripper_cmd"),
            ("artic_left.inputs:usePath", True),
            ("artic_left.inputs:robotPath", "/World/Robot"),
            # Right
            ("sub_right.inputs:topicName", "/right_gripper_cmd"),
            ("artic_right.inputs:usePath", True),
            ("artic_right.inputs:robotPath", "/World/Robot"),
        ],
        keys.CONNECT: [
            # Left
            ("tick.outputs:tick", "sub_left.inputs:execIn"),
            ("context.outputs:context", "sub_left.inputs:context"),
            ("sub_left.outputs:execOut", "artic_left.inputs:execIn"),
            ("sub_left.outputs:jointNames", "artic_left.inputs:jointNames"),
            ("sub_left.outputs:positionCommand", "artic_left.inputs:positionCommand"),
            # Right
            ("tick.outputs:tick", "sub_right.inputs:execIn"),
            ("context.outputs:context", "sub_right.inputs:context"),
            ("sub_right.outputs:execOut", "artic_right.inputs:execIn"),
            ("sub_right.outputs:jointNames", "artic_right.inputs:jointNames"),
            ("sub_right.outputs:positionCommand", "artic_right.inputs:positionCommand"),
        ],
    },
)

# ============================
# Graph 3: State Telemetry
# ============================

# 注意: set_target_prims 用于设置 Target Prim 类型的属性
from isaacsim.core.nodes.scripts.utils import set_target_prims

og.Controller.edit(
    {"graph_path": "/World/State_Telemetry_Graph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("tick", "omni.graph.action.OnPlaybackTick"),
            ("context", "isaacsim.ros2.bridge.ROS2Context"),
            ("read_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("pub_js", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("pub_tf", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
        ],
        keys.SET_VALUES: [
            ("context.inputs:domain_id", 0),
            ("pub_js.inputs:topicName", "/joint_states"),
            ("pub_tf.inputs:topicName", "/tf"),
        ],
        keys.CONNECT: [
            ("tick.outputs:tick", "pub_js.inputs:execIn"),
            ("tick.outputs:tick", "pub_tf.inputs:execIn"),
            ("context.outputs:context", "pub_js.inputs:context"),
            ("context.outputs:context", "pub_tf.inputs:context"),
            ("read_time.outputs:simulationTime", "pub_js.inputs:timeStamp"),
            ("read_time.outputs:simulationTime", "pub_tf.inputs:timeStamp"),
        ],
    },
)

# Set target prims for joint state publisher
set_target_prims(
    primPath="/World/State_Telemetry_Graph/pub_js",
    inputName="inputs:targetPrim",
    targetPrimPaths=["/World/Robot"],
)

# Set target prims for TF publisher
set_target_prims(
    primPath="/World/State_Telemetry_Graph/pub_tf",
    inputName="inputs:targetPrims",
    targetPrimPaths=[
        "/World/Robot/base_link",
        "/World/Robot/left_flange_link",
        "/World/Robot/right_flange_link",
        "/World/Robot/camera_H_link",
        "/World/Robot/camera_L_link",
        "/World/Robot/camera_R_link",
    ],
)

# ============================
# Graph 4: Camera Publishing
# ============================

# 你需要先确认 Camera Prim 的精确路径, 替换下面的占位符
# 如果你还没有在 color_frame 下创建 Camera prim, 先手动创建

CAMERAS = [
    {
        "name": "head",
        # TODO: 替换为你的实际 Camera Prim 路径
        "prim": "/World/Robot/camera_H_link/camera_H_color_frame/CameraSensor",
        "rgb_topic": "/head_cam/rgb",
        "depth_topic": "/head_cam/depth",
        "frame_id": "camera_H_color_frame",
    },
    {
        "name": "left",
        "prim": "/World/Robot/left_flange_link/camera_L_link/camera_L_color_frame/CameraSensor",
        "rgb_topic": "/left_cam/rgb",
        "depth_topic": "/left_cam/depth",
        "frame_id": "camera_L_color_frame",
    },
    {
        "name": "right",
        "prim": "/World/Robot/right_flange_link/camera_R_link/camera_R_color_frame/CameraSensor",
        "rgb_topic": "/right_cam/rgb",
        "depth_topic": "/right_cam/depth",
        "frame_id": "camera_R_color_frame",
    },
]

cam_nodes = [
    ("tick", "omni.graph.action.OnPlaybackTick"),
    ("context", "isaacsim.ros2.bridge.ROS2Context"),
]
cam_values = [("context.inputs:domain_id", 0)]
cam_connects = []

for cam in CAMERAS:
    n = cam["name"]
    # RGB helper
    cam_nodes.append((f"{n}_rgb", "isaacsim.ros2.bridge.ROS2CameraHelper"))
    cam_values.extend([
        (f"{n}_rgb.inputs:topicName", cam["rgb_topic"]),
        (f"{n}_rgb.inputs:type", "rgb"),
        (f"{n}_rgb.inputs:frameId", cam["frame_id"]),
        (f"{n}_rgb.inputs:frameSkipCount", 5),
    ])
    cam_connects.extend([
        ("tick.outputs:tick", f"{n}_rgb.inputs:execIn"),
        ("context.outputs:context", f"{n}_rgb.inputs:context"),
    ])
    
    # Depth helper
    cam_nodes.append((f"{n}_depth", "isaacsim.ros2.bridge.ROS2CameraHelper"))
    cam_values.extend([
        (f"{n}_depth.inputs:topicName", cam["depth_topic"]),
        (f"{n}_depth.inputs:type", "depth"),
        (f"{n}_depth.inputs:frameId", cam["frame_id"]),
        (f"{n}_depth.inputs:frameSkipCount", 5),
    ])
    cam_connects.extend([
        ("tick.outputs:tick", f"{n}_depth.inputs:execIn"),
        ("context.outputs:context", f"{n}_depth.inputs:context"),
    ])

og.Controller.edit(
    {"graph_path": "/World/Camera_Publish_Graph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: cam_nodes,
        keys.SET_VALUES: cam_values,
        keys.CONNECT: cam_connects,
    },
)

# Set camera prims (must use set_target_prims for target type)
for cam in CAMERAS:
    n = cam["name"]
    for suffix in ["rgb", "depth"]:
        set_target_prims(
            primPath=f"/World/Camera_Publish_Graph/{n}_{suffix}",
            inputName="inputs:cameraPrim",
            targetPrimPaths=[cam["prim"]],
        )

print("=== All 4 Action Graphs created successfully ===")
```

---

## 6. 验证方法

### Step 1: 验证 State Telemetry (最先测)

```bash
# 按下 Isaac Sim 的 Play 按钮后:
ros2 topic echo /joint_states     # 应该看到所有关节名和位置数据
ros2 topic echo /tf               # 应该看到 base_link 等 frame 的变换
```

### Step 2: 验证 Arm Control

```bash
# 发布一个测试关节指令
ros2 topic pub --once /arm_joint_cmd sensor_msgs/msg/JointState \
  "{name: ['left_joint1'], position: [0.5]}"

# 机器人左臂第一个关节应该转动
```

### Step 3: 验证 Gripper Control

```bash
# 发布夹爪张开指令
ros2 topic pub --once /left_gripper_cmd sensor_msgs/msg/JointState \
  "{name: ['left_PGIA_joint1'], position: [0.04]}"

# 左夹爪应该张开
```

### Step 4: 验证 Camera

```bash
ros2 topic list | grep cam
# 应该看到:
# /head_cam/rgb
# /head_cam/depth
# /left_cam/rgb
# /left_cam/depth
# /right_cam/rgb
# /right_cam/depth

# 用 rqt 查看图像
ros2 run rqt_image_view rqt_image_view
```

### Topic 总览

| Topic | 类型 | 方向 | Graph |
|-------|------|------|-------|
| `/arm_joint_cmd` | sensor_msgs/JointState | ROS2 → Isaac | Graph 1 |
| `/left_gripper_cmd` | sensor_msgs/JointState | ROS2 → Isaac | Graph 2 |
| `/right_gripper_cmd` | sensor_msgs/JointState | ROS2 → Isaac | Graph 2 |
| `/joint_states` | sensor_msgs/JointState | Isaac → ROS2 | Graph 3 |
| `/tf` | tf2_msgs/TFMessage | Isaac → ROS2 | Graph 3 |
| `/head_cam/rgb` | sensor_msgs/Image | Isaac → ROS2 | Graph 4 |
| `/head_cam/depth` | sensor_msgs/Image | Isaac → ROS2 | Graph 4 |
| `/left_cam/rgb` | sensor_msgs/Image | Isaac → ROS2 | Graph 4 |
| `/left_cam/depth` | sensor_msgs/Image | Isaac → ROS2 | Graph 4 |
| `/right_cam/rgb` | sensor_msgs/Image | Isaac → ROS2 | Graph 4 |
| `/right_cam/depth` | sensor_msgs/Image | Isaac → ROS2 | Graph 4 |
