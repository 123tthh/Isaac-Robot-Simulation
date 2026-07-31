# 机器人 Isaac Sim 相机配置

> **项目背景**: 轮式双臂机器人（共享躯干 + 2×RM75-6FB）在 Isaac Sim 5.1 中进行物理仿真抓放托盘任务 ；机器人在驻车模式下使机械臂操作（机器人左右两个夹爪拿着托盘，
>
> 1、机器人基座运行到放置托盘位置1,左右手各自拿着一个托盘。放到传送带上（可行域内）（输入传送带只有一个入口，意味着左右机械臂留有足够空间，避免两个机械臂交叉/奇异）；
>
> 2、机器人轮式基座向左平移到输出传送带最佳位置，左右夹爪抓取新托盘（也是只有一个出口））；
>
> 3、夹爪电动（仅有开/关两个离散状态）：
>
> 完整工作循环： 阶段A：放置循环（输入传送带）  机器人驻车在输入传送带前  → 右臂先放托盘1到传送带（左臂等待）  → 左臂夹爪将托盘2给到右臂夹爪  → 左臂退出避让  → 右臂放托盘2到传送带  → 两个托盘都放完，左右手臂回到初始姿态 
>
> 阶段B：平移到输出传送带  轮式底盘向左平移  → 到达输出传送带最佳取托盘位置 
>
> 阶段C：抓取循环（输出传送带）  → 右臂夹爪抓取托盘1  → 左臂抓取右臂夹爪的托盘1  → 右臂抓取托盘2  → 返回阶段A 
>
> **控制架构**: ROS 2 + 修改版 OCS2 (Optimal Control for Switched Systems)  
> **传感器**: 3× Intel RealSense D455 (头部×1 + 左右腕各×1)  
> **任务流水线**: 仿真闭环 → 轨迹数据 → 强化学习训练 → Sim2Real 实机部署

---

## 1. 硬件规格参考

### 1.1 Intel RealSense D455

| 参数 | RGB 模块 | Depth 模块 |
|------|---------|-----------|
| 尺寸 | 124 × 26 × 29 mm | — |
| RGB↔Depth baseline | 59 mm | — |
| RGB 分辨率 | 1280×800 | — |
| Depth 分辨率 | 1280×720 | — |
| RGB FOV (H×V) | ~87° × ~58° | — |
| Depth FOV (H×V) | ~87° × ~58° | — |
| Depth 最小距离 | — | 0.52 m |
| Depth 最大距离 | — | 6 m |
| RGB clipping | 0.3 m – 10 m | — |
| 重量 | ~120 g（整体） | — |

### 1.2 安装位置

| 相机 | 挂载节点 | 朝向 | 用途 |
|------|---------|------|------|
| Head (H) | `head_camera_link` → `camera_H_link` | 前方偏下 ~5.5° | 全局感知，托盘粗定位 |
| Left (L) | `left_PGIA_link` → `camera_L_link` | 随左臂末端 | 左夹爪抓取精定位 |
| Right (R) | `right_PGIA_link` → `camera_R_link` | 随右臂末端 | 右夹爪抓取精定位 |

---

## 2. Isaac Sim 中的相机配置

### 2.1 USD 层级结构

每个 D455 在 Stage 中表现为以下层级（以头部为例）：

```
/World/Robot/
  head_camera_link              ← URDF link (相机安装基座)
  camera_H_link                 ← URDF link (相机本体, fixed joint 连到 head_camera_link)
    camera_H_color_frame        ← fixed joint 子 frame (RGB 光心位置)
      RGBCamera                 ← Camera prim (手动创建)
    camera_H_depth_frame        ← fixed joint 子 frame (Depth 光心位置, 偏移 59mm)
      DepthCamera               ← Camera prim (手动创建)
    camera_H_color_optical_frame   ← optical frame (用于 TF)
    camera_H_depth_optical_frame   ← optical frame (用于 TF)
```

**重要**: Isaac Sim URDF Importer 会将 fixed joint 连接的 link 展平为同级 prim（`head_camera_link` 和 `camera_H_link` 在 Stage 中看起来并列），但物理层级关系仍然保留。

### 2.2 相机 Prim 参数

#### RGB Camera (3 个, 参数相同)

| 参数 | 值 | 说明 |
|------|----|------|
| focal_length | 1.93 mm | Isaac Sim 焦距 |
| horizontal_aperture | 3.68 mm | 对应 HFOV ≈ 87.26° |
| vertical_aperture | 2.30 mm | 对应 VFOV ≈ 61.58° |
| clipping_range | [0.3, 10.0] m | |
| 分辨率 | 1280 × 800 | 匹配 D455 RGB 原生分辨率 |

#### Depth Camera (3 个, 参数相同)

| 参数 | 值 | 说明 |
|------|----|------|
| focal_length | 1.88 mm | |
| horizontal_aperture | 3.44 mm | 对应 HFOV ≈ 84.91° |
| vertical_aperture | 2.06 mm | 对应 VFOV ≈ 57.43° |
| clipping_range | [0.52, 6.0] m | near = D455 最小深度距离 |
| 分辨率 | 1280 × 720 | 匹配 D455 Depth 原生分辨率 |

### 2.3 内参矩阵 (像素坐标)

#### RGB

```
K_rgb = [671.30,    0,   640.0]
        [   0,   671.30, 400.0]
        [   0,      0,     1  ]
```

#### Depth

```
K_depth = [699.53,    0,   640.0]
          [   0,   657.09, 360.0]
          [   0,      0,     1  ]
```

### 2.4 配置一致性验证结果

| 检查项 | 头部 | 左腕 | 右腕 | 状态 |
|--------|------|------|------|------|
| RGB↔Depth 间距 | 59.0 mm | 59.0 mm | 59.0 mm | ✅ |
| RGB↔Depth 姿态一致 | ✅ | ✅ | ✅ | ✅ |
| Depth clip_near = 0.52m | ✅ | ✅ | ✅ | ✅ |
| Prim 路径有效 | ✅ | ✅ | ✅ | ✅ |
| Prim 类型 = Camera | ✅ | ✅ | ✅ | ✅ |

---

## 3. 相机随动行为

### 3.1 层级继承

Camera prim 作为 URDF frame 的子节点，其世界位姿 = 父节点世界位姿 × 自身 local transform。

```
世界坐标 = T_base→trunk × T_trunk→shoulder × ... × T_link7→camera_link × T_camera_link→color_frame × T_local_camera
```

当手臂关节运动时，Camera prim 的世界位姿自动跟随变化。报告中记录的世界坐标是某一时刻的快照值，不是固定值。

### 3.2 各相机运动特性

| 相机 | 运动自由度 | 说明 |
|------|-----------|------|
| Head | 取决于 body_joint + waist_joint | 躯干俯仰/升降时跟随 |
| Left | 随左臂 7 个关节 | 末端位姿完全由手臂姿态决定 |
| Right | 随右臂 7 个关节 | 同上 |

### 3.3 坐标系说明

- **Property 面板中的 Transform**: 相对于父 prim 的 **local transform**（创建时通常为零）
- **检查报告中的世界坐标**: 某一时刻的 **world transform 快照**
- **OCS2/ROS2 中使用**: 通过 `/tf` 话题获取实时世界坐标

---

## 4. Action Graph: Camera_Publish_Graph

### 4.1 节点清单

| # | 节点名 | 类型 | 说明 |
|---|--------|------|------|
| 1 | `on_playback_tick` | `omni.graph.action.OnPlaybackTick` | 触发源 |
| 2 | `ros2_context` | `isaacsim.ros2.bridge.ROS2Context` | ROS2 域 |
| 3 | `cam_head_rgb` | `isaacsim.ros2.bridge.ROS2CameraHelper` | 头部 RGB |
| 4 | `cam_head_depth` | `isaacsim.ros2.bridge.ROS2CameraHelper` | 头部 Depth |
| 5 | `cam_left_rgb` | `isaacsim.ros2.bridge.ROS2CameraHelper` | 左腕 RGB |
| 6 | `cam_left_depth` | `isaacsim.ros2.bridge.ROS2CameraHelper` | 左腕 Depth |
| 7 | `cam_right_rgb` | `isaacsim.ros2.bridge.ROS2CameraHelper` | 右腕 RGB |
| 8 | `cam_right_depth` | `isaacsim.ros2.bridge.ROS2CameraHelper` | 右腕 Depth |

### 4.2 连线

```
on_playback_tick.Tick → cam_head_rgb.ExecIn
                      → cam_head_depth.ExecIn
                      → cam_left_rgb.ExecIn
                      → cam_left_depth.ExecIn
                      → cam_right_rgb.ExecIn
                      → cam_right_depth.ExecIn

ros2_context.Context → (所有 6 个 CameraHelper 的 Context 输入)
```

### 4.3 CameraHelper 参数表

| 节点 | cameraPrim | topicName | type | frameId | width | height |
|------|-----------|-----------|------|---------|-------|--------|
| cam_head_rgb | `.../camera_H_color_frame/RGBCamera` | `/head_cam/color/image_raw` | `rgb` | `camera_H_color_optical_frame` | 1280 | 800 |
| cam_head_depth | `.../camera_H_depth_frame/DepthCamera` | `/head_cam/depth/image_rect_raw` | `depth` | `camera_H_depth_optical_frame` | 1280 | 720 |
| cam_left_rgb | `.../camera_L_color_frame/RGBCamera` | `/left_cam/color/image_raw` | `rgb` | `camera_L_color_optical_frame` | 1280 | 800 |
| cam_left_depth | `.../camera_L_depth_frame/DepthCamera` | `/left_cam/depth/image_rect_raw` | `depth` | `camera_L_depth_optical_frame` | 1280 | 720 |
| cam_right_rgb | `.../camera_R_color_frame/RGBCamera` | `/right_cam/color/image_raw` | `rgb` | `camera_R_color_optical_frame` | 1280 | 800 |
| cam_right_depth | `.../camera_R_depth_frame/DepthCamera` | `/right_cam/depth/image_rect_raw` | `depth` | `camera_R_depth_optical_frame` | 1280 | 720 |

---

## 5. ROS2 话题总览

### 5.1 相机话题

| 话题 | 消息类型 | 频率 | 说明 |
|------|---------|------|------|
| `/head_cam/color/image_raw` | `sensor_msgs/Image` | sim tick | 头部 RGB |
| `/head_cam/depth/image_rect_raw` | `sensor_msgs/Image` | sim tick | 头部 Depth |
| `/left_cam/color/image_raw` | `sensor_msgs/Image` | sim tick | 左腕 RGB |
| `/left_cam/depth/image_rect_raw` | `sensor_msgs/Image` | sim tick | 左腕 Depth |
| `/right_cam/color/image_raw` | `sensor_msgs/Image` | sim tick | 右腕 RGB |
| `/right_cam/depth/image_rect_raw` | `sensor_msgs/Image` | sim tick | 右腕 Depth |

### 5.2 感知输出话题 (ROS2 端感知节点发布)

| 话题 | 消息类型 | 说明 |
|------|---------|------|
| `/tray_pose` | `geometry_msgs/PoseStamped` | 检测到的托盘位姿 → OCS2 |

### 5.3 TF 树 (相机相关)

```
base_link
 └── trunk_link
      ├── head_camera_link
      │    └── camera_H_link
      │         ├── camera_H_color_frame
      │         │    └── camera_H_color_optical_frame
      │         └── camera_H_depth_frame
      │              └── camera_H_depth_optical_frame
      ├── left_Link7
      │    └── ... → camera_L_link
      │         ├── camera_L_color_frame → camera_L_color_optical_frame
      │         └── camera_L_depth_frame → camera_L_depth_optical_frame
      └── right_Link7
           └── ... → camera_R_link → (同上)
```

---

## 6. 相机姿态调整方法

### 6.1 场景：需要调整相机朝向

由于 URDF Importer 展平了 fixed joint 层级，直接在 Stage 中拖动 `camera_H_link` 会受到 articulation 约束限制。

### 6.2 推荐方案

| 调整幅度 | 方案 | 操作 | TF 一致性 |
|---------|------|------|-----------|
| 大角度 (>5°) | 修改 URDF 源文件 | 改 `camera_H_joint` 的 `origin rpy` → 重新导入 | ✅ 完全一致 |
| 小角度微调 | Camera prim 加 local offset | 在 RGBCamera/DepthCamera 上加 RotateXYZ op | ⚠️ TF 与渲染有偏差 |
| 一次性调整 | 打破/重建 FixedJoint | 删除 FixedJoint → 调整 → 重建 | ✅ 需要同步更新 TF |

### 6.3 URDF 修改示例

```xml
<!-- 头部相机多俯仰 10° -->
<joint name="camera_H_joint" type="fixed">
  <origin rpy="0 0.1745 0" xyz="0.0642 0 -0.0145"/>
  <parent link="head_camera_link"/>
  <child link="camera_H_link"/>
</joint>
```

### 6.4 Isaac Sim Script Editor 微调示例

```python
from pxr import UsdGeom, Gf

stage = omni.usd.get_context().get_stage()
cam = stage.GetPrimAtPath(
    "/World/Robot/camera_H_link/camera_H_color_frame/RGBCamera"
)
xform = UsdGeom.Xformable(cam)
# 清除已有 xform ops 后添加
xform.ClearXformOpOrder()
xform.AddRotateXYZOp().Set(Gf.Vec3f(0, 10, 0))  # 额外俯仰 10°
```

---

## 7. 性能考虑

### 7.1 渲染负载

6 个 Camera prim 每帧都需要渲染，对 GPU 压力较大：

| 配置 | 每帧渲染像素 | 建议 |
|------|------------|------|
| 全分辨率 | 6 × 1280 × ~760 ≈ 5.8M 像素 | 训练/验证时用 |
| 半分辨率 | 6 × 640 × ~380 ≈ 1.5M 像素 | 调试时用，改 CameraHelper 的 width/height |
| 仅启用需要的相机 | 按阶段启用 | 阶段 A/C 才需要相机 |

### 7.2 频率控制

相机不需要每个 sim tick 都发布。可以在 Action Graph 中加入 `Gate` 或计数器节点，降低到 10-30 Hz 发布频率。

---

## 8. 后续任务清单

| 优先级 | 任务 | 依赖 | 状态 |
|--------|------|------|------|
| P0 | 跑通 Arm_Control + State_Telemetry 最小闭环 | URDF 已修复导入 | 🔧 进行中 |
| P0 | OCS2 → ROS2 → Isaac Sim 通信链路 | 上一步完成 | ⏳ 待开始 |
| P1 | Gripper_Control_Graph 联调 | 手臂闭环跑通 | ⏳ 待开始 |
| P1 | Camera_Publish_Graph 联调 | 相机已配置 ✅ | ⏳ 待开始 |
| P2 | 感知节点 (托盘位姿检测) | 相机发布正常 | ⏳ 待开始 |
| P2 | 完整闭环: OCS2 + 感知 + 抓放 | 以上全部 | ⏳ 待开始 |
| P3 | Trajectory Logger | 完整闭环 | ⏳ 待开始 |
| P3 | 域随机化 (质量/摩擦/位置) | 完整闭环稳定 | ⏳ 待开始 |
