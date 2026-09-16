# Isaac Sim Stage 结构导出 (含物理诊断)
> SIM1 工程总纲: [`../PROJECT_OVERVIEW.md`](../PROJECT_OVERVIEW.md)
> 相关操作指导: [`../GAME_GUIDE.md`](../GAME_GUIDE.md)
> 生成时间: 2026-05-28 11:30:18
> Stage: /home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/522.usd

## 1. Stage 基本信息

| 项目 | 值 |
|------|----|
| 顶层 Prim 数 | 10 |
| 顶层 Prim 列表 | World, Environment, Render, physicsScene, Viewport_Measure, Camera, OmniverseKit_Persp, OmniverseKit_Front, OmniverseKit_Top, OmniverseKit_Right |
| Meters Per Unit | 1.0 |
| Up Axis | Z |

## 2. 核心环境资产物理状态 (Pallet & Conveyor)
> 警告: 如果以下组件缺少 Collision 或 RigidBody，将直接导致物理穿模。

### 组件组: `Pallet` (路径: /World/Mat/Pallet)
| 子组件名称 | 类型 | 碰撞 (Collision) | 刚体 (RigidBody) | 驱动 (Drive/Velocity) |
|------------|------|------------------|----------------|-----------------------|

### 组件组: `ConveyorGroup` (路径: /World/ConveyorGroup)
| 子组件名称 | 类型 | 碰撞 (Collision) | 刚体 (RigidBody) | 驱动 (Drive/Velocity) |
|------------|------|------------------|----------------|-----------------------|
| `Roller_00_L` | Cylinder | ✅ | ❌ | ❌ |
| `Roller_00_R` | Cylinder | ✅ | ❌ | ❌ |
| `Roller_01_L` | Cylinder | ✅ | ❌ | ❌ |
| `Roller_01_R` | Cylinder | ✅ | ❌ | ❌ |
| `Roller_02_L` | Cylinder | ✅ | ❌ | ❌ |
| `Roller_02_R` | Cylinder | ✅ | ❌ | ❌ |
| `Roller_03_L` | Cylinder | ✅ | ❌ | ❌ |
| `Roller_03_R` | Cylinder | ✅ | ❌ | ❌ |
| `Roller_04_L` | Cylinder | ✅ | ❌ | ❌ |
| `Roller_04_R` | Cylinder | ✅ | ❌ | ❌ |
| `Roller_05_L` | Cylinder | ✅ | ❌ | ❌ |
| `Roller_05_R` | Cylinder | ✅ | ❌ | ❌ |
| `Roller_06_L` | Cylinder | ✅ | ❌ | ❌ |
| `Roller_06_R` | Cylinder | ✅ | ❌ | ❌ |
| `Roller_07_L` | Cylinder | ✅ | ❌ | ❌ |
| `Roller_07_R` | Cylinder | ✅ | ❌ | ❌ |
| `Roller_08_L` | Cylinder | ✅ | ❌ | ❌ |
| `Roller_08_R` | Cylinder | ✅ | ❌ | ❌ |
| `Rail_Left_forward` | Cube | ✅ | ❌ | ❌ |
| `Rail_Right_forward` | Cube | ✅ | ❌ | ❌ |
| `Rail_Left_backward` | Cube | ✅ | ❌ | ❌ |
| `Rail_Right_backward` | Cube | ✅ | ❌ | ❌ |
| `Backward_Surface` | Cube | ✅ | ❌ | ❌ |
| `Stopper` | Cube | ✅ | ❌ | ❌ |
| `Forward_Surface` | Cube | ✅ | ❌ | ❌ |

### 组件组: `Pallet` (路径: /World/Pallet)
| 子组件名称 | 类型 | 碰撞 (Collision) | 刚体 (RigidBody) | 驱动 (Drive/Velocity) |
|------------|------|------------------|----------------|-----------------------|
| `CollisionProxy` | Cube | ✅ | ✅ | ❌ |
| `Back_CollisionProxy` | Cube | ✅ | ✅ | ❌ |
| `Pallet_Righthand` | Cube | ✅ | ✅ | ❌ |
| `Pallet_Lefthand` | Cube | ✅ | ✅ | ❌ |

### 组件组: `Back_ConveyorGroup` (路径: /World/Back_ConveyorGroup)
| 子组件名称 | 类型 | 碰撞 (Collision) | 刚体 (RigidBody) | 驱动 (Drive/Velocity) |
|------------|------|------------------|----------------|-----------------------|
| `Back_Rail_Left` | Cube | ✅ | ❌ | ❌ |
| `Back_Rail_Right` | Cube | ✅ | ❌ | ❌ |
| `Back_Surface` | Cube | ✅ | ❌ | ❌ |
| `Back_Stopper` | Cube | ✅ | ❌ | ❌ |

## 3. Robot 层级结构

- Robot `[Xform]`
  - base_link `[Xform, ArticulationRoot, RigidBody]` 🤖
    - camera_B_bottom_screw_frame `[Xform]`
    - camera_F_bottom_screw_frame `[Xform]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - camera_B_link `[Xform, RigidBody]`
    - camera_B_accel_frame `[Xform]`
      - camera_B_accel_optical_frame `[Xform]`
    - camera_B_color_frame `[Xform]`
      - camera_B_color_optical_frame `[Xform]`
    - camera_B_depth_frame `[Xform]`
      - camera_B_depth_optical_frame `[Xform]`
    - camera_B_gyro_frame `[Xform]`
      - camera_B_gyro_optical_frame `[Xform]`
    - camera_B_infra1_frame `[Xform]`
      - camera_B_infra1_optical_frame `[Xform]`
    - camera_B_infra2_frame `[Xform]`
      - camera_B_infra2_optical_frame `[Xform]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - joints `[Scope]`
    - camera_B_link_joint `[PhysicsFixedJoint]` 🔗
    - camera_F_link_joint `[PhysicsFixedJoint]` 🔗
    - caster_LB_joint `[PhysicsRevoluteJoint, Drive]` 🔗
    - caster_LF_joint `[PhysicsRevoluteJoint, Drive]` 🔗
    - caster_RB_joint `[PhysicsRevoluteJoint, Drive]` 🔗
    - caster_RF_joint `[PhysicsRevoluteJoint, Drive]` 🔗
    - laser_joint `[PhysicsFixedJoint]` 🔗
    - lidar_joint `[PhysicsFixedJoint]` 🔗
    - trunk_joint `[PhysicsFixedJoint]` 🔗
    - waist_joint `[PhysicsPrismaticJoint, Drive]` 🔗
    - body_joint `[PhysicsRevoluteJoint, Drive]` 🔗
    - head_camera_joint `[PhysicsFixedJoint]` 🔗
    - camera_H_link_joint `[PhysicsFixedJoint]` 🔗
    - left_attach_joint `[PhysicsFixedJoint]` 🔗
    - left_joint1 `[PhysicsRevoluteJoint, Drive]` 🔗
    - left_joint2 `[PhysicsRevoluteJoint, Drive]` 🔗
    - left_joint3 `[PhysicsRevoluteJoint, Drive]` 🔗
    - left_joint4 `[PhysicsRevoluteJoint, Drive]` 🔗
    - left_joint5 `[PhysicsRevoluteJoint, Drive]` 🔗
    - left_joint6 `[PhysicsRevoluteJoint, Drive]` 🔗
    - left_joint7 `[PhysicsRevoluteJoint, Drive]` 🔗
    - left_flange_joint `[PhysicsFixedJoint]` 🔗
    - camera_L_link_joint `[PhysicsFixedJoint]` 🔗
    - ee_left_joint `[PhysicsFixedJoint]` 🔗
    - left_PGIA_joint1 `[PhysicsPrismaticJoint, Drive]` 🔗
    - left_PGIA_joint2 `[PhysicsPrismaticJoint, Drive]` 🔗
    - right_attach_joint `[PhysicsFixedJoint]` 🔗
    - right_joint1 `[PhysicsRevoluteJoint, Drive]` 🔗
    - right_joint2 `[PhysicsRevoluteJoint, Drive]` 🔗
    - right_joint3 `[PhysicsRevoluteJoint, Drive]` 🔗
    - right_joint4 `[PhysicsRevoluteJoint, Drive]` 🔗
    - right_joint5 `[PhysicsRevoluteJoint, Drive]` 🔗
    - right_joint6 `[PhysicsRevoluteJoint, Drive]` 🔗
    - right_joint7 `[PhysicsRevoluteJoint, Drive]` 🔗
    - right_flange_joint `[PhysicsFixedJoint]` 🔗
    - camera_R_link_joint `[PhysicsFixedJoint]` 🔗
    - ee_right_joint `[PhysicsFixedJoint]` 🔗
    - right_PGIA_joint1 `[PhysicsPrismaticJoint, Drive]` 🔗
    - right_PGIA_joint2 `[PhysicsPrismaticJoint, Drive]` 🔗
    - wheel_L_joint `[PhysicsRevoluteJoint, Drive]` 🔗
    - wheel_R_joint `[PhysicsRevoluteJoint, Drive]` 🔗
  - camera_F_link `[Xform, RigidBody]`
    - camera_F_accel_frame `[Xform]`
      - camera_F_accel_optical_frame `[Xform]`
    - camera_F_color_frame `[Xform]`
      - camera_F_color_optical_frame `[Xform]`
    - camera_F_depth_frame `[Xform]`
      - camera_F_depth_optical_frame `[Xform]`
    - camera_F_gyro_frame `[Xform]`
      - camera_F_gyro_optical_frame `[Xform]`
    - camera_F_infra1_frame `[Xform]`
      - camera_F_infra1_optical_frame `[Xform]`
    - camera_F_infra2_frame `[Xform]`
      - camera_F_infra2_optical_frame `[Xform]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - caster_LB_link `[Xform, RigidBody]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - caster_LF_link `[Xform, RigidBody]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - caster_RB_link `[Xform, RigidBody]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - caster_RF_link `[Xform, RigidBody]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - laser_link `[Xform, RigidBody]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - lidar_link `[Xform, RigidBody]`
    - lidar_imu_link `[Xform]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - trunk_link `[Xform, RigidBody]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - waist_link `[Xform, RigidBody]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - body_link `[Xform, RigidBody]`
    - arm_attach_L_link `[Xform]`
    - arm_attach_R_link `[Xform]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - head_camera_link `[Xform, RigidBody]`
    - camera_H_bottom_screw_frame `[Xform]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - camera_H_link `[Xform, RigidBody]`
    - camera_H_accel_frame `[Xform]`
      - camera_H_accel_optical_frame `[Xform]`
    - camera_H_color_frame `[Xform]`
      - camera_H_color_optical_frame `[Xform]`
    - camera_H_depth_frame `[Xform]`
      - camera_H_depth_optical_frame `[Xform]`
    - camera_H_gyro_frame `[Xform]`
      - camera_H_gyro_optical_frame `[Xform]`
        - camera_H_imu_optical_frame `[Xform]`
    - camera_H_infra1_frame `[Xform]`
      - camera_H_infra1_optical_frame `[Xform]`
    - camera_H_infra2_frame `[Xform]`
      - camera_H_infra2_optical_frame `[Xform]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - left_base_link `[Xform, RigidBody]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - left_Link1 `[Xform, RigidBody]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - left_Link2 `[Xform, RigidBody]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - left_Link3 `[Xform, RigidBody]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - left_Link4 `[Xform, RigidBody]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - left_Link5 `[Xform, RigidBody]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - left_Link6 `[Xform, RigidBody]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - left_Link7 `[Xform, RigidBody]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - left_flange_link `[Xform, RigidBody]`
    - camera_L_bottom_screw_frame `[Xform]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - camera_L_link `[Xform, RigidBody]`
    - camera_L_accel_frame `[Xform]`
      - camera_L_accel_optical_frame `[Xform]`
    - camera_L_color_frame `[Xform]`
      - camera_L_color_optical_frame `[Xform]`
    - camera_L_depth_frame `[Xform]`
      - camera_L_depth_optical_frame `[Xform]`
    - camera_L_gyro_frame `[Xform]`
      - camera_L_gyro_optical_frame `[Xform]`
        - camera_L_imu_optical_frame `[Xform]`
    - camera_L_infra1_frame `[Xform]`
      - camera_L_infra1_optical_frame `[Xform]`
    - camera_L_infra2_frame `[Xform]`
      - camera_L_infra2_optical_frame `[Xform]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - left_PGIA_base_link `[Xform, RigidBody]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - left_PGIA_link1 `[Xform, RigidBody, Collision]` 💥
    - visuals `[Xform]`
    - collisions `[Xform]`
  - left_PGIA_link2 `[Xform, RigidBody, Collision]` 💥
    - visuals `[Xform]`
    - collisions `[Xform]`
  - right_base_link `[Xform, RigidBody]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - right_Link1 `[Xform, RigidBody]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - right_Link2 `[Xform, RigidBody]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - right_Link3 `[Xform, RigidBody]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - right_Link4 `[Xform, RigidBody]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - right_Link5 `[Xform, RigidBody]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - right_Link6 `[Xform, RigidBody]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - right_Link7 `[Xform, RigidBody]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - right_flange_link `[Xform, RigidBody]`
    - camera_R_bottom_screw_frame `[Xform]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - camera_R_link `[Xform, RigidBody]`
    - camera_R_accel_frame `[Xform]`
      - camera_R_accel_optical_frame `[Xform]`
    - camera_R_color_frame `[Xform]`
      - camera_R_color_optical_frame `[Xform]`
    - camera_R_depth_frame `[Xform]`
      - camera_R_depth_optical_frame `[Xform]`
    - camera_R_gyro_frame `[Xform]`
      - camera_R_gyro_optical_frame `[Xform]`
        - camera_R_imu_optical_frame `[Xform]`
    - camera_R_infra1_frame `[Xform]`
      - camera_R_infra1_optical_frame `[Xform]`
    - camera_R_infra2_frame `[Xform]`
      - camera_R_infra2_optical_frame `[Xform]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - right_PGIA_base_link `[Xform, RigidBody]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - right_PGIA_link1 `[Xform, RigidBody, Collision]` 💥
    - visuals `[Xform]`
    - collisions `[Xform]`
  - right_PGIA_link2 `[Xform, RigidBody, Collision]` 💥
    - visuals `[Xform]`
    - collisions `[Xform]`
  - wheel_L_link `[Xform, RigidBody]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - wheel_R_link `[Xform, RigidBody]`
    - visuals `[Xform]`
    - collisions `[Xform]`
  - Looks `[Scope]`
    - material_aluminum `[Material]`
      - Shader `[Shader]`
    - material_plastic `[Material]`
      - Shader `[Shader]`
    - material_C0C0C0 `[Material]`
      - Shader `[Shader]`
    - material_CAD1EE `[Material]`
      - Shader `[Shader]`
    - material_100100100 `[Material]`
      - Shader `[Shader]`
    - material_F0AC1E `[Material]`
      - Shader `[Shader]`

**Robot 子 Prim 总数: 253**

## 4. 关节 (Joints) 清单

| 关节路径 | 类型 | Body0 | Body1 |
|---------|------|-------|-------|
| `Robot/joints/camera_B_link_joint` | PhysicsFixedJoint | N/A | N/A |
| `Robot/joints/camera_F_link_joint` | PhysicsFixedJoint | N/A | N/A |
| `Robot/joints/caster_LB_joint` | PhysicsRevoluteJoint | N/A | N/A |
| `Robot/joints/caster_LF_joint` | PhysicsRevoluteJoint | N/A | N/A |
| `Robot/joints/caster_RB_joint` | PhysicsRevoluteJoint | N/A | N/A |
| `Robot/joints/caster_RF_joint` | PhysicsRevoluteJoint | N/A | N/A |
| `Robot/joints/laser_joint` | PhysicsFixedJoint | N/A | N/A |
| `Robot/joints/lidar_joint` | PhysicsFixedJoint | N/A | N/A |
| `Robot/joints/trunk_joint` | PhysicsFixedJoint | N/A | N/A |
| `Robot/joints/waist_joint` | PhysicsPrismaticJoint | N/A | N/A |
| `Robot/joints/body_joint` | PhysicsRevoluteJoint | N/A | N/A |
| `Robot/joints/head_camera_joint` | PhysicsFixedJoint | N/A | N/A |
| `Robot/joints/camera_H_link_joint` | PhysicsFixedJoint | N/A | N/A |
| `Robot/joints/left_attach_joint` | PhysicsFixedJoint | N/A | N/A |
| `Robot/joints/left_joint1` | PhysicsRevoluteJoint | N/A | N/A |
| `Robot/joints/left_joint2` | PhysicsRevoluteJoint | N/A | N/A |
| `Robot/joints/left_joint3` | PhysicsRevoluteJoint | N/A | N/A |
| `Robot/joints/left_joint4` | PhysicsRevoluteJoint | N/A | N/A |
| `Robot/joints/left_joint5` | PhysicsRevoluteJoint | N/A | N/A |
| `Robot/joints/left_joint6` | PhysicsRevoluteJoint | N/A | N/A |
| `Robot/joints/left_joint7` | PhysicsRevoluteJoint | N/A | N/A |
| `Robot/joints/left_flange_joint` | PhysicsFixedJoint | N/A | N/A |
| `Robot/joints/camera_L_link_joint` | PhysicsFixedJoint | N/A | N/A |
| `Robot/joints/ee_left_joint` | PhysicsFixedJoint | N/A | N/A |
| `Robot/joints/left_PGIA_joint1` | PhysicsPrismaticJoint | N/A | N/A |
| `Robot/joints/left_PGIA_joint2` | PhysicsPrismaticJoint | N/A | N/A |
| `Robot/joints/right_attach_joint` | PhysicsFixedJoint | N/A | N/A |
| `Robot/joints/right_joint1` | PhysicsRevoluteJoint | N/A | N/A |
| `Robot/joints/right_joint2` | PhysicsRevoluteJoint | N/A | N/A |
| `Robot/joints/right_joint3` | PhysicsRevoluteJoint | N/A | N/A |
| `Robot/joints/right_joint4` | PhysicsRevoluteJoint | N/A | N/A |
| `Robot/joints/right_joint5` | PhysicsRevoluteJoint | N/A | N/A |
| `Robot/joints/right_joint6` | PhysicsRevoluteJoint | N/A | N/A |
| `Robot/joints/right_joint7` | PhysicsRevoluteJoint | N/A | N/A |
| `Robot/joints/right_flange_joint` | PhysicsFixedJoint | N/A | N/A |
| `Robot/joints/camera_R_link_joint` | PhysicsFixedJoint | N/A | N/A |
| `Robot/joints/ee_right_joint` | PhysicsFixedJoint | N/A | N/A |
| `Robot/joints/right_PGIA_joint1` | PhysicsPrismaticJoint | N/A | N/A |
| `Robot/joints/right_PGIA_joint2` | PhysicsPrismaticJoint | N/A | N/A |
| `Robot/joints/wheel_L_joint` | PhysicsRevoluteJoint | N/A | N/A |
| `Robot/joints/wheel_R_joint` | PhysicsRevoluteJoint | N/A | N/A |

**关节总数: 41**

## 5. 可驱动关节 (Drive)

| 关节名 | 方向 | Stiffness | Damping | Max Force |
|--------|------|-----------|---------|-----------|
| `caster_LB_joint` | angular | 0.0 | 0.0 | 3.4028234663852886e+38 |
| `caster_LF_joint` | angular | 0.0 | 0.0 | 3.4028234663852886e+38 |
| `caster_RB_joint` | angular | 0.0 | 0.0 | 3.4028234663852886e+38 |
| `caster_RF_joint` | angular | 0.0 | 0.0 | 3.4028234663852886e+38 |
| `waist_joint` | linear | 12264.9951171875 | 4.905998229980469 | 500.0 |
| `body_joint` | angular | 5293.81005859375 | 2.1175241470336914 | 200.0 |
| `left_joint1` | angular | 2520.6201171875 | 1.0082480907440186 | 100.0 |
| `left_joint2` | angular | 1223.3082275390625 | 0.48932331800460815 | 100.0 |
| `left_joint3` | angular | 1209.7322998046875 | 0.48389288783073425 | 100.0 |
| `left_joint4` | angular | 425.0352478027344 | 0.17001409828662872 | 100.0 |
| `left_joint5` | angular | 420.11761474609375 | 0.16804705560207367 | 100.0 |
| `left_joint6` | angular | 118.37910461425781 | 0.047351643443107605 | 100.0 |
| `left_joint7` | angular | 114.77234649658203 | 0.045908939093351364 | 100.0 |
| `left_PGIA_joint1` | linear | 2000.0 | 200.0 | 500.0 |
| `left_PGIA_joint2` | linear | 2000.0 | 200.0 | 500.0 |
| `right_joint1` | angular | 2531.31689453125 | 1.0125267505645752 | 100.0 |
| `right_joint2` | angular | 1220.7757568359375 | 0.4883103370666504 | 100.0 |
| `right_joint3` | angular | 1207.7608642578125 | 0.4831043481826782 | 100.0 |
| `right_joint4` | angular | 424.5191650390625 | 0.16980765759944916 | 100.0 |
| `right_joint5` | angular | 419.5163269042969 | 0.16780653595924377 | 100.0 |
| `right_joint6` | angular | 118.33924865722656 | 0.04733569920063019 | 100.0 |
| `right_joint7` | angular | 114.73411560058594 | 0.04589364305138588 | 100.0 |
| `right_PGIA_joint1` | linear | 2000.0 | 200.0 | 500.0 |
| `right_PGIA_joint2` | linear | 2000.0 | 200.0 | 500.0 |
| `wheel_L_joint` | angular | 0.0 | 0.0 | 3.4028234663852886e+38 |
| `wheel_R_joint` | angular | 0.0 | 0.0 | 3.4028234663852886e+38 |

**可驱动关节数: 26**

## 6. 相机 (Camera) 清单

| 路径 | 焦距 (mm) | H_Aperture | V_Aperture | Clip Near | Clip Far |
|------|----------|------------|------------|-----------|----------|
| `/Camera` | 50.00 | 20.95 | 15.29 | 1.00 | 1000000.0 |
| `/OmniverseKit_Persp` | 18.15 | 20.95 | 15.29 | 0.01 | 10000000.0 |
| `/OmniverseKit_Front` | 50.00 | 50.00 | 50.00 | 1.00 | 10000000.0 |
| `/OmniverseKit_Top` | 50.00 | 50.00 | 50.00 | 1.00 | 10000000.0 |
| `/OmniverseKit_Right` | 50.00 | 50.00 | 50.00 | 1.00 | 10000000.0 |

## 7. 碰撞体 (Collision) 统计

| 碰撞体类型 | 数量 |
|-----------|------|
| Cube | 16 |
| Cylinder | 18 |
| Mesh | 2 |
| Plane | 1 |
| Xform | 4 |
| **总计** | **41** |

## 8. Action Graphs

### ActionGraphs

路径: `/World/ActionGraphs`

| 节点名 | 类型 |
|--------|------|
| `Arm_Control_Graph` | OmniGraph |
| `State_Telemetry_Graph` | OmniGraph |
| `Camera_Publish_Graph` | OmniGraph |
| `Gripper_Control_Graph` | OmniGraph |

### Arm_Control_Graph

路径: `/World/ActionGraphs/Arm_Control_Graph`

| 节点名 | 类型 |
|--------|------|
| `on_playback_tick` | omni.graph.action.OnPlaybackTick |
| `ros2_context` | isaacsim.ros2.bridge.ROS2Context |
| `ros2_subscribe_joint_state` | isaacsim.ros2.bridge.ROS2SubscribeJointState |
| `articulation_controller` | isaacsim.core.nodes.IsaacArticulationController |

### State_Telemetry_Graph

路径: `/World/ActionGraphs/State_Telemetry_Graph`

| 节点名 | 类型 |
|--------|------|
| `on_playback_tick` | omni.graph.action.OnPlaybackTick |
| `ros2_context` | isaacsim.ros2.bridge.ROS2Context |
| `read_sim_time` | isaacsim.core.nodes.IsaacReadSimulationTime |
| `ros2_publish_clock` | isaacsim.ros2.bridge.ROS2PublishClock |
| `ros2_publish_joint_state` | isaacsim.ros2.bridge.ROS2PublishJointState |

### Camera_Publish_Graph

路径: `/World/ActionGraphs/Camera_Publish_Graph`

| 节点名 | 类型 |
|--------|------|
| `on_playback_tick` | omni.graph.action.OnPlaybackTick |
| `ros2_context` | isaacsim.ros2.bridge.ROS2Context |
| `cam_head_rgb` | isaacsim.ros2.bridge.ROS2CameraHelper |
| `cam_head_depth` | isaacsim.ros2.bridge.ROS2CameraHelper |
| `cam_left_rgb` | isaacsim.ros2.bridge.ROS2CameraHelper |
| `cam_left_depth` | isaacsim.ros2.bridge.ROS2CameraHelper |
| `cam_right_rgb` | isaacsim.ros2.bridge.ROS2CameraHelper |
| `cam_right_depth` | isaacsim.ros2.bridge.ROS2CameraHelper |

### Gripper_Control_Graph

路径: `/World/ActionGraphs/Gripper_Control_Graph`

| 节点名 | 类型 |
|--------|------|
| `on_playback_tick` | omni.graph.action.OnPlaybackTick |
| `ros2_context` | isaacsim.ros2.bridge.ROS2Context |
| `sub_left_gripper` | isaacsim.ros2.bridge.ROS2Subscriber |
| `sub_right_gripper` | isaacsim.ros2.bridge.ROS2Subscriber |
| `script_right_gripper` | omni.graph.scriptnode.ScriptNode |
| `script_left_gripper` | omni.graph.scriptnode.ScriptNode |
| `artic_left_gripper` | isaacsim.core.nodes.IsaacArticulationController |
| `artic_right_gripper` | isaacsim.core.nodes.IsaacArticulationController |

## 9. Physics Scene 配置

路径: `/World/PhysicsScene`

| 属性 | 值 |
|------|----|
| physics:gravityDirection | (0, 0, -1) |
| physics:gravityMagnitude | 9.8100004196167 |
| physxScene:bounceThreshold | 0.0 |
| physxScene:broadphaseType | MBP |
| physxScene:collisionSystem | PCM |
| physxScene:enableCCD | True |
| physxScene:enableEnhancedDeterminism | False |
| physxScene:enableExternalForcesEveryIteration | False |
| physxScene:enableGPUDynamics | False |
| physxScene:enableResidualReporting | True |
| physxScene:enableSceneQuerySupport | True |
| physxScene:enableStabilization | False |
| physxScene:frictionCorrelationDistance | 0.02500000037252903 |
| physxScene:frictionOffsetThreshold | 0.03999999910593033 |
| physxScene:frictionType | patch |
| physxScene:gpuCollisionStackSize | 67108864 |
| physxScene:gpuFoundLostAggregatePairsCapacity | 1024 |
| physxScene:gpuFoundLostPairsCapacity | 262144 |
| physxScene:gpuHeapCapacity | 67108864 |
| physxScene:gpuMaxDeformableSurfaceContacts | 1048576 |
| physxScene:gpuMaxNumPartitions | 8 |
| physxScene:gpuMaxParticleContacts | 1048576 |
| physxScene:gpuMaxRigidContactCount | 524288 |
| physxScene:gpuMaxRigidPatchCount | 81920 |
| physxScene:gpuMaxSoftBodyContacts | 1048576 |
| physxScene:gpuTempBufferCapacity | 16777216 |
| physxScene:gpuTotalAggregatePairsCapacity | 1024 |
| physxScene:invertCollisionGroupFilter | False |
| physxScene:maxBiasCoefficient | inf |
| physxScene:maxPositionIterationCount | 255 |
| physxScene:maxVelocityIterationCount | 255 |
| physxScene:minPositionIterationCount | 1 |
| physxScene:minVelocityIterationCount | 0 |
| physxScene:reportKinematicKinematicPairs | False |
| physxScene:reportKinematicStaticPairs | False |
| physxScene:solverType | TGS |
| physxScene:timeStepsPerSecond | 60 |
| physxScene:updateType | Synchronous |

路径: `/physicsScene`

| 属性 | 值 |
|------|----|
| physics:gravityDirection | (0, 0, 0) |
| physics:gravityMagnitude | -inf |
| physxScene:bounceThreshold | 0.0 |
| physxScene:broadphaseType | MBP |
| physxScene:collisionSystem | PCM |
| physxScene:enableCCD | True |
| physxScene:enableEnhancedDeterminism | False |
| physxScene:enableExternalForcesEveryIteration | False |
| physxScene:enableGPUDynamics | False |
| physxScene:enableResidualReporting | False |
| physxScene:enableSceneQuerySupport | True |
| physxScene:enableStabilization | False |
| physxScene:frictionCorrelationDistance | 0.02500000037252903 |
| physxScene:frictionOffsetThreshold | 0.03999999910593033 |
| physxScene:frictionType | patch |
| physxScene:gpuCollisionStackSize | 67108864 |
| physxScene:gpuFoundLostAggregatePairsCapacity | 1024 |
| physxScene:gpuFoundLostPairsCapacity | 262144 |
| physxScene:gpuHeapCapacity | 67108864 |
| physxScene:gpuMaxDeformableSurfaceContacts | 1048576 |
| physxScene:gpuMaxNumPartitions | 8 |
| physxScene:gpuMaxParticleContacts | 1048576 |
| physxScene:gpuMaxRigidContactCount | 524288 |
| physxScene:gpuMaxRigidPatchCount | 81920 |
| physxScene:gpuMaxSoftBodyContacts | 1048576 |
| physxScene:gpuTempBufferCapacity | 16777216 |
| physxScene:gpuTotalAggregatePairsCapacity | 1024 |
| physxScene:invertCollisionGroupFilter | False |
| physxScene:maxBiasCoefficient | inf |
| physxScene:maxPositionIterationCount | 255 |
| physxScene:maxVelocityIterationCount | 255 |
| physxScene:minPositionIterationCount | 1 |
| physxScene:minVelocityIterationCount | 0 |
| physxScene:reportKinematicKinematicPairs | False |
| physxScene:reportKinematicStaticPairs | False |
| physxScene:solverType | TGS |
| physxScene:timeStepsPerSecond | 60 |
| physxScene:updateType | Synchronous |

## 10. 全局 Prim 类型统计

| 类型 | 数量 |
|------|------|
| Xform | 208 |
| OmniGraphNode | 25 |
| Material | 22 |
| PhysicsRevoluteJoint | 21 |
| Cube | 18 |
| Cylinder | 18 |
| PhysicsFixedJoint | 15 |
| Shader | 13 |
| Xform (untyped) | 6 |
| Scope | 5 |
| PhysicsPrismaticJoint | 5 |
| Camera | 5 |
| OmniGraph | 4 |
| Mesh | 3 |
| RenderProduct | 3 |
| PhysicsScene | 2 |
| Plane | 1 |
| DomeLight | 1 |
| DistantLight | 1 |
| RenderSettings | 1 |
| RenderVar | 1 |
| **总计** | **378** |
