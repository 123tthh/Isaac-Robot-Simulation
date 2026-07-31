# OCS2 Dynamic Null-space Regularization

## 目的

双臂都是 7 DOF，末端位姿约束不足以唯一决定所有关节角。OCS2 在满足同一个末端 target 时，可能沿 null-space 选择另一组关节解，这就是进入 OCS2 后“末端看起来差不多，但肩/肘/腕漂移”的来源。

之前使用固定 `stateCost.nominalState` 可以压住漂移，但它的问题是：固定 nominal 在整个 OCS2 运动过程中一直生效。权重一高，手臂会被持续拉回 home，表现为 `joint5/6/7` 像被固定、左右臂张不开；权重一低，又压不住进入 OCS2 时的漂移。

当前方案改为动态 null-space 正则：

```text
cost += 0.5 * sum_i w_i * (q_i - q_ref_i)^2
```

其中 `q_ref` 不是固定 home，而是运行时按策略更新：

- 进入 OCS2 / reset MPC 时：`q_ref = 当前观测关节姿态`。
- 当前默认跟随每帧有效 MPC 输出：controller 每次成功写出 MPC 动作后会把 `q_ref` 更新为最新有效 MPC 动作 / 滤波后的 policy state；fallback 积分时也会更新为 fallback 写出的关节动作。
- 若 `dynamicNullspaceCost.updateReferenceFromPolicy false`，`q_ref` 会锁定在进入 OCS2 / reset MPC 时的当前观测关节姿态，可用于诊断或强抑制漂移，但会明显降低示教时的可操作性。

当前默认效果是：动态参考会随成功执行的 MPC 动作移动，抑制无意义的帧间漂移，同时不把人工示教目标持续拉回进入 OCS2 时的姿态。

## 权重含义

`w_i` 是第 `i` 个规划关节偏离 `q_ref_i` 的代价系数。越大，OCS2 越不愿意让该关节相对上一帧动作发生额外变化；越小，该关节越自由。

这里的权重不是硬限制。它只是优化偏好，仍然会让位于：

- 末端位姿 target；
- 关节位置/速度 limit；
- self-collision 或其他约束；
- input cost 和 MPC 动力学。

关节索引顺序：

```text
0..6    left_joint1  .. left_joint7
7..13   right_joint1 .. right_joint7
```

## 当前配置

配置文件：

```text
/home/gtk/ros2_ws/src/robot/config/ocs2/task.info
```

当前固定 nominal 已关闭：

```text
stateCost.activate false
```

动态正则已开启：

```text
dynamicNullspaceCost.activate true
dynamicNullspaceCost.updateReferenceFromPolicy true
```

当前权重选择：

```text
left:  joint1=0.8, joint2=3.0, joint3=3.0, joint4=0.8, joint5=1.2, joint6=0.6, joint7=1.2
right: joint1=0.5, joint2=1.0, joint3=1.0, joint4=0.4, joint5=0.6, joint6=0.3, joint7=0.6
```

选择理由：

- 左臂 `joint2/joint3` 是主要漂移来源，当前提高到 `3.0`，在保留 `updateReferenceFromPolicy true` 手感的同时压住未选中左臂漂移。
- 左臂 `joint5/joint7` 提高到 `1.2`，因为最近 trace 中两个腕部关节也出现约 `0.5 rad` 漂移。
- 左臂 `joint6` 提高到 `0.6`，仍低于肩肘，避免过度限制姿态展开。
- 右臂给低/中等权重，防止右臂再次漂移，但避免影响目前可操作性。

## 修改位置

OCS2 library：

```text
/home/gtk/ros2_ws/src/ocs2_ros2/basic examples/ocs2_mobile_manipulator/src/MobileManipulatorInterface.cpp
/home/gtk/ros2_ws/src/ocs2_ros2/basic examples/ocs2_mobile_manipulator/include/ocs2_mobile_manipulator/MobileManipulatorInterface.h
```

controller 刷新 `q_ref`：

```text
/home/gtk/ros2_ws/src/arms_ros2_control/controller/ocs2_arm_controller/src/control/CtrlComponent.cpp
```

## 生效方式

`task.info` 是 symlink 到 install 目录，单纯改权重通常不需要 build；但这次新增了 C++ controller/library 逻辑，需要重新 build 对应包。build 后必须重启 OCS2 controller / launch，运行中的 controller 不会热加载新 cost。

重启后按顺序测试：

1. 左臂进入 OCS2 是否还漂移。
2. 左右臂是否还能正常张开、平移、旋转。
3. 如果左臂仍漂，优先只提高发生漂移的左臂关节，不要关闭 `updateReferenceFromPolicy`。
4. 如果动作又开始变小或发紧，优先保持 `updateReferenceFromPolicy true`，再把左 `joint2/joint3` 从 `3.0` 降到 `2.0`。

调参只改 `dynamicNullspaceCost.Q.arm`。不要重新启用固定 `stateCost`，除非只是临时诊断。
