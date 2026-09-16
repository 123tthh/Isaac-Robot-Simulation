# R1 机器人资源唯一来源

核心 OCS2/SIM1 的唯一机器人源码为：

```text
projects/ros2_ws/src/robot
```

该目录是 ROS 包 `r1_description`，运行时 URDF 和 mesh 分别来自：

```text
projects/ros2_ws/src/robot/urdf/r1_fixed.urdf
projects/ros2_ws/src/robot/meshes/
```

`r1_lerobot_sim` 的默认参数已经改为使用该目录。

以下目录不再作为核心资源来源：

- `projects/teleoperation/robot`：旧 RViz/LeRobot 独立副本；保留在当前工作树
  供历史对照，但被主 Git 忽略。
- `projects/IsaacLab-Arena/isaaclab_arena/r1_description`：Arena 可选实验副本。
- `projects/IsaacLab-Arena/isaaclab_arena/r1_description_full`：Arena 可选实验
  完整副本。

当前唯一的 Isaac 场景入口是：

```text
assets/scenes/scene.usd
```

它引用的项目内资源层位于：

```text
assets/scenes/r1_workcell/
```

`workcell.usd` 不是启动入口。场景资产不能用 ROS robot 目录替代。尤其在
`/World/Robot/base_link/visuals` 的历史引用修复完成前，必须完整保留
`assets/scenes/r1_workcell`。
