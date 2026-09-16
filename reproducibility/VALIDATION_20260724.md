> **Historical record / 历史记录**：本页描述其记录日期的状态；当前运行以[中文指南](../docs/getting-started.zh-CN.md) / [English guide](../docs/getting-started.md)为准。Historical failures and paths are not current release claims.

# 冷启动与迁移验证记录（2026-07-24）

## 范围

- 测试副本：`/tmp/isaac_ocs_reproduce_test`
- 唯一场景入口：`assets/scenes/scene.usd`
- 核心机器人资源：`projects/ros2_ws/src/robot`
- 核心运行不包含 `projects/IsaacLab-Arena`

## 已通过

1. 从不含构建产物、Arena、旧机器人副本和大数据的工程副本恢复。
2. 从锁定提交恢复 `robot_descriptions` submodule：
   `fff4d08cf07cff966747986bbc01af75ff23e2ad`。
3. 场景和保留的 SIM1 success 数据从独立制品恢复后逐文件
   `sha256sum -c` 通过。
4. `verify_project_layout.sh`：0 个失败。
5. 宿主核心 ROS 构建：25 个包完成，退出码 0。
6. `r1_description`、`ocs2_arm_controller`、`r1_lerobot_sim` 均从
   `/tmp/isaac_ocs_reproduce_test/projects/ros2_ws/install` 解析。
7. 宿主 preflight：0 个失败；GPU 为 RTX 5090 D v2。
8. Isaac Sim 5.1 从冷路径打开并遍历 `assets/scenes/scene.usd`，退出码 0。
9. 历史 Docker 镜像导出后重新 `docker load` 成功。
10. Docker 冷构建：25 个核心包完成，三个关键 ROS 包均从
    `/workspace/projects/ros2_ws/install_docker` 解析；未挂载 Arena；
    容器 Isaac Python 为 3.11.13。

## 已知边界

- `build --all` 会构建 `robot_descriptions` 中约百个可选包；当前空宿主缺少
  Grid Map、MoveIt Servo、Ignition Gazebo 依赖，且部分旧包与用户级
  CMake 4.3 策略不兼容。核心复现使用无 `--all` 的构建命令。
- 场景可加载，但 USD 报告既有警告：
  `base.usd</r1/base_link/visuals>` 指向
  `physics.usd</visuals/base_link>`，目标 prim 不存在。
- 历史镜像 `issac_ocs_docker:20260724-full` 的实际 Isaac Sim 是
  `5.0.0-rc.45`，并继承 Arena 环境变量；它只用于兼容和归档验证。
  严格复现镜像必须从 `nvcr.io/nvidia/isaac-sim:5.1.0` 重新构建。
