> **Historical record / 历史记录**：本页描述其记录日期的状态；当前运行以[中文指南](../docs/getting-started.zh-CN.md) / [English guide](../docs/getting-started.md)为准。Historical failures and paths are not current release claims.

# Isaac OCS 工程可用性验证报告（2026-07-23）

## 结论

工程的构建、Isaac Sim 5.1.0 启动、USD 结构和 ROS 2/OCS2 核心启动均已通过验证，
当前状态为“可启动、可继续联调”。正式控制前仍需在 Isaac Sim 中打开
`assets/scenes/r1_workcell/workcell.usd` 并按 Play，让 `/isaac_joint_states` 实际发布后，
再确认两个控制器均为 `active`。

## 已通过

- 统一构建完成：27 个当前运行链所需 ROS 2/OCS2 包构建成功。
- `project_control.sh preflight`：0 个失败。
- GPU：NVIDIA GeForce RTX 5090 D v2，驱动 580.126.09，显存 24455 MiB。
- Isaac Sim 无窗口启动：
  - 版本为 `5.1.0-rc.19`。
  - 输出 `app ready`。
  - 退出码为 0。
  - 补充依赖修复后没有 `[Error]` 或缺失模块。
- USD 只读结构检查：
  - `assets/scenes/r1_workcell/workcell.usd` 可打开。
  - 共 439 个 Prim。
  - 包含 `Arm_Control_Graph`。
  - 包含 `State_Telemetry_Graph`。
  - 包含 `Camera_Publish_Graph`，节点覆盖 head/left/right RGB+D。
  - 包含 `Gripper_Control_Graph`。
  - 包含双臂 7 轴、左右 PGIA 夹爪、轮子和多组相机 Prim。
- ROS 2/OCS2 短时烟雾测试：
  - URDF/TF 发布节点正常启动。
  - `joint_state_fill` 正常启动。
  - `odom_to_tf` 正常启动。
  - `topic_based_ros2_control` 硬件完成 initialize/configure/activate。
  - controller manager 以 100 Hz 启动。

## 已处理的问题

- 外部环境曾把 6.0.1 的 `ISAAC_PATH`/`EXP_PATH` 注入 5.1.0；统一入口现已清理冲突变量。
- 本机 Isaac Sim 5.1.0 包缺少启动扩展所需的 Python 依赖。项目现在在
  `vendor/isaac_sim_5_1_python/` 固定并隔离提供：
  - `click==8.1.7`
  - `psutil==5.9.8`
  - `typing_extensions==4.12.2`
- Kit 嵌入式 Python 忽略普通 `PYTHONPATH`，统一入口使用
  `/app/python/extraPaths` 加载项目补充依赖。
- ROS 2 Python 脚本的关闭流程改用 `rclpy.try_shutdown()`，避免重复 shutdown。

## 限制与警告

- `assets/scenes/r1_workcell/workcell.usd` 已发现 `Base_Drive_Graph`，包含底盘差速命令订阅和驱动节点。
- USD 有一条 `base_link/visuals` 未解析引用警告，可能影响底盘基础视觉显示；
  已识别的控制图、关节和相机 Prim 不受影响。
- 短时 ROS 烟雾测试没有让 Isaac 时间线进入 Play，因此
  `topic_based_ros2_control` 会等待初始 `/isaac_joint_states`，控制器不会在该测试中
  完成加载。这是预期的联调边界，不是构建失败。
- 烟雾测试使用外部 timeout 强制停止，第二次 SIGINT 可打断节点清理并产生退出日志；
  正式运行应在 Isaac 正常停止后只按一次 `Ctrl+C`。
- controller manager 报告无法启用 FIFO 实时调度；这不阻止功能启动，但高频正式实验
  可再按 ros2_control 的实时调度建议配置系统权限。

## 正式启动

终端 A：

```bash
cd ${PROJECT_ROOT}
./scripts/project_control.sh start-isaac \
  ${PROJECT_ROOT}/assets/scenes/r1_workcell/workcell.usd
```

场景加载后按 Play。

终端 B：

```bash
cd ${PROJECT_ROOT}
./scripts/project_control.sh launch-ocs2
```

终端 C：

```bash
cd ${PROJECT_ROOT}
./scripts/project_control.sh sim1-check
./scripts/project_control.sh sim1-reset
./scripts/project_control.sh sim1-teach
```

详细参数见 `docs/archive/startup-guide-alias.zh-CN.md`。
