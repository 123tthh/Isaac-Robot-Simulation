# 工程文档索引

本索引将当前可执行说明与历史研究记录分开。启动项目时优先阅读“当前入口”，历史文档不作为路径或命令的唯一依据。

## 当前入口

- [项目离职交接文档（2026-07-30）](PROJECT_HANDOVER_20260730_zh.md)：项目总览、依赖、接口、脚本、完成度、风险与接手顺序。
- [项目启动指南（2026-07-23）](startup_guide_20260723_zh.md)：宿主机启动、构建、运行检查与停止方法。
- [根目录说明](../README.md)：工程范围、目录和数据策略。
- [SIM1 工程总览](../projects/Trajectory/SIM1/PROJECT_OVERVIEW.md)：SIM1 模块和数据流。
- [SIM1 操作手册](../projects/Trajectory/SIM1/GAME_GUIDE.md)：示教、相机、底盘和回放参数。
- [轨迹格式](../projects/Trajectory/SIM1/TRACE_DATA_FORMAT.md)：CSV、相机和 LeRobot 格式。
- [615scene 相机诊断](../reports/camera_recording_diagnosis_20260723.md)：6 月/7 月相机帧数、QoS 与启动等待修复。
- [615scene USD 迁移记录](../reports/usd_scene_migration_20260723.md)：依赖清单、校验和与清理范围。
- [Docker 使用说明](../docker/README_DOCKER.md)：容器构建与启动。
- [Docker 当前验证记录](../docker/RUNTIME_VALIDATION.md)：正式镜像 ID、116 包编译结果与验证边界。
- [六路相机重建脚本](../scripts/rebuild_six_camera_render_products_20260730.py)：每次打开场景后重建 Session Layer RenderProduct 并绑定实际返回路径。

## 运维与验证

- [项目可用性验证报告（2026-07-23）](../reports/project_use_validation_20260723.md)
- [环境兼容性报告（2026-07-23）](../reports/environment_compatibility_report_20260723.md)
- [项目内容清单（2026-07-23）](../reports/project_inventory_20260723.md)
- [轨迹清理报告（2026-07-23）](../reports/trace_data_cleanup_report_20260723.md)
- [LeRobot v2.1 数据删除报告（2026-07-23）](../reports/lerobot_v21_deletion_report_20260723.md)

## 专项参考

- `projects/Trajectory/SIM1/graph/`：Isaac Action Graph 和场景结构导出。
- `projects/Trajectory/SIM1/isaac_script_editor_diagnostics/`：Isaac Script Editor 只读诊断。
- `projects/Trajectory/SIM1/底盘差速驱动/`：底盘控制说明与调试经验。
- `projects/Trajectory/SIM1/OCS2_DYNAMIC_NULLSPACE_REGULARIZATION.md`：OCS2 动态正则说明。

## 历史资料

以下文件保留用于追溯，但可能包含旧路径、旧参数或阶段性结论：

- `projects/Trajectory/SIM1/CLAUDE_HANDOFF_GRIPPER_PROJECT.md`
- `projects/Trajectory/SIM1/ROS2_LOG_CLEANUP_PLAN.md`
- `projects/Trajectory/Phase3_TaskPlan_updated.md`
- `data/ros2_log/`
- `spec/docker_build_requirements_zh.md`

这些资料仅用于了解原始工程背景；当前运行结论以交接文档和 Docker 当前验证记录为准。
