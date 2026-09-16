> **Historical record / 历史记录**：本页描述其记录日期的状态；当前运行以[中文指南](../docs/getting-started.zh-CN.md) / [English guide](../docs/getting-started.md)为准。Historical failures and paths are not current release claims.

# LeRobot v3.0 派生数据清理报告（2026-07-24）

已删除旧的 LeRobotDataset v3.0 派生输出，共约 91 MB：

- `/home/gtk/ros2_log/SIM1/trace_data/lerobot_v30`
- `${PROJECT_ROOT}/projects/teleoperation/sim1/trace_data/lerobot_v30`

`/home/gtk/teleoperation/sim1` 是工程 `projects/teleoperation` 的软链接，因此没有
额外第三份数据。

保留内容：

- `trace_cleaning/scripts/convert_ocs2_trace_to_lerobot_v30.py`
- `trace_cleaning/scripts/process_sim1_episode.py`
- 原始 CSV、清洗数据、相机数据及 v2.1 转换脚本

因此仍可按 `GAME_GUIDE.md` 或 `process_sim1_episode.py` 重新生成 v3.0 数据；
本次只删除已有派生结果，不影响生成能力。
