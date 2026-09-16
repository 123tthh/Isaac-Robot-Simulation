# 示教轨迹记录格式与数据说明

工程总纲见 [PROJECT_OVERVIEW.md](PROJECT_OVERVIEW.md)。操作命令和示教模式见 [GAME_GUIDE.md](GAME_GUIDE.md)。

本文说明 `~/teleoperation/sim1` 下示教、清洗、标注、转换后的数据格式。

## 1. 数据目录

默认数据目录如下：

```text
trace_data/
  raw/          原始示教 CSV 和同名失败标注
  cleaned/      清洗后可回放 CSV 和清洗报告
  direct/       LeRobot 直连回放时记录的检查 CSV
  lerobot_v30/  由清洗后 CSV 转出的 LeRobot dataset v3.0 目录
  lerobot_v21/  由清洗后 CSV 转出的 LeRobot dataset v2.1 目录
```

原始示教默认写入：

```text
trace_data/raw/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS.csv
trace_data/raw/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS.diagnostics.csv
trace_data/raw/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS.metadata.json
trace_data/raw/manual_ocs2_keyboard_trace_latest.csv
trace_data/raw/manual_ocs2_keyboard_trace_latest.diagnostics.csv
```

`manual_ocs2_keyboard_trace_latest.csv` 是最近一次成功示教的软链接，`./game_control.sh demo` 默认优先使用它。
`manual_ocs2_keyboard_trace_latest.diagnostics.csv` 是最近一次成功示教的诊断 CSV 软链接。

启用相机同步录制时，相机数据写入同一 run_id 对应的目录，不改变主 CSV 路径：

```text
trace_data/raw/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS/
  camera/
    videos/
      head_rgb.mp4
      left_rgb.mp4
      right_rgb.mp4
    depth/
      head_depth/*.npy
      left_depth/*.npy
      right_depth/*.npy
    camera_timestamps.csv
    summary.json
    camera_recorder.log
```

`camera_timestamps.csv` 保存每帧 ROS timestamp、wall timestamp 和相对文件路径。后续转换应按 timestamp 对齐轨迹和图像，不按帧序号强行对齐。

## 2. 原始示教 CSV

原始 CSV 由 `keyboard_ocs2_gripper_teleop.py` 在 `./game_control.sh teach` 时生成。每一行表示一次按键事件或周期性 tick。

字段如下：

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `wall_time` | float seconds | 系统 wall clock 时间戳，用于回放调度 |
| `ros_time_sec` | float seconds | ROS clock 时间戳 |
| `selected` | string | 当前键盘控制对象：`left`、`right`、`base` |
| `event` | string | 本行事件，如 `tick`、`w`、`left_close`、`base_forward`、`recover_right_initial` |
| `left_target_xyz` | 3 floats string | 左臂 OCS2 target 位置，格式 `x y z` |
| `right_target_xyz` | 3 floats string | 右臂 OCS2 target 位置，格式 `x y z` |
| `left_target_xyzw` | 4 floats string | 左臂 OCS2 target 四元数，格式 `x y z w` |
| `right_target_xyzw` | 4 floats string | 右臂 OCS2 target 四元数，格式 `x y z w` |
| `left_current_xyz` | 3 floats string | 左臂 OCS2 current 位置 |
| `right_current_xyz` | 3 floats string | 右臂 OCS2 current 位置 |
| `left_current_xyzw` | 4 floats string | 左臂 OCS2 current 四元数 |
| `right_current_xyzw` | 4 floats string | 右臂 OCS2 current 四元数 |
| `left_gripper_cmd` | float | 左夹爪命令，`>0` 打开，`<=0` 闭合/保持 |
| `right_gripper_cmd` | float | 右夹爪命令，`>0` 打开，`<=0` 闭合/保持 |
| `left_gripper_closed` | int/string | 左夹爪闭合状态，`1` 闭合/保持，`0` 打开 |
| `right_gripper_closed` | int/string | 右夹爪闭合状态，`1` 闭合/保持，`0` 打开 |
| `joint_names` | names string | `/joint_states.name`，空格分隔 |
| `joint_positions` | floats string | `/joint_states.position`，顺序对应 `joint_names` |
| `effort_joint_names` | names string | 力矩来源 joint state 的 name，优先 `/isaac_joint_states.name` |
| `joint_efforts` | floats string | 关节 effort/力矩，顺序对应 `effort_joint_names` |

力矩来源规则：

```text
优先记录 /isaac_joint_states.effort
如果没有收到 /isaac_joint_states，则回退 /joint_states.effort
如果两者都没有 effort，则 joint_efforts 为空
```

用于 LeRobot 转换时，必须能从 `joint_names/joint_positions` 和 `effort_joint_names/joint_efforts` 中找到以下 14 个机械臂关节：

```text
left_joint1 left_joint2 left_joint3 left_joint4 left_joint5 left_joint6 left_joint7
right_joint1 right_joint2 right_joint3 right_joint4 right_joint5 right_joint6 right_joint7
```

## 3. 诊断 CSV

每次示教会额外生成一个同名诊断 CSV：

```text
trace_data/raw/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS.diagnostics.csv
```

诊断 CSV 与原始示教 CSV 同步写入，每一行对应同一次 `event` 或 `tick`。它的目的不是回放，而是排查 OCS2、FSM、target 跟踪误差、关节力矩是否异常。

核心字段如下：

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `wall_time` | float seconds | 系统 wall clock 时间戳 |
| `ros_time_sec` | float seconds | ROS clock 时间戳 |
| `event` | string | 对应主 CSV 的事件 |
| `selected` | string | 当前控制对象 |
| `fsm_command` | int/string | 最近收到的 `/fsm_command` |
| `fsm_is_ocs2` | int/string | `fsm_command == 3` 时为 `1`，否则为 `0`；未收到时为空 |
| `fsm_not_ocs2_count` | int | 记录期间收到非 OCS2 FSM 命令的累计次数 |
| `left_target_current_dx/dy/dz` | float | 左臂 target xyz 减 current xyz |
| `left_target_current_norm` | float | 左臂 target-current 位置误差范数 |
| `right_target_current_dx/dy/dz` | float | 右臂 target xyz 减 current xyz |
| `right_target_current_norm` | float | 右臂 target-current 位置误差范数 |
| `left_orientation_error_deg` | float | 左臂 target-current 姿态误差，单位 degree |
| `right_orientation_error_deg` | float | 右臂 target-current 姿态误差，单位 degree |
| `blocked_translation_count` | int | 示教安全门控累计拒绝的平移命令数量 |
| `blocked_rotation_count` | int | 示教安全门控累计拒绝的旋转命令数量 |
| `workspace_clamp_count` | int | target 被示教起点工作空间边界钳制的累计次数 |
| `hard_lag_block_count` | int | target-current 超过 hard lag 且不再收敛时被硬阻止的累计次数 |
| `watchdog_freeze_right` | int | 运行时 watchdog 是否已冻结右臂 target，`1` 表示冻结 |
| `watchdog_freeze_count` | int | watchdog 触发右臂冻结的累计次数 |
| `watchdog_severe_count` | int | watchdog 检测到严重发散的累计次数 |
| `watchdog_joint_limit_count` | int | watchdog 检测到 `right_joint1` 超出安全范围的累计次数 |
| `left_gripper_closed` | int/string | 左夹爪闭合状态 |
| `right_gripper_closed` | int/string | 右夹爪闭合状态 |
| `ocs2_sqp_not_converged_count` | int | OCS2 日志中匹配到 `SQP did not converge` 的累计次数 |
| `ocs2_cost_explosion_count` | int | OCS2 日志中 cost 为 NaN/Inf 或超过阈值的累计次数 |
| `ocs2_last_cost` | float | 最近一次从 OCS2 日志中解析到的 cost |
| `ocs2_last_log_match` | string | 最近一次异常日志行，已压缩为空格分隔 |
| `camera_recorder_status` | string | 相机 recorder 状态，如 `disabled`、`running`、`stopped`、`failed_to_start` |
| `camera_output_dir` | string | 本次相机输出目录，未启用时为空 |
| `camera_warning` | string | 相机 topic publisher count 为 0、启动失败或退出异常时的 WARNING |

诊断 CSV 还包含数值化关节轨迹列：

```text
left_joint1_position ... left_joint7_position
right_joint1_position ... right_joint7_position
left_joint1_effort ... left_joint7_effort
right_joint1_effort ... right_joint7_effort
```

这些列直接用于画 joint1 到 joint7 的轨迹和力矩曲线，不需要再解析主 CSV 中的 `joint_names/joint_positions` 字符串。

OCS2 日志扫描默认目录：

```text
$ROS_LOG_DIR/latest
```

如果没有 `ROS_LOG_DIR/latest`，则扫描：

```text
~/.ros/log
```

可以在示教时指定日志目录和 cost 爆炸阈值：

```bash
./game_control.sh teach --ocs2-log-dir ~/.ros/log/latest --ocs2-cost-explosion-threshold 1000000
```

默认 `./game_control.sh teach` 不启用 workspace 可达域边界；只有显式传入成对的 `left/right-max-target-offset` 和 `left/right-max-axis-offset` 时，才会启用对应手臂的 workspace clamp。

左右臂 workspace clamp 示例。左右臂参数数值一致；涉及 Y 方向时按左右镜像解释：

```bash
./game_control.sh teach \
  --step 0.003 \
  --rot-step-deg 0.5 \
  --left-max-target-offset 0.45 \
  --left-max-axis-offset 0.40 0.40 0.25 \
  --right-max-target-offset 0.45 \
  --right-max-axis-offset 0.40 0.40 0.25
```

含义：

- `max-position-error`：target-current 超过该值后，只允许误差减小的恢复方向。
- `hard-position-error`：超过该值且 candidate 不能减小误差时，直接要求暂停或按 `Z/X` 慢速恢复对应手臂。
- `left/right-max-target-offset`：target 相对本次示教起点的球面最大偏移。
- `left/right-max-axis-offset`：target 相对本次示教起点的 xyz 分轴最大偏移。
- `workspace_clamp_count` 增加表示脚本已经把 target 钳到边界，继续同方向按键不会再外推。

workspace clamp 触发时终端会打印命中的边界：

```text
[reach] right workspace clamp: offset_from_center=(+0.400, -0.043, +0.012) m, axis_limit=(0.400, 0.400, 0.250) m, sphere_norm=0.402/0.450 m, hit_axis=X+, hit_axis_arm=X+
```

`event` 字段还会标记输入来源和保护动作。`terminal` 模式中单次按键通常记录为按键名或 `blocked_translation`、`blocked_rotation`；`pygame` 模式中连续速度输入会记录为 `pygame_translation`、`pygame_rotation` 或 `pygame_motion`。watchdog 冻结后继续操作右臂会记录 `watchdog_blocked_key`。

Y 方向镜像规则：

```text
left  Y+ = Y_OUT, Y- = Y_IN
right Y- = Y_OUT, Y+ = Y_IN
```

默认 `./game_control.sh teach` 不启用运行时 watchdog。显式传入以下参数后才启用：

```bash
./game_control.sh teach \
  --watchdog-position-error 0.15 \
  --watchdog-orientation-error-deg 30.0 \
  --watchdog-severe-position-error 0.30 \
  --watchdog-severe-orientation-error-deg 60.0 \
  --right-joint1-lower-safety -2.2 \
  --right-joint1-upper-safety 1.2
```

启用后的含义：

```text
right target-current position error > 0.15 m  -> capture current pose once as frozen target
right orientation error > 30 deg              -> capture current pose once as frozen target
right target-current position error > 0.30 m  -> severe warning
right orientation error > 60 deg              -> severe warning
right_joint1 outside [-2.2, +1.2] rad         -> freeze right target
```

冻结后示教器会持续发布首次触发 watchdog 时捕获的 fixed frozen target，不会每个 tick 跟随最新 current；同时拒绝右臂平移旋转按键，直到按 `X` 慢速恢复右臂并重建 target。

注意：如果 OCS2 只把信息打印在另一个终端 stdout，且没有进入 ROS log 文件，诊断 CSV 无法捕获这些终端文本。此时应把 OCS2 launch 的输出同时写入文件，或检查 `~/.ros/log/latest` 下是否有对应日志。

## 4. 事件种类

常见 `event` 值：

| 事件 | 含义 |
| --- | --- |
| `tick` | 周期记录行，没有新按键 |
| `w/s/a/d/r/f` | 平移 target |
| `base_forward` / `base_backward` | 选中底盘后，方向键上下发布底盘前进/后退命令 |
| `base_left_arc_90` / `base_right_arc_90` | 选中底盘后，方向键左右触发底盘左/右圆弧 90° |
| `base_stop` / `base_blocked_multi_arrow` | 底盘停止，或同时按多个方向键被阻止 |
| `u/o/i/k/j/l` | 旋转 target |
| `blocked_translation` | 平移命令被安全门控拒绝，target 未继续推远 |
| `blocked_rotation` | 旋转命令被安全门控拒绝，target 姿态未继续推远 |
| `toggle_gripper` | 切换选中夹爪开合 |
| `toggle_gripper_ignored` | 选中底盘时按 `Space`，没有夹爪动作 |
| `left_close` / `left_open` | 左夹爪闭合/打开 |
| `right_close` / `right_open` | 右夹爪闭合/打开 |
| `recover_left_initial` / `recover_right_initial` | 按 `Z/X`，对应手臂通过 MoveJ 慢速恢复最佳初始姿态并重建 target |
| `recover_left_failed` / `recover_right_failed` | 恢复初始姿态时缺少关节状态等原因失败 |
| `quit` | 按 `Q` 或 `Esc` 结束记录 |
| `keyboard_interrupt` | Ctrl+C 中断 |
| `reset_after_task` | 退出时执行复位后记录的状态 |

清洗脚本会移除 `quit`、`keyboard_interrupt`、`reset_after_task` 等退出/复位行，保留动作相关区间。

## 5. 清洗后数据

清洗脚本：

```bash
./trace_cleaning/scripts/clean_ocs2_trace.py trace_data/raw/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS.csv
```

带相机数据的 episode 应使用 SIM1 episode 清洗脚本：

```bash
./trace_cleaning/scripts/clean_sim1_episode.py \
  trace_data/raw/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS.csv
```

默认输出：

```text
trace_data/cleaned/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS_cleaned.csv
trace_data/cleaned/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS_cleaned.report.md
trace_data/cleaned/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS_cleaned/
  camera/
    videos/*.mp4
    depth/*/*.npy
    camera_timestamps.csv
    summary.json
```

清洗后 CSV 字段与原始 CSV 相同，但会做两件事：

- 删除长时间无动作、只观察但未控制的数据段。
- 压缩 `wall_time` 和 `ros_time_sec` 的时间间隔，使回放不会在观察空档停很久。

`clean_sim1_episode.py` 会把同一保留区间应用到相机数据：RGB mp4 被重新编码裁剪，depth npy 按 timestamp 筛选复制，`camera_timestamps.csv` 使用与 cleaned trace 一致的压缩后 wall/ROS timestamp。后续 LeRobot 转换仍按 timestamp 最近邻对齐，不按帧序号对齐。

清洗报告包含：

```text
original_rows
cleaned_rows
removed_rows
original_duration_sec
cleaned_duration_sec
kept_intervals
kept event counts
```

## 6. 失败标注文件

`./game_control.sh teach` 正常结束后，会生成同名标注：

```text
trace_data/raw/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS.label.md
```

标注选项：

```text
success
joint_torque_disappeared
singularity_suspected
other_failure
unlabeled_invalid_choice
unlabeled_noninteractive
```

该文件用于记录本次示教是否发生失败、力矩突然消失或疑似奇异。

同名 metadata 文件：

```text
trace_data/raw/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS.metadata.json
```

metadata 记录 trace、diagnostics、camera 输出目录、相机 recorder 状态、各相机 topic 的 publisher count，以及“按 ROS/wall timestamp 对齐，不按帧序号对齐”的同步说明。

## 7. LeRobot Dataset 转换

一键处理一次 raw 采集：

```bash
./trace_cleaning/scripts/process_sim1_episode.py \
  trace_data/raw/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS.csv
```

该脚本会保留 raw 不动，生成 cleaned 数据，然后分别输出 v3.0 和 v2.1：

```text
trace_data/lerobot_v30/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS_cleaned/
trace_data/lerobot_v21/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS_cleaned/
```

`./game_control.sh teach` 成功录制后默认会执行该后处理；如需采集完整 raw RGB+D 但跳过自动清洗/转换，可设置：

```bash
SIM1_PROCESS_AFTER_TEACH=0 ./game_control.sh teach --input-mode pygame \
  --linear-speed 0.025 \
  --angular-speed-deg 4.0 \
  --control-rate 50 \
  --base-speed 0.25 \
  --base-yaw-rate 1.0 \
  --base-turn-mode arc90 \
  --max-position-error 0.08 \
  --hard-position-error 0.15 \
  --max-orientation-error-deg 15.0 \
  --watchdog-position-error 0.15 \
  --watchdog-orientation-error-deg 30.0 \
  --watchdog-severe-position-error 0.30 \
  --watchdog-severe-orientation-error-deg 60.0 \
  --right-joint1-lower-safety -2.2 \
  --right-joint1-upper-safety 1.2 \
  --record-cameras \
  --camera-depth-save-every 1
```

LeRobot 官方文档差异：

- v2.1 是 episode-based：`data/chunk-000/episode_000000.parquet`、`videos/chunk-000/<video_key>/episode_000000.mp4`、`meta/episodes.jsonl`。
- v3.0 是 file-based：`data/chunk-000/file-000.parquet`、`videos/<video_key>/chunk-000/file-000.mp4`、`meta/episodes/chunk-000/file-000.parquet`。

转换脚本：

```bash
./trace_cleaning/scripts/convert_ocs2_trace_to_lerobot_v30.py \
  trace_data/cleaned/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS_cleaned.csv \
  --output trace_data/lerobot_v30/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS

./trace_cleaning/scripts/convert_ocs2_trace_to_lerobot_v21.py \
  trace_data/cleaned/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS_cleaned.csv \
  --output trace_data/lerobot_v21/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS
```

v3.0 输出目录：

```text
trace_data/lerobot_v30/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS/
  data/chunk-000/file-000.parquet
  videos/observation.images.head/chunk-000/file-000.mp4
  videos/observation.images.left_wrist/chunk-000/file-000.mp4
  videos/observation.images.right_wrist/chunk-000/file-000.mp4
  meta/info.json
  meta/stats.json
  meta/tasks.parquet
  meta/episodes/chunk-000/file-000.parquet
```

v2.1 输出目录：

```text
trace_data/lerobot_v21/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS/
  data/chunk-000/episode_000000.parquet
  videos/chunk-000/observation.images.head/episode_000000.mp4
  videos/chunk-000/observation.images.left_wrist/episode_000000.mp4
  videos/chunk-000/observation.images.right_wrist/episode_000000.mp4
  meta/info.json
  meta/tasks.jsonl
  meta/episodes.jsonl
  meta/episodes_stats.jsonl
```

主要数据列：

| LeRobot 字段 | 形状 | 含义 |
| --- | ---: | --- |
| `observation.state` | 16 | 14 个机械臂关节位置 + 左右夹爪闭合状态 |
| `observation.state.pose` | 16 | 左右 target 位姿各 7 维 + 左右夹爪闭合状态 |
| `observation.effort` | 14 | 14 个机械臂关节 effort/力矩 |
| `action` | 16 | 下一帧 `observation.state`，最后一帧复用自身 |
| `action.pose` | 16 | 下一帧 `observation.state.pose`，最后一帧复用自身 |
| `timestamp` | 1 | 从本 episode 第一帧开始的秒数 |
| `frame_index` | 1 | episode 内帧号 |
| `episode_index` | 1 | episode 编号 |
| `index` | 1 | dataset 全局帧号 |
| `task_index` | 1 | task 编号 |

`observation.state` 顺序：

```text
left_joint1 left_joint2 left_joint3 left_joint4 left_joint5 left_joint6 left_joint7
left_gripper_closed
right_joint1 right_joint2 right_joint3 right_joint4 right_joint5 right_joint6 right_joint7
right_gripper_closed
```

`observation.effort` 顺序：

```text
left_joint1 left_joint2 left_joint3 left_joint4 left_joint5 left_joint6 left_joint7
right_joint1 right_joint2 right_joint3 right_joint4 right_joint5 right_joint6 right_joint7
```

老轨迹如果没有力矩列，只能用于格式兼容检查：

```bash
./trace_cleaning/scripts/convert_ocs2_trace_to_lerobot_v30.py \
  trace_data/cleaned/manual_ocs2_keyboard_trace_20260529_154535_cleaned.csv \
  --output trace_data/lerobot_v30/manual_ocs2_keyboard_trace_20260529_154535_cleaned_lerobot_v30 \
  --allow-missing-effort \
  --missing-gripper-closed 1.0

./trace_cleaning/scripts/convert_ocs2_trace_to_lerobot_v21.py \
  trace_data/cleaned/manual_ocs2_keyboard_trace_20260529_154535_cleaned.csv \
  --output trace_data/lerobot_v21/manual_ocs2_keyboard_trace_20260529_154535_cleaned_lerobot_v21 \
  --allow-missing-effort \
  --missing-gripper-closed 1.0
```

这种情况下 `observation.effort` 会填 0，不能用于分析真实力矩消失。

## 8. 回放使用的数据

OCS2 target 回放使用 CSV：

```bash
TRACE=trace_data/cleaned/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS_cleaned.csv ./game_control.sh demo
```

回放只需要 CSV 中的：

```text
wall_time
left_target_xyz
right_target_xyz
left_target_xyzw
right_target_xyzw
left_gripper_cmd
right_gripper_cmd
```

`joint_positions`、`joint_efforts`、`*_gripper_closed` 主要用于数据集转换、诊断和失败分析。

如果 CSV 中存在 `base_cmd_vx` 和 `base_cmd_wz`，`replay_ocs2_target_trace.py` 会在回放 OCS2 target 和夹爪命令的同时发布：

```text
/sim1/base_diff_cmd
```

旧 CSV 没有底盘字段时，回放保持原行为。

## 9. 底盘独立记录 CSV

底盘独立记录写入：

```text
trace_data/base_raw/
  base_drive_trace_YYYYmmdd_HHMMSS.csv
  base_drive_trace_latest.csv
```

`base_drive_trace_latest.csv` 优先是指向最近一次记录的软链接；如果运行环境不支持软链接，则脚本退出时复制一份 latest 文件。

底盘控制语义以 Isaac Sim `Base_Drive_Graph` 当前使用的 `base_control/script_node.py` 为准。该 ScriptNode 使用 `/sim1/base_diff_cmd` 的 `linear.x` 控制直行/后退，用 `linear.x = 0` 时的 `angular.z` 正负触发左/右 90°圆弧转弯。

注意：`left_arc_90` / `right_arc_90` 不是 ROS2 侧持续角速度命令，而是外部触发命令。触发后，Isaac Sim ScriptNode 在内部继续执行圆弧闭环。没有 `/sim1/base_drive_debug` 时，ROS2 recorder 只能记录外部 `/sim1/base_diff_cmd` 输入和 `/isaac_joint_states` 的实际执行结果；有 `/sim1/base_drive_debug` 时，会额外记录 ScriptNode 内部每帧的 `arc_active`、目标 yaw、yaw 误差、内部 `cmd_vx/cmd_wz` 和理论左右轮命令。

记录脚本：

```bash
python3 base_drive_recorder.py --rate 50 --out-dir trace_data/base_raw
```

订阅 topic：

```text
/sim1/base_diff_cmd
/isaac_joint_states
/joint_states
/clock
/odom       可选，有则记录底盘位姿
/tf         可选，需显式 --record-tf
/tf_static  可选，需显式 --record-tf
```

字段：

| 字段 | 含义 |
| --- | --- |
| `wall_time` | 系统 wall clock 秒 |
| `ros_time_sec` | recorder 节点 ROS clock 秒 |
| `sim_time_sec` | 最近一次 `/clock` 仿真时间秒 |
| `base_cmd_vx` | 最近一次 `/sim1/base_diff_cmd.linear.x` |
| `base_cmd_wz` | 最近一次 `/sim1/base_diff_cmd.angular.z` |
| `base_cmd_mode` | `forward`、`backward`、`left_arc_90`、`right_arc_90`、`stop` 或 `manual` |
| `base_motion_event` | 由命令模式推导的事件，如 `cmd_forward`、`cmd_stop` |
| `base_left_wheel_cmd` / `base_right_wheel_cmd` | 有 `/sim1/base_drive_debug` 时来自 ScriptNode `left_wheel_cmd/right_wheel_cmd`；否则普通差速命令下按公式估算，`left_arc_90/right_arc_90` 时为空 |
| `base_debug_mode` | `/sim1/base_drive_debug` JSON 的 `mode`，例如 `IDLE`、`RAW_LINEAR`、`ARC_START`、`ARC_CLOSED_LOOP`、`ARC_DONE` |
| `base_arc_active` | debug JSON 的 `arc_active`，`1/0`；没有 debug 数据时为空 |
| `base_arc_armed` | debug JSON 的 `arc_armed`，`1/0` |
| `base_arc_dir` | debug JSON 的 `arc_dir`，左转通常为 `1`，右转通常为 `-1` |
| `base_arc_target_yaw_deg` | debug JSON 的 `target_yaw_deg` |
| `base_arc_current_yaw_deg` | debug JSON 的 `current_yaw_deg` |
| `base_arc_error_deg` | debug JSON 的 `error_yaw_deg` |
| `base_script_cmd_vx` / `base_script_cmd_wz` | ScriptNode 本帧实际用于差速解算的 `cmd_vx/cmd_wz` |
| `base_raw_vx` / `base_raw_wz` | ScriptNode 收到的外部原始 `raw_vx/raw_wz` |
| `base_arc_radius` / `base_arc_linear_speed` | debug JSON 中的圆弧控制参数快照 |
| `base_debug_frame` | debug JSON 的 `frame` |
| `base_debug_stamp_wall_time` | debug JSON 的 `stamp_wall_time` |
| `base_yaw_ok` | debug JSON 的 `yaw_ok`，`1/0` |
| `wheel_L_position` / `wheel_R_position` | 左右主动轮关节位置，优先来自 `/isaac_joint_states` |
| `wheel_L_velocity` / `wheel_R_velocity` | 左右主动轮关节速度，优先来自 `/isaac_joint_states` |
| `wheel_L_effort` / `wheel_R_effort` | 左右主动轮 effort，优先来自 `/isaac_joint_states` |
| `caster_LF/RF/LB/RB_position` | 四个 caster 关节位置 |
| `caster_LF/RF/LB/RB_velocity` | 四个 caster 关节速度 |
| `base_x` / `base_y` / `base_z` / `base_yaw_deg` | 从 `/odom` 或 TF 得到的底盘位姿；没有来源时为空 |
| `base_wheel_slip_hint` | 预留轮滑提示；第一版为空 |

关节状态优先级：

```text
/isaac_joint_states 中存在对应 joint 和字段 -> 使用 /isaac_joint_states
否则 -> fallback 到 /joint_states
仍缺失 -> 写空字符串
```

普通差速命令的 wheel command 估算：

```text
left  = (vx - wz * L / 2) / r
right = (vx + wz * L / 2) / r

base_left_wheel_cmd  = left  * LEFT_SIGN
base_right_wheel_cmd = right * RIGHT_SIGN

当前 script_node.py:
L = 0.55
r = 0.10
LEFT_SIGN = -1.0
RIGHT_SIGN = -1.0
```

没有 `/sim1/base_drive_debug` 时，`base_left_wheel_cmd/base_right_wheel_cmd` 只是外部命令的理论估算；`left_arc_90/right_arc_90` 下真实 wheel command 由 Isaac ScriptNode 内部闭环生成，因此写空，不伪造。有 `/sim1/base_drive_debug` 时，这两个字段改为记录 ScriptNode 实际输出给 ArticulationController 的理论轮速命令。分析实际运动时仍优先看 `wheel_L_velocity/wheel_R_velocity/wheel_L_effort/wheel_R_effort`，这些字段来自 `/isaac_joint_states`。

`base_cmd_mode` 判断规则：

```text
vx > 0                         forward
vx < 0                         backward
vx == 0 且 wz > 0              left_arc_90
vx == 0 且 wz < 0              right_arc_90
vx == 0 且 wz == 0             stop
其他                            manual
```

## 10. 示教 CSV 新增底盘字段

`keyboard_ocs2_gripper_teleop.py` 只在主 CSV 尾部追加底盘字段，原有字段顺序不变：

| 字段 | 含义 |
| --- | --- |
| `base_cmd_vx` | 本行最近发布的底盘 `linear.x` |
| `base_cmd_wz` | 本行最近发布的底盘 `angular.z` |
| `base_cmd_mode` | `forward/backward/left_arc_90/right_arc_90/stop/manual` |
| `base_motion_event` | `base_forward`、`base_backward`、`base_left_arc_90`、`base_right_arc_90`、`base_stop` 或 `base_blocked_multi_arrow` |
| `base_left_wheel_cmd` | 有 debug 数据时来自 ScriptNode `left_wheel_cmd`；否则普通差速命令下估算，arc90 时为空 |
| `base_right_wheel_cmd` | 有 debug 数据时来自 ScriptNode `right_wheel_cmd`；否则普通差速命令下估算，arc90 时为空 |
| `base_x` / `base_y` / `base_yaw_deg` | 如果 `/odom` 可用，则记录底盘平面位姿；否则为空 |
| `base_debug_mode` | `/sim1/base_drive_debug` JSON 的 `mode`；无 debug 数据时为空 |
| `base_arc_active` / `base_arc_armed` | debug JSON 的 arc 状态，`1/0` |
| `base_arc_dir` | debug JSON 的 `arc_dir` |
| `base_arc_target_yaw_deg` / `base_arc_current_yaw_deg` / `base_arc_error_deg` | debug JSON 的目标 yaw、当前 yaw 和误差 |
| `base_script_cmd_vx` / `base_script_cmd_wz` | ScriptNode 本帧内部闭环输出的底盘命令 |
| `base_raw_vx` / `base_raw_wz` | ScriptNode 本帧看到的外部原始命令 |
| `base_arc_radius` / `base_arc_linear_speed` | debug JSON 的圆弧参数 |
| `base_debug_frame` / `base_debug_stamp_wall_time` / `base_yaw_ok` | debug JSON 的帧号、wall time 和 yaw 读取状态 |

示教 diagnostics CSV 追加：

| 字段 | 含义 |
| --- | --- |
| `base_motion_enabled` | `1` 表示本次示教启用底盘方向键 |
| `base_blocked_multi_arrow_count` | pygame 模式多个方向键同时按下并被阻止的累计次数 |
| `base_arc_active` / `base_arc_armed` / `base_arc_dir` | 来自 `/sim1/base_drive_debug` 的 arc 状态；无 debug 数据时为空 |
| `base_arc_target_yaw_deg` / `base_arc_current_yaw_deg` / `base_arc_error_deg` | 来自 debug JSON 的 yaw 闭环状态 |
| `base_debug_mode` / `base_script_cmd_vx` / `base_script_cmd_wz` / `base_raw_vx` / `base_raw_wz` | ScriptNode 调试快照 |
| `base_debug_frame` / `base_debug_stamp_wall_time` / `base_yaw_ok` | ScriptNode 调试帧信息 |
| `base_wheel_slip_hint` | 预留轮滑提示；第一版为空 |

清洗脚本已将 `base_forward`、`base_backward`、`base_left_arc_90`、`base_right_arc_90`、`base_stop` 和 `base_blocked_multi_arrow` 视为动作事件，因此只有底盘运动的片段不会被 idle 清洗规则删除。
