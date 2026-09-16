# Local validation / 本机验证 — 2026-09-16

This supersedes the runtime conclusions in `LOCAL_VALIDATION_20260915.md`.
本报告更新昨日“完整场景 ROS Bridge 未打通”的结论。旧日志保留为历史证据。

## Measured results / 实测结果

- Full Isaac Sim 5.1 scene + ROS Bridge + OCS2 starts and runs with the GUI.
  完整场景、桥接、OCS2 与 GUI 已运行。
- USD wrapper fixed: original `Z` / `1.0 m` / `60 Hz` stage metadata is copied to
  the root layer. The previous wrapper defaulted to Y-up / 0.01 m.
- Kit gets an update after enabling ROS extensions and before stage loading;
  the earlier native crash no longer reproduces in these runs.
- External ROS now uses Fast DDS like Isaac. Reliable camera subscribers receive
  all six streams. Prior mixed middleware / best-effort runs lost entire streams.
- RViz target manager is disabled for pygame sessions. It previously published
  competing end-effector goals. A chassis odometry graph now publishes `/odom`.
- Timing settings were adjusted for interactive operation. Measured playback is
  about 0.65–0.75× wall time with six cameras on this host, not guaranteed real-time.
- OCS2 pose weights changed from position 10/orientation 5 to 1000/100 while
  preserving joint limits. Static final errors: left 0.415 mm, right 0.884 mm.
  This verifies small local motions only, not an arbitrary workspace or grasp.

## Final recorded sample / 最终样本

`outputs/sessions/auto10s_synced_20260916/trace.csv` contains 491 rows from a
10-second wall-time automation loop; first-to-last saved row span is **9.801 s**,
with **6.317 s** of simulation time. ROS trace time and camera stamps now share
simulation time. Six cameras contain **375–376 frames each**, 640×480, RGB `rgb8`
and depth `32FC1`. Final tool errors are **0.419 mm left / 0.953 mm right**.

The automatic base segment logged commands but only ~0.026 mm net motion: an
idle pygame handler also sent stop commands before the automatic forward command.
The handler conflict has been corrected; per the user's request, no new recording
was made after that correction. The earlier real-keyboard run measured ~0.215 m
forward motion in 3 seconds (`outputs/sessions/realtime_20260916/trace.csv`).
自动样本的底盘段不能视为有效运动演示；键盘底盘运动在前一段有独立实测证据。

## Export checks / 导出检查

Both v2.1 and v3.0 exports passed the local validator:

- 131 frames each at exact 20 FPS after idle removal (6.55 seconds of video).
- Three RGB videos each decode to exactly 131 frames at 640×480.
- Finite joint state, pose, effort and action values; consistent feature dimensions.
- Correct v3 episode video chunk/file/time offsets.
- Three lossless float32 depth sidecars, 251/251/252 frames with timestamp index.

Outputs:
`projects/teleoperation/sim1/trace_data/lerobot_v21/auto10s_synced_20260916_cleaned`
and the matching `lerobot_v30` directory. Machine-readable evidence:
[recording](recording_validation_20260916.json),
[export](lerobot_export_validation_20260916.json).

The converter previously copied whole videos without matching row selection/FPS;
exports now resample data and encode selected camera frames on the same 20 Hz grid.
LeRobot standard video features contain RGB; depth is an auxiliary lossless index.
The current 16-element arm/gripper interface does not include base commands.
`action` is next measured state. No official training-loader or policy-training
compatibility test was performed. Do not report this as a trained-model evaluation.

## Reproduction / 复现

Use [English operations](../docs/getting-started.md) or
[中文指南](../docs/getting-started.zh-CN.md). `sim1-session` owns its child processes and
finalizes recordings on exit. No new recordings are required to inspect these results.
Earlier raw episodes were not supplied; these are new local validation samples.

Reference for format and synchronization requirements:
[official LeRobot dataset implementation](https://github.com/huggingface/lerobot/blob/main/src/lerobot/datasets/lerobot_dataset.py).
NVIDIA's installed `test_ros2_odometry.py` supplies the odometry graph pattern;
its `carter_stereo.py` example supplies the bridge-before-stage update sequence.

## Freeze and independent evaluation / 冻结与独立测试

The [core freeze](../releases/2026-09-16/FREEZE.md) contains 2,912 file hashes and
a compact source archive. The subsequent [OmniSim evaluation](../evaluations/omnisim/README.md)
is outside that core: compatibility import and bounded gripper motion work, but
object contact/grasp validation remains unresolved. No new Isaac recording was
made after the user's stop instruction.

核心冻结后仅补充独立测试和文档；未继续 Isaac 录制。OmniSim 的方块/地面对照
失败及关节限位查询差异均记录在独立报告，不能视为完整抓取验证通过。
