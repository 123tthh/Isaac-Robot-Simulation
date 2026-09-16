> **Historical record / 历史记录**：本页描述其记录日期的状态；当前运行以[中文指南](../docs/getting-started.zh-CN.md) / [English guide](../docs/getting-started.md)为准。Historical failures and paths are not current release claims.

# 615scene 六路相机录制/展示诊断（2026-07-23）

## 结论

6 月数据的六路相机均有有效帧；7 月 15 日数据的六路相机全部为 0 帧。7 月失败不是“视频文件太短”而是录制启动时发现六个 ROS publisher 均为 0：

`head_rgb, head_depth, left_rgb, left_depth, right_rgb, right_depth`

因此该轨迹不应被当作有效相机数据保留。

## 证据

| 轨迹 | 六路帧数 | 录制时 publisher 发现 | 结果 |
|---|---:|---|---|
| `manual_ocs2_keyboard_trace_20260616_164117` | 约 10,828–10,834/路 | 6/6 | 有效 |
| `manual_ocs2_keyboard_trace_20260616_170217` | 约 6,954–6,957/路 | 6/6 | 有效 |
| `manual_ocs2_keyboard_trace_20260616_174453` | 约 17,437–17,446/路 | 6/6 | 有效 |
| `manual_ocs2_keyboard_trace_20260715_133454` | 0/路 | 0/6 | 失败 |

证据文件：各轨迹 `camera/summary.json` 与同名 `.metadata.json`。

## 已修正

- `tools/camera_recorder.py` 默认 QoS 改为 `best_effort`；这是 Isaac Sim 5.1.0 传感器图像的兼容默认值。
- `tools/camera_grid_viewer.py` 默认 QoS 改为 `best_effort`；仍为单个 OpenCV 窗口中的 2×3 六格布局：上排 RGB（head/left/right），下排 depth（head/left/right）。
- `keyboard_ocs2_gripper_teleop.py` 的 `--camera-reliability` 默认改为 `best_effort`。
- 录制启动前最多等待 5 秒重新发现相机 publisher，避免 Isaac Sim 刚进入 Play 时立即把 publisher 误报为缺失。
- 空帧录制仍会在元数据中明确 `camera_warning`，不会静默生成“看似成功”的相机数据。

## 运行前检查

1. 用 `workcell.usd` 启动 Isaac Sim。
2. 进入 Play，使 `/World/ActionGraphs/Camera_Publish_Graph` 执行。
3. 运行 `./game_control.sh camera-grid`，确认六格状态从 `pub=0/no publisher` 变为 `pub=1` 并出现帧计数。
4. 再运行 `teach --record-cameras --open-camera-grid`。如果仍为 0，保留该轨迹的 metadata/log 作为失败诊断，不纳入有效数据集。

## 2026-07-23 日志根因补充

用户提供的 Isaac Sim 5.1.0 日志同时出现：

```text
Failed to find rigid body at .../Realsense_H/RSD455
.../Realsense_L/RSD455
.../Realsense_R/RSD455
Provided pattern list did not match any rigid bodies
```

这是 `workcell.usd` 中 RealSense 远程 S3 引用未解析导致的级联故障；仅调整 ROS QoS 不能修复该问题。现在已下载官方 Isaac Sim 5.1 `rsd455.usd` 到 `assets/scenes/r1_workcell/isaac_assets/`，并把三个引用改成相对本地引用 `./isaac_assets/rsd455.usd`。场景仍保留 `base.usd` 内已知的 `base_link/visuals` 未解析 prim 警告，但不再依赖 RealSense 网络资产。

修复后必须关闭旧 Isaac Sim 进程，重新用目标场景启动并按 Play；旧进程不会自动重新组合已打开的 USD 层。

## 修复后只读验证

`project_control.sh inspect-usd ... --show-camera-attributes` 已确认六个
`isaacsim.ros2.bridge.ROS2CameraHelper` 节点仍存在且 `inputs:enabled=True`：

- `head_cam/color/image_raw`、`head_cam/depth/image_rect_raw`
- `left_cam/color/image_raw`、`left_cam/depth/image_rect_raw`
- `right_cam/color/image_raw`、`right_cam/depth/image_rect_raw`

验证退出码为 0；本次无 GPU 的 headless 检查只验证 USD 组合与图配置，不替代桌面 Isaac Sim 按 Play 后的实时 ROS topic 验收。

## 2026-07-24 `workcell.usd` 运行时复核

通过 Isaac Sim 5.1.0 Kit 运行时重新检查后，发现原始问题不是六个相机
Prim 或 RenderProduct 路径，而是 `/World/ActionGraphs/Camera_Publish_Graph`
中 7 个节点缺少 `NodeGraphNodeAPI`。这些节点虽有正确的执行连接，却没有被
OmniGraph 调度，因此 ROS 2 camera writer 没有挂载。

已将以下节点补回 `NodeGraphNodeAPI` 并保存到目标场景：

- `on_playback_tick`
- `ros2_context`
- `cam_head_rgb`、`cam_head_depth`
- `cam_left_rgb`、`cam_left_depth`
- `cam_right_rgb`、`cam_right_depth`

修复前的场景备份：
`backups/615scene_camera_repair_20260723/workcell.usd.before_camera_graph_schema_20260724`

修复后的 Kit 日志已出现六路 `isaacsim.ros2.bridge.ROS2PublishImage`
`Attaching`，对应话题为：

`head_cam/{color/image_raw,depth/image_rect_raw}`、
`left_cam/{color/image_raw,depth/image_rect_raw}`、
`right_cam/{color/image_raw,depth/image_rect_raw}`。

因此当前目标确实是 `assets/scenes/r1_workcell/workcell.usd`，且六路
相机发布图已修复。日志中的 `RSD455` PhysX tensor rigid-body pattern 错误
仍是原场景中嵌套刚体配置的独立问题；强行给三个 RSD455 增加刚体会造成嵌套
刚体错误，所以未采用该方式。`base.usd` 的未解析视觉引用和部分 MDL
缺失也属于独立的显示/材质警告。

## Script Editor 操作入口

可在 Isaac Sim 的 Script Editor 打开并执行：
`scripts/repair_scene_cameras.py`。

脚本默认 `APPLY_REPAIR = False`，只读取当前已打开的 `workcell.usd`，并生成
`reports/615scene_camera_script_editor_20260724.md`。确认报告后，将脚本顶部
改为 `APPLY_REPAIR = True` 再执行，会先创建带时间戳的 USD 备份，再补齐缺失的
`NodeGraphNodeAPI`；修复后关闭并重新打开场景。
