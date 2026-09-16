> **Historical record / 历史记录**：本页描述其记录日期的状态；当前运行以[中文指南](../docs/getting-started.zh-CN.md) / [English guide](../docs/getting-started.md)为准。Historical failures and paths are not current release claims.

# 615scene 相机 Script Editor 检查报告

生成时间：`2026-07-24T10:21:12`
目标场景：`${PROJECT_ROOT}/assets/scenes/r1_workcell/workcell.usd`
检查模式：`READ_ONLY`

- root_layer: `file:${PROJECT_ROOT}/assets/scenes/r1_workcell/workcell.usd`

## Camera prims

| side | RSD455 | color camera | depth camera |
|---|---|---|---|
| H | `True` | `True` | `True` |
| L | `True` | `True` | `True` |
| R | `True` | `True` | `True` |

## Camera_Publish_Graph

| node | NodeGraphNodeAPI |
|---|---|
| `on_playback_tick` | `True` |
| `ros2_context` | `True` |
| `cam_head_rgb` | `True` |
| `cam_head_depth` | `True` |
| `cam_left_rgb` | `True` |
| `cam_left_depth` | `True` |
| `cam_right_rgb` | `True` |
| `cam_right_depth` | `True` |

## ROS 2 camera writer configuration

| stream | topic | type | render product |
|---|---|---|---|
| `head_rgb` | `head_cam/color/image_raw` | `rgb` | `/Render/OmniverseKit/HydraTextures/head_rgb` |
| `head_depth` | `head_cam/depth/image_rect_raw` | `depth` | `/Render/OmniverseKit/HydraTextures/head_depth` |
| `left_rgb` | `left_cam/color/image_raw` | `rgb` | `/Render/OmniverseKit/HydraTextures/left_rgb` |
| `left_depth` | `left_cam/depth/image_rect_raw` | `depth` | `/Render/OmniverseKit/HydraTextures/left_depth` |
| `right_rgb` | `right_cam/color/image_raw` | `rgb` | `/Render/OmniverseKit/HydraTextures/right_rgb` |
| `right_depth` | `right_cam/depth/image_rect_raw` | `depth` | `/Render/OmniverseKit/HydraTextures/right_depth` |

## 结论

八个相机图节点均含 `NodeGraphNodeAPI`，无需图架构修复。
相机 Prim、ROS 2 话题和 RenderProduct 静态配置均通过。
