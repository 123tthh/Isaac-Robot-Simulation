# head_rgb RenderProduct 修复报告（2026-07-24）

执行状态：`未执行；等待人工确认`

## 已生成内容

- 修复脚本：`scripts/repair_head_rgb_renderproduct_20260724.py`
- 默认开关：`DRY_RUN = True`
- 目标 USD：`assets/scenes/615scene_20260723/615scene.usd`
- 目标节点：`/World/ActionGraphs/Camera_Publish_Graph/cam_head_rgb`

## 计划的唯一修改

- 原 renderProductPath：`/Render/OmniverseKit/HydraTextures/head_rgb`
- 新 RenderProduct：由 `rep.create.render_product()` 使用 RGB camera
  `/World/Robot/camera_H_link/Realsense_H/RSD455/Camera_OmniVision_OV9782_Color`、
  分辨率 `(640, 480)`、名称 `head_rgb_test` 创建。
- 仅将 `cam_head_rgb.inputs:renderProductPath` 指向该新路径。

脚本在实际修改前会生成
`615scene_backup_before_head_rgb_fix_YYYYmmdd_HHMMSS.usd`，并在本报告中写入实际
RenderProduct 路径、其 `camera` relationship 和绑定结果。

## ROS 2 验收方法

在 Isaac Sim 重新打开场景并按 Play 后：

```bash
ros2 topic list | rg '^/head_cam/color/image_raw$'
ros2 topic info /head_cam/color/image_raw
ros2 topic hz /head_cam/color/image_raw
```

尚未执行脚本，未创建 RenderProduct，未修改 USD。
