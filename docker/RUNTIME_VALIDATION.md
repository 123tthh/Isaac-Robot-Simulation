# Docker 当前验收记录

> 验收日期：2026-07-30  
> 项目：`/home/gtk/isaac_ocs_project`  
> 结论：当前 Docker 版本通过已实现功能的构建与运行态验收。

本文只记录当前有效状态，不记录已清理的中间结果。

## 1. 正式镜像

| 项目 | 当前值 |
| --- | --- |
| 正式标签 | `issac_ocs_docker:5.1.0-repro`、`issac_ocs_docker:latest` |
| 镜像 ID | `sha256:7922d3590bde19a51e09c7ccfa0343aefbfce60d4e40a986f4140d9aea3d94d7` |
| 创建时间 | `2026-07-30T09:51:58.972362779Z` |
| 镜像大小 | `19,988,441,031` bytes |
| 基础镜像 | Isaac Sim 5.1.0，Dockerfile 中固定 digest |
| Isaac Sim | 5.1.0-rc.19 |
| ROS 2 | Jazzy |
| Gazebo | Harmonic |
| RMW | `rmw_fastrtps_cpp` |
| 默认 ROS Domain | `73` |

两个正式标签指向同一个镜像 ID。当前没有临时验收标签或验收容器。

## 2. 镜像与 GPU 验收

最终镜像由 `docker/Dockerfile` 使用 `--no-cache` 重建。官方 Isaac compatibility checker 结果：

```text
GPU: NVIDIA GeForce RTX 5090 D v2
Driver: 580.126.09
VRAM: approximately 25.64 GB
Operating system: Ubuntu 24.04
System checking result: PASSED
```

Isaac Python 关键模块导入结果：

```text
torch=2.7.0+cu128
vuer=0.0.26
onnxruntime=1.28.0
IMPORTS_PASS
```

Dockerfile 在全局 Python 依赖安装后恢复 Isaac 自带
`packaging/_structures.py`，并在构建阶段检查
`onnxruntime`、`packaging`、`pytest`、`torch` 和 `vuer`。

## 3. ROS 2 全工作区编译

源码共发现 119 个 ROS 包。以下 3 个包需要另行取得 Raisim SDK 与许可证，不属于当前支持集合：

- `ocs2_raisim_core`
- `ocs2_legged_robot_raisim`
- `ocs2_legged_robot_mpcnet`

其余 116 个包在隔离目录中完成全量构建：

| 项目 | 当前值 |
| --- | --- |
| build | `projects/ros2_ws/build_docker_acceptance_20260730` |
| install | `projects/ros2_ws/install_docker_acceptance_20260730` |
| log | `projects/ros2_ws/log_docker_acceptance_20260730` |
| 结果 | `116 packages finished` |
| failed | `0` |
| aborted | `0` |
| not processed | `0` |
| 用时 | `16min 22s` |

`ocs2_isaac.launch.py` 更新后又单独重建了 `r1_description`，结果为
`1 package finished`。Python 语法检查通过。

复现完整构建：

```bash
docker run --rm --user "$(id -u):$(id -g)" \
  -e ROS_DISTRO_TARGET=jazzy \
  -e GZ_VERSION=harmonic \
  -e ROS_DOMAIN_ID=73 \
  -e ROS2_BUILD_BASE=/workspace/projects/ros2_ws/build_docker_acceptance_20260730 \
  -e ROS2_INSTALL_BASE=/workspace/projects/ros2_ws/install_docker_acceptance_20260730 \
  -e ROS2_LOG_BASE=/workspace/projects/ros2_ws/log_docker_acceptance_20260730 \
  -e COLCON_HOME=/tmp/isaac_ocs_colcon_home \
  -v /home/gtk/isaac_ocs_project:/workspace \
  -w /workspace \
  issac_ocs_docker:5.1.0-repro \
  /bin/bash /workspace/scripts/docker_build_ros2_ws_20260730.sh
```

## 4. Docker GUI、Stage 与 Play

Docker GUI 使用 X11、NVIDIA 直接渲染和 Isaac Sim 5.1.0 启动成功。验收时打开：

```text
/workspace/assets/scenes/scene.usd
```

Timeline 进入 Play，ROS 2 Bridge 和 Simulation Control 服务可用。验收完成后 GUI、ROS 进程和临时容器均已停止并删除；当前没有运行中的 Docker Stage。

启动器 `docker/run_isaac_ocs_docker.sh` 当前处理两项容器权限：

- 默认容器 HOME 为 `/tmp/isaac-ocs-home-<uid>`，普通用户可写；
- 通过 `--group-add "${ISAAC_SIM_GID:-1234}"` 允许宿主 uid 1000 执行官方 `/isaac-sim`。

## 5. 六路相机

启动时执行 `scripts/rebuild_six_camera_render_products_20260730.py`。六个
RenderProduct 均在当前 Session Layer 创建，并将函数实际返回路径绑定到
CameraHelper：

```text
/Render/OmniverseKit/HydraTextures/head_rgb
/Render/OmniverseKit/HydraTextures/head_depth
/Render/OmniverseKit/HydraTextures/left_rgb
/Render/OmniverseKit/HydraTextures/left_depth
/Render/OmniverseKit/HydraTextures/right_rgb
/Render/OmniverseKit/HydraTextures/right_depth
```

六个当前图像话题：

```text
/head_cam/color/image_raw
/head_cam/depth/image_rect_raw
/left_cam/color/image_raw
/left_cam/depth/image_rect_raw
/right_cam/color/image_raw
/right_cam/depth/image_rect_raw
```

5 秒同步录制结果：

| 流 | 帧数 | 分辨率 | 编码 |
| --- | ---: | --- | --- |
| Head RGB | 73 | 640×480 | `rgb8` |
| Head Depth | 64 | 640×480 | `32FC1` |
| Left RGB | 59 | 640×480 | `rgb8` |
| Left Depth | 72 | 640×480 | `32FC1` |
| Right RGB | 58 | 640×480 | `rgb8` |
| Right Depth | 59 | 640×480 | `32FC1` |

3 个 RGB MP4 均可用 OpenCV 解码，帧数与摘要一致；21 个 Depth NPY 均已落盘，
抽样数组为 `(480, 640) float32` 且 307,200 个值均为有限值。

证据：

```text
outputs/camera_session_rebuild/latest.json
outputs/docker_acceptance_20260730/six_camera_recording/summary.json
outputs/docker_acceptance_20260730/six_camera_recording/camera_timestamps.csv
outputs/docker_acceptance_20260730/six_camera_recording/videos/
outputs/docker_acceptance_20260730/six_camera_recording/depth/
```

## 6. OCS2、机械臂、夹爪与底盘

Docker 中启动：

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/projects/ros2_ws/install_docker_acceptance_20260730/setup.bash
export ROS_DOMAIN_ID=73
ros2 launch r1_description ocs2_isaac.launch.py \
  enable_rviz:=false enable_target_manager:=false
```

当前控制器状态：

```text
ocs2_arm_controller     active
joint_state_broadcaster active
```

运行态结果：

| 功能 | 当前验收结果 |
| --- | --- |
| OCS2 双臂 | 左臂发送 `+3 mm X` 目标后，实际末端 x 增加约 `2.06 mm`；反向目标可回退 |
| 夹爪闭合 | 四个 PGIA 关节反馈约为 `0.0405, 0.0445, 0.0405, 0.0445 rad` |
| 夹爪打开 | 四个 PGIA 关节反馈约为 `0.1167, 0.1200, 0.1166, 0.1200 rad` |
| 差速底盘 | `0.5 m/s × 2 s` 指令后，`/World/Robot/base_link.x` 从 `-2.90387` 变为 `-2.76061 m`，实际前进约 `143 mm` |

机械臂运行记录：

```text
outputs/docker_acceptance_20260730/ocs2_motion_trace.csv
outputs/docker_acceptance_20260730/ocs2_motion_trace.diagnostics.csv
outputs/docker_acceptance_20260730/ocs2_motion_trace.metadata.json
```

当前场景不发布 `/odom`；`base_drive_path_runner.py` 的 `--use-odom` 和 `--use-tf`
仍是预留参数。底盘验收使用 Simulation Control 的 `/get_entity_state` 读取
`/World/Robot/base_link` 实际刚体位姿。

## 7. 数据工具与未实现范围

以下 Docker CLI 入口完成启动冒烟测试：

- `trace_cleaning/scripts/process_sim1_episode.py --help`
- `replay_ocs2_target_trace.py --help`
- `tools/camera_recorder.py --help`

当前项目没有自动域随机化、自动成功判定、批量扩增或模型训练入口；也没有
BC/ACT/Diffusion/VLA/RL 模型交付物。这些是后续研发目标，不计入本次已实现
Docker 控制链的通过项，交接时不得写成已完成。

## 8. 场景保护与当前结论

验收前后根场景均为：

```text
642541c807f6a587d10a67884413ebfe242ea551825336ce4fdc650659af804f  assets/scenes/scene.usd
```

本次 Docker 验收没有修改本地 USD。相机 RenderProduct 修复只作用于当前
Session Layer，不把会话路径写回根场景，也不删除 `Replicator` 或
`Replicator_*`。

当前结论：

```text
Docker 镜像重建：通过
官方兼容性检查：通过
ROS 2 支持包全量编译：通过
Docker GUI + Stage + Play：通过
OCS2 launch 与控制器：通过
机械臂运动：通过
夹爪开合：通过
底盘实际运动：通过
六路相机实时发布与录制：通过
现有数据工具 CLI：通过
```
