> **Historical record / 历史记录**：本页描述其记录日期的状态；当前运行以[中文指南](../getting-started.zh-CN.md) / [English guide](../getting-started.md)为准。Historical failures and paths are not current release claims.

# Isaac OCS 项目离职交接文档

> 历史记录（2026-07-30）：本稿的 Humble、`/home/gtk` 和 Docker 验收命令属于当时机器。当前本机启动请使用 [启动指南](startup-guide-alias.zh-CN.md) 与 [2026-09-15 验证报告](../../reports/LOCAL_VALIDATION_20260915.md)。

> 更新日期：2026-07-30
> 项目目录：`${PROJECT_ROOT}`
> 当前基线：本地 Isaac Sim 5.1.0 + ROS 2 Humble；Docker Isaac Sim 5.1.0 + ROS 2 Jazzy + Gazebo Harmonic

## 1. 当前结论

项目用于在 Isaac Sim 中控制 R1 轮式双臂机器人，覆盖双臂 OCS2 MPC、夹爪、差速底盘、三组 RGB-D 相机、人工示教、轨迹回放与数据转换。

项目原验收已经通过。2026-07-30 当前副本又完成了本地相机会话修复和 Docker
全链路复核：

| 检查项 | 当前结果 |
| --- | --- |
| 本地六路相机 RenderProduct 重建 | 通过，六个 RenderProduct 创建、绑定回读均正常 |
| 本地六路 ROS 图像消息 | 通过，六个话题均收到实际图像消息和正确 `frame_id` |
| 关闭并重新打开本地场景 | 通过，默认启动会自动重建会话级 RenderProduct |
| Docker 镜像 | `issac_ocs_docker:5.1.0-repro` 与 `issac_ocs_docker:latest` 已固定到同一验收镜像 |
| Docker 镜像 ID | `sha256:7922d3590bde19a51e09c7ccfa0343aefbfce60d4e40a986f4140d9aea3d94d7` |
| Docker ROS 工作区编译 | 116 个支持包全部完成，0 failed、0 aborted |
| Docker 安装包集合 | 116/116，缺失 0、多余 0 |
| Docker GPU/基础运行环境 | 官方 compatibility checker `PASSED`；RTX 5090 D v2、ROS 2 Jazzy、Harmonic 与 Isaac Python 依赖通过 |
| Docker GUI + Stage + Play | 通过，Isaac Sim 5.1.0-rc.19 打开 `assets/scenes/scene.usd` 并进入 Play |
| Docker OCS2 | `ocs2_arm_controller`、`joint_state_broadcaster` 均为 active |
| Docker 机械臂 | 通过，左臂 `+3 mm X` 指令产生约 `+2.06 mm` 实际末端位移 |
| Docker 夹爪 | 通过，左右 PGIA 关节开合反馈均发生预期变化 |
| Docker 底盘 | 通过，`base_link.x` 实际前进约 `143 mm` |
| Docker 六路相机录制 | 通过，六路均有 640×480 实际帧；3 个 MP4 和 21 个 Depth NPY 可读 |

验收完成后已停止并删除临时容器。当前没有运行 Isaac Sim，也没有打开 Stage。
当前只保留两个正式镜像标签和本次成功验收证据。

## 2. 项目目标与完成度

目标链路：

```text
人工/RViz 目标
  -> OCS2 MPC
  -> ros2_control/topic bridge
  -> Isaac Sim 双臂、夹爪和底盘
  -> RGB-D、关节与诊断数据
  -> 轨迹清洗/回放/LeRobot 转换
  -> 后续模仿学习或强化学习
```

| 模块 | 当前状态 | 备注 |
| --- | --- | --- |
| Isaac Sim 场景和 Action Graph | 已具备，原验收通过 | 唯一入口为 `assets/scenes/scene.usd` |
| 双臂 OCS2 控制 | 已具备，本地原验收与 Docker 当前实测通过 | 运行入口为 `ocs2_isaac.launch.py` |
| 左右夹爪 | 已具备，本地原验收与 Docker 当前实测通过 | 使用 `Float64MultiArray` 命令 |
| 差速底盘 | 已具备，Docker 当前实际位姿验收通过 | SIM1 提供录制与固定路径脚本 |
| 三路 RGB + 三路 Depth | 当前本地与 Docker 实测通过 | RenderProduct 是会话级对象，见第 6 节 |
| 示教、回放、轨迹清洗 | 已具备 | 当前保留一组 2026-07-15 原始/清洗轨迹 |
| LeRobot 转换 | 脚本已具备 | 本地当前没有可交付的完整视觉 LeRobot 数据集 |
| 自动随机化、自动判定、批量扩增 | 尚未完成 | 后续研发目标 |
| BC/ACT/Diffusion/VLA/RL 训练 | 尚未形成当前交付物 | 需先获得有效视觉数据集 |
| Sim2Real | 尚未完成 | 需真机标定和安全验收 |

## 3. 运行环境与依赖

### 3.1 本地

| 项目 | 当前值 |
| --- | --- |
| Isaac Sim | `/home/gtk/isaac-sim-5.1`，5.1.0-rc.19 |
| ROS 2 | Humble，`/opt/ros/humble` |
| ROS 工作区 | `projects/ros2_ws` |
| 本地 overlay | `projects/ros2_ws/{build,install,log}` |
| GPU | NVIDIA GeForce RTX 5090 D v2 |
| 驱动 | 580.126.09 |
| 默认 ROS Domain | 项目运行建议使用 `ROS_DOMAIN_ID=73` |

`/home/gtk/isaacsim-6.0.1` 是另一套 Isaac 安装，不属于本项目 5.1 运行基线。启动 Isaac 前不要 source ROS 环境；统一入口会主动清理会污染 Isaac Python 的 ROS/Python 环境变量。

### 3.2 Docker

| 项目 | 当前值 |
| --- | --- |
| 基础镜像 | `nvcr.io/nvidia/isaac-sim:5.1.0@sha256:93b0f99635ab126fb5b33298d513c11520f119f0ee60ff8414ccef67ea977829` |
| 正式标签 | `issac_ocs_docker:5.1.0-repro`、`issac_ocs_docker:latest` |
| 镜像 ID | `sha256:7922d3590bde19a51e09c7ccfa0343aefbfce60d4e40a986f4140d9aea3d94d7` |
| ROS 2 | Jazzy |
| Gazebo | Harmonic，`GZ_VERSION=harmonic` |
| RMW | `rmw_fastrtps_cpp` |
| Isaac Python | 3.11.13 |
| ONNX Runtime C++ SDK | 1.19.0，安装在 `/opt/onnxruntime` |
| 本次验收 overlay | `projects/ros2_ws/{build_docker_acceptance_20260730,install_docker_acceptance_20260730,log_docker_acceptance_20260730}` |

依赖来源：

| 依赖 | 文件/目录 |
| --- | --- |
| apt/ROS 依赖 | `docker/Dockerfile`、`dependencies/docker-apt-packages.lock` |
| Python 锁定 | `dependencies/docker-python-requirements.lock` |
| 离线 Python wheel | `dependencies/docker-wheelhouse/` |
| ONNX Runtime C++ 固定包 | `dependencies/docker_source_cache/onnxruntime-linux-x64-1.19.0.tgz` |
| 外部仓库版本 | `dependencies/repositories.lock.yaml` |
| 本地 Isaac Python 补充包 | `vendor/isaac_sim_5_1_python/` |

ROS 工作区共发现 119 个包。Docker 完整支持构建为 116 个；以下三个 Raisim 包需要另行取得 SDK 和许可证，当前明确排除：

- `ocs2_raisim_core`
- `ocs2_legged_robot_raisim`
- `ocs2_legged_robot_mpcnet`

## 4. 目录地图

| 路径 | 作用 |
| --- | --- |
| `assets/scenes/scene.usd` | 唯一场景启动入口 |
| `assets/scenes/r1_workcell/` | 场景引用层和模型资产，不可随意移动 |
| `projects/ros2_ws/src/` | ROS 2、OCS2、控制器和机器人资源源码 |
| `projects/teleoperation/sim1/` | 示教、夹爪、底盘、相机、回放和数据工具 |
| `scripts/` | 项目统一入口、相机修复、Docker 构建和 USD 工具 |
| `docker/` | Dockerfile、启动器、Compose、别名和当前验证记录 |
| `dependencies/` | 锁定依赖、wheelhouse、ONNX Runtime 固定包 |
| `data/ros2_log/` | 运行日志和诊断材料 |
| `outputs/` | 当前工具输出；相机重建结果在其子目录 |
| `backups/camera_session_fix_20260730/` | 相机修复前后备份和校验清单 |
| `reproducibility/manifests/` | 当前镜像及资产校验信息 |

根目录 `.git/` 当前不是可用的完整 Git 仓库，不能把它当成可靠回滚手段；以备份目录和 SHA-256 清单为准。

## 5. 核心架构

### 5.1 双臂

```text
/left_target/stamped、/right_target/stamped
  -> ocs2_arm_controller
  -> topic_based_ros2_control
  -> /arm_joint_cmd
  -> Isaac /World/ActionGraphs/Arm_Control_Graph

Isaac /World/ActionGraphs/State_Telemetry_Graph
  -> /isaac_joint_states
  -> joint_state_fill.py
  -> /joint_states
```

核心包：

| 包 | 用途 |
| --- | --- |
| `r1_description` | R1 Xacro/URDF、launch、TF |
| `ocs2_arm_controller` | 左右 7 轴 OCS2 MPC |
| `topic_based_ros2_control` | ROS topic 与 ros2_control 桥 |
| `arms_target_manager` | RViz 交互目标 |
| `r1_lerobot_sim` | 离线回放 |
| `ocs2_*` | OCS2 核心、Pinocchio、求解器和示例 |

### 5.2 夹爪

| 话题 | 类型 | 值 |
| --- | --- | --- |
| `/left_gripper_controller/commands` | `std_msgs/msg/Float64MultiArray` | `[1.0]` 打开，`[0.0]` 闭合/保持 |
| `/right_gripper_controller/commands` | `std_msgs/msg/Float64MultiArray` | `[1.0]` 打开，`[0.0]` 闭合/保持 |

场景中由 `/World/ActionGraphs/Gripper_Control_Graph` 接收。

### 5.3 底盘

| 话题 | 用途 |
| --- | --- |
| `/sim1/base_diff_cmd` | 差速底盘命令 |
| `/isaac_joint_states` | Isaac 原始关节状态 |
| `/joint_states` | 补齐后的标准关节状态 |

SIM1 的 `base-record`、`base-path-demo` 提供命令录制和固定路径回放。

当前场景没有 `/odom` publisher。`base_drive_path_runner.py` 的 `--use-odom` 和
`--use-tf` 是预留参数；当前底盘实际位姿可通过 Isaac Simulation Control 的
`/get_entity_state` 查询 `/World/Robot/base_link`。

## 6. 六路相机当前方案

相机发布节点位于 `/World/ActionGraphs/Camera_Publish_Graph`。六路输出：

| 名称 | ROS 图像话题 | 当前 `frame_id` |
| --- | --- | --- |
| Head RGB | `/head_cam/color/image_raw` | `head_camera_color` |
| Head Depth | `/head_cam/depth/image_rect_raw` | `head_camera_depth` |
| Left RGB | `/left_cam/color/image_raw` | `left_camera_color` |
| Left Depth | `/left_cam/depth/image_rect_raw` | `left_camera_depth` |
| Right RGB | `/right_cam/color/image_raw` | `right_camera_color` |
| Right Depth | `/right_cam/depth/image_rect_raw` | `right_camera_depth` |

关键规则：

- 命名 RenderProduct 在当前 Isaac/Replicator 行为下创建于 Session Layer。
- 根 USD 不保存六个会话级 `renderProductPath`。
- 每次重新打开场景后，由 `scripts/rebuild_camera_render_products.py` 重建。
- 脚本只处理 `head/left/right × rgb/depth`；不处理 IMU，不删除 `Replicator` 或 `Replicator_*`。
- 脚本先禁用并清空六个 CameraHelper，等待 Hydra 释放资源，再创建 RenderProduct。
- CameraHelper 必须绑定 `rep.create.render_product()` 实际返回的路径，不能假定名称没有 `_01` 等后缀。
- 六个句柄保存在 `builtins._ocs_six_camera_render_products`，直到当前 Isaac 会话结束。

默认启动：

```bash
cd ${PROJECT_ROOT}
export ROS_DOMAIN_ID=73
./scripts/project_control.sh start-isaac
```

无额外参数时，统一入口执行相机重建脚本；脚本自行打开 `assets/scenes/scene.usd`。启动后手动点击 Play。测试时若需要自动 Play：

```bash
OCS_CAMERA_REBUILD_AUTOPLAY=1 \
  ./scripts/project_control.sh start-isaac
```

最后一次成功报告：

```text
outputs/camera_session_rebuild/latest.json
status=ok
created=6
failures={}
```

### 根场景的 RenderProduct 状态

当前并不是“退回某个编号的固定 render path”。正确状态是：

- 根 USD 中六个错误/陈旧 RenderProduct 规格已移除；
- 六个 Helper 的有效路径由每次会话实际返回值重新绑定；
- 当前根场景 SHA-256 为 `642541c807f6a587d10a67884413ebfe242ea551825336ce4fdc650659af804f`；
- 修复前备份 SHA-256 为 `e3778c83fee7352fb72d16c1ea7837378c681e16daa0574cf0f3815272b71bd7`。

不要手工把 `/Render/OmniverseKit/HydraTextures/...` 会话路径重新写回根 USD。

## 7. 主要 ROS 2 接口

| 接口 | 类型/用途 |
| --- | --- |
| `/left_target/stamped` | 左臂末端目标 |
| `/right_target/stamped` | 右臂末端目标 |
| `/left_current_pose` | 左臂当前末端位姿 |
| `/right_current_pose` | 右臂当前末端位姿 |
| `/arm_joint_cmd` | 双臂关节位置命令 |
| `/isaac_joint_states` | Isaac 原始 JointState |
| `/joint_states` | ROS 标准 JointState |
| `/left_gripper_controller/commands` | 左夹爪命令 |
| `/right_gripper_controller/commands` | 右夹爪命令 |
| `/sim1/base_diff_cmd` | 底盘差速命令 |
| `/fsm_command` | HOME/OCS2 状态命令；脚本使用 1 和 3 |
| `/clock` | 仿真时钟 |
| `/reset_simulation` | `simulation_interfaces/srv/ResetSimulation` |
| `/set_simulation_state` | `simulation_interfaces/srv/SetSimulationState` |

检查：

```bash
source /opt/ros/humble/setup.bash
source projects/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=73
./scripts/project_control.sh sim1-check
```

## 8. 脚本入口

### 8.1 宿主统一入口

```bash
./scripts/project_control.sh preflight
./scripts/project_control.sh build
./scripts/project_control.sh build --all
./scripts/project_control.sh start-isaac
./scripts/project_control.sh launch-ocs2
./scripts/project_control.sh sim1-check
./scripts/project_control.sh sim1-reset
./scripts/project_control.sh sim1-teach
./scripts/project_control.sh sim1-demo
```

默认 `build` 只构建主链路依赖闭包；`build --all` 是本地全工作区构建，不等同于 Docker 构建脚本。

### 8.2 SIM1

```bash
projects/teleoperation/sim1/game_control.sh check
projects/teleoperation/sim1/game_control.sh reset
projects/teleoperation/sim1/game_control.sh teach
projects/teleoperation/sim1/game_control.sh teach-with-cameras
projects/teleoperation/sim1/game_control.sh demo
projects/teleoperation/sim1/game_control.sh camera-grid
projects/teleoperation/sim1/game_control.sh base-record
projects/teleoperation/sim1/game_control.sh base-path-demo
projects/teleoperation/sim1/game_control.sh direct-demo
```

重要文件：

| 文件 | 用途 |
| --- | --- |
| `keyboard_ocs2_gripper_teleop.py` | 双臂/夹爪示教与轨迹记录 |
| `replay_ocs2_target_trace.py` | OCS2 目标轨迹回放 |
| `replay_lerobot_episode.py` | 直接关节回放；不要与 OCS2 同时发布 |
| `tools/camera_grid_viewer.py` | 2×3 六相机实时查看 |
| `tools/camera_recorder.py` | 六相机录制 |
| `base_drive_recorder.py` | 底盘命令/状态录制 |
| `base_drive_path_runner.py` | 底盘固定路径 |
| `trace_cleaning/scripts/` | 清洗和 LeRobot 转换 |

### 8.3 Docker

构建镜像：

```bash
cd ${PROJECT_ROOT}
docker build --no-cache \
  -t issac_ocs_docker:5.1.0-repro \
  -f docker/Dockerfile .
```

完整构建 116 个支持包：

```bash
docker run --rm --user "$(id -u):$(id -g)" \
  -e ROS_DISTRO_TARGET=jazzy \
  -e GZ_VERSION=harmonic \
  -e ROS_DOMAIN_ID=73 \
  -e ROS2_BUILD_BASE=/workspace/projects/ros2_ws/build_docker_acceptance_20260730 \
  -e ROS2_INSTALL_BASE=/workspace/projects/ros2_ws/install_docker_acceptance_20260730 \
  -e ROS2_LOG_BASE=/workspace/projects/ros2_ws/log_docker_acceptance_20260730 \
  -e COLCON_HOME=/tmp/isaac_ocs_colcon_home \
  -v ${PROJECT_ROOT}:/workspace \
  -w /workspace \
  issac_ocs_docker:5.1.0-repro \
  /bin/bash /workspace/scripts/build_ros_workspace_docker.sh
```

也可以先启动长期容器，再使用别名：

```bash
bash docker/run_isaac_ocs_docker.sh --replace
docker exec -it demo_vla bash
ocs_build_all
```

Docker 启动器使用 host network/IPC/PID、`--privileged`、X11 和全部 GPU，只应在可信宿主使用。

## 9. 当前源码兼容点

| 文件 | 当前作用 |
| --- | --- |
| `gz_ros2_control_plugin.cpp` | 对 Humble 与 Jazzy `hardware_interface::ResourceManager` API 做版本条件兼容 |
| `PinocchioGeometryInterface.h` | 根据可用头文件选择 `coal` 或 `hpp::fcl` |
| `SphereApproximation.h/.cpp` | 同一套 `coal`/`hpp::fcl` 条件兼容 |
| `cgal5_colcon/CMakeLists.txt` | 固定 CGAL 5.3 源码包 SHA-256 |
| `docker/Dockerfile` | 固定 Isaac 基础镜像与 ONNX Runtime C++ SDK |
| `docker/run_isaac_ocs_docker.sh` | 为普通用户提供可写 HOME，并加入 Isaac 安装目录所需 GID |
| `scripts/build_ros_workspace_docker.sh` | 支持通过环境变量指定隔离的 build/install/log 目录 |
| `r1_description/launch/ocs2_isaac.launch.py` | 主 `robot_state_publisher` 发布含 `<ros2_control>` 的控制 URDF；规划 URDF 单独发布到 `/ocs2_robot_description` |

生成或修改 Isaac/ROS Python、C++ 头文件时，继续遵守 `AGENTS.md`：先检索 `/home/gtk/ai_docs/` 的本地官方文档，并在新 Python 脚本或 C++ 头文件顶部列出具体参考路径。

## 10. 数据状态

当前保留的主轨迹：

```text
projects/teleoperation/sim1/trace_data/raw/
  manual_ocs2_keyboard_trace_20260715_133454.csv
  manual_ocs2_keyboard_trace_20260715_133454.diagnostics.csv
  manual_ocs2_keyboard_trace_20260715_133454.metadata.json
  manual_ocs2_keyboard_trace_20260715_133454.label.md

projects/teleoperation/sim1/trace_data/cleaned/
  manual_ocs2_keyboard_trace_20260715_133454_cleaned.csv
  manual_ocs2_keyboard_trace_20260715_133454_cleaned.report.md
```

这组轨迹的运动标签可用于回放参考，但相机帧不足，不能作为完整视觉训练样本。

Docker 当前六路相机验收数据：

```text
outputs/docker_acceptance_20260730/six_camera_recording/
  summary.json
  camera_timestamps.csv
  videos/head_rgb.mp4
  videos/left_rgb.mp4
  videos/right_rgb.mp4
  depth/head_depth/*.npy
  depth/left_depth/*.npy
  depth/right_depth/*.npy
```

该目录证明六路流可以同时录制，但它只有约 5 秒，不是已标注的操作任务数据集，
不能替代正式示教采集。

## 11. 标准启动顺序

终端 1：

```bash
cd ${PROJECT_ROOT}
export ROS_DOMAIN_ID=73
./scripts/project_control.sh preflight
./scripts/project_control.sh start-isaac
```

等待脚本输出六个 `[OK]`，然后点击 Play。

终端 2：

```bash
cd ${PROJECT_ROOT}
export ROS_DOMAIN_ID=73
source /opt/ros/humble/setup.bash
source projects/ros2_ws/install/setup.bash
./scripts/project_control.sh launch-ocs2
```

终端 3：

```bash
cd ${PROJECT_ROOT}
export ROS_DOMAIN_ID=73
source /opt/ros/humble/setup.bash
source projects/ros2_ws/install/setup.bash
./scripts/project_control.sh sim1-check
```

六路相机检查：

```bash
ros2 topic list | grep -E 'head_cam|left_cam|right_cam'
ros2 topic echo --once /head_cam/color/image_raw
ros2 topic echo --once /head_cam/depth/image_rect_raw
ros2 topic echo --once /left_cam/color/image_raw
ros2 topic echo --once /left_cam/depth/image_rect_raw
ros2 topic echo --once /right_cam/color/image_raw
ros2 topic echo --once /right_cam/depth/image_rect_raw
```

## 12. 交接注意事项与后续优先级

1. 不要删除 `Replicator`、`Replicator_*`，也不要把会话 RenderProduct 路径保存进根 USD。
2. 不要混用本地 `install/` 与 Docker `install_docker/`。
3. 不要把 ROS 2 Rolling 当作当前运行时；它只是本地 API 文档版本。
4. 不要重新引入三个 Raisim 包，除非已经取得匹配 SDK 和许可证。
5. 修改场景前先复制 `assets/scenes/scene.usd` 并记录 SHA-256。
6. 首要后续工作是采集带任务标签的六路 RGB-D 成功示教，再建设自动随机化、自动判定、批量扩增和训练数据闭环。
7. 当前仓库没有自动随机化、自动成功判定、批量训练或 BC/ACT/Diffusion/VLA/RL 训练入口，不得在交付说明中写成已完成。

## 13. 当前验证证据和备份

| 内容 | 路径 |
| --- | --- |
| 相机重建成功报告 | `outputs/camera_session_rebuild/latest.json` |
| Docker 全量成功构建日志 | `projects/ros2_ws/log_docker_acceptance_20260730/build_2026-07-30_09-08-04/` |
| Docker OCS2 运行记录 | `outputs/docker_acceptance_20260730/ocs2_motion_trace.csv` |
| Docker 六路相机录制摘要 | `outputs/docker_acceptance_20260730/six_camera_recording/summary.json` |
| Docker 当前验证说明 | `docker/RUNTIME_VALIDATION.md` |
| Docker 当前源码/文档备份 | `backups/docker_acceptance_20260730/final/` |
| Docker 备份校验清单 | `backups/docker_acceptance_20260730/final/MANIFEST.sha256` |
| 修复前场景/启动器备份 | `backups/camera_session_fix_20260730/pre_fix/` |
| 修复后完整备份 | `backups/camera_session_fix_20260730/final/` |
| 修复后校验清单 | `backups/camera_session_fix_20260730/final/MANIFEST.sha256` |
| 当前 Docker 镜像清单 | `reproducibility/manifests/docker_image.sha256` |

优先阅读：

- `docs/archive/project-notes-2026-07-30.zh-CN.md`
- `docs/archive/startup-guide-alias.zh-CN.md`
- `projects/teleoperation/sim1/PROJECT_OVERVIEW.md`
- `projects/teleoperation/sim1/GAME_GUIDE.md`
- `projects/teleoperation/sim1/TRACE_DATA_FORMAT.md`
- `docker/README_DOCKER.md`
