# Docker 使用说明

项目代码、场景和数据保留在宿主目录，通过 `/workspace` 挂载到容器；镜像只提供固定的软件环境。

## 当前基线

| 项目 | 值 |
| --- | --- |
| Isaac Sim | 5.1.0，基础镜像 digest 已固定 |
| ROS 2 | Jazzy |
| Gazebo | Harmonic |
| 正式镜像 | `issac_ocs_docker:5.1.0-repro` |
| 当前镜像 ID | `sha256:7922d3590bde19a51e09c7ccfa0343aefbfce60d4e40a986f4140d9aea3d94d7` |
| 工作区 | `/workspace/projects/ros2_ws` |
| Docker overlay | `build_docker`、`install_docker`、`log_docker` |

本地官方参考：

- `/home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_container.md`
- `/home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.md`
- `/home/gtk/ai_docs/docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.md`

## 构建镜像

```bash
cd /home/gtk/isaac_ocs_project
docker build --no-cache \
  -t issac_ocs_docker:5.1.0-repro \
  -f docker/Dockerfile .
docker tag issac_ocs_docker:5.1.0-repro issac_ocs_docker:latest
```

Dockerfile 包含 ROS 2 Jazzy、Gazebo Harmonic、MoveIt、Grid Map、ros2_control、Pinocchio/Coal 和 ONNX Runtime C++ SDK 1.19.0。

## 完整编译 ROS 2 工作区

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

工作区共 119 个包；脚本构建 116 个。三个需要单独 Raisim SDK/许可证的包会被明确跳过：

- `ocs2_raisim_core`
- `ocs2_legged_robot_raisim`
- `ocs2_legged_robot_mpcnet`

当前验证结果为 116 个完成，0 failed，0 aborted。

## 启动

```bash
cd /home/gtk/isaac_ocs_project
export ROS_DOMAIN_ID=73
bash docker/run_isaac_ocs_docker.sh --replace
docker exec -it demo_vla bash
```

容器内常用命令：

```bash
ocs_build_all
docker_env_check.sh
ocs_launch
sim1_check
sim1_reset
sim1_teach
sim1_demo
sim1_clean
sim1_convert
```

启动器使用全部 GPU、host network/IPC/PID、`--privileged` 和 X11，只能在可信宿主环境运行。

宿主本地 overlay 与 Docker overlay 不可混用：

- 本地：`projects/ros2_ws/{build,install,log}`
- Docker：`projects/ros2_ws/{build_docker,install_docker,log_docker}`

## 当前验证范围

已验证无缓存镜像重建、官方 compatibility checker、116 包完整编译、
Docker GUI + Stage + Play、OCS2 launch、机械臂运动、左右夹爪、差速底盘实际
位姿变化、六路相机实时发布与录制，以及现有数据工具 CLI。

当前仓库没有自动随机化、自动成功判定、批量扩增或模型训练入口；这些属于后续
研发范围。

详见 [RUNTIME_VALIDATION.md](RUNTIME_VALIDATION.md)。
