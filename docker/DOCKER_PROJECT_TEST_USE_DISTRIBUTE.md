# Docker 版本工程测试、使用、分发指南

本文说明如何测试、日常使用和分发当前 Docker 版本工程。目标是让接收方在不改
ROS2/OCS2 业务代码、不复制大体积 Isaac Sim 安装目录进镜像的前提下，复现
Isaac Sim + ROS2 + OCS2 + RViz2 + SIM1 主链路。

## 目录结构约定

宿主机必须准备 `/workspace`，工程和数据通过 bind mount 给容器使用：

```text
/workspace
  /projects/ros2_ws
  /projects/teleoperation
  /data/ros2_log
  /assets/datasets/sac-m
  /outputs
```

主 USD：

```text
/workspace/assets/datasets/sac-m/61.usd
```

注意：不能只分发或复制 `61.usd`、`522.usd`。必须同步完整 `sac-m` 目录，
包括：

```text
configuration/physics.usd
configuration/base.usd
mesh/material/texture 等依赖资产
```

目录不完整时，Isaac Sim 可能只创建 ROS2 publisher endpoint，但 `/clock` 和
`/isaac_joint_states` 没有真实消息。

## 运行前宿主机要求

宿主机需要具备：

```text
NVIDIA Driver
Docker
NVIDIA Container Toolkit
Isaac Sim 5.1
X11 图形桌面
```

允许容器访问 X11：

```bash
xhost +local:docker
# 如果当前用户不在 docker 组：
xhost +local:root
```

如果 Docker 需要 sudo，本文命令前加 `sudo` 即可。

## 准备 workspace

```bash
sudo mkdir -p /workspace/projects /workspace/data /workspace/assets/datasets/sac-m /workspace/outputs

sudo rsync -a /home/gtk/ros2_ws /workspace/projects/
sudo rsync -a /home/gtk/teleoperation /workspace/projects/
sudo rsync -a /home/gtk/ros2_log /workspace/data/

sudo rsync -a --info=progress2 \
  /home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/ \
  /workspace/assets/datasets/sac-m/
```

检查关键文件：

```bash
test -f /workspace/assets/datasets/sac-m/61.usd
test -f /workspace/assets/datasets/sac-m/522.usd
test -f /workspace/assets/datasets/sac-m/configuration/physics.usd
```

## 构建镜像

默认镜像名：

```text
issac_ocs_docker
```

工业交付以源码 + Dockerfile 为准。不要把 `docker commit demo_vla
issac_ocs_docker:latest` 生成的本机热修复镜像作为最终交付来源。

构建：

```bash
sudo docker build -t issac_ocs_docker -f docker/Dockerfile .
```

如果基础镜像名称不同：

```bash
sudo docker build \
  --build-arg BASE_IMAGE=<your_isaaclab_ros2_ocs2_base_image> \
  -t issac_ocs_docker \
  -f docker/Dockerfile .
```

推荐带版本 tag 构建可追溯镜像：

```bash
sudo docker build \
  -t issac_ocs_docker:2026-06-08-rviz-ocs2-sim1 \
  -f docker/Dockerfile .

sudo docker tag \
  issac_ocs_docker:2026-06-08-rviz-ocs2-sim1 \
  issac_ocs_docker:latest
```

不要在 Dockerfile 中 COPY Isaac Sim 安装目录、USD 数据集、`trace_data`、
`ros2_log`。这些内容应通过 `/workspace` 挂载。

## 启动容器

推荐：

```bash
bash docker/run_issac_ocs_docker.sh --replace
```

脚本会：

```text
同步完整 sac-m 资产目录
挂载 /workspace
启用 GPU
启用 X11
使用 host network
使用 host IPC
使用宿主机 UID/GID 启动容器
```

启动后检查：

```bash
sudo docker inspect demo_vla \
  --format 'NetworkMode={{.HostConfig.NetworkMode}} IpcMode={{.HostConfig.IpcMode}} User={{.Config.User}}'
```

期望：

```text
NetworkMode=host IpcMode=host User=1000:1000
```

不要用 `docker restart demo_vla` 作为 Fast DDS 问题恢复方式。需要重建运行态时：

```bash
bash docker/run_issac_ocs_docker.sh --replace
```

## 进入容器

普通使用：

```bash
sudo docker exec -it demo_vla bash
```

安装系统包或修复权限时进入 root：

```bash
sudo docker exec -u 0 -it demo_vla bash
```

镜像内提供两个辅助脚本：

```bash
build_ros2_core_for_isaac.sh
validate_isaac_ocs2_core.sh
```

`build_ros2_core_for_isaac.sh` 构建 SIM1 / Isaac / OCS2 主链路包。
`validate_isaac_ocs2_core.sh` 在 Isaac Sim 已 Play 且 OCS2/RViz 已启动后验证
ROS2 CLI、核心 Isaac topics 和 controller 状态。

## 基础测试

进入容器后：

```bash
source /opt/ros/humble/setup.bash
source /workspace/projects/ros2_ws/install/setup.bash 2>/dev/null || true

docker_env_check.sh

ros2 pkg prefix simulation_interfaces && echo SIMULATION_INTERFACES_OK
ros2 topic --help >/dev/null && echo ROS2_TOPIC_OK
ros2 launch --help >/dev/null && echo ROS2_LAUNCH_OK
ros2 control --help >/dev/null && echo ROS2_CONTROL_OK
ros2 pkg prefix rviz2 && echo RVIZ2_PKG_OK
```

测试 X11：

```bash
xeyes
```

测试 RViz2：

```bash
rviz2 --help >/dev/null && echo RVIZ2_OK
rviz2
```

如果 `xeyes` 能显示但 `rviz2` 不显示，记录 Qt/OpenGL 错误，重点检查：

```text
DISPLAY
QT_X11_NO_MITSHM
XAUTHORITY
/tmp/.X11-unix
NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display
```

## Isaac Sim 联调测试

宿主机 Isaac Sim：

```text
打开 /workspace/assets/datasets/sac-m/61.usd
确认 Console 没有 unresolved asset / missing reference 阻塞错误
点击 Play
```

容器内验证 DDS 数据流：

```bash
source /opt/ros/humble/setup.bash
source /workspace/projects/ros2_ws/install/setup.bash
export ROS2_DISABLE_DAEMON=1

ros2 topic list | grep -E "clock_test|^/clock$|isaac_joint_states|arm_joint_cmd|gripper"

timeout 5 ros2 topic echo /clock_test --once
timeout 5 ros2 topic echo /clock --once
timeout 5 ros2 topic echo /isaac_joint_states --once
```

期望三者都有消息。若 topic 可见但 echo 无消息，先看：

```text
docker/FAST_DDS_DOCKER_IPC_UID_NOTES.md
```

关键要求：

```text
NetworkMode=host
IpcMode=host
User=宿主机 Isaac Sim 用户 UID/GID
```

## OCS2 + RViz2 启动

只有 `/isaac_joint_states` 能收到消息后，再启动 OCS2/RViz2：

```bash
cleanup_ros_runtime.sh || true

source /opt/ros/humble/setup.bash
source /workspace/projects/ros2_ws/install/setup.bash

cd /workspace/projects/ros2_ws
ros2 launch r1_description ocs2_isaac.launch.py enable_rviz:=true
```

另一个终端验证 controller：

```bash
sudo docker exec -it demo_vla bash -lc '
source /opt/ros/humble/setup.bash
source /workspace/projects/ros2_ws/install/setup.bash

timeout 10 ros2 service call /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers "{}"

ros2 control list_controllers --controller-manager /controller_manager
'
```

期望：

```text
joint_state_broadcaster active
ocs2_arm_controller active
```

如果 `ocs2_arm_controller` 报 CppAD generated 目录无权限：

```bash
sudo docker exec -u 0 demo_vla bash -lc '
find /workspace/projects/ros2_ws/install/r1_description/share/r1_description/ocs2 \
  -path "*/cppad_generated" -type d -exec chown -R 1000:1000 {} +
'
```

然后重新启动 launch。

## SIM1 检查和使用

检查：

```bash
sudo docker exec -it demo_vla bash -lc '
source /opt/ros/humble/setup.bash
source /workspace/projects/ros2_ws/install/setup.bash
cd /workspace/projects/teleoperation/sim1
./game_control.sh check
'
```

常用命令：

```bash
sim1_check
sim1_teach
sim1_demo
sim1_reset
```

最小 teach 复测顺序：

第一次测试 teach 时不要先使用 `--record-cameras`，也不要先用 pygame。先跑
terminal 模式，确认能进入示教并写入 `trace_data/raw`：

```bash
cd /workspace/projects/teleoperation/sim1
./game_control.sh teach --input-mode terminal \
  --linear-speed 0.025 \
  --angular-speed-deg 4.0 \
  --control-rate 50
```

terminal 模式通过后，再测试 pygame：

```bash
./game_control.sh teach --input-mode pygame \
  --linear-speed 0.025 \
  --angular-speed-deg 4.0 \
  --control-rate 50
```

只有相机 topic 存在后，再测试相机录制：

```bash
./game_control.sh teach --input-mode pygame \
  --linear-speed 0.025 \
  --angular-speed-deg 4.0 \
  --control-rate 50 \
  --record-cameras \
  --camera-depth-save-every 1
```

相机 topic 诊断：

```bash
ros2 topic list | grep -E "cam|camera|image|depth|rgb|color"
```

期望存在：

```text
/head_cam/color/image_raw
/head_cam/depth/image_rect_raw
/left_cam/color/image_raw
/left_cam/depth/image_rect_raw
/right_cam/color/image_raw
/right_cam/depth/image_rect_raw
```

如果这些 topic 不存在，不要强行使用 `--record-cameras`。需要在 Isaac Sim UI
检查：

```text
Camera_Publish_Graph 是否存在
Camera_Publish_Graph 是否 enabled
OnPlaybackTick 是否连接
ROS2 Context 是否有效
render product 是否有效
```

reset simulation 诊断：

```bash
ros2 service list | grep -E "reset|simulation"
ros2 pkg prefix simulation_interfaces
```

如果 `simulation_interfaces` 存在但 `/reset_simulation` 不存在，容器包没有问题，
是 Isaac Sim 侧 simulation control extension/service 没启用。不要把它作为 teach
主链路失败处理。

当前关键通过条件：

```text
/clock 有消息
/isaac_joint_states 有消息
RViz2 能显示
controller_manager/list_controllers 能响应
joint_state_broadcaster active
ocs2_arm_controller active
```

## 开发者测试清单

每次修改 Dockerfile、run 脚本、entrypoint 或 ROS2 依赖后，至少执行：

```bash
sudo docker build -t issac_ocs_docker -f docker/Dockerfile .
bash docker/run_issac_ocs_docker.sh --replace

sudo docker inspect demo_vla \
  --format 'NetworkMode={{.HostConfig.NetworkMode}} IpcMode={{.HostConfig.IpcMode}} User={{.Config.User}}'

sudo docker exec -it demo_vla bash -lc '
source /opt/ros/humble/setup.bash
source /workspace/projects/ros2_ws/install/setup.bash 2>/dev/null || true

docker_env_check.sh
ros2 pkg prefix simulation_interfaces && echo SIMULATION_INTERFACES_OK
ros2 topic --help >/dev/null && echo ROS2_TOPIC_OK
ros2 launch --help >/dev/null && echo ROS2_LAUNCH_OK
ros2 control --help >/dev/null && echo ROS2_CONTROL_OK
rviz2 --help >/dev/null && echo RVIZ2_OK
'
```

如果 `simulation_interfaces` 输出 `Package not found`，说明正在使用的镜像没有按
最新 Dockerfile 构建。修复方式是重新 build：

```bash
sudo docker build -t issac_ocs_docker -f docker/Dockerfile .
bash docker/run_issac_ocs_docker.sh --replace
```

不要用旧的已提交镜像做分发。

Isaac Sim 已 Play 时，再执行：

```bash
sudo docker exec -it demo_vla bash -lc '
source /opt/ros/humble/setup.bash
source /workspace/projects/ros2_ws/install/setup.bash
export ROS2_DISABLE_DAEMON=1

timeout 5 ros2 topic echo /clock --once
timeout 5 ros2 topic echo /isaac_joint_states --once
'
```

最后执行：

```bash
sudo docker exec -it demo_vla bash -lc '
source /opt/ros/humble/setup.bash
source /workspace/projects/ros2_ws/install/setup.bash
cd /workspace/projects/teleoperation/sim1
./game_control.sh check
'
```

## 分发方式 A：分发源码和 Dockerfile

适合接收方能访问基础镜像和 apt 源的情况。

交付内容：

```text
docker/Dockerfile
docker/entrypoint.sh
docker/run_issac_ocs_docker.sh
docker/README_DOCKER.md
docker/DOCKER_PROJECT_TEST_USE_DISTRIBUTE.md
docker/FAST_DDS_DOCKER_IPC_UID_NOTES.md
scripts/docker_env_check.sh
scripts/cleanup_ros_runtime.sh
scripts/build_ros2_core_for_isaac.sh
scripts/validate_isaac_ocs2_core.sh
VERSION.md
TEST_REPORT.md
SHA256SUMS
```

接收方执行：

```bash
sudo docker build -t issac_ocs_docker -f docker/Dockerfile .
bash docker/run_issac_ocs_docker.sh --replace
```

同时需要单独同步 `/workspace/projects` 和完整 `/workspace/assets/datasets/sac-m`。

## 分发方式 B：导出镜像 tar

适合接收方无法稳定重新 build，但 Docker 版本兼容的情况。

导出：

```bash
sudo docker save issac_ocs_docker:2026-06-08-rviz-ocs2-sim1 | \
  gzip > issac_ocs_docker_2026-06-08-rviz-ocs2-sim1.tar.gz
sha256sum issac_ocs_docker_2026-06-08-rviz-ocs2-sim1.tar.gz \
  > issac_ocs_docker_2026-06-08-rviz-ocs2-sim1.tar.gz.sha256
```

接收方导入：

```bash
sha256sum -c issac_ocs_docker_2026-06-08-rviz-ocs2-sim1.tar.gz.sha256
gunzip -c issac_ocs_docker_2026-06-08-rviz-ocs2-sim1.tar.gz | sudo docker load
```

导入后仍然使用本工程的 run 脚本启动：

```bash
bash docker/run_issac_ocs_docker.sh --replace
```

镜像 tar 仍不包含 `/workspace/assets/datasets/sac-m`、`trace_data`、
`ros2_log`、Isaac Sim 安装目录。它们应作为外部数据按需同步。

## 分发方式 C：私有 Registry

适合多台机器或团队内部使用。

打 tag：

```bash
sudo docker tag issac_ocs_docker <registry>/<namespace>/issac_ocs_docker:<version>
```

推送：

```bash
sudo docker push <registry>/<namespace>/issac_ocs_docker:<version>
```

接收方拉取：

```bash
sudo docker pull <registry>/<namespace>/issac_ocs_docker:<version>
IMAGE_NAME=<registry>/<namespace>/issac_ocs_docker:<version> \
  bash docker/run_issac_ocs_docker.sh --replace
```

建议 version 包含日期和用途，例如：

```text
2026-06-05-rviz-ocs2-fastdds
```

## 接收方交付清单

给接收方时至少包含：

```text
镜像获取方式：Dockerfile build / tar.gz / registry
run 脚本：docker/run_issac_ocs_docker.sh
使用说明：docker/README_DOCKER.md
本文档：docker/DOCKER_PROJECT_TEST_USE_DISTRIBUTE.md
Fast DDS 排障：docker/FAST_DDS_DOCKER_IPC_UID_NOTES.md
完整 sac-m 资产同步命令
workspace 目录约定
Isaac Sim 打开 61.usd 并 Play 的说明
基础测试命令和期望输出
```

## 禁止事项

不要执行：

```text
apt upgrade
apt dist-upgrade
删除 trace_data/raw
删除 trace_data/cleaned
删除 ros2_log
删除 USD 或 sac-m 资产
把 Isaac Sim 安装目录 COPY 进镜像
把大体积数据集 COPY 进镜像
改 topic 名称
改 OCS2 控制参数
在 /isaac_joint_states 没有消息前反复 spawn controller
```

## 常见问题定位顺序

1. 容器配置：

   ```bash
   docker inspect demo_vla \
     --format 'NetworkMode={{.HostConfig.NetworkMode}} IpcMode={{.HostConfig.IpcMode}} User={{.Config.User}}'
   ```

2. Isaac Sim 是否 Play，是否打开完整 `/workspace/assets/datasets/sac-m/61.usd`。

3. `/clock`、`/isaac_joint_states` 是否有实际消息。

4. RViz2/X11 是否能显示。

5. `controller_manager/list_controllers` 是否响应。

6. `joint_state_broadcaster` 和 `ocs2_arm_controller` 是否 active。

7. `./game_control.sh check` 剩余 warning。
