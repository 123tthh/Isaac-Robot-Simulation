你现在接手一个 Isaac Sim 5.1 + ROS2 + OCS2 + Isaac Lab Arena 强化学习训练工程。目标是把当前依赖本地路径的工程整理成一个可迁移 Docker 环境，镜像命名为 `issac_ocs_docker`，运行容器默认命名为 `demo_vla`。

请先不要直接大改代码。按以下步骤执行，并在每一步给出修改文件列表和验证命令。

背景路径如下：

- 当前 ROS2/OCS2 工作区：`~/ros2_ws`
- 当前示教/回放/数据工程：`~/teleoperation`，核心目录是 `~/teleoperation/sim1`
- 当前历史日志和夹爪调试脚本：`~/ros2_log`
- 当前 Isaac Sim 5.1 是本地部署的，平时通过图标启动
- 当前 USD/场景资源：~/Desktop/IssacLab_arena_assets/datasets/sac-m/61.usd
- 目标容器内统一根目录：`/workspace`

工程目标：

1. 不要让容器内代码强依赖 `/home/gtk`、`/home/bt`、`Desktop` 等宿主机路径。
2. 统一使用环境变量：
   - `WORKSPACE=/workspace`
   - `ROS2_WS=/workspace/projects/ros2_ws`
   - `TRAJECTORY_DIR=/workspace/projects/teleoperation`
   - `SIM1_DIR=/workspace/projects/teleoperation/sim1`
   - `ROS2_LOG_DIR=/workspace/data/ros2_log`
   - `ISAAC_USD_PATH=/workspace/assets/scenes/scene.usd`
   - `OUTPUT_DIR=/workspace/outputs`
3. 工程代码、示教数据、USD、训练输出尽量通过 `-v /workspace:/workspace` 挂载，不要全部打死进镜像。
4. Docker 镜像只负责提供 ROS2/OCS2/Isaac Lab Arena/Python 依赖、启动脚本、环境检查脚本。
5. 生成的容器需要支持 GPU、host 网络、大 shm，并兼容以下运行风格：

```bash
sudo docker run -itd --gpus all \
  --privileged=true --net=host \
  -m 500G --shm-size=100G \
  -v /workspace:/workspace \
  --name demo_vla \
  issac_ocs_docker
```

请完成以下任务：

第一步：扫描工程中的本地强耦合路径。

在 `~/ros2_ws`、`~/teleoperation`、`~/ros2_log` 中搜索：

- `/home/gtk`
- `/home/bt`
- `/workspace`
- `Desktop`
- `Isaac`
- `isaac`
- `.usd`
- `522.usd`
- `ros2_ws`
- `Trajectory`
- `ros2_log`

输出一份 `docker/PATH_AUDIT.md`，按文件列出硬编码路径、用途、建议替换成哪个环境变量。不要删除原文件。

第二步：创建 Docker 目录结构。

在当前项目根目录或 `~/teleoperation/sim1` 下创建：

```text
docker/
  Dockerfile
  entrypoint.sh
  run_issac_ocs_docker.sh
  docker-compose.yml
  .env.example
  PATH_AUDIT.md
scripts/
  docker_env_check.sh
```

第三步：生成 `.env.example`。

内容至少包括：

```bash
WORKSPACE=/workspace
ROS2_WS=/workspace/projects/ros2_ws
TRAJECTORY_DIR=/workspace/projects/teleoperation
SIM1_DIR=/workspace/projects/teleoperation/sim1
ROS2_LOG_DIR=/workspace/data/ros2_log
ISAAC_USD_PATH=/workspace/assets/scenes/scene.usd
OUTPUT_DIR=/workspace/outputs
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
NVIDIA_VISIBLE_DEVICES=all
NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display
```

第四步：生成 `entrypoint.sh`。

要求：

- `set -e`
- 自动 source `/opt/ros/humble/setup.bash`，如果存在
- 自动 source `$ROS2_WS/install/setup.bash`，如果存在
- 自动导出上面的环境变量
- 提供 shell alias：
  - `ocs_check`
  - `ocs_launch`
  - `sim1_check`
  - `sim1_teach`
  - `sim1_demo`
  - `sim1_reset`
  - `sim1_clean`
  - `sim1_convert`
- 默认进入 `/workspace`
- 不要依赖固定用户名 `gtk` 或 `joe`
- 不要在 entrypoint 中强制 useradd，避免 UID/GID 冲突

第五步：生成 Dockerfile。

要求：

- 基础镜像先基于当前可用的 Isaac Lab Arena / ROS2 镜像。如果无法判断基础镜像，请写成可配置 ARG，例如：
  `ARG BASE_IMAGE=isaaclab_arena:ros2_ocs2`
- 安装常用工具：git、vim、tmux、curl、wget、python3-pip、python3-venv、build-essential、cmake、rsync
- 不要把大数据、trace_data、ros2_log、USD 复制进镜像
- COPY `entrypoint.sh` 和 `scripts/docker_env_check.sh`
- 设置 `ENTRYPOINT ["/entrypoint.sh"]`
- 默认 `CMD ["/bin/bash"]`

第六步：生成 `run_issac_ocs_docker.sh`。

要求：

- 自动创建 `/workspace/projects`、`/workspace/data`、`/workspace/assets`、`/workspace/outputs`
- 检查 `/workspace/projects/ros2_ws` 是否存在
- 检查 `/workspace/projects/teleoperation/sim1` 是否存在
- 检查 `$ISAAC_USD_PATH` 是否存在，不存在只 warning，不要退出
- 如果已有同名容器 `demo_vla`，提示用户是否删除，或提供 `--replace` 参数
- 运行命令使用：

```bash
sudo docker run -itd --gpus all \
  --privileged=true --net=host \
  -m 500G --shm-size=100G \
  -v /workspace:/workspace \
  -e WORKSPACE=/workspace \
  -e ROS2_WS=/workspace/projects/ros2_ws \
  -e TRAJECTORY_DIR=/workspace/projects/teleoperation \
  -e SIM1_DIR=/workspace/projects/teleoperation/sim1 \
  -e ROS2_LOG_DIR=/workspace/data/ros2_log \
  -e ISAAC_USD_PATH=/workspace/assets/scenes/scene.usd \
  -e OUTPUT_DIR=/workspace/outputs \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display \
  --name demo_vla \
  issac_ocs_docker
```

第七步：生成 `scripts/docker_env_check.sh`。

检查：

- `nvidia-smi`
- `python3 --version`
- `ros2 --help`
- `/opt/ros/humble/setup.bash`
- `$ROS2_WS/install/setup.bash`
- `$SIM1_DIR/game_control.sh`
- `$ISAAC_USD_PATH`
- `ros2 topic list`
- `ros2 service list`
- `$SIM1_DIR/game_control.sh check`

检查失败时只报告，不要直接删除任何文件。

第八步：修复路径强耦合。

只对明显安全的脚本进行最小修改：

- 把 `/home/gtk/ros2_ws` 替换为 `${ROS2_WS:-/workspace/projects/ros2_ws}`
- 把 `/home/gtk/teleoperation` 替换为 `${TRAJECTORY_DIR:-/workspace/projects/teleoperation}`
- 把 `/home/gtk/teleoperation/sim1` 替换为 `${SIM1_DIR:-/workspace/projects/teleoperation/sim1}`
- 把 `/home/gtk/ros2_log` 替换为 `${ROS2_LOG_DIR:-/workspace/data/ros2_log}`
  - 把硬编码 USD 路径替换为 `${ISAAC_USD_PATH:-/workspace/assets/scenes/scene.usd}`

每次修改前备份原文件为 `.bak_dockerize`。

第九步：输出最终使用说明 `docker/README_DOCKER.md`。

必须包含：

- 什么是镜像，什么是容器
- 如何准备 `/workspace` 目录
- 如何构建镜像：

```bash
sudo docker build -t issac_ocs_docker -f docker/Dockerfile .
```

- 如何运行容器：

```bash
bash docker/run_issac_ocs_docker.sh --replace
```

- 如何进入容器：

```bash
sudo docker exec -it demo_vla bash
```

- 如何检查环境：

```bash
docker_env_check.sh
```

- 如何启动 SIM1 检查：

```bash
sim1_check
```

- 如何启动示教：

```bash
sim1_teach
```

- 如何启动回放：

```bash
sim1_demo
```

- 如何查看 GPU：

```bash
nvidia-smi
```

第十步：不要做这些事：

- 不要删除 `trace_data/raw`
- 不要删除 `trace_data/cleaned`
- 不要删除 `ros2_log`
- 不要把大体积 Isaac Sim 安装目录直接 COPY 进镜像，除非我明确要求
- 不要把路径继续写死为 `/home/gtk`
- 不要在 entrypoint 中强制创建固定用户 `gtk`、`joe`
- 不要破坏当前本机可以运行的流程

完成后请给出：

1. 修改过的文件列表
2. 新增文件列表
3. 构建命令
4. 运行命令
5. 验证命令
6. 仍然需要我手动确认的路径，例如 Isaac Sim 安装路径和 USD 路径
