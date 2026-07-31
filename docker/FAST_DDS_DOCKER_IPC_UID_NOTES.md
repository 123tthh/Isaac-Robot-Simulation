# Fast DDS Host Docker 通信问题总结

## 现象

Isaac Sim 在宿主机运行，ROS2 节点在 `demo_vla` Docker 容器内运行时，出现过以下现象：

```text
ros2 topic list 可以看到 /clock、/clock_test、/isaac_joint_states
ros2 topic info 可以看到 Publisher count: 1
ros2 topic echo --once 收不到任何消息
```

这说明 DDS discovery 已经通，但 DDS data path 没有通。问题不在 USD、Isaac Sim
Action Graph、ROS2 Bridge endpoint 或 topic 名称本身。

## 根因

### 1. Docker IPC namespace 隔离

原容器状态是：

```text
NetworkMode=host IpcMode=private
```

Fast DDS 在同机通信时可能使用 shared-memory transport。宿主机 Isaac Sim 和
Docker 容器处在不同 IPC namespace 时，topic endpoint 可以被发现，但共享内存
数据路径会失败。

修复后要求：

```text
NetworkMode=host IpcMode=host
```

`docker/run_issac_ocs_docker.sh` 中需要保留：

```bash
--network host --ipc host
```

### 2. 容器 root 用户与宿主机 Isaac Sim 用户权限不兼容

只加 `--ipc host` 还不够。如果容器内 ROS2 进程以 root 运行，它会在宿主机
`/dev/shm` 中创建 root 拥有的 `fastrtps_*` shared-memory 文件。宿主机 Isaac Sim
通常以普通用户 UID 1000 运行，可能无法向这些 root-owned 端口写入数据。

验证结果：

```text
容器 root 用户: topic 可见，但 /clock_test、/clock、/isaac_joint_states 无数据
容器 UID 1000 用户: 三个 topic 都能收到消息
```

因此容器默认需要用宿主机用户 UID/GID 启动：

```bash
--user "${HOST_UID}:${HOST_GID}"
```

脚本中必须兼容 `sudo bash docker/run_issac_ocs_docker.sh --replace` 场景，优先使用：

```bash
HOST_UID="${HOST_UID:-${SUDO_UID:-$(id -u)}}"
HOST_GID="${HOST_GID:-${SUDO_GID:-$(id -g)}}"
```

### 3. 非 root 启动后需要可写 `/tmp`

镜像里曾残留 root 拥有的：

```text
/tmp/isaac_ocs_aliases.bash
```

非 root 用户启动容器时，entrypoint 写该文件会失败：

```text
/entrypoint.sh: line 29: /tmp/isaac_ocs_aliases.bash: Permission denied
```

运行脚本通过给容器挂一个干净的 tmpfs `/tmp` 解决：

```bash
--tmpfs /tmp:rw,exec,nosuid,size=4g
```

同时设置：

```bash
-e XDG_RUNTIME_DIR="/tmp/runtime-${HOST_UID}"
```

避免 Qt/RViz2 使用镜像中不可写的旧 runtime 目录。

### 4. 非 root 运行 OCS2 时 CppAD generated 目录权限

ROS2 workspace 之前以 root 编译/安装，导致：

```text
/workspace/projects/ros2_ws/install/r1_description/share/r1_description/ocs2/*/cppad_generated
```

属于 root。非 root 启动 `ocs2_arm_controller` 时会失败：

```text
boost::filesystem::create_directory: Permission denied:
"/workspace/projects/ros2_ws/install/r1_description/share/r1_description/ocs2/dynamics_flow_map/cppad_generated/cppadcg_tmp-..."
```

最小修复是只调整这些 generated 运行期目录权限：

```bash
sudo docker exec -u 0 demo_vla bash -lc '
find /workspace/projects/ros2_ws/install/r1_description/share/r1_description/ocs2 \
  -path "*/cppad_generated" -type d -exec chown -R 1000:1000 {} +
'
```

不要改 OCS2 控制参数，不要改 topic 名称。

## 当前正确启动方式

```bash
bash docker/run_issac_ocs_docker.sh --replace
```

不要用 `docker restart demo_vla` 作为恢复手段。Fast DDS shared-memory 状态异常后，
应删除并重新 run 容器：

```bash
bash docker/run_issac_ocs_docker.sh --replace
```

验证容器配置：

```bash
docker inspect demo_vla --format 'NetworkMode={{.HostConfig.NetworkMode}} IpcMode={{.HostConfig.IpcMode}} User={{.Config.User}}'
```

期望：

```text
NetworkMode=host IpcMode=host User=1000:1000
```

## ROS2 数据流验证

进入容器：

```bash
sudo docker exec -it demo_vla bash
```

验证：

```bash
source /opt/ros/humble/setup.bash
source /workspace/projects/ros2_ws/install/setup.bash
export ROS2_DISABLE_DAEMON=1

timeout 5 ros2 topic echo /clock_test --once
timeout 5 ros2 topic echo /clock --once
timeout 5 ros2 topic echo /isaac_joint_states --once
```

当前已验证：

```text
/clock_test: OK
/clock: OK
/isaac_joint_states: OK
```

`/clock` 和 `/isaac_joint_states` 的 publisher 已显示为 Isaac Action Graph 节点：

```text
/_World_ActionGraphs_State_Telemetry_Graph_ros2_publish_clock
/_World_ActionGraphs_State_Telemetry_Graph_ros2_publish_joint_state
```

## OCS2/RViz2 验证

启动：

```bash
cleanup_ros_runtime.sh || true
source /opt/ros/humble/setup.bash
source /workspace/projects/ros2_ws/install/setup.bash
cd /workspace/projects/ros2_ws
ros2 launch r1_description ocs2_isaac.launch.py enable_rviz:=true
```

当前已验证：

```text
RViz2 OpenGL: OK
joint_state_broadcaster: active
ocs2_arm_controller: active
controller_manager/list_controllers: responsive
```

SIM1 check 当前关键链路通过：

```text
ros2 control list_controllers:
joint_state_broadcaster active
ocs2_arm_controller active
```

残留 warning：

```text
[check] ROS environment
Package not found
```

该 warning 不再是 RViz2、ROS2 CLI、Fast DDS data path 或 controller_manager 阻塞点。

