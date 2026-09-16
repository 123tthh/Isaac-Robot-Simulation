> **Historical record / 历史记录**：本页描述其记录日期的状态；当前运行以[中文指南](../docs/getting-started.zh-CN.md) / [English guide](../docs/getting-started.md)为准。Historical failures and paths are not current release claims.

# 本机复刻与问题核查（2026-09-15）

本报告记录公开工程在 Ubuntu 24.04、RTX 5090、Isaac Sim 5.1.0 与 OmniSim 8.5.1 的实际运行结果。它是局部仿真验证，不是机器人硬件性能证明。

## 邮件中的问题

1. **LFS 属实。**收到的源码包包含 374 个 LFS 指针，其中 265 个是 `.obj/.dae/.STL/.stl` 网格；机器人 `meshes/` 的 39 个文件和主场景 `scene.usd` 均不可直接解析。已从公开上游拉取**全部 374 个**真实 LFS 文件回本机项目，逐文件检查为 0 个缺失、0 个指针，共 667.5 MiB。`python3 scripts/prepare_portable_urdf.py --check` 现在报告 77 个引用、31 个独立网格、0 个缺失、0 个网格指针、0 个场景指针。
2. **ROS 包 URI 属实。**原 `r1_fixed.urdf` 的 77 个网格引用都是 `package://r1_description/meshes/...`。保留它给 ROS；`r1_fixed_portable.urdf` 改为相对 `../meshes/...`，供独立导入。
3. **双臂失效的归因不完整。**OmniSim Newton 在真实网格与相对 URI 都修复后，仍报 `left_base_link` 未注册物理体。此时问题在固定连接的安装座与导入器的物理体归并。`r1_fixed_omnisim.urdf` 把七个动态关节的固定前驱变换合入关节原点，才通过 Newton finalize 和物理步进。该变体改变导入拓扑，安装座/法兰的碰撞表现需要额外核对，不能直接拿来断言抓取误差。
4. **公开场景还缺材质。**相机外壳的三种 MDL 模块在九处引用但文件缺失。`scene_portable.usda` 以相对 sublayer 加载主场景，并用 Isaac 的 `OmniPBR.mdl` 替代这些材质；物理不变，外壳外观可能变化。

## Isaac Sim 实测

- 本机 `nvidia-smi` 在沙盒外正常：RTX 5090、驱动 580.173.02、32 GB 显存。
- `scene_portable.usda` 在 Isaac Sim 5.1 headless 加载并推进 120 帧；`/World/Robot/base_link` 的动态关节 articulation 有效，USD 中 41 个 joint prim。见 `outputs/isaac_scene_smoke.json`。
- 不 source ROS 的独立 URDF 导入也成功：Isaac 内置导入器把 `r1_fixed_portable.urdf` 导为 `/r1`，108 个 USD Mesh prim 都有顶点，双臂及两侧夹爪 link 均存在，41 个 USD joint prim。见 `outputs/isaac_urdf_import.json`。导入日志对无惯性 base_link 和 in-memory stage 的 visual reference 有警告；本检查证明相对网格能导入，未证明所有碰撞/惯性完全正确。
- RGB-D 探针输出 320×240 RGB PNG 和深度 NPY；39,102 个有效深度像素，RGB 值 10–255。见 `outputs/rgbd_probe/report.json`。
- 直接控制采集 300 帧关节 CSV，26 DOF；`left_joint1` 目标 0.15 rad、终值 0.1499488 rad；两指闭合目标 0.04 m、终值 0.0402869/0.0397573 m。见 `outputs/isaac_direct_episode/report.json` 与 `joint_trace.csv`。
- **ROS Bridge 未打通。**默认 Python 环境启用 `isaacsim.ros2.bridge` 时请求设置 Jazzy/RMW/库路径。清洁环境中指定 Isaac 内置 Jazzy 库后，扩展独立启动并加载内置 Python 3.11 `rclpy`，见 `outputs/isaac_ros_bridge_internal_boot_20260915.log`；同环境加载此项目场景时原生层 `std::out_of_range`/abort，见 `outputs/isaac_ros_bridge_internal_scene_20260915.log`。使用系统 `/opt/ros/jazzy/lib` 也段错误，见 `outputs/isaac_scene_bridge_system_ros_20260915.log`。场景内五个 ROS ActionGraph 全部临时停用后仍段错误，见 `outputs/isaac_ros_bridge_no_graphs_20260915.log`，因此目前无法归因到单个旧节点。上述 CSV 是直接 dynamic_control 的局部数据，未验证 ROS 话题、OCS2 控制器或 LeRobot 实际录制流程。

## OmniSim 实测

- 从公开上游克隆 OmniSim 8.5.1 到 `/tmp/omnisim-source`，完成原生构建；使用临时 `/tmp/omnisim-physics` 安装其固定版本的 Newton/MuJoCo/Warp/USD 依赖。`python -m omnisim doctor` 报二进制、ABI 和物理 sidecar coherent。
- `outputs/omnisim_r1_import.omniworld` 从相对路径导入兼容 URDF；Newton/MuJoCo CPU 后端注册 27 个动态体、一个静态地板与 26 个求解器关节。原 portable URDF 导入失败，日志 `outputs/omnisim_r1_import_20260915.log`；兼容版成功，日志 `outputs/omnisim_r1_rewired_20260915.log`。
- 双指位置关节映射为 `left_PGIA_joint1_motor`、`left_PGIA_joint2_motor`，关节范围都是 0.04–0.12 m。开至 0.08 m 后实测 0.077668/0.081799 m；再闭至 0.04 m 后实测 0.040000/0.041858 m。见 `outputs/omnisim_left_gripper_open.json` 与 `outputs/omnisim_left_gripper_close.json`。
- 接触查询返回地板与轮/脚轮接触；没有任务物体与夹爪接触证据。当前世界没有固定抓取轨迹与目标物体，尚不能给邮件承诺的“仅改变物体位姿”完成率、终端误差表。导入器还警告碰撞网格缩放 1.05 未应用在 boundingObject；接触几何不能默认为与原 URDF 完全等同。

## 构建状态与复刻

本机只有 ROS Jazzy，缺系统级 ros2-control、Pinocchio 开发包与 Docker，且 `sudo -n` 要求密码。R1 所需的 `--packages-up-to` 依赖闭包 **25 个 ROS 包全部编译通过**，包括 `r1_description`、`r1_lerobot_sim`、`ocs2_arm_controller`、`arms_target_manager`、`topic_based_ros2_control`，日志为 `outputs/ros_jazzy_overlay_build_ninth_20260915.log`。为了继续核查，下载的 ROS 依赖仅解压在 `/tmp/ros-jazzy-overlay`，没有系统安装；工作区另有 OCS2 示例/其他机器人共计 75 包，全包构建日志为 `outputs/ros_jazzy_overlay_build_all_20260915.log`。临时目录不是项目的一部分，其他电脑仍需安装发行版依赖；`dependencies/` 和 `docker/` 是原工程锁定清单及容器路线。

`ISAAC_SIM_ROOT=... ./scripts/project.sh preflight` 在沙盒外报告 0 个失败，包括 GPU、场景、Isaac 入口和 ROS 包；原预检日志见 `outputs/project_preflight_20260915.log`。75 包全包构建在与 R1 无关的 `grid_map_filters_rsl` 缺 `grid_map_cv` 处停下；它没有推翻 R1 依赖闭包的 25 包构建结果。

R1 补充包 `arms_teleop`、`basic_joint_controller`、`gripper_hardware` 与 `r1_moveit_config` 的 `--packages-up-to` 再编过 8 包，见 `outputs/ros_jazzy_r1_extras_20260915.log`。

LeRobot ROS replay node 和 RViz launch 的旧 `/workspace/projects/ros2_ws` 默认路径已改用已安装 `r1_description` 包共享目录；`ros2 launch r1_lerobot_sim r1_lerobot_rviz_replay.launch.py --show-args` 在设置项目内 `ROS_LOG_DIR` 后显示本机 URDF/网格默认路径正确，见 `outputs/ros_r1_lerobot_launch_args_20260915.log`。这只检查启动参数，未回放 episode。

OCS2 `ocs2_robotic_assets` 原本在构建时把 `PROJECT_SOURCE_DIR` 写入生成头文件。模板和生成头已改成查询 `AMENT_PREFIX_PATH` 中的安装资源目录（可用 `OCS2_ROBOTIC_ASSETS_PATH` 覆盖）；重编包成功，并以 C++14 小程序实际找到 `install/ocs2_robotic_assets/share/ocs2_robotic_assets`，见 `outputs/ros_robotic_assets_rebuild_20260915.log`。

Docker 脚本中的 `/workspace` 是容器内固定挂载点，仍由启动器 `-v` 在各电脑映射本地工程目录；它不是依赖旧用户名的宿主机绝对路径。

从真正的 Git 克隆复刻资产和局部 Isaac 测试：

```bash
git lfs install
git lfs pull
python3 scripts/prepare_portable_urdf.py
python3 scripts/prepare_omnisim_urdf.py
export ISAAC_SIM_ROOT=/path/to/isaac-sim-standalone-5.1.0-linux-x86_64
$ISAAC_SIM_ROOT/python.sh scripts/prepare_portable_scene.py
$ISAAC_SIM_ROOT/python.sh scripts/isaac_urdf_import_smoke.py
ISAAC_USD_PATH="$PWD/assets/scenes/scene_portable.usda" $ISAAC_SIM_ROOT/python.sh scripts/isaac_scene_smoke.py
$ISAAC_SIM_ROOT/python.sh scripts/isaac_rgbd_probe.py
$ISAAC_SIM_ROOT/python.sh scripts/isaac_direct_episode.py
```

OmniSim 复刻时先按其公开仓库说明安装原生编译与 Newton/MuJoCo/Warp 依赖，然后：

```bash
git clone https://github.com/omnilink-tech/omnisim.git /path/to/omnisim
cd /path/to/omnisim
git submodule update --init --recursive
python3 -m omnisim doctor
python3 -m omnisim build all --jobs 8
python3 -m omnisim validate-urdf /path/to/Isaac-Robot-Simulation-main/projects/ros2_ws/src/robot/urdf/r1_fixed_omnisim.urdf --check-meshes
python3 -m omnisim run-headless /path/to/Isaac-Robot-Simulation-main/outputs/omnisim_r1_import.omniworld --duration 1 --require-newton
```

工程提取包没有有效 `.git` 元数据，不能在此目录直接 `git lfs pull`；这里的真实 LFS 资产已从上游检出。要验证完整 ROS/LeRobot 流程，仍需要补齐 ROS 系统依赖、解决 Isaac ROS Bridge 段错误，并提供一个可回放的原始 SIM1/LeRobot episode 或明确的任务轨迹。当前档案中没有所需 `trace_data`/`sim1_success` 任务数据。

## SIM1 主机依赖与统一命令

本机 `ros2-jazzy` Conda 环境已安装 `pygame==2.6.1`、`pandas==3.0.3`、`pyarrow==24.0.0`；ROS Jazzy 的 `rclpy`、OpenCV 与 RViz2 来自 `/opt/ros/jazzy`/Ubuntu deb。设置 `PYTHONNOUSERSITE=1` 后，pygame dummy 窗口、parquet 往返、示教器、相机记录器和 LeRobot 转换器的导入/参数入口通过。RViz2 在真实桌面会话下短时加载项目配置并报告 OpenGL 4.6。安装与验证日志在 `outputs/conda_ros2_jazzy_*.log`，操作说明见 `dependencies/CONDA_SIM1_JAZZY.md`。

`scripts/project.sh` 是当前稳定入口，承接构建、预检、Isaac、OCS2 与 SIM1 命令；`scripts/activate_sim1_conda.sh` 在项目根目录 source 后配置 Conda、Jazzy、工作区、日志目录。沙盒外运行 `./scripts/project.sh preflight` 报 0 个失败，显卡、便携场景和 `r1_description`/`ocs2_arm_controller`/`simulation_interfaces` 均通过。`sim1-camera-grid`、`sim1-base-record`、`sim1-base-path-demo` 的 `--help` 加载通过；缺 episode 时 `sim1-direct-demo` 明确拒绝执行。后处理脚本把输入 CSV 解析为绝对路径后再传给在 SIM1 目录运行的子脚本；成功示教生成的 latest 符号链接改为相对目标，搬迁项目目录后仍可解析。这些入口检查未采集新 episode，也未解除 ROS Bridge 崩溃。

原布局检查曾要求一条固定日期的历史示教 CSV，实际本机没有 `trace_data/raw`。`scripts/verify_project_layout.sh` 已改为检查工程布局并报告可用 raw trace 数量；原始数据缺席现在作为 `INFO`，本机运行 0 个失败，不再把未拷贝的数据误判成源码损坏。

统一构建入口首次在未指定临时开发库覆盖层时于 `topic_based_ros2_control` 链接阶段失败，错误 `cannot find -lcap`；本机只有运行时 `libcap.so.2`，缺 `libcap-dev` 开发链接名。设置 `ROS_DEPENDENCY_OVERLAY=/tmp/ros-jazzy-overlay` 后，`./scripts/project.sh build` 完成 25 个核心包、0 个失败，日志 `outputs/project_build_wrapper_20260915.log`。变量由构建入口映射到 CMake、头文件和链接器搜索路径；它是本机临时 deb 覆盖层，不是可迁移的工程内路径。

激活脚本已核对重复 `source`：隔离 ROS `PYTHONPATH` 后 Conda CLI 不再被系统旧版 `typing_extensions` 污染；设置覆盖层时两次激活后 OCS2 与 ros2_control 共享库用 `ldd` 检查无 `not found`。再运行稳定入口构建 25 包完成，日志 `outputs/project_build_wrapper_final_20260915.log`。
