# 使用与复现

[English](getting-started.md) · [验证报告](../reports/VALIDATION_20260916.md)

所有命令在项目根目录执行。当前主机基线为 Ubuntu 24.04、Isaac Sim 5.1、
ROS 2 Jazzy、Conda Python 3.12；Isaac 使用独立的 Python 3.11。
`ISAAC_SIM_ROOT` 指向本机外部安装目录，工程资产由仓库位置解析。

## 准备环境

```bash
git lfs install
git lfs pull
python3 scripts/prepare_portable_urdf.py
export ISAAC_SIM_ROOT=/path/to/isaac-sim-standalone-5.1.0-linux-x86_64
source scripts/activate_sim1_conda.sh
./run.sh build
./run.sh preflight
```

ZIP 源码包不能直接执行 Git LFS，请从 Git 克隆补齐真实网格和 USD。
系统依赖见 `dependencies/docker-apt-packages.lock`，额外 Conda wheel 见
`dependencies/sim1-requirements.txt`，在环境内使用 `pip install --no-deps -r ...`。
RViz2 使用 ROS 系统包。本机缺失的 deb 依赖已放入忽略提交的
`.runtime/ros-jazzy`，激活脚本自动检测；其他机器应按依赖清单安装，移动工程后重编。

需要重建便携场景时运行：

```bash
"$ISAAC_SIM_ROOT/python.sh" scripts/prepare_portable_scene.py
```

外层 USD 必须保留 **Z 轴向上、米单位、每秒 60 时间码**。
脚本还将缺失的相机外壳 MDL 材质替换为 OmniPBR。

## 完整示教与采集

```bash
export ROS_DOMAIN_ID=73
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source scripts/activate_sim1_conda.sh
./run.sh sim1-session --name my_episode
```

统一入口打开 Isaac GUI 并开始 Play，等待控制器启动，打开 pygame 示教与
六路相机录制，再打开 2×3 RGB/深度预览。将焦点放在 **pygame 窗口**：
`1` 左臂、`2` 右臂、`3` 底盘；底盘用方向键；机械臂用 `W/S`、`A/D`、
`R/F` 平移；`V/B` 左夹爪闭/开，`N/M` 右夹爪闭/开。
按 `Q` 完成记录并退出本会话进程。输出为
`outputs/sessions/my_episode/trace.csv` 和 `trace/camera/`，已有会话名会拒绝覆盖。

pygame 模式关闭 RViz 目标管理器，防止两个发布者覆盖同一末端目标。
关节约束保持开启；当前 OCS2 权重仅验证了小幅示教，不代表任意抓取任务已通过。

也可以分终端运行：

```bash
./run.sh start-isaac
./run.sh launch-ocs2 enable_rviz:=false enable_target_manager:=false
./run.sh sim1-teach
./run.sh sim1-camera-grid
```

`start-isaac` 只负责场景与 Play；**录制由 session 或 teach 启动**。
支持 `--headless`、`--duration SECONDS`、`--no-play`。
使用 RViz 标记控制时打开目标管理器并停止 pygame。

## 转换与验证

```bash
./run.sh sim1-process outputs/sessions/my_episode/trace.csv --with-v21
python scripts/validate_lerobot_export.py \
  projects/teleoperation/sim1/trace_data/lerobot_v21/my_episode_cleaned \
  projects/teleoperation/sim1/trace_data/lerobot_v30/my_episode_cleaned
```

清洗会去掉长时间空闲段。导出统一按墙钟时间重采样为 20 FPS，轨迹和 RGB
使用同一时间网格；原始 `ros_time_sec` 保留仿真时间。LeRobot 标准视频特征为
三路 RGB；三路 float32 米制深度以无损 `depth/*.npy` 与
`depth/index.parquet` 辅助索引保留，不冒充 RGB 视频。原始 CSV 含底盘命令和里程计。
LeRobot 状态/动作维度继续采用原有双臂与夹爪 16 维接口，未擅自拼入底盘量。
`action` 表示下一帧测量状态，不是标定后的硬件命令。

验证器检查元数据、有限状态/力矩数值、固定 FPS 时间戳、视频逐帧解码和帧数、
深度路径与数据类型/尺寸。尚未执行官方 LeRobot 训练加载器。
如日后需要新增自动测试样本，可运行
`./run.sh sim1-session --name unique_test --validate-10s`。

## 接口与排查

| 接口 | 含义 |
| --- | --- |
| `/clock` | 仿真时钟 |
| `/isaac_joint_states` | Isaac 实测关节 |
| `/arm_joint_cmd` | OCS2 关节位置命令 |
| `/left_target/stamped`、`/right_target/stamped` | 每臂只能有一个有效目标发布源 |
| `/odom` | `odom` 到 `base_link` 的底盘里程计 |
| `/sim1/base_diff_cmd` | Twist 底盘命令 |
| `/{head,left,right}_cam/color/image_raw` | 三路 RGB，reliable QoS |
| `/{head,left,right}_cam/depth/image_rect_raw` | 三路 float32 深度，reliable QoS |

场景颠倒首先检查外层 USD 元数据。此前 Bridge 原生崩溃已通过“启用扩展后先
更新一帧，再加载场景”的顺序修复。控制迟钝时查看日志中的 `playing`、
`simulation_time` 和 `frames`；仅打开 GUI 不代表时间正常推进。
Isaac 与外部 ROS 统一使用 Fast DDS；本机混用中间件时曾出现图像严重丢失与降速。
