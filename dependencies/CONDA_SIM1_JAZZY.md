# Conda / ROS Jazzy runtime

[中文使用指南](../docs/getting-started.zh-CN.md) · [English operations](../docs/getting-started.md)

Ubuntu 24.04 + ROS Jazzy provides RViz2, rclpy, NumPy, SciPy and OpenCV.
Conda manages Python 3.12 and the pinned extra wheels in `sim1-requirements.txt`.
Isaac Sim 5.1 uses its own Python 3.11 and internal ROS libraries.

Ubuntu/ROS 提供 RViz2、rclpy 及数值/图像库，Conda 管理 Python 3.12 和额外 wheel。
Isaac 内嵌 Python 3.11 必须与外部 ROS 环境隔离。

```bash
conda create -n ros2-jazzy python=3.12 pip
conda activate ros2-jazzy
python -m pip install --no-deps -r dependencies/sim1-requirements.txt
source scripts/activate_sim1_conda.sh
./scripts/project.sh build
```

Install system dependencies using `docker-apt-packages.lock` as the package list.
The local `.runtime/ros-jazzy` overlay is auto-detected if present. It contains
extracted debs used on this host, including controller-manager, joint-state-broadcaster
and development libraries. It is ignored by Git; it must be recreated or replaced
by system-installed packages on another machine. `ROS_DEPENDENCY_OVERLAY` can
select another overlay. Avoid a volatile `/tmp` directory for continued operation.

本机 `.runtime/ros-jazzy` 自动检测，包含补齐的控制器和开发包。
此目录不提交 Git；复刻时按清单安装依赖，或恢复覆盖层。
可用 `ROS_DEPENDENCY_OVERLAY` 指定其他位置。工作区移动后应重新构建。
