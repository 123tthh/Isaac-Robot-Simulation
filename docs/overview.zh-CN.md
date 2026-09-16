# 项目介绍

[English](overview.md) · [开始使用](getting-started.zh-CN.md) · [文档目录](README.md)

本项目提供 R1 轮式双臂机器人的仿真、键盘示教、RGB-D 数据采集与
LeRobot 数据转换流程。运行基线为 Isaac Sim 5.1、ROS 2 Jazzy 和 OCS2；
外部 ROS Python 环境通过 Conda 管理，Isaac 使用自身 Python。

## 工作流程

1. Isaac 加载便携场景并开始仿真，通过 ROS Bridge 发布关节、里程计和相机数据。
2. pygame 选择左臂、右臂或底盘；OCS2 将末端目标转换为机械臂关节命令。
3. 采集轨迹、底盘反馈及头部/双腕三组 RGB-D，即六路图像流。
4. 清洗、重采样并导出 LeRobot v2.1/v3.0，离线检查数据与视频是否一致。

## 项目入口

根目录 **`run.sh`** 是命令入口，执行 `./run.sh` 显示帮助。

| 任务 | 在项目根目录执行 |
| --- | --- |
| 检查环境 | `./run.sh preflight` |
| 编译 ROS 核心包 | `./run.sh build` |
| 只启动仿真 | `./run.sh start-isaac` |
| 示教并录制六路相机 | `./run.sh sim1-session --name my_episode` |
| 转换两种 LeRobot 格式 | `./run.sh sim1-process outputs/sessions/my_episode/trace.csv --with-v21` |

启动前先按[环境与操作指南](getting-started.zh-CN.md)安装依赖并激活环境。
录制不是默认行为：无参数入口只显示帮助，`start-isaac` 不启动录制。

## 目录约定

| 位置 | 用途 |
| --- | --- |
| `assets/` | 场景和网格，克隆后需要拉取 Git LFS 真实文件 |
| `projects/ros2_ws/` | ROS 工作区，唯一机器人源码为 `src/robot/` |
| `projects/teleoperation/sim1/` | 已有示教、录制与转换模块；保留内部路径兼容性 |
| `projects/physics_parameters/` | 独立物理参数研究脚本与参考结果 |
| `scripts/` | 启动实现、资源准备与校验工具 |
| `dependencies/`、`docker/` | 依赖清单和容器定义 |
| `docs/` | 当前项目介绍、操作指南和明确标识的历史说明 |
| `reports/`、`evaluations/` | 本机验证与独立跨仿真器结果 |
| `releases/`、`reproducibility/` | 冻结快照、校验清单和外部源码恢复资料 |
| `outputs/`、`data/ros2_log/`、`.runtime/` | 本机数据、日志与依赖，不是源码分发内容 |

当前用户文档采用稳定英文文件名，中文版本以 `.zh-CN.md` 结尾。
日期用于历史证据和发布版本；ROS 包名、第三方接口和本项目维护的模块目录使用描述性名称。旧的 `scripts/project.sh` 仍作为兼容入口；日期版入口已移除。

## 可移植性与验证边界

工程内路径从脚本位置或项目标记解析，不绑定用户名。外部 Isaac 安装通过
`ISAAC_SIM_ROOT` 指定。容器 `/workspace`、ROS `/opt/ros` 和 USD `/World`
分别是容器约定、系统安装路径和场景节点，不应机械替换成文件相对路径。
移动工程后需重建 ROS `build/install/log`，不能复制旧 CMake 缓存当作可移植构建。

六路采集和两种 LeRobot 本地导出已检查。最后自动底盘修正未重新录制，官方
LeRobot 训练加载器未验证；OmniSim 的接触问题仍未解决。当前证据见
[本机报告](../reports/VALIDATION_20260916.md)与
[跨仿真器报告](../evaluations/omnisim/README.md)。旧交接材料中的完成声明不作为当前结论。
