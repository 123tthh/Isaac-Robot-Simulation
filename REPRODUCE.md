# Isaac OCS 冷启动复现

## 核心范围

核心运行由 Isaac Sim 5.1.0、ROS 2 Humble、OCS2/SIM1、当前 USD 场景和
`r1_description` ROS 包组成。IsaacLab-Arena 不属于核心依赖。

本文件参考：

- `/home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_container.md`
- `/home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/omniverse_usd/open_usd.md`
- `/home/gtk/ai_docs/docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.md`

## 恢复步骤

1. 克隆主仓库并执行 `git lfs pull`。
2. 按 `dependencies/repositories.lock.yaml` 恢复核心外部仓库。
3. 将场景/数据制品解压回 manifests 声明的相对路径。唯一场景入口是
   `assets/scenes/scene.usd`；`assets/scenes/615scene_20260723/` 是其依赖层，
   不是另一个启动场景。
4. 安装 ROS 2 Humble underlay，或加载已校验的 Docker 镜像。
5. 构建核心宿主 overlay：

   ```bash
   ./scripts/project_control_20260723.sh build
   ```

   `build --all` 会连同 `robot_descriptions` 中约百个可选机器人包一起构建，
   需要额外安装 MoveIt、Grid Map、Ignition Gazebo 等依赖；它不是 OCS2/SIM1
   冷启动的必要步骤。

6. 检查布局和场景：

   ```bash
   ./scripts/verify_project_layout.sh
   ./scripts/project_control_20260723.sh preflight
   ./scripts/project_control_20260723.sh inspect-usd
   ```

7. 按 `GAME_GUIDE.md` 的 A/B/C 顺序启动 Isaac Sim、OCS2 和 SIM1。

工程可以位于任意绝对路径；只有外部 Isaac Sim 安装默认值是
`/home/gtk/isaac-sim-5.1`，可通过 `ISAAC_SIM_ROOT` 覆盖。

2026-07-24 的完整冷启动实测结果见
`reproducibility/VALIDATION_20260724.md`。
