> **Historical record / 历史记录**：本页描述其记录日期的状态；当前运行以[中文指南](../docs/getting-started.zh-CN.md) / [English guide](../docs/getting-started.md)为准。Historical failures and paths are not current release claims.

# Isaac OCS 工程迁移收尾报告（2026-07-24）

## 结论

`/home/gtk/teleoperation`、`/home/gtk/ros2_ws` 和 `/home/gtk/ros2_log`
的数据入口已统一到 `${PROJECT_ROOT}`。前两者继续作为观察期兼容
链接；旧日志实体目录已在逐文件核验后删除并替换为兼容链接。

当前唯一运行场景：

`${PROJECT_ROOT}/assets/scenes/scene.usd`

最终 SHA-256：

`e3778c83fee7352fb72d16c1ea7837378c681e16daa0574cf0f3815272b71bd7`

## 日志与数据

- 操作前未发现 Isaac Sim、夹爪脚本或日志文件写入进程。
- `left_gripper_v45.log` 和 `right_gripper_v45.log` 从旧目录合并后分别与
  源文件逐字节一致：
  - `d8525c31e08f6473819a6b98899b22be217e6fd7b1af7d2a9e01e1a8acc0be32`
  - `cf4d9fa0e53ade62220e4e75240882abc9417981a565c61a852666c85488a757`
- 合并前副本位于
  `backups/ros2_log_finalization_20260724/`。
- 工程独有 LeRobot v3.0 数据保留 8 个文件，复核哈希未变化。
- 九条 `00_CURRENT_ACTIVE` 链接均为工程内部相对链接。
- 三条历史 `latest` 链接均已恢复，`data/ros2_log` 内失效链接数为 0。
- `/home/gtk/ros2_log` 现指向
  `${PROJECT_ROOT}/data/ros2_log`。
- 删除的旧实体目录不再可单独恢复；其有效内容已经过比较并保存在工程目录，
  旧目录独有的 14 个文件全部是 Python 字节码缓存。

## 路径修复

- 三个活动夹爪脚本改用 `ROS2_LOG_DIR`，默认写入工程日志目录。
- 场景内左右夹爪 ScriptNode 的内嵌脚本已迁移，复查待迁移项为 0。
- 修改场景前备份：
  `backups/scene_runtime_paths_20260724/scene.usd.20260724_151817.bak`。
- 独立 `projects/teleoperation/robot/r1_fixed.urdf` 的 31 个唯一 mesh URI
  已改为 `package://r1_description/meshes/...`；31 个资源全部存在，
  `check_urdf` 解析通过。
- Script Editor 辅助脚本已统一打开 `assets/scenes/scene.usd`，不再执行
  `/home/gtk/teleoperation/...`。

## Docker

- 合并了 host IPC/PID、host network、宿主 UID/GID、4 GiB `/tmp` tmpfs、
  GPU、X11/Xauthority 兼容配置。
- 宿主没有 `.Xauthority` 时，CLI 启动器不再挂载不存在的文件；Compose
  使用 `/dev/null` 安全回退。
- 合并了旧 Docker 构建依赖与三份运行验证文档。
- 宿主和容器使用独立 overlay：
  - 宿主：`build/install/log`
  - 容器：`build_docker/install_docker/log_docker`
- 容器隔离 overlay 完成 27 个包构建。
- 当前 `issac_ocs_docker:latest` 实测通过：
  - 用户 `1000:1000`
  - host network、host IPC、host PID
  - `/tmp` tmpfs
  - RTX 5090 D v2，驱动 580.126.09
  - `r1_description` 与 `ocs2_arm_controller` 包发现
  - Fast DDS 单次发布/订阅收到 `docker_ok`
- 随后已执行无缓存完整构建。新镜像
  `issac_ocs_docker:latest` / `issac_ocs_docker:20260724-full`
  的镜像 ID 为
  `3fc6e462bfc8c269e77818342698b477b060adc18c041dde89067d4bf182582e`，
  大小约 24.0 GB。镜像内置 `/entrypoint.sh` 已通过 GPU、ROS 包和 DDS
  烟雾测试，旧镜像标签及 dangling 镜像记录已删除。

## 构建与烟雾测试

- 宿主核心工作区：27 个包构建成功。
- 工程布局检查：0 个失败。
- 当前 USD：500 个 prim，Arm、State、Gripper、Camera、Base 五组图存在。
- ROS launch 可初始化 robot state publisher、topic-based ros2_control
  硬件和 controller manager。
- Codex 文件系统沙箱禁止宿主 UDP socket 和宿主 NVML，因此该次受限宿主
  launch 出现 `Operation not permitted`；宿主 GPU 已在 Isaac USD 检查中
  成功使用，Docker 的 GPU 与 DDS 测试均在宿主命名空间通过。

## 已知遗留

打开当前场景时仍有一条资产级警告：
`/World/Robot/base_link/visuals` 引用的
`r1_workcell/configuration/physics.usd</visuals/base_link>`
无法解析。该依赖目录因此继续保留；此警告不影响本次路径和日志迁移结论，
但应作为后续场景资产修复任务处理。
