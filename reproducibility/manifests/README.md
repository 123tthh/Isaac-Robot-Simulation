# 校验清单

- `scene_assets.sha256`：唯一入口 `assets/scenes/scene.usd` 及其项目内依赖层。
- `robot_resources.sha256`：唯一机器人源码 `projects/ros2_ws/src/robot`。
- `sim1_data.sha256`：唯一保留的 SIM1 success 数据族。
- `nested_patches.sha256`：嵌套仓库 tracked patch。
- `external_artifacts.sha256`：不进入 Git 的数据、源码和镜像归档。
- `nested_repositories.tsv`：外部仓库 URL、commit 和 dirty 状态。
- `docker_image.sha256`：Docker 镜像 ID、RepoDigest 或离线归档哈希。
- `SHA256SUMS`：复现目录中补丁和外部归档的总清单。
