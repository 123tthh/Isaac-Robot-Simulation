# 615scene USD 依赖迁移记录（2026-07-23）

## 迁移结果

已将 `615scene.usd` 的本地有效依赖迁移到：

`assets/scenes/615scene_20260723/`

保留的相对布局如下：

```text
615scene.usd
equipment.usd
configuration/513_base.usd
configuration/513_physics.usd
configuration/513_robot.usd
configuration/513_sensor.usd
isaac_assets/rsd455.usd
```

迁移前后本地依赖 SHA-256 已逐项比对一致。目标场景已用 Isaac Sim 5.1.0 headless `Usd.Stage.Open` 验证，退出码为 0。原场景的 RealSense S3 远程引用已替换为本地 `isaac_assets/rsd455.usd`，不再要求资产服务器可达。当前仍保留一个已知外部警告：

- `513_base.usd` 中 `base_link/visuals` 对 `513_physics.usd` 的未解析 prim path；
这不是本次复制造成的路径断裂；相机发布图、机器人和控制图仍被发现。

## 清理范围

清理只针对工程内的 `assets/datasets/sac-m/`，共 51 个旧文件（含旧场景、旧配置、测试 USD/MTL）。

外部源目录 `/home/gtk/Desktop/IssacLab_arena_assets/datasets/sac-m/` 未修改。

## 当前入口

所有默认启动脚本、Docker 配置和检查文档已指向：

`assets/scenes/615scene_20260723/615scene.usd`
