# Naming and relocation validation / 命名与迁移验证

The active scene directory is `assets/scenes/r1_workcell/`; root physics overrides
are `assets/scenes/physics_overrides.usd`. Dated command implementations use stable
names under `scripts/`. `projects/Trajectory/SIM1` moved to
`projects/teleoperation/sim1`. Existing local recordings moved with that directory
but are excluded from Git publication.

运行时检查：Isaac Sim 5.1 无界面短时运行正常退出，场景、ROS Bridge、控制图和六路
RenderProduct 均就绪；心跳为 `playing=true`，观测到 487 帧、8.117 秒仿真时间。
此次未运行录制。110 个 Python 文件语法检查通过，shell 和目录检查通过。
迁移到含空格路径的入口检查、36 个脚本的两种路径解析模式均通过。

Both existing LeRobot exports passed the local validator after relocation:
131 frames, 20 FPS, three decoded RGB videos and three lossless depth sidecars.
This does not test the official LeRobot training loader.

[Path checks](layout_portability_20260916.json),
[USD dependency scan](scene_asset_resolution.json),
[earlier capture validation](VALIDATION_20260916.md).

The standalone USD dependency scanner does not resolve Isaac's built-in MDL search
paths. Its remaining missing material entries are built-in OmniPBR/OmniGlass and
three original camera housing materials already overridden by the portable layer.
No renamed USD layer is unresolved. Native startup produced no ERROR or Python
traceback in this run. Existing behavioral limitations remain in the earlier report.

主分支发布工程与测试结论，独立 OmniSim 测试脚本和原始查询证据在
[develop/omnisim](https://github.com/123tthh/Isaac-Robot-Simulation/tree/develop/omnisim/evaluations/omnisim)。
