# Cross-simulator validation / 跨仿真器验证

The independent OmniSim evaluation is maintained on
[develop/omnisim](https://github.com/123tthh/Isaac-Robot-Simulation/tree/develop/omnisim/evaluations/omnisim).
That branch includes reproducible worlds, command scripts, joint mapping and raw
JSON results. This default branch contains the Isaac simulation and acquisition project.

独立 OmniSim 测试统一放在 `develop/omnisim` 分支，包含场景、脚本和实测证据。
主分支保留本入口及 Isaac 工程。

The hydrated, relative-path URDF still required a fixed-parent compatibility
variant in the tested OmniSim build. That variant imports and completes bounded
finger motion (closing error 1.803–1.859 mm). Object contact/grasp validation did
not pass: a cube/floor control failed even without R1. Joint-limit readback also
needs investigation. These results do not establish a completed grasp or hardware transfer.

兼容模型导入与夹爪小幅运动通过；物体接触、抓取和部分关节限位查询仍存在问题，
不将导入成功视为完整动力学验证通过。
