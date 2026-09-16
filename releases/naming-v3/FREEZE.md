# Naming and portability release

Current project entry: `./run.sh`. Documentation: [English](../../docs/overview.md)
and [中文](../../docs/overview.zh-CN.md).

Project-owned active filenames no longer use scene dates or historical scene
numbers. Scene layers live in `assets/scenes/r1_workcell/`, with descriptive
`base.usd`, `physics.usd`, `robot.usd` and `sensors.usd` configuration layers.
Teleoperation lives in `projects/teleoperation/sim1/`; physics studies live in
`projects/physics_parameters/`. Embedded USD references and launch paths were
updated together. Historical reports and previous releases retain evidence identifiers.
Third-party package/API names and USD prim names retain their original contracts.

本版继续规范场景、脚本和模块命名；同步修正 USD 内部引用，而非仅在文件系统更名。
历史证据与旧快照保留原版本标识，旧清单不代表当前工作树。

## Verification / 验证

- 110 project Python files parse; shell syntax and layout checks pass.
- Relocated entrypoint works under a path containing spaces.
- 36 physics script path resolvers pass file and Script Editor modes.
- Scene resolves 494 prims with Z-up and 1 metre units; no missing renamed USD layers.
- Isaac Sim headless startup reaches READY with ROS Bridge and six camera products;
  Play advances to at least 487 frames / 8.117 simulation seconds during the short run,
  and the process exits normally. No recording was started.
- Existing v2.1 and v3.0 exports still pass local checks after moving their directory.

[Verification details](../../reports/naming_validation.md).

```bash
python3 scripts/freeze_project.py releases/naming-v3 --verify
(cd releases/naming-v3 && sha256sum -c SHA256SUMS)
```

Source and evidence archives are local release artifacts; Git publishes manifests
and source files directly, with binary model assets through Git LFS. Raw recordings,
local virtual environments, build products and test-output directories are excluded.
The independent OmniSim scripts and evidence are published on `develop/omnisim`;
the default branch provides an index to that branch.

冻结不宣称已完成新的抓取验证。自动底盘修正未重新录制、官方 LeRobot 训练加载器
未运行和 OmniSim 接触异常的既有边界保持不变。
