# Layout v2 freeze / 工程整理第二版冻结

This supersedes the working-tree layout of the first 2026-09-16 freeze. Earlier
release archives and checksums remain unchanged historical snapshots; the old
core manifest is not expected to match the renamed/updated working tree.
本版本按用户要求整理目录、介绍文档和入口；此前快照保持原样，不覆盖旧校验值。

## Changes / 变更

| Previous / 原位置 | Current / 当前位置 |
| --- | --- |
| `项目交接文档.md` | Replaced by `docs/overview.md` and `docs/overview.zh-CN.md` |
| `物理仿真与参数获取/` | `projects/physics_parameters/` |
| `README_CN.md` | `README.zh-CN.md` |
| `docs/OPERATIONS_EN.md`, `docs/OPERATIONS_ZH.md` | `docs/getting-started.md`, `docs/getting-started.zh-CN.md` |
| Dated document index | `docs/README.md` |
| Root guide aliases and old handover notes | `docs/archive/` |
| Dated launcher implementation | `scripts/project_control.sh`; old command forwards |
| Public command | `./run.sh` |

Physics experiment directories use `search_v0`–`search_v8`, `scene_tools`, and
`results`. Desktop output paths now resolve from the project root. Robot legacy
mesh paths are relative to their URDF. External simulator installation remains
configured through `ISAAC_SIM_ROOT`; ROS/system/container and USD namespaces are
intentional. Build products, local virtual environments and raw data are not portable
source deliverables and are excluded from the freeze.

参数脚本移除作者桌面依赖；旧 URDF 中的本机网格路径改为文件相对路径。
ROS 工作区迁移后需重编。研究脚本此次只验证路径和语法，没有重跑参数搜索。

## Checks / 校验

- Relocated launcher works from another working directory and a path with spaces.
- 36 physics script resolvers pass file-based and Script Editor environment modes.
- Shell syntax and project layout checks pass.
- Existing capture and OmniSim conclusions retain their original limits; no new
  recording or dynamics validation was performed.

[Machine-readable path checks](../../reports/layout_portability_20260916.json),
[recording validation](../../reports/VALIDATION_20260916.md),
[OmniSim evidence](../../evaluations/omnisim/README.md).

```bash
python3 scripts/verify_portability.py
bash scripts/verify_project_layout.sh
python3 scripts/freeze_project.py releases/2026-09-16-layout-v2 --verify
(cd releases/2026-09-16-layout-v2 && sha256sum -c SHA256SUMS)
```

`core_manifest.json` includes runtime source, physics experiments, current docs,
Docker definitions, tests, assets and public entrypoints. `source.tar.gz` is a compact
source snapshot: binary/large assets remain separately distributed and hashed.
`evidence.tar.gz` and `evidence_manifest.json` preserve reports and the separate
OmniSim evaluation. This is a local content freeze, not a Git tag or published release.
