# Core freeze / 核心冻结 — 2026-09-16

Frozen core: **2,912 files**. `core_manifest.json` records SHA-256 for source,
configuration and assets. `source.tar.gz` is a compact source snapshot (large
binary assets are omitted; their hashes remain in the manifest). `SHA256SUMS`
checks the manifest and source archive. This extracted workspace has no usable
Git metadata, so this is a content freeze rather than a fabricated Git tag.

已冻结 2,912 个核心文件。源码压缩包不重复打包大型二进制资产；资产校验值
保留在清单中。当前目录不是有效 Git 检出，不伪造 commit/tag。

From repository root / 在项目根目录校验：

```bash
python3 scripts/freeze_project.py releases/2026-09-16 --verify
(cd releases/2026-09-16 && sha256sum -c SHA256SUMS)
```

Current evidence / 当前证据：
[validation](../../reports/VALIDATION_20260916.md),
[recording metrics](../../reports/recording_validation_20260916.json),
[LeRobot export checks](../../reports/lerobot_export_validation_20260916.json).

Runtime data, build/install products, local dependency overlays, reports and
cross-simulator evaluations are outside the core freeze. The separate OmniSim
folder may add results without changing these core hashes. README and operator
index may link the new evidence. The small automatic-base handler correction was
not re-recorded after the user's stop instruction; that limitation remains in the
validation report. Official training-loader and hardware transfer are unverified.

运行数据、构建产物、本地依赖、报告和独立跨仿真器测试不属于核心清单。
新增测试结果不修改核心校验值。自动底盘段最后的修正未新增录制复测，边界见报告。

## Final documentation snapshot / 最终文档快照

After the separate evaluation, `documentation_manifest.json` and
`documentation.tar.gz` freeze the project Markdown, reports and
`evaluations/omnisim/` evidence as a supplementary snapshot. `SHA256SUMS` covers
both core and documentation artifacts. These records preserve the unresolved
OmniSim contact failure; they do not promote it to a passing result.

独立测试结束后，补充文档清单与归档，保存中英文说明、验证报告和 OmniSim 证据。
冻结不等于所有测试通过，未解决事项保留在报告中。
