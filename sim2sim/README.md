# R1 sim2sim checks / 跨仿真器检查

**Current public OmniSim follow-up:** [joint 7, original URDF import and contact probes](followup/README.md) compare v8.5.1 with v9.0.0-rc.2. The simple-motion claim below is the earlier v8.5.1 baseline.

**当前公开版复核：**[第七关节、原始 URDF 导入与接触探针](followup/README.md)对比了 v8.5.1 和 v9.0.0-rc.2；下述简单运动结果属于旧版基线。

[Validation claim / 验证声明](CLAIM.md)

**Current result: simple fixed-base motion PASS.** The folder name describes the
cross-simulator workflow; this run validates only the OmniSim side, not a paired
Isaac/OmniSim equivalence benchmark. No training dataset or video was recorded.

**当前结果：固定底座简单运动通过。** 本目录用于跨仿真器工作；此次只验证 OmniSim
侧的简单运动，没有开展两引擎配对等价评估，也没有录制数据或视频。

![Native OmniSim R1 screenshot](images/omnisim_r1_motion.png)

Native viewport captured after the test using a separately commanded pose; the
image is illustrative, while JSON records establish the movement claim.
截图为测试后另行设置姿态的原生画面；运动判定以 JSON 实测记录为依据。

## Files / 文件

| Path | Purpose / 用途 |
| --- | --- |
| `worlds/r1_simple_motion.omniworld` | Relative URDF reference and fixed-base test fixture |
| `scripts/check_urdf_assets.py` | Relative paths, mesh hydration and input hashes |
| `scripts/check_simple_motion.py` | Measured arm/finger command sequence and assertions |
| `results/simple_motion.json` | Complete native API responses and acceptance checks |
| `results/summary.json` | Compact movement and error measurements |
| `results/provenance.json` | Engine and dependency versions |
| `results/urdf_assets.json` | Mesh/URDF integrity records |
| `CLAIM.md` | Bounded claim and unresolved limitations |

URDFs are maintained at
[`projects/ros2_ws/src/robot/urdf/`](../projects/ros2_ws/src/robot/urdf):
`r1_fixed_portable.urdf` for standalone relative paths and `r1_fixed_omnisim.urdf`
for the tested OmniSim fixed-parent compatibility workaround.

## Reproduce / 复现

Use an installed/built OmniSim checkout at commit
`0b9b07f5b646295bf4599a06e30f616b99090581`, with dependencies listed in
[provenance](results/provenance.json). From the project root:

```bash
git lfs install
git lfs pull
python3 sim2sim/scripts/check_urdf_assets.py
export OMNISIM_HOME=/path/to/omnisim
python3 "$OMNISIM_HOME/scripts/harness/omnisim_harness.py" --port 6989
```

In another terminal / 另一个终端：

```bash
python3 sim2sim/scripts/check_simple_motion.py --url http://127.0.0.1:6989
```

Optional native screenshot after loading the world / 加载场景后可截图：

```bash
python3 sim2sim/scripts/capture_screenshot.py
```

The script exits nonzero if a measured acceptance check fails. It writes diagnostic
JSON only. Preserve the existing `results/` before rerunning if needed. The engine
installation is external; project URDF/mesh/world references are relative. Absolute
paths inside saved native responses record the tested host, not required installation paths.
