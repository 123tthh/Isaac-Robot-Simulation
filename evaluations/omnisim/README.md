# Cross-simulator validation / 跨仿真器验证

> New bounded motion check / 新增简单运动检查：
> [sim2sim](../../sim2sim/README.md) · [CLAIM](../../sim2sim/CLAIM.md).
> Fixed-base arm/finger motion passes; earlier contact limitations below remain unresolved.

## Result / 结论 — 2026-09-16

**Import and a bounded gripper cycle work with the compatibility URDF. Object
contact/grasp validation did not pass.** These tests ran separately after the
[Isaac core freeze](../../releases/2026-09-16/FREEZE.md); no additional Isaac
recording was made. This is not a matched Isaac-versus-OmniSim grasp benchmark.

**兼容 URDF 能导入，左夹爪固定开合周期通过 3 mm 端点误差阈值；物体接触与抓取
尚未验证通过。** 测试在核心冻结后独立进行，不新增 Isaac 录制，不改变冻结源码。

## URDF portability / URDF 可移植性

1. Git LFS pointers are not meshes. The local assets were hydrated; reproduction
   requires `git lfs install` and `git lfs pull`. A source ZIP alone is insufficient.
2. `r1_fixed_portable.urdf` uses repository-relative mesh paths (77 references,
   31 unique files). The ROS variant retains its package URIs.
3. Hydrated meshes and relative URLs alone did **not** resolve this OmniSim build's
   `left_base_link` registration failure. The [original failure log](results/portable_import_failure_20260915.log)
   is retained from 2026-09-15. The separate `r1_fixed_omnisim.urdf` composes fixed
   predecessor transforms into seven dynamic-joint origins and attaches those
   joints to the corresponding fixed-chain leaders. It imports successfully;
   ROS/Isaac keep their original model. This is an importer workaround, not proof
   of equivalent mass, collision or control behavior.

   Exact fatal line after mesh hydration and relative-path conversion:

   ```text
   FATAL: [newton-enforce] A joint's parent body 'left_base_link' resolved to Newton but never registered a Newton body, so the joint would be silently inert -- and Newton is the only backend, so that part of the articulation would run with no physics at all. Fix the model so the body registers with Newton.
   ```

   Asset origins and unresolved redistribution terms are recorded separately in
   the [`r1_description` provenance audit](../../projects/ros2_ws/src/robot/PROVENANCE.md).

网格与路径可移植性检查不能代替物理体注册检查：
本机补齐网格后仍复现父物理体注册错误，固定父链兼容变体才完成导入。
保留原始模型，兼容模型仅供此处测试。

## Measurements / 实测

OmniSim **8.5.1**, source commit
`0b9b07f5b646295bf4599a06e30f616b99090581`, clean source checkout;
Newton 1.5.0 / MuJoCo 3.11.0 CPU, 16 ms step. Exact dependency versions and input
hashes: [provenance](results/provenance.json).

Each variant reloads the same world and commands
`left_PGIA_joint{1,2}_motor` to 0.08 m then 0.04 m, settling 80 steps per command.
The only authored change between variants is the 0.04 m cube's initial pose;
mass remains 0.05 kg. The commanded span is 50% of the URDF range [0.04, 0.12] m.
The scene has 28 dynamic bodies including the cube and 1 registered static floor.

| Cube initial XYZ (m) | Maximum open error (mm) | Maximum close error (mm) | Both endpoints <3 mm | Target contact reported at endpoint | Object displacement (m) |
| --- | ---: | ---: | --- | --- | ---: |
| (0, 0.35, 0.8) | 2.330 | 1.843 | Yes | No | 129.789 |
| (0.05, 0.35, 0.8) | 2.345 | 1.859 | Yes | No | 108.905 |
| (0.10, 0.35, 0.8) | 2.377 | 1.803 | Yes | No | 91.200 |

Raw responses: [pose 0](results/pose_0.json), [pose 1](results/pose_1.json),
[pose 2](results/pose_2.json); [summary CSV](results/summary.csv);
[all 26 joint mappings](results/joint_mapping.csv).
Displacement is measured from the initial cube position; it is **not** error
relative to a grasp goal. The objects fell far below the floor, so no grasp
completion or physically credible object trajectory is claimed. The cube was
not positioned using a verified grasp transform. Queries sample endpoint
contacts, not a complete contact history; an empty response is not proof of no
contact throughout the motion. Each pose has one run, without repeatability statistics.

三组只改变方块初始位置，关节命令与其他参数一致。夹爪端点误差小于 3 mm，
但方块落到地面以下，不能将该结果当作抓取完成或可信的物体运动轨迹。
表中位移不是抓取目标误差；接触只查询端点，不代表全过程接触记录。

## Unresolved limits and control / 未解决问题与对照

- The [cube-only control](worlds/cube_floor_control.omniworld) removes R1.
  The load confirms **1 dynamic + 1 static body**, including the floor, yet after
  160 requested steps the reported cube Z is **−298.845 m** at reported simulation
  time 3984 ms. [Raw control result](results/cube_floor_control.json).
  Thus the observed failure does not require the R1 URDF. Its precise native
  integration/contact cause remains unconfirmed in this build. Declaring the cube
  explicitly as a Newton Robot did not resolve it.
- OmniSim warns that collision mesh scale 1.05 is ignored for two R1 meshes.
  Collision equivalence with Isaac is therefore unverified.
- The joint API reports [0, 0] for both seventh arm joints, while the URDF says
  [−6.283, 6.283] rad. That metadata discrepancy needs resolution before using
  this import for full arm range validation. Continuous wheel joints also report
  zero stops; this should not be confused with bounded URDF limits.
- No vision, compliance, payload, electronics, certification, physical transfer,
  or trained-policy evaluation was performed. No email has been sent.

移除 R1 的方块/地面对照仍失败，因此问题不限于 R1 导入。当前未确认底层精确原因。
另有碰撞网格缩放被忽略、两侧第七关节限位查询不一致的问题；这些限制均保留，
不将模型导入成功扩展为完整动力学验证通过。

## Reproduce / 复现

Install/build the pinned upstream checkout following its
[Linux bootstrap instructions](https://github.com/omnilink-tech/omnisim/tree/0b9b07f5b646295bf4599a06e30f616b99090581).
The local installation is external to this project's frozen core. Set its path:

```bash
git clone https://github.com/omnilink-tech/omnisim.git /path/to/omnisim
git -C /path/to/omnisim checkout 0b9b07f5b646295bf4599a06e30f616b99090581
export OMNISIM_HOME=/path/to/omnisim
# After installing the upstream native and Python dependencies:
python3 "$OMNISIM_HOME/scripts/harness/omnisim_harness.py" --port 6989
```

In a second terminal, from this project's root, with LFS assets present:

```bash
python3 evaluations/omnisim/scripts/run_evaluation.py --url http://127.0.0.1:6989
python3 evaluations/omnisim/scripts/run_control.py --url http://127.0.0.1:6989
python3 scripts/freeze_project.py releases/2026-09-16 --verify
```

The scripts overwrite their evaluation JSON/CSV results. Preserve this evidence
directory before rerunning. World and mesh references are relative; absolute
paths inside saved RPC responses are historical runtime evidence, not launch
requirements. 本目录重跑会更新结果，需保留基线时先复制结果目录。
