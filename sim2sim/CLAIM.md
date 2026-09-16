# Validation claim / 验证声明

**PASS: the R1 OmniSim compatibility model supports the tested fixed-base arm and
finger motions in the local OmniSim build.** This is a measured joint-motion claim,
not a claim of full sim-to-sim equivalence or successful grasping.

**通过：本地 OmniSim 中，R1 兼容模型完成了本次固定底座下的双臂和夹爪简单运动。**
此声明仅限下述被测关节与配置，不表示完整 sim2sim 等价、接触或抓取验证通过。

## Configuration and acceptance / 配置与判定

- OmniSim 8.5.1, commit `0b9b07f5b646295bf4599a06e30f616b99090581`.
- Newton 1.5.0, MuJoCo 3.11.0 CPU; 16 ms physics step; `staticBase TRUE`.
- 26 dynamic + 2 static bodies registered (the static bodies are R1 root and floor).
- Five phases: home → arm offset → arm return → fingers open → fingers close.
- Arm commands: left/right joint 1 = +0.15/−0.15 rad, both joint 4 = −0.20 rad,
  then back to zero. All four PGIA fingers command 0.04 → 0.08 → 0.04 m.
- 160 settling steps per phase; nominal 2.56 s each. The harness also advances
  during load/control operations, so this is not an exact 12.8 s episode claim.
- Arm endpoint tolerance 0.03 rad; finger endpoint tolerance 0.003 m. Actual
  excursions must exceed 0.05 rad and 0.01 m respectively; commands must not be
  clamped and joints must report position controllability.
- **5/5 phases passed**, all eight movement checks passed; reported base displacement **0 m**.

## Measurements / 实测

Errors below are maxima over the five sampled endpoints, not continuous tracking
errors. Excursion compares the outward/open endpoint with the initial home endpoint.

| Motor / 电机名 | Unit / 单位 | Actual excursion / 实际位移 | Max endpoint error / 最大端点误差 |
| --- | --- | ---: | ---: |
| left_joint1_motor | rad | 0.149544 | 0.000248 |
| right_joint1_motor | rad | 0.149553 | 0.000743 |
| left_joint4_motor | rad | 0.199597 | 0.009843 |
| right_joint4_motor | rad | 0.199596 | 0.009843 |
| left_PGIA_joint1_motor | m | 0.037638 | 0.002362 |
| left_PGIA_joint2_motor | m | 0.040000 | 0.001820 |
| right_PGIA_joint1_motor | m | 0.037638 | 0.002362 |
| right_PGIA_joint2_motor | m | 0.040000 | 0.001820 |

[Full raw responses](results/simple_motion.json) · [Summary](results/summary.json) ·
[Versions](results/provenance.json) · [URDF asset checks](results/urdf_assets.json)

Both standalone URDFs have 77 relative mesh references to 31 unique existing,
hydrated mesh files. No reference is an absolute path, ROS package URI or LFS
pointer in this check. Mesh hashes and URDF hashes are preserved in the result.

两份独立加载 URDF 的 77 个网格引用均为相对路径，31 个唯一网格已存在且不是 LFS
指针。兼容变体保留原模型，单独处理 OmniSim 固定父链注册问题；不能据此断言
缺失网格是此前注册错误的唯一原因。

## Limits / 边界

Only joints 1 and 4 on each arm and the four fingers were exercised. No mobile-base,
full seven-axis range, end-effector Cartesian accuracy, collision/grasp, vision,
LeRobot, compliance, payload or physical transfer evaluation was performed.
There is one measured sequence; repeatability statistics were not collected.
The fixed root is a test fixture, not evidence that wheel-ground contact works.

The earlier cube/floor failure, ignored collision-mesh scaling and joint-7 limit
readback discrepancy remain documented in the
[earlier evaluation](../evaluations/omnisim/README.md). This test does not resolve them.

本次不含底盘行驶、完整七轴范围、末端笛卡尔精度、接触抓取、视觉或录制功能。
之前的方块/地面异常、碰撞缩放与第七关节限位查询差异仍未解决。
