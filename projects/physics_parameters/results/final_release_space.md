# Safe Release Pose Set

Generated: 2026-04-23T11:30:43.612766

- Rate threshold: 0.8
- Conservative shrink: 0.6

## Notes
- Y/Yaw 搜索含锯齿/孤立点，robust 区间比 naive 更保守
- Pitch/Mass/μ 搜索全域成功，已按实际工况 clip
- Z 是 Pitch 的函数（托盘几何贴合补偿）
- Roll 固定为 0°（保持水平）
- X 固定为 0.045m（输送机入口）
- sim2real: Y 额外收缩 50% (真机 μ_roller > sim)
- sim2real: Pitch clip 到 15.0°
- sim2real: Yaw 额外收缩 70%

## Per-dimension Analysis

### Y (mm)
- Points scanned: 51
- Full scan range: [-25.00, 25.00] mm
- Naive safe (from JSON): [-20.00, 21.00] mm
- Robust contiguous: [-20.00, 21.00] mm
- Conservative (center 60%): [-11.80, 12.80] mm
- Failure breakdown: success=230, rail_lock=25, stuck=0

### Yaw (deg)
- Points scanned: 5
- Full scan range: [-15.00, -11.00] deg
- Naive safe (from JSON): None
- Robust contiguous: None
- Conservative (center 60%): None
- Failure breakdown: success=0, rail_lock=13, stuck=0

### Pitch (deg)
- Points scanned: 22
- Full scan range: [4.00, 25.00] deg
- Naive safe (from JSON): [4.00, 25.00] deg
- Robust contiguous: [4.00, 25.00] deg
- Conservative (center 60%): [8.20, 20.80] deg
- Failure breakdown: success=66, rail_lock=0, stuck=0

### Mass (kg)
- Points scanned: 14
- Full scan range: [0.70, 2.00] kg
- Naive safe (from JSON): [0.70, 2.00] kg
- Robust contiguous: [0.70, 2.00] kg
- Conservative (center 60%): [0.96, 1.74] kg

### mu_roller_s ()
- Points scanned: 17
- Full scan range: [0.02, 0.10]
- Naive safe (from JSON): [0.02, 0.10]
- Robust contiguous: [0.02, 0.10]
- Conservative (center 60%): [0.04, 0.08]

## Final Release Pose Sets

### ★ Sim2Real Recommended (real deployment)
| Dim | Range | Unit |
|---|---|---|
| X | 0.045 | m (fixed) |
| Y | [-5.6, 6.6] | mm |
| Z | [1.2620, 1.2732] | m (fn of pitch) |
| Roll | 0.0 | ° (fixed) |
| Pitch | [8.2, 15.0] | ° |
| Yaw | None | ° |
| Mass | [0.96, 1.74] | kg |
| μ_roller_s | [0.036, 0.084] | - |

### Conservative (sim training core)
| Dim | Range | Unit |
|---|---|---|
| X | 0.045 | m (fixed) |
| Y | [-11.8, 12.8] | mm |
| Z | [1.2620, 1.2832] | m (fn of pitch) |
| Roll | 0.0 | ° (fixed) |
| Pitch | [8.2, 20.8] | ° |
| Yaw | None | ° |
| Mass | [0.96, 1.74] | kg |
| μ_roller_s | [0.036, 0.084] | - |

### Nominal (feasible region)
| Dim | Range | Unit |
|---|---|---|
| X | 0.045 | m (fixed) |
| Y | [-20.0, 21.0] | mm |
| Z | [1.2552, 1.2907] | m (fn of pitch) |
| Roll | 0.0 | ° (fixed) |
| Pitch | [4.0, 25.0] | ° |
| Yaw | None | ° |
| Mass | [0.70, 2.00] | kg |
| μ_roller_s | [0.020, 0.100] | - |

### Full (curriculum outermost)
| Dim | Range | Unit |
|---|---|---|
| X | 0.045 | m (fixed) |
| Y | [-25.0, 25.0] | mm |
| Z | [1.2552, 1.2907] | m (fn of pitch) |
| Roll | 0.0 | ° (fixed) |
| Pitch | [4.0, 25.0] | ° |
| Yaw | [-15.0, -11.0] | ° |
| Mass | [0.70, 2.00] | kg |
| μ_roller_s | [0.020, 0.100] | - |