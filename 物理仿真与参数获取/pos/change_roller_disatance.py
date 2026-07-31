from pxr import UsdGeom, Gf
import omni.usd

stage = omni.usd.get_context().get_stage()

# ════════════════════════════════════════
# ★ 可调整参数
# ════════════════════════════════════════
ROLLER_P_NEW = 0.025   # ★ 新轴心间距（米），当前25mm，按需修改

ANGLE_DEG    = 4.0     # ★ 坡度角（与场景一致）
ROLLER_R     = 0.010   # ★ 滚轮半径（与场景一致）

import math
AR = math.radians(ANGLE_DEG)

# ════════════════════════════════════════
# 读取 Roller_00_L/R 当前位置作为基准
# ════════════════════════════════════════
ref_L = stage.GetPrimAtPath("/World/Conveyor/Roller_00_L")
ref_R = stage.GetPrimAtPath("/World/Conveyor/Roller_00_R")

if not ref_L.IsValid() or not ref_R.IsValid():
    print("❌ Roller_00_L 或 Roller_00_R 不存在，请检查路径")
else:
    # 读取基准位置
    t_L = ref_L.GetAttribute("xformOp:translate").Get()
    t_R = ref_R.GetAttribute("xformOp:translate").Get()

    base_x  = t_L[0]   # 基准X（两者应相同）
    base_yL = t_L[1]   # 左轮Y（负值）
    base_yR = t_R[1]   # 右轮Y（正值）
    base_z  = t_L[2]   # 基准Z

    print(f"基准 Roller_00_L : ({t_L[0]:.4f}, {t_L[1]:.4f}, {t_L[2]:.4f})")
    print(f"基准 Roller_00_R : ({t_R[0]:.4f}, {t_R[1]:.4f}, {t_R[2]:.4f})")
    print(f"新轴心间距       : {ROLLER_P_NEW*1000:.1f}mm")
    print()

    # ════════════════════════════════════
    # 更新 Roller_00~08 的X和Z坐标
    # Y坐标保持不变（左右对称不受间距影响）
    # ════════════════════════════════════
    ROLLER_N = 9

    for i in range(ROLLER_N):
        # 新坐标：以Roller_00为基准，沿坡面方向累加
        new_x = base_x + i * ROLLER_P_NEW
        new_z = base_z - i * ROLLER_P_NEW * math.sin(AR)

        for side, base_y in [("L", base_yL), ("R", base_yR)]:
            path  = f"/World/Conveyor/Roller_{i:02d}_{side}"
            prim  = stage.GetPrimAtPath(path)

            if not prim.IsValid():
                print(f"⚠ {path} 不存在，跳过")
                continue

            # 只更新X和Z，Y保持原值
            old_t = prim.GetAttribute("xformOp:translate").Get()
            new_t = Gf.Vec3d(new_x, old_t[1], new_z)
            prim.GetAttribute("xformOp:translate").Set(new_t)

            print(f"  Roller_{i:02d}_{side}: "
                  f"X {old_t[0]:.4f}→{new_x:.4f}  "
                  f"Z {old_t[2]:.4f}→{new_z:.4f}  "
                  f"Y={old_t[1]:.4f}(不变)")

    total_len = (ROLLER_N - 1) * ROLLER_P_NEW
    print(f"\n✓ 完成")
    print(f"  滚轮覆盖长度: {total_len*1000:.1f}mm"
          f"（Roller_00 到 Roller_08）")
    print(f"  末端Z下降  : {total_len*math.sin(AR)*1000:.2f}mm")