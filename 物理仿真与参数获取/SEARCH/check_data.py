# 运行这个确认所有参数正确
import math
print("=== 搜索参数确认 ===")
print(f"托盘基准位置  : X={0.045:.4f} Y={0.00253:.5f} Z={1.26751:.5f}")
print(f"托盘尺寸      : X={0.17999*1000:.1f} Y={0.25782*1000:.1f} H={0.09729*1000:.1f}mm")
print(f"输送带末端X   : {0.500:.3f}m")
print(f"成功判定X     : {0.500*0.9:.3f}m")
print(f"等效μ基准     : {0.060*0.40:.4f}")
print(f"tan(4°)       : {math.tan(math.radians(4)):.4f}")
print(f"下滑条件      : {0.060*0.40:.4f} < {math.tan(math.radians(4)):.4f} = "
      f"{'✓满足' if 0.060*0.40 < math.tan(math.radians(4)) else '✗不满足'}")
print(f"\n网格搜索规模  : 7×7 × 5次 = 245次仿真")
print(f"预计时间      : ~{245*5/60:.0f}分钟")