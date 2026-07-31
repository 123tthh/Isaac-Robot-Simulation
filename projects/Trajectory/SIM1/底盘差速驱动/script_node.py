import math
import time
import builtins

import carb
import omni.graph.core as og
import omni.usd
from pxr import UsdGeom, Gf


# ============================================================
# SIM1 差速底盘 ScriptNode — 圆弧转弯闭环版
# ============================================================
#
# 输入：
#   linear_velocity  ← ROS2SubscribeTwist.outputs:linearVelocity
#   angular_velocity ← ROS2SubscribeTwist.outputs:angularVelocity
#
# 输出：
#   joint_names      → IsaacArticulationController.inputs:jointNames
#   velocity_cmds    → IsaacArticulationController.inputs:velocityCommand
#
# 控制语义：
#   linear.x > 0 / < 0:
#       正常前进 / 后退
#
#   linear.x = 0 且 angular.z > 0:
#       触发一次“左转 90°圆弧运动”
#
#   linear.x = 0 且 angular.z < 0:
#       触发一次“右转 90°圆弧运动”
#
#   圆弧转弯时：
#       不再原地自转
#       而是以 ARC_RADIUS 半径前进转弯
#
# ============================================================


# ============================================================
# 1. 差速底盘几何参数
# ============================================================

# 主动轮半径，单位 m。
# 作用：把底盘线速度 m/s 转换成轮子角速度 rad/s。
WHEEL_RADIUS = 0.10

# 左右主动轮中心距，单位 m。
# 作用：差速解算时决定左右轮速度差。
WHEEL_SEPARATION = 0.55

# 最大轮速，单位 rad/s。
# 作用：防止速度过大导致 PhysX 打滑、爆炸或轮子穿模。
MAX_WHEEL_SPEED = 25


# ============================================================
# 2. 左右轮方向符号
# ============================================================

# 你已经实测：
# LEFT_SIGN = -1.0
# RIGHT_SIGN = -1.0
# 时，linear.x 正值对应机器人真实前进。
LEFT_SIGN = -1.0
RIGHT_SIGN = -1.0


# ============================================================
# 3. 普通直行参数
# ============================================================

# 直行速度增益。
# 一般保持 1.0。若 linear.x=0.5 太快，可以改外部 ROS 命令，不建议改这里。
LINEAR_GAIN = 1.0

# 普通角速度增益。
# 只有在 ENABLE_ARC_90_TURN=False 时才主要使用。
RAW_TURN_GAIN = 1.0


# ============================================================
# 4. 圆弧 90°转弯参数
# ============================================================

# 是否启用“angular.z 触发圆弧 90°闭环转弯”。
ENABLE_ARC_90_TURN = True

# 圆弧转弯读取 yaw 的 prim。
# 你现在确认 base_link 的 Z 角能反映底盘方向，所以用 base_link。
YAW_PRIM_PATH = "/World/Robot/base_link"

# 每次触发转弯的目标角度，单位 degree。
# 左转：当前 yaw + 90
# 右转：当前 yaw - 90
ARC_TARGET_DEG = 90.0

# 圆弧半径，单位 m。
# R 越小，转弯越急；R 越大，轨迹越像汽车大弯。
# 建议先用 0.5。若空间不够，可改。
ARC_RADIUS = 0.16

# 圆弧转弯线速度，单位 m/s。
# 注意：这是转弯时主动给的前进速度。
# 太小 caster 不容易摆正；太大容易打滑。
ARC_LINEAR_SPEED = 0.45

# 到达目标角阈值，单位 degree。
# 小于这个误差就认为转弯完成并停车。
ARC_DONE_EPS_DEG = 2.0

# 接近目标角时开始减速的角度范围，单位 degree。
# 例如 25°：误差小于 25° 时逐渐降低速度，减少过冲。
ARC_SLOWDOWN_DEG = 25.0

# 圆弧转弯最低速度比例。
# 防止接近目标时速度太小，被 caster 静摩擦吃掉。
ARC_MIN_SPEED_SCALE = 0.45

# 是否允许 0 命令取消正在执行的圆弧转弯。
# False 推荐：外部发 once 后，即使后续收到 0，也继续转到目标角。
ZERO_CMD_CANCELS_ARC = False


# ============================================================
# 5. 命令死区
# ============================================================

# 小于该值的 linear.x 认为是 0。
VX_DEADBAND = 1e-4

# 小于该值的 angular.z 认为是 0。
WZ_DEADBAND = 1e-4


# ============================================================
# 6. 轮子关节名
# ============================================================

LEFT_JOINT = "wheel_L_joint"
RIGHT_JOINT = "wheel_R_joint"


# ============================================================
# 7. ScriptNode 状态存储
# ============================================================

if not hasattr(builtins, "_SIM1_BASE_ARC_STATE"):
    builtins._SIM1_BASE_ARC_STATE = {
        # 是否正在执行圆弧转弯
        "arc_active": False,

        # 是否允许新的 angular.z 触发圆弧转弯
        # 防止 ROS2SubscribeTwist 保持上一条非零命令导致反复触发。
        "arc_armed": True,

        # 目标 yaw，单位 degree，范围 [-180, 180]
        "target_yaw_deg": 0.0,

        # 当前转弯方向：+1 左转，-1 右转
        "arc_dir": 0.0,

        # 帧计数
        "frame": 0,
    }


# ============================================================
# 8. 工具函数
# ============================================================

def _safe_float(x, default=0.0):
    """安全转换 float，避免 NaN / Inf / None。"""
    try:
        y = float(x)
        if math.isnan(y) or math.isinf(y):
            return default
        return y
    except Exception:
        return default


def _vec_get(v, idx, attr, default=0.0):
    """
    兼容 Isaac Sim 中 Vector3 的不同表现形式：
      1. v[0], v[1], v[2]
      2. v.x, v.y, v.z
    """
    if v is None:
        return default

    try:
        return _safe_float(v[idx], default)
    except Exception:
        pass

    try:
        return _safe_float(getattr(v, attr), default)
    except Exception:
        pass

    return default


def _clip(x, lo, hi):
    """限幅。"""
    return max(lo, min(hi, x))


def _norm_deg(a):
    """
    把角度归一化到 [-180, 180]。
    避免 yaw 从 179° 跳到 -179° 时误差计算错误。
    """
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def _get_world_yaw_deg(prim_path):
    """
    读取 prim 的世界 yaw 角，单位 degree。

    方法：
      1. 读取 LocalToWorld 矩阵；
      2. 取 prim 局部 X 轴在世界坐标中的方向；
      3. 用 atan2(y, x) 得到 yaw。

    这样比直接读 Property 面板欧拉角更稳。
    """
    try:
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return None

        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            return None

        cache = UsdGeom.XformCache()
        mat = cache.GetLocalToWorldTransform(prim)

        x_axis_world = mat.TransformDir(Gf.Vec3d(1.0, 0.0, 0.0))

        yaw_rad = math.atan2(float(x_axis_world[1]), float(x_axis_world[0]))
        yaw_deg = math.degrees(yaw_rad)

        return _norm_deg(yaw_deg)

    except Exception as e:
        carb.log_warn("[SIM1 BaseArc] yaw read failed: {}".format(e))
        return None


def _diff_drive_to_wheels(vx, wz):
    """
    差速解算。

    输入：
      vx: 底盘前进速度 m/s
      wz: 底盘 yaw 角速度 rad/s

    输出：
      left, right: 左右轮角速度 rad/s

    标准公式：
      left  = (vx - wz * L / 2) / r
      right = (vx + wz * L / 2) / r
    """
    left = (vx - wz * WHEEL_SEPARATION * 0.5) / WHEEL_RADIUS
    right = (vx + wz * WHEEL_SEPARATION * 0.5) / WHEEL_RADIUS

    left = _clip(left, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)
    right = _clip(right, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)

    left *= LEFT_SIGN
    right *= RIGHT_SIGN

    return float(left), float(right)


def _compute_arc_cmd(current_yaw_deg, st):
    """
    圆弧转弯控制。

    核心思想：
      1. 根据当前 yaw 和目标 yaw 计算角度误差；
      2. 未到目标时，持续给前进速度 vx；
      3. 同时给 wz = vx / R；
      4. 到目标角后停车。

    这不是原地旋转，而是“汽车式定半径转弯”。
    """
    err_deg = _norm_deg(st["target_yaw_deg"] - current_yaw_deg)

    if abs(err_deg) <= ARC_DONE_EPS_DEG:
        return 0.0, 0.0, err_deg, True

    # 根据误差方向决定转向方向
    turn_dir = 1.0 if err_deg > 0.0 else -1.0

    # 接近目标角时减速，减少过冲
    scale = abs(err_deg) / max(ARC_SLOWDOWN_DEG, 1e-3)
    scale = _clip(scale, ARC_MIN_SPEED_SCALE, 1.0)

    vx = ARC_LINEAR_SPEED * scale

    # 汽车式圆弧关系：wz = v / R
    wz = turn_dir * vx / ARC_RADIUS

    return float(vx), float(wz), float(err_deg), False


# ============================================================
# 9. OmniGraph 生命周期函数
# ============================================================

def setup(db):
    """ScriptNode 初始化。"""
    st = builtins._SIM1_BASE_ARC_STATE
    st["arc_active"] = False
    st["arc_armed"] = True
    st["target_yaw_deg"] = 0.0
    st["arc_dir"] = 0.0
    st["frame"] = 0

    carb.log_info("[SIM1 BaseArc] setup")
    carb.log_info("[SIM1 BaseArc] yaw prim = {}".format(YAW_PRIM_PATH))
    carb.log_info("[SIM1 BaseArc] wheel radius = {:.3f} m".format(WHEEL_RADIUS))
    carb.log_info("[SIM1 BaseArc] wheel separation = {:.3f} m".format(WHEEL_SEPARATION))
    carb.log_info("[SIM1 BaseArc] signs L/R = {:.1f}, {:.1f}".format(LEFT_SIGN, RIGHT_SIGN))
    carb.log_info("[SIM1 BaseArc] arc radius = {:.3f} m".format(ARC_RADIUS))
    carb.log_info("[SIM1 BaseArc] arc speed = {:.3f} m/s".format(ARC_LINEAR_SPEED))


def cleanup(db):
    """ScriptNode 清理：停车。"""
    try:
        db.outputs.joint_names = [LEFT_JOINT, RIGHT_JOINT]
        db.outputs.velocity_cmds = [0.0, 0.0]
        db.outputs.execOut = og.ExecutionAttributeState.ENABLED
    except Exception:
        pass

    carb.log_info("[SIM1 BaseArc] cleanup stop")


def compute(db):
    """每帧执行。"""
    st = builtins._SIM1_BASE_ARC_STATE
    st["frame"] += 1

    # ------------------------------------------------------------
    # 读取 ROS2 Twist
    # ------------------------------------------------------------
    lin = db.inputs.linear_velocity
    ang = db.inputs.angular_velocity

    raw_vx = _vec_get(lin, 0, "x", 0.0)
    raw_wz = _vec_get(ang, 2, "z", 0.0)

    if abs(raw_vx) < VX_DEADBAND:
        raw_vx = 0.0

    if abs(raw_wz) < WZ_DEADBAND:
        raw_wz = 0.0

    # ------------------------------------------------------------
    # 读取当前 yaw
    # ------------------------------------------------------------
    current_yaw_deg = _get_world_yaw_deg(YAW_PRIM_PATH)
    yaw_ok = current_yaw_deg is not None

    vx_cmd = 0.0
    wz_cmd = 0.0
    mode = "IDLE"
    err_deg = 0.0

    # ------------------------------------------------------------
    # 0 命令
    # ------------------------------------------------------------
    if raw_vx == 0.0 and raw_wz == 0.0:
        # angular.z 回零后，允许下一次转弯触发
        st["arc_armed"] = True

        if st["arc_active"]:
            if ZERO_CMD_CANCELS_ARC:
                st["arc_active"] = False
                vx_cmd = 0.0
                wz_cmd = 0.0
                mode = "ARC_CANCELLED_BY_ZERO"
            else:
                # 不取消，继续完成当前圆弧转弯
                mode = "ARC_CLOSED_LOOP"
        else:
            mode = "IDLE"
            vx_cmd = 0.0
            wz_cmd = 0.0

    # ------------------------------------------------------------
    # 普通直行/后退
    # ------------------------------------------------------------
    elif raw_vx != 0.0:
        # 只要 linear.x 非零，就退出圆弧模式，按普通速度控制
        st["arc_active"] = False

        vx_cmd = LINEAR_GAIN * raw_vx
        wz_cmd = RAW_TURN_GAIN * raw_wz
        mode = "RAW_LINEAR"

    # ------------------------------------------------------------
    # angular.z 触发圆弧 90°转弯
    # ------------------------------------------------------------
    elif raw_wz != 0.0:
        if ENABLE_ARC_90_TURN and yaw_ok:
            if (not st["arc_active"]) and st["arc_armed"]:
                direction = 1.0 if raw_wz > 0.0 else -1.0

                st["target_yaw_deg"] = _norm_deg(
                    current_yaw_deg + direction * ARC_TARGET_DEG
                )
                st["arc_dir"] = direction
                st["arc_active"] = True
                st["arc_armed"] = False

                carb.log_info(
                    "[SIM1 BaseArc] ARC START current={:.2f} deg target={:.2f} deg dir={:+.0f}".format(
                        current_yaw_deg,
                        st["target_yaw_deg"],
                        direction
                    )
                )

            mode = "ARC_CLOSED_LOOP"

        else:
            # yaw 读取失败时退回普通原始转向
            vx_cmd = 0.0
            wz_cmd = RAW_TURN_GAIN * raw_wz
            mode = "RAW_TURN"

    # ------------------------------------------------------------
    # 圆弧闭环执行
    # ------------------------------------------------------------
    if st["arc_active"] and yaw_ok:
        vx_cmd, wz_cmd, err_deg, done = _compute_arc_cmd(current_yaw_deg, st)
        mode = "ARC_CLOSED_LOOP"

        if done:
            st["arc_active"] = False
            vx_cmd = 0.0
            wz_cmd = 0.0
            mode = "ARC_DONE"

            carb.log_info(
                "[SIM1 BaseArc] ARC DONE yaw={:.2f} deg target={:.2f} deg err={:.2f} deg".format(
                    current_yaw_deg,
                    st["target_yaw_deg"],
                    err_deg
                )
            )

    # ------------------------------------------------------------
    # 差速解算并输出
    # ------------------------------------------------------------
    left, right = _diff_drive_to_wheels(vx_cmd, wz_cmd)

    db.outputs.joint_names = [LEFT_JOINT, RIGHT_JOINT]
    db.outputs.velocity_cmds = [left, right]
    db.outputs.execOut = og.ExecutionAttributeState.ENABLED

    # ------------------------------------------------------------
    # 日志
    # ------------------------------------------------------------
    if st["frame"] % 20 == 0 or mode in ("ARC_DONE", "ARC_CANCELLED_BY_ZERO"):
        if yaw_ok:
            show_err = _norm_deg(st["target_yaw_deg"] - current_yaw_deg) if st["arc_active"] else 0.0
            carb.log_info(
                "[SIM1 BaseArc] mode={} raw_vx={:.3f} raw_wz={:.3f} "
                "yaw={:.2f} target={:.2f} err={:.2f} "
                "cmd_vx={:.3f} cmd_wz={:.3f} L={:.3f} R={:.3f}".format(
                    mode,
                    raw_vx,
                    raw_wz,
                    current_yaw_deg,
                    st["target_yaw_deg"],
                    show_err,
                    vx_cmd,
                    wz_cmd,
                    left,
                    right
                )
            )
        else:
            carb.log_warn(
                "[SIM1 BaseArc] mode={} yaw=N/A raw_vx={:.3f} raw_wz={:.3f} "
                "cmd_vx={:.3f} cmd_wz={:.3f} L={:.3f} R={:.3f}".format(
                    mode,
                    raw_vx,
                    raw_wz,
                    vx_cmd,
                    wz_cmd,
                    left,
                    right
                )
            )

    return True
