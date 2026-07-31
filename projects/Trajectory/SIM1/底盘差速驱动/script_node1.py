import math
import time
import builtins

import carb
import omni.graph.core as og
import omni.usd
from pxr import UsdGeom, Gf


# ============================================================
# SIM1 差速底盘 ScriptNode — 平滑圆弧转弯闭环版
# ============================================================
#
# 目标：
#   1. linear.x 控制普通前进/后退
#   2. angular.z > 0 触发左转 90°圆弧
#   3. angular.z < 0 触发右转 90°圆弧
#   4. 不再使用原地转向，避免 caster 抖动
#
# Graph 端口要求：
#   inputs:
#     linear_velocity
#     angular_velocity
#   outputs:
#     joint_names
#     velocity_cmds
#     execOut
#
# ============================================================


# ============================================================
# 1. 底盘几何参数
# ============================================================

# 主动轮半径，单位 m
WHEEL_RADIUS = 0.10

# 左右主动轮中心距，单位 m
WHEEL_SEPARATION = 0.55

# 轮子最大角速度，单位 rad/s
# 不建议太大。25 会让 PhysX 和 caster 明显抖。
MAX_WHEEL_SPEED = 12.0


# ============================================================
# 2. 左右轮方向符号
# ============================================================

# 实测确认：两个都取 -1 时，linear.x 正值对应机器人前进
LEFT_SIGN = -1.0
RIGHT_SIGN = -1.0


# ============================================================
# 3. 普通直行参数
# ============================================================

# 直行速度增益
LINEAR_GAIN = 1.0

# 非闭环模式下的角速度增益，一般不用
RAW_TURN_GAIN = 1.0


# ============================================================
# 4. 圆弧转弯参数
# ============================================================

ENABLE_ARC_90_TURN = True

# 读取底盘 yaw 的 prim
YAW_PRIM_PATH = "/World/Robot/base_link"

# 每次触发转弯的目标角度
ARC_TARGET_DEG = 90.0

# 圆弧转弯半径，单位 m
# 关键：必须大于 WHEEL_SEPARATION / 2，否则内侧轮会反转，caster 抖动明显。
ARC_RADIUS = 0.65

# 转弯前进速度，单位 m/s
# 0.25~0.35 比较稳；0.5 容易抖。
ARC_LINEAR_SPEED = 0.28

# 到达目标的角度阈值
ARC_DONE_EPS_DEG = 3.0

# 连续多少帧满足到达条件才停车
ARC_DONE_CONFIRM_FRAMES = 6

# 接近目标多少度开始减速
ARC_SLOWDOWN_DEG = 45.0

# 减速时最低速度比例，防止太慢被静摩擦吃掉
ARC_MIN_SPEED_SCALE = 0.25

# 允许小幅过冲时直接完成，避免在目标附近反复修正
ARC_OVERSHOOT_STOP_DEG = 8.0

# 是否允许 0 命令取消转弯
# 推荐 False：发 once 后自动转完 90°
ZERO_CMD_CANCELS_ARC = False


# ============================================================
# 5. 平滑参数：抑制抖动的关键
# ============================================================

# vx 最大变化率，单位 m/s^2
# 越小越平滑，但响应越慢。
MAX_VX_ACCEL = 0.45

# wz 最大变化率，单位 rad/s^2
MAX_WZ_ACCEL = 1.00

# 单个轮子的最大角加速度，单位 rad/s^2
MAX_WHEEL_ACCEL = 18.0

# 停车时允许更大的减速度，避免滑太远
MAX_STOP_VX_ACCEL = 1.20
MAX_STOP_WZ_ACCEL = 2.50
MAX_STOP_WHEEL_ACCEL = 35.0


# ============================================================
# 6. 命令死区
# ============================================================

VX_DEADBAND = 1e-4
WZ_DEADBAND = 1e-4


# ============================================================
# 7. 关节名
# ============================================================

LEFT_JOINT = "wheel_L_joint"
RIGHT_JOINT = "wheel_R_joint"


# ============================================================
# 8. 状态存储
# ============================================================

if not hasattr(builtins, "_SIM1_BASE_ARC_SMOOTH_STATE"):
    builtins._SIM1_BASE_ARC_SMOOTH_STATE = {
        "arc_active": False,
        "arc_armed": True,
        "target_yaw_deg": 0.0,
        "arc_dir": 0.0,

        "prev_err_deg": 0.0,
        "done_count": 0,

        "last_vx": 0.0,
        "last_wz": 0.0,
        "last_left": 0.0,
        "last_right": 0.0,
        "last_time": None,

        "frame": 0,
    }


# ============================================================
# 9. 工具函数
# ============================================================

def _safe_float(x, default=0.0):
    try:
        y = float(x)
        if math.isnan(y) or math.isinf(y):
            return default
        return y
    except Exception:
        return default


def _vec_get(v, idx, attr, default=0.0):
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
    return max(lo, min(hi, x))


def _norm_deg(a):
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def _slew(current, target, max_rate, dt):
    """
    一阶斜坡限速。

    current:
      上一帧输出

    target:
      当前目标输出

    max_rate:
      最大变化率，单位为 每秒变化量

    dt:
      帧间隔，秒
    """
    max_delta = max_rate * dt
    delta = target - current

    if delta > max_delta:
        return current + max_delta
    if delta < -max_delta:
        return current - max_delta
    return target


def _get_dt(st):
    now = time.time()
    if st["last_time"] is None:
        dt = 1.0 / 60.0
    else:
        dt = now - st["last_time"]
        dt = _clip(dt, 1.0 / 240.0, 0.1)

    st["last_time"] = now
    return dt


def _get_world_yaw_deg(prim_path):
    """
    读取 prim 世界 yaw。
    使用 local X 轴在世界 XY 平面的朝向计算 yaw。
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
        carb.log_warn("[SIM1 BaseArcSmooth] yaw read failed: {}".format(e))
        return None


def _safe_arc_radius():
    """
    防止转弯半径过小。

    对差速底盘：
      如果 R < wheel_separation / 2
      内侧轮会反转，相当于接近原地转向。

    这对 4 caster 结构很容易产生抖动。
    """
    min_radius = WHEEL_SEPARATION * 0.5 + 0.10
    if ARC_RADIUS < min_radius:
        return min_radius
    return ARC_RADIUS


def _diff_drive_to_wheels_raw(vx, wz):
    """
    不做平滑，只做差速解算和轮速限幅。
    """
    left = (vx - wz * WHEEL_SEPARATION * 0.5) / WHEEL_RADIUS
    right = (vx + wz * WHEEL_SEPARATION * 0.5) / WHEEL_RADIUS

    left = _clip(left, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)
    right = _clip(right, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)

    left *= LEFT_SIGN
    right *= RIGHT_SIGN

    return float(left), float(right)


def _compute_arc_target_cmd(current_yaw_deg, st):
    """
    计算圆弧转弯目标 vx / wz。

    未到目标：
      vx = ARC_LINEAR_SPEED * scale
      wz = sign(error) * vx / radius

    到目标：
      vx = 0
      wz = 0
    """
    err_deg = _norm_deg(st["target_yaw_deg"] - current_yaw_deg)

    # 判断是否跨过目标且过冲不大
    crossed_target = (st["prev_err_deg"] * err_deg) < 0.0
    small_overshoot = abs(err_deg) <= ARC_OVERSHOOT_STOP_DEG

    if abs(err_deg) <= ARC_DONE_EPS_DEG or (crossed_target and small_overshoot):
        st["done_count"] += 1
    else:
        st["done_count"] = 0

    st["prev_err_deg"] = err_deg

    if st["done_count"] >= ARC_DONE_CONFIRM_FRAMES:
        return 0.0, 0.0, err_deg, True

    turn_dir = 1.0 if err_deg > 0.0 else -1.0

    # 接近目标时减速
    scale = abs(err_deg) / max(ARC_SLOWDOWN_DEG, 1e-3)
    scale = _clip(scale, ARC_MIN_SPEED_SCALE, 1.0)

    vx = ARC_LINEAR_SPEED * scale

    radius = _safe_arc_radius()
    wz = turn_dir * vx / radius

    return float(vx), float(wz), float(err_deg), False


# ============================================================
# 10. OmniGraph 生命周期函数
# ============================================================

def setup(db):
    st = builtins._SIM1_BASE_ARC_SMOOTH_STATE

    st["arc_active"] = False
    st["arc_armed"] = True
    st["target_yaw_deg"] = 0.0
    st["arc_dir"] = 0.0

    st["prev_err_deg"] = 0.0
    st["done_count"] = 0

    st["last_vx"] = 0.0
    st["last_wz"] = 0.0
    st["last_left"] = 0.0
    st["last_right"] = 0.0
    st["last_time"] = None

    st["frame"] = 0

    carb.log_info("[SIM1 BaseArcSmooth] setup")
    carb.log_info("[SIM1 BaseArcSmooth] yaw prim = {}".format(YAW_PRIM_PATH))
    carb.log_info("[SIM1 BaseArcSmooth] wheel radius = {:.3f}".format(WHEEL_RADIUS))
    carb.log_info("[SIM1 BaseArcSmooth] wheel separation = {:.3f}".format(WHEEL_SEPARATION))
    carb.log_info("[SIM1 BaseArcSmooth] signs L/R = {:.1f}, {:.1f}".format(LEFT_SIGN, RIGHT_SIGN))
    carb.log_info("[SIM1 BaseArcSmooth] requested radius = {:.3f}, safe radius = {:.3f}".format(
        ARC_RADIUS, _safe_arc_radius()
    ))
    carb.log_info("[SIM1 BaseArcSmooth] arc speed = {:.3f}".format(ARC_LINEAR_SPEED))


def cleanup(db):
    try:
        db.outputs.joint_names = [LEFT_JOINT, RIGHT_JOINT]
        db.outputs.velocity_cmds = [0.0, 0.0]
        db.outputs.execOut = og.ExecutionAttributeState.ENABLED
    except Exception:
        pass

    carb.log_info("[SIM1 BaseArcSmooth] cleanup stop")


def compute(db):
    st = builtins._SIM1_BASE_ARC_SMOOTH_STATE
    st["frame"] += 1

    dt = _get_dt(st)

    # ------------------------------------------------------------
    # 读取 ROS2 Twist
    # ------------------------------------------------------------
    raw_vx = _vec_get(db.inputs.linear_velocity, 0, "x", 0.0)
    raw_wz = _vec_get(db.inputs.angular_velocity, 2, "z", 0.0)

    if abs(raw_vx) < VX_DEADBAND:
        raw_vx = 0.0
    if abs(raw_wz) < WZ_DEADBAND:
        raw_wz = 0.0

    # ------------------------------------------------------------
    # 读取 yaw
    # ------------------------------------------------------------
    current_yaw_deg = _get_world_yaw_deg(YAW_PRIM_PATH)
    yaw_ok = current_yaw_deg is not None

    target_vx = 0.0
    target_wz = 0.0
    mode = "IDLE"
    err_deg = 0.0
    arc_done_now = False

    # ------------------------------------------------------------
    # 0 命令
    # ------------------------------------------------------------
    if raw_vx == 0.0 and raw_wz == 0.0:
        st["arc_armed"] = True

        if st["arc_active"]:
            if ZERO_CMD_CANCELS_ARC:
                st["arc_active"] = False
                st["done_count"] = 0
                target_vx = 0.0
                target_wz = 0.0
                mode = "ARC_CANCELLED_BY_ZERO"
            else:
                mode = "ARC_CLOSED_LOOP"
        else:
            target_vx = 0.0
            target_wz = 0.0
            mode = "IDLE"

    # ------------------------------------------------------------
    # 普通直行/后退
    # ------------------------------------------------------------
    elif raw_vx != 0.0:
        st["arc_active"] = False
        st["done_count"] = 0

        target_vx = LINEAR_GAIN * raw_vx
        target_wz = RAW_TURN_GAIN * raw_wz
        mode = "RAW_LINEAR"

    # ------------------------------------------------------------
    # angular.z 触发圆弧转弯
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
                st["done_count"] = 0
                st["prev_err_deg"] = _norm_deg(st["target_yaw_deg"] - current_yaw_deg)

                carb.log_info(
                    "[SIM1 BaseArcSmooth] ARC START current={:.2f} target={:.2f} dir={:+.0f}".format(
                        current_yaw_deg,
                        st["target_yaw_deg"],
                        direction
                    )
                )

            mode = "ARC_CLOSED_LOOP"

        else:
            target_vx = 0.0
            target_wz = RAW_TURN_GAIN * raw_wz
            mode = "RAW_TURN"

    # ------------------------------------------------------------
    # 圆弧闭环
    # ------------------------------------------------------------
    if st["arc_active"] and yaw_ok:
        target_vx, target_wz, err_deg, done = _compute_arc_target_cmd(current_yaw_deg, st)
        mode = "ARC_CLOSED_LOOP"

        if done:
            st["arc_active"] = False
            target_vx = 0.0
            target_wz = 0.0
            mode = "ARC_DONE"
            arc_done_now = True

            carb.log_info(
                "[SIM1 BaseArcSmooth] ARC DONE yaw={:.2f} target={:.2f} err={:.2f}".format(
                    current_yaw_deg,
                    st["target_yaw_deg"],
                    err_deg
                )
            )

    # ------------------------------------------------------------
    # 平滑 vx / wz
    # ------------------------------------------------------------
    stopping = (target_vx == 0.0 and target_wz == 0.0)

    vx_acc = MAX_STOP_VX_ACCEL if stopping else MAX_VX_ACCEL
    wz_acc = MAX_STOP_WZ_ACCEL if stopping else MAX_WZ_ACCEL

    vx_cmd = _slew(st["last_vx"], target_vx, vx_acc, dt)
    wz_cmd = _slew(st["last_wz"], target_wz, wz_acc, dt)

    st["last_vx"] = vx_cmd
    st["last_wz"] = wz_cmd

    # ------------------------------------------------------------
    # 差速解算
    # ------------------------------------------------------------
    target_left, target_right = _diff_drive_to_wheels_raw(vx_cmd, wz_cmd)

    wheel_acc = MAX_STOP_WHEEL_ACCEL if stopping else MAX_WHEEL_ACCEL

    left = _slew(st["last_left"], target_left, wheel_acc, dt)
    right = _slew(st["last_right"], target_right, wheel_acc, dt)

    st["last_left"] = left
    st["last_right"] = right

    # ------------------------------------------------------------
    # 输出
    # ------------------------------------------------------------
    db.outputs.joint_names = [LEFT_JOINT, RIGHT_JOINT]
    db.outputs.velocity_cmds = [float(left), float(right)]
    db.outputs.execOut = og.ExecutionAttributeState.ENABLED

    # ------------------------------------------------------------
    # 日志
    # ------------------------------------------------------------
    if st["frame"] % 20 == 0 or arc_done_now:
        if yaw_ok:
            show_err = _norm_deg(st["target_yaw_deg"] - current_yaw_deg) if st["arc_active"] else 0.0
            carb.log_info(
                "[SIM1 BaseArcSmooth] mode={} raw_vx={:.3f} raw_wz={:.3f} "
                "yaw={:.2f} target={:.2f} err={:.2f} "
                "target_vx={:.3f} target_wz={:.3f} "
                "cmd_vx={:.3f} cmd_wz={:.3f} "
                "L={:.3f} R={:.3f}".format(
                    mode,
                    raw_vx,
                    raw_wz,
                    current_yaw_deg,
                    st["target_yaw_deg"],
                    show_err,
                    target_vx,
                    target_wz,
                    vx_cmd,
                    wz_cmd,
                    left,
                    right
                )
            )
        else:
            carb.log_warn(
                "[SIM1 BaseArcSmooth] yaw=N/A mode={} raw_vx={:.3f} raw_wz={:.3f} "
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
