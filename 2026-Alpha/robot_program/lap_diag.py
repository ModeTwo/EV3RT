"""2026-11-17(ユーザー、LAP 制作者の話): しばらく充電・放置したあとの1回目だけ LAP の動きが想定と違う理由を調べるログ。
走りは変えない。タッチからラップゲートまでの間だけ、0.2秒ごと(ODOMETRY_SAMPLE と同じ間隔)に
    LAP_DIAG t=<タッチからの秒> battery_mv=.. battery_ma=.. right_power=.. left_power=.. dist=..
を出す(車輪の回転の速さは直前の ODOMETRY_SAMPLE の right_rpm/left_rpm)。タッチ時とラップゲート時に電池の値を1行ずつ出す。
候補: 電池の電圧(高いと同じ出力でも速い)、ギア・モーターの冷え(電圧と関係なく1回目だけ遅い)。
"""
import time

from .runtime import runtime

ACTIVE = False
_T0 = None
_LAST = {"right": None, "left": None}
_PATCHED = False


def _battery():
    try:
        return int(runtime.hub.get_battery_voltage()), int(runtime.hub.get_battery_current())
    except Exception:
        return -1, -1


def _patch(motor, side):
    if motor is None or getattr(motor, "_lap_diag_patched", False):
        return
    orig = motor.set_power

    def set_power(power, *args, **kwargs):
        _LAST[side] = power
        return orig(power, *args, **kwargs)

    try:
        motor.set_power = set_power
        motor._lap_diag_patched = True
    except Exception:
        pass


def start():
    """タッチで呼ぶ(pre_touch_gyro.PreTouchGyroDrift.terminate)。"""
    global ACTIVE, _T0, _PATCHED
    if ACTIVE:
        return
    _T0 = time.monotonic()
    if not _PATCHED:
        _patch(runtime.right_motor, "right")
        _patch(runtime.left_motor, "left")
        _PATCHED = True
    mv, ma = _battery()
    print("LAP_DIAG_BATT at=touch battery_mv=%d battery_ma=%d" % (mv, ma), flush=True)
    ACTIVE = True


def stop(at="lap gate"):
    """ラップゲートのあと(gyro_scale.ensure_scaled_gyro で倍率補正を有効にした時点)で呼ぶ。"""
    global ACTIVE
    if not ACTIVE:
        return
    ACTIVE = False
    mv, ma = _battery()
    print("LAP_DIAG_BATT at=%s t=%.2f battery_mv=%d battery_ma=%d" % (at, time.monotonic() - _T0, mv, ma), flush=True)


def sample(distance_mm):
    """plotter の ODOMETRY_SAMPLE のたびに呼ばれる(同じスレッド)。"""
    if not ACTIVE:
        return
    if time.monotonic() - _T0 > 60.0:   # ラップゲートで止まらなかったとき(倍率補正を切った較正など)の保険
        stop("timeout 60s")
        return
    mv, ma = _battery()
    r, l = _LAST["right"], _LAST["left"]
    print("LAP_DIAG t=%.2f battery_mv=%d battery_ma=%d right_power=%s left_power=%s dist=%.0f" % (
        time.monotonic() - _T0, mv, ma, "-" if r is None else "%+d" % int(r), "-" if l is None else "%+d" % int(l), distance_mm),
        flush=True)
