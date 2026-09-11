"""Course bearings for ET sumo only: up=0, right=90, clockwise positive."""

import math
from dataclasses import dataclass
from typing import Optional


def normalize_bearing(value):
    # 方位角はコース図の上を0度として0以上360未満へそろえる。
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("Bearing must be finite")
    return value % 360.0


def bearing_delta(target, current):
    # 旋回量は方位角と区別し、最短方向の符号付き角度差として扱う。
    return (normalize_bearing(target) - normalize_bearing(current) + 180.0) % 360.0 - 180.0


@dataclass
class SumoBearingReference:
    # センサー自身や他工程の角度定義は変更せず、相撲専用の対応関係を保持する。
    reset_bearing_deg: Optional[float] = None
    reset_gyro_deg: Optional[float] = None

    def register(self, bearing, gyro):
        self.reset_bearing_deg = normalize_bearing(bearing)
        if not math.isfinite(float(gyro)):
            raise ValueError("Gyro angle must be finite")
        self.reset_gyro_deg = float(gyro)

    def bearing(self, gyro):
        if self.reset_bearing_deg is None or self.reset_gyro_deg is None:
            raise RuntimeError("Sumo bearing reference is not registered after device reset")
        # 既存SPIKE制御と同じ取付前提：生ジャイロの増加は物理的な時計回り。
        return normalize_bearing(self.reset_bearing_deg + float(gyro) - self.reset_gyro_deg)

    def legacy_target(self, bearing, gyro, course):
        if course not in (-1, 1):
            raise ValueError("Course must be +1 or -1")
        # 現在値に近い連続ジャイロ目標へ変換し、既存Behaviorの-course補正へ渡す。
        raw_target = float(gyro) + bearing_delta(bearing, self.bearing(gyro))
        return -course * raw_target


def choose_search_bearing(current, offset=50.0, garage=180.0, prefer_clockwise=False):
    # 現在方位±50度のうち、ガレージ方位から離れる候補を採用する。
    plus = normalize_bearing(current + abs(offset))
    minus = normalize_bearing(current - abs(offset))
    plus_gap = abs(bearing_delta(plus, garage))
    minus_gap = abs(bearing_delta(minus, garage))
    if math.isclose(plus_gap, minus_gap, abs_tol=1e-9):
        return plus if prefer_clockwise else minus
    return plus if plus_gap > minus_gap else minus


def initial_sumo_bearing(mission, override=None):
    # 単体だけ配置方向を指定できる。通し走行は全体スタートの下向き180度を登録する。
    if override is not None and mission != "sumo":
        raise ValueError("--sumo-initial-bearing is available only with --mission sumo")
    return normalize_bearing(override if override is not None else (0.0 if mission == "sumo" else 180.0))
