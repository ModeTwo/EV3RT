"""Lap gate phase composition."""

from ..features.start_to_lap_gate import build_start_to_lap_gate


def build_lap_gate_phase(context, config):
    # LAPゲート工程はNo.2の担当ファイルだけで完結する。
    return build_start_to_lap_gate(context, config)

