"""今回の相撲ミッションの実行順を表すビヘイビアツリー。

alphaが機器初期化・タッチ開始・制御周期を担当する。各子ノードは同じ
制御器を共有し、自分の工程だけを1周期ずつ進める。録画や制動の所有者は
従来の制御器のままにし、工程の切替時にカメラ世代を作り直さない。
"""
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence

from .sumo_push_route import build_sumo_push_route


# 状態名は既存のCSV/summaryと互換。較正なし・較正ありで共通の工程に対応する。
PHASE_STAGE = {
    "RECORD_START": 0,
    "DRIVE": 0,
    "SETTLE": 0,
    "OBSERVE": 0,
    "TURN": 0,
    "NC_CAPTURE": 1,
    "NC_CAPTURE_SETTLE": 1,
    "ROUTE_CAPTURE": 1,
    "ROUTE_CAPTURE_SETTLE": 1,
    "NC_TURN": 2,
    "NC_TURN_SETTLE": 2,
    "ROUTE_TURN": 2,
    "ROUTE_TURN_SETTLE": 2,
    "NC_PUSH": 3,
    "NC_BACK": 3,
    "NC_BACK_TURN": 3,
    "NC_BACK_APPROACH": 3,
    "NC_DONE": 3,
    "ROUTE_PUSH": 3,
    "ROUTE_HOLD": 3,
}


class SumoPushStage(Behaviour):
    """一工程の子ノード。工程が切り替わったら次へ制御を渡す。"""

    def __init__(self, name, tree, stage):
        super().__init__(name)
        self.tree = tree
        self.stage = stage

    def update(self):
        engine = self.tree.engine
        if self.tree.engine_started and engine.status == Status.FAILURE:
            return Status.FAILURE
        if self.tree.engine_started and engine.status == Status.SUCCESS:
            return Status.SUCCESS

        phase = getattr(engine, "phase", None) if self.tree.engine_started else None
        current_stage = PHASE_STAGE.get(phase)
        if current_stage is not None and current_stage > self.stage:
            # 例: 戻り旋回が不要なら、旋回ノードを成功で通過する。
            return Status.SUCCESS
        if current_stage is not None and current_stage < self.stage:
            # 実行順が逆行した場合に後続のモーター出力を出さない。
            return Status.FAILURE

        if not self.tree.engine_ticked_this_cycle:
            self.tree.engine_ticked_this_cycle = True
            self.tree.engine_started = True
            self.tree.tick_engine()

        if engine.status == Status.FAILURE:
            return Status.FAILURE
        if engine.status == Status.SUCCESS:
            return Status.SUCCESS
        current_stage = PHASE_STAGE.get(getattr(engine, "phase", None))
        if current_stage is not None and current_stage > self.stage:
            return Status.SUCCESS
        return Status.RUNNING


class SumoPushSequence(Sequence):
    """工程の順序を管理し、制御器の更新を制御周期内で1回に制限する。"""

    def __init__(self, engine):
        super().__init__(name=engine.name, memory=True)
        self.engine = engine
        self.tick_engine = engine.tick_once
        self.engine_ticked_this_cycle = False
        self.engine_started = False
        self.add_children(
            [
                SumoPushStage("規定距離を直進してボトルへ正対", self, 0),
                SumoPushStage("捕捉接近と判定", self, 1),
                SumoPushStage("必要な場合だけ戻り旋回", self, 2),
                SumoPushStage("押し出し・正対位置へ戻る・録画終了", self, 3),
            ]
        )

    def initialise(self):
        # 同じツリーを再実行するときは、前回の成功/失敗状態を使い回さない。
        self.engine_started = False

    def tick(self):
        # Sequenceは一工程成功時に次の子も同周期に実行する。共有制御器を
        # 二重tickしないため、周期ごとにこのフラグを初期化する。
        self.engine_ticked_this_cycle = False
        yield from super().tick()

    def terminate(self, new_status):
        if self.engine.status == Status.RUNNING and new_status != Status.RUNNING:
            # Ctrl+C、失敗、外側のツリーによる取消を従来の制動・録画終了へ伝える。
            self.engine.stop(Status.INVALID)


def build_sumo_push_tree(context, config):
    """規定接近 → 捕捉 → 必要なら旋回 → 押し出しのツリーを構築。"""
    from ..runtime import runtime

    engine = build_sumo_push_route(context, config)
    # 単体は開始向きを0°、結合で利用するときはスタート基準で
    # LEFT=-90° / RIGHT=+90°。相撲の床面方位登録とは別の値。
    if config.mission_mode in ("sumo-stage-all", "sumo-stage-push"):
        test = config.sumo_stage_test
        engine.approach_offset_from_reset_deg = (
            test.approach_offset_left_deg if runtime.course == 1
            else test.approach_offset_right_deg
        )
    else:
        engine.approach_offset_from_reset_deg = (
            0.0 if config.mission_mode == "sumo-push-route-test"
            else -90.0 * runtime.course
        )
    return SumoPushSequence(engine)
