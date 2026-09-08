"""Reusable bottle condition behaviors."""

from py_trees.behaviour import Behaviour
from py_trees.common import Status

from py_etrobo_util import BottleColor, TargetInterested

from ..runtime import runtime


def _color_value(color):
    # Contextが列挙型または文字列のどちらを保持していても同じ値として比較する。
    return getattr(color, "value", color)


class IsBottleInsight(Behaviour):
    # 指定色のボトルが一定面積以上で連続検出された場合に成功する。
    def __init__(
        self,
        name: str,
        color: BottleColor,
        min_area: int = 150,
        min_frames: int = 2,
        set_target: bool = True,
    ) -> None:
        super().__init__(name)
        self.color = color
        self.min_area = min_area
        self.min_frames = min_frames
        self.set_target = set_target
        self.hits = 0
        self.running = False

    def update(self) -> Status:
        runtime.require("plotter", "video")
        if not self.running:
            self.running = True
            if self.set_target:
                runtime.video.set_target_interested(TargetInterested.BOTTLE)
            self.logger.info(
                "%+06d %s.watching for color=%s"
                % (
                    runtime.plotter.get_distance(),
                    self.__class__.__name__,
                    self.color.name,
                )
            )

        insight, color, _, _, _, area, _ = runtime.video.get_bottle_stamped()
        matches = (
            insight
            and area >= self.min_area
            and (self.color == BottleColor.NONE or color == self.color)
        )
        self.hits = self.hits + 1 if matches else 0
        if self.hits >= self.min_frames:
            return Status.SUCCESS
        return Status.FAILURE

    def terminate(self, new_status: Status) -> None:
        # FAILURE後も連続フレーム数を維持し、次のtickでデバウンス判定を継続する。
        pass


class HasCaughtBottle(Behaviour):
    # RaceContextに保存された取得済みボトル色が指定色と一致するか確認する。
    def __init__(self, name: str, color: BottleColor, context) -> None:
        super().__init__(name)
        self.color = color
        self.context = context

    def update(self) -> Status:
        caught_color = self.context.bottle_color
        caught_value = _color_value(caught_color)
        none_value = _color_value(BottleColor.NONE)
        if self.color == BottleColor.NONE:
            caught = caught_color is not None and caught_value != none_value
        else:
            caught = caught_value == _color_value(self.color)

        self.logger.info(
            "%s.want=%s caught=%s -> %s"
            % (
                self.__class__.__name__,
                self.color.name,
                caught_value,
                "SUCCESS" if caught else "FAILURE",
            )
        )
        return Status.SUCCESS if caught else Status.FAILURE
