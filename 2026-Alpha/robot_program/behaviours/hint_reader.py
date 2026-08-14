"""Reusable hint card reader behavior."""

from py_trees.behaviour import Behaviour
from py_trees.common import Status

from py_etrobo_util import TargetInterested

from ..runtime import runtime


class ReadHintCard(Behaviour):
    # カメラでHintカードの文字列を読み取り、未復号のままRaceContextへ保存する。
    # パスワード入力、復号、走行指示SEQ生成はPC側の責務とする。
    def __init__(self, name: str, hint_number: int, context) -> None:
        super().__init__(name)
        if hint_number not in (1, 2):
            raise ValueError("hint_number must be 1 or 2")
        self.hint_number = hint_number
        self.context = context
        self.running = False

    def update(self) -> Status:
        runtime.require("plotter", "video")
        if not self.running:
            self.running = True
            runtime.video.set_target_interested(TargetInterested.QRCODE)
            self.logger.info(
                "%+06d %s.reading hint%d"
                % (
                    runtime.plotter.get_distance(),
                    self.__class__.__name__,
                    self.hint_number,
                )
            )

        raw_text = runtime.video.get_QR_text()
        if not raw_text:
            return Status.RUNNING

        if self.hint_number == 1:
            self.context.hint1 = raw_text
        else:
            self.context.hint2 = raw_text
        runtime.video.set_target_interested(TargetInterested.LINE)
        self.logger.info(
            "%+06d %s.hint%d captured"
            % (
                runtime.plotter.get_distance(),
                self.__class__.__name__,
                self.hint_number,
            )
        )
        return Status.SUCCESS

    def terminate(self, new_status: Status) -> None:
        # 読取終了または中断時に、後続走行で使用するライン認識モードへ戻す。
        if runtime.video is not None:
            runtime.video.set_target_interested(TargetInterested.LINE)
        self.running = False

