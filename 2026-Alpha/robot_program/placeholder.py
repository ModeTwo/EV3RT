"""Temporary behavior used until each feature is implemented."""

from py_trees.behaviour import Behaviour
from py_trees.common import Status


class PendingFeature(Behaviour):
    # 未実装機能がツリー全体の構築を妨げないよう、現時点では即座に成功を返す。
    # 各担当者はこのノードを実際の動作ノードへ置き換える。
    def update(self) -> Status:
        self.logger.warning("PendingFeature skipped: " + self.name)
        return Status.SUCCESS
