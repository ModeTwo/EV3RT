"""Feature 06 subtree factory."""

from py_trees.composites import Sequence
from ..placeholder import PendingFeature


def build_read_hint(context, config, hint_number):
    # No.6 読取位置の調整、画像認識、結果保持、PCへの転送を担当する。
    # 同じ担当ファイルをヒントカード1と2の双方から利用する。
    root = Sequence(name=f"read_hint{hint_number}", memory=True)
    root.add_children([PendingFeature(name=f"read_hint{hint_number}_pending")])
    return root

