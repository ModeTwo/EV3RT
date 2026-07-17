"""ボトル捕捉、ヒント取得、色別ボトルデリバリーをまとめて実機テストする。"""

import argparse

from py_trees.trees import BehaviourTree

from .common import (
    add_runtime_arguments,
    build_isolated_tree,
    configure_wireless,
    reset_test_state,
    run_on_raspberry_pi,
)


def build_behaviour_tree() -> BehaviourTree:
    return build_isolated_tree(
        test_name="ボトルデリバリー単体テスト",
        section_names=[
            "ボトル色検出とキャッチ",
            "ヒントカード1取得",
            "ヒントカード2取得",
            "ボトルデリバリー",
        ],
        require_hint2_password=True,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ボトルデリバリー単体テスト")
    add_runtime_arguments(parser, wireless=True)
    args = parser.parse_args()
    reset_test_state()
    configure_wireless(args)
    run_on_raspberry_pi(build_behaviour_tree(), args.logfile)
