"""ガレージへのライントレースと白検出後の完全停止だけを実機テストする。"""

import argparse

from py_trees.trees import BehaviourTree

from .common import add_runtime_arguments, build_isolated_tree, reset_test_state, run_on_raspberry_pi


def build_behaviour_tree() -> BehaviourTree:
    return build_isolated_tree(
        test_name="ガレージ停止単体テスト",
        section_names=["ガレージ停止"],
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ガレージ停止単体テスト")
    add_runtime_arguments(parser)
    args = parser.parse_args()
    reset_test_state()
    run_on_raspberry_pi(build_behaviour_tree(), args.logfile)
