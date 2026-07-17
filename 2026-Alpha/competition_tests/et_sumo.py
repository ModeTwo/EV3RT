"""ET相撲のボトル探索からガレージラインへ向くまでを実機テストする。"""

import argparse

from py_trees.trees import BehaviourTree

from .common import add_runtime_arguments, build_isolated_tree, reset_test_state, run_on_raspberry_pi


def build_behaviour_tree() -> BehaviourTree:
    return build_isolated_tree(
        test_name="ET相撲単体テスト",
        section_names=["ET相撲"],
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ET相撲単体テスト")
    add_runtime_arguments(parser)
    args = parser.parse_args()
    reset_test_state()
    run_on_raspberry_pi(build_behaviour_tree(), args.logfile)
