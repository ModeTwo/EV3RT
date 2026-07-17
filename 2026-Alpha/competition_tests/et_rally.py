"""受信済みまたはJSON指定の最適経路でETラリーだけを実機テストする。"""

import argparse

from py_trees.trees import BehaviourTree

from .common import (
    add_runtime_arguments,
    build_isolated_tree,
    configure_wireless,
    load_route_file,
    reset_test_state,
    run_on_raspberry_pi,
)


def build_behaviour_tree() -> BehaviourTree:
    return build_isolated_tree(
        test_name="ETラリー単体テスト",
        section_names=[
            "ボトルデリバリー1周完了・次のLAPゲート通過前",
            "ETラリー3周",
        ],
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ETラリー単体テスト")
    add_runtime_arguments(parser, wireless=True)
    parser.add_argument(
        "--route-file",
        default=None,
        help="通信を使わずに試す場合の経路JSON。未指定時はUDP受信を待つ。",
    )
    args = parser.parse_args()
    reset_test_state()
    if args.route_file:
        load_route_file(args.route_file)
    else:
        configure_wireless(args)
    run_on_raspberry_pi(build_behaviour_tree(), args.logfile)
