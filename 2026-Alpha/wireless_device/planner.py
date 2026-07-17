"""ヒントのゲート位置から、障害物を避けたETラリー経路を計算する。"""

from __future__ import annotations

import heapq
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

from .protocol import GateHint, GridPosition, HintSet


@dataclass(frozen=True)
class Point:
    """コース上の位置を、左下原点の直交座標で表す。

    属性:
        x: 原点から右方向への距離。単位はmm。
        y: 原点から上方向への距離。単位はmm。
    """

    x: float
    y: float

    def distance_to(self, other: "Point") -> float:
        """現在位置から別の位置までの直線距離を求める。

        引数:
            other: 距離を測る相手側のコース座標。

        戻り値:
            2点間のユークリッド距離。単位はmm。
        """
        return math.hypot(other.x - self.x, other.y - self.y)


@dataclass(frozen=True)
class Segment:
    """ゲートや障害物を表す、コース上の線分を保持する。

    属性:
        first: 線分を構成する一方の端点。
        second: 線分を構成するもう一方の端点。
    """

    first: Point
    second: Point


@dataclass(frozen=True)
class PlannerConfig:
    """コース実測値、探索条件、走行体の安全余裕を保持する。

    属性:
        cell_mm: 隣接するゲートポジション間の実距離。単位はmm。
        grid_size: x方向とy方向それぞれのゲートポジション数。
        course_margin_mm: ゲート領域の外側にも探索を許す余白。単位はmm。
        astar_resolution_mm: A*探索で使用する1セルの一辺。単位はmm。
        gate_approach_mm: ゲート中央の手前と奥に置く進入・退出点までの距離。単位はmm。
        obstacle_clearance_mm: 走行経路と、通過対象を含む各ゲートとの安全距離。単位はmm。
        start_mm: 第1周の開始位置を表すコース座標。
        finish_mm: 最終周後に向かう目標位置。目標を追加しない場合は ``None``。
        laps: 赤→青→黄の順にゲートを通過する周回数。
        heading_sign: コース座標の角度を実機のジャイロ符号へ合わせる倍率。
            右旋回を正とする場合は1、逆の場合は-1を指定する。
    """

    cell_mm: float
    grid_size: int
    course_margin_mm: float
    astar_resolution_mm: float
    gate_approach_mm: float
    obstacle_clearance_mm: float
    start_mm: Point
    finish_mm: Optional[Point]
    laps: int
    heading_sign: float

    @classmethod
    def load(cls, path: str | Path) -> "PlannerConfig":
        """JSONファイルを読み込み、数値型をそろえた経路計算設定を作る。

        引数:
            path: 経路計算設定JSONのファイルパス。

        戻り値:
            JSON内の値から生成した経路計算設定。

        例外:
            FileNotFoundError: 指定されたJSONファイルが存在しない場合。
            json.JSONDecodeError: ファイルの内容が正しいJSONでない場合。
            KeyError: 必須の設定項目が存在しない場合。
            TypeError: 座標配列などの型や要素数が不正な場合。
            ValueError: 設定値を所定の数値型へ変換できない場合。
        """
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        finish = data.get("finish_mm")
        return cls(
            cell_mm=float(data["cell_mm"]),
            grid_size=int(data.get("grid_size", 5)),
            course_margin_mm=float(data["course_margin_mm"]),
            astar_resolution_mm=float(data["astar_resolution_mm"]),
            gate_approach_mm=float(data["gate_approach_mm"]),
            obstacle_clearance_mm=float(data["obstacle_clearance_mm"]),
            start_mm=Point(*map(float, data["start_mm"])),
            finish_mm=Point(*map(float, finish)) if finish is not None else None,
            laps=int(data.get("laps", 3)),
            heading_sign=float(data.get("heading_sign", 1.0)),
        )


class RoutePlanner:
    """離散A*探索と直線平滑化により、安全距離を保つゲート通過経路を求める。"""

    def __init__(self, config: PlannerConfig) -> None:
        """設定値からA*探索に使用するコース範囲とセル数を初期化する。

        引数:
            config: コース寸法、探索解像度、安全距離などを保持する設定。

        戻り値:
            なし。
        """
        self.config = config
        # G1から最後のゲート列までの幅に外周余白を加え、探索可能範囲を決める。
        maximum = (config.grid_size - 1) * config.cell_mm
        margin = config.course_margin_mm
        self.min_x = -margin
        self.min_y = -margin
        self.max_x = maximum + margin
        self.max_y = maximum + margin
        self.nx = int(round((self.max_x - self.min_x) / config.astar_resolution_mm)) + 1
        self.ny = int(round((self.max_y - self.min_y) / config.astar_resolution_mm)) + 1

    def grid_position_to_point(self, position: GridPosition) -> Point:
        """ゲート表記を、左下原点で上向きを正とするmm座標へ変換する。

        引数:
            position: G1-1を左上、G5-5を右下とするゲートポジション。

        戻り値:
            G1-5を原点として、右方向をx正、上方向をy正とするコース座標。
        """
        return Point(
            x=(position.x - 1) * self.config.cell_mm,
            y=(self.config.grid_size - position.y) * self.config.cell_mm,
        )

    def gate_segment(self, gate: GateHint) -> Segment:
        """ゲートの両端をコース座標へ変換し、障害物として扱う線分を作る。

        引数:
            gate: 色と2つのゲートポジションを保持するゲート情報。

        戻り値:
            ゲートの両端をmm座標で結んだ線分。
        """
        return Segment(
            self.grid_position_to_point(gate.first),
            self.grid_position_to_point(gate.second),
        )

    @staticmethod
    def _point_to_segment_distance(point: Point, segment: Segment) -> float:
        """点から線分上の最も近い位置までの距離を求める。

        引数:
            point: 距離を測る対象位置。
            segment: ゲートまたは障害物を表す線分。

        戻り値:
            点と線分の最短距離。単位はmm。
        """
        ax, ay = segment.first.x, segment.first.y
        bx, by = segment.second.x, segment.second.y
        vx, vy = bx - ax, by - ay
        length_sq = vx * vx + vy * vy
        if length_sq == 0:
            return point.distance_to(segment.first)
        # 線分方向への射影位置を0～1に制限し、線分外なら最寄りの端点を採用する。
        t = ((point.x - ax) * vx + (point.y - ay) * vy) / length_sq
        t = max(0.0, min(1.0, t))
        nearest = Point(ax + t * vx, ay + t * vy)
        return point.distance_to(nearest)

    def _blocked(self, point: Point, obstacles: Iterable[Segment]) -> bool:
        """指定位置がいずれかの障害物へ近付き過ぎているか判定する。

        引数:
            point: 走行可能かを判定するコース座標。
            obstacles: 障害物として扱うゲート線分の集まり。

        戻り値:
            安全距離未満の障害物が1つでもあれば ``True``、なければ ``False``。
        """
        return any(
            self._point_to_segment_distance(point, obstacle) < self.config.obstacle_clearance_mm
            for obstacle in obstacles
        )

    def _to_cell(self, point: Point) -> tuple[int, int]:
        """連続的なコース座標を、最も近いA*探索セルへ変換する。

        引数:
            point: 変換するmm単位のコース座標。

        戻り値:
            探索範囲内へ収めた ``(x方向セル番号, y方向セル番号)``。
        """
        resolution = self.config.astar_resolution_mm
        ix = round((point.x - self.min_x) / resolution)
        iy = round((point.y - self.min_y) / resolution)
        return max(0, min(self.nx - 1, ix)), max(0, min(self.ny - 1, iy))

    def _to_point(self, cell: tuple[int, int]) -> Point:
        """A*探索セルを、その格子点に対応するコース座標へ変換する。

        引数:
            cell: ``(x方向セル番号, y方向セル番号)``。

        戻り値:
            セルに対応するmm単位のコース座標。
        """
        resolution = self.config.astar_resolution_mm
        return Point(self.min_x + cell[0] * resolution, self.min_y + cell[1] * resolution)

    def _nearest_free(self, cell: tuple[int, int], obstacles: list[Segment]) -> tuple[int, int]:
        """指定セルが塞がれている場合、最も近い走行可能セルを幅優先で探す。

        引数:
            cell: 探索を始める ``(x方向セル番号, y方向セル番号)``。
            obstacles: 安全距離を確保する対象となるゲート線分の一覧。

        戻り値:
            指定セル自身、または上下左右へ広げて最初に見つかった走行可能セル。

        例外:
            ValueError: 探索範囲内に走行可能なセルが存在しない場合。
        """
        if not self._blocked(self._to_point(cell), obstacles):
            return cell
        # 一方向へ偏らず近いセルから調べるため、幅優先探索の待ち行列を使用する。
        queue = [cell]
        seen = {cell}
        while queue:
            current = queue.pop(0)
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                candidate = current[0] + dx, current[1] + dy
                if candidate in seen or not (0 <= candidate[0] < self.nx and 0 <= candidate[1] < self.ny):
                    continue
                if not self._blocked(self._to_point(candidate), obstacles):
                    return candidate
                seen.add(candidate)
                queue.append(candidate)
        raise ValueError("コース上に走行可能なセルがありません")

    def _line_clear(self, first: Point, second: Point, obstacles: list[Segment]) -> bool:
        """2点を結ぶ直線全体が障害物との安全距離を満たすか調べる。

        引数:
            first: 判定する直線の開始位置。
            second: 判定する直線の終了位置。
            obstacles: 安全距離を確保する対象となるゲート線分の一覧。

        戻り値:
            直線上の全確認点が走行可能なら ``True``、1点でも塞がれていれば ``False``。
        """
        distance = first.distance_to(second)
        # 探索セルの半分以下の間隔で確認し、セル間の障害物を見落としにくくする。
        step = max(1.0, self.config.astar_resolution_mm / 2.0)
        samples = max(1, int(math.ceil(distance / step)))
        for index in range(samples + 1):
            ratio = index / samples
            point = Point(
                first.x + (second.x - first.x) * ratio,
                first.y + (second.y - first.y) * ratio,
            )
            if self._blocked(point, obstacles):
                return False
        return True

    def shortest_path(self, start: Point, goal: Point, obstacles: list[Segment]) -> list[Point]:
        """障害物との安全距離を守る、離散グリッド上の最短経路を返す。

        引数:
            start: 経路探索を開始するコース座標。
            goal: 到達目標となるコース座標。
            obstacles: 回避対象となるゲート線分の一覧。

        戻り値:
            開始位置と目標位置を含み、不要な折れを直線化した座標列。

        例外:
            ValueError: 走行可能セルがない、または目標までの経路が見つからない場合。
        """
        # 開始・目標が安全距離内に入っていれば、最寄りの走行可能セルへ補正する。
        start_cell = self._nearest_free(self._to_cell(start), obstacles)
        goal_cell = self._nearest_free(self._to_cell(goal), obstacles)
        queue: list[tuple[float, tuple[int, int]]] = [(0.0, start_cell)]
        came_from: dict[tuple[int, int], tuple[int, int]] = {}
        cost = {start_cell: 0.0}
        # 上下左右に加えて斜め移動を許し、斜めの移動コストは実距離に合わせる。
        moves = (
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
            (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
        )
        while queue:
            _, current = heapq.heappop(queue)
            if current == goal_cell:
                break
            for dx, dy, move_cost in moves:
                candidate = current[0] + dx, current[1] + dy
                if not (0 <= candidate[0] < self.nx and 0 <= candidate[1] < self.ny):
                    continue
                if self._blocked(self._to_point(candidate), obstacles):
                    continue
                next_cost = cost[current] + move_cost
                if next_cost >= cost.get(candidate, math.inf):
                    continue
                cost[candidate] = next_cost
                came_from[candidate] = current
                # 目標までの直線距離を推定値に使い、到達が有望なセルを優先する。
                heuristic = math.hypot(goal_cell[0] - candidate[0], goal_cell[1] - candidate[1])
                heapq.heappush(queue, (next_cost + heuristic, candidate))
        if goal_cell not in cost:
            raise ValueError(f"経路を探索できません: start={start}, goal={goal}")

        # 目標から親セルを逆にたどり、開始位置から進む順番へ並べ直す。
        cells = [goal_cell]
        while cells[-1] != start_cell:
            cells.append(came_from[cells[-1]])
        cells.reverse()
        points = [start, *[self._to_point(cell) for cell in cells[1:-1]], goal]
        return self._smooth(points, obstacles)

    def _smooth(self, points: list[Point], obstacles: list[Segment]) -> list[Point]:
        """A*の細かな折れを、障害物に接触しない最長の直線へまとめる。

        引数:
            points: A*探索が生成した、開始位置から目標位置までの座標列。
            obstacles: 直線化後も安全距離を確保する対象となるゲート線分の一覧。

        戻り値:
            開始位置と目標位置を保ち、途中の不要な座標を取り除いた座標列。
        """
        if len(points) <= 2:
            return points
        result = [points[0]]
        index = 0
        while index < len(points) - 1:
            # 現在位置から最も遠い接続可能点を探し、その間の細かな折れを省略する。
            candidate = len(points) - 1
            while candidate > index + 1:
                if self._line_clear(points[index], points[candidate], obstacles):
                    break
                candidate -= 1
            result.append(points[candidate])
            index = candidate
        return result

    def _gate_crossing_points(self, gate: GateHint, current: Point) -> tuple[Point, Point, Point]:
        """現在位置側からゲートを直角に通過する3つの目標点を求める。

        引数:
            gate: 通過対象となるゲート情報。
            current: ゲートへ向かい始める現在のコース座標。

        戻り値:
            現在位置側の進入点、ゲート中央、反対側の退出点を並べた組。
        """
        segment = self.gate_segment(gate)
        # 端点の平均を取り、実際に通過すべきゲート中央を決める。
        midpoint = Point(
            (segment.first.x + segment.second.x) / 2.0,
            (segment.first.y + segment.second.y) / 2.0,
        )
        vx = segment.second.x - segment.first.x
        vy = segment.second.y - segment.first.y
        length = math.hypot(vx, vy)
        # ゲート線分に直交する単位ベクトルを作り、進入・退出方向として使う。
        nx, ny = -vy / length, vx / length
        side = (current.x - midpoint.x) * nx + (current.y - midpoint.y) * ny
        if side < 0:
            # 進入点が必ず現在位置と同じ側になるよう、必要なら向きを反転する。
            nx, ny = -nx, -ny
        clearance = self.config.gate_approach_mm
        approach = Point(midpoint.x + nx * clearance, midpoint.y + ny * clearance)
        exit_point = Point(midpoint.x - nx * clearance, midpoint.y - ny * clearance)
        return approach, midpoint, exit_point

    def _waypoints_for_lap(
        self,
        start: Point,
        gates: tuple[GateHint, GateHint, GateHint],
        all_obstacles: list[Segment],
    ) -> list[Point]:
        """1周分のゲートを指定順に通過する座標列を作る。

        引数:
            start: この周回を開始するコース座標。
            gates: 通過順に並べた赤・青・黄のゲート情報。
            all_obstacles: 経路探索で安全距離を確保する全ゲート線分。

        戻り値:
            周回開始位置から最後のゲート退出点までを順に並べた座標列。

        例外:
            ValueError: いずれかのゲート進入点までの経路が見つからない場合。
        """
        waypoints = [start]
        current = start
        for gate in gates:
            # ゲート手前までは障害物を避け、中央から退出点までは直角に横断する。
            approach, midpoint, exit_point = self._gate_crossing_points(gate, current)
            approach_path = self.shortest_path(current, approach, all_obstacles)
            waypoints.extend(approach_path[1:])
            waypoints.extend((midpoint, exit_point))
            current = exit_point
        return waypoints

    def _heading(self, first: Point, second: Point) -> float:
        """2点間の進行方向を、実機へ渡す絶対方位へ変換する。

        引数:
            first: 移動を開始するコース座標。
            second: 移動先となるコース座標。

        戻り値:
            上方向を0度、右方向を+90度とし、-180度以上180度未満へ
            正規化した絶対角度。ジャイロの向きは設定の符号で補正する。
        """
        heading = math.degrees(math.atan2(second.x - first.x, second.y - first.y))
        heading *= self.config.heading_sign
        return (heading + 180.0) % 360.0 - 180.0

    def to_segments(self, points: list[Point]) -> list[list[float | int]]:
        """連続する座標列を走行体用の ``[絶対角度, 距離mm]`` 配列へ変換する。

        引数:
            points: 走行順に並べたコース座標の一覧。

        戻り値:
            各直線移動の絶対角度と整数mm距離を組にした一覧。
            1mm未満の移動は除外し、連続する同一角度の移動は1組へまとめる。
        """
        result: list[list[float | int]] = []
        for first, second in zip(points, points[1:]):
            distance = first.distance_to(second)
            if distance < 1.0:
                continue
            heading = round(self._heading(first, second), 1)
            length = max(1, int(round(distance)))
            # 同じ絶対方位が続く区間は距離だけを加算し、実機への指示数を減らす。
            if result and result[-1][0] == heading:
                result[-1][1] += length
            else:
                result.append([heading, length])
        return result

    def plan(self, hints: HintSet) -> dict:
        """赤→青→黄を設定周回数だけ通り、最終目標へ進む経路を作る。

        引数:
            hints: ヒントカードから復元し、配置方向まで検証済みの3色ゲート情報。

        戻り値:
            無線通信で走行体へ返す経路メッセージ。絶対角度の基準、ゲート通過順、
            元のヒント、および周回ごとの ``[絶対角度, 距離mm]`` 配列を含む。

        例外:
            ValueError: いずれかのゲート進入点または最終目標までの経路を
                探索できない場合。
        """
        gates = hints.ordered_gates()
        # 通過するゲートも横断時以外は障害物とみなし、正面の進入点へ誘導する。
        obstacles = [self.gate_segment(gate) for gate in gates]
        current = self.config.start_mm
        laps = []
        for lap_index in range(self.config.laps):
            waypoints = self._waypoints_for_lap(current, gates, obstacles)
            current = waypoints[-1]
            # 最終周だけは、最後のゲート退出後に競技戦略で指定した終了位置へ向かう。
            if lap_index == self.config.laps - 1 and self.config.finish_mm is not None:
                finish_path = self.shortest_path(current, self.config.finish_mm, obstacles)
                waypoints.extend(finish_path[1:])
                current = self.config.finish_mm
            laps.append({"segments": self.to_segments(waypoints)})
        return {
            "type": "route",
            "angle_type": "absolute",
            "coordinate_frame": {"zero": "course_up", "positive": "clockwise"},
            "gate_order": [gate.color for gate in gates],
            "hints": {
                "hint1": hints.red.encoded(),
                "hint2": f"{hints.blue.encoded()}/{hints.yellow.encoded()}",
            },
            "laps": laps,
        }
