"""マーカー(et_rally_marker_detector.detect()の出力)の4隅から、
走行体の向き誤差(度)を推定する、ETラリー専用の計算モジュール。

【設計方針(2026-09、会話にもとづく)】
案A(角度のみの補正)をまず実現することを目的とし、横ズレ・前後ズレ・距離の
mm換算(参考プログラムでいうPoseErrorEstimator相当)は、今回のスコープに含めない。
案Aに必要なのは「マーカーがどれだけ傾いて見えるか」だけであり、以下の理由で、
参考プログラムのPositionEstimator(基準スナップショットとの相対差分)ではなく、
「正しい向きのときの見え方(絶対基準)」との差分を使う設計にした:

  - 目的が「未知の基準位置とのズレを知ること」ではなく、「走行体の向きを
    ジャイロとは独立な方法で正しい値へ戻すこと」なので、走行のたびに変わる
    相対スナップショットより、固定の絶対基準のほうが素直に目的に合う。
  - 相対差分方式だと、最初に登録する基準そのものが正しい向きだったという
    保証が要る(基準登録時にも同じズレ検出問題が残る)。絶対基準方式なら、
    その基準1つを較正すればよい。

EXPECTED_MARKER_ANGLE_DEGとANGLE_TO_HEADING_GAINは、どちらも実測前の
プレースホルダー。wireless_device/et_rally_lqi/のPLACEHOLDER_K_*と同じ位置づけで、
実機でマーカーを撮影して初めて較正できる値であり、走行体が無い現時点では
埋められない(このモジュール自体は、これらの値を差し替えるだけで動く形に
しておくことが目的)。
"""
import math


# 【要実測・プレースホルダー】走行体が正しい向き(ジャイロ目標角どおり)で
# マーカーを見たときに、マーカー上辺がカメラ画像上で何度に見えるか。
# カメラの取り付け角度・マーカーとの相対位置で決まる値で、0とは限らない。
EXPECTED_MARKER_ANGLE_DEG = 0.0

# 【要実測・プレースホルダー】画像上のマーカー回転角の差(度)を、
# 実際の走行体の向き誤差(度)に変換する係数。マーカーを正面から見ている
# 場合は1.0に近いはずだが、カメラの取り付け角度・マーカーまでの距離に
# よって変わるため、実測で較正するまでは1.0と仮定する。
ANGLE_TO_HEADING_GAIN = 1.0

# この角度を超える補正量は、誤検出(別の模様を拾った等)を疑って採用しない。
# 旋回直後の残差はフェーズ2で0.5度未満まで詰めている想定なので、それより
# 大きく外れる値は、カメラ側の誤検出の可能性のほうが高いと判断する。
MAX_TRUSTED_CORRECTION_DEG = 8.0


def _distance(p1, p2):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    return math.sqrt(dx * dx + dy * dy)


def normalize_angle(angle):
    """角度を-180〜+180度に正規化する。"""
    return (angle + 180.0) % 360.0 - 180.0


def calculate_marker_features(corners):
    """4隅([左上,右上,右下,左下]の順、et_rally_marker_detector.detect()の出力と同じ)から、
    中心・横幅・高さ・上辺の角度を求める。

    戻り値: dict、または corners が不正な場合はNone。
    """
    if corners is None or len(corners) != 4:
        return None

    top_left, top_right, bottom_right, bottom_left = corners

    center_x = (top_left[0] + top_right[0] + bottom_right[0] + bottom_left[0]) / 4.0
    center_y = (top_left[1] + top_right[1] + bottom_right[1] + bottom_left[1]) / 4.0

    width = (_distance(top_left, top_right) + _distance(bottom_left, bottom_right)) / 2.0
    height = (_distance(top_left, bottom_left) + _distance(top_right, bottom_right)) / 2.0

    dx = top_right[0] - top_left[0]
    dy = top_right[1] - top_left[1]
    angle = math.degrees(math.atan2(dy, dx))

    return {
        "center_x": center_x, "center_y": center_y,
        "width": width, "height": height, "angle": angle,
    }


def estimate_heading_correction_deg(
        corners, *,
        expected_angle_deg=EXPECTED_MARKER_ANGLE_DEG,
        gain=ANGLE_TO_HEADING_GAIN,
        max_trusted_deg=MAX_TRUSTED_CORRECTION_DEG):
    """マーカーの4隅から、走行体に与えるべき向き補正量(度、相対値)を推定する。

    符号の意味は、呼び出し側(behaviours/et_rally_camera_correction.py)が
    そのままEtRallyTurnLqi(target_type=RELATIVE)へtargetとして渡す前提。
    つまり戻り値だけ現在のジャイロ目標へ加算すれば、正しい向きへ近づく符号。

    戻り値: (correction_deg, raw_marker_angle_deg) のtuple。
        信頼できない(隅が不正/補正量が大きすぎる)場合は (None, raw_marker_angle_deg または None)。
    """
    features = calculate_marker_features(corners)
    if features is None:
        return None, None

    raw_angle = features["angle"]
    diff = normalize_angle(raw_angle - expected_angle_deg)
    correction = gain * diff

    if abs(correction) > max_trusted_deg:
        return None, raw_angle

    return correction, raw_angle
