"""ET rally phase composition."""

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.common import ParallelPolicy
from py_trees.composites import Parallel, Sequence

from py_etrobo_util.plotter import ET_RALLY_TIRE_DIAMETER, TIRE_DIAMETER, UNCALIBRATED_TIRE_DIAMETER

from ..behaviours.device_control import ArmDirection
from ..behaviours.et_rally_arm import (EtRallyArmAlreadyUp, EtRallyArmUpLater, arm_up_before_touch, arm_while_driving,
                                      make_arm_up_down)
from ..features.execute_strategy import build_execute_strategy
from ..behaviours.motor_control import StopNow
from ..behaviours.et_rally_drive import set_move_carry_integral, set_move_tuning
from ..features.receive_strategy import build_receive_strategy
from .. import gyro_scale
from ..gyro_scale import EnableGyroScale
from ..et_rally_heading import (EtRallyArmUpWithDriftMeasure, EtRallyPostStopGyroMeasure, InstallEtRallyHeading,
                                RestoreEtRallyHeading)
from ..runtime import runtime
from ..et_rally_start_pose import EtRallyApplyStartPose, EtRallyStartPoseCapture
from ..et_rally_camera_correct import EtRallyCameraCorrect
from ..et_rally_course_pose import (EtRallyAnchorCoursePose, EtRallyCameraLogStart, EtRallyCameraLogStop,
                                    EtRallyCameraProcessStart, EtRallyCameraProcessStop, EtRallyReleaseCoursePose)


class EtRallyArrivedAtStart(Behaviour):
    """2026-11-11: スタート位置に着いた時刻をログに出す(本番に近い流れのテストで、ここから停止までを測る)。"""

    def update(self):
        self.logger.info("ARRIVED AT START: distance %d" % runtime.plotter.get_distance())
        return Status.SUCCESS


class SetGyroScaleDisabled(Behaviour):
    # DD-19較正用: EtRallyRunByGyro等が内部で無条件に呼ぶensure_scaled_gyro()を、
    # この工程の間だけ無効化する(--rally-gyro-tire-calibration off)。
    def __init__(self, name, disabled):
        super().__init__(name)
        self.disabled = disabled

    def update(self):
        gyro_scale.set_disabled(self.disabled)
        self.logger.info("gyro scale disabled=%s" % self.disabled)
        return Status.SUCCESS


class SetMoveCarryIntegral(Behaviour):
    # 2026-10-30: 直進の向きのPIDのI成分を、前の直進から引き継ぐかどうか(et_rally_drive.py)。
    def __init__(self, name, enabled):
        super().__init__(name)
        self.enabled = enabled

    def update(self):
        set_move_carry_integral(self.enabled)
        self.logger.info("move carry integral=%s" % self.enabled)
        return Status.SUCCESS


class SetMoveTuning(Behaviour):
    # 2026-10-31: 直進の左右の出力を小数のまま出すか、向きのPIDのIを変えるか(et_rally_drive.py)。
    def __init__(self, name, fine_power, pid_i):
        super().__init__(name)
        self.fine_power = fine_power
        self.pid_i = pid_i

    def update(self):
        set_move_tuning(self.fine_power, self.pid_i)
        self.logger.info("move tuning: fine_power=%s pid_i=%s" % (self.fine_power, self.pid_i))
        return Status.SUCCESS


class SetTireDiameter(Behaviour):
    # ETラリー専用のタイヤ径較正を、この工程の間だけ走行距離計算へ適用する。
    def __init__(self, name, diameter):
        super().__init__(name)
        self.diameter = diameter

    def update(self):
        runtime.require("plotter")
        runtime.plotter.tire_diameter = self.diameter
        self.logger.info("Plotter tire diameter set to %.2f mm" % self.diameter)
        return Status.SUCCESS


def _plan_gates(config):
    """計画のファイルの gates(ゲートの位置)。読めなければ None(脚の点を外さない)。"""
    try:
        import json
        from ..features.execute_strategy import _resolve_plan_path
        path = _resolve_plan_path(getattr(config, "et_rally_plan_path", None))
        return json.load(open(path, encoding="utf-8")).get("gates")
    except Exception:
        return None


def build_et_rally_phase(context, config):
    # PCまたは固定planが全周回分を一つのSEQとして返すため、SEQは一度だけ実行する。
    # ボトル配送までアームを下げたまま運んでいるため、ETラリー走行の前に上げ直す。
    root = Sequence(name="et_rally", memory=True)
    # DD-19ベースライン計測用: Falseならジャイロ倍率補正・タイヤ径較正(DD-17)を適用しない。
    gyro_tire_calibration = getattr(config, "et_rally_gyro_tire_calibration_enabled", True)
    children = []
    # 2026-10-29: 四捨五入前の向き・止まっている間のずれの差し引き(robot_program/et_rally_heading.py)。
    float_heading = bool(getattr(config, "et_rally_gyro_float_heading", None))
    drift_compensation = bool(getattr(config, "et_rally_gyro_drift_compensation", None))
    # 2026-10-30: 左回り(headingが減る向き)だけの倍率。
    scale_neg = getattr(config, "et_rally_gyro_scale_neg", None)
    scale_pos = getattr(config, "et_rally_gyro_scale_pos", None)
    install_heading = gyro_tire_calibration and (float_heading or drift_compensation or scale_neg is not None
                                                 or scale_pos is not None)
    if gyro_tire_calibration:
        children.extend([
            # 通しの実行ではラップゲート後に有効済み(何もしない)。ETラリーだけ単独で走らせる場合の保険。
            EnableGyroScale("et_rally gyro scale"),
            SetTireDiameter("et_rally tire diameter", ET_RALLY_TIRE_DIAMETER),
        ])
    else:
        # TIRE_DIAMETER(57.33)自体が既に2段階較正済みの値のため、「補正なし」の
        # ベースラインでは、較正前の最初の想定値(55.0)を明示的に使う。
        children.append(SetTireDiameter("et_rally tire diameter uncalibrated", UNCALIBRATED_TIRE_DIAMETER))
        # EtRallyRunByGyro等はフラグに関わらず内部でensure_scaled_gyro()を呼ぶため、
        # 窓口自体を無効化してジャイロ倍率補正を確実に無しにする。
        children.append(SetGyroScaleDisabled("et_rally gyro scale disabled", True))
    if install_heading:
        children.append(InstallEtRallyHeading("et_rally heading", float_heading, drift_compensation, scale_neg,
                                              scale_pos))
    # 2026-11-10: カメラ(スタートの取り込み・走行中の画像)は子プロセスで動かす。開くのに時間がかかるので最初に起動する。
    camera_log_every = getattr(config, "et_rally_camera_log_every", None)
    use_start_pose = bool(getattr(config, "et_rally_start_pose", None))
    camera_correct = getattr(config, "et_rally_camera_correct", None)
    if camera_log_every or use_start_pose or camera_correct:
        # 2026-11-10: ゲートの脚の上の点は地図から外す(計画のファイルの gates)。親(走行中の補正)と子(スタートの取り込み)の両方で使う。
        gates = _plan_gates(config)
        from .. import et_rally_start_pose as _sp
        _sp.set_gate_legs(gates)
        children.append(EtRallyCameraProcessStart(
            "et_rally camera process", log_every=camera_log_every,
            exposure=getattr(config, "et_rally_camera_exposure", None),
            calib_path=getattr(config, "et_rally_camera_calib_path", None),
            prestart=(getattr(config, "mission_mode", None) == "rally-drive"), gates=gates))
    # 2026-11-11: 本番に近い流れのテスト。スタート位置の手前に置いてタッチし、この距離だけ進んでスタート位置に着いてから、
    # ジャイロの測り直し・スタートの取り込みをして走り出す(本番は前の種目からスタート位置へ来て、タッチなしで始まる)。
    pre_move_mm = getattr(config, "et_rally_pre_move_mm", None)
    if pre_move_mm:
        from ..behaviours.et_rally_path_segment import EtRallyPathSegment
        children.append(EtRallyPathSegment(
            name="et_rally move to start", kind="line", end_heading_deg=0.0, length_mm=float(pre_move_mm), power=40,
            pid_p=4.0, pid_i=0.6, pid_d=0.06, stop=True))
        children.append(EtRallyArrivedAtStart("et_rally arrived at start"))
    # 2026-11-06: 計画から見て間に合うなら、アームは走りながら上げる(et_rally_arm.arm_while_driving)。
    drive_arm, drive_arm_reason = arm_while_driving(config)
    if arm_up_before_touch(config):
        arm_up = EtRallyArmAlreadyUp("et_rally arm up")
    elif drive_arm:
        arm_up = EtRallyArmUpLater("et_rally arm up later", drive_arm_reason)
    else:
        arm_up = make_arm_up_down("et_rally arm up", ArmDirection.UP, config, from_bottom=True)
    # 2026-10-29: ハブのプログラムを書き換えた版(ブレーキ中もハブがジャイロのくせを測り直す)用に、
    # アームを上げ切って振動が収まってから、この秒数だけブレーキをかけて止まる。
    hub_recal_sec = getattr(config, "et_rally_gyro_hub_recal_sec", None)
    if hub_recal_sec and not (install_heading and drift_compensation):
        settle_sec = getattr(config, "et_rally_gyro_drift_settle_sec", None)
        arm_up = EtRallyArmUpWithDriftMeasure(
            "et_rally arm up", arm_up,
            settle_sec=1.0 if settle_sec is None else settle_sec,
            measure_sec=hub_recal_sec)
    if install_heading and drift_compensation:
        # 2026-10-29: アームを上げ切って振動が収まってから、止まってジャイロのずれの速さを測る
        # (et_rally_heading.py)。既定は振動待ち1秒+測定2秒。
        settle_sec = getattr(config, "et_rally_gyro_drift_settle_sec", None)
        measure_sec = getattr(config, "et_rally_gyro_drift_extra_sec", None)
        arm_up = EtRallyArmUpWithDriftMeasure(
            "et_rally arm up", arm_up,
            settle_sec=1.0 if settle_sec is None else settle_sec,
            measure_sec=2.0 if measure_sec is None else measure_sec)
    if getattr(config, "et_rally_start_pose", None):
        # 2026-11-06: 止まっている間にカメラで位置と向きを測り、走り出しに取り込む(et_rally_start_pose.py)。
        # アームを上げてジャイロを測る間に並べて撮る(アームが動いている間の画像は使わない)。
        pose_sec = getattr(config, "et_rally_start_pose_sec", None)
        ready = None
        if isinstance(arm_up, EtRallyArmUpWithDriftMeasure):
            ready = lambda node=arm_up: getattr(node, "arm_done_time", None) is not None
        capture = EtRallyStartPoseCapture("et_rally start pose", duration_sec=2.0 if pose_sec is None else pose_sec,
                                          calib_path=getattr(config, "et_rally_camera_calib_path", None), ready=ready)
        if ready is not None:
            both = Parallel(name="et_rally arm/gyro + start pose", policy=ParallelPolicy.SuccessOnAll(synchronise=True))
            both.add_children([arm_up, capture])
            children.append(both)
        else:
            children.extend([arm_up, capture])
        rejoin_min = getattr(config, "et_rally_start_pose_rejoin_min_mm", None)
        children.append(EtRallyApplyStartPose(
            "et_rally apply start pose", rejoin_min_mm=5.0 if rejoin_min is None else rejoin_min,
            plan_frame_known=(config.et_rally_strategy_source == "file")))
        if not camera_log_every and not camera_correct:
            children.append(EtRallyCameraProcessStop("et_rally camera process stop"))
    else:
        children.append(arm_up)
    carry_integral = bool(getattr(config, "et_rally_move_carry_integral", None))
    if carry_integral:
        children.append(SetMoveCarryIntegral("et_rally move carry integral", True))
    fine_power = bool(getattr(config, "et_rally_move_fine_power", None))
    move_pid_i = getattr(config, "et_rally_move_pid_i", None)
    move_tuning = fine_power or move_pid_i is not None
    if move_tuning:
        children.append(SetMoveTuning("et_rally move tuning", fine_power, move_pid_i))
    if config.et_rally_strategy_source == "received":
        children.append(build_receive_strategy(context, config))
    elif config.et_rally_strategy_source != "file":
        raise ValueError(
            "Unknown ET rally strategy source: " + str(config.et_rally_strategy_source)
        )
    # 2026-11-08: コース上の自分の位置(et_rally_course_pose.py)。スタートの取り込みか、走行中の画像を残すときに持つ。
    track_course = install_heading and (getattr(config, "et_rally_start_pose", None) or camera_log_every or camera_correct)
    if track_course:
        children.append(EtRallyAnchorCoursePose("et_rally course pose"))
    if camera_log_every:
        children.append(EtRallyCameraLogStart("et_rally camera log", every_n=camera_log_every,
                                              exposure=getattr(config, "et_rally_camera_exposure", None)))
    execute = build_execute_strategy(context, config)
    if camera_correct and track_course:
        # 2026-11-10: 走りながらカメラで位置と向きを直す(試作)。走行が終わったら止める。
        corrected = Parallel(name="et_rally drive + camera correct", policy=ParallelPolicy.SuccessOnSelected(children=[execute]))
        corrected.add_children([execute, EtRallyCameraCorrect("et_rally camera correct", camera_correct[0], camera_correct[1])])
        execute = corrected
    if drive_arm:
        # 走りながらアームを上げる(走りとアームを同じtickで進める)。両方終わったら次へ。
        driving = Parallel(name="et_rally drive + arm up", policy=ParallelPolicy.SuccessOnAll(synchronise=True))
        driving.add_children([execute, make_arm_up_down("et_rally arm up while driving", ArmDirection.UP, config, from_bottom=True)])
        children.append(driving)
    else:
        children.append(execute)
    # ET用の直進は、区間が連続するゲート通過中に止まらないよう終了時のブレーキを持たない。
    # 最後の区間の後は、ここで明示的に停止する(単体テストでは、この後は何も動かさないため必須)。
    children.append(StopNow(name="et_rally final stop"))
    if camera_log_every or camera_correct:
        children.append(EtRallyCameraLogStop("et_rally camera log stop"))
    if track_course:
        children.append(EtRallyReleaseCoursePose("et_rally course pose release"))
    post_measure_sec = getattr(config, "et_rally_gyro_post_measure_sec", None)
    if post_measure_sec:
        children.append(EtRallyPostStopGyroMeasure("et_rally gyro post measure", post_measure_sec))
    # calibration有効時はET_RALLY_TIRE_DIAMETERから、無効時はUNCALIBRATED_TIRE_DIAMETERから、
    # いずれの場合も共通デフォルト(TIRE_DIAMETER)へ戻す。
    children.append(SetTireDiameter("et_rally tire diameter restore", TIRE_DIAMETER))
    if carry_integral:
        children.append(SetMoveCarryIntegral("et_rally move carry integral off", False))
    if move_tuning:
        children.append(SetMoveTuning("et_rally move tuning off", False, None))
    if install_heading:
        children.append(RestoreEtRallyHeading("et_rally heading restore"))
    if not gyro_tire_calibration:
        # 後続の工程(同一プロセス内の別ミッション等)に影響させないよう元に戻す。
        children.append(SetGyroScaleDisabled("et_rally gyro scale re-enabled", False))
    root.add_children(children)
    return root
