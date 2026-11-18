"""LAP camera merge and gate control using the standard VideoThread."""
import json, math, time, os


def white_sided_rejoin_enabled():
    """Opt-in comparison; invalid options fail before the robot starts."""
    value = os.environ.get('LAP_WHITE_SIDED_REJOIN', 'off').lower()
    if value not in ('on', 'off'):
        raise ValueError('LAP_WHITE_SIDED_REJOIN must be on or off')
    return value == 'on'
from collections import deque, Counter
from py_trees.behaviour import Behaviour
from py_trees.common import Status, ParallelPolicy
from py_trees.composites import Parallel, Sequence
from py_trees.decorators import Timeout
from simple_pid import PID
from ..runtime import runtime
from ..timing import CONTROL_INTERVAL_SEC
from ..behaviours.lap_gyro_v83 import RunByGyro
from ..behaviours.gyro_drive import RunByGyro as DevTKRunByGyro
from .start_to_lap_gate import RunByGyroSpeedDownatCurve, speed_profile
from .lap_straight_window import final_zero_window
from .lap_recovery_options import (
    decision_offset_mm,
    unseen_estimate_distance,
    dark_heading_mode,
    approach_mode,
    blue_unconfirmed_limit_mm,
    confirmed_slow_distance,
    dark_entry_edge_mode,
    entry_heading_limit,
    far_entry_policy,
    far_peak_deg,
    gate_policy,
    gate_recovery_mode,
    gate_stop_measure_enabled,
    lap_camera_reference_x,
    line_entry_mode,
    merge_deadband,
    merge_length,
    merge_peak,
    post_gate_steer_mode,
    trace_target_v,
    trial_variant,
)
from ..diagnostics.lap_recovery_observation import FloorMotionRecorder, DumpTransitionDiagnostics
from ..behaviours.camera_merge_demo import CameraMergeDemo, ForwardOnlyTrace, IMAGE_EDGE_X, IMAGE_DEADBAND, CAMERA_MAX_AGE_SEC, TRACE_POWER, TRACE_MM, MERGE_LIMIT_MM, MERGE_PEAK_DEG, MERGE_POWER
from py_etrobo_util import TraceSide, Color, ColorClassifier
from ..heading_trace_blend_trial import mix_heading_trace
from .lap_dark_heading import white_side_turn
from ..behaviours.white_recovery_shadow import white_recovery_shadow
from ..behaviours.lap_projected_distance import ProjectedDistance
from ..gyro_scale import EnableGyroScale
from ..behaviours.detect_bottle_color import DetectBottleColor
from ..behaviours.motor_control import StopNow

FINAL_OBSERVATION_START_MM=4350.0

CAMERA_CORRECTION_START_MM=4450.0

CAMERA_CORRECTION_DEADLINE_MM=4650.0

FINAL_STRAIGHT_TEST_START_MM = 4200.0

CAMERA_CORRECTION_LENGTH_MM=600.0

CAMERA_CORRECTION_PEAK_DEG=8.0

LAP_BLUE_MIN_V=65

LAP_WEAK_BLUE_MIN_V=60

LAP_WEAK_BLUE_MIN_S=90

LAP_WEAK_BLUE_SAMPLES=3

WEAK_BLUE_TO_BLACK_MIN_MM=40.0

ALIGN_LIMIT_MM=160.0

ALIGN_LIMIT_SEC=0.8

LINE_CROSS_LIMIT_MM=300.0

LINE_CROSS_DARK_V=35

LINE_CROSS_RECOVERY_WHITE_SAMPLES = 3

LINE_CROSS_BRIGHTNESS_RISE=8

LINE_CROSS_EXIT_HEADING_DEG=2.0


EARLY_EXIT_EDGE_MAX_V=60

EARLY_EXIT_MEMORY_MM=150.0

EARLY_EXIT_MIN_HEADING_DEG=2.0

RECOVERY_LIMIT_MM=180.0

RECOVERY_LIMIT_SEC=1.0

RECOVERY_MAX_ATTEMPTS=2

RECOVERY_PEAK_DEG=8.0

FALLBACK_AFTER_GATE_MM=120.0
UNSEEN_GATE_FALLBACK_AFTER_MM=0.0  # 青未検知時は推定ゲート位置まで
UNSEEN_DISTANCE_REDUCTION_MM=100.0
UNSEEN_ESTIMATE_SLOW_BEFORE_GATE_MM=190.0
UNSEEN_ESTIMATE_MIN_SLOW_MM=50.0

GATE_STOP_RECORD_SEC=0.5

# Explicit comparison mode; normal full-mission behavior remains unchanged.
REACQUIRE_MODE=os.environ.get('LAP_REACQUIRE_MODE', 'baseline').lower()
if REACQUIRE_MODE not in ('baseline', 'extended'):
    raise ValueError('LAP_REACQUIRE_MODE must be baseline or extended')
REACQUIRE_LIMIT_MM=100.0 if REACQUIRE_MODE == 'extended' else 60.0
REACQUIRE_LIMIT_SEC=0.25 if REACQUIRE_MODE == 'extended' else 0.15

ENTRY_SIDE_LIMIT_MM=90.0

ENTRY_SIDE_LIMIT_SEC=0.22

BLUE_TO_BLACK_LIMIT_MM=120.0
LATE_BLUE_BLACK_GAP_MM=150.0
WEAK_BLUE_MODE=os.environ.get('LAP_WEAK_BLUE_MODE','baseline').lower()
GATE_TARGET_MODE=os.environ.get('LAP_GATE_TARGET','baseline').lower()
if WEAK_BLUE_MODE not in ('baseline','early'):
    raise ValueError('LAP_WEAK_BLUE_MODE must be baseline or early')
if GATE_TARGET_MODE not in ('baseline','gray_center'):
    raise ValueError('LAP_GATE_TARGET must be baseline or gray_center')
GRAY_CENTER_FROM_BLUE_MM=270.0  # measured distance from blue leading edge
BLUE_APPROACH_TARGET_MM=GRAY_CENTER_FROM_BLUE_MM-30.0
SHORT_BLUE_MAX_MM=60.0  # provisional: observed 33 mm failure vs 75-99 mm passes



def confirmed_black_approach_mm(gate, normal_mm):
    """Keep normal approach unless black arrived unusually late after blue."""
    blue = getattr(gate, 'blue_progress', None)
    black = getattr(gate, 'black_progress', None)
    if blue is None or black is None:
        return float(normal_mm), 'missing_blue_black_progress'
    blue, black = float(blue), float(black)
    if not math.isfinite(blue) or not math.isfinite(black) or black < blue:
        return float(normal_mm), 'invalid_blue_black_progress'
    gap = black - blue
    if gap < SHORT_BLUE_MAX_MM:
        return float(normal_mm), 'short_blue_fixed_after_black'
    if GATE_TARGET_MODE == 'gray_center':
        return max(0.0, BLUE_APPROACH_TARGET_MM - gap), 'gray_center_estimate'
    if gap <= LATE_BLUE_BLACK_GAP_MM:
        return float(normal_mm), 'normal_black'
    return min(float(normal_mm), max(0.0, blue_unconfirmed_limit_mm() - gap)), 'late_black'

# One bounded second merge only when the first merge ended on white.
LATE_MERGE_LENGTH_MM=180.0
LATE_MERGE_PEAK_DEG=12.0
LATE_MERGE_MIN_OFFSET_PX=30.0

# The three far-entry recordings began at least 65 px from the 160 px reference.
FAR_ENTRY_OFFSET_PX=60.0
FAR_ENTRY_RETURN_MM=180.0
FAR_ENTRY_MIN_HEADING_DEG=5.0
FAR_VOTE_GRACE_SEC=0.10
FAR_VOTE_GRACE_MM=40.0

# Empirical image threshold, not a floor-distance calibration.
ADAPTIVE_INITIAL_OFFSET_PX=84.0
ADAPTIVE_RAMP_MM=100.0



LIVE_CAMERA_MAX_HEADING_DEG = 8.0

LIVE_CAMERA_HEADING_DEG_PER_PX = 0.3

LIVE_CAMERA_ALIGNMENT_DEG = 6.0

PROFILE_REJOIN_WHITE_SAMPLES = 3

PROFILE_REJOIN_ALIGNED_DEG = 3.0



APPROACH_MIN_AREA_PX=2000

# 2026-11-18(devSS): ボトルへカメラで向かう(--lap-bottle-aim、既定オフ)。
# 最後の直進で線に戻る→それる→戻るの蛇行があり、ボトルに横ずれ・斜めで当たって片方のアームから外へそれる。
# ボトルが正面近くに続けて見えたら、線をたどるのをやめて fallback(床の青→黒で止める距離を決める、見えなければ推定のゲート)へ移り、
# その向きを heading_origin ではなく、画像ごとのボトルの方向にする。ボトルが近すぎて見えなくなったら最後の向きのまま。
BOTTLE_AIM = False
BOTTLE_AIM_START_MM = 4500.0      # LAP の進んだ距離。ボトルの色を読み始めるのは 4300
BOTTLE_AIM_MIN_AREA = 300         # 画素の面積
BOTTLE_AIM_HITS = 3               # 正面近くに同じ色が続けて見えた画像の数
BOTTLE_AIM_START_MAX_DEG = 12.0   # 始めるのは、車体の正面からこの角度以内に見えたボトルだけ
BOTTLE_AIM_OFFSET_DEG = 0.0       # 狙う向きを右へずらす(度)。アームの真ん中が車体の正面からずれていたら使う
BOTTLE_AIM_GAIN = 1.0             # ボトルの方向のずれのうち、向きを変える割合(遅れを足し戻すので1でよい、11/18 机上)
BOTTLE_AIM_SMOOTH = 0.6           # 向かう向きを新しい値へ寄せる割合(画像ごと)
BOTTLE_AIM_LATENCY_SEC = 0.155    # 画像の遅れ。11/18 のログで約4枚(1枚約35ms)遅れていた。この分だけ前の向きを基準にする
BOTTLE_AIM_UNDER_AREA = 2500      # この面積を超えて見えたあとで見えなくなったら、アームの間に入ったとしてまっすぐ進む
BOTTLE_AIM_MAX_DEG = 15.0         # 線の向き(heading_origin)からの向きの変え幅の上限
BOTTLE_AIM_POWER = None           # None なら今の fallback と同じ出力
# 2026-11-18(devSS): 床の青→黒が見えなかったとき、推定のゲートを手前にする量(--lap-unseen-gate-shorten-mm、既定0)。
# 11/18 のAのログ: 青→黒が見えた回は 5210〜5256mm で止まり、見えなかった回は推定のゲート(5290.5)まで進んで 5291〜5304mm。
UNSEEN_GATE_SHORTEN_MM = 0.0
# 11/18 1回目: 画素→角度の式(HORIZON_DISTANCE から)が実際の約1.7倍の角度を出して ±15°で振れ、ボトルの左へ抜けた。
# ロボットAのカメラは左へ2.3°向いている。--rally-camera-calib の較正(レンズと取り付け)で車体から見た角度を出す。
_AIM_CAM = None


def _bottle_aim_prepare():
    """LAP を走り出す前に一度だけ較正を読む(走行中に読まない)。"""
    global _AIM_CAM
    try:
        from .. import et_rally_start_pose as sp
        _AIM_CAM = sp.GroundCamera(sp.DEFAULT_CALIB, build_map=False)
        print('BOTTLE_AIM calibration %s' % getattr(sp.DEFAULT_CALIB, 'name', sp.DEFAULT_CALIB), flush=True)
    except Exception as error:
        _AIM_CAM = None
        print('BOTTLE_AIM calibration unavailable (%r); using the image formula' % (error,), flush=True)


def _bottle_right_deg(cx, bottom):
    """ボトルが車体の正面から右へ何度に見えるか(画面は 640x480 の真ん中16:9 を 320x180 に縮めたもの)。"""
    if _AIM_CAM is not None:
        import cv2
        import numpy as np
        u, v = 2.0 * float(cx), 2.0 * float(bottom) + 60.0
        n = cv2.undistortPoints(np.array([[[u, v]]], np.float64), _AIM_CAM.K, _AIM_CAM.dist).reshape(2)
        ray = _AIM_CAM.R0.T @ np.array([n[0], n[1], 1.0])
        return -math.degrees(math.atan2(ray[1], ray[0])) - BOTTLE_AIM_OFFSET_DEG
    from py_etrobo_util.video import FRAME_WIDTH
    return (float(cx) - FRAME_WIDTH / 2.0) / 7.5 - BOTTLE_AIM_OFFSET_DEG

APPROACH_HITS=3

APPROACH_MAX_AGE_SEC=0.25














def lap_trace_side(direction,course):
    if direction not in (-1,1) or course not in (-1,1):raise ValueError('invalid trace direction or course')
    side=TraceSide.NORMAL if direction*(-course)==1 else TraceSide.OPPOSITE
    if trial_variant()[1]['reverse_side']:side=TraceSide.OPPOSITE if side==TraceSide.NORMAL else TraceSide.NORMAL
    return side

class BlueThenBlackGate(Behaviour):
    """Accept BLACK only after a distinct earlier BLUE sample on the final straight."""
    def __init__(self,lap, *, allow_weak_blue=False):
        super().__init__('lap gate blue then black');self.lap=lap;self.classifier=ColorClassifier()
        self.allow_weak_blue = allow_weak_blue

    def initialise(self):
        self.saw_blue=False;self.confirmed=False;self.blue_progress=None;self.black_progress=None;self.black_hsv=None;self.last_hsv=None;self.last_hsv_time=None;self.black_window_expired=False
        self.weak_blue_progress = None
        self.weak_blue_samples = 0
        self.weak_blue_last_progress = None
        self.measure_projection = None

    def _confirm(self, progress, raw_progress, hsv, source):
        self.confirmed = True
        self.black_progress = progress
        self.black_hsv = hsv
        # Anchor the post-gate distance at the BLACK sample, not at the later
        # handoff tick. The robot continues moving during that handoff.
        self.measure_projection = ProjectedDistance(runtime.plotter, self.lap.heading_origin, runtime.course)
        self.logger.info('LAP_GATE_BLUE_BLACK progress_mm=%.1f raw_progress_mm=%.1f blue_mm=%.1f hsv=%s mono_sec=%.6f source=%s'
                         % (progress, raw_progress, self.blue_progress, hsv, time.monotonic(), source))
        return Status.SUCCESS

    def _check_weak_blue(self, progress, raw_progress, hsv, color):
        if self.weak_blue_progress is not None:
            delta = progress - self.weak_blue_progress
            if WEAK_BLUE_TO_BLACK_MIN_MM <= delta <= BLUE_TO_BLACK_LIMIT_MM and color == Color.BLACK:
                self.saw_blue = True
                self.blue_progress = self.weak_blue_progress
                return self._confirm(progress, raw_progress, hsv, 'weak')
            if delta <= BLUE_TO_BLACK_LIMIT_MM:
                return Status.RUNNING
            # A confirmed weak-blue candidate remains the fallback anchor even
            # if black is not seen within the bounded search window.
            self.saw_blue = True
            self.blue_progress = self.weak_blue_progress
            self.black_window_expired = True
            self.logger.warning('LAP_GATE_WEAK_BLUE_EXPIRED blue_mm=%.1f progress_mm=%.1f' % (self.blue_progress, progress))
            return Status.RUNNING

        h, s, v = hsv
        if 205 <= h <= 215 and s >= LAP_WEAK_BLUE_MIN_S and LAP_WEAK_BLUE_MIN_V <= v < LAP_BLUE_MIN_V:
            last = self.weak_blue_last_progress
            if last is None or progress < last:
                self.weak_blue_samples = 1
            elif progress > last:
                self.weak_blue_samples += 1
            self.weak_blue_last_progress = progress
            if self.weak_blue_samples >= LAP_WEAK_BLUE_SAMPLES:
                self.weak_blue_progress = progress
                self.logger.info('LAP_GATE_WEAK_BLUE progress_mm=%.1f hsv=%s samples=%d' % (progress, hsv, self.weak_blue_samples))
                if WEAK_BLUE_MODE == 'early':
                    self.saw_blue = True
                    self.blue_progress = progress
                    self.logger.info('LAP_GATE_WEAK_BLUE_ACCEPTED progress_mm=%.1f hsv=%s samples=%d' % (progress, hsv, self.weak_blue_samples))
        else:
            self.weak_blue_samples = 0
            self.weak_blue_last_progress = None
        return Status.RUNNING

    def update(self):
        runtime.require('plotter','color_sensor')
        raw_progress=float(runtime.plotter.get_distance())-self.lap.origin_distance
        progress=self.lap.projected_progress() if getattr(self.lap,'trace_projection',None) is not None else raw_progress
        hsv=tuple(runtime.color_sensor.get_raw_color_hsv());self.last_hsv=hsv;self.last_hsv_time=time.monotonic();color=self.classifier.classify_single(*hsv)
        if not self.saw_blue:
            trial_blue=(205<=hsv[0]<=215 and hsv[1]>=60 and LAP_BLUE_MIN_V<=hsv[2]<=100)
            if trial_blue:
                self.saw_blue=True;self.blue_progress=progress
                self.logger.info('LAP_GATE_BLUE progress_mm=%.1f raw_progress_mm=%.1f hsv=%s mono_sec=%.6f' % (progress,raw_progress,hsv,time.monotonic()))
            elif self.allow_weak_blue:
                return self._check_weak_blue(progress, raw_progress, hsv, color)
            return Status.RUNNING
        if progress-self.blue_progress>BLUE_TO_BLACK_LIMIT_MM:
            if not self.black_window_expired:
                self.black_window_expired=True
                self.logger.warning('LAP_GATE_BLACK_WINDOW_EXPIRED blue_mm=%.1f progress_mm=%.1f max_mm=%.1f' % (self.blue_progress,progress,BLUE_TO_BLACK_LIMIT_MM))
            return Status.RUNNING
        if color==Color.BLACK:
            if (WEAK_BLUE_MODE == 'early' and self.weak_blue_progress is not None
                    and progress-self.blue_progress < WEAK_BLUE_TO_BLACK_MIN_MM):
                return Status.RUNNING
            return self._confirm(progress, raw_progress, hsv, 'strict')
        return Status.RUNNING

class TrialLoggedTrace(ForwardOnlyTrace):
    """Capture the actual clamped commands; emit later when the trial stops."""
    def __init__(self,owner,**kwargs):
        super().__init__(**kwargs);self.owner=owner
    def set_motor_power(self,left_power,right_power):
        heading=-runtime.course*float(runtime.gyro_sensor.get_angle())
        name,variant=trial_variant()
        near=getattr(self.owner,'approach_near',False);approach=approach_mode()
        strong=near and approach in ('heading_strong','line_half')
        scale=variant['line_scale']*(0.5 if near and approach=='line_half' else 1.0)
        line_input=(left_power-right_power)*0.5*scale
        floor_note = ''
        if dark_heading_mode() == 'on':
            h, s, v = runtime.color_sensor.get_raw_color_hsv()
            line_input = white_side_turn(h, s, v, runtime.course, self.trace_side == TraceSide.NORMAL)
            floor_note = '_DARK_HEADING_V%d_S%d' % (v, s)
        gain=4.0 if strong else variant['heading_gain'];heading_limit=20.0 if strong else 16.0
        speed=self.owner.straight_power
        left,right,error,line,correction,mode=mix_heading_trace(speed+line_input,speed-line_input,heading,self.owner.heading_origin,runtime.course,gain=gain,heading_limit=heading_limit)
        turn=(left-right)/2.0;limit=variant.get('turn_limit',50.0)
        turn=max(-limit,min(limit,turn));base=speed-abs(turn)
        left,right=round(base+turn),round(base-turn)
        mode=mode+'_'+name+('_NEAR_'+approach if near else '')+floor_note
        super().set_motor_power(left,right)
        progress=float(runtime.plotter.get_distance())-self.owner.origin_distance
        self.owner.trace_motor_samples.append((time.monotonic(),progress,left,right,heading,error,line,correction,mode))
        self.owner.trace_motor_total+=1

class PostGateApproachTrace(ForwardOnlyTrace):
    """Preserve the existing post-gate output unless a trial mode is active."""
    def __init__(self,owner,**kwargs):
        super().__init__(**kwargs);self.owner=owner
    def set_motor_power(self,left_power,right_power):
        owner=self.owner;near=getattr(owner,'approach_near',False);approach=approach_mode()
        if not near:
            return super().set_motor_power(left_power,right_power)
        heading=-runtime.course*float(runtime.gyro_sensor.get_angle());reference=float(owner.heading_origin)
        if runtime.course not in (-1,1) or not all(map(math.isfinite,(heading,reference,left_power,right_power))):
            raise ValueError('invalid post-gate heading input')
        error=(reference-heading+180.0)%360.0-180.0
        line=(left_power-right_power)*0.5;correction=0.0
        left,right=max(0,min(100,left_power)),max(0,min(100,right_power))
        if approach in ('heading_strong','line_half'):
            correction=max(-20.0,min(20.0,-runtime.course*4.0*error))
            scale=0.5 if approach=='line_half' else 1.0
            turn=max(-TRACE_POWER,min(TRACE_POWER,line*scale+correction))
            left,right=round(TRACE_POWER+turn),round(TRACE_POWER-turn)
        super().set_motor_power(left,right)
        progress=float(runtime.plotter.get_distance())-owner.origin_distance
        owner.trace_motor_samples.append((time.monotonic(),progress,left,right,heading,error,line,correction,'POST_GATE_NEAR_'+approach))
        owner.trace_motor_total+=1

class Peak100ContinuousLap(RunByGyro):
    """Use profile's PWM100 as wheel maximum; preserve output only on success."""
    def __init__(self,*args,**kwargs):
        kwargs['peak_power_steering']=True
        super().__init__(*args,**kwargs);self.handoff_complete=False;self.side_source=None;self.camera_direction=None;self.profile_target=self.target;self.pre_gate_trace=None;self.pre_gate_side=None;self.pre_gate_white_count=0;self.floor_classifier=ColorClassifier();self.trace_motor_samples=deque(maxlen=500);self.trace_motor_total=0;self.trace_projection=None;self.trace_projection_base=None;self.transition_phase=None;self.gate_latched=False;self.fallback_done=False
        self.fallback_estimated_gate_mm=float(self.distance_limit_mm)  # existing LAP distance boundary, provisional
        self.line_entry = 'baseline'
        self.gate_recovery = 'baseline'
        self.confirmed_fallback_slow = False
        self.confirmed_fallback_after_mm = FALLBACK_AFTER_GATE_MM
        self.gate_stop_measure = False
        self.final_straight_test = False
        self.cross_white_recovery = False
        self.live_camera_recovery = False
        self.profile_white_rejoin = False
        self.profile_rejoin_source = None
        self.straight_start_mm = None
        self.straight_power = 100
        self.observation_start_mm = FINAL_OBSERVATION_START_MM
        self.camera_start_mm = CAMERA_CORRECTION_START_MM
        self.camera_deadline_mm = CAMERA_CORRECTION_DEADLINE_MM

    def _initial_profile_progress(self):
        return FINAL_STRAIGHT_TEST_START_MM if self.final_straight_test else 0.0

    def initialise(self):
        self.native_capture_enabled = False
        self.aim_active = False; self.aim_hits = 0; self.aim_color = None; self.aim_last_frame = -1
        self.aim_under = False; self.aim_target = None; self.aim_logged = 0; self.aim_heads = deque(maxlen=60); self.aim_max_area = 0
        self.handoff_complete=False;self.camera_direction=None;self.camera_search_logged=False;self.target=self.profile_target;self.pre_gate_trace=None;self.pre_gate_side=None;self.pre_gate_white_count=0;self.floor_classifier=ColorClassifier();self.trace_motor_samples=deque(maxlen=500);self.trace_motor_total=0;self.trace_projection=None;self.trace_projection_base=None
        self.transition_phase=None;self.transition_start=None;self.transition_time=None;self.gate_latched=False;self.recovery_attempts=0;self.edge_count=0;self.recovery_direction=None;self.frozen_camera_frame=None;self.frozen_camera_stream=None;self.fallback_done=False;self.fallback_reason=None;self.fallback_gate_base=None;self.transition_samples=deque(maxlen=500);self.transition_total=0;self._diag_context=None;self.entry_seek_parent=None;self.approach_near=False;self.entry_dark_hold=None;self.entry_dark_hold_used=False
        self.cross_direction = None
        self.cross_saw_dark = False
        self.cross_pending_entry_dark = False
        self.unseen_estimate_started = False
        self.unseen_estimate_cancelled = False
        self.cross_initial_mid_tone = False
        self.cross_min_v = None
        self.cross_last_v = None
        self.cross_rise_count = 0
        self.cross_shallow = False
        self.early_edge_progress = None
        self.early_edge_samples = 0
        self.early_white = None
        self.early_return_direction = None
        self.fallback_gate_slow = False
        self.fallback_blue_slow_logged = False
        self.gate_stop_time = None
        self.gate_stop_counts = None
        self.live_vote_key = None
        self.live_approach_direction = 0
        self.profile_white_progress = None
        self.profile_white_samples = 0
        self.profile_rejoin_vote = None
        self.profile_rejoin_edge_distance = None
        self.initial_merge_end = None
        self.late_merge_used = False
        self.late_merge_white_count = 0
        self.late_merge_last_progress = None
        self.far_entry_hold = False
        self.far_entry_hold_end = None
        self.far_entry_disabled_reason = None
        self.far_vote_last_frame = None
        self.far_vote_grace_frame = None
        self.far_vote_grace_start = None
        self.far_vote_grace_progress = None
        self.adaptive_initial_offset = 0.0
        self.adaptive_initial_progress = None
        self.adaptive_boost_start = None
        self.adaptive_boost_cancelled = False
        self.adaptive_white_count = 0
        self.adaptive_white_progress = None

    def mark_gate_stop(self, command_time):
        if self.gate_stop_time is not None:
            return
        self.gate_stop_time = command_time
        try:
            self.gate_stop_counts = (float(runtime.left_motor.get_count()), float(runtime.right_motor.get_count()))
        except (OSError, RuntimeError, TypeError, ValueError) as exc:
            self.logger.warning('LAP_GATE_STOP_ENCODER_UNAVAILABLE reason=%s' % exc)
            return
        self.logger.info('LAP_GATE_STOP_BEGIN mono_sec=%.6f encoder_mono_sec=%.6f gate_confirmed=%s fallback=%s'
                         % (command_time, time.monotonic(), self.gate_latched, self.fallback_done))

    def _transition_start(self,phase,distance,now):
        self.transition_phase=phase;self.transition_start=distance;self.transition_time=now;self.edge_count=0
        if phase == 'REACQUIRE':
            self.reacquire_last_floor = None
            self.reacquire_floor_hits = 0
        self.logger.info('LAP_TRANSITION phase=%s raw_mm=%.1f mono_sec=%.6f' % (phase,distance-self.origin_distance,now))

    def _profile_update(self):
        # Until the correction window, execute the exact devTK drive cycle.
        # Its power update occurs after each cycle for the speed-down variant.
        if not self.running or self.camera_start_mm is None or float(runtime.plotter.get_distance()) - self.origin_distance < self.camera_start_mm:
            gate = self.completion_condition
            self.completion_condition = None
            try:
                result = DevTKRunByGyro._update_distance_target(self)
                if self.devtk_speed_config is not None:
                    progress = float(runtime.plotter.get_distance()) - self.origin_distance
                    self.power = speed_profile(progress, self.devtk_speed_config)
                    if self.straight_start_mm is not None and progress >= self.straight_start_mm:
                        self.power = min(self.power, self.straight_power)
                return result
            finally:
                self.completion_condition = gate
        gate=self.completion_condition
        if not self.running or gate is None:return super().update()
        progress=float(runtime.plotter.get_distance())-self.origin_distance
        if progress>=self.completion_min_mm and not self.gate_latched:
            gate.tick_once()
            if gate.status==Status.SUCCESS:self.gate_latched=True
        if progress>=self.distance_limit_mm:return self._start_fallback('gate_distance_limit')
        self.completion_condition=None
        try:return super().update()
        finally:self.completion_condition=gate

    def _transition_drive(self,target_heading,heading, *, slow=False):
        error=(target_heading-heading+180.0)%360.0-180.0
        strong=(not slow and self.transition_phase=='FALLBACK' and self.approach_near and approach_mode() in ('heading_strong','line_half'))
        gain,limit=(4.0,20.0) if strong else (2.0,16.0)
        turn=max(-limit,min(limit,gain*error))
        base=TRACE_POWER if slow else self.straight_power-abs(turn)
        if getattr(self, 'aim_active', False) and BOTTLE_AIM_POWER is not None:
            base=min(base, BOTTLE_AIM_POWER)
        runtime.left_motor.set_brake(False);runtime.right_motor.set_brake(False)
        left=round(base-runtime.course*turn);right=round(base+runtime.course*turn)
        runtime.left_motor.set_power(left);runtime.right_motor.set_power(right)
        context=self._diag_context
        if context is not None:
            row=dict(context,phase=self.transition_phase,command_time=time.monotonic(),target_heading=target_heading,heading=heading,left=left,right=right)
            self.transition_samples.append(row);self.transition_total+=1

    def _update_live_camera_seek(self, distance, now, hsv, heading, error):
        if self.gate_latched:
            return self._finish_distance_run(Status.SUCCESS)
        gate = self.completion_condition
        if (getattr(gate, 'saw_blue', False) or getattr(gate, 'weak_blue_progress', None) is not None
                or getattr(gate, 'weak_blue_samples', 0)):
            self._transition_drive(self.heading_origin, heading)
            return Status.RUNNING

        snapshot = self.side_source.snapshot if self.side_source is not None else None
        if snapshot and 0 <= now - snapshot['capture_time'] <= CAMERA_MAX_AGE_SEC:
            side = snapshot['side']
            direction = {'LEFT': -1, 'CENTER': 0, 'RIGHT': 1}[side]
        else:
            side, direction = None, 0
        self.camera_direction = direction
        _, saturation, value = hsv
        on_line = saturation <= 45 and value <= 82
        white = saturation <= 45 and value >= 95
        vote_key = (side, on_line, white)
        if vote_key != self.live_vote_key:
            self.logger.info('LAP_LIVE_CAMERA raw_mm=%.1f side=%s frame=%s hsv=%s heading=%.2f geometry=%s'
                             % (distance - self.origin_distance, side,
                                snapshot['frame'] if side is not None else None, hsv, heading,
                                snapshot.get('geometry') if side is not None else None))
            self.live_vote_key = vote_key

        if on_line:
            if (self.live_approach_direction and self.live_approach_direction == direction
                    and value <= LINE_CROSS_DARK_V and self.line_entry == 'cross_rejoin'
                    and -error * direction * (-runtime.course) > 0):
                self.frozen_camera_frame = snapshot['frame']
                self.frozen_camera_stream = snapshot['stream']
                self._start_line_cross(distance, now, hsv, direction, 'camera', snapshot['frame'])
                return self._update_line_cross(distance, now, hsv, heading, error)
            if direction and trace_target_v() <= value <= 82 and abs(error) <= LIVE_CAMERA_ALIGNMENT_DEG:
                self.frozen_camera_frame = snapshot['frame']
                self.frozen_camera_stream = snapshot['stream']
                return self._start_pre_gate_trace(hsv, direction)
            self.live_approach_direction = 0
        else:
            self.live_approach_direction = direction if white else 0
        target = self.heading_origin
        if white and direction:
            offset = abs(snapshot['x'] - lap_camera_reference_x())
            angle = min(LIVE_CAMERA_MAX_HEADING_DEG, offset * LIVE_CAMERA_HEADING_DEG_PER_PX)
            target += direction * (-runtime.course) * angle
        self._transition_drive(target, heading)
        return Status.RUNNING

    def _start_pre_gate_trace(self, hsv, direction):
        self.pre_gate_side = lap_trace_side(direction, runtime.course)
        self.pre_gate_trace = TrialLoggedTrace(owner=self, name='pre-gate line trace', target=trace_target_v(),
            power=self.straight_power, power_min=self.straight_power, pid_p=.50, pid_i=.000001, pid_d=.045, trace_side=self.pre_gate_side)
        self.transition_phase = 'TRACE'
        self.pre_gate_white_count = 0
        self.logger.info('LAP_TRANSITION phase=TRACE hsv=%s direction=%d side=%s gate_latched=%s'
                         % (hsv, direction, self.pre_gate_side.name, self.gate_latched))
        if self.gate_latched:
            return self._finish_distance_run(Status.SUCCESS)
        return self.pre_gate_trace.update()

    def _profile_rejoin_pending_gate(self):
        gate = self.completion_condition
        return bool(gate and (getattr(gate, 'saw_blue', False)
                    or getattr(gate, 'weak_blue_progress', None) is not None
                    or getattr(gate, 'weak_blue_samples', 0) > 0))

    def _update_adaptive_peak(self, progress, hsv, now):
        """Add at most two degrees for a far initial vote confirmed on white."""
        if self.adaptive_initial_progress is None or self.adaptive_boost_cancelled:
            return
        h, s, value = hsv
        white = s <= 45 and value >= 95
        if self.adaptive_boost_start is not None:
            if not white or self._profile_rejoin_pending_gate():
                self.adaptive_boost_cancelled = True
                self.logger.info('LAP_ADAPTIVE_PEAK_END progress_mm=%.1f reason=%s'
                                 % (progress, 'floor_not_white' if not white else 'blue_candidate'))
            return
        if not white:
            self.adaptive_white_count = 0;self.adaptive_white_progress = None
            return
        if self.adaptive_white_progress is None or progress < self.adaptive_white_progress:
            self.adaptive_white_count = 1
        elif progress > self.adaptive_white_progress:
            self.adaptive_white_count += 1
        self.adaptive_white_progress = progress
        if (far_peak_deg() <= merge_peak() or self.adaptive_initial_offset < ADAPTIVE_INITIAL_OFFSET_PX
                or self.adaptive_white_count < 3 or self._profile_rejoin_pending_gate()
                or progress > self.adaptive_initial_progress + merge_length() / 2
                or self.distance_limit_mm - progress < 200.0):
            return
        source = self.profile_rejoin_source
        snapshot = source.snapshot if source is not None else None
        expected = 'LEFT' if self.camera_direction < 0 else 'RIGHT'
        if (not snapshot or snapshot.get('side') != expected
                or snapshot.get('stream') != self.frozen_camera_stream
                or not isinstance(snapshot.get('frame'), int) or self.frozen_camera_frame is None
                or snapshot['frame'] <= self.frozen_camera_frame
                or not 0 <= now - snapshot.get('capture_time', -math.inf) <= CAMERA_MAX_AGE_SEC):
            return
        self.adaptive_boost_start = progress
        selected_peak = min(far_peak_deg(), merge_peak() + 2.0)
        self.logger.info('LAP_ADAPTIVE_PEAK_START progress_mm=%.1f initial_offset_px=%.1f side=%s frame=%d target_deg=%.1f ramp_mm=%.1f'
                         % (progress,self.adaptive_initial_offset,expected,snapshot['frame'],selected_peak,ADAPTIVE_RAMP_MM))

    def _maybe_start_late_merge(self, progress, hsv, now):
        """Retry once only after the original merge, before any line or gate handoff."""
        if (self.late_merge_used or self.initial_merge_end is None
                or progress < self.initial_merge_end or self.camera_direction not in (-1, 1)
                or self._profile_rejoin_pending_gate()):
            return False
        _, saturation, value = hsv
        if saturation > 45 or value < 95:
            self.late_merge_white_count = 0
            self.late_merge_last_progress = None
            return False
        if self.late_merge_last_progress is None or progress < self.late_merge_last_progress:
            self.late_merge_white_count = 1
        elif progress > self.late_merge_last_progress:
            self.late_merge_white_count += 1
        self.late_merge_last_progress = progress
        if self.late_merge_white_count < 3 or progress + LATE_MERGE_LENGTH_MM > self.distance_limit_mm:
            return False
        snapshot = self.side_source.snapshot if self.side_source is not None else None
        expected = 'LEFT' if self.camera_direction < 0 else 'RIGHT'
        if (not snapshot or snapshot.get('side') != expected
                or snapshot.get('stream') != self.frozen_camera_stream
                or not isinstance(snapshot.get('frame'), int)
                or self.frozen_camera_frame is None or snapshot['frame'] <= self.frozen_camera_frame
                or not 0 <= now - snapshot.get('capture_time', -math.inf) <= CAMERA_MAX_AGE_SEC
                or not isinstance(snapshot.get('x'), (int, float))
                or not math.isfinite(snapshot['x'])
                or self.camera_direction * (snapshot['x'] - lap_camera_reference_x()) < LATE_MERGE_MIN_OFFSET_PX):
            return False
        start, direction, original, course = progress, self.camera_direction, self.profile_target, runtime.course
        if course not in (-1, 1):
            return False
        def late_target(distance):
            phase = (distance - start) / LATE_MERGE_LENGTH_MM
            offset = direction * (-course) * LATE_MERGE_PEAK_DEG * math.sin(math.pi * phase) ** 2 if 0 <= phase <= 1 else 0.0
            return original(distance) + offset
        self.target = late_target
        self.late_merge_used = True
        self.logger.info('LAP_LATE_MERGE_START progress_mm=%.1f side=%s x=%.1f frame=%d peak_deg=%.1f length_mm=%.1f'
                         % (progress, expected, snapshot['x'], snapshot['frame'], LATE_MERGE_PEAK_DEG, LATE_MERGE_LENGTH_MM))
        return True

    def _maybe_start_profile_rejoin(self, progress, hsv):
        _, saturation, value = hsv
        if saturation > 45 or value < 95:
            self.profile_white_progress = None
            self.profile_white_samples = 0
            return None
        if self.profile_white_progress is None or progress < self.profile_white_progress:
            self.profile_white_samples = 1
        elif progress > self.profile_white_progress:
            self.profile_white_samples += 1
        self.profile_white_progress = progress
        if self.far_entry_hold:
            return None
        if (self.profile_white_samples < PROFILE_REJOIN_WHITE_SAMPLES
                or self.camera_direction not in (-1, 0, 1) or self._profile_rejoin_pending_gate()
                or progress >= self.distance_limit_mm - RECOVERY_LIMIT_MM):
            return None

        now = time.monotonic()
        snapshot = self.profile_rejoin_source.snapshot
        center_origin = self.camera_direction == 0
        if center_origin:
            if (not snapshot or snapshot.get('side') not in ('LEFT', 'RIGHT')
                    or not isinstance(snapshot.get('x'), (int, float))
                    or not math.isfinite(snapshot['x'])
                    or abs(snapshot['x'] - lap_camera_reference_x()) < 20.0):
                return None
            opposite = snapshot['side']
        else:
            opposite = 'RIGHT' if self.camera_direction < 0 else 'LEFT'
        if (snapshot is None or snapshot['side'] != opposite
                or snapshot['stream'] != self.frozen_camera_stream
                or snapshot['frame'] <= self.frozen_camera_frame
                or not 0 <= now - snapshot['capture_time'] <= CAMERA_MAX_AGE_SEC):
            return None
        if not center_origin and far_entry_policy() == 'evidence' and self.far_entry_disabled_reason is None:
            gyro = float(runtime.gyro_sensor.get_angle())
            heading = -runtime.course * gyro
            toward = self.camera_direction * (-runtime.course) * ((heading - self.heading_origin + 180.0) % 360.0 - 180.0)
            if (math.isfinite(toward) and toward >= FAR_ENTRY_MIN_HEADING_DEG
                    and self.far_entry_hold_end is not None and progress < self.far_entry_hold_end):
                self.far_entry_hold = True
                self.logger.info('LAP_FAR_ENTRY_HOLD_START source=white_heading progress_mm=%.1f heading=%.1f toward=%.1f side=%s frame=%d white_samples=%d hold_until_mm=%.1f'
                                 % (progress, heading, toward, snapshot['side'], snapshot['frame'], self.profile_white_samples, self.far_entry_hold_end))
                return None
        distance = float(runtime.plotter.get_distance())
        try:
            projection = ProjectedDistance(runtime.plotter, self.heading_origin, runtime.course)
        except ValueError:
            return self._finish_distance_run(Status.FAILURE)
        self.trace_projection = projection
        self.trace_projection_base = progress
        self.profile_rejoin_vote = snapshot
        self.target = self.profile_target
        self.logger.info('LAP_PROFILE_WHITE_REJOIN_START raw_mm=%.1f from_direction=%d to_side=%s frame=%d white_samples=%d'
                         % (progress, self.camera_direction, opposite, snapshot['frame'], self.profile_white_samples))
        self._transition_start('PROFILE_REJOIN_ALIGN', distance, now)
        return self._transition_update()

    def _update_profile_rejoin(self, distance, now, hsv, heading, error):
        if self.gate_latched:
            return self._finish_distance_run(Status.SUCCESS)
        if self._profile_rejoin_pending_gate():
            self._transition_drive(self.heading_origin, heading)
            return Status.RUNNING
        aligning = self.transition_phase == 'PROFILE_REJOIN_ALIGN'
        delta = distance - self.transition_start
        elapsed = now - self.transition_time
        limit_mm = ALIGN_LIMIT_MM if aligning else RECOVERY_LIMIT_MM
        limit_sec = ALIGN_LIMIT_SEC if aligning else RECOVERY_LIMIT_SEC
        if not 0 <= delta < limit_mm or not 0 <= elapsed < limit_sec:
            reason = 'profile_rejoin_align_bound' if aligning else 'profile_rejoin_seek_bound'
            return self._start_fallback(reason)
        initial_vote = self.profile_rejoin_vote
        snapshot = self.profile_rejoin_source.snapshot
        supported = (snapshot is not None and snapshot['side'] == initial_vote['side']
                     and snapshot['stream'] == initial_vote['stream']
                     and snapshot['frame'] > initial_vote['frame']
                     and snapshot['capture_time'] > initial_vote['capture_time']
                     and 0 <= now - snapshot['capture_time'] <= CAMERA_MAX_AGE_SEC)
        _, saturation, value = hsv
        on_line = saturation <= 45 and value <= 82
        if aligning and (abs(error) > PROFILE_REJOIN_ALIGNED_DEG or not supported) and not on_line:
            self._transition_drive(self.heading_origin, heading)
            return Status.RUNNING
        if aligning:
            self._transition_start('PROFILE_REJOIN_SEEK', distance, now)

        direction = 1 if initial_vote['side'] == 'RIGHT' else -1
        signed_direction = direction * (-runtime.course)
        if on_line:
            if (not aligning and supported and value <= LINE_CROSS_DARK_V
                    and self.line_entry == 'cross_rejoin' and -error * signed_direction > 0):
                self.camera_direction = direction
                self.frozen_camera_frame = snapshot['frame']
                self.frozen_camera_stream = snapshot['stream']
                self._start_line_cross(distance, now, hsv, direction, 'profile_rejoin')
                return self._update_line_cross(distance, now, hsv, heading, error)
            if self.profile_rejoin_edge_distance is not None and distance > self.profile_rejoin_edge_distance:
                self._transition_start('GATE_REJOIN', distance, now)
                self.logger.info('LAP_PROFILE_WHITE_REJOIN_FLOOR raw_mm=%.1f hsv=%s' % (distance - self.origin_distance, hsv))
            self.profile_rejoin_edge_distance = distance
            self._transition_drive(self.heading_origin, heading)
            return Status.RUNNING

        self.profile_rejoin_edge_distance = None
        white = saturation <= 45 and value >= 95
        adjustment = signed_direction * RECOVERY_PEAK_DEG if white and supported else 0.0
        self._transition_drive(self.heading_origin + adjustment, heading)
        return Status.RUNNING

    def _observe_early_line_exit(self, progress, gyro, hsv):
        if self.transition_phase is not None or self.camera_direction not in (None, 0):
            return
        _, saturation, value = hsv
        last_edge = self.early_edge_progress
        if saturation <= 45 and value <= EARLY_EXIT_EDGE_MAX_V and progress != last_edge:
            recent_edge = last_edge is not None and 0 < progress - last_edge <= EARLY_EXIT_MEMORY_MM
            self.early_edge_samples = 2 if recent_edge else 1
            self.early_edge_progress = progress
        heading = -runtime.course * gyro
        relative = (heading - self.heading_origin + 180.0) % 360.0 - 180.0
        if (saturation > 45 or value < 95 or self.early_edge_samples < 2
                or not 0 < progress - self.early_edge_progress <= EARLY_EXIT_MEMORY_MM
                or not math.isfinite(relative) or abs(relative) < EARLY_EXIT_MIN_HEADING_DEG):
            self.early_white = None
            self.early_return_direction = None
            return
        direction = runtime.course * (1 if relative > 0 else -1)
        previous = self.early_white
        if previous == (progress, direction):
            return
        self.early_return_direction = direction if previous and previous[1] == direction and progress > previous[0] else None
        self.early_white = (progress, direction)

    def _start_line_cross(self, distance, now, hsv, direction, source, frame=None):
        self.cross_direction = direction
        self.cross_entry_frame = frame if source == 'camera' else None
        self.cross_dark_observed = hsv[1] <= 45 and hsv[2] <= LINE_CROSS_DARK_V
        self.cross_pending_entry_dark = bool(source == 'dark_entry' and self.cross_dark_observed)
        # V40..60 alone is not proof of black: require a later white run and a
        # fresh camera vote on the other side before initiating recovery.
        self.cross_initial_mid_tone = bool(source == 'camera' and isinstance(frame, int)
                                          and hsv[1] <= 45 and 40 <= hsv[2] <= 60)
        self.cross_white_samples = 0
        self.cross_last_white_distance = None
        self._transition_start('CROSS', distance, now)
        self.logger.info('LAP_LINE_CROSS_START progress_mm=%.1f hsv=%s direction=%d frame=%s source=%s'
                         % (distance - self.origin_distance, hsv, direction, frame, source))
        if self.cross_initial_mid_tone:
            self.logger.info('LAP_MID_ENTRY_CANDIDATE v=%d frame=%d; await white and opposite camera vote' % (hsv[2], frame))

    def _update_early_return(self, distance, now, hsv, heading, error):
        if self.gate_latched:
            return self._finish_distance_run(Status.SUCCESS)
        delta = distance - self.transition_start
        elapsed = now - self.transition_time
        if not 0 <= delta < LINE_CROSS_LIMIT_MM or not 0 <= elapsed < ALIGN_LIMIT_SEC:
            return self._start_fallback('early_return_bound')
        if getattr(self.completion_condition, 'saw_blue', False):
            self._transition_drive(self.heading_origin, heading)
            return Status.RUNNING
        direction = self.early_return_direction * (-runtime.course)
        if -error * direction > 0 and self.floor_classifier.classify_single(*hsv) == Color.BLACK:
            self._start_line_cross(distance, now, hsv, self.early_return_direction, 'early_return')
            return self._update_line_cross(distance, now, hsv, heading, error)
        self._transition_drive(self.heading_origin + direction * RECOVERY_PEAK_DEG, heading)
        return Status.RUNNING

    def _update_line_cross(self, distance, now, hsv, heading, error):
        if self.gate_latched:
            return self._finish_distance_run(Status.SUCCESS)
        delta = distance - self.transition_start
        elapsed = now - self.transition_time
        if not 0 <= delta < LINE_CROSS_LIMIT_MM or not 0 <= elapsed < ALIGN_LIMIT_SEC:
            self.logger.warning('LAP_LINE_CROSS_BOUND raw_mm=%.1f elapsed=%.3f dark_seen=%s' % (delta, elapsed, self.cross_saw_dark))
            return self._start_fallback('line_cross_bound')
        gate = self.completion_condition
        if (getattr(gate, 'saw_blue', False)
                or (self.live_camera_recovery and (getattr(gate, 'weak_blue_progress', None) is not None
                                                   or getattr(gate, 'weak_blue_samples', 0)))):
            self._transition_drive(self.heading_origin, heading)
            return Status.RUNNING

        if ((self.cross_white_recovery or self.live_camera_recovery or self.cross_initial_mid_tone) and self.cross_entry_frame is not None
                and self._cross_needs_recovery(distance, now, hsv)):
            snapshot = self.side_source.snapshot
            self.logger.info('LAP_CROSS_WHITE_RECOVERY raw_mm=%.1f cross_mm=%.1f white_samples=%d '
                             'from_direction=%d to_side=%s frame=%d hsv=%s'
                             % (distance - self.origin_distance, delta, self.cross_white_samples,
                                self.cross_direction, snapshot['side'], snapshot['frame'], hsv))
            return self._start_camera_recovery(distance, now)
        if (self.live_camera_recovery and self.cross_white_samples >= LINE_CROSS_RECOVERY_WHITE_SAMPLES
                and not self._recovery_snapshot_valid(now)):
            self._transition_drive(self.heading_origin, heading)
            return Status.RUNNING

        direction = self.cross_direction * (-runtime.course)
        toward_exit = -error * direction
        if toward_exit < 0:
            self.cross_saw_dark = False
            self.cross_min_v = None
            self.cross_last_v = None
            self.cross_rise_count = 0
            self.cross_shallow = False
        if toward_exit > 0:
            _, saturation, value = hsv
            target_v = trace_target_v()
            if not self.cross_saw_dark and saturation <= 45 and value <= min(LINE_CROSS_DARK_V, target_v - 5):
                self.cross_saw_dark = True
                self.cross_pending_entry_dark = False
                self.logger.info('LAP_LINE_CROSS_DARK raw_mm=%.1f hsv=%s heading=%.2f' % (delta, hsv, heading))
            elif self.cross_saw_dark and saturation <= 45 and target_v <= value <= 82 and abs(error) <= entry_heading_limit():
                self.logger.info('LAP_LINE_CROSS_EXIT raw_mm=%.1f hsv=%s heading=%.2f direction=%d'
                                 % (delta, hsv, heading, self.cross_direction))
                return self._start_pre_gate_trace(hsv, self.cross_direction)
            elif (dark_entry_edge_mode() == 'on' and self.cross_pending_entry_dark
                  and delta <= 120.0 and elapsed <= 0.4 and saturation <= 45
                  and target_v <= value <= 82 and abs(error) <= entry_heading_limit()
                  and not self._profile_rejoin_pending_gate()
                  and self._recovery_snapshot_valid(now)
                  and self.side_source.snapshot.get('side') == ('LEFT' if self.cross_direction < 0 else 'RIGHT')):
                self.cross_pending_entry_dark = False
                self.logger.info('LAP_DARK_ENTRY_EDGE_EXIT raw_mm=%.1f hsv=%s heading=%.2f direction=%d frame=%s'
                                 % (delta, hsv, heading, self.cross_direction, self.side_source.snapshot['frame']))
                return self._start_pre_gate_trace(hsv, self.cross_direction)
            if self.line_entry in ('cross_soft', 'cross_rejoin') and self.cross_saw_dark and not self.cross_shallow:
                self._check_line_cross_brightness_rise(saturation, value, target_v, delta, heading)
        angle = LINE_CROSS_EXIT_HEADING_DEG if self.cross_shallow else RECOVERY_PEAK_DEG
        self._transition_drive(self.heading_origin + direction * angle, heading)
        return Status.RUNNING

    def _cross_needs_recovery(self, distance, now, hsv):
        _, saturation, value = hsv
        if saturation <= 45 and value <= min(LINE_CROSS_DARK_V, trace_target_v() - 5):
            self.cross_dark_observed = True
        if ((self.cross_dark_observed and not self.live_camera_recovery) or self.recovery_attempts
                or getattr(self.completion_condition, 'weak_blue_progress', None) is not None):
            return False
        if saturation > 45 or value < 95:
            self.cross_white_samples = 0
            self.cross_last_white_distance = None
            return False
        previous = self.cross_last_white_distance
        if previous is None or distance > previous:
            self.cross_white_samples += 1
        elif distance < previous:
            self.cross_white_samples = 1
        self.cross_last_white_distance = distance
        if self.cross_white_samples < LINE_CROSS_RECOVERY_WHITE_SAMPLES or not self._recovery_snapshot_valid(now):
            return False
        snapshot = self.side_source.snapshot
        opposite = 'RIGHT' if self.cross_direction < 0 else 'LEFT'
        return (snapshot['side'] == opposite and snapshot['frame'] > self.cross_entry_frame
                and snapshot['capture_time'] > self.transition_time)

    def _check_line_cross_brightness_rise(self, saturation, value, target_v, delta, heading):
        if saturation > 45 or value >= target_v:
            self.cross_last_v = None
            self.cross_rise_count = 0
            return
        self.cross_min_v = value if self.cross_min_v is None else min(self.cross_min_v, value)
        if self.cross_last_v is not None and value > self.cross_last_v:
            self.cross_rise_count += 1
        else:
            self.cross_rise_count = 0
        self.cross_last_v = value
        if self.cross_rise_count >= 2 and value - self.cross_min_v >= LINE_CROSS_BRIGHTNESS_RISE:
            self.cross_shallow = True
            self.logger.info('LAP_LINE_CROSS_SHALLOW raw_mm=%.1f v=%d min_v=%d heading=%.2f target_offset_deg=%.1f'
                             % (delta, value, self.cross_min_v, heading, self.cross_direction * (-runtime.course) * LINE_CROSS_EXIT_HEADING_DEG))

    def _capture_transition_context(self,now,raw,progress,gyro,hsv,sensor_time):
        session=getattr(self.side_source,'session',None)
        if session is not None:
            with session.lock:observation=session.latest
        else:observation=None
        vote=self.side_source.snapshot if self.side_source is not None else None
        obs=observation if isinstance(observation,dict) else {}
        line=obs.get('lap_line') or {}
        self._diag_context={'phase':self.transition_phase,'control_time':now,'sensor_time':sensor_time,'raw_mm':raw,'projected_mm':progress,'gyro_deg':gyro,'hsv':hsv,
            'frame':obs.get('frame'),'stream_id':obs.get('stream_id'),'session':obs.get('session'),'capture_time':obs.get('capture_time'),'image':obs.get('image'),'line_x':line.get('x_px'),'line_valid':line.get('valid'),
            'vote_frame':vote.get('frame') if isinstance(vote,dict) else None,'vote_side':vote.get('side') if isinstance(vote,dict) else None,'gate_latched':self.gate_latched}

    def _transition_update(self):
        now=time.monotonic();distance=float(runtime.plotter.get_distance());gyro=float(runtime.gyro_sensor.get_angle())
        if runtime.course not in (-1,1) or not all(map(math.isfinite,(now,distance,gyro))):return self._finish_distance_run(Status.FAILURE)
        raw=distance-self.origin_distance
        if raw<0:return self._finish_distance_run(Status.FAILURE)
        try:self.trace_projection.update(runtime.plotter)
        except ValueError:return self._finish_distance_run(Status.FAILURE)
        progress=self.projected_progress()
        gate_read=False
        if not self.gate_latched and self.completion_condition is not None:
            self.completion_condition.tick_once()
            gate_read=True
            if self.completion_condition.status==Status.SUCCESS:
                self.gate_latched=True;self.logger.info('LAP_TRANSITION gate_latched=True progress_mm=%.1f' % progress)
        plan = self._unseen_estimate_plan()
        gate = self.completion_condition
        blue_evidence = self._unseen_blue_evidence(gate)
        if self.unseen_estimate_started and blue_evidence and not self.unseen_estimate_cancelled:
            self.unseen_estimate_cancelled = True
            self.logger.info('LAP_UNSEEN_ESTIMATE_CANCEL reason=blue_evidence progress_mm=%.1f' % progress)
        if (self.transition_phase != 'FALLBACK' and plan and not self.gate_latched
                and not blue_evidence and progress >= plan[0]):
            return self._start_fallback('unseen_estimate')
        if self.transition_phase=='FALLBACK':
            if self.gate_latched and self.fallback_gate_base is None:
                self.fallback_gate_base=float(self.completion_condition.black_progress)
                self.logger.info('LAP_FALLBACK_GATE source=blue_then_black progress_mm=%.1f' % self.fallback_gate_base)
            gate=self.completion_condition
            if self.fallback_gate_base is not None:
                base=self.fallback_gate_base;source='blue_then_black'
            elif gate is not None and getattr(gate,'saw_blue',False) and getattr(gate,'blue_progress',None) is not None:
                blue=float(gate.blue_progress)
                if not math.isfinite(blue):return self._finish_distance_run(Status.FAILURE)
                # Blue without a confirmed black uses the same estimated gate
                # boundary as lost-line fallback; blue still requests slow drive.
                base=self.fallback_estimated_gate_mm;source='estimated_blue_unconfirmed'
            else:
                base=self.fallback_estimated_gate_mm;source='estimated'
            slow_confirmed = self.confirmed_fallback_slow and self.fallback_gate_base is not None
            slow_blue = self.confirmed_fallback_slow and gate is not None and getattr(gate, 'saw_blue', False)
            if (source == 'estimated' and plan and not self.unseen_estimate_cancelled
                    and not blue_evidence and progress >= plan[0]):
                if not self.unseen_estimate_started:
                    if progress > plan[1] - UNSEEN_ESTIMATE_MIN_SLOW_MM:
                        self.unseen_estimate_cancelled = True
                        self.logger.warning('LAP_UNSEEN_ESTIMATE_SKIPPED reason=late progress_mm=%.1f target_mm=%.1f' % (progress,plan[1]))
                    else:
                        self.unseen_estimate_started = True
                        self.logger.info('LAP_UNSEEN_ESTIMATE_SLOW progress_mm=%.1f target_mm=%.1f power=%d' % (progress,plan[1],TRACE_POWER))
                if self.unseen_estimate_started and progress >= plan[1]:
                    self.fallback_done = True
                    self.logger.warning('LAP_UNSEEN_ESTIMATE_DONE progress_mm=%.1f target_mm=%.1f blue_confirmed=False' % (progress,plan[1]))
                    return self._finish_distance_run(Status.SUCCESS)
            after_mm = (self.confirmed_fallback_after_mm if slow_confirmed else
                        UNSEEN_GATE_FALLBACK_AFTER_MM if source in ('estimated', 'estimated_blue_unconfirmed') else FALLBACK_AFTER_GATE_MM)
            if slow_confirmed:
                after_mm, approach_reason = confirmed_black_approach_mm(gate, self.confirmed_fallback_after_mm)
                if not self.fallback_gate_slow:
                    self.logger.info('LAP_GATE_APPROACH_PLAN source=fallback reason=%s blue_mm=%s black_mm=%s after_black_mm=%.1f total_limit_mm=%.1f'
                                     % (approach_reason, getattr(gate,'blue_progress',None), getattr(gate,'black_progress',None), after_mm, blue_unconfirmed_limit_mm()))
            traveled_after = progress - base
            measure_projection = getattr(gate, 'measure_projection', None)
            if slow_confirmed and measure_projection is not None:
                try:
                    traveled_after = measure_projection.update(runtime.plotter)
                except ValueError:
                    return self._finish_distance_run(Status.FAILURE)
            if traveled_after >= after_mm:
                self.fallback_done=True
                self.logger.info('LAP_FALLBACK_DONE source=%s progress_mm=%.1f gate_base_mm=%.1f after_mm=%.1f' % (source,progress,base,traveled_after))
                return self._finish_distance_run(Status.SUCCESS)
            if gate_read and getattr(self.completion_condition,'last_hsv',None) is not None:
                diagnostic_hsv=self.completion_condition.last_hsv;diagnostic_time=self.completion_condition.last_hsv_time
            else:
                diagnostic_hsv=tuple(runtime.color_sensor.get_raw_color_hsv());diagnostic_time=time.monotonic()
            self._capture_transition_context(now,raw,progress,gyro,diagnostic_hsv,diagnostic_time)
            if slow_confirmed and not self.fallback_gate_slow:
                self.fallback_gate_slow = True
                self.logger.info('LAP_FALLBACK_GATE_SLOW power=%d gate_mm=%.1f target_after_mm=%.1f'
                                 % (TRACE_POWER, self.fallback_gate_base, after_mm))
            if slow_blue and not slow_confirmed and not getattr(self, 'fallback_blue_slow_logged', False):
                self.fallback_blue_slow_logged = True
                self.logger.info('LAP_FALLBACK_BLUE_SLOW power=%d blue_mm=%.1f gate_confirmed=False'
                                 % (TRACE_POWER, gate.blue_progress))
            self._transition_drive(self._bottle_aim_heading(-runtime.course*gyro),-runtime.course*gyro, slow=slow_confirmed or slow_blue or self.unseen_estimate_started)
            return Status.RUNNING
        if self.transition_phase == 'GATE_REJOIN' and self.gate_latched:
            return self._finish_distance_run(Status.SUCCESS)
        if progress>=self.distance_limit_mm:return self._start_fallback('gate_distance_limit')
        hsv=tuple(runtime.color_sensor.get_raw_color_hsv());h,s,v=hsv
        sensor_time=time.monotonic();self._capture_transition_context(now,raw,progress,gyro,hsv,sensor_time)
        heading=-runtime.course*gyro;error=(self.heading_origin-heading+180.0)%360.0-180.0
        if self.transition_phase == 'ENTRY_RETURN':
            return self._update_early_return(distance, now, hsv, heading, error)
        if self.transition_phase == 'CAMERA_SEEK':
            return self._update_live_camera_seek(distance, now, hsv, heading, error)
        if self.transition_phase in ('PROFILE_REJOIN_ALIGN', 'PROFILE_REJOIN_SEEK'):
            return self._update_profile_rejoin(distance, now, hsv, heading, error)
        if self.transition_phase == 'CROSS':
            return self._update_line_cross(distance, now, hsv, heading, error)
        if self.transition_phase == 'GATE_REJOIN':
            self.edge_count = self.edge_count + 1 if s <= 45 and v >= 95 else 0
            if self.edge_count >= 3 and not getattr(self.completion_condition, 'saw_blue', False):
                return self._start_fallback('gate_rejoin_lost_floor')
            self._transition_drive(self.heading_origin, heading)
            return Status.RUNNING
        edge=(s<=45 and 40<=v<=82)
        if (self.transition_phase in ('ALIGN', 'ENTRY_SEEK') and s <= 45
                and v <= LINE_CROSS_DARK_V and abs(error) <= entry_heading_limit()
                and self._recovery_snapshot_valid(now) and not self._profile_rejoin_pending_gate()):
            snapshot = self.side_source.snapshot
            direction = {'LEFT': -1, 'RIGHT': 1}[snapshot['side']]
            self.logger.info('LAP_DARK_CROSS_START hsv=%s frame=%s side=%s' % (hsv, snapshot['frame'], snapshot['side']))
            self.frozen_camera_frame = snapshot['frame']
            self.frozen_camera_stream = snapshot['stream']
            self._start_line_cross(distance, now, hsv, direction, 'dark_entry', snapshot['frame'])
            return self._update_line_cross(distance, now, hsv, heading, error)
        if self.transition_phase=='ENTRY_SEEK':
            # Left course: preserve the observed approach heading briefly when
            # the floor has become deeply dark, instead of straightening on it.
            if (not self.entry_dark_hold_used and runtime.course == 1 and s <= 45
                    and v <= LINE_CROSS_DARK_V and not self._profile_rejoin_pending_gate()
                    and 2.0 <= (heading-self.heading_origin)*self.camera_direction*(-runtime.course) <= 14.0):
                self.entry_dark_hold=(distance,now,heading)
                self.entry_dark_hold_used=True
                self.logger.info('LAP_ENTRY_DARK_HOLD_START raw_mm=%.1f heading=%.1f hsv=%s limit_mm=30.0' % (raw,heading,hsv))
            if self.entry_dark_hold is not None:
                start,started,target=self.entry_dark_hold
                traveled=distance-start;held=now-started
                if 0 <= traveled < 30.0 and 0 <= held < 0.15 and s <= 45 and v <= 45 and not self._profile_rejoin_pending_gate():
                    self._transition_drive(target,heading)
                    return Status.RUNNING
                self.logger.info('LAP_ENTRY_DARK_HOLD_END raw_mm=%.1f traveled_mm=%.1f elapsed=%.3f hsv=%s' % (raw,traveled,held,hsv))
                self.entry_dark_hold=None
            delta=distance-self.transition_start;elapsed=now-self.transition_time
            parent=self.entry_seek_parent
            parent_expired=parent is not None and (distance-parent[0]>=RECOVERY_LIMIT_MM or now-parent[1]>=RECOVERY_LIMIT_SEC or distance<parent[0] or now<parent[1])
            # If the ordinary vote is missing, a fresh white-sided vote may
            # select the trace edge only while the floor itself is at an edge.
            if (white_sided_rejoin_enabled() and not parent_expired
                    and 0 <= delta < ENTRY_SIDE_LIMIT_MM and 0 <= elapsed < ENTRY_SIDE_LIMIT_SEC
                    and edge and abs(error) <= entry_heading_limit()
                    and not self._profile_rejoin_pending_gate()
                    and not self._recovery_snapshot_valid(now)):
                candidate = self._white_sided_snapshot(now)
                if candidate is not None:
                    direction = {'LEFT': -1, 'RIGHT': 1}[candidate['side']]
                    self.logger.info('LAP_WHITE_SIDED_ENTRY frame=%s side=%s x=%s hsv=%s heading=%.2f' %
                                     (candidate['frame'], candidate['side'], candidate.get('x'), hsv, heading))
                    return self._start_pre_gate_trace(hsv, direction)
            if delta<0 or elapsed<0 or delta>=ENTRY_SIDE_LIMIT_MM or elapsed>=ENTRY_SIDE_LIMIT_SEC or parent_expired:
                self.logger.warning('LAP_ENTRY_SEEK_UNAVAILABLE reason=bound raw_mm=%.1f elapsed=%.3f' % (delta,elapsed))
                # A single decision on the boundary tick, not an extended
                # search. Keep a small cap for normal sampling overshoot.
                if (white_sided_rejoin_enabled() and not parent_expired
                        and 0 <= delta <= ENTRY_SIDE_LIMIT_MM + 20.0
                        and 0 <= elapsed <= ENTRY_SIDE_LIMIT_SEC + 0.04
                        and s <= 45 and (v <= LINE_CROSS_DARK_V or edge)
                        and abs(error) <= entry_heading_limit()
                        and not self._profile_rejoin_pending_gate()):
                    candidate = self._white_sided_snapshot(now)
                    if candidate is not None:
                        direction = {'LEFT': -1, 'RIGHT': 1}[candidate['side']]
                        self.logger.info('LAP_WHITE_SIDED_BOUND frame=%s side=%s hsv=%s heading=%.2f raw_mm=%.1f elapsed=%.3f' %
                                         (candidate['frame'], candidate['side'], hsv, heading, delta, elapsed))
                        if v <= LINE_CROSS_DARK_V:
                            self.frozen_camera_frame = candidate['frame']
                            self.frozen_camera_stream = candidate['stream']
                            self._start_line_cross(distance, now, hsv, direction, 'dark_entry', candidate['frame'])
                            return self._update_line_cross(distance, now, hsv, heading, error)
                        return self._start_pre_gate_trace(hsv, direction)
                if (delta >= 0 and elapsed >= 0 and not parent_expired and s <= 45 and v >= 95
                        and self._recovery_snapshot_valid(now) and self.recovery_attempts < 1):
                    self.logger.info('LAP_ENTRY_SEEK_RETRY source=fresh_camera_white hsv=%s' % (hsv,))
                    return self._start_camera_recovery(distance,now)
                return self._start_fallback('entry_seek_bound')
            if edge and abs(error)<=entry_heading_limit() and self._recovery_snapshot_valid(now):
                self.logger.info('LAP_ENTRY_SEEK_READY raw_mm=%.1f elapsed=%.3f hsv=%s' % (delta,elapsed,hsv))
                self._transition_start('ALIGN',distance,now)
            else:
                self._transition_drive(self.heading_origin,heading)
                return Status.RUNNING
        if self.transition_phase=='REACQUIRE':
            delta=distance-self.transition_start;elapsed=now-self.transition_time
            if self.gate_recovery == 'floor':
                if s <= 45 and v <= 82:
                    if self.reacquire_last_floor != raw:
                        self.reacquire_last_floor = raw
                        self.reacquire_floor_hits += 1
                else:
                    self.reacquire_last_floor = None
                    self.reacquire_floor_hits = 0
            if delta<0 or elapsed<0 or delta>=REACQUIRE_LIMIT_MM or elapsed>=REACQUIRE_LIMIT_SEC:
                if delta >= 0 and elapsed >= 0 and self.gate_recovery == 'floor' and self.reacquire_floor_hits >= 2:
                    self.logger.info('LAP_GATE_FLOOR_REJOIN progress_mm=%.1f hsv=%s samples=%d' % (progress, hsv, self.reacquire_floor_hits))
                    if self.gate_latched:
                        return self._finish_distance_run(Status.SUCCESS)
                    self._transition_start('GATE_REJOIN', distance, now)
                    self._transition_drive(self.heading_origin, heading)
                    return Status.RUNNING
                self.logger.warning('LAP_REACQUIRE_UNAVAILABLE reason=bound raw_mm=%.1f elapsed=%.3f' % (delta,elapsed))
                return self._start_fallback('reacquire_bound')
            if (self._white_sided_snapshot(now) is not None if white_sided_rejoin_enabled()
                    else self._recovery_snapshot_valid(now)):
                self.logger.info('LAP_REACQUIRE_READY raw_mm=%.1f elapsed=%.3f' % (delta,elapsed))
                return self._start_camera_recovery(distance,now)
            self._transition_drive(self.heading_origin,heading)
            return Status.RUNNING
        if self.transition_phase in ('ALIGN','RECOVER'):
            delta=distance-self.transition_start;elapsed=now-self.transition_time
            limit_mm=ALIGN_LIMIT_MM if self.transition_phase=='ALIGN' else RECOVERY_LIMIT_MM
            limit_s=ALIGN_LIMIT_SEC if self.transition_phase=='ALIGN' else RECOVERY_LIMIT_SEC
            if delta<0 or elapsed<0 or delta>=limit_mm or elapsed>=limit_s:
                self.logger.warning('LAP_CORRECTION_UNAVAILABLE phase=%s reason=bound raw_mm=%.1f elapsed=%.3f' % (self.transition_phase,delta,elapsed))
                return self._start_fallback('transition_'+self.transition_phase.lower()+'_bound')
            self.edge_count=self.edge_count+1 if edge and abs(error)<=entry_heading_limit() else 0
            if self.edge_count>=1:
                snapshot=self.side_source.snapshot if self.side_source is not None else None
                if (not snapshot or snapshot.get('side') not in ('LEFT','RIGHT')
                    or not isinstance(snapshot.get('frame'),int)
                    or (self.frozen_camera_stream is not None and snapshot.get('stream')!=self.frozen_camera_stream)
                    or not 0<=now-snapshot.get('capture_time',-math.inf)<=CAMERA_MAX_AGE_SEC):
                    self.logger.warning('LAP_BLEND_UNAVAILABLE reason=fresh_trace_side_unknown')
                    prior_phase=self.transition_phase
                    self.entry_seek_parent=(self.transition_start,self.transition_time) if prior_phase=='RECOVER' else None
                    self._transition_start('ENTRY_SEEK',distance,now)
                    self.logger.info('LAP_ENTRY_SEEK_START source=%s motion=gyro limits_mm=%.1f limits_sec=%.2f' % (prior_phase,ENTRY_SIDE_LIMIT_MM,ENTRY_SIDE_LIMIT_SEC))
                    self._transition_drive(self.heading_origin,heading)
                    return Status.RUNNING
                direction={'LEFT':-1,'RIGHT':1}[snapshot['side']]
                self.logger.info('LAP_BLEND_HANDOFF hsv=%s heading=%.2f error=%.2f frame=%s stream=%s capture_time=%s side=%s votes=%s initial_direction=%s' % (hsv,heading,error,snapshot.get('frame'),snapshot.get('stream'),snapshot.get('capture_time'),snapshot.get('side'),snapshot.get('vote_counts'),self.camera_direction))
                return self._start_pre_gate_trace(hsv, direction)
            if self.transition_phase=='ALIGN' and s<=45 and v>=95:
                return self._start_camera_recovery(distance,now)
            target=self.heading_origin
            if self.transition_phase=='RECOVER':
                target+=self.recovery_direction*(-runtime.course)*RECOVERY_PEAK_DEG*math.sin(math.pi*delta/RECOVERY_LIMIT_MM)**2
            self._transition_drive(target,heading)
            return Status.RUNNING
        if self.transition_phase=='TRACE':
            if self.gate_latched:return self._finish_distance_run(Status.SUCCESS)
            self.pre_gate_white_count=self.pre_gate_white_count+1 if s<=45 and v>=95 else 0
            if self.pre_gate_white_count>=3:
                self.pre_gate_trace.running=False  # retain current output until recovery commands in this tick
                self.pre_gate_trace=None
                return self._start_camera_recovery(distance,now)
            return self.pre_gate_trace.update()
        return self._finish_distance_run(Status.FAILURE)

    def _recovery_snapshot_valid(self,now,snapshot=None):
        if snapshot is None:
            snapshot=self.side_source.snapshot if self.side_source is not None else None
        if not (snapshot and snapshot.get('side') in ('LEFT','RIGHT') and isinstance(snapshot.get('frame'),int)
                and (self.frozen_camera_frame is None or snapshot['frame']>self.frozen_camera_frame)
                and 0<=now-snapshot.get('capture_time',-math.inf)<=CAMERA_MAX_AGE_SEC):
            return False
        current=snapshot.get('stream'); frozen=self.frozen_camera_stream
        if frozen is None or current == frozen:
            return True
        # The VideoThread retains its stream UUID and increasing frame number
        # when the delivery bottle reader starts a new vision session.
        if (not isinstance(current,tuple) or not isinstance(frozen,tuple)
                or len(current)!=2 or len(frozen)!=2 or current[0]!=frozen[0]
                or not isinstance(current[1],int) or not isinstance(frozen[1],int)
                or current[1]<=frozen[1]):
            return False
        video=getattr(runtime,'video',None)
        sessions=getattr(video,'_vision_sessions',None)
        if sessions is None or sessions.capture_token() != (current[1], 'bottle'):
            return False
        self.logger.info('LAP_REACQUIRE_SESSION_ADVANCE frozen=%s current=%s frame=%d' % (frozen,current,snapshot['frame']))
        return True

    def _white_sided_snapshot(self, now):
        source = self.profile_rejoin_source
        snapshot = source.snapshot if source is not None else None
        if snapshot is None or not self._recovery_snapshot_valid(now, snapshot):
            return None
        geometry = snapshot.get('geometry') or {}
        if not isinstance(geometry.get('paired_white_rows'), int) or geometry['paired_white_rows'] < 4:
            return None
        return snapshot

    def _start_camera_recovery(self,distance,now):
        if self.recovery_attempts>=RECOVERY_MAX_ATTEMPTS:
            self.logger.warning('LAP_CORRECTION_UNAVAILABLE reason=recovery_attempt_limit');return self._start_fallback('recovery_attempt_limit')
        candidate = self._white_sided_snapshot(now) if white_sided_rejoin_enabled() else None
        if (candidate is None if white_sided_rejoin_enabled() else not self._recovery_snapshot_valid(now)):
            self._transition_start('REACQUIRE',distance,now)
            self.logger.info('LAP_REACQUIRE_WAIT motion=gyro limits_mm=%.1f limits_sec=%.2f' % (REACQUIRE_LIMIT_MM,REACQUIRE_LIMIT_SEC))
            self._transition_drive(self.heading_origin,-runtime.course*float(runtime.gyro_sensor.get_angle()))
            return Status.RUNNING
        snapshot=candidate if white_sided_rejoin_enabled() else self.side_source.snapshot
        if white_sided_rejoin_enabled():
            self.logger.info('LAP_WHITE_SIDED_RECOVERY frame=%s side=%s x=%s geometry=%s' %
                             (snapshot['frame'], snapshot['side'], snapshot.get('x'), snapshot.get('geometry')))
        self.recovery_attempts+=1;self.recovery_direction={'LEFT':-1,'RIGHT':1}[snapshot['side']]
        self.frozen_camera_frame=snapshot['frame'];self.frozen_camera_stream=snapshot['stream']
        self.logger.info('LAP_RECOVERY_VOTE frame=%s stream=%s capture_time=%s side=%s x=%s votes=%s' % (snapshot.get('frame'),snapshot.get('stream'),snapshot.get('capture_time'),snapshot.get('side'),snapshot.get('x'),snapshot.get('vote_counts')))
        self._transition_start('RECOVER',distance,now)
        self._transition_drive(self.heading_origin,-runtime.course*float(runtime.gyro_sensor.get_angle()))
        return Status.RUNNING

    def _bottle_aim_frame(self):
        """新しい画像ならボトルの観測 (frame, color, cx, bottom, area, blind) を返す。古い・見えないなら None。"""
        try:
            session, frame_id, observation = runtime.video.get_bottle_observation()
        except Exception:
            return None
        if frame_id is None or frame_id <= self.aim_last_frame:
            return None
        self.aim_last_frame = frame_id
        insight, color, cx, theta, bottom, area, blind = observation
        if not insight:
            return (frame_id, None, None, None, None, None)
        return (frame_id, color, float(cx), int(bottom), int(area), bool(blind))

    def _bottle_aim_ready(self):
        progress = float(runtime.plotter.get_distance()) - self.origin_distance
        if progress < BOTTLE_AIM_START_MM or progress >= self.distance_limit_mm:
            return False
        obs = self._bottle_aim_frame()
        if obs is None:
            return False
        frame_id, color, cx, bottom, area, blind = obs
        right = _bottle_right_deg(cx, bottom) if color is not None else None
        ok = (color is not None and area >= BOTTLE_AIM_MIN_AREA and not blind
              and abs(right) <= BOTTLE_AIM_START_MAX_DEG)
        if not ok:
            self.aim_hits, self.aim_color = 0, None
            return False
        self.aim_hits = self.aim_hits + 1 if color == self.aim_color else 1
        self.aim_color = color
        if self.aim_hits < BOTTLE_AIM_HITS:
            return False
        self.logger.info('BOTTLE_AIM_START progress_mm=%.1f color=%s cx=%.0f bottom=%d area=%d right_deg=%.2f phase=%s gate_latched=%s'
                         % (progress, getattr(color, 'value', color), cx, bottom, area, right, self.transition_phase, self.gate_latched))
        # 2026-11-18: ここからはこの色だけを探す(ほかの色のボトルや床の青い線を選ばない)。
        # 次にボトルを読むとき(begin_bottle_read)に解除される。
        try:
            runtime.video.set_bottle_color(color)
        except Exception as error:
            self.logger.warning('BOTTLE_AIM color lock failed: %r' % (error,))
        return True

    def _bottle_aim_heading(self, heading):
        """fallback で向かう向き。ボトルへ向かうときは画像ごとに更新する。"""
        if not getattr(self, 'aim_active', False):
            return self.heading_origin
        now = time.monotonic()
        self.aim_heads.append((now, heading))
        if self.aim_target is None:
            self.aim_target = heading
        obs = self._bottle_aim_frame()
        if obs is None:
            return self.aim_target
        frame_id, color, cx, bottom, area, blind = obs
        # 下の端に掛かった(blind)ボトルも横の位置は使える。画面の左右の端に切れたものは使わない。
        use = (color is not None and color == self.aim_color and area >= BOTTLE_AIM_MIN_AREA and 8.0 <= cx <= 312.0
               and not self.aim_under)
        right = None
        past = heading
        if use:
            self.aim_max_area = max(self.aim_max_area, area)
            right = _bottle_right_deg(cx, bottom)
            # 画像を撮ったときの向き(遅れの分だけ前)を基準にする
            for t, h in self.aim_heads:
                if t <= now - BOTTLE_AIM_LATENCY_SEC:
                    past = h
            # 画面の右 = 実際の右。heading は右コースで右回りに増え、左コースで左回りに増える。
            want = past - runtime.course * BOTTLE_AIM_GAIN * right
            offset = (want - self.heading_origin + 180.0) % 360.0 - 180.0
            offset = max(-BOTTLE_AIM_MAX_DEG, min(BOTTLE_AIM_MAX_DEG, offset))
            want = self.heading_origin + offset
            self.aim_target += BOTTLE_AIM_SMOOTH * ((want - self.aim_target + 180.0) % 360.0 - 180.0)
        elif color != self.aim_color and self.aim_max_area >= BOTTLE_AIM_UNDER_AREA and not self.aim_under:
            # 大きく見えていたボトルが見えなくなった = アームの間(カメラの下)に入った。そこからは最後に求めたボトルの向きのまま。
            # 11/18 09:34(A・左コース)はここで今の向き(-7°、揺れの端)に切り替えたため、ボトルの向き(+2°)から
            # 9°ずれたまま進んで取りこぼした。
            self.aim_under = True
            self.logger.info('BOTTLE_AIM_UNDER progress_mm=%.1f heading=%.2f keep_target=%.2f max_area=%d'
                             % (float(runtime.plotter.get_distance()) - self.origin_distance, heading, self.aim_target, self.aim_max_area))
        if self.aim_logged < 200:
            self.aim_logged += 1
            progress = float(runtime.plotter.get_distance()) - self.origin_distance
            self.logger.info('BOTTLE_AIM frame=%d progress_mm=%.1f heading=%.2f past=%.2f target=%.2f origin=%.2f used=%s right_deg=%s color=%s cx=%s bottom=%s area=%s blind=%s'
                             % (frame_id, progress, heading, past, self.aim_target, self.heading_origin, use,
                                None if right is None else round(right, 2),
                                getattr(color, 'value', color), cx, bottom, area, blind))
        return self.aim_target

    def _start_fallback(self,reason):
        distance=float(runtime.plotter.get_distance());gyro=float(runtime.gyro_sensor.get_angle());now=time.monotonic()
        if runtime.course not in (-1,1) or not all(map(math.isfinite,(distance,gyro,now))):return self._finish_distance_run(Status.FAILURE)
        if self.trace_projection is None:
            self.trace_projection_base=distance-self.origin_distance
            try:self.trace_projection=ProjectedDistance(runtime.plotter,self.heading_origin,runtime.course)
            except ValueError:return self._finish_distance_run(Status.FAILURE)
        if self.pre_gate_trace is not None:self.pre_gate_trace.running=False;self.pre_gate_trace=None
        self.fallback_reason=reason;self.fallback_gate_base=(float(self.completion_condition.black_progress) if self.gate_latched else None)
        self._transition_start('FALLBACK',distance,now)
        self.logger.warning('LAP_FALLBACK_START reason=%s estimated_gate_mm=%.1f gate_latched=%s unseen_gate_after_mm=%.1f' % (reason,self.fallback_estimated_gate_mm,self.gate_latched,UNSEEN_GATE_FALLBACK_AFTER_MM))
        return self._transition_update()
    @staticmethod
    def _unseen_blue_evidence(gate):
        return bool(gate is None or getattr(gate, 'saw_blue', False)
                    or getattr(gate, 'weak_blue_progress', None) is not None
                    or getattr(gate, 'weak_blue_samples', 0))
    def _unseen_estimate_plan(self):
        distance = unseen_estimate_distance()
        if distance is None or self.camera_start_mm is None:
            return None
        slow_at = self.fallback_estimated_gate_mm - UNSEEN_ESTIMATE_SLOW_BEFORE_GATE_MM
        return slow_at, slow_at + max(0.0, distance - UNSEEN_DISTANCE_REDUCTION_MM)
    def _sync_lap_bottle_session(self, now):
        # Rebind the LAP reference only after a fresh frame from the same
        # camera confirms the currently active bottle-reading session.
        frozen = self.frozen_camera_stream
        source = self.side_source
        session = getattr(source, 'session', None)
        if frozen is None or session is None:
            return
        with session.lock:
            observation = session.latest
        if not isinstance(observation, dict):
            return
        current = (observation.get('stream_id'), observation.get('session'))
        frame = observation.get('frame')
        captured = observation.get('capture_time')
        if (not isinstance(frozen, tuple) or len(frozen) != 2
                or not isinstance(frozen[1], int) or not isinstance(current[1], int)
                or current[0] != frozen[0] or current[1] <= frozen[1]
                or not isinstance(frame, int) or not isinstance(self.frozen_camera_frame, int)
                or frame <= self.frozen_camera_frame or not isinstance(captured, (int, float))
                or not math.isfinite(captured) or not 0 <= now - captured <= CAMERA_MAX_AGE_SEC):
            return
        video = getattr(runtime, 'video', None)
        sessions = getattr(video, '_vision_sessions', None)
        if sessions is None or sessions.capture_token() != (current[1], 'bottle'):
            return
        self.frozen_camera_stream = current
        vote = self.profile_rejoin_vote
        if isinstance(vote, dict) and vote.get('stream') == frozen:
            self.profile_rejoin_vote = dict(vote, stream=current)
        self.logger.info('LAP_CAMERA_SESSION_SYNC old=%s new=%s frame=%d' % (frozen, current, frame))

    def update(self):
        if self.running and self.straight_start_mm is not None:
            progress = float(runtime.plotter.get_distance()) - self.origin_distance
            if progress >= self.straight_start_mm:
                self.power = min(self.power, self.straight_power)
        if self.running:
            self._sync_lap_bottle_session(time.monotonic())
        if self.running and BOTTLE_AIM and not getattr(self, 'aim_active', False) and self._bottle_aim_ready():
            self.aim_active = True
            self.aim_target = None
            if self.transition_phase != 'FALLBACK':
                return self._start_fallback('bottle_aim')
        if self.running and self.transition_phase is not None:return self._transition_update()
        if self.running and self.camera_start_mm is None:
            progress = float(runtime.plotter.get_distance()) - self.origin_distance
            if progress >= self.distance_limit_mm:
                return self._start_fallback('no_final_zero_window')
            return self._profile_update()
        if self.running:
            plan = self._unseen_estimate_plan()
            gate = self.completion_condition
            if plan and not self._unseen_blue_evidence(gate):
                progress = float(runtime.plotter.get_distance()) - self.origin_distance
                if self.trace_projection is not None:
                    try:self.trace_projection.update(runtime.plotter)
                    except ValueError:return self._finish_distance_run(Status.FAILURE)
                    progress = self.projected_progress()
                if progress >= plan[0]:
                    gate.tick_once()
                    if gate.status == Status.SUCCESS:
                        self.gate_latched = True
                        return self._finish_distance_run(Status.SUCCESS)
                    if not self._unseen_blue_evidence(gate):
                        return self._start_fallback('unseen_estimate')
        if self.running and self.live_camera_recovery:
            distance = float(runtime.plotter.get_distance())
            progress = distance - self.origin_distance
            if progress >= self.camera_start_mm:
                self.trace_projection_base = progress
                self.trace_projection = ProjectedDistance(runtime.plotter, self.heading_origin, runtime.course)
                self._transition_start('CAMERA_SEEK', distance, time.monotonic())
                return self._transition_update()
        if self.running and self.camera_direction is None:
            progress=float(runtime.plotter.get_distance())-self.origin_distance
            if progress>=self.camera_start_mm:
                if progress>self.camera_deadline_mm:
                    self.logger.warning('LAP_CAMERA_CORRECTION deadline exceeded at %.1f mm; starting fallback' % progress)
                    return self._start_fallback('camera_deadline')
                snapshot=self.side_source.snapshot if self.side_source is not None else None
                if snapshot is None or not 0<=time.monotonic()-snapshot['capture_time']<=CAMERA_MAX_AGE_SEC:
                    if progress>=self.camera_deadline_mm:
                        self.logger.warning('LAP_CAMERA_CORRECTION unavailable at %.1f mm; starting fallback' % progress)
                        return self._start_fallback('camera_unavailable')
                    if not getattr(self,'camera_search_logged',False):
                        self.logger.info('LAP_CAMERA_SEARCH start_mm=%.1f deadline_mm=%.1f; continue profile' % (progress,self.camera_deadline_mm));self.camera_search_logged=True
                    return self._profile_update()
                self.camera_direction={'LEFT':-1,'RIGHT':1,'CENTER':0}[snapshot['side']]
                deadband=max(12.0, merge_deadband())
                if abs(snapshot['x']-lap_camera_reference_x())<=deadband:self.camera_direction=0
                self.logger.info('LAP_CAMERA_DEADBAND x=%.1f reference=%.1f deadband_px=%.1f voted_side=%s direction=%d' % (snapshot['x'],lap_camera_reference_x(),deadband,snapshot['side'],self.camera_direction))
                self.frozen_camera_frame=snapshot.get('frame')
                self.frozen_camera_stream=snapshot.get('stream')
                self.far_vote_last_frame = snapshot.get('frame')
                start=progress;direction=self.camera_direction;original=self.profile_target;course=runtime.course;peak=merge_peak();length=merge_length()
                self.initial_merge_end = start + length
                self.adaptive_initial_offset = direction * (snapshot['x'] - lap_camera_reference_x())
                self.adaptive_initial_progress = start
                if course not in (-1,1):return self._finish_distance_run(Status.FAILURE)
                self.far_entry_hold = bool(far_entry_policy() == 'fixed60' and direction and direction * (snapshot['x'] - lap_camera_reference_x()) >= FAR_ENTRY_OFFSET_PX)
                self.far_entry_hold_end = self.distance_limit_mm - FAR_ENTRY_RETURN_MM
                self.logger.info('LAP_FAR_ENTRY_POLICY mode=%s initial_x=%.1f direction=%d' % (far_entry_policy(), snapshot['x'], direction))
                if self.far_entry_hold and self.far_entry_hold_end <= start + length / 2:
                    self.far_entry_hold = False
                if self.far_entry_hold:
                    self.logger.info('LAP_FAR_ENTRY_HOLD_START x=%.1f side=%s peak_deg=%.1f hold_until_mm=%.1f return_mm=%.1f'
                                     % (snapshot['x'],snapshot['side'],peak,self.far_entry_hold_end,FAR_ENTRY_RETURN_MM))
                def camera_target(distance):
                    phase=(distance-start)/length
                    actual_peak = peak
                    if self.adaptive_boost_start is not None and not self.adaptive_boost_cancelled:
                        t = max(0.0, min(1.0, (distance - self.adaptive_boost_start) / ADAPTIVE_RAMP_MM))
                        actual_peak += min(2.0, max(0.0, far_peak_deg() - peak)) * t * t * (3.0 - 2.0 * t)
                    if self.far_entry_hold and distance >= start + length / 2:
                        remaining = max(0.0, min(1.0, (distance - self.far_entry_hold_end) / FAR_ENTRY_RETURN_MM))
                        adjustment=direction*(-course)*actual_peak*math.cos(math.pi * remaining / 2)**2
                    else:
                        adjustment=direction*(-course)*actual_peak*math.sin(math.pi*phase)**2 if 0<=phase<=1 else 0.0
                    return original(distance)+adjustment
                self.target=camera_target
                self.logger.info('LAP_CAMERA_CORRECTION start_mm=%.1f side=%s x=%.1f frame=%s peak_deg=%.1f length_mm=%.1f frozen=True votes=%s frames=%s' %
                    (progress,snapshot['side'],snapshot['x'],snapshot['frame'],peak,length,snapshot.get('vote_counts'),snapshot.get('vote_frames')))
        if self.running:
            progress=float(runtime.plotter.get_distance())-self.origin_distance
            if self.far_entry_hold:
                snapshot = self.side_source.snapshot if self.side_source is not None else None
                gate = self.completion_condition
                now = time.monotonic()
                if gate is not None and self._profile_rejoin_pending_gate():
                    self.far_entry_hold = False;self.far_entry_disabled_reason = 'blue_candidate'
                elif progress >= self.distance_limit_mm:
                    self.far_entry_hold = False;self.far_entry_disabled_reason = 'distance_bound'
                elif snapshot and snapshot.get('stream') == self.frozen_camera_stream and 0 <= now - snapshot.get('capture_time', -math.inf) <= CAMERA_MAX_AGE_SEC:
                    self.far_vote_last_frame = snapshot.get('frame')
                    if self.far_vote_grace_start is not None:
                        self.logger.info('LAP_FAR_VOTE_GRACE_END reason=vote_restored progress_mm=%.1f' % progress)
                    self.far_vote_grace_start = None;self.far_vote_grace_progress = None;self.far_vote_grace_frame = None
                else:
                    source = self.side_source
                    session = getattr(source, 'session', None)
                    if session is not None:
                        with session.lock: observation = session.latest
                    else:
                        observation = None
                    obs = observation if isinstance(observation, dict) else {}
                    stream = (obs.get('stream_id'), obs.get('session'))
                    frame, captured = obs.get('frame'), obs.get('capture_time')
                    advancing = (getattr(source, 'last_reason', None) == 'no_majority'
                                 and stream == self.frozen_camera_stream and isinstance(frame, int)
                                 and isinstance(self.far_vote_last_frame, int) and frame > self.far_vote_last_frame
                                 and (self.far_vote_grace_frame is None or frame >= self.far_vote_grace_frame)
                                 and isinstance(captured, (int, float)) and math.isfinite(captured)
                                 and 0 <= now - captured <= FAR_VOTE_GRACE_SEC)
                    if advancing and self.far_vote_grace_start is None:
                        self.far_vote_grace_start = now;self.far_vote_grace_progress = progress
                        self.logger.info('LAP_FAR_VOTE_GRACE_START progress_mm=%.1f frame=%d age_ms=%.1f' % (progress, frame, 1000 * (now - captured)))
                    if advancing:
                        self.far_vote_grace_frame = frame
                    grace = (advancing and self.far_vote_grace_start is not None
                             and 0 <= now - self.far_vote_grace_start <= FAR_VOTE_GRACE_SEC
                             and 0 <= progress - self.far_vote_grace_progress <= FAR_VOTE_GRACE_MM)
                    if not grace:
                        self.far_entry_hold = False
                        self.far_entry_disabled_reason = 'vote_grace_expired' if self.far_vote_grace_start is not None else 'camera_or_vote_unavailable'
                        self.logger.info('LAP_FAR_VOTE_UNAVAILABLE reason=%s source_reason=%s obs_stream=%s frame=%s age_ms=%s'
                                         % (self.far_entry_disabled_reason, getattr(source,'last_reason',None), stream, frame,
                                            round(1000*(now-captured),1) if isinstance(captured,(int,float)) and math.isfinite(captured) else None))
                if self.far_entry_disabled_reason:
                    self.target = self.profile_target
                    self.logger.info('LAP_FAR_ENTRY_HOLD_END reason=%s progress_mm=%.1f' % (self.far_entry_disabled_reason,progress))
            if self.late_merge_used and self.completion_condition is not None and self._profile_rejoin_pending_gate():
                self.target = self.profile_target
            if (self.line_entry == 'cross_rejoin' and self.camera_direction == 0
                    and self.early_return_direction in (-1, 1) and self.completion_condition is not None
                    and not getattr(self.completion_condition, 'saw_blue', False)):
                now = time.monotonic()
                distance = float(runtime.plotter.get_distance())
                self.trace_projection_base = progress
                self.trace_projection = ProjectedDistance(runtime.plotter, self.heading_origin, runtime.course)
                self._transition_start('ENTRY_RETURN', distance, now)
                self.logger.info('LAP_EARLY_RETURN_START progress_mm=%.1f last_edge_mm=%.1f direction=%d'
                                 % (progress, self.early_edge_progress, self.early_return_direction))
                return self._transition_update()
            if self.completion_condition is not None and self.pre_gate_trace is None and self.camera_direction in (-1,0,1) and progress>=self.observation_start_mm:
                hsv=tuple(runtime.color_sensor.get_raw_color_hsv())
                if self.profile_white_rejoin:
                    result = self._maybe_start_profile_rejoin(progress, hsv)
                    if result is not None:
                        return result
                if self.floor_classifier.classify_single(*hsv)==Color.BLACK:
                    self.trace_projection_base=progress
                    self.trace_projection=ProjectedDistance(runtime.plotter,self.heading_origin,runtime.course)
                    now = time.monotonic()
                    distance = float(runtime.plotter.get_distance())
                    snapshot = self.side_source.snapshot if self.side_source is not None else None
                    if (self.line_entry in ('cross', 'cross_soft', 'cross_rejoin') and not getattr(self.completion_condition, 'saw_blue', False)
                            and self.camera_direction in (-1, 1) and snapshot
                            and snapshot.get('side') == ('LEFT' if self.camera_direction < 0 else 'RIGHT')
                            and snapshot.get('stream') == self.frozen_camera_stream
                            and 0 <= now - snapshot.get('capture_time', -math.inf) <= CAMERA_MAX_AGE_SEC):
                        self._start_line_cross(distance, now, hsv, self.camera_direction, 'camera', snapshot.get('frame'))
                    else:
                        self._transition_start('ALIGN', distance, now)
                        self.logger.info('LAP_BLACK_ALIGN progress_mm=%.1f hsv=%s direction=%d' % (progress,hsv,self.camera_direction))
                    return self._transition_update()
                self._update_adaptive_peak(progress, hsv, time.monotonic())
                self._maybe_start_late_merge(progress, hsv, time.monotonic())
            if self.pre_gate_trace is not None:
                # All trace distance limits use longitudinal projection from the LAP reference.
                raw_distance=float(runtime.plotter.get_distance())
                raw_progress=raw_distance-self.origin_distance
                try:
                    self.trace_projection.update(runtime.plotter)
                except ValueError:
                    self.logger.error('LAP_PROJECTION_INVALID raw_progress_mm=%.1f' % raw_progress)
                    return self._finish_distance_run(Status.FAILURE)
                progress=self.projected_progress()
                self.completion_condition.tick_once()
                if self.completion_condition.status==Status.SUCCESS:return self._finish_distance_run(Status.SUCCESS)
                if progress>=self.distance_limit_mm:return self._finish_distance_run(Status.FAILURE)
                h,s,v=runtime.color_sensor.get_raw_color_hsv()
                self.pre_gate_white_count=self.pre_gate_white_count+1 if s<=45 and v>=95 else 0
                if self.pre_gate_white_count>=12:
                    # Capture once at loss; diagnostics only. Existing stop remains unconditional.
                    now=time.monotonic();source=self.side_source
                    if source is not None and getattr(source,'session',None) is not None:
                        with source.session.lock: observation=source.session.latest
                        vote=source.snapshot
                    else:observation=None;vote=None
                    gyro_at_loss=float(runtime.gyro_sensor.get_angle())
                    self.logger.error('LAP_PRE_GATE_TRACE_LOST progress_mm=%.1f hsv=%s' % (progress,(h,s,v)))
                    result=self._finish_distance_run(Status.FAILURE)
                    assessment=white_recovery_shadow(observation,vote,now,lap_camera_reference_x(),IMAGE_DEADBAND,CAMERA_MAX_AGE_SEC)
                    assessment.update(floor_hsv=(h,s,v),raw_progress_mm=raw_progress,projected_progress_mm=progress,gyro_deg=gyro_at_loss,mono_sec=now)
                    self.logger.info('LAP_WHITE_RECOVERY_SHADOW '+json.dumps(assessment,separators=(',',':'),allow_nan=False))
                    return result
                return self.pre_gate_trace.update()
        return self._profile_update()
    def projected_progress(self):
        return self.trace_projection_base+self.trace_projection.forward
    def _finish_distance_run(self,status):
        if status==Status.SUCCESS:
            self.handoff_complete=True
            self.logger.info('LAP_BOUNDARY distance_limit_mm=%.3f raw_progress_mm=%.1f projected_progress_mm=%.1f mono_sec=%.6f gyro_deg=%.2f gate_sensor_confirmed=%s source=%s' % (self.distance_limit_mm, float(runtime.plotter.get_distance())-self.origin_distance, self.projected_progress() if self.trace_projection is not None else float(runtime.plotter.get_distance())-self.origin_distance, time.monotonic(), float(runtime.gyro_sensor.get_angle()),bool(self.gate_latched),'fallback' if self.fallback_done else 'blue_then_black'))
            return status
        if self.completion_condition is not None:
            self.logger.error('LAP_GATE_NOT_CONFIRMED reason=distance_limit_or_drive_failure blue_seen=%s limit_mm=%.1f raw_progress_mm=%.1f projected_progress_mm=%.1f' % (getattr(self.completion_condition,'saw_blue',False),self.distance_limit_mm,float(runtime.plotter.get_distance())-self.origin_distance,self.projected_progress() if self.trace_projection is not None else float(runtime.plotter.get_distance())-self.origin_distance))
        return super()._finish_distance_run(status)
    def terminate(self,status):
        if self.pre_gate_trace is not None and status!=Status.SUCCESS:
            self.pre_gate_trace.terminate(status)
        if status==Status.SUCCESS and self.handoff_complete:
            self.running=False
            return
        super().terminate(status)

class RecentSide(Behaviour):
    """Majority among a short window of distinct fresh camera frames."""
    WINDOW_FRAMES=5
    MIN_VOTES=3
    def __init__(self,session,lap, *, rejoin=False):
        super().__init__('watch rejoin line' if rejoin else 'watch line near lap end')
        self.session=session;self.lap=lap
        self.rejoin = rejoin
    def initialise(self):
        self.snapshot=None;self.last_frame=None;self.stream=None;self.side=None;self.streak=0;self.rejects=0
        self.last_reason='no_samples';self.last_line=None;self.last_log=-1.0;self.votes=deque(maxlen=self.WINDOW_FRAMES)
    def update(self):
        if not self.lap.running:return Status.RUNNING
        runtime.require('plotter')
        progress=float(runtime.plotter.get_distance())-self.lap.origin_distance
        observation_start = getattr(self.lap, 'observation_start_mm', FINAL_OBSERVATION_START_MM)
        if observation_start is None or progress < observation_start:return Status.RUNNING
        if not getattr(self.lap, 'native_capture_enabled', False):
            runtime.video.set_lap_observation(True, save_images=True)
            self.lap.native_capture_enabled = True
        with self.session.lock:obs=self.session.latest
        line_key = 'lap_rejoin_line' if self.rejoin else 'lap_line'
        now=time.monotonic();line=(obs or {}).get(line_key) or {};captured=(obs or {}).get('capture_time');x=line.get('x_px')
        age=(now-captured) if isinstance(captured,(float,int)) and math.isfinite(captured) else None
        self.last_line={'valid':line.get('valid'),'track_state':line.get('track_state'),'reason':line.get('track_reason'),'x_px':x,'image_width':line.get('image_width'),'age_ms':round(age*1000,1) if age is not None else None}
        require_white_sides = self.rejoin or getattr(self.lap, 'live_camera_recovery', False)
        # Never retain votes after their capture-time validity has expired.
        self.votes=deque((v for v in self.votes if 0<=now-v['capture_time']<=CAMERA_MAX_AGE_SEC),maxlen=self.WINDOW_FRAMES)
        stream=((obs or {}).get('stream_id'),(obs or {}).get('session'));frame=(obs or {}).get('frame')
        if not isinstance(stream[0],str) or not stream[0] or not isinstance(stream[1],int) or not isinstance(frame,int):
            self.votes.clear();self.snapshot=None;self.last_reason='invalid_frame_identity';return Status.RUNNING
        if stream!=self.stream:
            self.stream=stream;self.last_frame=None;self.votes.clear();self.snapshot=None
        if self.last_frame is not None and frame<self.last_frame:
            self.votes.clear();self.snapshot=None;self.last_frame=frame;self.last_reason='frame_reversed';return Status.RUNNING
        if frame!=self.last_frame:
            self.last_frame=frame
            valid=(age is not None and 0<=age<=CAMERA_MAX_AGE_SEC and line.get('valid') and line.get('track_state')=='CANDIDATE'
                   and (not require_white_sides or line.get('track_reason') == 'white_sided')
                   and line.get('image_width')==320 and isinstance(x,(int,float)) and math.isfinite(x) and 0<=x<320)
            if valid:
                side='RIGHT' if x-lap_camera_reference_x()>IMAGE_DEADBAND else 'LEFT' if x-lap_camera_reference_x() < -IMAGE_DEADBAND else 'CENTER'
            else:
                side=None;self.rejects+=1
            # A no-line frame occupies a slot but supplies no invented direction.
            if age is not None and 0<=age<=CAMERA_MAX_AGE_SEC:
                vote = {'side':side,'x':x,'frame':frame,'stream':stream,'capture_time':captured}
                if require_white_sides and valid:
                    vote['geometry'] = {key: line.get(key) for key in ('near_uv', 'far_uv', 'paired_white_rows')}
                self.votes.append(vote)
        counts=Counter(v['side'] for v in self.votes if v['side'] is not None)
        winners=[side for side,n in counts.items() if n>=self.MIN_VOTES and n>sum(counts.values())-n]
        if len(winners)==1:
            winner=winners[0];self.snapshot=next(dict(v) for v in reversed(self.votes) if v['side']==winner)
            self.snapshot['vote_counts']=dict(counts);self.snapshot['vote_frames']=len(self.votes);self.side=winner;self.streak=counts[winner];self.last_reason='majority_ready'
        else:
            self.snapshot=None;self.side=None;self.streak=0;self.last_reason='no_majority'
        if self.last_log<0 or now-self.last_log>=0.5:
            marker = 'LAP_PROFILE_REJOIN_VOTE' if self.rejoin else 'LAP_SIDE_VOTE'
            self.logger.info('%s progress_mm=%.1f counts=%s frames=%d no_line=%d result=%s line=%s lap_processing_ms=%s' %
                 (marker,progress,dict(counts),len(self.votes),sum(v['side'] is None for v in self.votes),self.side,self.last_line,(obs or {}).get('lap_processing_ms')));self.last_log=now
        return Status.RUNNING

class RollingCameraMerge(CameraMergeDemo):
    """Use the pre-gate snapshot without stopping or waiting after boundary."""
    def __init__(self, session, side, *, post_gate_steer='baseline', continuous_exit=False):
        super().__init__(session, allow_both_courses=True)
        self.side_source = side
        self.post_gate_steer = post_gate_steer
        self.continuous_exit = continuous_exit

    def terminate(self, status):
        if status == Status.SUCCESS and self.continuous_exit:
            self.trace = None
            return
        super().terminate(status)

    def initialise(self):
        self.phase='PRESELECTED';self.trace=None;self.white_count=0;self.post_trace_projection=None
        self.post_gate_gyro_active = False
        self.post_trace_target_mm = TRACE_MM
        # The successful LAP node intentionally retains its last command.
    def _fail(self,reason):
        fallback_reasons={'trace_lost_white','line_not_found_within_distance','no_fresh_pre_gate_side',
                          'gate_passed_trace_side_unknown','center_camera_floor_not_trace_edge',
                          'post_gate_floor_unknown','camera_context_changed'}
        lap=getattr(self.side_source,'lap',None)
        if reason in fallback_reasons and lap is not None and callable(getattr(lap,'_start_fallback',None)) and not getattr(lap,'fallback_done',False):
            if self.trace is not None:self.trace.running=False;self.trace=None
            self.phase='POST_FALLBACK'
            self.logger.warning('LAP_POST_FALLBACK reason=%s' % reason)
            return lap._start_fallback('post_'+reason)
        return super()._fail(reason)
    def _start_trace_projection(self):
        distance=float(runtime.plotter.get_distance());heading=-runtime.course*float(runtime.gyro_sensor.get_angle())
        lap=getattr(self.side_source,'lap',None);reference=getattr(lap,'heading_origin',heading)
        self.post_trace_projection=ProjectedDistance(runtime.plotter,reference,runtime.course)

    def _drive_post_gate_gyro(self, distance, heading):
        lap = self.side_source.lap
        reference = float(lap.heading_origin)
        if runtime.course not in (-1, 1) or not all(map(math.isfinite, (distance, heading, reference))):
            return self._fail('invalid_post_gate_gyro_input')
        error = (reference - heading + 180.0) % 360.0 - 180.0
        turn = max(-16.0, min(16.0, -runtime.course * 2.0 * error))
        left, right = round(TRACE_POWER + turn), round(TRACE_POWER - turn)
        for motor, power in ((runtime.left_motor, left), (runtime.right_motor, right)):
            motor.set_brake(False)
            motor.set_power(power)
        lap.trace_motor_samples.append((time.monotonic(), distance - lap.origin_distance,
                                        left, right, heading, error, 0.0, turn, 'POST_GATE_GYRO'))
        lap.trace_motor_total += 1
        return Status.RUNNING

    def _update_projected_trace(self):
        runtime.require('left_motor','right_motor','color_sensor','gyro_sensor','plotter')
        now=time.monotonic();h,s,v=runtime.color_sensor.get_raw_color_hsv()
        distance=float(runtime.plotter.get_distance());heading=-runtime.course*float(runtime.gyro_sensor.get_angle())
        try:
            projected=self.post_trace_projection.update(runtime.plotter)
        except ValueError:
            return self._fail('trace_projection_invalid')
        raw=self.post_trace_projection.traveled
        if projected>=self.post_trace_target_mm:
            lap = getattr(self.side_source, 'lap', None)
            command_time = time.monotonic() if getattr(lap, 'gate_stop_measure', False) else None
            if not self.continuous_exit:
                self._stop()
            if command_time is not None:
                lap.mark_gate_stop(command_time)
            self.logger.info('DEMO_TRACE_DONE projected_mm=%.1f raw_mm=%.1f' % (projected,raw));return Status.SUCCESS
        if self.post_gate_gyro_active:
            result = self._drive_post_gate_gyro(distance, heading)
            if result != Status.RUNNING:
                return result
        else:
            self.white_count=self.white_count+1 if v>=95 and s<=45 else 0
            if self.white_count>=12:return self._fail('trace_lost_white')
            if self.trace.update()!=Status.RUNNING:return self._fail('trace_failed')
        if now-self.last_trace_log>=.1:
            self.logger.info('DEMO_TRACE mono_sec=%.6f projected_mm=%.1f raw_mm=%.1f heading=%.2f hsv=%s white_count=%d' % (now,projected,raw,heading,(h,s,v),self.white_count));self.last_trace_log=now
        return Status.RUNNING
    def update(self):
        if self.phase=='POST_FALLBACK':
            return self.side_source.lap._transition_update()
        if self.phase=='TRACE' and self.post_trace_projection is not None:
            return self._update_projected_trace()
        if self.phase=='PRESELECTED':
            if getattr(getattr(self.side_source,'lap',None),'fallback_done',False):
                self.logger.info('LAP_FALLBACK_HANDOFF bypass_post_gate_trace=True')
                return Status.SUCCESS
            runtime.require('color_sensor','gyro_sensor','plotter')
            now=time.monotonic();snapshot=self.side_source.snapshot
            h,s,v=runtime.color_sensor.get_raw_color_hsv()
            boundary_lap=getattr(self.side_source,'lap',None)
            boundary_dist=float(runtime.plotter.get_distance())
            boundary_progress=boundary_dist-getattr(boundary_lap,'origin_distance',boundary_dist)
            self.logger.info('LAP_FLOOR_BOUNDARY mono_sec=%.6f progress_mm=%.1f gyro_deg=%.2f hsv=%s' % (now,boundary_progress,float(runtime.gyro_sensor.get_angle()),(h,s,v)))
            frozen_direction=getattr(getattr(self.side_source,'lap',None),'camera_direction',None)
            # The preceding gate condition is BLUE followed by classified BLACK.
            gate=getattr(boundary_lap,'completion_condition',None)
            gate_confirmed=bool(getattr(gate,'confirmed',False))
            if gate_confirmed:
                self.post_trace_target_mm, _ = confirmed_black_approach_mm(gate, boundary_lap.confirmed_fallback_after_mm)
            if gate_confirmed and self.post_gate_steer == 'gyro':
                self.post_gate_gyro_active = True
                self.phase = 'TRACE'
                self.last_trace_log = 0.0
                measure_projection = getattr(gate, 'measure_projection', None)
                if measure_projection is not None:
                    self.post_trace_target_mm, approach_reason = confirmed_black_approach_mm(gate, boundary_lap.confirmed_fallback_after_mm)
                    self.post_trace_projection = measure_projection
                    self.logger.info('LAP_GATE_APPROACH_PLAN source=post_gate reason=%s blue_mm=%s black_mm=%s after_black_mm=%.1f total_limit_mm=%.1f'
                                     % (approach_reason, getattr(gate,'blue_progress',None), getattr(gate,'black_progress',None), self.post_trace_target_mm, blue_unconfirmed_limit_mm()))
                else:
                    self._start_trace_projection()
                self.logger.info('LAP_POST_GATE_GYRO_START target_heading=%.2f power=%d projected_target_mm=%.1f'
                                 % (boundary_lap.heading_origin, TRACE_POWER, self.post_trace_target_mm))
                return self._update_projected_trace()
            if gate_confirmed and getattr(boundary_lap,'pre_gate_side',None) is not None and frozen_direction not in (-1,1):
                frozen_direction=-1  # actual pre-gate side is retained below
            if gate_confirmed and frozen_direction not in (-1,1) and snapshot and 0<=now-snapshot['capture_time']<=CAMERA_MAX_AGE_SEC:
                frozen_direction={'LEFT':-1,'RIGHT':1}.get(snapshot['side'])
            if gate_confirmed and frozen_direction not in (-1,1):return self._fail('gate_passed_trace_side_unknown')
            # The remembered BLACK event selects tracing even if the next floor read has advanced.
            if gate_confirmed and frozen_direction in (-1,1):
                distance=float(runtime.plotter.get_distance())
                if runtime.course not in (-1,1) or not math.isfinite(distance):return self._fail('invalid_or_unsupported_course')
                self.direction=frozen_direction
                side=getattr(boundary_lap,'pre_gate_side',None) or lap_trace_side(-frozen_direction,runtime.course)
                self.trace=PostGateApproachTrace(owner=boundary_lap,name='demo line trace',target=trace_target_v(),power=TRACE_POWER,power_min=TRACE_POWER,
                    pid_p=.50,pid_i=.000001,pid_d=.045,trace_side=side)
                self.trace_origin=distance;self.phase='TRACE';self.last_trace_log=0.0
                self.logger.info('LAP_FLOOR_HANDOFF floor_hsv=%s frozen_direction=%d trace_side=%s camera_majority=%s' %
                                 ((h,s,v),frozen_direction,side.name,snapshot['side'] if snapshot else None))
                self.post_trace_projection = getattr(gate, 'measure_projection', None)
                if self.post_trace_projection is None:
                    self._start_trace_projection()
                return self._update_projected_trace()
            self.logger.info('LAP_FLOOR_CHECK floor_hsv=%s frozen_direction=%s camera_majority=%s' %
                             ((h,s,v),frozen_direction,snapshot['side'] if snapshot else None))
            if snapshot is None or not 0<=now-snapshot['capture_time']<=CAMERA_MAX_AGE_SEC:
                self.logger.error('LAP_SIDE_DIAGNOSTIC reason=%s streak=%s rejects=%s last_line=%s' % (self.side_source.last_reason,self.side_source.streak,self.side_source.rejects,self.side_source.last_line))
                return self._fail('no_fresh_pre_gate_side')
            with self.session.lock:current=self.session.latest
            if current is None or (current.get('stream_id'),current.get('session'))!=snapshot['stream']:
                return self._fail('camera_context_changed')
            current_x=snapshot['x'];current_side=snapshot['side']
            h,s,v=runtime.color_sensor.get_raw_color_hsv()
            if current_side=='CENTER':
                if not (s<=45 and 65<=v<=82):return self._fail('center_camera_floor_not_trace_edge')
                distance=float(runtime.plotter.get_distance())
                if runtime.course not in (-1,1) or not math.isfinite(distance):return self._fail('invalid_or_unsupported_course')
                self.direction=-1;self.trace=PostGateApproachTrace(owner=boundary_lap,name='demo line trace',target=trace_target_v(),power=TRACE_POWER,power_min=TRACE_POWER,
                    pid_p=.50,pid_i=.000001,pid_d=.045,trace_side=lap_trace_side(1,runtime.course))
                self.trace_origin=distance;self.phase='TRACE';self.last_trace_log=0.0
                self.logger.info('LAP_CENTER_HANDOFF x=%.1f floor_hsv=%s trace_side=NORMAL trial=True' % (current_x,(h,s,v)))
                self._start_trace_projection()
                return self._update_projected_trace()
            if not (s<=45 and (65<=v<=82 or 90<=v<=100)):
                return self._fail('post_gate_floor_unknown')
            gyro=float(runtime.gyro_sensor.get_angle());distance=float(runtime.plotter.get_distance())
            if runtime.course not in (-1,1) or not all(map(math.isfinite,(gyro,distance))):return self._fail('invalid_or_unsupported_course')
            self.direction=1 if snapshot['side']=='RIGHT' else -1
            heading=-runtime.course*gyro
            self.initial_heading=heading;self.previous_heading=heading;self.relative_heading=0.0;self.origin=distance
            self.pid=PID(1.8,0.0,.03,setpoint=0,sample_time=CONTROL_INTERVAL_SEC,output_limits=(-20,20))
            self.phase='MERGE'
            self.logger.info('LAP_MERGE_FROZEN side=%s frame=%d x=%.1f peak_deg=5.0 relative_to=lap_boundary floor_hsv=%s' %
                             (snapshot['side'],snapshot['frame'],snapshot['x'],(h,s,v)))
        if self.phase=='MERGE':
            # Keep the generic demo's motion, but construct the selected trial
            # trace on the first edge tick rather than issuing a V75 command.
            runtime.require('left_motor','right_motor','color_sensor','gyro_sensor','plotter')
            now=time.monotonic();h,s,v=runtime.color_sensor.get_raw_color_hsv()
            distance=float(runtime.plotter.get_distance());gyro=float(runtime.gyro_sensor.get_angle())
            if runtime.course not in (-1,1) or not all(map(math.isfinite,(now,distance,gyro))):
                return self._fail('invalid_or_unsupported_course')
            proc=getattr(self.session,'proc',None)
            if proc is None or proc.poll() is not None:return self._fail('camera_process_stopped')
            heading=-runtime.course*gyro
            self.relative_heading+=(heading-self.previous_heading+180.0)%360.0-180.0;self.previous_heading=heading
            progress=distance-self.origin
            if progress<0 or progress>=MERGE_LIMIT_MM:return self._fail('line_not_found_within_distance')
            if s<=45 and 65<=v<=82:
                side=TraceSide.OPPOSITE if (self.direction==1)==(runtime.course==-1) else TraceSide.NORMAL
                boundary_lap=getattr(self.side_source,'lap',None)
                if boundary_lap is None:return self._fail('invalid_trial_context')
                self.trace=PostGateApproachTrace(owner=boundary_lap,name='demo line trace',target=trace_target_v(),power=TRACE_POWER,power_min=TRACE_POWER,
                    pid_p=.50,pid_i=.000001,pid_d=.045,trace_side=side)
                self.trace_origin=distance;self.phase='TRACE';self.last_trace_log=0.0
                self.logger.info('DEMO_HANDOFF progress_mm=%.1f relative_deg=%.2f floor_hsv=%s trace_side=%s target_v=%d' %
                                 (progress,self.relative_heading,(h,s,v),side.name,trace_target_v()))
                self._start_trace_projection()
                return self._update_projected_trace()
            target=self.direction*(-runtime.course)*MERGE_PEAK_DEG*math.sin(math.pi*progress/MERGE_LIMIT_MM)**2
            self.pid.setpoint=target;turn=max(-20.0,min(20.0,float(self.pid(self.relative_heading))))
            left=round(MERGE_POWER-runtime.course*turn);right=round(MERGE_POWER+runtime.course*turn)
            runtime.left_motor.set_brake(False);runtime.right_motor.set_brake(False)
            runtime.left_motor.set_power(left);runtime.right_motor.set_power(right)
            self.logger.info('DEMO_MERGE mono_sec=%.6f progress=%.1f target=%.2f relative=%.2f hsv=%s left=%d right=%d' % (now,progress,target,self.relative_heading,(h,s,v),left,right))
            return Status.RUNNING
        result=super().update()
        if self.phase=='TRACE' and self.post_trace_projection is None:
            self._start_trace_projection()
        return result




class _CurrentVideoSession:
    """Compatibility view of the one production VideoThread, without owning a camera."""
    def __init__(self):
        import threading
        self.lock = threading.RLock()
        self.proc = self
    def poll(self):
        return None
    @property
    def latest(self):
        return runtime.video.get_lap_observation() if runtime.video is not None else None


class _StartLapObservations(Behaviour):
    def __init__(self):
        super().__init__('enable LAP camera observations')
    def update(self):
        runtime.require('video')
        runtime.video.set_lap_observation(False)
        if BOTTLE_AIM:
            _bottle_aim_prepare()
        return Status.SUCCESS


class _DeferredBottleObservation(Behaviour):
    """Start the shared bottle session only near the final straight."""
    def __init__(self, context, lap):
        super().__init__('LAP bottle observation window')
        self.lap = lap
        self.child = DetectBottleColor('LAP detect bottle while moving', context,
                                      min_area=150, min_frames=3, while_moving=True)
        self.started = False
    def initialise(self):
        self.started = False
        self.aim_last_frame = -1
        self.aim_logged = 0
        self.sonar_next = 0.0
        self.sonar_logged = 0
    def _log_sonar(self):
        # 2026-11-18(devSS): ボトルがアームの間に入ったことを前の超音波センサーで分かるかの調べ。見るだけ。
        # 相撲と同じく 0.05 秒ごとに読み、進んだ距離と一緒に出す(最大200行)。
        now = time.monotonic()
        if self.sonar_logged >= 200 or now < self.sonar_next or runtime.sonar_sensor is None:
            return
        self.sonar_next = now + 0.05
        try:
            raw = runtime.sonar_sensor.get_distance()
        except Exception as error:
            self.sonar_logged = 200
            self.logger.warning('LAP_SONAR unavailable: %r' % (error,))
            return
        self.sonar_logged += 1
        progress = float(runtime.plotter.get_distance()) - self.lap.origin_distance
        self.logger.info('LAP_SONAR progress_mm=%.1f raw=%s' % (progress, raw))
    def _log_bottle_aim(self):
        # 2026-11-18(devSS): ボトルをつかみ損ねる(片方のアームに当たって外へそれる)調べ。見るだけ。
        # 新しい画像ごとに、選んだボトルの画面上の位置を出す。床の上の位置は PC でカメラの較正から求める
        # (u=2*cx, v=2*bottom+60 が 640x480 の画素。bottom は色の帯の下端で、床からの高さのぶん遠くに出る)。
        if self.aim_logged >= 200:
            return
        try:
            session, frame_id, observation = runtime.video.get_bottle_observation()
        except Exception:
            return
        if frame_id is None or frame_id <= self.aim_last_frame:
            return
        self.aim_last_frame = frame_id
        insight, color, cx, theta, bottom, area, blind = observation
        if not insight:
            return
        self.aim_logged += 1
        progress = float(runtime.plotter.get_distance()) - self.lap.origin_distance
        gyro = float(runtime.gyro_sensor.get_angle())
        lap_phase = getattr(self.lap, 'transition_phase', None)
        self.logger.info('BOTTLE_AIM_OBS frame=%d progress_mm=%.1f gyro_deg=%.2f heading_origin=%s lap_phase=%s fallback=%s '
                         'color=%s cx=%d bottom=%d area=%d blind=%s theta=%.2f'
                         % (frame_id, progress, gyro, getattr(self.lap, 'heading_origin', None), lap_phase,
                            getattr(self.lap, 'fallback_done', None), getattr(color, 'value', color),
                            int(cx), int(bottom), int(area), bool(blind), float(theta)))
    def update(self):
        if not self.started:
            if not self.lap.running:
                return Status.RUNNING
            progress = float(runtime.plotter.get_distance()) - self.lap.origin_distance
            if progress < 4300.0:
                return Status.RUNNING
            self.started = True
        self.child.tick_once()
        self._log_bottle_aim()
        self._log_sonar()
        return self.child.status
    def terminate(self, status):
        if self.started:
            self.child.stop(status)


class _FinishLapApproach(Behaviour):
    """Record completed motion once so AT does not repeat its approach."""
    def __init__(self, context, lap):
        super().__init__('finish LAP bottle approach')
        self.context, self.lap = context, lap
    def update(self):
        result = 'fallback' if self.lap.fallback_done else 'confirmed'
        if result == 'confirmed' and not self.lap.gate_latched:
            self.logger.error('LAP_HANDOFF missing confirmed gate')
            return Status.FAILURE
        self.context.lap_bottle_approach_result = result
        self.logger.info('LAP_BOTTLE_APPROACH_COMPLETE result=%s gate_confirmed=%s bottle_color=%s' %
                         (result, self.lap.gate_latched, self.context.bottle_color))
        runtime.video.set_lap_observation(False)
        return Status.SUCCESS


def build_lap_camera_recovery(context, config, original):
    """Attach v83 recovery to the exact devTK profile selected by the normal tree."""
    print('LAP_WHITE_SIDED_REJOIN mode=%s' % ('on' if white_sided_rejoin_enabled() else 'off'), flush=True)
    if isinstance(original, Sequence):
        if len(original.children) != 1:
            raise ValueError('devTK LAP sequence is not a single gyro profile')
        original = original.children[0]
    if not isinstance(original, DevTKRunByGyro) or not original.distance_target:
        raise ValueError('devTK LAP requires distance profile')
    lap = Peak100ContinuousLap(
        name=original.name, target=original.target, power=original.power,
        pid_p=original.pid_p, pid_i=original.pid_i, pid_d=original.pid_d,
        target_type=original.target_type, distance_limit_mm=original.distance_limit_mm,
        feedforward_gain=original.feedforward_gain, wheel_tread_mm=original.wheel_tread_mm,
        cross_track_lookahead_mm=original.cross_track_lookahead_mm,
        max_heading_correction_deg=original.max_heading_correction_deg,
        profile_log_interval_sec=original.profile_log_interval_sec)
    lap.devtk_speed_config = original.config if isinstance(original, RunByGyroSpeedDownatCurve) else None
    default_straight_power = config.start_lap_final_straight_power if config.mission_mode == "full" else 100
    straight_power = os.environ.get("LAP_STRAIGHT_POWER", str(default_straight_power))
    if straight_power not in ("100", "90", "85", "80", "75"):
        raise ValueError("LAP_STRAIGHT_POWER must be 100, 90, 85, 80 or 75")
    lap.straight_power = int(straight_power)
    window = final_zero_window(original.target, original.distance_limit_mm, decision_offset_mm=decision_offset_mm())
    if window is None:
        lap.observation_start_mm = lap.camera_start_mm = lap.camera_deadline_mm = None
        print('LAP_FINAL_STRAIGHT unavailable; continue profile and use gate-distance fallback', flush=True)
    else:
        straight_start, lap.observation_start_mm, lap.camera_start_mm, lap.camera_deadline_mm = window
        lap.straight_start_mm = straight_start
        print("LAP_STRAIGHT_POWER start_mm=%.1f power=%d" % (straight_start, lap.straight_power), flush=True)
        print('LAP_FINAL_STRAIGHT start_mm=%.1f observation_mm=%.1f decision_mm=%.1f deadline_mm=%.1f' % window, flush=True)
    lap.profile_white_rejoin = True
    lap.line_entry = 'cross_rejoin'
    lap.gate_recovery = 'baseline'
    lap.confirmed_fallback_slow = True
    lap.confirmed_fallback_after_mm = 120.0
    if UNSEEN_GATE_SHORTEN_MM:
        lap.fallback_estimated_gate_mm = float(lap.distance_limit_mm) - UNSEEN_GATE_SHORTEN_MM
    print('LAP_UNSEEN_GATE estimated_gate_mm=%.1f shorten_mm=%.1f' % (lap.fallback_estimated_gate_mm, UNSEEN_GATE_SHORTEN_MM), flush=True)
    gate = BlueThenBlackGate(lap, allow_weak_blue=True)
    lap.completion_condition, lap.completion_min_mm = gate, lap.observation_start_mm if lap.observation_start_mm is not None else math.inf
    continuous_to = config.mission_mode in ('lap-to', 'lap-to-sumo', 'full')
    session = _CurrentVideoSession()
    side = RecentSide(session, lap)
    lap.side_source = side
    lap.profile_rejoin_source = RecentSide(session, lap, rejoin=True)
    motion = Sequence(name='LAP gate then bottle approach', memory=True)
    motion.add_children([lap, EnableGyroScale(),
                         Timeout(name='LAP post-gate approach timeout',
                                 child=RollingCameraMerge(session, side, post_gate_steer='gyro', continuous_exit=continuous_to), duration=10)])
    run = Parallel(name='LAP camera recovery and bottle color', policy=ParallelPolicy.SuccessOnOne())
    floor_recorder = FloorMotionRecorder(lap, defer_output=continuous_to)
    run.add_children([floor_recorder, side, lap.profile_rejoin_source, motion,
                      _DeferredBottleObservation(context, lap)])
    root = Sequence(name='LAP recovery and bottle approach', memory=True)
    root.lap_controller, root.floor_recorder = lap, floor_recorder
    root_children = [_StartLapObservations(), Timeout(name='LAP recovery timeout', child=run, duration=60)]
    if not continuous_to:
        root_children.append(StopNow(name='LAP approach stop'))
    root_children.append(_FinishLapApproach(context, lap))
    if not continuous_to:
        root_children.append(DumpTransitionDiagnostics(lap))
    root.add_children(root_children)
    print('LAP_REACQUIRE_POLICY mode=%s limit_mm=%.1f limit_sec=%.2f' % (REACQUIRE_MODE, REACQUIRE_LIMIT_MM, REACQUIRE_LIMIT_SEC), flush=True)
    print('LAP_GRAY_BLUE_POLICY target=%s weak=%s gray_from_blue_mm=%.1f approach_target_mm=%.1f' % (GATE_TARGET_MODE, WEAK_BLUE_MODE, GRAY_CENTER_FROM_BLUE_MM, BLUE_APPROACH_TARGET_MM), flush=True)
    print('LAP_CAMERA_REFERENCE_X value=%.1f' % lap_camera_reference_x(), flush=True)
    print('LAP_DEVTK_PROFILE source=%s correction_start_mm=%s' % (type(original).__name__, lap.camera_start_mm), flush=True)
    print('LAP_RECOVERY enabled=True trace_target_v=%d gate=combined post_gate=gyro confirmed_mm=120 fallback_unseen_mm=0 fallback_blue_mm=120' % trace_target_v(), flush=True)
    print('LAP_ADAPTIVE_PEAK_POLICY far_peak_deg=%.1f initial_offset_px=%.1f ramp_mm=%.1f' % (far_peak_deg(), ADAPTIVE_INITIAL_OFFSET_PX, ADAPTIVE_RAMP_MM), flush=True)
    print('LAP_DARK_ENTRY_EDGE_POLICY mode=%s' % dark_entry_edge_mode(), flush=True)
    print('LAP_GATE_DISTANCE_POLICY short_blue_lt_mm=%.1f short_after_black_mm=120 blue_unconfirmed=estimated_gate' % SHORT_BLUE_MAX_MM, flush=True)
    print('LAP_UNSEEN_ESTIMATE after_slow_mm=%s slow_before_estimated_gate_mm=%.1f' % (unseen_estimate_distance(), UNSEEN_ESTIMATE_SLOW_BEFORE_GATE_MM), flush=True)
    return root
