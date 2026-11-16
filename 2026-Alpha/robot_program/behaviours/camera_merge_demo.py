"""Trial-only camera side -> bounded smooth approach -> color line tracing."""
import math,time
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from simple_pid import PID
from py_etrobo_util import TraceSide
from ..runtime import runtime
from ..timing import CONTROL_INTERVAL_SEC
from .line_trace import TraceLine

# Trial assumptions, not calibrated production settings or physical distances.
IMAGE_EDGE_X=186.0
IMAGE_DEADBAND=6.0
CAMERA_MAX_AGE_SEC=.25
CHOOSE_TIMEOUT_SEC=3.0
MERGE_LIMIT_MM=400.0
MERGE_PEAK_DEG=5.0
MERGE_POWER=40
TRACE_MM=200.0
TRACE_POWER=40

class ForwardOnlyTrace(TraceLine):
    def set_motor_power(self,left_power,right_power):
        super().set_motor_power(max(0,min(100,left_power)),max(0,min(100,right_power)))

class CameraMergeDemo(Behaviour):
    def __init__(self,session,allow_both_courses=False):
        super().__init__('camera merge demo');self.session=session;self.allow_both_courses=allow_both_courses
    def initialise(self):
        self.phase='CHOOSE';self.started=time.monotonic();self.last_frame=None
        self.stream=None;self.candidate=None;self.streak=0;self.trace=None;self.white_count=0
        self._stop()
    def _stop(self):
        for motor in (runtime.left_motor,runtime.right_motor):
            if motor is not None:motor.set_power(0);motor.set_brake(True)
    def _fail(self,reason):
        self.logger.error('DEMO_FAIL phase=%s reason=%s' % (self.phase,reason));self._stop();return Status.FAILURE
    def update(self):
        runtime.require('left_motor','right_motor','color_sensor','gyro_sensor','plotter')
        now=time.monotonic();h,s,v=runtime.color_sensor.get_raw_color_hsv()
        distance=float(runtime.plotter.get_distance());gyro=float(runtime.gyro_sensor.get_angle())
        if runtime.course not in (-1,1) or not all(map(math.isfinite,(distance,gyro))):return self._fail('invalid_sensors')
        if runtime.course!=-1 and not self.allow_both_courses:return self._fail('trial_supports_right_course_only')
        proc=getattr(self.session,'proc',None)
        if proc is not None and proc.poll() is not None:return self._fail('camera_process_stopped')
        heading=-runtime.course*gyro
        if self.phase=='CHOOSE':
            if now-self.started>CHOOSE_TIMEOUT_SEC:return self._fail('camera_side_unknown')
            if not (s<=45 and 90<=v<=100):return self._fail('initial_floor_not_white')
            with self.session.lock:obs=self.session.latest
            if not obs:return Status.RUNNING
            captured=obs.get('capture_time')
            if not isinstance(captured,(int,float)) or not math.isfinite(captured):
                self.candidate=None;self.streak=0;return Status.RUNNING
            age=now-captured
            line=obs.get('line') or {};x=line.get('x_px')
            if not math.isfinite(age) or not 0<=age<=CAMERA_MAX_AGE_SEC or not line.get('valid') or line.get('track_state')!='TRACKED' or not isinstance(x,(int,float)) or not math.isfinite(x) or not 0<=x<320 or line.get('image_width')!=320:
                self.candidate=None;self.streak=0;return Status.RUNNING
            stream=(obs.get('stream_id'),obs.get('session'))
            if not isinstance(stream[0],str) or not stream[0] or not isinstance(stream[1],int):return self._fail('camera_context_missing')
            if self.stream is not None and stream!=self.stream:return self._fail('camera_context_changed')
            self.stream=stream;frame=obs.get('frame')
            if not isinstance(frame,int):return self._fail('camera_frame_invalid')
            if frame==self.last_frame:return Status.RUNNING
            if self.last_frame is not None and frame<self.last_frame:return self._fail('camera_frame_reversed')
            self.last_frame=frame
            side='RIGHT' if x-IMAGE_EDGE_X>IMAGE_DEADBAND else 'LEFT' if x-IMAGE_EDGE_X < -IMAGE_DEADBAND else 'UNKNOWN'
            self.logger.info('DEMO_SIDE_OBS frame=%d age_ms=%.1f x=%.1f side=%s floor_hsv=%s' % (frame,age*1000,x,side,(h,s,v)))
            if side=='UNKNOWN':self.candidate=None;self.streak=0;return Status.RUNNING
            self.streak=self.streak+1 if side==self.candidate else 1;self.candidate=side
            if self.streak<3:return Status.RUNNING
            self.direction=1 if side=='RIGHT' else -1
            self.initial_heading=heading;self.previous_heading=heading;self.relative_heading=0.0;self.origin=distance
            self.pid=PID(1.8,0.0,.03,setpoint=0,sample_time=CONTROL_INTERVAL_SEC,output_limits=(-20,20))
            self.phase='MERGE'
            self.logger.info('DEMO_SIDE frozen=%s frame=%s x=%.1f stream=%s session=%s; assumptions=trial_uncalibrated' % (side,frame,x,*stream))
        if self.phase=='MERGE':
            self.relative_heading+=(heading-self.previous_heading+180.0)%360.0-180.0;self.previous_heading=heading
            progress=distance-self.origin
            if progress < 0 or progress>=MERGE_LIMIT_MM:return self._fail('line_not_found_within_distance')
            if s<=45 and 65<=v<=82:
                # White floor approached the target edge: continue without an inserted wait.
                right_side=self.direction==1
                side=TraceSide.OPPOSITE if right_side==(runtime.course==-1) else TraceSide.NORMAL
                self.trace=ForwardOnlyTrace(name='demo line trace',target=75,power=TRACE_POWER,power_min=TRACE_POWER,
                                     pid_p=.50,pid_i=.000001,pid_d=.045,trace_side=side)
                self.trace_origin=distance;self.phase='TRACE';self.last_trace_log=0.0
                self.logger.info('DEMO_HANDOFF progress_mm=%.1f relative_deg=%.2f floor_hsv=%s trace_side=%s' % (progress,self.relative_heading,(h,s,v),side.name))
            else:
                target=self.direction*(-runtime.course)*MERGE_PEAK_DEG*math.sin(math.pi*progress/MERGE_LIMIT_MM)**2
                self.pid.setpoint=target;turn=max(-20.0,min(20.0,float(self.pid(self.relative_heading))))
                left=round(MERGE_POWER-runtime.course*turn);right=round(MERGE_POWER+runtime.course*turn)
                runtime.left_motor.set_brake(False);runtime.right_motor.set_brake(False)
                runtime.left_motor.set_power(left);runtime.right_motor.set_power(right)
                self.logger.info('DEMO_MERGE mono_sec=%.6f progress=%.1f target=%.2f relative=%.2f hsv=%s left=%d right=%d' % (now,progress,target,self.relative_heading,(h,s,v),left,right))
                return Status.RUNNING
        if self.phase=='TRACE':
            progress=distance-self.trace_origin
            if progress<0:return self._fail('reverse_distance')
            if progress>=TRACE_MM:
                self._stop();self.logger.info('DEMO_TRACE_DONE distance_mm=%.1f' % progress);return Status.SUCCESS
            self.white_count=self.white_count+1 if v>=95 and s<=45 else 0
            if self.white_count>=12:return self._fail('trace_lost_white')
            status=self.trace.update()
            if status!=Status.RUNNING:return self._fail('trace_failed')
            if now-self.last_trace_log>=.1:
                self.logger.info('DEMO_TRACE mono_sec=%.6f distance_mm=%.1f hsv=%s white_count=%d' % (now,progress,(h,s,v),self.white_count));self.last_trace_log=now
            return Status.RUNNING
        return self._fail('invalid_phase')
    def terminate(self,status):
        if self.trace is not None:self.trace.terminate(status)
        self._stop()
