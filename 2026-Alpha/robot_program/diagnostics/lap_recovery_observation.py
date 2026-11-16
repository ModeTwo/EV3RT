"""LAP observation and bounded diagnostics; floor observation also informs control."""
import json
import time
from collections import deque
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from ..runtime import runtime

class FloorMotionRecorder(Behaviour):
    """Bounded diagnostic-only samples; emit after the motion branch ends."""
    def __init__(self, lap, bottle_heading_recorder=None, *, defer_output=False):
        super().__init__('record lap floor samples')
        self.lap = lap
        self.bottle_heading_recorder = bottle_heading_recorder
        self.defer_output = defer_output
    def initialise(self):self.samples=deque(maxlen=500);self.total=0
    def update(self):
        if not self.lap.running and not self.lap.handoff_complete:return Status.RUNNING
        runtime.require('plotter','gyro_sensor','color_sensor')
        distance=float(runtime.plotter.get_distance());progress=distance-self.lap.origin_distance
        if progress<4300.0:return Status.RUNNING
        gyro_sensor = runtime.gyro_sensor
        if self.bottle_heading_recorder is not None:
            read_start = time.monotonic()
            gyro = float(gyro_sensor.get_angle())
            sampled = (read_start + time.monotonic()) / 2
            phase = self.lap.transition_phase or ('POST_BOUNDARY' if self.lap.handoff_complete else 'LAP')
            self.bottle_heading_recorder.record_gyro(sampled, gyro, gyro_sensor, phase)
        else:
            gyro = float(gyro_sensor.get_angle())
        hsv=tuple(runtime.color_sensor.get_raw_color_hsv())
        if self.lap.running and getattr(self.lap, 'line_entry', None) == 'cross_rejoin':
            self.lap._observe_early_line_exit(progress, gyro, hsv)
        phase='POST_BOUNDARY' if self.lap.handoff_complete else 'LAP'
        self.samples.append((time.monotonic(),progress,gyro,hsv,phase));self.total+=1
        return Status.RUNNING
    def terminate(self,status):
        if self.defer_output and status == Status.INVALID and self.lap.handoff_complete:
            return
        self.report()

    def report(self):
        self.logger.info('LAP_FLOOR_RECORD total=%d retained=%d dropped=%d sample_period=control_tick' %
                         (self.total,len(self.samples),self.total-len(self.samples)))
        for mono,progress,gyro,hsv,phase in self.samples:
            self.logger.info('LAP_FLOOR_SAMPLE mono_sec=%.6f progress_mm=%.1f gyro_deg=%.2f hsv=%s phase=%s' %
                             (mono,progress,gyro,hsv,phase))
        commands=getattr(self.lap,'trace_motor_samples',())
        total=getattr(self.lap,'trace_motor_total',0)
        self.logger.info('LAP_TRACE_MOTOR_RECORD total=%d retained=%d dropped=%d' % (total,len(commands),total-len(commands)))
        for mono,progress,left,right,heading,error,line,correction,mode in commands:
            self.logger.info('LAP_TRACE_MOTOR mono_sec=%.6f progress_mm=%.1f left=%d right=%d heading=%.2f error=%.2f line_delta=%.2f gyro_delta=%.2f mode=%s' % (mono,progress,left,right,heading,error,line,correction,mode))

class FlushLapFloorDiagnostics(Behaviour):
    """Write the deferred floor data after the whole test has stopped."""
    def __init__(self, recorder):
        super().__init__('dump deferred lap floor diagnostics')
        self.recorder = recorder
    def update(self):
        if self.recorder.lap.handoff_complete:
            self.recorder.report()
        return Status.SUCCESS


class DumpTransitionDiagnostics(Behaviour):
    """Write bounded observations after the trial's explicit motor stop."""
    def __init__(self,lap):super().__init__('dump transition diagnostics');self.lap=lap
    def update(self):
        samples=self.lap.transition_samples
        self.logger.info('LAP_DIAG total=%d retained=%d dropped=%d' % (self.lap.transition_total,len(samples),self.lap.transition_total-len(samples)))
        for row in samples:self.logger.info('LAP_DIAG_SAMPLE '+json.dumps(row,separators=(',',':'),allow_nan=False))
        return Status.SUCCESS


