"""Repeated labelled static measurements in one alpha session.

配置キー・測定開始/途中保存/終了を管理し、各測定区間はSumoSensorTestへ任せる。"""

from collections import Counter
from dataclasses import replace
from datetime import datetime, timezone
import time
from uuid import uuid4
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from ..runtime import runtime
from .sumo_sensor_test import SumoSensorTest
from .sumo_sensor_keys import TerminalKeys

LABEL_KEYS = {
    "1": "empty",
    "2": "ahead-far",
    "3": "ahead-near",
    "4": "mouth",
    "5": "inside",
    "6": "left-out",
    "7": "right-out",
    "8": "fallen",
}


class SensorSegment(SumoSensorTest):
    def __init__(self, context, settings, motion_settings, batch, repeat):
        super().__init__("keyboard sensor segment", context, settings, motion_settings)
        self.batch, self.repeat = batch, repeat
        self.complete = False
        self.end_reason = "interrupted"

    def _sample(self):
        super()._sample()
        self.samples[-1].update(repeat=self.repeat, interactive_session_id=self.batch)

    def _record_snapshot(self):
        row = super()._record_snapshot()
        row.update(probe_repeat=self.repeat, probe_interactive_session_id=self.batch)
        return row

    def _details(self):
        row = super()._details()
        row["interactive"] = dict(
            session_id=self.batch,
            repeat=self.repeat,
            segment_complete=self.complete,
            segment_end_reason=self.end_reason,
        )
        return row

    def _finish(self, status, reason):
        self.complete = status == Status.SUCCESS and reason.startswith(
            "labelled stationary measurement complete"
        )
        self.end_reason = reason
        return super()._finish(status, reason)

    def operator_stop(self, reason):
        if self.phase != "FINALIZING":
            self._finish(Status.SUCCESS, reason + "; partial measurement")


class InteractiveSumoSensorTest(Behaviour):
    def __init__(self, name, context, settings, motion_settings):
        super().__init__(name)
        self.context, self.settings, self.motion_settings = (
            context,
            settings,
            motion_settings,
        )
        self.child = None
        self.keys = None

    def _brake(self):
        errors = []
        for motor in (runtime.left_motor, runtime.right_motor):
            for method, value in (("set_power", 0), ("set_brake", True)):
                try:
                    getattr(motor, method)(value)
                except Exception as error:
                    errors.append(error)
        if errors:
            raise errors[0]

    def _help(self):
        self.logger.info(
            "SENSOR KEYS 1=empty 2=ahead-far 3=ahead-near 4=mouth 5=inside 6=left-out 7=right-out 8=fallen"
        )
        self.logger.info(
            "SENSOR KEYS SPACE/Enter/S=start, P=end current segment early, Q=save and finish, H=help"
        )
        if self.child is not None:
            self.logger.info(
                "SENSOR KEYS BUSY label=%s; keep placement unchanged until SAVED"
                % self.child.settings.label
            )
        else:
            self.logger.info(
                "SENSOR KEYS READY label=%s next_repeat=%d duration=%.1fs; place bottle, then SPACE"
                % (self.label, self.repeats[self.label] + 1, self.settings.duration_sec)
            )

    def initialise(self):
        runtime.require("left_motor", "right_motor", "video", "sonar_sensor")
        self._brake()
        self.child = None
        self.exit_requested = False
        self.repeats = Counter()
        self.saved_segments = 0
        self.label = self.settings.label or "empty"
        self.forward, self.lateral = self.settings.forward_mm, self.settings.lateral_mm
        self.batch = (
            datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ_") + uuid4().hex[:8]
        )
        self.phase = "INTERACTIVE_IDLE"
        self.keys = TerminalKeys()
        try:
            self.keys.open()
        except Exception:
            self.keys.close()
            raise
        self._help()

    def _start_segment(self):
        self.repeats[self.label] += 1
        settings = replace(
            self.settings,
            interactive=False,
            label=self.label,
            forward_mm=self.forward,
            lateral_mm=self.lateral,
        )
        self.child = SensorSegment(
            self.context,
            settings,
            self.motion_settings,
            self.batch,
            self.repeats[self.label],
        )
        self.phase = "INTERACTIVE_MEASURING"
        self.logger.info(
            "SENSOR KEYS START label=%s repeat=%d; keep bottle still until SAVED"
            % (self.label, self.repeats[self.label])
        )
        self.child.tick_once()

    def update(self):
        for key in self.keys.poll():
            key = key.lower()
            if key == "q":
                self.exit_requested = True
                if self.child:
                    self.child.operator_stop("operator quit")
            elif key == "h":
                self._help()
            elif self.exit_requested:
                continue
            elif self.child:
                if key == "p":
                    self.child.operator_stop("operator stopped segment")
                elif key in LABEL_KEYS or key in (" ", "\r", "\n", "s"):
                    self.logger.info(
                        "SENSOR KEYS BUSY: wait for SAVED; P ends this segment early"
                    )
            elif key in LABEL_KEYS:
                self.label = LABEL_KEYS[key]
                # A new physical placement must not inherit a previous measured coordinate.
                self.forward = self.lateral = None
                self._help()
            elif key in (" ", "\r", "\n", "s"):
                self._start_segment()
        if self.child is not None:
            self.child.tick_once()
            if self.child.status == Status.FAILURE:
                return Status.FAILURE
            if self.child.status == Status.SUCCESS:
                self.logger.info(
                    "SENSOR KEYS SAVED label=%s repeat=%d samples=%d complete=%s"
                    % (
                        self.child.settings.label,
                        self.child.repeat,
                        len(self.child.samples),
                        self.child.complete,
                    )
                )
                self.saved_segments += 1
                self.child = None
                self.phase = "INTERACTIVE_IDLE"
                if not self.exit_requested:
                    self._help()
        if self.exit_requested and self.child is None:
            return Status.SUCCESS
        else:
            return Status.RUNNING

    def terminate(self, new_status):
        try:
            if self.child is not None:
                self.child.stop(Status.INVALID)
                self.child = None
        finally:
            try:
                self._brake()
            finally:
                if self.keys is not None:
                    self.keys.close()
