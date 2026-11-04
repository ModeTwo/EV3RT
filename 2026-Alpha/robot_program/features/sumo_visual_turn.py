"""Robot IO adapter for image-guided turning; alpha owns all device lifecycle.

画像旋回方針と実モーターIOを接続するBT動作。正対終了や異常時は制動する。"""

import time
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from ..runtime import runtime
from .sumo_visual_policy import VisualTurnPolicy


class VisualBottleTurn(Behaviour):
    def __init__(self, name, context, bearing, session, settings, memory):
        super().__init__(name)
        self.context, self.bearing = context, bearing
        self.session, self.settings, self.memory = session, settings, memory
        self.policy = None
        self.telemetry = {}
        self.commands = []
        self.last_power = None
        self.phase = "READY"
        self.main_power = settings.visual_fast_power

    def _command(self, power):
        if power == self.last_power:
            return
        if power:
            try:
                runtime.left_motor.set_power(power)
                runtime.right_motor.set_power(-power)
            except Exception:
                self.last_power = None
                self._command(0)
                raise
        else:
            errors = []
            for motor in (runtime.left_motor, runtime.right_motor):
                for method, value in (("set_power", 0), ("set_brake", True)):
                    try:
                        getattr(motor, method)(value)
                    except Exception as error:
                        errors.append(error)
            if errors:
                raise errors[0]
        self.last_power = power
        if len(self.commands) < 512:
            self.commands.append(dict(t=time.monotonic(), power=power))

    def initialise(self):
        self.policy = VisualTurnPolicy(
            self.settings,
            self.memory,
            self.session,
            time.monotonic(),
            runtime.gyro_sensor.get_angle(),
        )
        self._command(0)

    def update(self):
        try:
            self.telemetry = self.policy.step(
                time.monotonic(),
                runtime.gyro_sensor.get_angle(),
                runtime.video.get_sumo_alignment_observation(self.session),
            )
            self.phase = self.telemetry["visual_stage"]
            self._command(self.telemetry["visual_power"])
            if self.phase == "DONE":
                return Status.SUCCESS
            elif self.phase == "FAILED":
                return Status.FAILURE
            else:
                return Status.RUNNING
        except Exception:
            self.last_power = None
            self._command(0)
            raise

    def terminate(self, new_status):
        self.last_power = None
        self._command(0)

    def result_details(self):
        return dict(
            initial_bearing_deg=self.bearing,
            session=self.session,
            final=dict(self.telemetry),
            commands=list(self.commands),
            samples=list(self.policy.trace) if self.policy else [],
        )
