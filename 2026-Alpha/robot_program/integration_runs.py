"""Consecutive integration runs using the production feature builders."""
import time
from dataclasses import replace

from py_trees.common import Status
from py_trees.behaviour import Behaviour
from py_trees.decorators import Decorator

from .runtime import runtime
from .behaviours.bottle import _color_value
from py_etrobo_util import BottleColor
from .behaviours.handoff import CaptureAtToHandoff
from .behaviours.motor_control import StopNow
from .features.catch_bottle import build_catch_bottle
from .features.to_hint_route import build_tantou_tree
from .phases.bottle_and_rally_preparation import build_bottle_delivery_final_phase
from .phases.et_rally import build_et_rally_phase
from .phases.et_sumo import build_et_sumo_phase
from .phases.finish import build_finish_phase


def valid_color(context):
    return _color_value(context.bottle_color) in {
        BottleColor.RED.value, BottleColor.BLUE.value, BottleColor.YELLOW.value}


def valid_handoff(context):
    return (valid_color(context) and context.at_to.distance_mm is not None
            and context.at_to.heading_deg is not None)


def valid_hints(context):
    return bool(context.hint1 and context.hint2 and context.hint1 != context.hint2)


class ObservedStage(Decorator):
    """Observe the real subtree without resetting devices or shared state."""
    def __init__(self, name, child, context, completed):
        super().__init__(name=name, child=child)
        self.context, self.completed = context, completed
        self.started_at = None

    def initialise(self):
        self.started_at = time.monotonic()
        self.report('START', Status.RUNNING)

    def update(self):
        status = self.decorated.status
        if status == Status.SUCCESS and not self.completed(self.context):
            self.logger.error('INTEGRATION missing handoff state: ' + self.name)
            return Status.FAILURE
        return status

    def terminate(self, new_status):
        if self.started_at is not None:
            self.report('END', new_status)
            self.started_at = None

    def report(self, event, status):
        c = self.context
        distance = runtime.plotter.get_distance() if runtime.plotter is not None else None
        gyro = runtime.gyro_sensor.get_angle() if runtime.gyro_sensor is not None else None
        elapsed = time.monotonic() - self.started_at if self.started_at is not None else 0.0
        self.logger.info(
            'INTEGRATION %s stage=%s status=%s elapsed=%.3f distance=%s gyro=%s '
            'color=%s at_to=%s hints=%s/%s delivered=%s ready=%s '
            'strategy=%s commands=%d sumo_line=%s sumo_ready=%s' % (
                event, self.name, status.name, elapsed, distance, gyro,
                c.bottle_color, c.at_to, bool(c.hint1), bool(c.hint2),
                c.bottle_delivered, c.rally_ready, c.strategy_status, len(c.strategy),
                c.sumo.garage_line_found, c.sumo.line_trace_ready))


class EnableStrategyRequests(Behaviour):
    def __init__(self, context):
        super().__init__('Enable Strategy requests after touch')
        self.context = context

    def update(self):
        self.context.strategy_requests_enabled = True
        return Status.SUCCESS


def sumo_exit_ready(context):
    return bool(context.sumo.garage_line_found and context.sumo.line_trace_ready
                and context.sumo.transport_completed)


def build_later_integration_children(context, config):
    mode = config.mission_mode
    nodes = []
    if mode in ('bottle-rally', 'rally-sumo'):
        if not context.hint1 or not context.hint2_gate_info:
            raise ValueError('Manual decoded rally hints are required')
        nodes.append(EnableStrategyRequests(context))
    if mode == 'bottle-rally':
        if not valid_color(context):
            raise ValueError('bottle-rally requires the held bottle color')
        nodes.append(ObservedStage('BOTTLE', build_bottle_delivery_final_phase(
            context, config), context, lambda c: c.bottle_delivered and c.rally_ready))
    if mode in ('bottle-rally', 'rally-sumo'):
        nodes.append(ObservedStage('RALLY', build_et_rally_phase(context, config),
                                   context, lambda c: bool(c.strategy)))
    if mode in ('rally-sumo', 'sumo-garage'):
        nodes.append(ObservedStage('SUMO', build_et_sumo_phase(context, config),
                                   context, sumo_exit_ready))
    if mode == 'sumo-garage':
        # Same blue-marker approach and final straight as production FINISH.
        nodes.append(ObservedStage('GARAGE', build_finish_phase(context, config),
                                   context, lambda c: True))
    nodes.append(StopNow(name='Integration range complete brake'))
    return nodes


def build_integration_children(context, config):
    mode = config.mission_mode
    if mode in ('bottle-rally', 'rally-sumo', 'sumo-garage'):
        return build_later_integration_children(context, config)
    if mode not in ('at-to', 'to-bottle', 'at-to-bottle'):
        raise ValueError('Unknown integration run: ' + mode)
    nodes = []
    if mode == 'to-bottle':
        if not valid_color(context):
            raise ValueError('to-bottle requires the held bottle color')
        nodes.append(CaptureAtToHandoff('TO integration placement origin', context))
    else:
        # Only entry setup differs: same blue-line approach as AT standalone.
        nodes.append(ObservedStage('AT', build_catch_bottle(
            context, replace(config, mission_mode='at')), context, valid_handoff))
    nodes.append(ObservedStage('TO', build_tantou_tree(context, config), context,
                               lambda c: valid_handoff(c) and valid_hints(c)))
    if mode != 'at-to':
        nodes.append(ObservedStage('BOTTLE', build_bottle_delivery_final_phase(
            context, config), context,
            lambda c: c.bottle_delivered and c.rally_ready))
    nodes.append(StopNow(name='Integration range complete brake'))
    return nodes
