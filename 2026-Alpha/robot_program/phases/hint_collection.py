"""The implemented RE -> AT -> TO mission ends in a stationary hold."""
from py_trees.composites import Sequence
from ..features.catch_bottle import build_catch_bottle
from ..features.to_hint_route import build_tantou_tree
from ..behaviours.motor_control import StopNow
from py_trees.behaviour import Behaviour
from py_trees.common import Status


class ReportHints(Behaviour):
    def __init__(self, context):
        super().__init__('Report acquired hints')
        self.context = context

    def update(self):
        c = self.context
        if not c.bottle_color or not c.hint1 or not c.hint2 or c.hint1 == c.hint2:
            self.logger.error('Hint mission ended without distinct hints and bottle color')
            return Status.FAILURE
        self.logger.info('HINT_COMPLETE bottle_color=%s hint1=%r hint2=%r' %
                         (c.bottle_color, c.hint1, c.hint2))
        return Status.SUCCESS


def build_hint_collection_phase(context, config):
    root = Sequence(name='AT_TO hint collection', memory=True)
    nodes = [
        build_catch_bottle(context, config),
        build_tantou_tree(context, config,
                          include_exit=config.mission_mode == 'hint2-return'),
    ]
    nodes.append(StopNow(name='Hint mission complete brake'))
    nodes.append(ReportHints(context))
    root.add_children(nodes)
    return root
