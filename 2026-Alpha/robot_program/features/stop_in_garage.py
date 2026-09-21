"""Feature 20: drive from the blue marker into the garage and brake."""
from .bt_imports import Parallel, ParallelPolicy, Sequence
from py_trees.decorators import Timeout
from ..behaviours.conditions import IsDistanceEarned
from ..behaviours.motor_control import StopNow
from .sumo_bearing_motion import RunAtBearing


def build_stop_in_garage(context, config):
    root = Sequence(name="stop_in_garage", memory=True)
    # straight = Parallel(name="garage straight", policy=ParallelPolicy.SuccessOnOne())
    # straight.add_children([
    #     # Reference absolute 0 means garage direction. Use the registered course
    #     # bearing so sumo-only and full runs share the same physical direction.
    #     RunAtBearing(name="garage straight bearing", context=context,
    #                  bearing=config.sumo.garage_bearing_deg,
    #                  power=60, pid_p=1.1, pid_i=0.1, pid_d=0.03),
    #     IsDistanceEarned(name="distance from garage blue",
    #                      delta_dist=config.garage_goal_distance_mm),
    # ])
    root.add_children([
        # Timeout(name="garage straight timeout", child=straight,
        #         duration=config.garage_straight_timeout_sec),
        StopNow(name="garage final brake"),
    ])
    return root
