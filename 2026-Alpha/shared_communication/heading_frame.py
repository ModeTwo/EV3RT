"""Heading contract at the planner/transport boundary; no route geometry here."""
import math

HEADING_FRAME = 'full-start-course-normalized-v1'
RALLY_START_HEADING = -90.0

def normalize(angle):
    return (angle + 180.0) % 360.0 - 180.0

def map_headings(steps, transform):
    if not isinstance(steps, list) or not steps:
        raise ValueError('Strategy must be a non-empty list')
    result = []
    for step in steps:
        if not isinstance(step, dict) or step.get('type') not in ('move', 'turn'):
            raise ValueError('Expected move or turn command')
        angle = step.get('target_heading_deg')
        if isinstance(angle, bool) or not isinstance(angle, (int, float)) or not math.isfinite(angle):
            raise ValueError('Expected finite target_heading_deg')
        result.append(dict(step, target_heading_deg=normalize(transform(angle))))
    return result

def planner_to_full_start(steps, course):
    # Planner angles are CCW-positive, relative to rally start, already mirrored
    # geometrically for Right. Robot headings use course-normalized CCW angles.
    if course not in ('left', 'right'):
        raise ValueError('Invalid course')
    sign = 1 if course == 'left' else -1
    return map_headings(steps, lambda angle: RALLY_START_HEADING + sign * angle)

def full_start_to_gyro(steps, mission_mode):
    # ResetDevice zeros the gyro before driving. A rally-entry profile
    # starts physically at the rally entrance. Never use arrival measurements
    # as an origin: that would propagate arrival errors into every target.
    # Bottle-final placement faces the delivery line, 180 degrees from full start.
    initial_heading = (-180.0 if mission_mode == 'bottle-rally' else
                       RALLY_START_HEADING if mission_mode in ('rally-drive', 'rally-sumo')
                       else 0.0)
    return map_headings(steps, lambda angle: angle - initial_heading)
