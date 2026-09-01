"""Shared implementation for the 41068 autonomous navigation demo.

Three layers, each usable on its own:

    gazebo_world   insert and remove real obstacles in a running simulation
    nav_observer   subscribe to the navigation stack and send Nav2 goals
    mission        run a bounded goal and report what the stack actually did

Installed as a package rather than duplicated inside each script, so the demo,
the obstacle injector and the whole test suite run exactly the same code.
"""

from .gazebo_world import (
    GazeboWorld,
    ObstacleSpec,
    PathBlocker,
    barrier_across_heading,
)
from .geometry import (
    path_closest_approach,
    path_heading,
    path_length,
    point_ahead_on_path,
    quaternion_from_yaw,
    yaw_from_quaternion,
)
from .mission import (
    ARRIVAL_TOLERANCE,
    MissionReport,
    MissionRunner,
    ReplanEvent,
    plan_divergence,
)
from .nav_observer import (
    NavObserver,
    PathSnapshot,
    init_ros,
)

__all__ = [
    'ARRIVAL_TOLERANCE',
    'GazeboWorld',
    'MissionReport',
    'MissionRunner',
    'NavObserver',
    'ObstacleSpec',
    'PathBlocker',
    'PathSnapshot',
    'ReplanEvent',
    'barrier_across_heading',
    'init_ros',
    'path_closest_approach',
    'path_heading',
    'path_length',
    'plan_divergence',
    'point_ahead_on_path',
    'quaternion_from_yaw',
    'yaw_from_quaternion',
]
