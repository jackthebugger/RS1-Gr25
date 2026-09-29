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
from .forest_obstacle_manager import (
    PATH_A,
    PATH_B,
    ForestGapConfig,
    ForestObstacleManager,
    GapSpec,
    ObstacleState,
    load_forest_gap_config,
)
from .geometry import (
    clearance_to_points,
    costmap_obstacle_cells,
    path_closest_approach,
    path_cost_stats,
    path_heading,
    path_length,
    path_obstacle_clearances,
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
    'PATH_A',
    'PATH_B',
    'ForestGapConfig',
    'ForestObstacleManager',
    'GapSpec',
    'GazeboWorld',
    'MissionReport',
    'MissionRunner',
    'NavObserver',
    'ObstacleSpec',
    'ObstacleState',
    'PathBlocker',
    'PathSnapshot',
    'ReplanEvent',
    'barrier_across_heading',
    'clearance_to_points',
    'costmap_obstacle_cells',
    'init_ros',
    'load_forest_gap_config',
    'path_closest_approach',
    'path_cost_stats',
    'path_heading',
    'path_length',
    'path_obstacle_clearances',
    'plan_divergence',
    'point_ahead_on_path',
    'quaternion_from_yaw',
    'yaw_from_quaternion',
]
