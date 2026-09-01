#!/usr/bin/env python3
"""Insert or remove an obstacle in the running Gazebo world.

Use this to demonstrate dynamic replanning by hand: start the simulation, send
the robot a goal, then run this while it is driving. The obstacle is a real
Gazebo model, so the lidar detects it, the Nav2 costmaps update, the current
path becomes invalid and Nav2 plans a new route by itself.

    # from the package directory (no ros2 run / sourcing required):
    python3 scripts/obstacle_injector.py --world simple_trees --x 0.0 --y -3.5 --name wall_1

    # via ros2 run (source ~/G25_RS1/RS1-Gr25/install/setup.bash first):
    ros2 run 41068_ignition_bringup obstacle_injector.py --x 1.5 --y -3.0

    # take it away again
    ros2 run 41068_ignition_bringup obstacle_injector.py --remove

The obstacle must be taller than the Husky's lidar plane (0.845 m) or it will
never be detected; the default height of 1.5 m satisfies this.
"""

import argparse
import os
import sys

# Allow `python3 scripts/obstacle_injector.py` without sourcing the workspace.
_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from rs1_nav import GazeboWorld, ObstacleSpec


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Insert or remove an obstacle in a running Gazebo world.',
    )
    parser.add_argument('--world', default='simple_trees',
                        help='Gazebo world name (default: simple_trees)')
    parser.add_argument('--name', default='dynamic_barrier',
                        help='Model name to insert or remove')
    parser.add_argument('--x', type=float, default=1.5, help='Obstacle centre X in metres')
    parser.add_argument('--y', type=float, default=0.0, help='Obstacle centre Y in metres')
    parser.add_argument('--size-x', type=float, default=0.4, help='Thickness along X')
    parser.add_argument('--size-y', type=float, default=3.0, help='Width along Y')
    parser.add_argument('--size-z', type=float, default=1.5,
                        help='Height; must exceed the 0.845 m lidar plane')
    parser.add_argument('--remove', action='store_true',
                        help='Remove the named model instead of inserting it')
    parser.add_argument('--replace', action='store_true',
                        help='Remove any existing model of this name before inserting')
    parser.add_argument('--wait', type=float, default=30.0,
                        help='Seconds to wait for the Gazebo world services (bounded)')
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    world = GazeboWorld(args.world)

    if not world.wait_until_available(max_wait=args.wait):
        print(
            f'Gazebo world "{args.world}" did not answer within {args.wait:.0f}s. '
            'Is the simulation running, and is the world name correct?',
            file=sys.stderr,
        )
        return 1

    if args.remove:
        return 0 if world.remove_model(args.name) else 1

    if args.replace:
        world.remove_model(args.name)

    spec = ObstacleSpec(
        name=args.name,
        x=args.x,
        y=args.y,
        size_x=args.size_x,
        size_y=args.size_y,
        size_z=args.size_z,
    )
    return 0 if world.spawn_obstacle(spec) else 1


if __name__ == '__main__':
    sys.exit(main())
