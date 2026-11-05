#!/usr/bin/env python3
"""Remove a stale Ignition model before ros_ign_gazebo create.

Orphan Gazebo servers keep husky1 at its last pose. A subsequent create with
the same name and allow_renaming=false (default) often no-ops while still
reporting success, so the operator sees the robot at the old end-goal pose.
This helper removes the model (safe if absent) and logs the intended spawn.
"""

from __future__ import annotations

import argparse
import os
import sys

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from rs1_nav.gazebo_world import GazeboWorld  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', required=True, help='Ignition world name')
    parser.add_argument('--name', required=True, help='Model name to remove')
    parser.add_argument('--x', default='', help='Intended spawn X (log only)')
    parser.add_argument('--y', default='', help='Intended spawn Y (log only)')
    parser.add_argument('--z', default='', help='Intended spawn Z (log only)')
    parser.add_argument('--yaw', default='', help='Intended spawn yaw (log only)')
    parser.add_argument(
        '--wait', type=float, default=25.0,
        help='Seconds to wait for world services before attempting remove',
    )
    args = parser.parse_args()

    pose = f'x={args.x} y={args.y} z={args.z} yaw={args.yaw}'
    print(
        f'[spawn] intended create name={args.name} world={args.world} {pose}',
        flush=True,
    )

    world = GazeboWorld(args.world, service_timeout=5.0)
    if not world.wait_until_available(max_wait=max(1.0, args.wait), poll_interval=1.0):
        print(
            f'[spawn] WARN world "{args.world}" create service not ready after '
            f'{args.wait:.0f}s — skipping pre-remove; create may leave a stale '
            f'"{args.name}" at a prior pose if the model already exists',
            flush=True,
        )
        return 0

    removed = world.remove_model(args.name)
    print(
        f'[spawn] pre-remove name={args.name} ack={removed} '
        f'(False is OK when model was absent)',
        flush=True,
    )
    return 0


if __name__ == '__main__':
    sys.exit(main())
