#!/usr/bin/env python3
"""Unit tests for GoalVisualSpec SDF (Issue C — pole above lidar, no collision).

Synthetic only — no Gazebo. Run:

    python3 test/goal_visual_unit_test.py
"""

from __future__ import annotations

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rs1_nav.gazebo_world import HUSKY_LIDAR_HEIGHT, GoalVisualSpec


def check(name, condition, detail=''):
    print(f'  {"PASS" if condition else "FAIL"}  {name}'
          + (f'  [{detail}]' if detail else ''))
    return condition


def _pole_z_span(sdf: str) -> tuple[float, float] | None:
    """Return (z_min, z_max) for goal_pole cylinder, or None if unparseable."""
    m = re.search(
        r'<visual name="goal_pole"><pose>([^<]+)</pose>'
        r'<geometry><cylinder><radius>[^<]+</radius><length>([^<]+)</length>',
        sdf,
    )
    if not m:
        return None
    pose_parts = m.group(1).split()
    if len(pose_parts) < 3:
        return None
    try:
        z_center = float(pose_parts[2])
        length = float(m.group(2))
    except ValueError:
        return None
    half = length * 0.5
    return (z_center - half, z_center + half)


def main() -> int:
    results = []
    sdf = GoalVisualSpec().to_sdf()

    results.append(check(
        'SDF has no <collision> elements',
        '<collision' not in sdf.lower(),
        f'len={len(sdf)}',
    ))
    results.append(check(
        'keeps goal_pole / goal_arrow / goal_arrow_head visuals',
        all(name in sdf for name in (
            'goal_pole', 'goal_arrow', 'goal_arrow_head',
        )),
    ))

    span = _pole_z_span(sdf)
    results.append(check(
        'goal_pole pose/length parseable',
        span is not None,
        f'span={span}',
    ))
    if span is not None:
        z_min, z_max = span
        results.append(check(
            f'pole z_min > HUSKY_LIDAR_HEIGHT ({HUSKY_LIDAR_HEIGHT})',
            z_min > HUSKY_LIDAR_HEIGHT,
            f'z_min={z_min:.3f}',
        ))
        results.append(check(
            'pole z_min > 1.0 m (plan gate)',
            z_min > 1.0,
            f'z_min={z_min:.3f}',
        ))
        results.append(check(
            'pole stays in preferred canopy band (z_min≥1.5, z_max≤6.0)',
            z_min >= 1.5 - 1e-9 and z_max <= 6.0 + 1e-9,
            f'z∈[{z_min:.3f},{z_max:.3f}]',
        ))

    ok = all(results)
    print(f'GOAL VISUAL UNIT TEST {"PASSED" if ok else "FAILED"}')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
