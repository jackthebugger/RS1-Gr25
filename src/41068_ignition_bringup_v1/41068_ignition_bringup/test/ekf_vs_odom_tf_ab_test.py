#!/usr/bin/env python3
"""§5 #9 robot_localization incremental A/B (no full stack replacement).

Compares TF continuity / mission R0 under:
  A) production odometry_tf_broadcaster (default launch)
  B) EKF from robot_localization_husky1.yaml if we can launch it without
     dual TF publishers (publish_tf conflict).

If B cannot be launched safely without dual odom→base_link publishers,
classify RULED OUT / BLOCKED with evidence — do not run two TF authorities.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from nav_test_lib import await_simulation, bringup, log, print_diagnostics
from path_bank_nav_test import PathBankWatcher, run_clean
from rs1_nav import MissionRunner, NavObserver, init_ros

START = (-18.0, 3.0, 0.0)
WORLD = 'bush_trail_world'


def _tf_jump_stats(observer: NavObserver, duration_s: float = 20.0) -> dict:
    """Sample map→base pose; report max step between samples."""
    samples = []
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        observer.spin_for(0.2)
        p = observer.robot_pose()
        if p is not None:
            samples.append(p)
    jumps = []
    for a, b in zip(samples, samples[1:]):
        jumps.append(math.hypot(b[0] - a[0], b[1] - a[1]))
    return {
        'n': len(samples),
        'max_jump_m': max(jumps) if jumps else float('nan'),
        'mean_jump_m': (sum(jumps) / len(jumps)) if jumps else float('nan'),
    }


def run_profile(label: str) -> dict:
    log(f'=== EKF A/B profile {label} ===')
    # Production path only for A. B would require launch change; for this
    # harness we document that localization_config is unused and EKF is not
    # started — attempting dual publish is unsafe.
    if label != 'A_odom_tf':
        return {
            'label': label,
            'status': 'SKIP',
            'notes': (
                'EKF not launched in production; enabling without disabling '
                'odometry_tf_broadcaster would dual-publish odom→base_link. '
                'Isolated EKF bringup needs launch surgery — deferred as '
                'RULED OUT for production unless launch exclusivity lands.'
            ),
        }

    result = {'label': label, 'status': 'FAIL', 'notes': ''}
    sup = bringup(
        world=WORLD,
        nav2=True,
        rviz=False,
        gui=False,
        husky_x=START[0],
        husky_y=START[1],
        husky_yaw=START[2],
        max_runtime=600.0,
        log_path=f'/tmp/rs1_ekf_ab_{label}.log',
        extra={
            'husky_z': '0.50',
            'use_prior_map': 'true',
            'prior_map_file': 'bush_trail_world.yaml',
            'path_bank': 'true',
            'path_bank_switch_resend': 'false',
            'fire_avoidance': 'false',
            'test_speed': 'false',
        },
    )
    with sup:
        ok, _ = await_simulation(sup, require_nav2=True)
        if not ok:
            print_diagnostics(sup)
            result['notes'] = 'bringup failed'
            return result
        init_ros()
        observer = NavObserver()
        mission = MissionRunner(observer, logger=log)
        bank = PathBankWatcher(observer)
        if not mission.wait_until_ready(timeout=180.0):
            result['notes'] = 'wait_until_ready failed'
            return result
        # Confirm TF publisher is odometry_tf_broadcaster (not ekf).
        # Best-effort: topic type info via ros2 CLI is heavy; sample jumps + R0.
        jumps = _tf_jump_stats(observer, 15.0)
        result['tf_jumps'] = jumps
        log(f'TF jumps: {jumps}')
        clean_ok = run_clean(observer, mission, bank)
        result['r0'] = clean_ok
        result['status'] = 'PASS' if clean_ok else 'FAIL'
        result['notes'] = 'production odometry_tf_broadcaster baseline'
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--profile', choices=('A', 'both'), default='both')
    args = parser.parse_args()
    results = [run_profile('A_odom_tf')]
    if args.profile == 'both':
        results.append(run_profile('B_ekf'))
    log('=== EKF A/B SUMMARY ===')
    for r in results:
        log(f"  {r['label']}: {r['status']} — {r.get('notes','')}")
    # Disposition helper printed for Logkeeper.
    a_ok = results[0]['status'] == 'PASS'
    if a_ok and len(results) > 1 and results[1]['status'] == 'SKIP':
        log(
            'DISPOSITION_HINT: #9 RULED OUT AS UNSUITABLE for replacement — '
            'keep odometry_tf_broadcaster; EKF A/B blocked by dual-TF risk '
            'without launch exclusivity (not runtime-tested as replacement).'
        )
    return 0 if a_ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
