#!/usr/bin/env python3
"""Genuine R2 bake: T11 with progress/arm gates + plugin verify + manifest.

Requires sensor_only T11 genuine R2 gates (8 m progress, 5 m arm).
"""

from __future__ import annotations

import argparse
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from nav_test_lib import await_simulation, bringup, log, print_diagnostics
from rs1_nav import GazeboWorld, NavObserver, init_ros
from rs1_nav.benchmark_manifest import (
    TrialManifest,
    git_snapshot,
    plugins_match,
    verify_nav2_plugins,
)

# Import after path setup
import sensor_only_costmap_test as sot

START = (-18.0, 3.0, 0.0)
WORLD = 'bush_trail_world'
PLANNER_PLUGIN = {
    'none': 'nav2_navfn_planner/NavfnPlanner',
    'smac2d': 'nav2_smac_planner/SmacPlanner2D',
}
CONTROLLER_PLUGIN = {
    'none': (
        'nav2_regulated_pure_pursuit_controller::'
        'RegulatedPurePursuitController'),
    'mppi': 'nav2_mppi_controller::MPPIController',
}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--planner', choices=('none', 'smac2d'), default='none')
    parser.add_argument('--controller', choices=('none', 'mppi'), default='none')
    parser.add_argument('--rep', type=int, default=1)
    parser.add_argument('--fair-speed', action='store_true', default=True)
    args = parser.parse_args()

    run_id = (
        f"bake_r2_p-{args.planner}_c-{args.controller}_r{args.rep}_"
        f"{int(time.time())}"
    )
    out_dir = f'/tmp/rs1_{run_id}'
    os.makedirs(out_dir, exist_ok=True)
    commit, dirty = git_snapshot('/home/jordan/git/RS1-Gr25')
    expected = {
        'planner': PLANNER_PLUGIN[args.planner],
        'controller': CONTROLLER_PLUGIN[args.controller],
    }
    manifest = TrialManifest(
        manifest_id=run_id,
        git_commit=commit,
        git_dirty_summary=dirty[:2000],
        launch_args={
            'world': WORLD,
            'planner_bake': args.planner,
            'controller_bake': args.controller,
            'test_speed': 'false',
        },
        plugins_expected=expected,
        scenario_id='R2_return_home',
        test_speed=False,
        repetition={'index': args.rep, 'of': 1},
        artifacts={'stdout': f'{out_dir}/stdout.log',
                   'bringup': f'{out_dir}/bringup.log'},
    )

    # Delegate to sensor_only CLI for scenario body — spawn subprocess for
    # isolation of argv/main; capture exit classification.
    cmd = [
        sys.executable, os.path.join(_HERE, 'sensor_only_costmap_test.py'),
        '--scenario', 't11',
        '--fair-speed',
        '--planner-bake', args.planner,
        '--controller-bake', args.controller,
    ]
    tee_path = f'{out_dir}/stdout.log'
    t0 = time.monotonic()
    import subprocess
    env = os.environ.copy()
    env['PYTHONPATH'] = _HERE + os.pathsep + env.get('PYTHONPATH', '')
    with open(tee_path, 'w', encoding='utf-8') as out:
        out.write(f'CMD {" ".join(cmd)}\n')
        out.flush()
        proc = subprocess.run(
            cmd, stdout=out, stderr=subprocess.STDOUT, env=env, cwd=_HERE)
    # Map exit: 0 PASS, 2 INVALID_GEOMETRY, else FAIL
    if proc.returncode == 0:
        status = 'PASS'
    elif proc.returncode == 2:
        status = 'INVALID_GEOMETRY'
    else:
        status = 'FAIL'
    manifest.status = status
    manifest.timing = {'wall_s': round(time.monotonic() - t0, 1)}
    # Plugin verify would need live node — record expected only for subprocess.
    manifest.plugins_verified = {'planner': None, 'controller': None,
                                 'note': 'verified inside child if bringup logged'}
    manifest.notes = f'sensor_only t11 exit={proc.returncode}'
    manifest.write(f'{out_dir}/manifest.json')
    log(f'R2 {status} manifest={out_dir}/manifest.json')
    return 0 if status == 'PASS' else (2 if status == 'INVALID_GEOMETRY' else 1)


if __name__ == '__main__':
    raise SystemExit(main())
