#!/usr/bin/env python3
"""Controlled R1 (Path A block → B) bake with plugin verify + manifest.

Fairness: test_speed:=false by default. Uses path_bank block scenario.
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
from path_bank_nav_test import PathBankWatcher, run_block
from rs1_nav import GazeboWorld, MissionRunner, NavObserver, init_ros
from rs1_nav.benchmark_manifest import (
    TrialManifest,
    git_snapshot,
    plugins_match,
    verify_nav2_plugins,
)

START = (-18.0, 3.0, 0.0)
WORLD = 'bush_trail_world'
PLANNER_PLUGIN = {
    'none': 'nav2_navfn_planner/NavfnPlanner',
    'smac2d': 'nav2_smac_planner/SmacPlanner2D',
}
# Entry 054: production FollowPath is MPPI. 'none' and 'mppi' both expect MPPI.
# Historical RPP bake used pre-ADOPT overlays (see eng-log Entry 053).
CONTROLLER_PLUGIN = {
    'none': 'nav2_mppi_controller::MPPIController',
    'mppi': 'nav2_mppi_controller::MPPIController',
}
def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--planner', choices=('none', 'smac2d'), default='none')
    parser.add_argument('--controller', choices=('none', 'mppi'), default='none')
    parser.add_argument(
        '--smoother', choices=('none', 'constrained'), default='none')
    parser.add_argument('--rep', type=int, default=1)
    parser.add_argument('--test-speed', action='store_true',
                        help='Allow harness test_speed (default false for bake)')
    args = parser.parse_args()

    run_id = (
        f"bake_r1_p-{args.planner}_c-{args.controller}_s-{args.smoother}_"
        f"r{args.rep}_{int(time.time())}"
    )
    out_dir = f'/tmp/rs1_{run_id}'
    os.makedirs(out_dir, exist_ok=True)
    commit, dirty = git_snapshot('/home/jordan/git/RS1-Gr25')
    # Plugin verify covers planner/controller only (GetParameters).
    expected = {
        'planner': PLANNER_PLUGIN[args.planner],
        'controller': CONTROLLER_PLUGIN[args.controller],
    }
    test_speed = bool(args.test_speed)
    manifest = TrialManifest(
        manifest_id=run_id,
        git_commit=commit,
        git_dirty_summary=dirty[:2000],
        launch_args={
            'world': WORLD,
            'planner_bake': args.planner,
            'controller_bake': args.controller,
            'smoother_bake': args.smoother,
            'test_speed': 'true' if test_speed else 'false',
        },
        plugins_expected=expected,
        scenario_id='R1_path_a_block',
        test_speed=test_speed,
        repetition={'index': args.rep, 'of': 1},
        artifacts={'stdout': f'{out_dir}/stdout.log',
                   'bringup': f'{out_dir}/bringup.log'},
    )

    t0 = time.monotonic()
    status = 'FAIL'
    notes = ''
    with open(f'{out_dir}/stdout.log', 'w', encoding='utf-8') as out_fh:
        def tee(msg: str) -> None:
            line = f'[{time.strftime("%H:%M:%S")}] {msg}'
            print(line)
            out_fh.write(line + '\n')
            out_fh.flush()

        import path_bank_nav_test as pbn
        pbn.log = tee

        tee(f'START {run_id} expected={expected} test_speed={test_speed}')
        sup = bringup(
            world=WORLD,
            nav2=True,
            rviz=False,
            gui=False,
            husky_x=START[0],
            husky_y=START[1],
            husky_yaw=START[2],
            max_runtime=900.0,
            log_path=f'{out_dir}/bringup.log',
            extra={
                'husky_z': '0.50',
                'use_prior_map': 'true',
                'prior_map_file': 'bush_trail_world.yaml',
                'path_bank': 'true',
                'path_bank_switch_resend': 'true',
                'fire_avoidance': 'false',
                'test_speed': 'true' if test_speed else 'false',
                'planner_bake': args.planner,
                'controller_bake': args.controller,
                'smoother_bake': args.smoother,
            },
        )
        with sup:
            ok, _ = await_simulation(sup, require_nav2=True)
            if not ok:
                print_diagnostics(sup)
                notes = 'bringup failed'
                tee(f'FAIL {notes}')
            else:
                init_ros()
                observer = NavObserver()
                mission = MissionRunner(observer, logger=tee)
                bank = PathBankWatcher(observer)
                if not mission.wait_until_ready(timeout=180.0):
                    notes = 'wait_until_ready failed'
                    tee(f'FAIL {notes}')
                else:
                    world = GazeboWorld(WORLD, logger=lambda m: tee(f'  {m}'))
                    if not world.wait_until_available(max_wait=30.0):
                        notes = 'gazebo unavailable'
                        tee(f'FAIL {notes}')
                    else:
                        verified = verify_nav2_plugins(observer)
                        manifest.plugins_verified = verified
                        tee(f'plugins_verified={verified}')
                        if not plugins_match(verified, expected):
                            notes = f'plugin mismatch {expected}'
                            tee(f'FAIL {notes}')
                        else:
                            tee('PASS plugin identity')
                            ok = run_block(observer, mission, bank, world)
                            status = 'PASS' if ok else 'FAIL'
                            if not ok:
                                notes = 'run_block failed'
                            tee(f'R1 {status}')

    manifest.status = status
    manifest.notes = notes
    manifest.timing = {'wall_s': round(time.monotonic() - t0, 1)}
    manifest.metrics = {'r1_success': status == 'PASS'}
    manifest.write(f'{out_dir}/manifest.json')
    log(f'manifest -> {out_dir}/manifest.json status={status}')
    return 0 if status == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
