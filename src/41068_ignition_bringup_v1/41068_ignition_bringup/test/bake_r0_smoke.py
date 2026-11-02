#!/usr/bin/env python3
"""Controlled R0 smoke for NavFn vs Smac2D (or RPP vs MPPI) with plugin verify.

Usage:
  python3 test/bake_r0_smoke.py --planner none --controller none
  python3 test/bake_r0_smoke.py --planner smac2d --controller none
  python3 test/bake_r0_smoke.py --planner none --controller mppi

Fairness: test_speed:=false. Writes manifest under /tmp/rs1_bake_*.
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
from path_bank_nav_test import PathBankWatcher, run_clean
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
    args = parser.parse_args()

    run_id = (
        f"bake_p-{args.planner}_c-{args.controller}_r{args.rep}_"
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
            'smoother_bake': 'none',
            'test_speed': 'false',
        },
        plugins_expected=expected,
        scenario_id='R0_clean',
        test_speed=False,
        repetition={'index': args.rep, 'of': 1},
        artifacts={
            'stdout': f'{out_dir}/stdout.log',
            'bringup': f'{out_dir}/bringup.log',
            'manifest': f'{out_dir}/manifest.json',
        },
    )

    t0 = time.monotonic()
    status = 'FAIL'
    notes = ''

    # Mirror log() into file via simple tee of path_bank log calls — use print.
    stdout_path = f'{out_dir}/stdout.log'
    with open(stdout_path, 'w', encoding='utf-8') as out_fh:
        def tee(msg: str) -> None:
            line = f'[{time.strftime("%H:%M:%S")}] {msg}'
            print(line)
            out_fh.write(line + '\n')
            out_fh.flush()

        # Temporarily redirect path_bank log by monkeypatching module logger
        import path_bank_nav_test as pbn
        pbn.log = tee

        tee(f'START {run_id} expected={expected}')
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
                'path_bank_switch_resend': 'false',
                'fire_avoidance': 'false',
                'test_speed': 'false',
                'planner_bake': args.planner,
                'controller_bake': args.controller,
                'smoother_bake': 'none',
            },
        )
        with sup:
            ok, _ = await_simulation(sup, require_nav2=True)
            if not ok:
                print_diagnostics(sup)
                notes = 'bringup/await_simulation failed'
                tee(f'FAIL {notes}')
            else:
                init_ros()
                observer = NavObserver()
                mission = MissionRunner(observer, logger=tee)
                bank = PathBankWatcher(observer)
                if not mission.wait_until_ready(timeout=180.0):
                    notes = 'mission.wait_until_ready failed'
                    tee(f'FAIL {notes}')
                else:
                    world = GazeboWorld(WORLD, logger=lambda m: tee(f'  {m}'))
                    if not world.wait_until_available(max_wait=30.0):
                        notes = 'gazebo services unavailable'
                        tee(f'FAIL {notes}')
                    else:
                        verified = verify_nav2_plugins(observer)
                        manifest.plugins_verified = verified
                        tee(f'plugins_verified={verified}')
                        if not plugins_match(verified, expected):
                            notes = f'plugin mismatch expected={expected}'
                            tee(f'FAIL {notes}')
                        else:
                            tee('PASS plugin identity')
                            ok = run_clean(observer, mission, bank)
                            status = 'PASS' if ok else 'FAIL'
                            if not ok:
                                notes = 'run_clean failed'
                            tee(f'R0 {status}')

    manifest.status = status
    manifest.notes = notes
    manifest.timing = {'wall_s': round(time.monotonic() - t0, 1)}
    manifest.metrics = {'r0_success': status == 'PASS'}
    manifest.write(f'{out_dir}/manifest.json')
    log(f'manifest -> {out_dir}/manifest.json status={status}')
    return 0 if status == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
