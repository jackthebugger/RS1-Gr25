#!/usr/bin/env python3
"""Bounded-timeout supervision helpers for simulation-in-the-loop navigation tests.

Every wait in this module has a maximum duration, a polling interval, an explicit
success condition and diagnostic output on failure. Nothing here can block
forever. Simulation process ownership lives in `rs1_nav.sim` so the demo and
the tests share the same launcher and the same cleanup.

Run the tests through `run_nav_tests.py`; this file is a library.
"""

import math
import os
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence, Tuple

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from rs1_nav.sim import (  # noqa: F401  re-exported for existing tests
    PACKAGE,
    SimSupervisor,
    WORKSPACE,
    bringup,
    find_orphans,
    log,
    ros2,
    ros_env,
    sweep_orphans,
)


# ---------------------------------------------------------------------------
# Bounded waiting
# ---------------------------------------------------------------------------

@dataclass
class WaitResult:
    ok: bool
    what: str
    elapsed: float
    detail: str = ''

    def __bool__(self) -> bool:
        return self.ok


def wait_until(
    what: str,
    check: Callable[[], Tuple[bool, str]],
    max_wait: float,
    poll_interval: float = 2.0,
    supervisor: Optional['SimSupervisor'] = None,
) -> WaitResult:
    """Poll `check` until it succeeds or `max_wait` seconds elapse.

    `check` returns (success, detail). If the supervised launch dies while we are
    waiting, give up immediately instead of burning the whole timeout.
    """
    started = time.monotonic()
    last_detail = 'no attempt completed'
    while True:
        if supervisor is not None and not supervisor.is_alive():
            elapsed = time.monotonic() - started
            log(f'  FAIL  {what} -- launch process exited after {elapsed:.1f}s')
            return WaitResult(False, what, elapsed, 'launch process exited')

        try:
            ok, last_detail = check()
        except Exception as exc:  # a probe crashing must not abort the test run
            ok, last_detail = False, f'probe raised {type(exc).__name__}: {exc}'

        elapsed = time.monotonic() - started
        if ok:
            log(f'  OK    {what} after {elapsed:.1f}s {last_detail}')
            return WaitResult(True, what, elapsed, last_detail)
        if elapsed >= max_wait:
            log(f'  FAIL  {what} -- timeout after {elapsed:.1f}s ({last_detail})')
            return WaitResult(False, what, elapsed, last_detail)
        time.sleep(poll_interval)


def probe_topic_exists(topic: str) -> Callable[[], Tuple[bool, str]]:
    def check() -> Tuple[bool, str]:
        code, out = ros2('topic list', timeout=12.0)
        if code == 124:
            return False, 'ros2 topic list timed out'
        topics = out.split()
        return (topic in topics), f'({len(topics)} topics visible)'
    return check


def probe_topic_publishing(topic: str, min_hz: float = 0.5, window: float = 4.0):
    """True once `topic` delivers messages. Uses `ros2 topic hz` with a hard cap."""
    def check() -> Tuple[bool, str]:
        code, out = ros2(
            f'topic hz {topic} --window 5', timeout=window + 8.0,
        )
        rates = []
        for line in out.splitlines():
            line = line.strip()
            if line.startswith('average rate:'):
                try:
                    rates.append(float(line.split(':', 1)[1]))
                except ValueError:
                    pass
        if not rates:
            return False, f'no rate reported (exit {code})'
        best = max(rates)
        return (best >= min_hz), f'{best:.2f} Hz'
    # `ros2 topic hz` never exits on its own, so the timeout above is the
    # mechanism that stops it. Wrap so the timeout is treated as normal.
    def bounded() -> Tuple[bool, str]:
        try:
            return check()
        except subprocess.TimeoutExpired:
            return False, 'hz probe timed out'
    return bounded


def probe_node_exists(fragment: str):
    def check() -> Tuple[bool, str]:
        code, out = ros2('node list', timeout=12.0)
        if code == 124:
            return False, 'ros2 node list timed out'
        nodes = out.split()
        hits = [n for n in nodes if fragment in n]
        return bool(hits), f'({len(nodes)} nodes; matched {hits[:3]})'
    return check


def probe_action_exists(action: str):
    def check() -> Tuple[bool, str]:
        code, out = ros2('action list', timeout=15.0)
        if code == 124:
            return False, 'ros2 action list timed out'
        actions = out.split()
        return (action in actions), f'({len(actions)} actions visible)'
    return check


def probe_tf(parent: str, child: str, timeout: float = 8.0):
    """True once tf2 can resolve parent->child."""
    def check() -> Tuple[bool, str]:
        code, out = ros2(
            f'run tf2_ros tf2_echo {parent} {child} --ros-args -p use_sim_time:=true '
            f'-r /tf:=/husky1/tf -r /tf_static:=/husky1/tf_static',
            timeout=timeout,
        )
        ok = 'Translation' in out
        if ok:
            return True, ''
        tail = [l for l in out.splitlines() if l.strip()][-1:] or ['no output']
        return False, tail[0][:120]
    return check


# ---------------------------------------------------------------------------
# Standard startup gate
# ---------------------------------------------------------------------------

@dataclass
class Stage:
    name: str
    result: WaitResult


def await_simulation(sup: SimSupervisor, *, require_nav2: bool = True) -> Tuple[bool, List[Stage]]:
    """Bounded startup verification: clock -> sensors -> TF -> map -> Nav2.

    Returns (all_ok, stages). Stops at the first failed stage so a broken run
    fails in seconds rather than minutes.
    """
    checks: List[Tuple[str, Callable[[], Tuple[bool, str]], float, float]] = [
        ('simulation clock', probe_topic_publishing('/clock', min_hz=5.0), 40.0, 2.0),
        ('lidar scan', probe_topic_publishing('/husky1/scan', min_hz=0.8), 60.0, 3.0),
        ('filtered odometry', probe_topic_publishing('/husky1/odom', min_hz=5.0), 40.0, 2.0),
        ('TF husky1_odom->husky1_base_link', probe_tf('husky1_odom', 'husky1_base_link'), 40.0, 3.0),
    ]
    if require_nav2:
        checks += [
            ('TF husky1_map->husky1_odom (SLAM)', probe_tf('husky1_map', 'husky1_odom'), 90.0, 3.0),
            ('SLAM map topic', probe_topic_exists('/husky1/map'), 90.0, 3.0),
            ('Nav2 navigate_to_pose action', probe_action_exists('/husky1/navigate_to_pose'), 180.0, 4.0),
        ]

    stages: List[Stage] = []
    for name, check, max_wait, poll in checks:
        result = wait_until(name, check, max_wait=max_wait, poll_interval=poll, supervisor=sup)
        stages.append(Stage(name, result))
        if not result.ok:
            return False, stages
    return True, stages


def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


def print_diagnostics(sup: SimSupervisor, *, topics: Sequence[str] = ()) -> None:
    """Collect only the diagnostics that help explain a failure."""
    log('--- DIAGNOSTICS ---')
    for label, command in (
        ('nodes', 'node list'),
        ('actions', 'action list'),
    ):
        _code, out = ros2(command, timeout=15.0)
        entries = out.split()
        log(f'{label} ({len(entries)}): {" ".join(entries[:25])}')
    for topic in topics:
        code, out = ros2(f'topic info {topic}', timeout=10.0)
        log(f'{topic}: {" ".join(out.split()) if code == 0 else "unavailable"}')
    errors = sup.log_tail(lines=25, grep='error')
    if errors.strip():
        log(f'launch log errors:\n{errors}')
    log('--- END DIAGNOSTICS ---')


def main() -> int:
    print(__doc__)
    print('This module is a library. Run run_nav_tests.py instead.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
