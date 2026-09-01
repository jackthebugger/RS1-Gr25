#!/usr/bin/env python3
"""Launch and tear down the Husky simulation with hard timeouts.

Used by the autonomous demo and the test suite. Every wait is bounded. The
process group is always signalled on exit so Gazebo, Nav2 and RViz cannot leak
into the next run.
"""

from __future__ import annotations

import os
import signal
import subprocess
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

PACKAGE = '41068_ignition_bringup'

ORPHAN_PATTERNS = (
    'ign gazebo',
    'ign-gazebo',
    'gazebo/worlds',
    'parameter_bridge',
    'rviz2',
    'async_slam_toolbox_node',
    'controller_server',
    'planner_server',
    'bt_navigator',
    'behavior_server',
    'smoother_server',
    'waypoint_follower',
    'velocity_smoother',
    'lifecycle_manager',
    'collision_monitor',
    'ekf_node',
    'robot_state_publisher',
    'basic_autonomy_demo',
    'obstacle_injector',
    'nav_observer',
    'nav_mission',
)

_PROTECTED_CMDLINE_MARKERS = (
    'cursor-server',
    'shellIntegration',
    'ros2cli.daemon',
)


def detect_workspace() -> str:
    """Locate the colcon workspace that contains this package."""
    prefix = os.environ.get('COLCON_PREFIX_PATH', '')
    for entry in prefix.split(os.pathsep):
        if not entry:
            continue
        trimmed = entry.rstrip('/')
        if os.path.basename(trimmed) == 'install':
            return os.path.dirname(trimmed)
        parent = os.path.dirname(trimmed)
        if os.path.isdir(os.path.join(parent, 'install')):
            return parent
    here = os.path.abspath(__file__)
    candidate = os.path.abspath(os.path.join(os.path.dirname(here), '..', '..', '..', '..'))
    if os.path.isdir(os.path.join(candidate, 'install')):
        return candidate
    return os.path.expanduser('~/G25_RS1/RS1-Gr25')


WORKSPACE = detect_workspace()


def log(message: str) -> None:
    print(f'[{time.strftime("%H:%M:%S")}] {message}', flush=True)


def ros_env() -> Dict[str, str]:
    env = dict(os.environ)
    env['ROS_LOCALHOST_ONLY'] = '1'
    env.setdefault('RCUTILS_LOGGING_BUFFERED_STREAM', '0')
    return env


def _bash(command: str, timeout: float) -> subprocess.CompletedProcess:
    sourced = (
        'set -o pipefail; '
        'source /opt/ros/humble/setup.bash >/dev/null 2>&1; '
        f'source {WORKSPACE}/install/setup.bash >/dev/null 2>&1; '
        f'{command}'
    )
    return subprocess.run(
        ['bash', '-lc', sourced],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=timeout,
        env=ros_env(),
        check=False,
    )


def ros2(args: str, timeout: float = 15.0) -> Tuple[int, str]:
    try:
        done = _bash(f'ros2 {args}', timeout=timeout)
        return done.returncode, done.stdout or ''
    except subprocess.TimeoutExpired as exc:
        partial = exc.stdout or ''
        if isinstance(partial, bytes):
            partial = partial.decode('utf-8', 'replace')
        return 124, partial


def _own_pid_lineage() -> set:
    lineage = set()
    pid = os.getpid()
    for _ in range(40):
        if pid <= 1:
            break
        lineage.add(pid)
        try:
            with open(f'/proc/{pid}/stat', 'r') as handle:
                pid = int(handle.read().split(') ', 1)[1].split()[1])
        except (OSError, IndexError, ValueError):
            break
    return lineage


def find_orphans() -> List[Tuple[int, str]]:
    protected = _own_pid_lineage()
    found = []
    try:
        entries = os.listdir('/proc')
    except OSError:
        return found
    for entry in entries:
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid in protected:
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as handle:
                cmdline = handle.read().replace(b'\0', b' ').decode('utf-8', 'replace').strip()
        except OSError:
            continue
        if not cmdline:
            continue
        if any(marker in cmdline for marker in _PROTECTED_CMDLINE_MARKERS):
            continue
        if any(pattern in cmdline for pattern in ORPHAN_PATTERNS):
            found.append((pid, cmdline))
    return found


def sweep_orphans(reason: str = '', settle: float = 3.0) -> None:
    for signal_number in (signal.SIGTERM, signal.SIGKILL):
        orphans = find_orphans()
        if not orphans:
            break
        names = sorted({cmd.split()[0].rsplit('/', 1)[-1] for _pid, cmd in orphans})
        log(f'Sweeping {len(orphans)} orphan(s){f" ({reason})" if reason else ""} '
            f'with {signal_number.name}: {", ".join(names)}')
        for pid, _cmd in orphans:
            try:
                os.kill(pid, signal_number)
            except (ProcessLookupError, PermissionError):
                pass
        time.sleep(2.0)

    remaining = find_orphans()
    if remaining:
        log(f'WARNING: {len(remaining)} orphan(s) survived the sweep: '
            f'{[pid for pid, _ in remaining]}')

    subprocess.run(
        ['bash', '-lc', 'source /opt/ros/humble/setup.bash >/dev/null 2>&1; ros2 daemon stop'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False, timeout=20,
    )
    time.sleep(settle)


@dataclass
class SimSupervisor:
    """Owns one `ros2 launch` invocation and guarantees it gets cleaned up."""

    launch_file: str
    launch_args: Dict[str, str] = field(default_factory=dict)
    log_path: str = '/tmp/nav_demo_sim.log'
    max_runtime: float = 600.0

    _proc: Optional[subprocess.Popen] = field(default=None, init=False, repr=False)
    _log: Optional[object] = field(default=None, init=False, repr=False)
    _started: float = field(default=0.0, init=False, repr=False)

    def start(self) -> None:
        sweep_orphans('pre-launch')
        args = ' '.join(f'{k}:={v}' for k, v in self.launch_args.items())
        command = f'ros2 launch {PACKAGE} {self.launch_file} {args}'
        log(f'LAUNCH  {command}')
        log(f'        log -> {self.log_path}')
        self._log = open(self.log_path, 'w')
        sourced = (
            'source /opt/ros/humble/setup.bash; '
            f'source {WORKSPACE}/install/setup.bash; '
            f'exec {command}'
        )
        self._proc = subprocess.Popen(
            ['bash', '-lc', sourced],
            stdout=self._log,
            stderr=subprocess.STDOUT,
            env=ros_env(),
            start_new_session=True,
        )
        self._started = time.monotonic()

    def is_alive(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def runtime(self) -> float:
        return time.monotonic() - self._started if self._started else 0.0

    def exceeded_max_runtime(self) -> bool:
        return self.runtime() > self.max_runtime

    def stop(self) -> None:
        if self._proc is not None and self._proc.poll() is None:
            pgid = os.getpgid(self._proc.pid)
            for sig, grace in ((signal.SIGINT, 8.0), (signal.SIGTERM, 5.0), (signal.SIGKILL, 2.0)):
                try:
                    os.killpg(pgid, sig)
                except ProcessLookupError:
                    break
                deadline = time.monotonic() + grace
                while time.monotonic() < deadline:
                    if self._proc.poll() is not None:
                        break
                    time.sleep(0.25)
                if self._proc.poll() is not None:
                    break
            log(f'Stopped launch (pgid {pgid}) after {self.runtime():.1f}s')
        if self._log is not None:
            self._log.close()
            self._log = None
        self._proc = None
        sweep_orphans('post-test')

    def __enter__(self) -> 'SimSupervisor':
        self.start()
        return self

    def __exit__(self, *_exc) -> None:
        self.stop()

    def log_tail(self, lines: int = 40, grep: Optional[str] = None) -> str:
        try:
            with open(self.log_path, 'r', errors='replace') as handle:
                content = handle.readlines()
        except OSError:
            return '(no launch log)'
        if grep:
            content = [line for line in content if grep.lower() in line.lower()]
        return ''.join(content[-lines:])


def bringup(
    *,
    world: str = 'simple_trees',
    nav2: bool = True,
    rviz: bool = False,
    gui: bool = False,
    husky_x: float = 0.0,
    husky_y: float = 0.0,
    husky_yaw: float = 0.0,
    max_runtime: float = 600.0,
    log_path: str = '/tmp/nav_demo_sim.log',
    extra: Optional[Dict[str, str]] = None,
) -> SimSupervisor:
    args = {
        'world': world,
        'nav2': 'true' if nav2 else 'false',
        'rviz': 'true' if rviz else 'false',
        'gui': 'true' if gui else 'false',
        'husky_x': f'{husky_x}',
        'husky_y': f'{husky_y}',
        'husky_yaw': f'{husky_yaw}',
    }
    if extra:
        args.update(extra)
    return SimSupervisor(
        launch_file='41068_ignition_husky.launch.py',
        launch_args=args,
        log_path=log_path,
        max_runtime=max_runtime,
    )
