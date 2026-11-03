"""Versioned trial manifest helpers for controlled Nav2 bake-offs (Prompt §8).

Write one JSON document per trial. Plugin identity must be filled at runtime via
``verify_nav2_plugins`` — YAML intent alone is not evidence.
"""

from __future__ import annotations

import json
import subprocess
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


# Locked bush_trail bake poses (Phase One evidence report).
DEFAULT_START = {'x': -18.0, 'y': 3.0, 'z': 0.50, 'yaw': 0.0}
DEFAULT_GOAL = {'x': 16.0, 'y': -0.1, 'z': 0.0, 'yaw': 0.0}
MIN_DIST_HOME_AT_RETURN_ARM_M = 5.0
MIN_PROGRESS_BEFORE_B_BLOCK_M = 8.0


@dataclass
class TrialManifest:
    manifest_id: str
    schema_version: int = 1
    git_commit: str = ''
    git_dirty_summary: str = ''
    world_name: str = 'bush_trail_world'
    map_yaml: str = 'bush_trail_world.yaml'
    start: Dict[str, float] = field(default_factory=lambda: dict(DEFAULT_START))
    goal: Dict[str, float] = field(default_factory=lambda: dict(DEFAULT_GOAL))
    launch_file: str = '41068_ignition_husky.launch.py'
    launch_args: Dict[str, str] = field(default_factory=dict)
    plugins_expected: Dict[str, str] = field(default_factory=dict)
    plugins_verified: Dict[str, Optional[str]] = field(
        default_factory=lambda: {'planner': None, 'controller': None})
    scenario_id: str = ''
    obstacle: Dict[str, Any] = field(default_factory=dict)
    reset_validation: Dict[str, Any] = field(default_factory=dict)
    test_speed: bool = False
    repetition: Dict[str, int] = field(default_factory=dict)
    timing: Dict[str, Any] = field(default_factory=dict)
    metrics: Dict[str, Any] = field(default_factory=dict)
    artifacts: Dict[str, str] = field(default_factory=dict)
    status: str = 'NOT_VERIFIED'  # PASS|FAIL|BLOCKED|INVALID_GEOMETRY|NOT_VERIFIED
    notes: str = ''

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    def write(self, path: str) -> str:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        with p.open('w', encoding='utf-8') as fh:
            json.dump(self.to_dict(), fh, indent=2, sort_keys=True)
            fh.write('\n')
        return str(p)


def git_snapshot(repo: str) -> Tuple[str, str]:
    def _run(args: List[str]) -> str:
        try:
            out = subprocess.check_output(
                args, cwd=repo, stderr=subprocess.DEVNULL, text=True)
            return out.strip()
        except (subprocess.CalledProcessError, FileNotFoundError, OSError):
            return ''

    commit = _run(['git', 'rev-parse', 'HEAD'])
    dirty = _run(['git', 'status', '--short'])
    return commit, dirty


def verify_nav2_plugins(
    observer,
    *,
    ns: str = 'husky1',
    timeout_s: float = 10.0,
) -> Dict[str, Optional[str]]:
    """Read effective planner/controller plugin class names via GetParameters."""
    from rcl_interfaces.srv import GetParameters

    out: Dict[str, Optional[str]] = {'planner': None, 'controller': None}

    def _get(node_name: str, param: str) -> Optional[str]:
        cli = observer.create_client(GetParameters, f'/{ns}/{node_name}/get_parameters')
        if not cli.wait_for_service(timeout_sec=timeout_s):
            return None
        req = GetParameters.Request()
        req.names = [param]
        fut = cli.call_async(req)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not fut.done():
            observer.spin_for(0.05)
        if not fut.done():
            return None
        resp = fut.result()
        if resp is None or not resp.values:
            return None
        val = resp.values[0]
        if val.type == 4:  # PARAMETER_STRING
            return val.string_value
        return None

    out['planner'] = _get('planner_server', 'GridBased.plugin')
    out['controller'] = _get('controller_server', 'FollowPath.plugin')
    return out


def plugins_match(
    verified: Dict[str, Optional[str]],
    expected: Dict[str, str],
) -> bool:
    for key, want in expected.items():
        got = verified.get(key)
        if got is None or got != want:
            return False
    return True
