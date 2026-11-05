"""Prompt §7 stall-class timeline artefact (Wave A4).

Records time-aligned rows that distinguish:
VALID_DISCARDED, BLOCKED_TREATED_VALID, B_NOT_SELECTED, HOME_NEVER_REQUESTED,
PLAN_NO_FOLLOW, GATE_STOP_OK, SM_STUCK, OK_PROGRESS, OTHER.
"""

from __future__ import annotations

import csv
import json
import math
import time
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional, Tuple


STALL_CLASSES = (
    'VALID_DISCARDED',
    'BLOCKED_TREATED_VALID',
    'B_NOT_SELECTED',
    'HOME_NEVER_REQUESTED',
    'PLAN_NO_FOLLOW',
    'GATE_STOP_OK',
    'SM_STUCK',
    'OK_PROGRESS',
    'OTHER',
)


@dataclass
class TimelineRow:
    wall_mono_s: float
    sim_time_s: float
    owner_phase: str = ''
    command_authority: str = ''
    stop_reason: str = ''
    reject_code: str = ''
    inhibits: str = ''
    bank_state: str = ''
    active_candidate_id: str = ''
    candidate_statuses: str = ''
    path_len_m: float = float('nan')
    switches: int = -1
    active_route_valid: str = ''
    validity_fail_reason: str = ''
    fresh_planner_request: str = 'none'
    keepout_hit_A: str = ''
    keepout_hit_B: str = ''
    keepout_hit_home: str = ''
    follow_path_status: str = ''
    compute_path_status: str = ''
    cancel_or_preempt: str = ''
    cmd_v: float = float('nan')
    cmd_w: float = float('nan')
    odom_v: float = float('nan')
    odom_w: float = float('nan')
    gate_effective_auth: str = ''
    gate_zero_reason: str = ''
    pose_x: float = float('nan')
    pose_y: float = float('nan')
    dist_home_m: float = float('nan')
    dist_goal_m: float = float('nan')
    stall_class: str = 'OTHER'
    note: str = ''


@dataclass
class StallTimeline:
    rows: List[TimelineRow] = field(default_factory=list)
    events: List[Dict[str, Any]] = field(default_factory=list)

    def mark_event(self, label: str, **fields: Any) -> None:
        self.events.append({
            'wall_mono_s': time.monotonic(),
            'label': label,
            **fields,
        })

    def append(self, row: TimelineRow) -> None:
        if row.stall_class not in STALL_CLASSES:
            row.stall_class = 'OTHER'
        self.rows.append(row)

    def write_csv(self, path: str) -> str:
        if not self.rows:
            # still write header for empty artefact
            fieldnames = list(asdict(TimelineRow(
                wall_mono_s=0.0, sim_time_s=0.0)).keys())
        else:
            fieldnames = list(asdict(self.rows[0]).keys())
        with open(path, 'w', newline='', encoding='utf-8') as fh:
            wr = csv.DictWriter(fh, fieldnames=fieldnames)
            wr.writeheader()
            for row in self.rows:
                wr.writerow(asdict(row))
        return path

    def write_events_json(self, path: str) -> str:
        with open(path, 'w', encoding='utf-8') as fh:
            json.dump(self.events, fh, indent=2)
            fh.write('\n')
        return path


def classify_sample(
    *,
    bank_state: str,
    command_authority: str,
    active_route_valid: Optional[bool],
    odom_v: float,
    cmd_v: float,
    dist_goal_m: float,
    saw_switch: bool,
    home_requested: bool,
    plan_existed: bool,
    following: bool,
) -> str:
    """Heuristic stall-class tag for a snapshot (not a full diagnosis)."""
    auth = (command_authority or '').upper()
    st = (bank_state or '').upper()
    near_goal = (not math.isnan(dist_goal_m)) and dist_goal_m < 0.5
    stopped = (not math.isnan(odom_v)) and abs(odom_v) < 0.05
    zero_cmd = (not math.isnan(cmd_v)) and abs(cmd_v) < 1e-3

    if auth in ('', 'NONE') and zero_cmd and stopped and not near_goal:
        if st in ('STOPPING', 'STOPPED', 'HOLDING', 'COSTMAP_REBUILD', 'NO_SAFE_ROUTE'):
            return 'GATE_STOP_OK'
    if st == 'PATH_BLOCKED' and not saw_switch:
        return 'B_NOT_SELECTED'
    if st in ('NO_PATH_AVAILABLE',) and not home_requested:
        return 'HOME_NEVER_REQUESTED'
    if plan_existed and not following and stopped and not near_goal:
        return 'PLAN_NO_FOLLOW'
    if active_route_valid is False and st == 'FOLLOWING_PATH':
        return 'BLOCKED_TREATED_VALID'
    if active_route_valid is True and st in ('PATH_BLOCKED', 'SWITCHING_PATH'):
        return 'VALID_DISCARDED'
    if st == 'FOLLOWING_PATH' and stopped and not near_goal and zero_cmd:
        return 'SM_STUCK'
    if not stopped or near_goal:
        return 'OK_PROGRESS'
    return 'OTHER'


def dist_xy(
    pose: Optional[Tuple[float, float, float]],
    target: Tuple[float, float],
) -> float:
    if pose is None:
        return float('nan')
    return math.hypot(pose[0] - target[0], pose[1] - target[1])
