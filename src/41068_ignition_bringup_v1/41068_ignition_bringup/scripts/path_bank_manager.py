#!/usr/bin/env python3
"""Path-bank manager — mission-scoped alternative routes for Husky Nav2.

Authority model
---------------
* ``mission_nav_authority=bank_follow_path`` (Phase 3 default): this node owns
  frozen RouteInstance geometry and is the sole FollowPath client while
  ``command_authority=NAV``. No silent NavFn rewrite of the active route.
* ``mission_nav_authority=legacy_navigate_to_pose``: Nav2 (NavFn + IsPathValid
  BT + RPP) owns follow/replan timing; this node owns the candidate set and
  preempts NavigateToPose on switch (Phase 2 / rollback).

It does NOT replan every second. Validity is checked at ``check_hz`` (default
2 Hz). A switch only happens when the active bank path is footprint-invalid
(or centreline-invalid in legacy mode) on the live global costmap.

On switch (bank_follow_path):
  1. Mark current candidate BLOCKED (ROUTE_MARKED_BLOCKED)
  2. Cancel FollowPath (generation-fenced; ignore superseded results)
  3. Optional local costmap clear; oriented-footprint validate next
  4. Submit FollowPath only if generation matches and no latch/inhibit
  If no candidate remains valid → NO_PATH_AVAILABLE; Entry 024 / Phase Two
  Wave 1a: fresh ``ComputePathToPose`` home (RETURNING_TO_START →
  RETURNED_TO_START or NO_SAFE_ROUTE). Reverse-splice home is forbidden.

Entry 021 — evidence unblock:
  BLOCKED→AVAILABLE only when ``obstacle_memory/mask`` no longer occupies the
  corridor AND ``costmap_raw`` reports the path valid. Never use Gazebo GT or
  "a new plan exists". Unlock does **not** auto-preempt back to that path.
  Entry 024: once RETURNING_TO_START / RETURNED_TO_START / NO_SAFE_ROUTE,
  unlock does not resume the forward mission.

Static vs live
--------------
Planning bank at mission start prefers ``prior_map`` (static baseline).
Validity monitoring always uses the live ``global_costmap/costmap_raw``.
"""

from __future__ import annotations

import math
import os
import struct
import time
from collections import deque
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import replace
from enum import Enum
from typing import List, Optional, Sequence, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from builtin_interfaces.msg import Duration
from nav2_msgs.action import (
    ComputePathToPose,
    DriveOnHeading,
    FollowPath,
    NavigateToPose,
    Spin,
)
from nav2_msgs.msg import Costmap
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

from rs1_interfaces.msg import MissionOwnerStatus
from rs1_interfaces.srv import (
    DisableManualNav,
    DisableTeleop,
    EnableManualNav,
    EnableTeleop,
    StartMission,
    StopMission,
)
from rs1_nav.mission_owner import (
    CommandAuthority,
    MissionOwnerState,
    MissionPhase,
    SafetyInhibit,
    home_stuck_action,
    mission_stuck_action,
    path_blocked_none_timed_out,
    seed_timed_out,
)
from rs1_nav.geometry import path_length
from rs1_nav.path_bank import (
    LETHAL,
    CandidateState,
    PathBank,
    PathBankConfig,
    PathCandidate,
    build_path_bank,
    first_occupancy_hit_on_path,
    length_primary_seed_config,
    nav2_costmap_to_grid,
    occupancy_mask_intersects_path,
    occupancy_msg_to_grid,
    path_is_valid,
    nudge_polyline_away_from_lethal,
    point_clearance,
)
from rs1_nav.route_instance import (
    FOOTPRINT_HALF_W,
    FOOTPRINT_PAD,
    RouteBank,
    RouteInstance,
    along_track_yaw,
    densify_polyline,
    make_route_instance,
    mask_under_oriented_footprint,
    oriented_footprint_path_valid,
)

# Issue A: operator clearance floor = footprint half-width + pad.
OPERATOR_CLEARANCE_MIN_M = FOOTPRINT_HALF_W + FOOTPRINT_PAD  # 0.48 m
OPERATOR_CLEARANCE_ABS_MAX_M = 1.50  # hard refuse above this (corridor-aware)

# Issue B: bank-owned Spin reorient (sole primary) before mission FollowPath.
THETA_REORIENT_RAD = 0.60  # large misalignment marker
THETA_EXIT_RAD = 0.45  # hard FollowPath gate — never FP while |yaw| ≥ this
# Large far-gap Spins (~π) need >8 s at min_rotational_vel=0.4.
SPIN_TIMEOUT_S = 12.0
FALLBACK_TIMEOUT_S = 12.0
VX_MIN_RESTORE_DEADLINE_S = 0.5
FOLLOW_PATH_CANCEL_TIMEOUT_S = 1.0
NAV2_CANCEL_RETRY_TIMEOUT_S = 1.0
NAV2_RECOVERY_TIMEOUT_S = 5.0
POST_REORIENT_FORWARD_ONLY_S = 8.0
# NavFn paths start at the robot — pose[0]→pose[1] yaw is nearly current
# heading (vacuous gate). Score yaw to a look-ahead chord instead.
# 1.5 m was still near-aligned on GOAL fallback; 5 m catches first bend.
PATH_YAW_LOOKAHEAD_M = 5.0
FOLLOW_PATH_VX_MIN_PARAM = 'FollowPath.vx_min'
FOLLOW_PATH_VX_MIN_DEFAULT = 0.0  # NAV forward-only fallback if get fails
# Entry 073: sealed-start local unseal (NOT CostCritic weight — Humble
# fail_flag is collision-only; enabled_ is configure-time only).
LOCAL_OBSTACLE_LAYER_ENABLED_PARAM = 'obstacle_layer.enabled'
SEALED_UNSEAL_RESTORE_DEADLINE_S = 0.5
# Must exceed SEALED_UNSEAL_LOCUS_ARM_M so arm+restore cannot thrash (R12:
# clear_radius 2.5 with arm≤4.0 restored in <0.5 s before DriveOnHeading).
SEALED_UNSEAL_CLEAR_RADIUS_M = 5.0
SEALED_UNSEAL_MIN_HOLD_S = 8.0
# Must cover DriveOnHeading + Spin reorient + first FollowPath seconds.
# R12b: 12 s max_hold restored mid-reorient → Optimizer-fail on sealed start.
SEALED_UNSEAL_MAX_HOLD_S = 45.0
SEALED_UNSEAL_HARD_MAX_S = 60.0
SEALED_UNSEAL_LOCUS_ARM_M = 4.0
SEALED_UNSEAL_PATH_GAP_M = 5.0
LIDAR_DRIVE_MIN_CLEAR_M = 1.25
LIDAR_DRIVE_HALF_WIDTH_RAD = 0.45
PATH_B_TANGENT_DRIVE_M = 2.5
# Entry 083 Phase 1A: earlier active-path keepout invalidate / switch standoff.
# Dedicated active-gate horizon — do NOT raise memory_block_horizon_m globally
# (gui_stall distant-ghost false-abort risk).
EARLY_SWITCH_HORIZON_M = 8.0
SWITCH_STANDOFF_MIN_M = 4.0  # X: preferred dist(robot, locus) at switch arm
KEEPOUT_SEAT_DISK_M = 1.5  # Y: seated-in-keepout failure class
ENTRY083_BLOCK_DIAG_DIR = '/tmp/entry083_block_diag'
# Entry 081: spawn-stem snap detection (shared Path A/B prefix).
STEM_PREFIX_MAX_ARC_M = 8.0
STEM_NEAR_START_M = 3.0
STEM_ROBOT_AWAY_M = 5.0
STEM_EXCLUSIVE_GAP_M = 2.0
SHARED_PREFIX_SEP_M = 1.0


def wrap_angle(rad: float) -> float:
    """Wrap angle to (−π, π]."""
    return math.atan2(math.sin(rad), math.cos(rad))


def yaw_error(current_yaw: float, target_yaw: float) -> float:
    """Signed yaw error target − current, wrapped to (−π, π]."""
    return wrap_angle(float(target_yaw) - float(current_yaw))


def yaw_aligned(err_rad: float, theta_exit: float = THETA_EXIT_RAD) -> bool:
    """True iff absolute yaw error is strictly below the FollowPath gate."""
    return abs(float(err_rad)) < float(theta_exit)


def rank_switch_join_candidates(
    robot_xy: Tuple[float, float],
    robot_yaw: float,
    goal_xy: Optional[Tuple[float, float]],
    route_points: Sequence[Tuple[float, float]],
    route_yaws: Sequence[float],
    *,
    limit: int = 10,
    max_gap_m: float = 25.0,
    dedup_m: float = 2.5,
) -> List[Tuple[float, float, float, int]]:
    """Rank Path-B join poses for NavFn bridge (Entry 070 / 048 / 034).

    Distance still dominates, but after Path-A exclusive barrier stalls the
    nearest ahead joins often sit through keepout. Prefer joins with:
      - goal progress (closer to mission goal than the robot)
      - lateral offset from the robot's current heading (escape the pocket)
    Far near-goal snaps and >10 m hops remain penalised.
    """
    if not route_points:
        return []
    rx, ry = float(robot_xy[0]), float(robot_xy[1])
    yaw = float(robot_yaw)
    cy, sy = math.cos(yaw), math.sin(yaw)
    gx = gy = None
    robot_goal_d = None
    if goal_xy is not None:
        gx, gy = float(goal_xy[0]), float(goal_xy[1])
        robot_goal_d = math.hypot(rx - gx, ry - gy)
    stride = max(1, len(route_points) // 200)
    scored: List[Tuple[float, float, float, float, float, int]] = []
    for i in range(0, len(route_points), stride):
        x, y = float(route_points[i][0]), float(route_points[i][1])
        d = math.hypot(x - rx, y - ry)
        if d > float(max_gap_m):
            continue
        dx, dy = x - rx, y - ry
        along = dx * cy + dy * sy
        # Drop deep behind-robot samples from the queue entirely (R6 reverse).
        if along < -2.0:
            continue
        lateral = abs(-dx * sy + dy * cy)
        same_x_bonus = -0.75 if abs(dx) <= 4.0 else 0.0
        far_pen = 0.0 if d <= 10.0 else (d - 10.0) * 4.0
        goal_pen = 0.0
        progress_bonus = 0.0
        if gx is not None and gy is not None and robot_goal_d is not None:
            join_goal_d = math.hypot(x - gx, y - gy)
            if join_goal_d < 3.0:
                goal_pen = 2.5
            # Entry 034 goal-progress — but do not let near-goal snaps through
            # the Path-A pocket outrank lateral escapes (R5 ~10.6 m class).
            progress = max(0.0, robot_goal_d - join_goal_d)
            if lateral >= 3.0 or along <= 1.0:
                progress_bonus = progress * 0.50
            else:
                progress_bonus = progress * 0.12
        # Ahead into the Path-A pocket without lateral escape → penalise.
        escape_pen = 0.0
        if along > 3.0 and lateral < 4.0:
            escape_pen = (4.0 - lateral) * 2.5
        # Entry 070 R6: behind-robot joins (−14,5.7 class) caused reverse +
        # end_dist≈35 m. Strongly prefer forward / lateral-forward joins.
        behind_pen = 0.0
        if along < 0.0:
            behind_pen = abs(along) * 3.0 + 10.0
        lateral_bonus = min(lateral, 8.0) * 0.45
        yaw_i = (
            float(route_yaws[i]) if i < len(route_yaws) else 0.0
        )
        score = (
            d + same_x_bonus + far_pen + goal_pen + escape_pen + behind_pen
            - progress_bonus - lateral_bonus
        )
        scored.append((score, d, x, y, yaw_i, i))
    scored.sort(key=lambda t: t[0])
    out: List[Tuple[float, float, float, int]] = []
    for _rank, _d, x, y, yaw_i, i in scored:
        if any(math.hypot(x - ox, y - oy) < float(dedup_m) for ox, oy, _y, _i in out):
            continue
        out.append((x, y, yaw_i, i))
        if len(out) >= int(limit):
            break
    if not out:
        best = None
        for i in range(0, len(route_points), stride):
            x, y = float(route_points[i][0]), float(route_points[i][1])
            d = math.hypot(x - rx, y - ry)
            if best is None or d < best[0]:
                yaw_i = (
                    float(route_yaws[i]) if i < len(route_yaws) else 0.0
                )
                best = (d, x, y, yaw_i, i)
        if best is not None:
            out.append((best[1], best[2], best[3], best[4]))
    return out


def bridge_arc_length_m(path_xy: Sequence[Tuple[float, float]]) -> float:
    """Polyline arc length in metres."""
    if len(path_xy) < 2:
        return 0.0
    total = 0.0
    for (x0, y0), (x1, y1) in zip(path_xy, path_xy[1:]):
        total += math.hypot(float(x1) - float(x0), float(y1) - float(y0))
    return total


def bridge_detour_excessive(
    path_xy: Sequence[Tuple[float, float]],
    *,
    gap_m: float,
    max_arc_m: float = 32.0,
    max_ratio: float = 2.6,
) -> Tuple[bool, str]:
    """True if NavFn bridge is an absurd forest-loop vs chord gap (R7/R8 class)."""
    arc = bridge_arc_length_m(path_xy)
    gap = max(0.1, float(gap_m))
    limit = max(float(max_arc_m), gap * float(max_ratio))
    if arc > limit:
        return True, f'detour_arc={arc:.1f}m_gap={gap:.1f}m_limit={limit:.1f}m'
    return False, ''


def pick_escape_hop_pose(
    robot_xy: Tuple[float, float],
    robot_yaw: float,
    cost_at,
    *,
    hop_m: float = 3.5,
    samples: int = 16,
    free_cost_max: int = 80,
    attract_xy: Optional[Tuple[float, float]] = None,
    repel_xy: Optional[Tuple[float, float]] = None,
    hop_radii: Optional[Sequence[float]] = None,
    require_attract_progress: bool = True,
    min_repel_gain_m: float = 1.25,
) -> Optional[Tuple[float, float, float]]:
    """Pick a short free pose to leave a Path-A barrier pocket toward Path B.

    Returns ``(x, y, yaw)`` or None. Does not weaken keepout — only samples
    already-free cells. Entry 072: R10/R11 hops that only gained Path-B *x*
    while moving south left the robot on the barrier NE corner; MPPI then
    Optimizer-failed. Require ``repel_xy`` (Path-A block locus) distance gain
    and bias toward ``attract_xy`` (Path-B / GOAL).
    """
    if cost_at is None:
        return None
    rx, ry = float(robot_xy[0]), float(robot_xy[1])
    yaw = float(robot_yaw)
    cy, sy = math.cos(yaw), math.sin(yaw)
    radii: List[float]
    if hop_radii is not None and len(hop_radii) > 0:
        radii = [float(r) for r in hop_radii if float(r) > 0.5]
    else:
        base = float(hop_m)
        radii = sorted({base, max(base, 5.0), max(base, 6.5), max(base, 8.0)})
    ax = ay = None
    robot_attr_d = None
    if attract_xy is not None:
        ax, ay = float(attract_xy[0]), float(attract_xy[1])
        robot_attr_d = math.hypot(rx - ax, ry - ay)
    px = py = None
    robot_repel_d = None
    if repel_xy is not None:
        px, py = float(repel_xy[0]), float(repel_xy[1])
        robot_repel_d = math.hypot(rx - px, ry - py)
    best = None  # (score, x, y, heading)
    n = max(4, int(samples))
    for hop_r in radii:
        for k in range(n):
            # Start at +90° (left lateral) and sweep full circle.
            ang = yaw + 0.5 * math.pi + (2.0 * math.pi * k / n)
            ca, sa = math.cos(ang), math.sin(ang)
            x = rx + hop_r * ca
            y = ry + hop_r * sa
            mx = rx + 0.5 * hop_r * ca
            my = ry + 0.5 * hop_r * sa
            # Also require the 25%/75% samples free — soft-band midpoints
            # were accepting hops that MPPI could not leave (R10).
            q1x = rx + 0.25 * hop_r * ca
            q1y = ry + 0.25 * hop_r * sa
            q3x = rx + 0.75 * hop_r * ca
            q3y = ry + 0.75 * hop_r * sa
            samples_xy = ((x, y), (mx, my), (q1x, q1y), (q3x, q3y))
            costs = []
            ok = True
            for sx, sy_ in samples_xy:
                c = cost_at(sx, sy_)
                if c is None:
                    ok = False
                    break
                ci = int(c)
                if ci >= 255 or ci >= int(free_cost_max):
                    ok = False
                    break
                costs.append(ci)
            if not ok:
                continue
            dx, dy = x - rx, y - ry
            along = dx * cy + dy * sy
            lateral = abs(-dx * sy + dy * cy)
            if along < -0.5:
                continue  # no reverse hop (Issue B)
            attract_prog = 0.0
            if ax is not None and ay is not None and robot_attr_d is not None:
                hop_attr_d = math.hypot(x - ax, y - ay)
                attract_prog = robot_attr_d - hop_attr_d
                if require_attract_progress and attract_prog < 0.35:
                    continue  # must move meaningfully toward Path B / GOAL
            repel_gain = 0.0
            if px is not None and py is not None and robot_repel_d is not None:
                hop_repel_d = math.hypot(x - px, y - py)
                repel_gain = hop_repel_d - robot_repel_d
                if repel_gain < float(min_repel_gain_m):
                    continue  # must leave Path-A barrier locus
            # Prefer barrier-repel, then attract, then free/lateral.
            mean_c = sum(costs) / max(1, len(costs))
            score = (
                5.0 * repel_gain
                + 3.0 * attract_prog
                + 0.55 * lateral
                + 0.25 * max(0.0, along)
                + 0.15 * hop_r
                - 0.04 * float(mean_c)
            )
            if best is None or score > best[0]:
                best = (score, x, y, ang)
    if best is None:
        return None
    return (best[1], best[2], best[3])


def pick_drive_escape_heading(
    robot_xy: Tuple[float, float],
    robot_yaw: float,
    cost_at,
    *,
    repel_xy: Optional[Tuple[float, float]] = None,
    attract_xy: Optional[Tuple[float, float]] = None,
    drive_m: float = 2.5,
    samples: int = 16,
    free_cost_max: int = 80,
) -> Optional[float]:
    """Pick a forward DriveOnHeading yaw that leaves the Path-A barrier corner.

    Returns absolute map yaw or None. Forward-only (Issue B). Used when MPPI
    CostCritic Optimizer-fails because the footprint sits on the barrier edge
    (Nav2 keepout soft-stop / no Humble override_lethal_cost).
    """
    if cost_at is None:
        return None
    rx, ry = float(robot_xy[0]), float(robot_xy[1])
    yaw = float(robot_yaw)
    px = py = None
    robot_repel_d = None
    if repel_xy is not None:
        px, py = float(repel_xy[0]), float(repel_xy[1])
        robot_repel_d = math.hypot(rx - px, ry - py)
    ax = ay = None
    robot_attr_d = None
    if attract_xy is not None:
        ax, ay = float(attract_xy[0]), float(attract_xy[1])
        robot_attr_d = math.hypot(rx - ax, ry - ay)
    best = None  # (score, yaw)
    n = max(4, int(samples))
    dm = max(1.0, float(drive_m))
    for k in range(n):
        ang = yaw + (2.0 * math.pi * k / n)
        ca, sa = math.cos(ang), math.sin(ang)
        # Forward hemisphere only relative to current yaw (no reverse).
        along = ca * math.cos(yaw) + sa * math.sin(yaw)
        if along < 0.15:
            continue
        ok = True
        for frac in (0.35, 0.7, 1.0):
            sx = rx + frac * dm * ca
            sy_ = ry + frac * dm * sa
            c = cost_at(sx, sy_)
            if c is None:
                ok = False
                break
            ci = int(c)
            if ci >= 255 or ci >= int(free_cost_max):
                ok = False
                break
        if not ok:
            continue
        ex = rx + dm * ca
        ey = ry + dm * sa
        repel_gain = 0.0
        if px is not None and py is not None and robot_repel_d is not None:
            repel_gain = math.hypot(ex - px, ey - py) - robot_repel_d
            if repel_gain < 0.8:
                continue
        attract_prog = 0.0
        if ax is not None and ay is not None and robot_attr_d is not None:
            attract_prog = robot_attr_d - math.hypot(ex - ax, ey - ay)
        score = 4.0 * repel_gain + 2.0 * attract_prog + 0.5 * along
        if best is None or score > best[0]:
            best = (score, ang)
    if best is None:
        return None
    return float(best[1])


def path_b_tangent_yaw(
    robot_xy: Tuple[float, float],
    attract_xy: Tuple[float, float],
) -> float:
    """Absolute map yaw from robot toward Path-B / GOAL attractor (geometric).

    Entry 073: costmap-free heading for DriveOnHeading when the sealed local
    footprint makes ``pick_drive_escape_heading`` return None.
    """
    return math.atan2(
        float(attract_xy[1]) - float(robot_xy[1]),
        float(attract_xy[0]) - float(robot_xy[0]),
    )


def lidar_cone_clear(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    *,
    cone_center_rad: float = 0.0,
    half_width_rad: float = LIDAR_DRIVE_HALF_WIDTH_RAD,
    min_clear_m: float = LIDAR_DRIVE_MIN_CLEAR_M,
    range_min: float = 0.05,
    range_max_limit: float = 25.0,
) -> bool:
    """True if no valid return closer than ``min_clear_m`` in a base-frame cone.

    Used as the collision gate for Path-B-tangent DriveOnHeading while the
    local LIDAR ObstacleLayer is temporarily disabled (Entry 073).
    """
    if not ranges or angle_increment == 0.0:
        return False  # no evidence → do not drive open-loop blind
    amin = float(angle_min)
    ainc = float(angle_increment)
    half = abs(float(half_width_rad))
    center = float(cone_center_rad)
    lo = center - half
    hi = center + half
    min_c = float(min_clear_m)
    rmin = float(range_min)
    rmax = float(range_max_limit)
    for i, raw in enumerate(ranges):
        try:
            r = float(raw)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(r) or r < rmin or r > rmax:
            continue
        ang = amin + float(i) * ainc
        # Wrap-safe cone membership via angular distance.
        d = abs(math.atan2(math.sin(ang - center), math.cos(ang - center)))
        if d > half:
            continue
        if r < min_c:
            return False
        _ = (lo, hi)  # retained for readability / future sector tests
    return True


def should_arm_sealed_start_unseal(
    dist_to_path_b_m: float,
    dist_to_locus_m: Optional[float],
    *,
    min_path_gap_m: float = SEALED_UNSEAL_PATH_GAP_M,
    max_locus_m: float = SEALED_UNSEAL_LOCUS_ARM_M,
) -> bool:
    """True when A→B far-gap seats the robot on the Path-A exclusive barrier."""
    if float(dist_to_path_b_m) <= float(min_path_gap_m):
        return False
    if dist_to_locus_m is None:
        return float(dist_to_path_b_m) > 8.0  # far-gap without locus still unseal
    return float(dist_to_locus_m) <= float(max_locus_m)


def _resample_polyline_step(
    points: Sequence[Tuple[float, float]], step_m: float = 0.5
) -> List[Tuple[float, float]]:
    """Resample a polyline at approximately ``step_m`` arc spacing."""
    if not points:
        return []
    if len(points) == 1:
        return [(float(points[0][0]), float(points[0][1]))]
    out: List[Tuple[float, float]] = [
        (float(points[0][0]), float(points[0][1]))
    ]
    acc = 0.0
    step = max(1e-3, float(step_m))
    for a, b in zip(points, points[1:]):
        ax, ay = float(a[0]), float(a[1])
        bx, by = float(b[0]), float(b[1])
        seg = math.hypot(bx - ax, by - ay)
        if seg < 1e-9:
            continue
        t = step - acc
        while t <= seg:
            u = t / seg
            out.append((ax + u * (bx - ax), ay + u * (by - ay)))
            t += step
        acc = seg - (t - step)
    last = (float(points[-1][0]), float(points[-1][1]))
    if math.hypot(out[-1][0] - last[0], out[-1][1] - last[1]) > 1e-6:
        out.append(last)
    return out


def nearest_vertex_index(
    points: Sequence[Tuple[float, float]], rx: float, ry: float
) -> int:
    """Index of the polyline vertex nearest ``(rx, ry)``."""
    if not points:
        return 0
    best_i, best_d = 0, float('inf')
    for i, (x, y) in enumerate(points):
        d = (float(x) - rx) * (float(x) - rx) + (float(y) - ry) * (float(y) - ry)
        if d < best_d:
            best_d, best_i = d, i
    return best_i


def arc_length_to_index(
    points: Sequence[Tuple[float, float]], i: int
) -> float:
    """Arc length from start through vertex ``i`` (inclusive)."""
    if not points or i < 0:
        return 0.0
    end = min(int(i) + 1, len(points))
    return float(path_length(points[:end]))


def shared_prefix_end_index(
    path_a: Sequence[Tuple[float, float]],
    path_b: Sequence[Tuple[float, float]],
    sep_m: float = SHARED_PREFIX_SEP_M,
) -> int:
    """Last index on ``path_b`` still inside the shared spawn stem.

    Sharedness is measured on 0.5 m-resampled polylines, then mapped back onto
    ``path_b`` by **arc length** (never compare densified vs resampled indices).
    Empty ``path_a`` → stem capped at ``STEM_PREFIX_MAX_ARC_M``.
    """
    if not path_b:
        return 0
    if not path_a:
        # Arc-only stem: last vertex with arc ≤ STEM_PREFIX_MAX_ARC_M.
        last = 0
        for j in range(len(path_b)):
            if arc_length_to_index(path_b, j) <= STEM_PREFIX_MAX_ARC_M:
                last = j
            else:
                break
        return last
    a = _resample_polyline_step(path_a, 0.5)
    b = _resample_polyline_step(path_b, 0.5)
    last_r = 0
    n = min(len(a), len(b))
    for i in range(n):
        if math.hypot(a[i][0] - b[i][0], a[i][1] - b[i][1]) <= float(sep_m):
            last_r = i
        else:
            break
    prefix_arc = arc_length_to_index(b, last_r)
    # Cap stem to STEM_PREFIX_MAX_ARC_M for safety.
    prefix_arc = min(prefix_arc, STEM_PREFIX_MAX_ARC_M)
    last = 0
    for j in range(len(path_b)):
        if arc_length_to_index(path_b, j) <= prefix_arc + 1e-6:
            last = j
        else:
            break
    return last


def is_spawn_stem_snap(
    *,
    robot_xy: Tuple[float, float],
    bank_start: Tuple[float, float],
    target_points: Sequence[Tuple[float, float]],
    path_a_points: Sequence[Tuple[float, float]],
) -> bool:
    """True when nearest target vertex is on the shared spawn stem, not exclusive B."""
    if not target_points:
        return False
    rx, ry = float(robot_xy[0]), float(robot_xy[1])
    sx, sy = float(bank_start[0]), float(bank_start[1])
    if math.hypot(rx - sx, ry - sy) < STEM_ROBOT_AWAY_M:
        return False  # still at spawn — normal FollowPath OK

    ni = nearest_vertex_index(target_points, rx, ry)
    nx, ny = float(target_points[ni][0]), float(target_points[ni][1])
    prefix_end = shared_prefix_end_index(path_a_points, target_points)
    ni_arc = arc_length_to_index(target_points, ni)
    prefix_arc = arc_length_to_index(target_points, prefix_end)

    stem_by_arc = ni_arc <= STEM_PREFIX_MAX_ARC_M
    stem_by_start = math.hypot(nx - sx, ny - sy) <= STEM_NEAR_START_M
    # Compare by arc length (prefix_end already mapped onto target_points).
    stem_by_shared = ni_arc <= prefix_arc + 1e-6

    if not (stem_by_arc or stem_by_start or stem_by_shared):
        return False

    exclusive = [
        (float(p[0]), float(p[1]))
        for j, p in enumerate(target_points)
        if arc_length_to_index(target_points, j) > STEM_PREFIX_MAX_ARC_M
    ]
    if not exclusive:
        return False
    stride = max(1, len(exclusive) // 50)
    d_excl = min(
        math.hypot(rx - x, ry - y) for x, y in exclusive[::stride]
    )
    return d_excl > STEM_EXCLUSIVE_GAP_M


def should_restore_sealed_start_unseal(
    dist_to_locus_m: Optional[float],
    armed_at: float,
    now: float,
    *,
    clear_radius_m: float = SEALED_UNSEAL_CLEAR_RADIUS_M,
    min_hold_s: float = SEALED_UNSEAL_MIN_HOLD_S,
    max_hold_s: float = SEALED_UNSEAL_MAX_HOLD_S,
    hard_max_s: float = SEALED_UNSEAL_HARD_MAX_S,
    defer: bool = False,
) -> Tuple[bool, str]:
    """Whether to restore local LIDAR ObstacleLayer (Entry 073 safety exit).

    Prefer ``cleared_locus``. Soft ``max_hold`` restores only when not deferred
    (reorient / splice still running). ``hard_max`` always restores.
    """
    held = float(now) - float(armed_at)
    if held >= float(hard_max_s):
        return True, 'hard_max'
    if held < float(min_hold_s):
        return False, ''
    if (
        dist_to_locus_m is not None
        and float(dist_to_locus_m) >= float(clear_radius_m)
    ):
        return True, 'cleared_locus'
    if held >= float(max_hold_s) and not defer:
        return True, 'max_hold'
    return False, ''


def bridge_prefix_blocked(
    path_xy: Sequence[Tuple[float, float]],
    cost_at,
    *,
    prefix_m: float = 5.0,
    skip_m: float = 1.50,
    lethal_threshold: int = 254,
    min_lethal_hits: int = 2,
) -> Tuple[bool, str]:
    """True if NavFn bridge prefix clips true lethal (not soft inflation).

    Entry 070 R6: inscribed (253) near Path-A barrier rejected *every* escape
    bridge (robot sits in inflation). Require ≥2 true lethal (254) samples
    after ``skip_m`` so soft-band alone does not starve FollowPath.
    ``cost_at(x, y) -> Optional[int]``.
    """
    if not path_xy or cost_at is None:
        return False, ''
    acc = 0.0
    prev = path_xy[0]
    hits = 0
    first_hit_m = None
    for i in range(1, len(path_xy)):
        x, y = float(path_xy[i][0]), float(path_xy[i][1])
        step = math.hypot(x - prev[0], y - prev[1])
        if step < 1e-9:
            prev = (x, y)
            continue
        acc += step
        prev = (x, y)
        if acc < float(skip_m):
            continue
        if acc > float(prefix_m):
            break
        c = cost_at(x, y)
        if c is None:
            continue
        ci = int(c)
        if ci >= 255:
            continue  # unknown
        if ci >= int(lethal_threshold):
            hits += 1
            if first_hit_m is None:
                first_hit_m = acc
            if hits >= int(min_lethal_hits):
                return True, (
                    f'prefix_lethal_at_{first_hit_m:.2f}m_cost_{ci}'
                    f'_hits_{hits}'
                )
    return False, ''


LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


def _unpack_xy_cloud(cloud: PointCloud2) -> List[Tuple[float, float]]:
    """Extract XY from a dense xyz float32 PointCloud2 (fire_memory_cloud)."""
    if cloud is None or cloud.point_step < 8 or int(cloud.width) == 0:
        return []
    pts: List[Tuple[float, float]] = []
    raw = bytes(cloud.data)
    step = int(cloud.point_step)
    n = int(cloud.width) * max(1, int(cloud.height))
    for i in range(n):
        off = i * step
        if off + 8 > len(raw):
            break
        x, y = struct.unpack_from('<ff', raw, off)
        if math.isfinite(x) and math.isfinite(y):
            pts.append((float(x), float(y)))
    return pts


class MissionNavState(str, Enum):
    IDLE = 'IDLE'
    INITIALISING = 'INITIALISING'
    PLANNING = 'PLANNING'
    FOLLOWING_PATH = 'FOLLOWING_PATH'
    PATH_BLOCKED = 'PATH_BLOCKED'
    SWITCHING_PATH = 'SWITCHING_PATH'
    NO_PATH_AVAILABLE = 'NO_PATH_AVAILABLE'
    # Entry 024: all banked routes exhausted → navigate back to mission start.
    RETURNING_TO_START = 'RETURNING_TO_START'
    RETURNED_TO_START = 'RETURNED_TO_START'
    ARRIVED = 'ARRIVED'
    FAILED = 'FAILED'
    # Phase Two §3.2 / APPROVE watch-out: timer-excluded terminal (do NOT use
    # NO_PATH_AVAILABLE here — that state stays in _on_timer active set and
    # can thrash). UX string / owner.detail also 'NO_SAFE_ROUTE'.
    NO_SAFE_ROUTE = 'NO_SAFE_ROUTE'
    SAFE_HOLD = 'SAFE_HOLD'
    NAV2_CANCEL_FAILED = 'NAV2_CANCEL_FAILED'
    FIRE_UNLOCALIZED_HOLD = 'FIRE_UNLOCALIZED_HOLD'


class RefuseContext(str, Enum):
    """Why bare FollowPath was refused under spawn-stem snap (Entry 084)."""

    SYNC_SWITCH_ARM = 'SYNC_SWITCH_ARM'
    ASYNC_BRIDGE = 'ASYNC_BRIDGE'
    CENTRAL_GATE_POST_DRIVE = 'CENTRAL_GATE_POST_DRIVE'
    CENTRAL_GATE_STUCK = 'CENTRAL_GATE_STUCK'
    CENTRAL_GATE_OTHER = 'CENTRAL_GATE_OTHER'


class PathBankManager(Node):
    def __init__(self) -> None:
        super().__init__('path_bank_manager')
        self._cbg = ReentrantCallbackGroup()

        self.declare_parameter('robot_name', 'husky1')
        self.declare_parameter('enabled', True)
        self.declare_parameter('max_candidates', 4)
        self.declare_parameter('exclusion_radius_m', 2.5)
        self.declare_parameter('min_path_separation_m', 2.0)
        self.declare_parameter('min_clearance_m', 0.35)
        # A2: length demoted 0.45→0.35 so bilateral centreline can compete.
        self.declare_parameter('length_weight', 0.35)
        self.declare_parameter('clearance_weight', 0.35)
        self.declare_parameter('cost_weight', 0.15)
        self.declare_parameter('smoothness_weight', 0.05)
        self.declare_parameter('bilateral_weight', 0.20)
        self.declare_parameter('clearance_ref_m', 2.0)
        # A1 soft DT (mission bank only — not Wave-1 home ComputePath).
        self.declare_parameter('soft_dt_max_cost', 80.0)
        self.declare_parameter('soft_dt_decay_m', 0.40)
        self.declare_parameter('nudge_target_clearance_m', 0.80)
        # Live operator centreline preference (apply next bind/rebuild/switch).
        self.declare_parameter('operator_clearance_m', 0.80)
        self.declare_parameter('check_hz', 2.0)
        # 0.50 m (was 0.38): bank Dijkstra leaves centreline farther from
        # trail walls so RPP local footprint check does not knife-edge.
        self.declare_parameter('footprint_inflate_m', 0.50)
        # Entry 019+: after PGM↔Gazebo alignment, default launch enables
        # switch_resend (preempt onto next banked route). Parameter still
        # defaults False here so unit/manual node starts stay observe-only
        # unless launch sets path_bank_switch_resend:=true.
        self.declare_parameter('auto_switch', True)
        self.declare_parameter('switch_resend_goal', False)
        self.declare_parameter('validity_grace_s', 15.0)
        # Entry 023: after A→B switch, pause validity so Keepout/local costmap
        # and NavFn settle before the new active path can be re-blocked.
        self.declare_parameter('post_switch_grace_s', 8.0)
        # Active-path memory gate: only marks within this distance of the robot
        # can invalidate the active route. Distant trail-wall ghosts (e.g.
        # (-23,-8) near spawn, or (23.5,19.7) ~8 m ahead on Path B) previously
        # aborted routes before a real near-field block. Keep ≤ demo
        # trigger_distance_m (6.0) so gap_a still invalidates Path A when the
        # robot is ~5 m from the spawned box (gui_stall_fix residual).
        self.declare_parameter('memory_block_horizon_m', 5.0)
        # Entry 063 / G6: FIRE PointCloud2 corridor refuse radius for unlock
        # (trail half-width ≈0.8 m). Not an OccupancyGrid blunt full-path block.
        self.declare_parameter('fire_unlock_corridor_radius_m', 0.80)
        self.declare_parameter('cancel_on_no_path', False)
        # Entry 024: when the bank is exhausted, preempt NavigateToPose home.
        self.declare_parameter('return_home_on_all_blocked', True)
        self.declare_parameter('home_yaw', 0.0)
        # TEST-ONLY (default false): relocate recorded home to an unreachable
        # pose when ALL_BLOCKED → return-home starts, so ComputePathToPose /
        # footprint validation fails → NO_SAFE_ROUTE. Production must leave
        # this unset/false (N6 Gazebo harness only).
        self.declare_parameter('test_force_unreachable_home', False)
        # Keep 254 (LETHAL). Using 253 (inscribed) false-blocks alternate
        # bank routes on soft inflation and yields active_id=None (T2 flake).
        self.declare_parameter('lethal_threshold', 254)
        # Phase 2: goal_pose / bank following arms NAV authority (legacy NTP).
        # StopMission / inhibits force NONE via MissionOwnerState.
        self.declare_parameter('publish_command_authority', True)
        self.declare_parameter('pending_failsafe_s', 15.0)
        # Phase 3: bank-owned FollowPath (default). Rollback: legacy NTP.
        self.declare_parameter('mission_nav_authority', 'bank_follow_path')

        def _as_bool(value) -> bool:
            if isinstance(value, str):
                return value.strip().lower() in ('1', 'true', 'yes', 'on')
            return bool(value)

        robot = str(self.get_parameter('robot_name').value).strip('/') or 'husky1'
        self._robot = robot
        self._map_frame = f'{robot}_map'
        self._base_frame = f'{robot}_base_link'
        self._enabled = _as_bool(self.get_parameter('enabled').value)
        self._auto_switch = _as_bool(self.get_parameter('auto_switch').value)
        self._switch_resend = _as_bool(self.get_parameter('switch_resend_goal').value)
        self._grace_s = float(self.get_parameter('validity_grace_s').value)
        self._post_switch_grace_s = float(
            self.get_parameter('post_switch_grace_s').value)
        self._memory_block_horizon_m = float(
            self.get_parameter('memory_block_horizon_m').value)
        self._cancel_on_no_path = _as_bool(self.get_parameter('cancel_on_no_path').value)
        self._return_home_on_all_blocked = _as_bool(
            self.get_parameter('return_home_on_all_blocked').value)
        self._home_yaw = float(self.get_parameter('home_yaw').value)
        auth = str(self.get_parameter('mission_nav_authority').value).strip()
        if auth not in ('bank_follow_path', 'legacy_navigate_to_pose'):
            self.get_logger().warn(
                f'Unknown mission_nav_authority={auth!r}; using bank_follow_path'
            )
            auth = 'bank_follow_path'
        self._mission_nav_authority = auth
        self._use_follow_path = auth == 'bank_follow_path'
        self._bank_ready_at: Optional[float] = None
        # Entry 023: absolute monotonic deadline after a bank switch.
        self._switch_grace_until: float = 0.0
        # Ignore NavigateToPose / FollowPath result callbacks from goals we
        # intentionally superseded during a bank switch (preempt/abort expected).
        self._ignore_goal_results = 0
        self._ignore_fp_results = 0
        # Per-submit epoch so cancel results do not swallow the next goal's abort
        # (counter-based ignore raced after Path A→B and left FollowPath dead).
        self._fp_goal_epoch: int = 0
        self._fp_cancelled_epochs: set = set()
        self._fp_submit_t: float = 0.0
        # Entry 023: one abort→resend per mission goal to recover ownership
        # when the post-switch NavigateToPose/FollowPath aborts while FOLLOWING_PATH.
        # Entry 024: also used once while RETURNING_TO_START (resend home).
        self._abort_resend_used = False
        # Generation captured when a FollowPath goal is submitted (fence).
        self._fp_submit_generation: int = -1
        self._switch_action_token = 0
        self._fp_cancel_context = None
        self._fp_cancel_retry_used = False
        self._fp_cancel_started_at = 0.0
        self._fp_cancel_deadline = 0.0
        self._fp_cancel_reason = ''
        self._home_route: Optional[RouteInstance] = None
        # Owner stuck recovery (BT inactive): progress watchdog + orphaned handle.
        self.declare_parameter('stuck_timeout_s', 8.0)
        self.declare_parameter('stuck_progress_m', 0.25)
        # Phase Two Wave 0 hang bounds (tunable).
        # Ladder (explicit — do not collapse wording):
        #   Mission stuck N → fresh home once → (plan fail | home N | T₂) → §3.2
        #   T₁ / T_seed → §3.2 direct (no home)
        self.declare_parameter('stuck_recovery_budget', 8)
        self.declare_parameter('terminal_path_blocked_none_s', 20.0)  # T₁
        self.declare_parameter('home_stuck_timeout_s', 60.0)  # T₂
        self.declare_parameter('start_seed_timeout_s', 30.0)  # T_seed
        self.declare_parameter('far_gap_rebuild_budget', 3)  # Entry 049
        self._stuck_timeout_s = float(self.get_parameter('stuck_timeout_s').value)
        self._stuck_progress_m = float(self.get_parameter('stuck_progress_m').value)
        self._stuck_budget_n = int(self.get_parameter('stuck_recovery_budget').value)
        self._t1_path_blocked_none_s = float(
            self.get_parameter('terminal_path_blocked_none_s').value)
        self._t2_home_stuck_s = float(
            self.get_parameter('home_stuck_timeout_s').value)
        self._t_seed_s = float(self.get_parameter('start_seed_timeout_s').value)
        self._far_gap_rebuild_budget = int(
            self.get_parameter('far_gap_rebuild_budget').value)
        self._last_progress_xy: Optional[Tuple[float, float]] = None
        self._last_progress_t: float = 0.0
        self._stuck_recover_until: float = 0.0
        self._stuck_recoveries: int = 0
        self._path_blocked_none_since: float = 0.0
        self._home_entered_at: float = 0.0
        # Wave-1 return-home latch (T₁ reads this; stuck retains sibling flag).
        self._home_attempted: bool = False
        self._home_attempted_from_stuck: bool = False
        self._home_plan_pending: bool = False
        self._home_plan_attempt_count: int = 0  # C2: max 2 while nav_mode=home
        self._home_fp_route_uid: Optional[str] = None
        self._rebuild_resume_mode: Optional[str] = None  # 'HOME'|'MISSION'|None
        self._last_home_footprint_reason: str = ''
        # W0 / H6: machine-readable raise token for status + GUI banner suffix.
        self._last_no_safe_detail: str = ''
        # W1a via FSM scratch (initialized here; product geometry gated on W0).
        self._last_home_navfn_pts: List[Tuple[float, float]] = []
        # Densified/nudged forward-slice pts at C1 fail — lethal_at_<i> indexes these.
        self._last_home_c1_fail_pts: List[Tuple[float, float]] = []
        self._home_via_phase: Optional[str] = None
        self._home_via_generation: int = -1
        self._home_via_v_star: Optional[Tuple[float, float]] = None
        self._home_via_leg1_pts: List[Tuple[float, float]] = []
        self._home_via_leg_timeout_s: float = 8.0
        self._home_via_leg_started_at: float = 0.0
        self._home_via_candidate_queue: List[Tuple[float, float]] = []
        self._dist_home_at_return_arm: float = -1.0
        self._pending_follow_seed_since: float = 0.0
        # Planning is deliberately isolated from the ROS executor.  The
        # generation token makes cancellation safe even when Python cannot
        # interrupt a running CPU-bound worker immediately.
        self._planning_executor = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix='path-bank')
        self._planning_future: Optional[Future] = None
        self._planning_token = 0
        self._planning_mission_id = -1
        self._planning_started_at = 0.0
        self._planning_stage = ''
        self._bank_expansion_pending = False
        self._planning_grid = None
        self._planning_start: Optional[Tuple[float, float]] = None
        self._planning_goal: Optional[Tuple[float, float]] = None
        # Entry 049: rebuilds of join queue instead of premature home.
        self._far_gap_rebuilds: int = 0
        # Entry 081: stem-snap bridge failure budget (does not reset each switch).
        self._stem_snap_bridge_fails: int = 0
        # Entry 082: after join+GOAL NavFn exhaust, allow one Drive retry then
        # escalate — never orphaned_handle forever while "live-valid".
        self._bridge_exhaust_drive_retries: int = 0
        # Entry 083 Phase 0: one-shot Switch NavFn seating diagnostic per mission.
        self._entry083_block_diag_logged: bool = False

        self._cfg = PathBankConfig(
            max_candidates=int(self.get_parameter('max_candidates').value),
            exclusion_radius_m=float(self.get_parameter('exclusion_radius_m').value),
            min_path_separation_m=float(self.get_parameter('min_path_separation_m').value),
            min_clearance_m=float(self.get_parameter('min_clearance_m').value),
            lethal_threshold=int(self.get_parameter('lethal_threshold').value),
            length_weight=float(self.get_parameter('length_weight').value),
            clearance_weight=float(self.get_parameter('clearance_weight').value),
            cost_weight=float(self.get_parameter('cost_weight').value),
            smoothness_weight=float(self.get_parameter('smoothness_weight').value),
            bilateral_weight=float(self.get_parameter('bilateral_weight').value),
            clearance_ref_m=float(self.get_parameter('clearance_ref_m').value),
            soft_dt_max_cost=float(self.get_parameter('soft_dt_max_cost').value),
            soft_dt_decay_m=float(self.get_parameter('soft_dt_decay_m').value),
            nudge_target_clearance_m=float(
                self.get_parameter('nudge_target_clearance_m').value),
        )
        self._footprint_inflate = float(self.get_parameter('footprint_inflate_m').value)
        self._operator_clearance_m = float(
            self.get_parameter('operator_clearance_m').value)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._prior_map: Optional[OccupancyGrid] = None
        self._live_costmap: Optional[Costmap] = None
        self._memory_mask: Optional[OccupancyGrid] = None
        self._bank: Optional[PathBank] = None
        self._route_bank: Optional[RouteBank] = None
        # Entry 035: world XY of the corridor that caused BLOCKED. Evidence
        # unblock only requires memory clear near this locus (not far-field
        # lidar residuals elsewhere on the polyline that clear_max cannot
        # reach after the robot approaches the authorising clear site).
        self._block_locus: dict = {}
        # Match T5/T14 mask accept_radius (0.60 m). Search-ring halo (~1.1 m
        # wall-echo) must not pin unlock once the footprint corridor at the
        # block locus is free; active-path gate still uses full-path memory.
        self._block_locus_radius_m = 0.60
        # Entry 062 / Wave-1: paths blocked this mission (pick harden).
        self._ever_blocked_ids: set = set()
        # Sticky operator/demo corridor latch (G5 primary). While set,
        # evidence unlock must refuse AVAILABLE even if locus disk is clear.
        self._operator_block_latch: dict = {}  # path_id -> True
        self._operator_block_seat_xy: dict = {}  # path_id -> (x, y)
        # NB-M1 pin (R2 prefer A): full-path refuse-unlock only for corridors
        # that ever carried a demo/operator latch this mission. Never-demo
        # blocks keep Entry 035 locus-scoped unlock.
        self._ever_demo_latched_ids: set = set()
        # Entry 063 / G6: typed FIRE centres from fire_memory_cloud (cached,
        # including empty). GUI CLEAR must not wipe this; unlock refuses while
        # any centre is within fire_unlock_corridor_radius_m of unsliced cand.
        self._fire_memory_centres: List[Tuple[float, float]] = []
        self._fire_unlock_corridor_radius_m = float(
            self.get_parameter('fire_unlock_corridor_radius_m').value)
        # path_id → 'FIRE' | 'LIDAR' | 'DEMO' | 'OTHER' (manager-local Wave-1).
        self._block_source: dict = {}
        # Live demo obstacle seats from GUI ForestObstacleManager publishes.
        self._demo_obstacle_poses: List[Tuple[float, float]] = []
        # Pin (R2 m1): associate block locus ↔ demo seat within this radius.
        # bush_trail gap_a/gap_b centres are ~32 m apart; 3.0 m covers the
        # 0.45×2.2 box + locus sample noise without cross-seat latch.
        self._operator_latch_assoc_radius_m = 3.0
        # Known bush_trail seats (fallback when poses lag one tick).
        self._known_demo_seats: Tuple[Tuple[float, float], ...] = (
            (-1.3, -11.5),  # gap_a / Path A
            (-5.1, 21.1),   # gap_b / Path B
        )
        # Entry 063 / Workstream A: exact submitted FollowPath Goal.path cache.
        self._last_submitted_path: Optional[Path] = None
        self._follow_path_ref_epoch: int = -1
        self._odom_trail: List[Tuple[float, float]] = []
        self._odom_trail_max = 200
        self._state = MissionNavState.IDLE
        self._goal_xyyaw: Optional[Tuple[float, float, float]] = None
        # Entry 024: mission-start home; nav_mode 'mission' | 'home' | None.
        self._home_xyyaw: Optional[Tuple[float, float, float]] = None
        self._nav_mode: Optional[str] = None
        self._goal_handle = None
        self._fp_goal_handle = None
        self._robot_xyyaw: Optional[Tuple[float, float, float]] = None
        self._amcl_seen = False
        self._switch_count = 0
        self._building = False
        self._owner = MissionOwnerState()
        self._pending_failsafe_s = float(
            self.get_parameter('pending_failsafe_s').value)
        self._publish_authority = _as_bool(
            self.get_parameter('publish_command_authority').value)
        self._pending_fire_since: Optional[float] = None
        # Queued Path A→B switch while authority is transiently NONE (rebuild).
        self._switch_pending = False
        self._forward_only_until = 0.0
        # T5/sensor-only observation transport: cancel FollowPath + suppress
        # stuck resend without StopMission latch / lifecycle CRITICAL (Entry 035).
        self._observation_hold = False

        self.create_subscription(OccupancyGrid, 'prior_map', self._on_prior, LATCHED)
        self.create_subscription(
            Costmap, 'global_costmap/costmap_raw', self._on_costmap, 2)
        self.create_subscription(
            OccupancyGrid, 'obstacle_memory/mask', self._on_memory_mask, LATCHED)
        # Entry 062: observe GUI demo poses for operator/demo corridor latch.
        self.create_subscription(
            PoseArray, 'demo_obstacle/poses', self._on_demo_obstacle_poses, LATCHED)
        # Entry 063 / G6: typed FIRE centres (volatile depth-10 publisher).
        self.create_subscription(
            PointCloud2, 'fire_memory_cloud', self._on_fire_memory_cloud, 10)
        self.create_subscription(Path, 'plan', self._on_plan, 5)
        self.create_subscription(PoseStamped, 'goal_pose', self._on_goal_pose, 10)
        self.create_subscription(
            Bool, 'mission_owner/pending_fire', self._on_pending_fire, LATCHED)
        self.create_subscription(
            Bool, 'mission_owner/observation_hold',
            self._on_observation_hold, LATCHED)
        try:
            from std_msgs.msg import UInt64
            self.create_subscription(
                UInt64, 'obstacle_memory/memory_revision',
                self._on_memory_revision, LATCHED)
        except Exception:  # noqa: BLE001
            pass
        # §G starts only on typed FIRE mark/clear — not every LiDAR fingerprint
        # bump (that thrash cancelled FollowPath every ~2s and froze T2).
        self.create_subscription(
            String, 'obstacle_memory/events', self._on_memory_event, 50)
        self._memory_revision = 0
        self._rebuild_retries = 0
        self._rebuild_pending = False
        self._rebuild_rev_snapshot = 0
        self._rebuild_verify_after = 0.0
        self._pending_follow_seed = False
        # Map-frame pose for StartMission bank build (amcl) + odom fallback.
        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl_pose, 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        # Soft cancel from Status GUI (GoalVisual / arrival) — does not replace
        # StopMission; clears bank follow without going through the gate.
        self.create_subscription(
            Bool, 'path_bank/cancel', self._on_cancel_request, 10)

        self._state_pub = self.create_publisher(String, 'path_bank/state', 10)
        self._status_pub = self.create_publisher(String, 'path_bank/status', LATCHED)
        # Latched so late harness subscribers (block exclusive-point) still see
        # geometry after one-shot publish at bank bind (Entry 034).
        self._active_path_pub = self.create_publisher(
            Path, 'path_bank/active_path', LATCHED)
        self._alt_path_pubs = [
            self.create_publisher(
                Path, f'path_bank/candidate_{i}', LATCHED)
            for i in range(1, self._cfg.max_candidates + 1)
        ]
        self._marker_pub = self.create_publisher(MarkerArray, 'path_bank/markers', 10)
        # Optional bounded odom trail (grey) — distinct from FollowPath ref.
        self._odom_trail_pub = self.create_publisher(
            Path, 'path_bank/odom_trail', 5)

        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose', callback_group=self._cbg)
        self._fp_client: Optional[ActionClient] = None
        self._compute_path_client: Optional[ActionClient] = None
        self._spin_client: Optional[ActionClient] = None
        self._drive_on_heading_client: Optional[ActionClient] = None
        self._controller_get_cli = self.create_client(
            GetParameters, 'controller_server/get_parameters',
            callback_group=self._cbg,
        )
        self._controller_set_cli = self.create_client(
            SetParameters, 'controller_server/set_parameters',
            callback_group=self._cbg,
        )
        # Issue B reorient state (Spin primary; temp vx_min=0 fallback only).
        self._reorient_active = False
        self._reorient_allow_fp = False  # one-shot after yaw gate passes
        self._reorient_phase: Optional[str] = None  # 'spin' | 'fallback'
        self._reorient_route: Optional[RouteInstance] = None
        self._reorient_generation: Optional[int] = None
        self._reorient_allow_navfn = True
        self._reorient_started_at = 0.0
        self._reorient_saved_vx_min: Optional[float] = None
        self._reorient_vx_min_asserted = False
        self._reorient_path_msg: Optional[Path] = None  # bridge msg path
        self._reorient_path_uid = ''
        self._reorient_path_id = 0
        self._spin_goal_handle = None
        self._spin_action_token = 0
        self._splice_used = False
        self._splice_pending = False
        self._switch_bridge_join_i = 0
        if self._use_follow_path:
            self._fp_client = ActionClient(
                self, FollowPath, 'follow_path', callback_group=self._cbg)
            # Humble alt after centreline-nudge + failure_tolerance still hit
            # RPP collision loops: NavFn splice to a RouteInstance checkpoint.
            self._compute_path_client = ActionClient(
                self, ComputePathToPose, 'compute_path_to_pose',
                callback_group=self._cbg)
            # Issue B: bank-owned Spin via behavior_server → cmd_vel_nav.
            self._spin_client = ActionClient(
                self, Spin, 'spin', callback_group=self._cbg)
            # Entry 072: forward DriveOnHeading to leave Path-A barrier corner
            # when MPPI CostCritic seals the start pose (no reverse / Backup).
            self._drive_on_heading_client = ActionClient(
                self, DriveOnHeading, 'drive_on_heading',
                callback_group=self._cbg)
        else:
            self._drive_on_heading_client = None
        # Entry 027: clear stale Path-A local costs after bank switch so RPP
        # collision lookahead does not hard-zero on the new Path B plan.
        self._clear_local_cli = self.create_client(
            ClearEntireCostmap,
            'local_costmap/clear_entirely_local_costmap',
            callback_group=self._cbg,
        )
        # Entry 073: temporary local LIDAR ObstacleLayer disable so clear sticks
        # (live scan otherwise re-seals CostCritic in <1 update cycle).
        # FIRE stays on fire_obstacle_layer — never disabled here.
        # Costmap2DROS node is local_costmap/local_costmap (see bringup log
        # husky1.local_costmap.local_costmap). clear_entirely lives on the
        # outer name; set_parameters is on the inner Costmap2DROS.
        self._local_costmap_set_cli = self.create_client(
            SetParameters,
            'local_costmap/local_costmap/set_parameters',
            callback_group=self._cbg,
        )
        self._local_costmap_set_cli_alt = self.create_client(
            SetParameters,
            'local_costmap/set_parameters',
            callback_group=self._cbg,
        )
        self._sealed_unseal_active = False
        self._sealed_unseal_armed_at = 0.0
        self._sealed_unseal_layer_was_enabled = True
        self._sealed_unseal_layer_disabled = False
        self._sealed_unseal_forward_latch = False
        # Latest scan for Path-B-tangent DriveOnHeading collision gate.
        self._scan_ranges: Optional[List[float]] = None
        self._scan_angle_min = 0.0
        self._scan_angle_inc = 0.0
        self._scan_range_min = 0.05
        self._scan_range_max = 25.0
        self.create_subscription(
            LaserScan, 'scan', self._on_scan, 5, callback_group=self._cbg)
        # Entry 072: ask obstacle_memory to clear LIDAR-only cells under the
        # robot footprint (Humble KeepoutFilter has no override_lethal_cost).
        # Never clears FIRE; radius capped in the memory manager (~1.0 m).
        self._footprint_escape_pub = self.create_publisher(
            PoseStamped, 'obstacle_memory/footprint_escape_clear', 2)
        # Entry 073: third approach is local unseal + Path-B-tangent drive —
        # hop budget stays 0 on switch/stuck (no more hop variants).
        self._escape_hop_count = 0
        self._escape_hop_budget = 0
        self._drive_escape_used = False
        self._drive_escape_pending_join = False
        self._drive_goal_handle = None
        # Stock bringup: per-node ChangeState on bt_navigator (activate/
        # deactivate). Split lifecycle_manager_bt deferred after rs1_navigation
        # params failures (No critics / DWB).
        self._bt_change_state_cli = self.create_client(
            ChangeState,
            'bt_navigator/change_state',
            callback_group=self._cbg,
        )

        self._authority_pub = self.create_publisher(
            String, 'mission_owner/command_authority', LATCHED)
        self._owner_status_pub = self.create_publisher(
            MissionOwnerStatus, 'mission_owner/status', LATCHED)
        self._forward_only_pub = self.create_publisher(
            Bool, 'path_bank/forward_only', LATCHED)

        self.create_service(
            StartMission, 'mission_owner/start_mission',
            self._srv_start_mission, callback_group=self._cbg)
        self.create_service(
            StopMission, 'mission_owner/stop_mission',
            self._srv_stop_mission, callback_group=self._cbg)
        self.create_service(
            EnableManualNav, 'mission_owner/enable_manual_nav',
            self._srv_enable_manual_nav, callback_group=self._cbg)
        self.create_service(
            DisableManualNav, 'mission_owner/disable_manual_nav',
            self._srv_disable_manual_nav, callback_group=self._cbg)
        self.create_service(
            EnableTeleop, 'mission_owner/enable_teleop',
            self._srv_enable_teleop, callback_group=self._cbg)
        self.create_service(
            DisableTeleop, 'mission_owner/disable_teleop',
            self._srv_disable_teleop, callback_group=self._cbg)

        hz = float(self.get_parameter('check_hz').value)
        self.create_timer(1.0 / max(hz, 0.1), self._on_timer, callback_group=self._cbg)
        self.create_timer(1.0, self._publish_status, callback_group=self._cbg)
        self.create_timer(0.5, self._publish_owner_status, callback_group=self._cbg)

        # Fail-closed until a Start / legacy goal arms NAV.
        self._publish_authority_now()

        self.get_logger().info(
            f'path_bank_manager ready (robot={robot}, enabled={self._enabled}, '
            f'max_candidates={self._cfg.max_candidates}, '
            f'mission_nav_authority={self._mission_nav_authority}, '
            f'owner_phase={self._owner.phase.value})'
        )

    # -- callbacks ---------------------------------------------------------

    def _on_set_parameters(self, params: List[Parameter]) -> SetParametersResult:
        """Latch ``operator_clearance_m``; apply on next bind/rebuild/switch only.

        R3-N2: reject below footprint floor or above absolute max (do not clamp
        silently). Corridor-aware fail-safe at apply time still seeks max
        feasible mid-corridor via centre-seeking nudge.
        """
        result = SetParametersResult()
        result.successful = True
        for param in params:
            if param.name != 'operator_clearance_m':
                continue
            try:
                val = float(param.value)
            except (TypeError, ValueError):
                result.successful = False
                result.reason = 'operator_clearance_m must be a float'
                return result
            if val < OPERATOR_CLEARANCE_MIN_M - 1e-9:
                result.successful = False
                result.reason = (
                    f'operator_clearance_m must be >= '
                    f'{OPERATOR_CLEARANCE_MIN_M:.2f} m '
                    f'(FOOTPRINT_HALF_W+PAD)'
                )
                return result
            if val > OPERATOR_CLEARANCE_ABS_MAX_M + 1e-9:
                result.successful = False
                result.reason = (
                    f'operator_clearance_m must be <= '
                    f'{OPERATOR_CLEARANCE_ABS_MAX_M:.2f} m '
                    f'(corridor seal fail-safe)'
                )
                return result
            self._operator_clearance_m = val
            self.get_logger().info(
                f'operator_clearance_m latched to {val:.2f} m '
                f'(applies on next bind/rebuild/switch)'
            )
        return result

    def _mission_nudge_target_m(self) -> float:
        """Centre-seeking bind target: max(config, operator clearance)."""
        cfg_tgt = float(self._cfg.nudge_target_clearance_m)
        op = float(self._operator_clearance_m)
        return max(cfg_tgt, op, OPERATOR_CLEARANCE_MIN_M)

    def _on_prior(self, msg: OccupancyGrid) -> None:
        self._prior_map = msg

    def _on_costmap(self, msg: Costmap) -> None:
        self._live_costmap = msg

    def _on_memory_mask(self, msg: OccupancyGrid) -> None:
        self._memory_mask = msg
        # Mask clear can unlock BLOCKED routes without waiting for the next
        # costmap tick (T5 observation hold — planner may be quiet).
        if (
            self._bank is not None
            and self._live_costmap is not None
            and self._state in (
                MissionNavState.FOLLOWING_PATH,
                MissionNavState.PATH_BLOCKED,
                MissionNavState.NO_PATH_AVAILABLE,
            )
        ):
            self._try_evidence_unblock(nav2_costmap_to_grid(self._live_costmap))

    def _on_demo_obstacle_poses(self, msg: PoseArray) -> None:
        """Track GUI forest demo seats for sticky operator-block latch (Entry 062)."""
        poses: List[Tuple[float, float]] = []
        for p in msg.poses:
            poses.append((float(p.position.x), float(p.position.y)))
        self._demo_obstacle_poses = poses
        # CLEAR FIRE / clear_obstacles republishes empty poses — drop all latches.
        if not poses:
            self._clear_operator_block_latches(reason='demo_poses_empty')
            return
        # Gap-gone for a single seat: clear that path's latch only when the
        # associated seat is gone AND locus disk + full-path memory are clear.
        self._maybe_clear_operator_latches_for_gone_gaps()

    def _note_robot_pose(self, x: float, y: float, yaw: float) -> None:
        self._robot_xyyaw = (x, y, yaw)
        self._append_odom_trail(x, y)
        if self._last_progress_xy is None:
            self._last_progress_xy = (x, y)
            self._last_progress_t = time.monotonic()
            return
        dx = x - self._last_progress_xy[0]
        dy = y - self._last_progress_xy[1]
        if math.hypot(dx, dy) >= self._stuck_progress_m:
            self._last_progress_xy = (x, y)
            self._last_progress_t = time.monotonic()

    def _append_odom_trail(self, x: float, y: float) -> None:
        """Bounded map-frame trail for RViz (distinct from FollowPath ref)."""
        if self._odom_trail:
            lx, ly = self._odom_trail[-1]
            if math.hypot(x - lx, y - ly) < 0.15:
                return
        self._odom_trail.append((float(x), float(y)))
        if len(self._odom_trail) > self._odom_trail_max:
            self._odom_trail = self._odom_trail[-self._odom_trail_max :]
        path_msg = Path()
        path_msg.header.frame_id = self._map_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for px, py in self._odom_trail:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = px
            ps.pose.position.y = py
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        self._odom_trail_pub.publish(path_msg)

    def _on_fire_memory_cloud(self, msg: PointCloud2) -> None:
        """Cache typed FIRE centres (incl. empty). Never cleared by GUI CLEAR."""
        self._fire_memory_centres = _unpack_xy_cloud(msg)

    def _fire_cloud_intersects_corridor(
        self,
        points: List[Tuple[float, float]],
        radius_m: Optional[float] = None,
    ) -> Optional[Tuple[float, float]]:
        """Return first FIRE centre within radius of unsliced candidate polyline.

        Uses bank centreline samples (``cand.points``), not live-wall densify.
        """
        centres = self._fire_memory_centres
        if not centres or not points:
            return None
        r = (
            float(self._fire_unlock_corridor_radius_m)
            if radius_m is None else float(radius_m)
        )
        r2 = r * r
        for fx, fy in centres:
            for x, y in points:
                if (fx - x) * (fx - x) + (fy - y) * (fy - y) <= r2:
                    return (float(fx), float(fy))
        return None

    def _record_block_source(self, cand: PathCandidate, reason: str) -> None:
        """Tag path_id with FIRE / DEMO / OTHER for G6 unlock refuse (B2)."""
        pid = int(cand.path_id)
        pts = list(cand.points)
        hit = self._fire_cloud_intersects_corridor(pts)
        reason_l = (reason or '').lower()
        if hit is not None or 'fire' in reason_l or 'thermal' in reason_l:
            self._block_source[pid] = 'FIRE'
            self.get_logger().info(
                f'block_source Path {pid}=FIRE'
                + (
                    f' cloud=({hit[0]:.2f},{hit[1]:.2f})'
                    if hit is not None else f' reason={reason}'
                )
            )
            return
        locus = self._block_locus.get(pid)
        if locus is not None and self._nearest_demo_seat(locus) is not None:
            self._block_source[pid] = 'DEMO'
            return
        if pid not in self._block_source:
            self._block_source[pid] = 'OTHER'

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        # Prefer AMCL / map-frame localisation when available.
        self._amcl_seen = True
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        self._note_robot_pose(
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            yaw,
        )

    def _on_odom(self, msg: Odometry) -> None:
        # Fallback when amcl_pose is absent (SLAM+EKF stack uses /odom).
        if self._amcl_seen:
            return
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        self._note_robot_pose(
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            yaw,
        )

    def _on_cancel_request(self, msg: Bool) -> None:
        """GUI soft-cancel: drop active bank follow without StopMission latch."""
        if not msg.data:
            return
        self._cancel_nav()
        self._bank = None
        self._goal_xyyaw = None
        self._set_state(MissionNavState.IDLE)
        self._clear_active_path()

    def _on_observation_hold(self, msg: Bool) -> None:
        """Sensor-only transport hold — pause FollowPath, keep bank validity.

        Unlike StopMission this does not latch STOPPED / bump generation, so
        evidence_unblock still runs while the harness open-loops into FOV.
        Gate switches to TELEOP so open-loop publishes on cmd_vel_teleop reach
        the robot without fighting NAV smoother output.
        """
        hold = bool(msg.data)
        if hold == self._observation_hold:
            return
        self._observation_hold = hold
        if hold:
            self._cancel_follow_path()
            self._cancel_nav()  # also cancel return-home NTP/FollowPath
            # Do not bump generation / change phase — only redirect the gate.
            self._owner.command_authority = CommandAuthority.TELEOP
            self._publish_authority_now()
            self.get_logger().info(
                'observation_hold ON — FollowPath cancelled; gate=TELEOP; '
                'bank validity continues'
            )
        else:
            # D4: restore NAV under RETURNING / home, not only NAVIGATING.
            returning_home = (
                self._owner.phase == MissionPhase.RETURNING_TO_START
                or self._nav_mode == 'home'
            )
            if (
                (
                    self._owner.phase == MissionPhase.NAVIGATING
                    or returning_home
                )
                and not self._owner.has_blocking_inhibit()
                and not self._owner.operator_stop_latch
            ):
                self._owner.command_authority = CommandAuthority.NAV
            else:
                self._owner.command_authority = CommandAuthority.NONE
            self._publish_authority_now()
            self.get_logger().info(
                f'observation_hold OFF — gate={self._owner.command_authority.value}'
            )
            if (
                returning_home
                and self._owner.command_authority == CommandAuthority.NAV
                and self._use_follow_path
            ):
                # Resubmit once if home route still C1-valid; else one replan.
                if (
                    self._home_route is not None
                    and self._home_route_c1_oriented_valid(self._home_route)[0]
                ):
                    self._send_follow_path(
                        self._home_route, allow_navfn_bridge=False)
                elif self._home_plan_attempt_count < 2:
                    if not self._request_fresh_home_plan():
                        self.get_logger().warn(
                            'observation_hold OFF: home replan unavailable; '
                            'staying RETURNING'
                        )
                else:
                    self.get_logger().warn(
                        'observation_hold OFF: home route invalid and plan '
                        'budget exhausted; staying RETURNING'
                    )

    def _on_pending_fire(self, msg: Bool) -> None:
        """Bridge signals unlocalized thermal-positive (§F)."""
        if msg.data:
            if SafetyInhibit.THERMAL_FIRE_PENDING not in self._owner.active_inhibits:
                if SafetyInhibit.PENDING_FIRE_UNRESOLVED in self._owner.active_inhibits:
                    return  # terminal — do not downgrade
                self.raise_blocking_inhibit(SafetyInhibit.THERMAL_FIRE_PENDING)
            return
        if SafetyInhibit.PENDING_FIRE_UNRESOLVED in self._owner.active_inhibits:
            self.get_logger().warn(
                'Ignoring pending_fire=False while FIRE_UNLOCALIZED_HOLD '
                'requires operator intervention'
            )
            return
        # Pending cleared by association/mark — only clear THERMAL_FIRE_PENDING
        # if still that code (not terminal unresolved). §S1 clear-without-mark
        # is handled separately; association path drops pending here.
        if SafetyInhibit.THERMAL_FIRE_PENDING in self._owner.active_inhibits:
            self._owner.clear_inhibit(SafetyInhibit.THERMAL_FIRE_PENDING)
            self._pending_fire_since = None
            self._publish_authority_now()
            self.get_logger().info(
                'THERMAL_FIRE_PENDING cleared (thermal associated / pending dropped)'
            )

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        if not self._enabled:
            return
        # Legacy goal_pose path: reject if safety inhibits block Start.
        reject = self._owner.start_reject_code()
        if reject and SafetyInhibit.HOLD_REBUILD_TIMEOUT not in self._owner.active_inhibits:
            # Allow HOLD_REBUILD alone to be handled by StartMission rebuild path;
            # goal_pose cannot clear other inhibits.
            if any(
                i in self._owner.active_inhibits
                for i in (
                    SafetyInhibit.THERMAL_FIRE_PENDING,
                    SafetyInhibit.PENDING_FIRE_UNRESOLVED,
                    SafetyInhibit.SENSOR_AUTHORITY_FAULT,
                    SafetyInhibit.HOLD_PERSISTENT_SAFETY,
                    SafetyInhibit.ALL_BLOCKED_NO_HOME,
                )
            ):
                self.get_logger().warn(
                    f'Ignoring goal_pose while inhibit active ({reject})'
                )
                return
        if self._owner.phase == MissionPhase.HOLDING:
            self.get_logger().warn('Ignoring goal_pose while HOLDING')
            return
        yaw = _yaw_from_quat(msg.pose.orientation)
        self._goal_xyyaw = (msg.pose.position.x, msg.pose.position.y, yaw)
        self._set_state(MissionNavState.PLANNING)
        self._bank = None
        self._route_bank = None
        self._home_route = None
        self._home_xyyaw = None
        self._nav_mode = None
        self._home_attempted = False
        self._home_attempted_from_stuck = False
        self._home_plan_pending = False
        self._home_plan_attempt_count = 0
        self._home_fp_route_uid = None
        self._rebuild_resume_mode = None
        self._last_home_footprint_reason = ''
        self._last_no_safe_detail = ''
        self._last_home_navfn_pts = []
        self._home_via_phase = None
        self._home_via_generation = -1
        self._home_via_v_star = None
        self._home_via_leg1_pts = []
        self._dist_home_at_return_arm = -1.0
        self._far_gap_rebuilds = 0
        self._stem_snap_bridge_fails = 0
        self._bridge_exhaust_drive_retries = 0
        self._entry083_block_diag_logged = False
        self._path_blocked_none_since = 0.0
        self._switch_count = 0
        self._abort_resend_used = False
        # Phase-2 legacy: treat goal_pose as StartMission-equivalent for gate.
        if not self._owner.has_blocking_inhibit():
            self._owner.accept_normal_start()
            self._publish_authority_now()
        self.get_logger().info(
            f'Goal pose received ({self._goal_xyyaw[0]:.2f}, {self._goal_xyyaw[1]:.2f}); '
            'will build path bank on next plan/costmap'
        )
        if self._use_follow_path and self._robot_xyyaw is not None:
            start = (self._robot_xyyaw[0], self._robot_xyyaw[1])
            goal = (self._goal_xyyaw[0], self._goal_xyyaw[1])
            self._build_bank(start, goal)

    def _on_plan(self, msg: Path) -> None:
        if not self._enabled or self._building:
            return
        # Entry 024: ignore plan chatter while returning / returned home.
        if self._state in (
            MissionNavState.RETURNING_TO_START,
            MissionNavState.RETURNED_TO_START,
        ):
            return
        # FollowPath mode owns geometry — ignore plan chatter once bank exists
        # (NavFn rewrite must not silently replace the active RouteInstance).
        if self._use_follow_path and self._bank is not None:
            return
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(points) < 2:
            return
        # First plan of a mission: build bank if we do not have one yet.
        if self._bank is None:
            start = points[0]
            goal = points[-1]
            if self._goal_xyyaw is not None:
                goal = (self._goal_xyyaw[0], self._goal_xyyaw[1])
            else:
                # NavigateToPose clients do not publish goal_pose; recover the
                # mission goal from the first Nav2 plan endpoint.
                self._goal_xyyaw = (goal[0], goal[1], 0.0)
            self._build_bank(start, goal)
            # Minimal FollowPath seed: first plan only seeds geometry; cancel
            # any NTP. _build_bank already submitted FollowPath for active route.
            if self._use_follow_path:
                self._cancel_nav(ntp_only=True)
        if self._state in (MissionNavState.PLANNING, MissionNavState.SWITCHING_PATH, MissionNavState.IDLE):
            self._set_state(MissionNavState.FOLLOWING_PATH)
            self._nav_mode = 'mission'

    def _on_scan(self, msg: LaserScan) -> None:
        """Cache latest LIDAR for sealed-start DriveOnHeading collision gate."""
        try:
            self._scan_ranges = [float(r) for r in msg.ranges]
            self._scan_angle_min = float(msg.angle_min)
            self._scan_angle_inc = float(msg.angle_increment)
            self._scan_range_min = float(msg.range_min)
            self._scan_range_max = float(msg.range_max)
        except (TypeError, ValueError, AttributeError):
            pass

    def _on_timer(self) -> None:
        if not self._enabled:
            return
        if self._pending_follow_seed and self._use_follow_path:
            if self._try_complete_follow_path_bank():
                self.get_logger().info('Deferred FollowPath bank seed completed')
                self._pending_follow_seed_since = 0.0
            else:
                self._try_start_follow_path_bank()
        self._maybe_finish_rebuild()
        self._maybe_restore_sealed_start_unseal()
        self._publish_forward_only()
        self._tick_follow_path_cancellation()
        self._maybe_complete_pending_switch()
        # Hang bounds before bank switch (Wave 0) — may terminalize.
        self._maybe_terminal_hang_bounds()
        if self._state == MissionNavState.NO_SAFE_ROUTE:
            return
        # Republish candidate Path topics so late subscribers (T2 exclusive
        # finder) still see geometry — FollowPath mode has no /plan chatter.
        if self._bank is not None and self._bank.candidates:
            self._publish_bank_paths()
        self._maybe_owner_stuck_recover()
        if self._bank is None or self._live_costmap is None:
            return
        # Entry 024: do not run bank switch logic while returning / returned.
        # NO_SAFE_ROUTE intentionally excluded (timer thrash watch-out).
        if self._state not in (
            MissionNavState.FOLLOWING_PATH,
            MissionNavState.PATH_BLOCKED,
            MissionNavState.NO_PATH_AVAILABLE,
        ):
            return
        # Allow StaticLayer + ObstacleLayer + inflation to settle before the
        # bank starts invalidating candidates (clean-start false positives).
        now = time.monotonic()
        if self._bank_ready_at is not None:
            if (now - self._bank_ready_at) < self._grace_s:
                return
        # Entry 023: post-switch settle — do not re-block the newly active path
        # while NavFn / KeepoutFilter / local costmap catch up.
        if now < self._switch_grace_until:
            return

        live = nav2_costmap_to_grid(self._live_costmap)
        # Entry 021: evidence-based BLOCKED→AVAILABLE (no auto path flip).
        self._try_evidence_unblock(live)

        active = self._bank.active()
        if active is None:
            # Entry 062 W1-PICK: never bare select_best_available — recheck
            # costmap + memory (+ prior-block harden) via _pick_next_valid.
            chosen = self._pick_next_valid(live)
            if chosen is not None:
                self._sync_route_bank_active(chosen.path_id)
                self._publish_bank_paths()
                # W1-RESUME: only leave NO_PATH after a successful validated pick.
                if (
                    self._state == MissionNavState.NO_PATH_AVAILABLE
                    and self._nav_mode != 'home'
                ):
                    self._set_state(MissionNavState.FOLLOWING_PATH)
            return

        ok, reason = self._active_route_valid(active, live)
        if ok:
            return

        self.get_logger().warn(
            f'Active Path {active.path_id} invalid on live costmap: {reason}'
        )
        self._set_state(MissionNavState.PATH_BLOCKED)
        self._mark_path_blocked(active, reason)
        self.get_logger().info(
            f'ROUTE_MARKED_BLOCKED Path {active.path_id}: {reason}'
        )
        self._publish_bank_paths()

        if not self._auto_switch:
            return

        self._set_state(MissionNavState.SWITCHING_PATH)
        if self._use_follow_path:
            self._switch_follow_path(live)
            return

        nxt = self._pick_next_valid(live)
        if nxt is None:
            self.get_logger().error(
                'No valid banked path remains'
                + (' (all_blocked)' if self._bank.all_blocked() else '')
            )
            self._set_state(MissionNavState.NO_PATH_AVAILABLE)
            if self._return_home_on_all_blocked:
                self._begin_return_to_start()
                return
            # Legacy advisory mode: leave IsPathValid BT in control.
            self.get_logger().error(
                'return_home_on_all_blocked=false; leaving Nav2 IsPathValid BT '
                'in control'
            )
            if self._cancel_on_no_path:
                self._cancel_nav()
            return

        self._switch_count += 1
        self.get_logger().info(
            f'Switching to Path {nxt.path_id} (score={nxt.score:.1f}, '
            f'switch #{self._switch_count})'
        )
        self._publish_bank_paths()
        if self._switch_resend and self._goal_xyyaw is not None:
            # Preempt with a fresh NavigateToPose (do not cancel first). A
            # cancel→resend race was marking FAILED when the aborted goal's
            # result arrived after state returned to FOLLOWING_PATH.
            # Live costmap already contains the blockage so NavFn avoids it.
            self._nav_mode = 'mission'
            if self._goal_handle is not None:
                self._ignore_goal_results += 1
            self._send_goal(*self._goal_xyyaw)
            # Entry 027: drop Path-A barrier leftovers from the rolling local
            # window so FollowPath/RPP does not collision-hold on Path B.
            self._clear_local_costmap()
        # Entry 023: settle window before re-validating the newly active path.
        if self._post_switch_grace_s > 0.0:
            self._switch_grace_until = time.monotonic() + self._post_switch_grace_s
        self._set_state(MissionNavState.FOLLOWING_PATH)

    def _memory_intersects_path(self, points: List[Tuple[float, float]]) -> bool:
        """True if obstacle_memory mask marks occupied near the path.

        Checks a 1-cell neighbourhood around each sample so thin keepout marks
        one cell off the polyline still count (Entry 023 T2 reliability).
        """
        return self._first_memory_hit_on_path(points) is not None

    def _first_memory_hit_on_path(
        self, points: List[Tuple[float, float]]
    ) -> Optional[Tuple[float, float]]:
        mask = self._memory_mask
        if mask is None or not mask.data or not points:
            return None
        info = mask.info
        return first_occupancy_hit_on_path(
            points,
            width=int(info.width),
            height=int(info.height),
            resolution=float(info.resolution),
            origin_x=float(info.origin.position.x),
            origin_y=float(info.origin.position.y),
            data=mask.data,
            sample_stride=self._cfg.sample_stride,
            occupied_threshold=50,
            neighbourhood=1,
        )

    def _memory_intersects_path_near(
        self,
        points: List[Tuple[float, float]],
        locus: Tuple[float, float],
        radius_m: float,
    ) -> bool:
        """Memory gate scoped to path samples within radius of a block locus."""
        if not points:
            return False
        lx, ly = locus
        r2 = float(radius_m) * float(radius_m)
        near = [
            (x, y) for (x, y) in points
            if (x - lx) * (x - lx) + (y - ly) * (y - ly) <= r2
        ]
        if not near:
            return False
        return self._memory_intersects_path(near)

    def _memory_intersects_path_ahead(
        self,
        points: List[Tuple[float, float]],
        *,
        horizon_m: Optional[float] = None,
    ) -> bool:
        """Memory gate limited to samples within ``horizon_m`` of the robot.

        Spatially irrelevant keepout far down the corridor must not abort the
        active route while the robot is still approaching (prompt §6.2 / G2).
        """
        if not points:
            return False
        horizon = (
            self._memory_block_horizon_m if horizon_m is None else float(horizon_m)
        )
        if horizon <= 0.0 or self._robot_xyyaw is None:
            return self._memory_intersects_path(points)
        rx, ry = self._robot_xyyaw[0], self._robot_xyyaw[1]
        h2 = horizon * horizon
        near = [
            (x, y) for (x, y) in points
            if (x - rx) * (x - rx) + (y - ry) * (y - ry) <= h2
        ]
        if not near:
            return False
        return self._memory_intersects_path(near)

    def _mask_occupied_near_xy(
        self, xy: Tuple[float, float], radius_m: float
    ) -> bool:
        """True if any mask cell ≥50 lies within radius_m of xy (disk)."""
        mask = self._memory_mask
        if mask is None or not mask.data:
            return False
        info = mask.info
        w, h = int(info.width), int(info.height)
        res = float(info.resolution)
        if w <= 0 or h <= 0 or res <= 0.0:
            return False
        ox = float(info.origin.position.x)
        oy = float(info.origin.position.y)
        cx = int((xy[0] - ox) / res)
        cy = int((xy[1] - oy) / res)
        r_cells = max(1, int(math.ceil(float(radius_m) / res)))
        r2 = float(radius_m) * float(radius_m)
        data = mask.data
        for dy in range(-r_cells, r_cells + 1):
            for dx in range(-r_cells, r_cells + 1):
                ix, iy = cx + dx, cy + dy
                if not (0 <= ix < w and 0 <= iy < h):
                    continue
                if int(data[iy * w + ix]) < 50:
                    continue
                wx = ox + (ix + 0.5) * res
                wy = oy + (iy + 0.5) * res
                if (wx - xy[0]) * (wx - xy[0]) + (wy - xy[1]) * (wy - xy[1]) <= r2:
                    return True
        return False

    def _candidate_points_for_gate(
        self, cand: PathCandidate
    ) -> List[Tuple[float, float]]:
        pts = list(cand.points)
        if self._use_follow_path and self._route_bank is not None:
            route = self._route_bank.by_path_id(cand.path_id)
            if route is not None:
                pts_t, _yaws = self._route_forward_from_robot(route)
                pts = list(pts_t)
        return pts

    def _record_block_locus(self, cand: PathCandidate, reason: str) -> None:
        """Remember where the corridor went lethal for evidence-unblock scope.

        ``lethal_at_index_N`` indexes the same forward-from-robot sample list
        used by ``_active_route_valid`` / ``_candidate_route_valid`` — not the
        raw ``cand.points`` array (Entry 035 T5: wrong locus kept Path 1
        BLOCKED after the barrier corridor cleared).
        """
        pts = self._candidate_points_for_gate(cand)
        # Prefer the sample that actually failed the live/prior gate.
        if 'lethal_at_index_' in reason:
            try:
                idx_s = reason.split('lethal_at_index_', 1)[1].split('_', 1)[0]
                idx = int(idx_s)
                if 0 <= idx < len(pts):
                    self._block_locus[cand.path_id] = (
                        float(pts[idx][0]), float(pts[idx][1]))
                    return
            except (ValueError, IndexError, TypeError):
                pass
        hit = self._first_memory_hit_on_path(pts)
        if hit is not None:
            self._block_locus[cand.path_id] = hit
            return
        if pts:
            # Nearest path sample to the robot (block usually at front).
            if self._robot_xyyaw is not None:
                rx, ry = self._robot_xyyaw[0], self._robot_xyyaw[1]
                best = min(
                    pts,
                    key=lambda p: (p[0] - rx) * (p[0] - rx) + (p[1] - ry) * (p[1] - ry),
                )
                self._block_locus[cand.path_id] = (float(best[0]), float(best[1]))
            else:
                self._block_locus[cand.path_id] = (float(pts[0][0]), float(pts[0][1]))

    def _mark_path_blocked(self, cand: PathCandidate, reason: str) -> None:
        self._record_block_locus(cand, reason)
        self._record_block_source(cand, reason)
        self._bank.mark_blocked(cand.path_id, reason)
        self._ever_blocked_ids.add(int(cand.path_id))
        locus = self._block_locus.get(cand.path_id)
        if locus is not None:
            self.get_logger().info(
                f'Block locus Path {cand.path_id}: '
                f'({locus[0]:.2f},{locus[1]:.2f}) reason={reason}'
            )
            self._maybe_set_operator_block_latch(cand.path_id, locus)

    def _nearest_demo_seat(
        self, xy: Tuple[float, float]
    ) -> Optional[Tuple[float, float]]:
        """Return nearest *live* demo seat within association radius.

        Known bush_trail centres are labels only — latch arms only when a
        pending/active pose is present on ``demo_obstacle/poses`` (R2 m1 pin:
        assoc radius ``_operator_latch_assoc_radius_m`` = 3.0 m). When a live
        pose is near a known seat, snap to the seat XY for stable clear checks.
        """
        if not self._demo_obstacle_poses:
            return None
        ax, ay = xy
        r = float(self._operator_latch_assoc_radius_m)
        r2 = r * r
        best: Optional[Tuple[float, float]] = None
        best_d2 = r2
        for sx, sy in self._demo_obstacle_poses:
            d2 = (sx - ax) * (sx - ax) + (sy - ay) * (sy - ay)
            if d2 <= best_d2:
                best_d2 = d2
                best = (float(sx), float(sy))
        if best is None:
            return None
        # Snap to known gap_a/gap_b seat when the live pose is that gap.
        for kx, ky in self._known_demo_seats:
            if (best[0] - kx) * (best[0] - kx) + (best[1] - ky) * (best[1] - ky) <= r2:
                return (float(kx), float(ky))
        return best

    def _maybe_set_operator_block_latch(
        self, path_id: int, locus: Tuple[float, float]
    ) -> None:
        """Arm sticky demo/operator latch when block locus coincides with a gap."""
        seat = self._nearest_demo_seat(locus)
        if seat is None:
            return
        self._set_operator_block_latch(path_id, seat)

    def _set_operator_block_latch(
        self, path_id: int, seat_xy: Tuple[float, float]
    ) -> None:
        pid = int(path_id)
        self._operator_block_latch[pid] = True
        self._operator_block_seat_xy[pid] = (
            float(seat_xy[0]), float(seat_xy[1]))
        self._ever_demo_latched_ids.add(pid)
        self.get_logger().info(
            f'operator_block_latch Path {pid}: armed at seat '
            f'({seat_xy[0]:.2f},{seat_xy[1]:.2f})'
        )

    def _operator_block_latch_active(self, path_id: int) -> bool:
        return bool(self._operator_block_latch.get(int(path_id), False))

    def _clear_operator_block_latches(self, *, reason: str = '') -> None:
        if not self._operator_block_latch and not self._operator_block_seat_xy:
            return
        self._operator_block_latch.clear()
        self._operator_block_seat_xy.clear()
        self.get_logger().info(
            f'operator_block_latch: cleared all ({reason or "unspecified"})'
        )

    def _demo_pose_near_xy(
        self, xy: Tuple[float, float], radius_m: Optional[float] = None
    ) -> bool:
        r = (
            float(self._operator_latch_assoc_radius_m)
            if radius_m is None else float(radius_m)
        )
        r2 = r * r
        ax, ay = xy
        for sx, sy in self._demo_obstacle_poses:
            if (sx - ax) * (sx - ax) + (sy - ay) * (sy - ay) <= r2:
                return True
        return False

    def _maybe_clear_operator_latches_for_gone_gaps(self) -> None:
        """Clear per-path latch when its demo seat is gone and corridor memory free."""
        if not self._operator_block_latch:
            return
        to_clear: List[int] = []
        for pid, seat in list(self._operator_block_seat_xy.items()):
            if self._demo_pose_near_xy(seat):
                continue
            locus = self._block_locus.get(pid)
            if locus is not None and self._mask_occupied_near_xy(
                locus, self._block_locus_radius_m
            ):
                continue
            cand = None
            if self._bank is not None:
                for c in self._bank.candidates:
                    if c.path_id == pid:
                        cand = c
                        break
            if cand is not None and self._memory_intersects_path(list(cand.points)):
                continue
            to_clear.append(int(pid))
        for pid in to_clear:
            self._operator_block_latch.pop(pid, None)
            self._operator_block_seat_xy.pop(pid, None)
            self.get_logger().info(
                f'operator_block_latch Path {pid}: cleared (gap gone + memory clear)'
            )

    def _memory_intersects_route(self, route: RouteInstance) -> bool:
        """Oriented-footprint mask intersection for FollowPath validity."""
        mask = self._memory_mask
        if mask is None or not mask.data:
            return False
        info = mask.info
        return mask_under_oriented_footprint(
            route.points,
            route.yaw_rad,
            width=int(info.width),
            height=int(info.height),
            resolution=float(info.resolution),
            origin_x=float(info.origin.position.x),
            origin_y=float(info.origin.position.y),
            data=mask.data,
            occupied_threshold=50,
            sample_stride=self._cfg.sample_stride,
        )

    def _nearest_route_index_progress_aware(
        self,
        points: Sequence[Tuple[float, float]],
        rx: float,
        ry: float,
        *,
        window_m: float = 2.0,
    ) -> int:
        """Nearest route index preferring progress along the corridor.

        Among vertices within ``window_m`` of the robot, pick the one with
        **maximum** arc-from-start (avoids snapping back to a shared spawn
        stem when mid-corridor geometry is also nearby). Else global nearest.
        """
        if not points:
            return 0
        within: List[int] = []
        for i, (x, y) in enumerate(points):
            if math.hypot(float(x) - rx, float(y) - ry) <= float(window_m):
                within.append(i)
        if within:
            best_i = within[0]
            best_arc = -1.0
            for i in within:
                arc = arc_length_to_index(points, i)
                if arc > best_arc:
                    best_arc = arc
                    best_i = i
            return best_i
        return nearest_vertex_index(points, rx, ry)

    def _route_forward_from_robot(
        self, route: RouteInstance
    ) -> Tuple[Tuple[Tuple[float, float], ...], Tuple[float, ...]]:
        """Slice densified route from nearest pose to robot (forward only).

        Validating the original start corridor after the robot has moved causes
        false Path-B rejects when oriented footprints clip trail walls behind
        or beside the spawn (T2 / bush_trail). Entry 081: progress-aware nearest
        avoids stem snap when mid-corridor vertices are also within window.
        """
        pts = route.points
        yaws = route.yaw_rad
        if not pts or self._robot_xyyaw is None:
            return pts, yaws
        rx, ry = self._robot_xyyaw[0], self._robot_xyyaw[1]
        best_i = self._nearest_route_index_progress_aware(pts, rx, ry)
        # Skip the pose under the robot (footprint already occupies it).
        i0 = min(best_i + 1, len(pts) - 1)
        return pts[i0:], yaws[i0:]

    def _route_validity_forward_from_robot(
        self, route: RouteInstance
    ) -> Tuple[Tuple[Tuple[float, float], ...], Tuple[float, ...]]:
        """Forward slice for active-route validity (Entry 082).

        Unlike FollowPath publish, keep the nearest sample so a barrier the
        robot is pressed against still fails ``path_is_valid`` / memory gates.
        Progress-aware nearest is retained (T11 homeward / stem).
        """
        pts = route.points
        yaws = route.yaw_rad
        if not pts or self._robot_xyyaw is None:
            return pts, yaws
        rx, ry = self._robot_xyyaw[0], self._robot_xyyaw[1]
        best_i = self._nearest_route_index_progress_aware(pts, rx, ry)
        return pts[best_i:], yaws[best_i:]

    def _active_route_valid(
        self, active: PathCandidate, live_grid
    ) -> Tuple[bool, str]:
        """Validity for the active PathCandidate.

        Live active-path gate stays centreline (Entry 027 T2). Oriented
        footprint units pass, but bush_trail Path B false-rejected under load
        (lethal_at_* on trail walls after switch) — re-enable oriented live
        only with T2/path_bank_nav evidence. Candidate switch precheck and
        home-route validate may still use oriented helpers elsewhere.

        Entry 082: validity uses proximal-inclusive forward samples (not
        best_i+1), and also horizon-scopes the bank centreline so densify
        dodge + FOOTPRINT_ESCAPE cannot hide a keepout/lethal corridor hit.

        Entry 083 Phase 1A: active memory ahead uses EARLY_SWITCH_HORIZON_M
        (dedicated; memory_block_horizon_m stays 5.0 for other gates) so a
        keepout hit ~6 m ahead invalidates and switches before seating in the
        mid-corridor locus pocket.
        """
        pts = list(active.points)
        if self._use_follow_path and self._route_bank is not None:
            route = self._route_bank.by_path_id(active.path_id)
            if route is not None:
                pts_t, _yaws = self._route_validity_forward_from_robot(route)
                pts = list(pts_t)
        ok, reason = path_is_valid(
            list(pts),
            live_grid,
            lethal_threshold=self._cfg.lethal_threshold,
            sample_stride=self._cfg.sample_stride,
        )
        early_h = float(EARLY_SWITCH_HORIZON_M)
        # Horizon-scoped memory: distant trail-wall ghosts must not kill Path A
        # before the robot reaches the GUI demo gap (gui_stall_fix residual).
        # Entry 083: active gate uses dedicated early-switch horizon (8 m).
        if ok and self._memory_intersects_path_ahead(
            list(pts), horizon_m=early_h
        ):
            return False, 'obstacle_memory_mask'
        # Bank centreline within early horizon (densified route may have dodged).
        if ok and self._memory_intersects_path_ahead(
            list(active.points), horizon_m=early_h
        ):
            return False, 'obstacle_memory_mask'
        # Explicit hit-distance gate (Entry 083): nearest memory hit among
        # forward / near-horizon bank samples within EARLY_SWITCH_HORIZON_M.
        if ok and self._robot_xyyaw is not None:
            rx, ry = self._robot_xyyaw[0], self._robot_xyyaw[1]
            h2 = early_h * early_h
            hit = self._first_memory_hit_on_path(list(pts))
            if hit is None and active.points:
                near_bank_mem = [
                    (x, y) for (x, y) in active.points
                    if (x - rx) * (x - rx) + (y - ry) * (y - ry) <= h2
                ]
                if near_bank_mem:
                    hit = self._first_memory_hit_on_path(near_bank_mem)
            if hit is not None and math.hypot(
                rx - hit[0], ry - hit[1]
            ) <= early_h:
                return False, 'obstacle_memory_mask'
        # Lethal on bank centreline near robot even if densify samples are free.
        # Keep memory_block_horizon_m (5.0) — early horizon is for keepout switch.
        if ok and self._robot_xyyaw is not None and active.points:
            rx, ry = self._robot_xyyaw[0], self._robot_xyyaw[1]
            h = float(self._memory_block_horizon_m)
            h2 = h * h
            near_bank = [
                (x, y) for (x, y) in active.points
                if (x - rx) * (x - rx) + (y - ry) * (y - ry) <= h2
            ]
            if near_bank:
                ok_b, reason_b = path_is_valid(
                    near_bank,
                    live_grid,
                    lethal_threshold=self._cfg.lethal_threshold,
                    sample_stride=self._cfg.sample_stride,
                )
                if not ok_b:
                    return False, reason_b
        return ok, reason

    def _memory_intersects_route_points(
        self,
        points: Tuple[Tuple[float, float], ...],
        yaws: Tuple[float, ...],
    ) -> bool:
        mask = self._memory_mask
        if mask is None or not mask.data or not points:
            return False
        info = mask.info
        return mask_under_oriented_footprint(
            points,
            yaws,
            width=int(info.width),
            height=int(info.height),
            resolution=float(info.resolution),
            origin_x=float(info.origin.position.x),
            origin_y=float(info.origin.position.y),
            data=mask.data,
            occupied_threshold=50,
            sample_stride=self._cfg.sample_stride,
        )

    def _prior_block_locus_behind_forward_slice(
        self,
        fwd_pts: List[Tuple[float, float]],
        locus: Tuple[float, float],
        *,
        tol_m: float = 1.0,
    ) -> bool:
        """True if recorded block locus is not among remaining forward samples.

        When the robot is on Path B near gap_b, forward-slice of Path A drops
        samples near gap_a — empty-mask pick would otherwise false-accept A.
        """
        if not fwd_pts:
            return True
        lx, ly = locus
        tol2 = float(tol_m) * float(tol_m)
        for x, y in fwd_pts:
            if (x - lx) * (x - lx) + (y - ly) * (y - ly) <= tol2:
                return False
        return True

    def _candidate_route_valid(
        self, cand: PathCandidate, live_grid
    ) -> Tuple[bool, str]:
        """Switch precheck: centreline + memory (legacy T2 / Entry 027 gate).

        Entry 062 W1-PICK: for previously blocked candidates, also fail when
        (a) full unsliced centreline hits memory, (b) prior block locus lies
        behind the forward-slice start, or (c) unsliced centreline is lethal
        on the live costmap. First-time never-blocked candidates unchanged.
        """
        full_pts = list(cand.points)
        pts = full_pts
        sliced = False
        if self._use_follow_path and self._route_bank is not None:
            route = self._route_bank.by_path_id(cand.path_id)
            if route is not None:
                pts_t, _yaws = self._route_forward_from_robot(route)
                pts = list(pts_t)
                sliced = True
        ok, reason = path_is_valid(
            list(pts),
            live_grid,
            lethal_threshold=self._cfg.lethal_threshold,
            sample_stride=self._cfg.sample_stride,
        )
        if ok and self._memory_intersects_path(list(pts)):
            return False, 'obstacle_memory_mask'
        # Entry 063 G6 defense-in-depth: FIRE cloud corridor refuse on pick.
        if ok and self._fire_cloud_intersects_corridor(full_pts) is not None:
            return False, 'fire_memory_cloud'

        ever_blocked = (
            int(cand.path_id) in self._ever_blocked_ids
            or int(cand.path_id) in self._block_locus
        )
        if ok and ever_blocked:
            # (a) unsliced full-path memory
            if self._memory_intersects_path(full_pts):
                return False, 'obstacle_memory_mask_full'
            # (b) prior block locus behind forward slice
            locus = self._block_locus.get(cand.path_id)
            if (
                locus is not None
                and sliced
                and self._prior_block_locus_behind_forward_slice(pts, locus)
            ):
                return False, 'prior_block_locus_not_on_forward_slice'
            # (c) unsliced live costmap lethal on abandoned corridor
            ok_full, _full_reason = path_is_valid(
                full_pts,
                live_grid,
                lethal_threshold=self._cfg.lethal_threshold,
                sample_stride=self._cfg.sample_stride,
            )
            if not ok_full:
                return False, 'lethal_full_centreline'
        return ok, reason

    def _sync_route_bank_active(self, path_id: Optional[int]) -> None:
        if self._route_bank is None:
            return
        if path_id is None:
            self._route_bank.active_uid = None
            return
        route = self._route_bank.by_path_id(path_id)
        self._route_bank.active_uid = route.route_uid if route else None

    def _try_evidence_unblock(self, live_grid) -> None:
        """BLOCKED→AVAILABLE when memory is clear and static corridor is free.

        Uses the prior map for geometry when available (Entry 035): live
        costmap_raw still contains StaticLayer trail walls within ~0.5 m of
        mid-corridor barrier centres on bush_trail, which falsely kept Path 1
        BLOCKED after keepout cleared (T5). Memory mask remains the dynamic
        gate; live_grid is the fallback when no prior is loaded.

        Memory gate is scoped to ``_block_locus`` (Entry 035): far-field lidar
        residuals elsewhere on the polyline (outside clear_max after approach)
        must not keep the route BLOCKED once the authorising clear site is
        free — **except** Entry 062 Wave-1:
        * W1-LATCH: refuse while operator/demo corridor latch is armed
        * W1-UNLOCK-DEPTH (NB-M1 A): for ever-demo-latched corridors, also
          require full-path memory clear (defense-in-depth residual keepout)
        Never-demo blocks keep Entry 035 locus-scoped unlock.

        Does not change active_id or preempt NavigateToPose (no auto return).
        Entry 062 W1-RESUME: does **not** bare-promote NO_PATH→FOLLOWING;
        timer/`_pick_next_valid` owns any resume after unlock.
        """
        if self._bank is None:
            return
        grid = live_grid
        grid_src = 'live'
        if self._prior_map is not None:
            try:
                grid = occupancy_msg_to_grid(self._prior_map)
                grid_src = 'prior'
            except Exception:  # noqa: BLE001
                grid = live_grid
                grid_src = 'live'
        unlocked = False
        for cand in self._bank.candidates:
            if cand.state != CandidateState.BLOCKED:
                continue
            # W1-LATCH primary: demo/operator ownership still applies.
            if self._operator_block_latch_active(cand.path_id):
                self.get_logger().info(
                    f'evidence_unblock Path {cand.path_id}: refuse — '
                    'operator_block_latch'
                )
                continue
            # Geometry on the bank centreline (cand.points), not the densified
            # FollowPath route — oriented densify can knife-edge prior walls and
            # falsely keep BLOCKED after the keepout locus clears (Entry 035 T5).
            ok_geom, geom_reason = path_is_valid(
                list(cand.points),
                grid,
                lethal_threshold=self._cfg.lethal_threshold,
                sample_stride=self._cfg.sample_stride,
            )
            locus = self._block_locus.get(cand.path_id)
            if not ok_geom:
                if (
                    locus is not None
                    and not self._mask_occupied_near_xy(
                        locus, self._block_locus_radius_m)
                ):
                    self.get_logger().info(
                        f'evidence_unblock Path {cand.path_id}: mask locus clear '
                        f'but {grid_src} geometry still {geom_reason}'
                    )
                continue
            if locus is not None:
                # Disk + short path stub at the block locus (footprint corridor).
                if self._mask_occupied_near_xy(
                    locus, self._block_locus_radius_m
                ):
                    continue
                if self._memory_intersects_path_near(
                    list(cand.points), locus, self._block_locus_radius_m
                ):
                    continue
            elif self._memory_intersects_path(list(cand.points)):
                continue
            # W1-UNLOCK-DEPTH (NB-M1 A): demo-originated corridors require
            # full-path memory clear even after latch release.
            if int(cand.path_id) in self._ever_demo_latched_ids:
                hit = self._first_memory_hit_on_path(list(cand.points))
                if hit is not None:
                    self.get_logger().info(
                        f'evidence_unblock Path {cand.path_id}: refuse — '
                        f'full-path memory hit at ({hit[0]:.2f},{hit[1]:.2f})'
                    )
                    continue
            # Entry 063 / G6 co-primary B1+B2: typed FIRE cloud corridor refuse.
            # Independent of latch / ever-demo. GUI CLEAR never grants unlock
            # while FIRE centres remain near the unsliced candidate corridor.
            # B1: any cloud hit on unsliced cand.points → refuse.
            # B2: block_source=FIRE refuses until cloud corridor is clear.
            fire_hit = self._fire_cloud_intersects_corridor(list(cand.points))
            if fire_hit is not None:
                self.get_logger().info(
                    f'evidence_unblock Path {cand.path_id}: refuse — '
                    f'fire_memory_cloud near corridor '
                    f'({fire_hit[0]:.2f},{fire_hit[1]:.2f})'
                )
                continue
            if self._bank.mark_available(cand.path_id, 'evidence_clear'):
                unlocked = True
                self._block_locus.pop(cand.path_id, None)
                self._block_source.pop(int(cand.path_id), None)
                rx = ry = float('nan')
                if self._robot_xyyaw is not None:
                    rx, ry = self._robot_xyyaw[0], self._robot_xyyaw[1]
                locus_s = (
                    f'locus=({locus[0]:.2f},{locus[1]:.2f})'
                    if locus is not None else 'locus=None'
                )
                self.get_logger().info(
                    f'ROUTE_MARKED_AVAILABLE Path {cand.path_id} '
                    f'(memory clear near locus + {grid_src} geometry free; '
                    f'{locus_s} latch=0 robot=({rx:.2f},{ry:.2f}); '
                    'not auto-selected)'
                )
        if unlocked:
            self._publish_bank_paths()
            # Entry 062 W1-RESUME: do NOT promote NO_PATH→FOLLOWING solely
            # because something unlocked. Leave NO_PATH_AVAILABLE until the
            # timer / switch path runs _pick_next_valid successfully (or
            # _begin_return_to_start if exhausted). Entry 024: never resume
            # forward once return-to-start / home mode is armed.

    # -- bank construction -------------------------------------------------

    def _build_bank(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        *,
        prebuilt_bank: Optional[PathBank] = None,
    ) -> None:
        self._building = True
        self._set_state(MissionNavState.PLANNING)
        try:
            # Entry 024: capture mission-start home from first plan start.
            self._home_xyyaw = (float(start[0]), float(start[1]), self._home_yaw)
            self.get_logger().info(
                f'Home pose recorded ({self._home_xyyaw[0]:.2f}, '
                f'{self._home_xyyaw[1]:.2f}, yaw={self._home_xyyaw[2]:.2f})'
            )

            grid = None
            if self._prior_map is not None:
                grid = occupancy_msg_to_grid(self._prior_map)
                self.get_logger().info(
                    f'Building path bank from prior_map '
                    f'{grid.width}x{grid.height} @ {grid.resolution}m'
                )
            elif self._live_costmap is not None:
                grid = nav2_costmap_to_grid(self._live_costmap)
                self.get_logger().info('Building path bank from live global costmap')
            else:
                self.get_logger().warn('No prior_map or costmap yet; bank deferred')
                return

            bank = prebuilt_bank
            if bank is None:
                # Entry 081: FollowPath sync seed uses length-primary so Path 1
                # is the shortest valid corridor (matches async StartMission worker).
                seed_cfg = (
                    length_primary_seed_config(self._cfg)
                    if self._use_follow_path
                    else self._cfg
                )
                bank = build_path_bank(
                    grid, start, goal, seed_cfg,
                    footprint_inflate_m=self._footprint_inflate,
                    downsample_factor=4,
                )
            if not bank.candidates:
                self.get_logger().warn('Path bank generated 0 candidates')
                self._bank = bank
                self._set_state(MissionNavState.NO_PATH_AVAILABLE)
                if self._return_home_on_all_blocked:
                    self._begin_return_to_start()
                return

            chosen = bank.select_best_available()
            self._bank = bank
            assert chosen is not None
            for line in (c.summary() for c in bank.candidates):
                self.get_logger().info(f'  {line}')
            self.get_logger().info(
                f'Path bank ready: {len(bank.candidates)} candidates; '
                f'active=Path {chosen.path_id}'
            )
            if self._use_follow_path:
                salt = str(self._owner.mission_id or time.time())
                # Nudge centreline toward free space on the planning grid so
                # RPP footprint collision (local) is not knife-edging walls.
                nudge_grid = None
                if self._prior_map is not None:
                    nudge_grid = occupancy_msg_to_grid(self._prior_map)
                elif self._live_costmap is not None:
                    nudge_grid = nav2_costmap_to_grid(self._live_costmap)
                routes = []
                for c in bank.candidates:
                    pts = list(c.points)
                    if nudge_grid is not None:
                        # A1 centre-seeking: prefer ~0.80 m / operator clearance;
                        # nudge seeks max feasible mid-corridor if too narrow.
                        tgt = self._mission_nudge_target_m()
                        pts = nudge_polyline_away_from_lethal(
                            pts,
                            nudge_grid,
                            target_clearance_m=tgt,
                            max_offset_m=max(1.0, tgt + 0.35),
                        )
                    routes.append(make_route_instance(
                        c.path_id,
                        pts,
                        label=f'path_{c.path_id}',
                        mission_salt=salt,
                    ))
                self._route_bank = RouteBank(mission_salt=salt, routes=routes)
                active_route = self._route_bank.by_path_id(chosen.path_id)
                if active_route is not None:
                    self._route_bank.active_uid = active_route.route_uid
                self.get_logger().info(
                    f'RouteBank bound ({len(routes)} RouteInstances, '
                    f'salt={salt}, active_uid='
                    f'{self._route_bank.active_uid}, nudged='
                    f'{nudge_grid is not None})'
                )
            else:
                self._route_bank = None
            self._publish_bank_paths()
            self._bank_ready_at = time.monotonic()
            self._abort_resend_used = False
            self._switch_grace_until = 0.0
            self._nav_mode = 'mission'
            self._set_state(MissionNavState.FOLLOWING_PATH)
            if self._use_follow_path and self._route_bank is not None:
                active_route = self._route_bank.active()
                if active_route is not None:
                    # Do NOT clear local here — Entry 028 showed clear→rematch
                    # races; nudged geometry + failure_tolerance cover first
                    # FollowPath. Clear remains for intentional path switch.
                    self._send_follow_path(active_route)
        finally:
            self._building = False

    def _begin_return_to_start(self) -> None:
        """Entry 024 / Phase Two Wave 1a: fresh home plan after bank exhaustion.

        D1 handoff (Wave-1): cancel mission FollowPath + bump generation + clear
        switch **before** ComputePath request. Geometry is ComputePathToPose
        current→home only — reverse-splice recovery is forbidden (deleted).
        """
        if self._state in (
            MissionNavState.RETURNING_TO_START,
            MissionNavState.RETURNED_TO_START,
            MissionNavState.NO_SAFE_ROUTE,
        ) and self._nav_mode == 'home' and self._home_plan_pending:
            return
        if self._home_xyyaw is None:
            self.get_logger().error(
                'ALL_BLOCKED but no home pose recorded; cannot return to start'
            )
            self._raise_no_safe_route('no_home_pose')
            return
        # TEST-ONLY: force home plan/validate failure for N6 live evidence.
        # Re-read each call so the harness can arm the flag mid-mission.
        force_unreachable = False
        try:
            raw = self.get_parameter('test_force_unreachable_home').value
            if isinstance(raw, str):
                force_unreachable = raw.strip().lower() in (
                    '1', 'true', 'yes', 'on')
            else:
                force_unreachable = bool(raw)
        except Exception:  # noqa: BLE001
            force_unreachable = False
        if force_unreachable:
            # Far outside bush_trail free space — planner / footprint must fail.
            self._home_xyyaw = (80.0, 80.0, self._home_yaw)
            self.get_logger().warn(
                'TEST test_force_unreachable_home=true → home overridden to '
                f'({self._home_xyyaw[0]:.1f},{self._home_xyyaw[1]:.1f})'
            )
        hx, hy, hyaw = self._home_xyyaw
        self.get_logger().warn(
            f'ALL_BLOCKED → RETURNING_TO_START home=({hx:.2f}, {hy:.2f}) '
            '(fresh ComputePathToPose; no reverse-splice)'
        )
        # D1: cancel mission FP / NTP BEFORE plan request (always on).
        self._cancel_nav()
        self._owner.bump_generation()
        self._clear_switch_pending('begin_return_to_start')
        # A.3 latch — set only after early guards / home pose check succeeded.
        self._home_attempted = True
        self._home_plan_attempt_count = 0
        self._home_fp_route_uid = None
        self._home_route = None
        self._last_home_footprint_reason = ''
        # New return epoch: clear prior fail detail + via / navfn scratch.
        self._last_no_safe_detail = ''
        self._last_home_navfn_pts = []
        self._last_home_c1_fail_pts = []
        self._home_via_phase = None
        self._home_via_generation = -1
        self._home_via_v_star = None
        self._home_via_leg1_pts = []
        self._home_via_candidate_queue = []
        if self._robot_xyyaw is not None:
            self._dist_home_at_return_arm = math.hypot(
                self._robot_xyyaw[0] - hx, self._robot_xyyaw[1] - hy)
        else:
            self._dist_home_at_return_arm = -1.0
        # Owner ↔ bank sync (Phase Two §3.4).
        self._nav_mode = 'home'
        self._abort_resend_used = False
        self._stuck_recoveries = 0
        self._stuck_recover_until = 0.0
        self._home_entered_at = time.monotonic()
        self._last_progress_t = time.monotonic()
        if self._robot_xyyaw is not None:
            self._last_progress_xy = (
                self._robot_xyyaw[0], self._robot_xyyaw[1])
        self._owner.phase = MissionPhase.RETURNING_TO_START
        self._owner.command_authority = CommandAuthority.NAV
        self._owner.stop_reason = ''
        self._owner.detail = 'HOME_PLAN_PENDING'
        self._publish_authority_now()
        self._set_state(MissionNavState.RETURNING_TO_START)
        # Entry 063 A3: clear stale mission FollowPath ref during home-plan gap.
        self._clear_active_path()
        if self._use_follow_path:
            cur = self._robot_xyyaw
            if cur is None and self._bank is not None:
                cur = (self._bank.start[0], self._bank.start[1], self._home_yaw)
            if cur is None:
                self.get_logger().error(
                    'RETURNING_TO_START: no current pose; NO_SAFE_ROUTE'
                )
                self._raise_no_safe_route('home_no_pose')
                return
            # C2: readiness failure counts toward max=2 budget.
            if not self._request_fresh_home_plan():
                if self._home_plan_attempt_count < 2:
                    if self._request_fresh_home_plan():
                        self._owner.detail = 'RETURNING_TO_START'
                        return
                self.get_logger().error(
                    'RETURNING_TO_START: fresh home ComputePath unavailable'
                )
                self._raise_no_safe_route('home_plan_unavailable')
                return
            self._owner.detail = 'RETURNING_TO_START'
            return
        # Legacy NTP: same preempt pattern as path switch (Entry 023).
        if self._goal_handle is not None:
            self._ignore_goal_results += 1
        self._send_goal(hx, hy, hyaw)

    def _request_fresh_home_plan(self) -> bool:
        """Async ComputePathToPose current pose → _home_xyyaw on live costmap.

        Reverse-splice / densified straight fallback is intentionally absent.
        C2: increments ``_home_plan_attempt_count`` (max 2 while nav_mode=home).
        Entry 081: gate on ``_home_plan_readiness_ok`` — do **not** require
        ``_planning_start`` (mission-seed scratch unrelated to home).
        """
        if self._nav_mode == 'home' and self._home_plan_attempt_count >= 2:
            self.get_logger().warn(
                'Home plan attempt budget exhausted (max=2); not requesting'
            )
            return False
        ok, why = self._home_plan_readiness_ok()
        if not ok:
            # Preserve C2 budget consumption when the ComputePath server is down.
            if (
                self._nav_mode == 'home'
                and why == 'home_plan_unavailable'
                and self._compute_path_client is not None
                and not self._compute_path_client.server_is_ready()
            ):
                self._home_plan_attempt_count += 1
                self.get_logger().warn(
                    'compute_path_to_pose not ready for fresh home plan '
                    f'(attempt={self._home_plan_attempt_count}/2)')
            else:
                self.get_logger().warn(
                    f'Home plan readiness failed ({why}); not requesting'
                )
            return False
        rx, ry, ryaw = self._robot_xyyaw
        hx, hy, hyaw = self._home_xyyaw
        goal = ComputePathToPose.Goal()
        goal.planner_id = 'GridBased'
        goal.use_start = True
        goal.start.header.frame_id = self._map_frame
        goal.start.header.stamp = self.get_clock().now().to_msg()
        goal.start.pose.position.x = float(rx)
        goal.start.pose.position.y = float(ry)
        goal.start.pose.orientation.z = math.sin(0.5 * ryaw)
        goal.start.pose.orientation.w = math.cos(0.5 * ryaw)
        goal.goal.header = goal.start.header
        goal.goal.pose.position.x = float(hx)
        goal.goal.pose.position.y = float(hy)
        goal.goal.pose.orientation.z = math.sin(0.5 * hyaw)
        goal.goal.pose.orientation.w = math.cos(0.5 * hyaw)
        gen = self._owner.generation
        if self._nav_mode == 'home':
            self._home_plan_attempt_count += 1
        self._home_plan_pending = True
        fut = self._compute_path_client.send_goal_async(goal)
        fut.add_done_callback(
            lambda f, g=gen: self._home_plan_goal_response(f, g))
        self.get_logger().info(
            f'Fresh home ComputePathToPose → ({hx:.2f},{hy:.2f}) '
            f'from ({rx:.2f},{ry:.2f}) attempt={self._home_plan_attempt_count}/2'
        )
        return True

    def _retry_home_plan_or_raise(self, detail: str) -> None:
        """C2 attempt-2 branch table (R4 §1.5.1).

        Footprint / C1 miss → via async FSM (set count=2 once; never
        ``_request_fresh_home_plan`` for via legs). Planner-class fail →
        readiness wait + one identical fresh plan. Else honest NO_SAFE.
        """
        if self._nav_mode != 'home':
            self._raise_no_safe_route(detail)
            return
        if self._home_plan_attempt_count >= 2:
            self._raise_no_safe_route(detail)
            return
        token = (detail or '').strip()
        footprint_class = (
            token == 'home_footprint_invalid'
            or token.startswith('home_footprint_invalid')
        )
        plan_class = token.startswith('home_plan_') or token in (
            'home_plan_unavailable',
            'home_plan_goal_failed',
            'home_plan_rejected',
            'home_plan_result_failed',
            'home_plan_empty',
        )
        if footprint_class:
            # Attempt-2 via: set count=2 once; do not call _request_fresh_home_plan.
            self._home_plan_attempt_count = 2
            self.get_logger().warn(
                f'Home footprint fail ({token}) → via FSM '
                f'(attempt={self._home_plan_attempt_count}/2)'
            )
            self._home_via_fsm_enter()
            return
        if plan_class:
            ok, why = self._home_plan_readiness_ok()
            if not ok:
                # ≤0.50 s readiness wait when costmap stale / pose missing.
                t_end = time.monotonic() + 0.50
                while time.monotonic() < t_end:
                    ok, why = self._home_plan_readiness_ok()
                    if ok:
                        break
                    time.sleep(0.05)
            if self._request_fresh_home_plan():
                self.get_logger().warn(
                    f'Home plan retry after {token} '
                    f'(attempt={self._home_plan_attempt_count}/2 '
                    f'ready={ok}/{why})'
                )
                return
            self._raise_no_safe_route(token or 'home_plan_unavailable')
            return
        self._raise_no_safe_route(detail)

    def _home_plan_readiness_ok(self) -> Tuple[bool, str]:
        """§1.6 readiness gate before home ComputePath."""
        if self._robot_xyyaw is None:
            return False, 'home_no_pose'
        if self._home_xyyaw is None:
            return False, 'no_home_pose'
        if self._compute_path_client is None:
            return False, 'home_plan_unavailable'
        if not self._compute_path_client.server_is_ready():
            return False, 'home_plan_unavailable'
        if self._live_costmap is None:
            return False, 'home_costmap_missing'
        try:
            stamp = self._live_costmap.header.stamp
            now = self.get_clock().now()
            age = (now.nanoseconds - rclpy.time.Time.from_msg(stamp).nanoseconds) * 1e-9
            if age > 1.0:
                return False, 'home_costmap_stale'
        except Exception:  # noqa: BLE001
            return False, 'home_costmap_stale'
        return True, 'ok'

    def _home_via_invalidate(self) -> None:
        """Clear via FSM scratch (phase/gen/pending peers use raise path)."""
        self._home_via_phase = None
        self._home_via_generation = -1
        self._home_via_v_star = None
        self._home_via_leg1_pts = []
        self._home_via_leg_started_at = 0.0

    def _home_via_leg_callback_ok(self, expected_phase: str) -> bool:
        """R4 full via fence — gen equality + phase + home + not terminal.

        Normative: ``_home_via_generation == _owner.generation`` (no soft
        alternate). ASSEMBLE entry uses expected_phase ``LEG2_PENDING``
        (pre-transition), then sets phase to ASSEMBLE.
        """
        if self._home_via_generation != self._owner.generation:
            return False
        if self._home_via_phase != expected_phase:
            return False
        if self._nav_mode != 'home':
            return False
        if self._state == MissionNavState.NO_SAFE_ROUTE:
            return False
        if self._owner.phase == MissionPhase.STOPPED:
            return False
        if SafetyInhibit.ALL_BLOCKED_NO_HOME in self._owner.active_inhibits:
            return False
        return True

    def _home_via_fsm_enter(self) -> None:
        """Attempt-2 via: SELECT V* sync, then async LEG1 (R4 §1.5.1)."""
        self._home_via_invalidate()
        self._home_via_generation = int(self._owner.generation)
        self._home_via_phase = 'SELECT'
        self._home_plan_pending = True
        v_star = self._home_via_select_v_star()
        if v_star is None:
            self.get_logger().error(
                'Via SELECT: no free lateral V* → NO_SAFE '
                f'(c1={self._last_home_footprint_reason or "none"})'
            )
            self._raise_no_safe_route('home_footprint_invalid')
            return
        self._home_via_v_star = v_star
        self.get_logger().warn(
            f'Via SELECT V*=({v_star[0]:.2f},{v_star[1]:.2f}) → LEG1'
        )
        if not self._home_via_issue_leg(
            start_xyyaw=self._robot_xyyaw,
            goal_xy=v_star,
            phase='LEG1_PENDING',
        ):
            self._raise_no_safe_route('home_footprint_invalid')

    def _home_via_select_v_star(self) -> Optional[Tuple[float, float]]:
        """§1.4 steps 1–4: hit cell → lateral candidates → best clearance.

        ``lethal_at_<i>`` indexes the densified C1 forward-slice
        (``_last_home_c1_fail_pts``), not raw NavFn length.
        """
        if self._live_costmap is None or self._robot_xyyaw is None:
            return None
        if self._home_xyyaw is None:
            return None
        grid = nav2_costmap_to_grid(self._live_costmap)
        c1_pts = list(self._last_home_c1_fail_pts or [])
        navfn = list(self._last_home_navfn_pts or [])
        poly = c1_pts if len(c1_pts) >= 2 else navfn
        reason = self._last_home_footprint_reason or ''
        p_hit: Optional[Tuple[float, float]] = None
        tangent = (1.0, 0.0)
        if 'lethal_at_' in reason and poly:
            try:
                idx_s = reason.split('lethal_at_', 1)[1].split('_', 1)[0]
                i = int(idx_s)
                i = max(0, min(i, len(poly) - 1))
                p_hit = poly[i]
                if i + 1 < len(poly):
                    dx = poly[i + 1][0] - poly[i][0]
                    dy = poly[i + 1][1] - poly[i][1]
                elif i > 0:
                    dx = poly[i][0] - poly[i - 1][0]
                    dy = poly[i][1] - poly[i - 1][1]
                else:
                    dx, dy = 1.0, 0.0
                n = math.hypot(dx, dy) or 1.0
                tangent = (dx / n, dy / n)
            except (ValueError, IndexError):
                p_hit = None
        if p_hit is None:
            # Planner-fail / no index: first lethal along robot→home ray @0.25 m.
            rx, ry, _ = self._robot_xyyaw
            hx, hy, _ = self._home_xyyaw
            dx, dy = hx - rx, hy - ry
            seg = math.hypot(dx, dy)
            if seg < 0.25:
                return None
            ux, uy = dx / seg, dy / seg
            s = 0.0
            while s <= seg:
                x, y = rx + ux * s, ry + uy * s
                idx = grid.world_to_grid(x, y)
                if idx is not None:
                    cost = int(grid.costs[idx[1], idx[0]])
                    if cost >= LETHAL and cost != 255:
                        p_hit = (x, y)
                        tangent = (ux, uy)
                        break
                s += 0.25
            if p_hit is None:
                # Midpoint fallback for hit localization only.
                p_hit = (rx + 0.5 * dx, ry + 0.5 * dy)
                tangent = (ux, uy)
        nx, ny = -tangent[1], tangent[0]
        pad_need = FOOTPRINT_PAD + self._footprint_inflate
        seats: List[Tuple[float, float]] = list(self._demo_obstacle_poses or [])
        if not seats:
            seats = [(-1.3, -11.5), (-5.1, 21.1)]
        candidates: List[Tuple[float, Tuple[float, float]]] = []
        for L in (1.0, 1.5, 2.0):
            for sign in (1.0, -1.0):
                vx = p_hit[0] + sign * L * nx
                vy = p_hit[1] + sign * L * ny
                idx = grid.world_to_grid(vx, vy)
                if idx is None:
                    continue
                cost = int(grid.costs[idx[1], idx[0]])
                if cost >= LETHAL and cost != 255:
                    continue
                clr = point_clearance(vx, vy, grid)
                if clr < pad_need:
                    continue
                disk_ok = all(
                    math.hypot(vx - sx, vy - sy) >= 2.5 for sx, sy in seats
                )
                score = clr + (10.0 if disk_ok else 0.0)
                candidates.append((score, (vx, vy)))
        if not candidates:
            self._home_via_candidate_queue = []
            return None
        candidates.sort(key=lambda c: c[0], reverse=True)
        # Queue alternates for same attempt-2 if ASSEMBLE C1 rejects first V*.
        self._home_via_candidate_queue = [xy for _, xy in candidates[1:]]
        return candidates[0][1]

    def _home_via_issue_leg(
        self,
        *,
        start_xyyaw: Optional[Tuple[float, float, float]],
        goal_xy: Tuple[float, float],
        phase: str,
    ) -> bool:
        """Async ComputePath for one via leg — never ``_request_fresh_home_plan``."""
        if (
            self._compute_path_client is None
            or start_xyyaw is None
            or self._home_xyyaw is None
        ):
            return False
        if not self._compute_path_client.server_is_ready():
            return False
        sx, sy, syaw = start_xyyaw
        gx, gy = goal_xy
        # Goal yaw: toward home for leg1 (via), home yaw for leg2.
        if phase == 'LEG2_PENDING':
            gyaw = float(self._home_xyyaw[2])
        else:
            hx, hy, _ = self._home_xyyaw
            gyaw = math.atan2(hy - gy, hx - gx)
        goal = ComputePathToPose.Goal()
        goal.planner_id = 'GridBased'
        goal.use_start = True
        goal.start.header.frame_id = self._map_frame
        goal.start.header.stamp = self.get_clock().now().to_msg()
        goal.start.pose.position.x = float(sx)
        goal.start.pose.position.y = float(sy)
        goal.start.pose.orientation.z = math.sin(0.5 * syaw)
        goal.start.pose.orientation.w = math.cos(0.5 * syaw)
        goal.goal.header = goal.start.header
        goal.goal.pose.position.x = float(gx)
        goal.goal.pose.position.y = float(gy)
        goal.goal.pose.orientation.z = math.sin(0.5 * gyaw)
        goal.goal.pose.orientation.w = math.cos(0.5 * gyaw)
        self._home_via_phase = phase
        self._home_via_leg_started_at = time.monotonic()
        self._home_plan_pending = True
        # Capture generation at issue — fence requires equality with owner gen.
        via_gen = int(self._home_via_generation)
        fut = self._compute_path_client.send_goal_async(goal)
        fut.add_done_callback(
            lambda f, p=phase, g=via_gen: self._home_via_leg_goal_response(f, p, g)
        )
        self.get_logger().info(
            f'Via {phase} ComputePath ({sx:.2f},{sy:.2f})→({gx:.2f},{gy:.2f}) '
            f'via_gen={via_gen}'
        )
        return True

    def _home_via_leg_goal_response(self, future, phase: str, via_gen: int) -> None:
        if via_gen != self._home_via_generation:
            return
        if not self._home_via_leg_callback_ok(phase):
            return
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Via {phase} goal failed: {exc}')
            self._raise_no_safe_route('home_footprint_invalid')
            return
        if handle is None or not handle.accepted:
            self.get_logger().warn(f'Via {phase} ComputePath rejected')
            self._raise_no_safe_route('home_footprint_invalid')
            return
        result_fut = handle.get_result_async()
        result_fut.add_done_callback(
            lambda f, p=phase, g=via_gen: self._home_via_leg_result(f, p, g)
        )

    def _home_via_leg_result(self, future, phase: str, via_gen: int) -> None:
        if via_gen != self._home_via_generation:
            return
        if not self._home_via_leg_callback_ok(phase):
            return
        try:
            wrapped = future.result()
            status = wrapped.status
            result = wrapped.result
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Via {phase} result failed: {exc}')
            self._raise_no_safe_route('home_footprint_invalid')
            return
        if status != GoalStatus.STATUS_SUCCEEDED or result is None:
            self.get_logger().warn(f'Via {phase} status={status}')
            self._raise_no_safe_route('home_footprint_invalid')
            return
        path_msg = result.path
        if path_msg is None or len(path_msg.poses) < 2:
            self.get_logger().warn(f'Via {phase} empty path')
            self._raise_no_safe_route('home_footprint_invalid')
            return
        pts = [
            (float(ps.pose.position.x), float(ps.pose.position.y))
            for ps in path_msg.poses
        ]
        if phase == 'LEG1_PENDING':
            self._home_via_leg1_pts = pts
            v = self._home_via_v_star
            if v is None or self._home_xyyaw is None:
                self._raise_no_safe_route('home_footprint_invalid')
                return
            # Start leg2 at V* with yaw toward home.
            hx, hy, hyaw = self._home_xyyaw
            start_yaw = math.atan2(hy - v[1], hx - v[0])
            if not self._home_via_issue_leg(
                start_xyyaw=(v[0], v[1], start_yaw),
                goal_xy=(hx, hy),
                phase='LEG2_PENDING',
            ):
                self._raise_no_safe_route('home_footprint_invalid')
            return
        if phase == 'LEG2_PENDING':
            # R4 Minor: ASSEMBLE entry fence with expected_phase=LEG2_PENDING
            # (pre-transition), then set ASSEMBLE.
            if not self._home_via_leg_callback_ok('LEG2_PENDING'):
                return
            self._home_via_phase = 'ASSEMBLE'
            self._home_via_assemble_and_arm(pts)
            return
        self._raise_no_safe_route('home_footprint_invalid')

    def _home_via_assemble_and_arm(self, leg2_pts: List[Tuple[float, float]]) -> None:
        """Sync densify/nudge/C1 after both async legs; arm FollowPath or raise."""
        leg1 = list(self._home_via_leg1_pts or [])
        if len(leg1) < 2 or len(leg2_pts) < 2:
            self._raise_no_safe_route('home_footprint_invalid')
            return
        pts = list(leg1)
        if math.hypot(pts[-1][0] - leg2_pts[0][0], pts[-1][1] - leg2_pts[0][1]) < 0.05:
            pts.extend(leg2_pts[1:])
        else:
            pts.extend(leg2_pts)
        if self._home_xyyaw is not None:
            hx, hy, _ = self._home_xyyaw
            if math.hypot(pts[-1][0] - hx, pts[-1][1] - hy) > 0.05:
                pts.append((hx, hy))
        salt = str(self._owner.mission_id or time.time())
        home_route = make_route_instance(
            0, pts, label='return_home_via', densify_m=0.10, mission_salt=salt)
        if self._live_costmap is not None:
            live = nav2_costmap_to_grid(self._live_costmap)
            # Attempt-2 fixed nudge params (no further silent pad change).
            nudged_pts = nudge_polyline_away_from_lethal(
                list(home_route.points),
                live,
                target_clearance_m=1.0,
                max_offset_m=2.0,
            )
            home_route = make_route_instance(
                0,
                nudged_pts,
                label='return_home_via',
                densify_m=1.0e6,
                mission_salt=salt,
            )
        ok, reason = self._home_route_c1_oriented_valid(home_route)
        if not ok:
            v = self._home_via_v_star
            self.get_logger().error(
                f'Via ASSEMBLE C1 fail ({reason}) via='
                f'{None if v is None else f"({v[0]:.2f},{v[1]:.2f})"}'
            )
            # Same attempt-2: try next queued V* (still via; not identical replan).
            if self._home_via_candidate_queue:
                nxt = self._home_via_candidate_queue.pop(0)
                self._home_via_v_star = nxt
                self._home_via_leg1_pts = []
                self.get_logger().warn(
                    f'Via ASSEMBLE miss → retry next V*=({nxt[0]:.2f},{nxt[1]:.2f})'
                )
                if self._home_via_issue_leg(
                    start_xyyaw=self._robot_xyyaw,
                    goal_xy=nxt,
                    phase='LEG1_PENDING',
                ):
                    return
            self._raise_no_safe_route('home_footprint_invalid')
            return
        # R4 Minor 4: re-check terminal/fence immediately before FollowPath arm.
        if (
            self._state == MissionNavState.NO_SAFE_ROUTE
            or self._owner.phase == MissionPhase.STOPPED
            or SafetyInhibit.ALL_BLOCKED_NO_HOME in self._owner.active_inhibits
            or self._nav_mode != 'home'
            or self._home_via_generation != self._owner.generation
            or self._home_via_phase != 'ASSEMBLE'
        ):
            self.get_logger().warn(
                'Via ASSEMBLE: terminal/fence blocked FollowPath arm (no-op)'
            )
            return
        v = self._home_via_v_star
        self._home_route = home_route
        self._home_fp_route_uid = home_route.route_uid
        self._home_plan_attempt_count = 0
        self._home_plan_pending = False
        self._last_home_navfn_pts = []
        self._home_via_invalidate()
        self._publish_active_route_path(home_route)
        self._cancel_follow_path()
        self._clear_local_costmap()
        if self._owner.phase != MissionPhase.RETURNING_TO_START:
            self._owner.phase = MissionPhase.RETURNING_TO_START
        if self._owner.command_authority != CommandAuthority.NAV:
            self._owner.command_authority = CommandAuthority.NAV
            self._publish_authority_now()
        self._owner.detail = 'RETURNING_TO_START'
        self._send_follow_path(home_route, allow_navfn_bridge=False)
        self.get_logger().info(
            f'Via home FollowPath armed poses={len(home_route.points)} '
            f'route_uid={home_route.route_uid} '
            f'via={None if v is None else f"({v[0]:.2f},{v[1]:.2f})"}'
        )

    def _home_plan_goal_response(self, future, generation: int) -> None:
        # Watch-out: clear pending only on generation-matched paths so a newer
        # in-flight home plan is not unmarked after §G bump_generation.
        if generation != self._owner.generation:
            return
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self._home_plan_pending = False
            self.get_logger().warn(f'Home plan goal failed: {exc}')
            self._retry_home_plan_or_raise('home_plan_goal_failed')
            return
        if handle is None or not handle.accepted:
            self._home_plan_pending = False
            self.get_logger().warn('Home ComputePathToPose rejected')
            self._retry_home_plan_or_raise('home_plan_rejected')
            return
        result_fut = handle.get_result_async()
        result_fut.add_done_callback(
            lambda f, g=generation: self._home_plan_result(f, g))

    def _home_plan_result(self, future, generation: int) -> None:
        # Watch-out: do not clear pending before generation match.
        if generation != self._owner.generation:
            return
        self._home_plan_pending = False
        try:
            wrapped = future.result()
            status = wrapped.status
            result = wrapped.result
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Home plan result failed: {exc}')
            self._retry_home_plan_or_raise('home_plan_result_failed')
            return
        if status != GoalStatus.STATUS_SUCCEEDED or result is None:
            self.get_logger().warn(
                f'Home ComputePathToPose status={status}; retry/NO_SAFE')
            self._retry_home_plan_or_raise(f'home_plan_status_{status}')
            return
        path_msg = result.path
        if path_msg is None or len(path_msg.poses) < 2:
            self.get_logger().warn('Home plan returned empty path')
            self._retry_home_plan_or_raise('home_plan_empty')
            return
        pts = [
            (float(ps.pose.position.x), float(ps.pose.position.y))
            for ps in path_msg.poses
        ]
        # Persist raw NavFn poses before densify/C1 (W1a via input; clear on raise).
        self._last_home_navfn_pts = list(pts)
        home_route = self._build_home_route_from_points(pts)
        if home_route is None:
            self.get_logger().error(
                'RETURNING_TO_START: home RouteInstance failed validation '
                f'({self._last_home_footprint_reason or "unknown"})'
            )
            self._retry_home_plan_or_raise('home_footprint_invalid')
            return
        self._home_route = home_route
        self._home_fp_route_uid = home_route.route_uid
        self._home_plan_attempt_count = 0  # C2 reset on successful home FP arm
        self._publish_active_route_path(home_route)
        # D1 already cancelled mission FP at begin-return; cancel again for safety.
        self._cancel_follow_path()
        self._clear_local_costmap()
        # Re-affirm owner sync before FollowPath submit.
        if self._owner.phase != MissionPhase.RETURNING_TO_START:
            self._owner.phase = MissionPhase.RETURNING_TO_START
        if self._owner.command_authority != CommandAuthority.NAV:
            self._owner.command_authority = CommandAuthority.NAV
            self._publish_authority_now()
        self._owner.detail = 'RETURNING_TO_START'
        self._send_follow_path(home_route, allow_navfn_bridge=False)
        self.get_logger().info(
            f'Home FollowPath armed poses={len(home_route.points)} '
            f'route_uid={home_route.route_uid} '
            f'dist_home_at_return_arm={self._dist_home_at_return_arm:.2f}'
        )

    def _home_route_c1_oriented_valid(
        self, route: RouteInstance
    ) -> Tuple[bool, str]:
        """C1: oriented-accept the same forward polyline FollowPath will drive.

        Uses ``_route_forward_from_robot`` (nearest+1 under robot). Mid-path
        lethals on the forward slice still reject. No mark_exclude / pad weaken.
        """
        # W1a / §1.7: fail-closed when costmap missing (was vacuous accept).
        if self._live_costmap is None:
            return False, 'costmap_missing'
        live = nav2_costmap_to_grid(self._live_costmap)
        pts, yaws = self._route_forward_from_robot(route)
        if len(pts) < 2:
            pts, yaws = route.points, route.yaw_rad
        return oriented_footprint_path_valid(
            pts,
            yaws,
            live,
            lethal_threshold=self._cfg.lethal_threshold,
        )

    def _build_home_route_from_points(
        self, pts: List[Tuple[float, float]]
    ) -> Optional[RouteInstance]:
        """Densify planned points → RouteInstance; C1 forward-slice oriented accept.

        NavFn is centreline-only; mission RouteBank bind already runs
        ``nudge_polyline_away_from_lethal`` so FollowPath is not knife-edging
        bush_trail walls (Entry 034). Fresh home ComputePath skipped that
        nudge → C1 oriented+pad rejected mid/late path (GUI-DB dual-block
        ``lethal_at_28x`` near home approach) while a corridor-centred
        polyline would still be safe. Apply the same nudge on the live
        global costmap **before** C1. Sealed mid-path keepout still fails
        C1 honestly (no pad/lethal/mark_exclude weaken).

        No reverse-splice. Used by fresh-home ComputePath callback and units.
        """
        if len(pts) < 2:
            return None
        # Ensure exact home endpoint if planner undershot.
        if self._home_xyyaw is not None:
            hx, hy, _ = self._home_xyyaw
            if math.hypot(pts[-1][0] - hx, pts[-1][1] - hy) > 0.05:
                pts = list(pts) + [(hx, hy)]
        pts = list(pts)
        salt = str(self._owner.mission_id or time.time())
        # Densify first, then nudge densified vertices (Entry 034 helper), then
        # rebuild yaw **without** re-densifying. Re-densify after nudge
        # re-subdivides frozen-endpoint→nudged-mid chords and recreates
        # steep-yaw oriented lethals (GUI-DB lethal_at_28x class).
        home_route = make_route_instance(
            0, pts, label='return_home', densify_m=0.10, mission_salt=salt)
        nudged = False
        if self._live_costmap is not None:
            live = nav2_costmap_to_grid(self._live_costmap)
            # Mission RouteBank bind uses ~0.65/1.0 for centreline+RPP.
            # Home C1 is oriented+pad: Path-B mid ~(-13.7,15.7) still
            # lethal_at_* after 0.65/1.0 (GUI-DB dual-block). Escalate to
            # body-scale clearance; sealed mid-path still fails C1 honestly.
            tgt = max(0.85, self._footprint_inflate + 0.35)
            max_off = max(1.5, tgt + 0.65)
            nudged_pts = nudge_polyline_away_from_lethal(
                list(home_route.points),
                live,
                target_clearance_m=tgt,
                max_offset_m=max_off,
            )
            # densify_m huge → copy vertices + refresh yaw/fingerprint only.
            home_route = make_route_instance(
                0,
                nudged_pts,
                label='return_home',
                densify_m=1.0e6,
                mission_salt=salt,
            )
            nudged = True
            ok, reason = self._home_route_c1_oriented_valid(home_route)
            # One escalated re-nudge if first pass still knife-edges.
            if not ok and 'lethal' in (reason or ''):
                tgt2 = max(1.0, tgt + 0.15)
                max_off2 = max(2.0, max_off + 0.5)
                nudged_pts = nudge_polyline_away_from_lethal(
                    list(home_route.points),
                    live,
                    target_clearance_m=tgt2,
                    max_offset_m=max_off2,
                )
                home_route = make_route_instance(
                    0,
                    nudged_pts,
                    label='return_home',
                    densify_m=1.0e6,
                    mission_salt=salt,
                )
                self.get_logger().info(
                    f'Home route re-nudge after C1 miss ({reason}) '
                    f'tgt={tgt2:.2f} max_off={max_off2:.2f}'
                )
        ok, reason = self._home_route_c1_oriented_valid(home_route)
        if not ok:
            # NavFn knife-edge can remain C1-lethal after nudge while the
            # frozen mission corridor homeward (robot→start) is still safe.
            # Prefer that bank homeward polyline over honest NO_SAFE when it
            # passes the same C1 gate — not a traveled-path reverse-splice.
            alt = self._bank_homeward_route_if_c1_valid(salt)
            if alt is not None:
                self.get_logger().warn(
                    f'Home NavFn C1 miss ({reason}); using bank homeward '
                    f'route_uid={alt.route_uid} poses={len(alt.points)}'
                )
                self._last_home_footprint_reason = ''
                return alt
            self._last_home_footprint_reason = reason
            fwd, _ = self._route_forward_from_robot(home_route)
            if not fwd:
                fwd = list(home_route.points)
            # Persist densified forward slice so via SELECT indexes lethal_at_<i>
            # against the same polyline C1 used (not raw NavFn length).
            self._last_home_c1_fail_pts = list(fwd)
            keepout_hit = 'lethal' in (reason or '')
            hit_xy = ''
            if reason and 'lethal_at_' in reason:
                try:
                    idx_s = reason.split('lethal_at_', 1)[1].split('_', 1)[0]
                    hi = int(idx_s)
                    if 0 <= hi < len(fwd):
                        hit_xy = f' xy=({fwd[hi][0]:.2f},{fwd[hi][1]:.2f})'
                except (ValueError, IndexError, TypeError):
                    hit_xy = ''
            self.get_logger().warn(
                f'Home route footprint-invalid ({reason}); rejecting '
                f'(keepout_hit={keepout_hit}; C1 forward-slice; '
                f'nudged={nudged}{hit_xy})'
            )
            return None
        self._last_home_footprint_reason = ''
        self._last_home_c1_fail_pts = []
        return home_route

    def _bank_homeward_route_if_c1_valid(
        self, mission_salt: str
    ) -> Optional[RouteInstance]:
        """C1-gated homeward slice of a frozen mission RouteInstance.

        Walks each bank route from the vertex nearest the robot back toward
        the route start (recorded near home). Same oriented C1 as fresh
        ComputePath accept. Returns None when no corridor passes.
        """
        if self._route_bank is None or self._robot_xyyaw is None:
            return None
        if self._home_xyyaw is None:
            return None
        rx, ry, _ = self._robot_xyyaw
        hx, hy, _ = self._home_xyyaw
        live = None
        if self._live_costmap is not None:
            live = nav2_costmap_to_grid(self._live_costmap)
        # Prefer the active / last-blocked corridor (usually Path B at dual-block).
        routes = list(self._route_bank.routes)
        active = self._route_bank.active()
        if active is not None:
            routes = [active] + [r for r in routes if r.route_uid != active.route_uid]
        for route in routes:
            if len(route.points) < 2:
                continue
            # Entry 081: progress-aware nearest — mid-B exclusive, not stem.
            best_i = self._nearest_route_index_progress_aware(
                route.points, rx, ry)
            if best_i < 1:
                continue
            # Start→robot prefix, then reverse so travel is robot→start/home.
            prefix = list(route.points[: best_i + 1])
            homeward = list(reversed(prefix))
            # Snap endpoints to live robot / recorded home.
            homeward[0] = (float(rx), float(ry))
            if math.hypot(homeward[-1][0] - hx, homeward[-1][1] - hy) > 0.05:
                homeward.append((float(hx), float(hy)))
            if live is not None:
                tgt = max(0.85, self._footprint_inflate + 0.35)
                homeward = nudge_polyline_away_from_lethal(
                    homeward,
                    live,
                    target_clearance_m=tgt,
                    max_offset_m=max(1.5, tgt + 0.65),
                )
            cand = make_route_instance(
                0,
                homeward,
                label='return_home_bank',
                densify_m=0.10,
                mission_salt=mission_salt + '|bank_homeward',
            )
            # Yaw refresh only after densify+nudge already applied above when
            # live map present; densify_m=0.10 is fine for sparse prefix.
            ok, _reason = self._home_route_c1_oriented_valid(cand)
            if ok:
                return cand
        return None

    def _raise_no_safe_route(self, detail: str) -> None:
        """§3.2 terminal recovery — must not remain FOLLOWING_PATH.

        Sequence: cancel FP → command_authority NONE → raise
        ALL_BLOCKED_NO_HOME → owner STOPPED → bank/status NO_SAFE_ROUTE →
        clear active_path.

        H6: persist ``last_no_safe_detail`` and set owner.detail to
        ``NO_SAFE_ROUTE:<token>`` (not bare NO_SAFE_ROUTE).

        Raise-side via invalidation (R4): phase=None, gen=-1, pending=False,
        drop V*/leg1, clear navfn_pts. First raise bumps generation only via
        ``raise_inhibit`` (body has no local ``bump_generation``); re-raise
        else-branch does not bump — fence + sentinel remain mandatory.
        """
        token = (detail or '').strip() or 'unknown'
        self._last_no_safe_detail = token
        self._cancel_nav()
        self._pending_follow_seed = False
        self._pending_follow_seed_since = 0.0
        self._home_plan_pending = False
        self._nav_mode = None
        self._home_route = None
        self._home_fp_route_uid = None
        self._rebuild_resume_mode = None
        self._path_blocked_none_since = 0.0
        # Via FSM invalidate (fields may be absent pre-W1a — getattr-safe).
        self._home_via_phase = None
        self._home_via_generation = -1
        self._home_via_v_star = None
        self._home_via_leg1_pts = []
        self._home_via_candidate_queue = []
        self._last_home_navfn_pts = []
        self._last_home_c1_fail_pts = []
        self._owner.command_authority = CommandAuthority.NONE
        if SafetyInhibit.ALL_BLOCKED_NO_HOME not in self._owner.active_inhibits:
            # First raise: raise_inhibit bumps generation + STOPPED.
            self._owner.raise_inhibit(SafetyInhibit.ALL_BLOCKED_NO_HOME)
        else:
            # Re-raise: no bump — keep phase/gen sentinel invalidate above.
            self._owner.phase = MissionPhase.STOPPED
            self._owner.command_authority = CommandAuthority.NONE
            self._owner.stop_reason = SafetyInhibit.ALL_BLOCKED_NO_HOME.value
        self._owner.detail = f'NO_SAFE_ROUTE:{token}'
        self._set_state(MissionNavState.NO_SAFE_ROUTE)
        self._clear_active_path()
        # Publish owner status immediately (do not wait for 1 Hz timer) so
        # GUI / harnesses see STOPPED + ALL_BLOCKED_NO_HOME with bank state.
        self._publish_owner_status()
        self._publish_status()
        self.get_logger().error(
            f'NO_SAFE_ROUTE ({token}) → owner STOPPED auth=NONE '
            f'inhibit=ALL_BLOCKED_NO_HOME last_no_safe_detail={token}'
        )

    def _pick_next_valid(self, live_grid) -> Optional[PathCandidate]:
        self._validate_pending_candidates(live_grid)
        has_available_alternative = any(
            c.state == CandidateState.AVAILABLE
            and c.path_id != self._bank.active_id
            for c in self._bank.candidates
        )
        if (
            not has_available_alternative
            and not self._bank.all_blocked()
            and (
                self._bank_expansion_pending
                or any(
                    c.state in (CandidateState.PENDING, CandidateState.VALIDATING)
                    for c in self._bank.candidates
                )
            )
        ):
            self.get_logger().info(
                'Deferring exhaustion while path-bank expansion/validation is pending'
            )
            return None
        # Re-validate remaining AVAILABLE candidates against live costmap AND
        # obstacle_memory mask (Entry 023: same gate as active-path checks).
        remaining = sorted(
            [c for c in self._bank.candidates if c.state == CandidateState.AVAILABLE],
            key=lambda c: c.score,
            reverse=True,
        )
        for cand in remaining:
            ok, reason = self._candidate_route_valid(cand, live_grid)
            if not ok:
                self._mark_path_blocked(cand, f'precheck:{reason}')
                self.get_logger().info(
                    f'Path {cand.path_id} also invalid ({reason}); skipping'
                )
                continue
            cand.state = CandidateState.ACTIVE
            self._bank.active_id = cand.path_id
            self._sync_route_bank_active(cand.path_id)
            return cand
        return None

    def _validate_pending_candidates(self, live_grid) -> None:
        """Validate merged candidates before they can influence exhaustion."""
        if self._bank is None or live_grid is None:
            return
        for candidate in self._bank.candidates:
            if candidate.state != CandidateState.PENDING:
                continue
            candidate.state = CandidateState.VALIDATING
            ok, reason = self._candidate_route_valid(candidate, live_grid)
            if ok:
                candidate.state = CandidateState.AVAILABLE
                candidate.block_reason = ''
            else:
                candidate.state = CandidateState.BLOCKED
                candidate.block_reason = f'validation:{reason}'

    def _enter_safe_hold(self, reason: str) -> None:
        """Stop navigation while an old controller goal is being resolved."""
        self._switch_action_token += 1
        if self._fp_cancel_context is not None:
            self._fp_cancel_context['switch_token'] = self._switch_action_token
        self._set_state(MissionNavState.SAFE_HOLD)
        self._owner.phase = MissionPhase.HOLDING
        self._owner.command_authority = CommandAuthority.NONE
        self._owner.detail = reason
        self._publish_authority_now()
        self._clear_active_path()
        self._publish_status()
        self.get_logger().error(
            f'SAFE_HOLD reason={reason} authority=NONE active_path=cleared'
        )

    def _enter_nav2_cancel_failed(self, reason: str) -> None:
        """Terminalize an unconfirmed controller cancellation safely."""
        self._fp_cancel_context = None
        self._set_state(MissionNavState.NAV2_CANCEL_FAILED)
        self._owner.phase = MissionPhase.STOPPED
        self._owner.command_authority = CommandAuthority.NONE
        self._owner.detail = reason
        self._publish_authority_now()
        self._clear_active_path()
        self._publish_status()
        self.get_logger().error(
            f'NAV2_CANCEL_FAILED reason={reason} authority=NONE '
            'return_home=forbidden'
        )

    @staticmethod
    def _cancel_response_acknowledged(response) -> bool:
        """Accept only responses proving the old goal is no longer active."""
        if response is None:
            return False
        code = getattr(response, 'return_code', 0)
        # action_msgs/CancelGoal: NONE, UNKNOWN_GOAL_ID, GOAL_TERMINATED and
        # GOAL_CANCELING all mean the requested goal cannot continue running.
        return int(code) in (0, 2, 3, 4)

    def _begin_follow_path_cancellation(
        self, live_grid, generation: int, switch_token: int
    ) -> bool:
        """Request bounded cancellation while retaining the exact goal handle."""
        handle = self._fp_goal_handle
        if handle is None:
            return False
        epoch = int(self._fp_goal_epoch)
        self._fp_cancelled_epochs.add(epoch)
        self._fp_cancel_context = {
            'handle': handle,
            'epoch': epoch,
            'generation': generation,
            'switch_token': switch_token,
            'live_grid': live_grid,
            'retry': 0,
        }
        self._fp_cancel_retry_used = False
        self._fp_cancel_reason = 'route_switch'
        self._fp_cancel_started_at = time.monotonic()
        self._fp_cancel_deadline = (
            self._fp_cancel_started_at + FOLLOW_PATH_CANCEL_TIMEOUT_S)
        try:
            future = handle.cancel_goal_async()
            self._fp_cancel_context['future'] = future
            future.add_done_callback(
                lambda f, e=epoch, t=switch_token:
                self._on_follow_path_cancel_response(f, e, t)
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'FollowPath cancel request failed: {exc}')
            self._retry_follow_path_cancellation()
        self.get_logger().warn(
            f'FollowPath cancel requested epoch={epoch} '
            f'deadline={FOLLOW_PATH_CANCEL_TIMEOUT_S:.1f}s'
        )
        return True

    def _retry_follow_path_cancellation(self) -> None:
        context = self._fp_cancel_context
        if not context or self._fp_cancel_retry_used:
            return
        self._fp_cancel_retry_used = True
        context['retry'] = 1
        self._fp_cancel_started_at = time.monotonic()
        self._fp_cancel_deadline = (
            self._fp_cancel_started_at + NAV2_CANCEL_RETRY_TIMEOUT_S)
        try:
            future = context['handle'].cancel_goal_async()
            context['future'] = future
            future.add_done_callback(
                lambda f, e=context['epoch'], t=context['switch_token']:
                self._on_follow_path_cancel_response(f, e, t)
            )
            self.get_logger().warn(
                f'FollowPath cancellation retry requested exact epoch='
                f'{context["epoch"]} deadline={NAV2_CANCEL_RETRY_TIMEOUT_S:.1f}s'
            )
        except Exception as exc:  # noqa: BLE001
            self._enter_nav2_cancel_failed(
                f'cancel_retry_exception:{type(exc).__name__}')

    def _on_follow_path_cancel_response(
        self, future, epoch: int, switch_token: int
    ) -> None:
        context = self._fp_cancel_context
        if not context or context.get('epoch') != int(epoch):
            try:
                future.result()
            except Exception:
                pass
            self.get_logger().info(
                f'Processed stale FollowPath cancel response epoch={epoch}'
            )
            return
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f'FollowPath cancel response failed epoch={epoch}: {exc}')
            return
        if not self._cancel_response_acknowledged(response):
            self.get_logger().warn(
                f'FollowPath cancel not acknowledged epoch={epoch} '
                f'retry={context["retry"]}'
            )
            return
        self._fp_goal_handle = None
        self._fp_cancel_context = None
        self._fp_cancel_deadline = 0.0
        self.get_logger().info(
            f'FollowPath cancellation acknowledged epoch={epoch} '
            f'retry={context["retry"]}'
        )
        if (
            context.get('switch_token') != self._switch_action_token
            or context['generation'] != self._owner.generation
        ):
            return
        if self._owner.has_blocking_inhibit() or self._owner.operator_stop_latch:
            return
        if self._state == MissionNavState.SAFE_HOLD:
            self._owner.phase = MissionPhase.NAVIGATING
            self._owner.command_authority = CommandAuthority.NAV
            self._publish_authority_now()
        self._switch_follow_path(context['live_grid'])

    def _tick_follow_path_cancellation(self) -> None:
        context = self._fp_cancel_context
        if not context or time.monotonic() < self._fp_cancel_deadline:
            return
        if not self._fp_cancel_retry_used:
            self._enter_safe_hold('CANCEL_UNCONFIRMED')
            self._retry_follow_path_cancellation()
            return
        self._enter_nav2_cancel_failed(
            'cancel_unconfirmed_after_retry')

    def _switch_follow_path(self, live_grid) -> None:
        """Mark-blocked → cancel FP → clear → validate next → submit (gen fence)."""
        gen = self._owner.generation
        self._switch_action_token += 1
        switch_token = self._switch_action_token
        # Remove the old authoritative route before waiting for cancellation.
        self._clear_active_path()
        if self._begin_follow_path_cancellation(live_grid, gen, switch_token):
            self._enter_safe_hold('CANCEL_PENDING')
            return
        if gen != self._owner.generation:
            self.get_logger().info(
                'Aborting FollowPath switch — generation changed during cancel'
            )
            return
        if self._owner.has_blocking_inhibit() or self._owner.operator_stop_latch:
            self.get_logger().warn(
                'Aborting FollowPath switch — latch/inhibit active'
            )
            self._set_state(MissionNavState.PATH_BLOCKED)
            return
        if self._owner.command_authority == CommandAuthority.NONE:
            # Queue only — never re-arm NAV during COSTMAP_REBUILD/HOLDING/STOP*.
            self._switch_pending = True
            self.get_logger().warn(
                'Queuing FollowPath switch — command_authority=NONE '
                f'(phase={self._owner.phase.value}; retry when NAV under live mission)'
            )
            self._set_state(MissionNavState.PATH_BLOCKED)
            return

        self._switch_pending = False
        self._clear_local_costmap()
        nxt = self._pick_next_valid(live_grid)
        if nxt is None:
            has_available_alternative = any(
                c.state == CandidateState.AVAILABLE
                and c.path_id != self._bank.active_id
                for c in self._bank.candidates
            )
            if (
                not has_available_alternative
                and not self._bank.all_blocked()
                and (
                    self._bank_expansion_pending
                    or any(
                        c.state in (CandidateState.PENDING, CandidateState.VALIDATING)
                        for c in self._bank.candidates
                    )
                )
            ):
                self._set_state(MissionNavState.PATH_BLOCKED)
                self._switch_pending = True
                return
            self.get_logger().error(
                'No valid banked path remains'
                + (' (all_blocked)' if self._bank and self._bank.all_blocked() else '')
            )
            self._set_state(MissionNavState.NO_PATH_AVAILABLE)
            if self._return_home_on_all_blocked:
                self._begin_return_to_start()
                return
            if self._cancel_on_no_path:
                self._cancel_nav()
            return

        if gen != self._owner.generation:
            self.get_logger().info(
                'Aborting FollowPath switch — generation changed before submit'
            )
            return

        self._switch_count += 1
        self._abort_resend_used = False  # new route gets one ownership recovery
        self._stuck_recoveries = 0
        self._far_gap_rebuilds = 0
        self._bridge_exhaust_drive_retries = 0
        self._splice_used = False
        self._splice_pending = False
        self._switch_navfn_retry_used = False
        self._switch_join_queue = None
        self._switch_join_attempt = 0
        self._switch_goal_fallback_used = False
        self._switch_bridge_goal_only = False
        self._far_gap_grace_collapsed = False
        self._escape_hop_used = False
        self._escape_hop_count = 0
        self._escape_hop_pending_join = False
        self._drive_escape_used = False
        self._drive_escape_pending_join = False
        # Entry 083 Phase 1A: log standoff vs locus at switch arm.
        standoff_m = -1.0
        seat_disk_violation = False
        locus_arm = self._path_a_repel_xy()
        if self._robot_xyyaw is not None and locus_arm is not None:
            standoff_m = math.hypot(
                self._robot_xyyaw[0] - locus_arm[0],
                self._robot_xyyaw[1] - locus_arm[1],
            )
            seat_disk_violation = standoff_m < float(KEEPOUT_SEAT_DISK_M)
        self.get_logger().info(
            f'Switching to Path {nxt.path_id} via FollowPath '
            f'(score={nxt.score:.1f}, switch #{self._switch_count}) '
            f'switch_standoff_m={standoff_m:.2f} '
            f'seat_disk_violation={seat_disk_violation} '
            f'(min={SWITCH_STANDOFF_MIN_M} seat_disk={KEEPOUT_SEAT_DISK_M})'
        )
        self._publish_bank_paths()
        self._nav_mode = 'mission'
        route = None
        if self._route_bank is not None:
            route = self._route_bank.by_path_id(nxt.path_id)
            self._sync_route_bank_active(nxt.path_id)
        if route is None:
            self.get_logger().error(
                f'No RouteInstance for Path {nxt.path_id}; cannot FollowPath'
            )
            self._set_state(MissionNavState.FAILED)
            return
        # Entry 072/073: clear LIDAR-only keepout under footprint (Humble has
        # no override_lethal_cost). Barrier cells beyond ~1.0 m stay marked.
        self._request_footprint_escape_clear()
        # Do NOT preempt NTP in bank_follow_path mode.
        # Entry 073 (third distinct approach — NOT a hop variant):
        #   1) Arm local LIDAR ObstacleLayer disable + ClearEntirely so the
        #      sealed CostCritic footprint can open (FIRE layer untouched).
        #   2) Path-B-tangent DriveOnHeading with LIDAR cone collision gate
        #      (geometric yaw — costmap free-heading already failed R11).
        #   3) Path-B join NavFn / GOAL. Escape hop budget is 0 this pass.
        # Entry 081: spawn-stem snap forces the same bridge ladder; bare
        # FollowPath is forbidden when stem-snap and bridge fails.
        bridged = False
        stem = False
        nearest = 0.0
        if self._robot_xyyaw is not None:
            rx, ry, _ = self._robot_xyyaw
            nearest = min(
                math.hypot(rx - x, ry - y)
                for x, y in route.points[:: max(1, len(route.points) // 100)]
            )
            path_a_pts = self._path_a_points_for_stem(nxt.path_id)
            bank_start = (
                self._bank.start if self._bank is not None
                else (float(route.points[0][0]), float(route.points[0][1]))
            )
            stem = is_spawn_stem_snap(
                robot_xy=(rx, ry),
                bank_start=bank_start,
                target_points=route.points,
                path_a_points=path_a_pts,
            )
            force_bridge = stem or nearest > 2.0
            repel = self._path_a_repel_xy()
            locus_d = (
                math.hypot(rx - repel[0], ry - repel[1])
                if repel is not None else None
            )
            if should_arm_sealed_start_unseal(nearest, locus_d):
                self._arm_sealed_start_local_unseal('switch_far_gap')
            # DriveOnHeading often False mid-corridor; NavFn join/GOAL is
            # primary stem recovery. Still try Drive when stem or far-gap.
            if (
                (stem or nearest > SEALED_UNSEAL_PATH_GAP_M)
                and not getattr(self, '_drive_escape_used', False)
            ):
                bridged = self._request_drive_on_heading_escape(
                    route, generation=gen, action_token=switch_token)
                if bridged:
                    self.get_logger().info(
                        f'Switch far-gap {nearest:.1f}m → Path-B-tangent '
                        'DriveOnHeading (Entry 073 local unseal)'
                        + (' [stem-snap]' if stem else '')
                    )
            if not bridged and force_bridge:
                bridged = self._request_switch_navfn_bridge(
                    route, generation=gen)
                if bridged:
                    self.get_logger().info(
                        f'Switch far-gap {nearest:.1f}m → Path-B join NavFn'
                        + (' [stem-snap]' if stem else '')
                    )
                # Stem: always try GOAL fallback even when nearest ≤ 5.
                elif stem or nearest > 5.0:
                    bridged = self._request_switch_navfn_goal_fallback(
                        route, generation=gen)
                    if bridged:
                        self.get_logger().info(
                            f'Switch far-gap {nearest:.1f}m → GOAL NavFn '
                            '(join bridge could not start)'
                            + (' [stem-snap]' if stem else '')
                        )
        if not bridged:
            if stem:
                self.get_logger().warn(
                    f'STEM_SNAP_BRIDGE_FAILED Path {nxt.path_id} '
                    f'nearest={nearest:.1f}'
                )
                # Entry 084 R1: sole Inc via refuse helper (SYNC_SWITCH_ARM).
                self._refuse_bare_stem_follow_path(
                    route,
                    generation=gen,
                    context=RefuseContext.SYNC_SWITCH_ARM,
                )
                return
            self._send_follow_path(route, expected_generation=gen)
            self._stem_snap_bridge_fails = 0
        else:
            self._stem_snap_bridge_fails = 0
        # Longer grace after far-gap switches so first bridge can settle.
        grace = self._post_switch_grace_s
        nearest_gap = nearest
        if nearest_gap > 5.0 or stem:
            # R3: 16 s grace expired exactly when stuck recovery cancelled a
            # fresh GOAL NavFn and thrashed join bridges → 10.6 m stall.
            # Give the far-gap bridge time to leave the barrier pocket.
            grace = max(grace, 45.0)
        if grace > 0.0:
            self._switch_grace_until = time.monotonic() + grace
        # Prefer forward motion after route switch. Issue B: bank-owned Spin
        # (inside _send_follow_path yaw gate) is the primary rotate-then-drive;
        # forward_only remains the safety net through reorient + grace.
        fo_hold = max(grace, 8.0)
        if nearest_gap > 5.0 or stem:
            fo_hold = max(fo_hold, 45.0)
        self._forward_only_until = time.monotonic() + fo_hold
        self._publish_forward_only()
        self._set_state(MissionNavState.FOLLOWING_PATH)
        self._last_progress_t = time.monotonic()
        if self._robot_xyyaw is not None:
            self._last_progress_xy = (
                self._robot_xyyaw[0], self._robot_xyyaw[1])

    def _clear_switch_pending(self, reason: str = '') -> None:
        if not self._switch_pending:
            return
        self._switch_pending = False
        self.get_logger().info(
            'Cleared queued FollowPath switch'
            + (f' ({reason})' if reason else '')
        )

    def _publish_forward_only(self) -> None:
        msg = Bool()
        # Entry 073: sealed-unseal latch holds forward_only until ARRIVED/Stop
        # so near-goal MPPI cannot emit a reverse-then-recover sample.
        msg.data = (
            time.monotonic() < self._forward_only_until
            or bool(getattr(self, '_sealed_unseal_forward_latch', False))
        )
        self._forward_only_pub.publish(msg)

    def _maybe_complete_pending_switch(self) -> None:
        """Complete SWITCH_PENDING only under NAV + live mission phase."""
        if not self._switch_pending or not self._use_follow_path:
            return
        if self._owner.operator_stop_latch or self._owner.has_blocking_inhibit():
            self._clear_switch_pending('inhibit_or_latch')
            return
        if self._owner.phase in (
            MissionPhase.COSTMAP_REBUILD,
            MissionPhase.HOLDING,
            MissionPhase.STOPPING,
            MissionPhase.STOPPED,
            MissionPhase.IDLE,
        ):
            return  # wait — do not re-arm NAV here
        if self._owner.command_authority != CommandAuthority.NAV:
            return
        if self._owner.phase not in (
            MissionPhase.NAVIGATING,
            MissionPhase.SWITCHING,
        ):
            return
        if self._live_costmap is None or self._bank is None:
            return
        # Entry 080/081 normative: defer expansion ONLY when there is no
        # AVAILABLE alternative and the bank is not yet all_blocked. An
        # AVAILABLE path or all_blocked must complete the queued switch now.
        has_available_alternative = any(
            c.state == CandidateState.AVAILABLE
            and c.path_id != self._bank.active_id
            for c in self._bank.candidates
        )
        if (
            not has_available_alternative
            and not self._bank.all_blocked()
            and (
                self._bank_expansion_pending
                or any(
                    c.state in (CandidateState.PENDING, CandidateState.VALIDATING)
                    for c in self._bank.candidates
                )
            )
        ):
            return
        if self._fp_cancel_context is not None:
            return
        self.get_logger().info(
            'Completing queued FollowPath switch (authority=NAV, live mission)'
        )
        live = nav2_costmap_to_grid(self._live_costmap)
        self._set_state(MissionNavState.SWITCHING_PATH)
        self._switch_follow_path(live)

    # -- nav helpers -------------------------------------------------------

    def _arm_nav_authority(self) -> bool:
        """Open NAV authority when safe; return False if submit must abort.

        Does not promote IDLE/STOPPED → NAVIGATING. Authority to start a mission
        must come from accept_normal_start() / StartMission only.
        """
        if self._owner.abort_resend_forbidden() and self._owner.phase in (
            MissionPhase.STOPPING,
            MissionPhase.STOPPED,
            MissionPhase.HOLDING,
            MissionPhase.COSTMAP_REBUILD,
        ):
            return False
        if self._owner.has_blocking_inhibit() or self._owner.operator_stop_latch:
            return False
        if (
            self._owner.command_authority == CommandAuthority.NONE
            and self._owner.phase in (
                MissionPhase.NAVIGATING,
                MissionPhase.RETURNING_TO_START,
                MissionPhase.SWITCHING,
            )
        ):
            self._owner.command_authority = CommandAuthority.NAV
            self._publish_authority_now()
        return self._owner.command_authority == CommandAuthority.NAV

    def _route_checkpoint_ahead(
        self, route: RouteInstance, dist_m: float = 6.0
    ) -> Optional[Tuple[float, float, float]]:
        """Map pose ~dist_m along forward RouteInstance from the robot."""
        pts, yaws = self._route_forward_from_robot(route)
        if len(pts) < 2:
            return None
        remain = float(dist_m)
        for i in range(1, len(pts)):
            x0, y0 = pts[i - 1]
            x1, y1 = pts[i]
            seg = math.hypot(x1 - x0, y1 - y0)
            if seg < 1e-6:
                continue
            if seg >= remain:
                t = remain / seg
                x = x0 + t * (x1 - x0)
                y = y0 + t * (y1 - y0)
                return (x, y, float(yaws[i]))
            remain -= seg
        x, y = pts[-1]
        return (float(x), float(y), float(yaws[-1]))

    def _request_corridor_splice(self, route: RouteInstance) -> bool:
        """NavFn path robot→checkpoint along RouteInstance; FollowPath that splice."""
        if self._compute_path_client is None or self._robot_xyyaw is None:
            return False
        if self._splice_used or self._splice_pending:
            return False
        if not self._compute_path_client.server_is_ready():
            self.get_logger().warn('compute_path_to_pose not ready for splice')
            return False
        ckpt = self._route_checkpoint_ahead(route, 6.0)
        if ckpt is None:
            return False
        rx, ry, ryaw = self._robot_xyyaw
        gx, gy, gyaw = ckpt
        # Reject checkpoints that do not reduce distance-to-goal (Entry 034:
        # splice to (-21,-5) walked off-trail when Path 1 snaked south first).
        if self._goal_xyyaw is not None:
            tx, ty, _ = self._goal_xyyaw
            if math.hypot(gx - tx, gy - ty) >= math.hypot(rx - tx, ry - ty) - 0.25:
                self.get_logger().warn(
                    f'Splice checkpoint ({gx:.2f},{gy:.2f}) not closer to goal; '
                    'skip corridor splice'
                )
                self._splice_used = True  # do not retry a regressing splice
                return False
        goal = ComputePathToPose.Goal()
        goal.planner_id = 'GridBased'
        goal.use_start = True
        goal.start.header.frame_id = self._map_frame
        goal.start.header.stamp = self.get_clock().now().to_msg()
        goal.start.pose.position.x = float(rx)
        goal.start.pose.position.y = float(ry)
        goal.start.pose.orientation.z = math.sin(0.5 * ryaw)
        goal.start.pose.orientation.w = math.cos(0.5 * ryaw)
        goal.goal.header = goal.start.header
        goal.goal.pose.position.x = float(gx)
        goal.goal.pose.position.y = float(gy)
        goal.goal.pose.orientation.z = math.sin(0.5 * gyaw)
        goal.goal.pose.orientation.w = math.cos(0.5 * gyaw)
        gen = self._owner.generation
        self._splice_pending = True
        fut = self._compute_path_client.send_goal_async(goal)
        fut.add_done_callback(
            lambda f, g=gen, r=route: self._splice_goal_response(f, g, r))
        self.get_logger().info(
            f'Corridor splice ComputePathToPose → ({gx:.2f},{gy:.2f}) '
            f'along path_id={route.path_id}'
        )
        return True

    def _splice_goal_response(self, future, generation: int, route: RouteInstance) -> None:
        if generation != self._owner.generation:
            self._splice_pending = False
            return
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self._splice_pending = False
            self.get_logger().warn(f'Splice goal failed: {exc}')
            return
        if handle is None or not handle.accepted:
            self._splice_pending = False
            self.get_logger().warn('Splice ComputePathToPose rejected')
            return
        result_fut = handle.get_result_async()
        result_fut.add_done_callback(
            lambda f, g=generation, r=route: self._splice_result(f, g, r))

    def _splice_result(self, future, generation: int, route: RouteInstance) -> None:
        self._splice_pending = False
        if generation != self._owner.generation:
            return
        try:
            wrapped = future.result()
            status = wrapped.status
            result = wrapped.result
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Splice result failed: {exc}')
            return
        if status != GoalStatus.STATUS_SUCCEEDED or result is None:
            self.get_logger().warn(f'Splice ComputePathToPose status={status}')
            return
        path_msg = result.path
        if path_msg is None or len(path_msg.poses) < 2:
            self.get_logger().warn('Splice returned empty path')
            return
        # Reject splices that leave the bank corridor (prior run drove off-trail
        # to (-23,-8) when NavFn wandered).
        max_dev = 0.0
        for ps in path_msg.poses[:: max(1, len(path_msg.poses) // 40)]:
            px = float(ps.pose.position.x)
            py = float(ps.pose.position.y)
            best = min(
                math.hypot(px - rx, py - ry)
                for rx, ry in route.points[:: max(1, len(route.points) // 80)]
            )
            if best > max_dev:
                max_dev = best
        if max_dev > 1.5:
            self.get_logger().warn(
                f'Splice rejected — max deviation from RouteInstance '
                f'{max_dev:.2f} m > 1.5 m'
            )
            return
        self._splice_used = True
        self._abort_resend_used = False
        self.get_logger().info(
            f'Corridor splice OK poses={len(path_msg.poses)} max_dev={max_dev:.2f}m; '
            f'FollowPath splice (route_uid={route.route_uid})'
        )
        self._send_follow_path_msg(path_msg, route_uid=route.route_uid, path_id=route.path_id)

    def _join_candidates(
        self, route: RouteInstance, *, limit: int = 10
    ) -> list:
        """Ranked Path-B join poses for NavFn bridge (Entry 070/048/036)."""
        if self._robot_xyyaw is None or not route.points:
            return []
        rx, ry, ryaw = self._robot_xyyaw
        goal_xy = None
        if self._goal_xyyaw is not None:
            goal_xy = (self._goal_xyyaw[0], self._goal_xyyaw[1])
        return rank_switch_join_candidates(
            (rx, ry),
            ryaw,
            goal_xy,
            route.points,
            route.yaw_rad,
            limit=limit,
        )

    def _nearest_route_pose(
        self, route: RouteInstance
    ) -> Optional[Tuple[float, float, float, int]]:
        cands = self._join_candidates(route, limit=1)
        return cands[0] if cands else None

    def _try_next_switch_bridge(
        self, route: RouteInstance, generation: int
    ) -> bool:
        """After rejecting a NavFn bridge, rotate join or GOAL; True if armed."""
        if getattr(self, '_switch_escape_only', False):
            self._switch_escape_only = False
        if getattr(self, '_switch_bridge_goal_only', False):
            self._switch_bridge_goal_only = False
            self._switch_goal_fallback_used = False
        q = getattr(self, '_switch_join_queue', None) or []
        attempt = int(getattr(self, '_switch_join_attempt', 0))
        if attempt < len(q):
            if self._request_switch_navfn_bridge(route, generation=generation):
                return True
        if not getattr(self, '_switch_goal_fallback_used', False):
            if self._request_switch_navfn_goal_fallback(
                route, generation=generation
            ):
                return True
        self.get_logger().warn(
            'Switch NavFn bridges exhausted after reject; escalate (Entry 082)'
        )
        self._switch_join_queue = None
        self._switch_goal_fallback_used = False
        gap = None
        if self._robot_xyyaw is not None:
            nearest = self._nearest_route_pose(route)
            if nearest is not None:
                gap = math.hypot(
                    self._robot_xyyaw[0] - nearest[0],
                    self._robot_xyyaw[1] - nearest[1],
                )
        stem = self._route_is_spawn_stem_snap(route)
        if stem:
            # Entry 084: stem terminal → refuse (Inc) → exhaust stem_snap.
            self._refuse_bare_stem_follow_path(
                route,
                generation=generation,
                context=RefuseContext.ASYNC_BRIDGE,
            )
        else:
            self._on_switch_bridge_exhausted(
                route, generation=generation, gap=gap, stem_snap=False)
        return False

    def _on_switch_bridge_exhausted(
        self,
        route: RouteInstance,
        *,
        generation: int,
        gap: Optional[float],
        stem_snap: bool = False,
    ) -> None:
        """After join+GOAL NavFn fail: one Drive retry, else honest escalate.

        Entry 082: never leave FOLLOWING_PATH with orphaned_handle while
        ``active route still live-valid`` rebuilds until outer kill.
        Entry 084: ``stem_snap=True`` forces far-gap Drive eligibility even
        when measured gap ≤5 (exclusive-B far / stem vertex near). Live exhaust
        clears ``_drive_escape_used`` before retry — gate on
        ``_bridge_exhaust_drive_retries`` only, not the escape-used latch.
        """
        if generation != self._owner.generation:
            return
        retries = int(getattr(self, '_bridge_exhaust_drive_retries', 0))
        if (
            (stem_snap or (gap is not None and gap > 5.0))
            and retries < 1
            and not self._splice_pending
        ):
            self._bridge_exhaust_drive_retries = retries + 1
            self._drive_escape_used = False
            self._switch_join_queue = None
            self._switch_join_attempt = 0
            self._switch_goal_fallback_used = False
            if self._request_drive_on_heading_escape(
                route, generation=generation
            ):
                why = (
                    f'gap={gap:.1f}m' if gap is not None else 'gap=?'
                )
                if stem_snap:
                    why += ' stem_snap=True effective_far_gap=stem_forced'
                self.get_logger().warn(
                    f'Switch bridge exhausted ({why}) → '
                    'DriveOnHeading retry then re-bridge (Entry 082/084)'
                )
                return
        budget = max(1, int(getattr(self, '_far_gap_rebuild_budget', 3)))
        self._far_gap_rebuilds = max(
            int(getattr(self, '_far_gap_rebuilds', 0)), budget)
        self._stuck_recoveries = max(
            int(getattr(self, '_stuck_recoveries', 0)),
            int(self._stuck_budget_n),
        )
        self._fp_goal_handle = None
        self._last_progress_t = time.monotonic() - self._stuck_timeout_s - 0.1
        self._stuck_recover_until = 0.0
        self.get_logger().error(
            'Switch NavFn bridge exhausted'
            + (f' (gap={gap:.1f}m)' if gap is not None else '')
            + (' stem_snap=True' if stem_snap else '')
            + '; escalating stuck → home/NO_SAFE (Entry 082/084)'
        )
        self._maybe_owner_stuck_recover()

    def _request_footprint_escape_clear(self) -> None:
        """Publish robot pose so obstacle_memory clears LIDAR footprint traps."""
        if self._robot_xyyaw is None:
            return
        pub = getattr(self, '_footprint_escape_pub', None)
        if pub is None:
            return
        rx, ry, ryaw = self._robot_xyyaw
        ps = PoseStamped()
        ps.header.frame_id = self._map_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(rx)
        ps.pose.position.y = float(ry)
        ps.pose.orientation.z = math.sin(0.5 * float(ryaw))
        ps.pose.orientation.w = math.cos(0.5 * float(ryaw))
        pub.publish(ps)

    def _path_b_attract_xy(
        self, route: RouteInstance
    ) -> Optional[Tuple[float, float]]:
        """Nearest Path-B sample, else mission GOAL — escape-hop attractor."""
        if self._robot_xyyaw is None:
            return None
        rx, ry, _ = self._robot_xyyaw
        best = None
        if route.points:
            stride = max(1, len(route.points) // 100)
            for x, y in route.points[::stride]:
                d = math.hypot(rx - float(x), ry - float(y))
                if best is None or d < best[0]:
                    best = (d, float(x), float(y))
        if best is not None:
            return (best[1], best[2])
        if self._goal_xyyaw is not None:
            return (float(self._goal_xyyaw[0]), float(self._goal_xyyaw[1]))
        return None

    def _path_a_points_for_stem(
        self, target_path_id: int
    ) -> List[Tuple[float, float]]:
        """Path-A (or first non-target) polyline for stem-snap comparison."""
        if self._bank is None:
            return []
        # Prefer densified RouteInstance geometry when available.
        if self._route_bank is not None:
            for pid in (1,):
                if pid == target_path_id:
                    continue
                route = self._route_bank.by_path_id(pid)
                if route is not None and route.points:
                    return [(float(x), float(y)) for x, y in route.points]
            for route in getattr(self._route_bank, 'routes', []) or []:
                if int(getattr(route, 'path_id', -1)) == int(target_path_id):
                    continue
                pts = getattr(route, 'points', None)
                if pts:
                    return [(float(x), float(y)) for x, y in pts]
        for cand in self._bank.candidates:
            if cand.path_id == 1 and cand.path_id != target_path_id:
                return list(cand.points)
        for cand in self._bank.candidates:
            if cand.path_id != target_path_id:
                return list(cand.points)
        return []

    def _bank_start_xy(
        self, route: RouteInstance
    ) -> Tuple[float, float]:
        if self._bank is not None:
            return (float(self._bank.start[0]), float(self._bank.start[1]))
        if route.points:
            return (float(route.points[0][0]), float(route.points[0][1]))
        return (0.0, 0.0)

    def _route_is_spawn_stem_snap(self, route: RouteInstance) -> bool:
        """Entry 084 gate predicate — only ``is_spawn_stem_snap``."""
        if self._robot_xyyaw is None or not route.points:
            return False
        rx, ry, _ = self._robot_xyyaw
        return is_spawn_stem_snap(
            robot_xy=(rx, ry),
            bank_start=self._bank_start_xy(route),
            target_points=route.points,
            path_a_points=self._path_a_points_for_stem(route.path_id),
        )

    def _refuse_bare_stem_follow_path(
        self,
        route: RouteInstance,
        *,
        generation: int,
        context: RefuseContext,
    ) -> None:
        """Refuse bare FollowPath under spawn-stem snap (Entry 084).

        Sole owner of ``_stem_snap_bridge_fails`` increments. Never leaves
        FOLLOWING_PATH without an fp handle — ends in PATH_BLOCKED+pending
        or ``_on_switch_bridge_exhausted`` (no bare FP).
        """
        self._stem_snap_bridge_fails = (
            int(getattr(self, '_stem_snap_bridge_fails', 0)) + 1
        )
        fails = int(self._stem_snap_bridge_fails)
        self.get_logger().warn(
            f'STEM_SNAP_BARE_FP_REFUSED context={context.value} '
            f'fails={fails} path_id={route.path_id}'
        )
        budget = max(1, int(getattr(self, '_far_gap_rebuild_budget', 3)))
        if fails >= budget:
            self._stem_snap_bridge_fails = 0
            if self._bank is not None:
                for cand in self._bank.candidates:
                    if cand.path_id == route.path_id:
                        self._mark_path_blocked(
                            cand, 'stem_snap_bridge_exhausted')
                        break
            self.get_logger().error(
                f'Path {route.path_id} stem_snap_bridge_exhausted '
                f'after {budget} fails'
            )

        ctx = context
        if ctx in (
            RefuseContext.CENTRAL_GATE_STUCK,
            RefuseContext.CENTRAL_GATE_OTHER,
        ):
            bridge_active = (
                bool(self._splice_pending)
                or bool(getattr(self, '_switch_join_queue', None))
                or bool(getattr(self, '_switch_goal_fallback_used', False))
                or bool(getattr(self, '_switch_bridge_goal_only', False))
                or bool(getattr(self, '_switch_escape_only', False))
            )
            ctx = (
                RefuseContext.ASYNC_BRIDGE if bridge_active
                else RefuseContext.SYNC_SWITCH_ARM
            )

        if ctx == RefuseContext.SYNC_SWITCH_ARM:
            self._set_state(MissionNavState.PATH_BLOCKED)
            self._switch_pending = True
            return

        # ASYNC_BRIDGE / CENTRAL_GATE_POST_DRIVE → far-gap exhaust (stem).
        self._splice_pending = False
        gap = None
        if self._robot_xyyaw is not None:
            nearest = self._nearest_route_pose(route)
            if nearest is not None:
                gap = math.hypot(
                    self._robot_xyyaw[0] - nearest[0],
                    self._robot_xyyaw[1] - nearest[1],
                )
        self._on_switch_bridge_exhausted(
            route,
            generation=generation,
            gap=gap,
            stem_snap=True,
        )

    def _path_a_repel_xy(self) -> Optional[Tuple[float, float]]:
        """Path-A block locus for barrier-repel escape (Entry 072)."""
        if not self._block_locus:
            return None
        # Prefer the most recently blocked non-active path locus.
        active_pid = None
        if self._bank is not None and self._bank.active() is not None:
            active_pid = int(self._bank.active().path_id)
        for pid, xy in self._block_locus.items():
            if active_pid is not None and int(pid) == active_pid:
                continue
            if xy is not None and len(xy) >= 2:
                return (float(xy[0]), float(xy[1]))
        # Fallback: any locus.
        for xy in self._block_locus.values():
            if xy is not None and len(xy) >= 2:
                return (float(xy[0]), float(xy[1]))
        return None

    def _request_drive_on_heading_escape(
        self, route: RouteInstance, *, generation: int, action_token: Optional[int] = None
    ) -> bool:
        """Path-B-tangent DriveOnHeading under local unseal (Entry 073).

        Geometric attractor yaw (not costmap free-heading — R11 found none).
        LIDAR forward-cone clearance is the collision gate while the local
        LIDAR ObstacleLayer is temporarily disabled.
        """
        if (
            getattr(self, '_drive_on_heading_client', None) is None
            or self._robot_xyyaw is None
        ):
            return False
        if getattr(self, '_drive_escape_used', False) or self._splice_pending:
            return False
        if getattr(self, '_reorient_active', False):
            return False
        attract = self._path_b_attract_xy(route)
        if attract is None:
            return False
        rx, ry, ryaw = self._robot_xyyaw
        repel = self._path_a_repel_xy()
        if repel is not None:
            if math.hypot(rx - repel[0], ry - repel[1]) > SEALED_UNSEAL_LOCUS_ARM_M:
                return False  # not in the barrier corner pocket
        if not self._drive_on_heading_client.server_is_ready():
            return False
        tgt_yaw = path_b_tangent_yaw((rx, ry), attract)
        # Forward-only relative to current yaw (Issue B — no reverse drive).
        along = (
            math.cos(tgt_yaw) * math.cos(ryaw)
            + math.sin(tgt_yaw) * math.sin(ryaw)
        )
        if along < 0.05:
            self.get_logger().warn(
                'Path-B-tangent DriveOnHeading: attractor behind robot — skip'
            )
            return False
        # LIDAR cone in *current* base frame toward the target yaw offset.
        yaw_err = wrap_angle(tgt_yaw - ryaw)
        ranges = getattr(self, '_scan_ranges', None)
        if ranges is None:
            self.get_logger().warn(
                'Path-B-tangent DriveOnHeading: no LIDAR yet — skip'
            )
            return False
        if not lidar_cone_clear(
            ranges,
            float(getattr(self, '_scan_angle_min', 0.0)),
            float(getattr(self, '_scan_angle_inc', 0.0)),
            cone_center_rad=float(yaw_err),
            half_width_rad=LIDAR_DRIVE_HALF_WIDTH_RAD,
            min_clear_m=LIDAR_DRIVE_MIN_CLEAR_M,
            range_min=float(getattr(self, '_scan_range_min', 0.05)),
            range_max_limit=float(getattr(self, '_scan_range_max', 25.0)),
        ):
            self.get_logger().warn(
                'Path-B-tangent DriveOnHeading: LIDAR cone blocked — skip'
            )
            return False
        self._drive_escape_used = True
        self._drive_escape_pending_join = True
        self._drive_escape_route = route
        self._drive_escape_generation = generation
        self._drive_escape_token = (
            self._switch_action_token if action_token is None else action_token)
        self._drive_escape_yaw = float(tgt_yaw)
        self._request_footprint_escape_clear()
        if not getattr(self, '_sealed_unseal_active', False):
            self._arm_sealed_start_local_unseal('drive_escape')
        else:
            self._clear_local_costmap()
        locus_s = (
            f' locus=({repel[0]:.1f},{repel[1]:.1f})'
            if repel is not None else ''
        )
        if abs(yaw_err) > 0.35 and self._spin_client is not None:
            self.get_logger().info(
                f'Path-B-tangent DriveOnHeading: Spin yaw_err={yaw_err:.2f} '
                f'then +{PATH_B_TANGENT_DRIVE_M:.1f}m toward '
                f'attract=({attract[0]:.1f},{attract[1]:.1f}){locus_s}'
            )
            if self._spin_client.server_is_ready():
                goal = Spin.Goal()
                goal.target_yaw = float(yaw_err)
                goal.time_allowance = Duration(sec=12, nanosec=0)
                fut = self._spin_client.send_goal_async(goal)
                fut.add_done_callback(self._drive_escape_spin_response)
                return True
        return self._dispatch_drive_on_heading_escape()

    def _drive_escape_spin_response(self, future) -> None:
        if getattr(self, '_drive_escape_token', -1) != self._switch_action_token:
            return
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Drive-escape Spin send failed: {exc}')
            self._dispatch_drive_on_heading_escape()
            return
        if handle is None or not handle.accepted:
            self.get_logger().warn('Drive-escape Spin rejected — drive anyway')
            self._dispatch_drive_on_heading_escape()
            return
        result_fut = handle.get_result_async()
        result_fut.add_done_callback(self._drive_escape_spin_result)

    def _drive_escape_spin_result(self, future) -> None:
        if getattr(self, '_drive_escape_token', -1) != self._switch_action_token:
            return
        try:
            _ = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Drive-escape Spin result: {exc}')
        self._dispatch_drive_on_heading_escape()

    def _dispatch_drive_on_heading_escape(self) -> bool:
        if getattr(self, '_drive_on_heading_client', None) is None:
            self._drive_escape_pending_join = False
            return False
        if not self._drive_on_heading_client.server_is_ready():
            self._drive_escape_pending_join = False
            return False
        goal = DriveOnHeading.Goal()
        # Forward distance in base frame (+x). Re-check LIDAR at dispatch
        # (post-Spin) — sealed corner may still have a near return ahead.
        ranges = getattr(self, '_scan_ranges', None)
        if ranges is not None and not lidar_cone_clear(
            ranges,
            float(getattr(self, '_scan_angle_min', 0.0)),
            float(getattr(self, '_scan_angle_inc', 0.0)),
            cone_center_rad=0.0,
            half_width_rad=LIDAR_DRIVE_HALF_WIDTH_RAD,
            min_clear_m=LIDAR_DRIVE_MIN_CLEAR_M,
            range_min=float(getattr(self, '_scan_range_min', 0.05)),
            range_max_limit=float(getattr(self, '_scan_range_max', 25.0)),
        ):
            self.get_logger().warn(
                'Path-B-tangent DriveOnHeading: LIDAR blocked at dispatch — abort'
            )
            self._finish_drive_escape(ok=False)
            return False
        goal.target.x = float(PATH_B_TANGENT_DRIVE_M)
        goal.target.y = 0.0
        goal.target.z = 0.0
        goal.speed = 0.35
        goal.time_allowance = Duration(sec=20, nanosec=0)
        fut = self._drive_on_heading_client.send_goal_async(goal)
        fut.add_done_callback(self._drive_escape_goal_response)
        self.get_logger().info(
            f'Path-B-tangent DriveOnHeading → +{PATH_B_TANGENT_DRIVE_M:.1f} m '
            '@ 0.35 m/s (forward-only, LIDAR-gated)'
        )
        return True

    def _drive_escape_goal_response(self, future) -> None:
        if (
            getattr(self, '_drive_escape_token', -1) != self._switch_action_token
            or getattr(self, '_drive_escape_generation', -1)
            != self._owner.generation
        ):
            return
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'DriveOnHeading send failed: {exc}')
            self._finish_drive_escape(ok=False)
            return
        if handle is None or not handle.accepted:
            self.get_logger().warn('DriveOnHeading escape rejected')
            self._finish_drive_escape(ok=False)
            return
        self._drive_goal_handle = handle
        result_fut = handle.get_result_async()
        result_fut.add_done_callback(self._drive_escape_goal_result)

    def _drive_escape_goal_result(self, future) -> None:
        if (
            getattr(self, '_drive_escape_token', -1) != self._switch_action_token
            or getattr(self, '_drive_escape_generation', -1)
            != self._owner.generation
        ):
            return
        status = GoalStatus.STATUS_UNKNOWN
        try:
            wrapped = future.result()
            status = wrapped.status
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'DriveOnHeading result failed: {exc}')
        self._drive_goal_handle = None
        self._finish_drive_escape(ok=(status == GoalStatus.STATUS_SUCCEEDED))

    def _finish_drive_escape(self, *, ok: bool) -> None:
        if (
            getattr(self, '_drive_escape_token', -1) != self._switch_action_token
            or getattr(self, '_drive_escape_generation', -1)
            != self._owner.generation
        ):
            self._drive_escape_pending_join = False
            return
        pending = getattr(self, '_drive_escape_pending_join', False)
        self._drive_escape_pending_join = False
        route = getattr(self, '_drive_escape_route', None)
        gen = getattr(self, '_drive_escape_generation', self._owner.generation)
        if not pending or route is None:
            return
        self.get_logger().info(
            f'Path-B-tangent DriveOnHeading {"OK" if ok else "failed/partial"} '
            '— Path-B join/bridge next (Entry 073)'
        )
        self._abort_resend_used = False
        self._stuck_recoveries = 0
        self._far_gap_grace_collapsed = False
        self._set_state(MissionNavState.FOLLOWING_PATH)
        # Keep unseal armed through first FollowPath; timer restores on clear.
        if not getattr(self, '_sealed_unseal_active', False):
            self._arm_sealed_start_local_unseal('post_drive_escape')
        else:
            self._clear_local_costmap()
        # Entry 082: clear LIDAR keepout under the post-Drive pose before NavFn
        # (switch-time clear may be stale after +2.5 m motion). FIRE retained.
        self._request_footprint_escape_clear()
        # Entry 084 R12: central stem gate uses POST_DRIVE refuse context.
        self._send_follow_path(
            route,
            allow_navfn_bridge=True,
            refuse_context=RefuseContext.CENTRAL_GATE_POST_DRIVE,
        )

    def _request_switch_escape_hop(
        self, route: RouteInstance, *, generation: int
    ) -> bool:
        """Path-B-attracted / Path-A-repelled NavFn hop (Entry 072)."""
        if self._compute_path_client is None or self._robot_xyyaw is None:
            return False
        if self._splice_pending:
            return False
        budget = int(getattr(self, '_escape_hop_budget', 2))
        used = int(getattr(self, '_escape_hop_count', 0))
        if used >= budget:
            return False
        if self._live_costmap is None:
            return False
        if not self._compute_path_client.server_is_ready():
            return False
        self._request_footprint_escape_clear()
        live = nav2_costmap_to_grid(self._live_costmap)

        def _cost_at(x: float, y: float):
            idx = live.world_to_grid(x, y)
            if idx is None:
                return None
            return int(live.costs[idx[1], idx[0]])

        rx, ry, ryaw = self._robot_xyyaw
        attract = self._path_b_attract_xy(route)
        repel = self._path_a_repel_xy()
        hop = pick_escape_hop_pose(
            (rx, ry), ryaw, _cost_at,
            attract_xy=attract, repel_xy=repel)
        if hop is None and repel is not None:
            # Relax attract-only if barrier-repel free pose exists.
            hop = pick_escape_hop_pose(
                (rx, ry), ryaw, _cost_at,
                attract_xy=attract, repel_xy=repel,
                require_attract_progress=False)
        if hop is None:
            self.get_logger().warn(
                'Escape hop: no barrier-repel free pose — skipping to join'
            )
            return False
        gx, gy, gyaw = hop
        self._escape_hop_used = True
        self._escape_hop_count = used + 1
        self._switch_escape_only = True
        self._switch_bridge_goal_only = False
        goal = ComputePathToPose.Goal()
        goal.planner_id = 'GridBased'
        goal.use_start = True
        goal.start.header.frame_id = self._map_frame
        goal.start.header.stamp = self.get_clock().now().to_msg()
        goal.start.pose.position.x = float(rx)
        goal.start.pose.position.y = float(ry)
        goal.start.pose.orientation.z = math.sin(0.5 * ryaw)
        goal.start.pose.orientation.w = math.cos(0.5 * ryaw)
        goal.goal.header = goal.start.header
        goal.goal.pose.position.x = float(gx)
        goal.goal.pose.position.y = float(gy)
        goal.goal.pose.orientation.z = math.sin(0.5 * gyaw)
        goal.goal.pose.orientation.w = math.cos(0.5 * gyaw)
        self._splice_pending = True
        self._switch_bridge_join_i = 0
        fut = self._compute_path_client.send_goal_async(goal)
        fut.add_done_callback(
            lambda f, g=generation, r=route: self._switch_bridge_goal_response(
                f, g, r))
        attr_s = (
            f' attract=({attract[0]:.1f},{attract[1]:.1f})'
            if attract is not None else ''
        )
        rep_s = (
            f' repel=({repel[0]:.1f},{repel[1]:.1f})'
            if repel is not None else ''
        )
        self.get_logger().info(
            f'Switch escape hop #{self._escape_hop_count} → ({gx:.2f},{gy:.2f}) '
            f'gap={math.hypot(rx - gx, ry - gy):.1f}m{attr_s}{rep_s}'
        )
        return True

    def _maybe_log_entry083_block_diag(self, route: RouteInstance) -> None:
        """Phase 0: one-shot Switch NavFn seating diagnostic (Entry 083).

        Logs robot pose, distances, global start cost, keepout disk counts,
        and a cheap free-component note. No behavior change.
        """
        if getattr(self, '_entry083_block_diag_logged', False):
            return
        if self._robot_xyyaw is None:
            return
        self._entry083_block_diag_logged = True
        rx, ry, _ = self._robot_xyyaw
        locus = self._path_a_repel_xy()
        barrier = None
        if self._demo_obstacle_poses:
            barrier = min(
                self._demo_obstacle_poses,
                key=lambda p: (p[0] - rx) * (p[0] - rx) + (p[1] - ry) * (p[1] - ry),
            )
        goal = None
        if self._goal_xyyaw is not None:
            goal = (float(self._goal_xyyaw[0]), float(self._goal_xyyaw[1]))
        path_b = None
        if route is not None and route.points:
            path_b = min(
                route.points[:: max(1, len(route.points) // 80)],
                key=lambda p: (p[0] - rx) * (p[0] - rx) + (p[1] - ry) * (p[1] - ry),
            )

        def _dist(xy: Optional[Tuple[float, float]]) -> float:
            if xy is None:
                return float('nan')
            return math.hypot(rx - xy[0], ry - xy[1])

        start_cost = None
        start_class = 'unknown'
        free_comp = 'n/a'
        live = None
        if self._live_costmap is not None:
            try:
                live = nav2_costmap_to_grid(self._live_costmap)
                idx = live.world_to_grid(rx, ry)
                if idx is not None:
                    start_cost = int(live.costs[idx[1], idx[0]])
                    thr = int(self._cfg.lethal_threshold)
                    if start_cost >= thr:
                        start_class = 'lethal'
                    elif start_cost >= 253:
                        start_class = 'inscribed'
                    elif start_cost == 255:
                        start_class = 'unknown_cell'
                    else:
                        start_class = 'free'
                # Cheap BFS free-component vs Path-B / GOAL (bounded cells).
                free_comp = self._entry083_free_component_note(
                    live, (rx, ry), path_b, goal)
            except Exception as exc:  # noqa: BLE001
                free_comp = f'error:{exc}'

        keepout = {1.0: 0, 2.5: 0, 4.0: 0}
        mask = self._memory_mask
        if mask is not None and mask.data:
            try:
                info = mask.info
                w, h = int(info.width), int(info.height)
                res = float(info.resolution)
                ox = float(info.origin.position.x)
                oy = float(info.origin.position.y)
                data = mask.data
                for r in (1.0, 2.5, 4.0):
                    r2 = r * r
                    r_cells = max(1, int(math.ceil(r / res)))
                    cx = int((rx - ox) / res)
                    cy = int((ry - oy) / res)
                    n = 0
                    for dy in range(-r_cells, r_cells + 1):
                        for dx in range(-r_cells, r_cells + 1):
                            ix, iy = cx + dx, cy + dy
                            if not (0 <= ix < w and 0 <= iy < h):
                                continue
                            if int(data[iy * w + ix]) < 50:
                                continue
                            wx = ox + (ix + 0.5) * res
                            wy = oy + (iy + 0.5) * res
                            if (wx - rx) * (wx - rx) + (wy - ry) * (wy - ry) <= r2:
                                n += 1
                    keepout[r] = n
            except Exception:  # noqa: BLE001
                pass

        lines = [
            'Entry 083 Phase 0 block diag (first Switch NavFn)',
            f'robot_xy=({rx:.3f},{ry:.3f})',
            f'locus={locus} dist_locus={_dist(locus):.3f}',
            f'barrier={barrier} dist_barrier={_dist(barrier):.3f}',
            f'goal={goal} dist_goal={_dist(goal):.3f}',
            f'path_b={path_b} dist_path_b={_dist(path_b):.3f}',
            f'start_cost={start_cost} start_class={start_class}',
            f'keepout_counts_1_2p5_4m='
            f'{keepout[1.0]}/{keepout[2.5]}/{keepout[4.0]}',
            f'free_component={free_comp}',
            f'switch_standoff_min_m={SWITCH_STANDOFF_MIN_M} '
            f'keepout_seat_disk_m={KEEPOUT_SEAT_DISK_M} '
            f'early_switch_horizon_m={EARLY_SWITCH_HORIZON_M}',
        ]
        text = '\n'.join(lines) + '\n'
        try:
            os.makedirs(ENTRY083_BLOCK_DIAG_DIR, exist_ok=True)
            path = os.path.join(
                ENTRY083_BLOCK_DIAG_DIR, 'first_switch_navfn.txt')
            with open(path, 'w', encoding='utf-8') as fh:
                fh.write(text)
            self.get_logger().info(
                f'Entry083 Phase0 diag written → {path}\n{text.strip()}'
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f'Entry083 Phase0 diag file failed ({exc}); tee:\n{text.strip()}'
            )

    def _entry083_free_component_note(
        self,
        live,
        robot_xy: Tuple[float, float],
        path_b: Optional[Tuple[float, float]],
        goal: Optional[Tuple[float, float]],
        *,
        max_cells: int = 8000,
    ) -> str:
        """Bounded BFS: is robot free-connected to Path-B / GOAL?"""
        thr = int(self._cfg.lethal_threshold)
        start = live.world_to_grid(robot_xy[0], robot_xy[1])
        if start is None:
            return 'robot_off_grid'
        sx, sy = start
        if int(live.costs[sy, sx]) >= thr:
            return 'robot_start_lethal'
        targets = {}
        if path_b is not None:
            tb = live.world_to_grid(path_b[0], path_b[1])
            if tb is not None:
                targets['path_b'] = tb
        if goal is not None:
            tg = live.world_to_grid(goal[0], goal[1])
            if tg is not None:
                targets['goal'] = tg
        if not targets:
            return 'no_targets'
        seen = {(sx, sy)}
        q = deque([(sx, sy)])
        found = {k: False for k in targets}
        cells = 0
        while q and cells < max_cells:
            cx, cy = q.popleft()
            cells += 1
            for key, (tx, ty) in targets.items():
                if (cx, cy) == (tx, ty):
                    found[key] = True
            if all(found.values()):
                break
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < live.width and 0 <= ny < live.height):
                    continue
                if (nx, ny) in seen:
                    continue
                if int(live.costs[ny, nx]) >= thr:
                    continue
                seen.add((nx, ny))
                q.append((nx, ny))
        parts = [f'{k}={"yes" if v else "no"}' for k, v in found.items()]
        truncated = 'truncated' if cells >= max_cells and not all(found.values()) else 'ok'
        return f'{",".join(parts)} cells={cells} ({truncated})'

    def _request_switch_navfn_bridge(
        self, route: RouteInstance, *, generation: int
    ) -> bool:
        """NavFn robot→Path-B join, then FollowPath bridge+forward route.

        Straight densify bridges cross lethal forest between bush_trail Path A/B
        (Entry 034 block: 16 m bridge → cost-regulated stall → progress abort).
        """
        if self._compute_path_client is None or self._robot_xyyaw is None:
            return False
        if self._splice_pending:
            return False
        if not self._compute_path_client.server_is_ready():
            return False
        # Entry 083 Phase 0: capture seating once at first Switch NavFn.
        self._maybe_log_entry083_block_diag(route)
        # Entry 082: LIDAR-only start unseal for global KeepoutFilter (FIRE kept).
        self._request_footprint_escape_clear()
        if not getattr(self, '_switch_join_queue', None):
            self._switch_join_queue = self._join_candidates(route)
            self._switch_join_attempt = 0
        if not self._switch_join_queue:
            return False
        if self._switch_join_attempt >= len(self._switch_join_queue):
            self._switch_join_attempt = 0  # wrap once more for stuck retries
        gx, gy, gyaw, join_i = self._switch_join_queue[self._switch_join_attempt]
        self._switch_join_attempt += 1
        rx, ry, ryaw = self._robot_xyyaw
        goal = ComputePathToPose.Goal()
        goal.planner_id = 'GridBased'
        goal.use_start = True
        goal.start.header.frame_id = self._map_frame
        goal.start.header.stamp = self.get_clock().now().to_msg()
        goal.start.pose.position.x = float(rx)
        goal.start.pose.position.y = float(ry)
        goal.start.pose.orientation.z = math.sin(0.5 * ryaw)
        goal.start.pose.orientation.w = math.cos(0.5 * ryaw)
        goal.goal.header = goal.start.header
        goal.goal.pose.position.x = float(gx)
        goal.goal.pose.position.y = float(gy)
        goal.goal.pose.orientation.z = math.sin(0.5 * gyaw)
        goal.goal.pose.orientation.w = math.cos(0.5 * gyaw)
        self._splice_pending = True
        self._switch_bridge_join_i = join_i
        fut = self._compute_path_client.send_goal_async(goal)
        fut.add_done_callback(
            lambda f, g=generation, r=route: self._switch_bridge_goal_response(
                f, g, r))
        self.get_logger().info(
            f'Switch NavFn bridge → ({gx:.2f},{gy:.2f}) join_i={join_i} '
            f'path_id={route.path_id} gap={math.hypot(rx - gx, ry - gy):.1f}m '
            f'(cand {self._switch_join_attempt}/{len(self._switch_join_queue)})'
        )
        return True

    def _request_switch_navfn_goal_fallback(
        self, route: RouteInstance, *, generation: int
    ) -> bool:
        """Last-resort NavFn robot→mission goal when all Path-B joins fail."""
        if self._goal_xyyaw is None or self._robot_xyyaw is None:
            return False
        if self._compute_path_client is None or self._splice_pending:
            return False
        if not self._compute_path_client.server_is_ready():
            return False
        # Entry 083 Phase 0: still capture once if join never armed.
        self._maybe_log_entry083_block_diag(route)
        self._switch_goal_fallback_used = True
        self._switch_bridge_goal_only = True
        gx, gy, gyaw = self._goal_xyyaw
        rx, ry, ryaw = self._robot_xyyaw
        goal = ComputePathToPose.Goal()
        goal.planner_id = 'GridBased'
        goal.use_start = True
        goal.start.header.frame_id = self._map_frame
        goal.start.header.stamp = self.get_clock().now().to_msg()
        goal.start.pose.position.x = float(rx)
        goal.start.pose.position.y = float(ry)
        goal.start.pose.orientation.z = math.sin(0.5 * ryaw)
        goal.start.pose.orientation.w = math.cos(0.5 * ryaw)
        goal.goal.header = goal.start.header
        goal.goal.pose.position.x = float(gx)
        goal.goal.pose.position.y = float(gy)
        goal.goal.pose.orientation.z = math.sin(0.5 * gyaw)
        goal.goal.pose.orientation.w = math.cos(0.5 * gyaw)
        self._splice_pending = True
        self._switch_bridge_join_i = max(0, len(route.points) - 1)
        fut = self._compute_path_client.send_goal_async(goal)
        fut.add_done_callback(
            lambda f, g=generation, r=route: self._switch_bridge_goal_response(
                f, g, r))
        self.get_logger().warn(
            f'Switch NavFn GOAL fallback → ({gx:.2f},{gy:.2f}) '
            f'gap={math.hypot(rx - gx, ry - gy):.1f}m '
            f'(join bridge unavailable or exhausted)'
        )
        return True

    def _switch_bridge_goal_response(
        self, future, generation: int, route: RouteInstance
    ) -> None:
        if generation != self._owner.generation:
            self._splice_pending = False
            return
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self._splice_pending = False
            self.get_logger().warn(f'Switch bridge goal failed: {exc}')
            # Entry 084 R2: stem → refuse ASYNC_BRIDGE (no bare densify send).
            if self._route_is_spawn_stem_snap(route):
                self._refuse_bare_stem_follow_path(
                    route,
                    generation=generation,
                    context=RefuseContext.ASYNC_BRIDGE,
                )
                return
            self._send_follow_path(route, expected_generation=generation)
            return
        if handle is None or not handle.accepted:
            self._splice_pending = False
            self.get_logger().warn('Switch bridge ComputePathToPose rejected')
            # Entry 084 R3: stem → refuse; never densify bare FP.
            if self._route_is_spawn_stem_snap(route):
                self._refuse_bare_stem_follow_path(
                    route,
                    generation=generation,
                    context=RefuseContext.ASYNC_BRIDGE,
                )
                return
            self._send_follow_path(route, expected_generation=generation)
            return
        result_fut = handle.get_result_async()
        result_fut.add_done_callback(
            lambda f, g=generation, r=route: self._switch_bridge_result(f, g, r))

    def _switch_bridge_result(
        self, future, generation: int, route: RouteInstance
    ) -> None:
        self._splice_pending = False
        if generation != self._owner.generation:
            return
        try:
            wrapped = future.result()
            status = wrapped.status
            result = wrapped.result
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Switch bridge result failed: {exc}')
            # Entry 084 R4: stem → refuse; no densify.
            if self._route_is_spawn_stem_snap(route):
                self._refuse_bare_stem_follow_path(
                    route,
                    generation=generation,
                    context=RefuseContext.ASYNC_BRIDGE,
                )
                return
            self._send_follow_path(route, expected_generation=generation)
            return
        if status != GoalStatus.STATUS_SUCCEEDED or result is None:
            gap = None
            if self._robot_xyyaw is not None:
                nearest = self._nearest_route_pose(route)
                if nearest is not None:
                    gap = math.hypot(
                        self._robot_xyyaw[0] - nearest[0],
                        self._robot_xyyaw[1] - nearest[1],
                    )
            stem = self._route_is_spawn_stem_snap(route)
            # Entry 036/082/084: rotate join/GOAL for far-gap OR stem (even
            # when measured gap ≤5). Stem never falls through to densify.
            if stem or (gap is not None and gap > 5.0):
                q = getattr(self, '_switch_join_queue', None) or []
                attempt = int(getattr(self, '_switch_join_attempt', 0))
                if attempt < len(q):
                    self.get_logger().warn(
                        f'Switch bridge ComputePathToPose status={status}; '
                        f'trying next join candidate '
                        f'({attempt + 1}/{len(q)}'
                        + (f'; gap={gap:.1f}m' if gap is not None else '')
                        + ('; stem-snap' if stem else '')
                        + ')'
                    )
                    if self._request_switch_navfn_bridge(
                        route, generation=generation
                    ):
                        return
                if not getattr(self, '_switch_goal_fallback_used', False):
                    if self._request_switch_navfn_goal_fallback(
                        route, generation=generation
                    ):
                        return
                self.get_logger().warn(
                    f'Switch bridge ComputePathToPose status={status}; '
                    f'join candidates exhausted/skip densify'
                    + (f' (gap={gap:.1f}m)' if gap is not None else '')
                    + (' [stem-snap]' if stem else '')
                    + ' — escalate (Entry 082/084)'
                )
                self._switch_join_queue = None
                # Allow goal-fallback again on the next rebuild cycle.
                self._switch_goal_fallback_used = False
                if stem:
                    # Entry 084 R5/R8: refuse ASYNC_BRIDGE (Inc → exhaust).
                    self._refuse_bare_stem_follow_path(
                        route,
                        generation=generation,
                        context=RefuseContext.ASYNC_BRIDGE,
                    )
                else:
                    self._on_switch_bridge_exhausted(
                        route,
                        generation=generation,
                        gap=gap,
                        stem_snap=False,
                    )
                return
            # Entry 084 R7: non-stem gap≤5 densify only.
            self.get_logger().warn(
                f'Switch bridge ComputePathToPose status={status}; '
                f'falling back to densify bridge'
                + (f' (gap={gap:.1f}m)' if gap is not None else '')
            )
            self._send_follow_path(
                route, expected_generation=generation, allow_navfn_bridge=False)
            return
        bridge = result.path
        if bridge is None or len(bridge.poses) < 2:
            # Entry 084 R9/R10: stem → refuse; else densify.
            if self._route_is_spawn_stem_snap(route):
                self._refuse_bare_stem_follow_path(
                    route,
                    generation=generation,
                    context=RefuseContext.ASYNC_BRIDGE,
                )
                return
            self._send_follow_path(
                route, expected_generation=generation, allow_navfn_bridge=False)
            return
        # Entry 070: reject unexecutable / absurd NavFn bridges; nudge survivors.
        bridge_xy = [
            (float(ps.pose.position.x), float(ps.pose.position.y))
            for ps in bridge.poses
        ]
        gap_chord = 0.0
        if self._robot_xyyaw is not None and bridge_xy:
            gap_chord = math.hypot(
                self._robot_xyyaw[0] - bridge_xy[-1][0],
                self._robot_xyyaw[1] - bridge_xy[-1][1],
            )
        detour, detour_why = bridge_detour_excessive(
            bridge_xy, gap_m=gap_chord)
        if detour:
            self.get_logger().warn(
                f'Switch NavFn bridge rejected ({detour_why}); '
                'trying next join candidate'
            )
            if self._try_next_switch_bridge(route, generation):
                return
            return
        live = None
        if self._live_costmap is not None:
            live = nav2_costmap_to_grid(self._live_costmap)

            def _cost_at(x: float, y: float):
                idx = live.world_to_grid(x, y)
                if idx is None:
                    return None
                return int(live.costs[idx[1], idx[0]])

            blocked, why = bridge_prefix_blocked(bridge_xy, _cost_at)
            if blocked:
                self.get_logger().warn(
                    f'Switch NavFn bridge prefix blocked ({why}); '
                    'trying next join candidate'
                )
                if self._try_next_switch_bridge(route, generation):
                    return
                return
            # Soft-nudge bridge off knife-edge inflation so MPPI can track it.
            bridge_xy = nudge_polyline_away_from_lethal(
                bridge_xy,
                live,
                target_clearance_m=0.55,
                max_offset_m=0.90,
            )
        bridge_yaws = along_track_yaw(bridge_xy)
        path_msg = Path()
        path_msg.header.frame_id = self._map_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for (x, y), yaw in zip(bridge_xy, bridge_yaws):
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            half = 0.5 * float(yaw)
            ps.pose.orientation.z = math.sin(half)
            ps.pose.orientation.w = math.cos(half)
            path_msg.poses.append(ps)
        if getattr(self, '_switch_escape_only', False):
            # Short hop only — do not append Path-B route yet.
            self._switch_escape_only = False
            self._escape_hop_pending_join = True
            self.get_logger().info(
                f'Switch escape hop OK poses={len(path_msg.poses)} '
                f'route_uid={route.route_uid}'
            )
            settle = 12.0
        elif getattr(self, '_switch_bridge_goal_only', False):
            # Mission-goal fallback: follow NavFn to goal only (Path-B joins
            # unavailable/exhausted — Entry 036/070).
            self._switch_bridge_goal_only = False
            self.get_logger().info(
                f'Switch NavFn GOAL fallback OK poses={len(path_msg.poses)} '
                f'route_uid={route.route_uid}'
            )
            settle = 45.0
        else:
            join_i = int(getattr(self, '_switch_bridge_join_i', 0))
            join_i = max(0, min(join_i, len(route.points) - 1))
            # Append forward RouteInstance from join (skip duplicate join pose).
            for (x, y), yaw in zip(
                route.points[join_i + 1 :], route.yaw_rad[join_i + 1 :]
            ):
                ps = PoseStamped()
                ps.header = path_msg.header
                ps.pose.position.x = float(x)
                ps.pose.position.y = float(y)
                half = 0.5 * float(yaw)
                ps.pose.orientation.z = math.sin(half)
                ps.pose.orientation.w = math.cos(half)
                path_msg.poses.append(ps)
            self.get_logger().info(
                f'Switch NavFn bridge OK bridge_poses={len(bridge_xy)} '
                f'total={len(path_msg.poses)} route_uid={route.route_uid}'
            )
            settle = 16.0
        # Do NOT clear _abort_resend_used here. Entry 036 thrash:
        # abort → NavFn → clear flag → immediate abort → NavFn at ~10 Hz
        # never settled FollowPath. Intentional Path A→B switch already
        # re-arms the flag in _switch_follow_path.
        # Brief grace so controller can settle before ownership/stuck resend.
        # Entry 049 / Issue B R3: far-gap GOAL/bridge needs a long settle so
        # stuck recovery does not cancel NavFn at the first 8–16 s boundary.
        if getattr(self, '_switch_goal_fallback_used', False):
            settle = max(settle, 45.0)
        self._switch_grace_until = max(
            self._switch_grace_until, time.monotonic() + settle)
        self._forward_only_until = max(
            self._forward_only_until, time.monotonic() + settle)
        self._publish_forward_only()
        self._stuck_recoveries = 0
        self._last_progress_t = time.monotonic()
        if self._robot_xyyaw is not None:
            self._last_progress_xy = (
                self._robot_xyyaw[0], self._robot_xyyaw[1])
        self._send_follow_path_msg(
            path_msg, route_uid=route.route_uid, path_id=route.path_id)

    def _route_join_yaw(self, route: RouteInstance) -> Optional[float]:
        """Path tangent yaw ~PATH_YAW_LOOKAHEAD_M along robot-forward join."""
        if route is None or not route.points:
            return None
        pts, yaws = self._route_forward_from_robot(route)
        if len(pts) < 1:
            pts, yaws = route.points, route.yaw_rad
        if len(pts) >= 2:
            x0, y0 = float(pts[0][0]), float(pts[0][1])
            acc = 0.0
            look = pts[1]
            prev = pts[0]
            for p in pts[1:]:
                step = math.hypot(p[0] - prev[0], p[1] - prev[1])
                if step < 1e-6:
                    continue
                acc += step
                look = p
                prev = p
                if acc >= float(PATH_YAW_LOOKAHEAD_M):
                    break
            return math.atan2(look[1] - y0, look[0] - x0)
        if yaws:
            return float(yaws[0])
        return None

    def _path_msg_join_yaw(self, path_msg: Path) -> Optional[float]:
        """Yaw of the path chord ~PATH_YAW_LOOKAHEAD_M ahead (not pose[0]→[1]).

        NavFn / GOAL-fallback paths start at the robot, so the first segment is
        nearly aligned with current heading and would vacuous-pass the yaw gate
        while later geometry still demands reverse after forward_only expires.
        """
        if path_msg is None or len(path_msg.poses) < 1:
            return None
        poses = path_msg.poses
        if len(poses) < 2:
            q = poses[0].pose.orientation
            return math.atan2(2.0 * q.z * q.w, 1.0 - 2.0 * q.z * q.z)
        p0 = poses[0].pose.position
        x0, y0 = float(p0.x), float(p0.y)
        target = float(PATH_YAW_LOOKAHEAD_M)
        acc = 0.0
        prev_x, prev_y = x0, y0
        look_x, look_y = float(poses[1].pose.position.x), float(poses[1].pose.position.y)
        for ps in poses[1:]:
            x = float(ps.pose.position.x)
            y = float(ps.pose.position.y)
            step = math.hypot(x - prev_x, y - prev_y)
            if step < 1e-6:
                continue
            acc += step
            look_x, look_y = x, y
            prev_x, prev_y = x, y
            if acc >= target:
                break
        if abs(look_x - x0) < 1e-9 and abs(look_y - y0) < 1e-9:
            return None
        return math.atan2(look_y - y0, look_x - x0)

    def _mission_yaw_error_for_target(
        self, target_yaw: Optional[float]
    ) -> Optional[float]:
        if target_yaw is None or self._robot_xyyaw is None:
            return None
        return yaw_error(self._robot_xyyaw[2], float(target_yaw))

    def _abort_reorient(self, reason: str) -> None:
        """Cancel in-flight Spin / clear reorient state; restore vx_min."""
        self._spin_action_token += 1
        if self._spin_goal_handle is not None:
            try:
                self._spin_goal_handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
            self._spin_goal_handle = None
        was_active = self._reorient_active or self._reorient_vx_min_asserted
        self._reorient_active = False
        self._reorient_phase = None
        self._reorient_route = None
        self._reorient_generation = None
        self._reorient_path_msg = None
        self._reorient_allow_fp = False
        self._restore_follow_path_vx_min(reason=reason)
        if was_active:
            self.get_logger().info(f'Reorient aborted ({reason})')

    def _restore_follow_path_vx_min(self, *, reason: str) -> None:
        """Restore prior FollowPath.vx_min within restore deadline (all exits)."""
        if not self._reorient_vx_min_asserted:
            self._reorient_saved_vx_min = None
            return
        prior = (
            FOLLOW_PATH_VX_MIN_DEFAULT
            if self._reorient_saved_vx_min is None
            else float(self._reorient_saved_vx_min)
        )
        self._reorient_vx_min_asserted = False
        self._reorient_saved_vx_min = None
        t0 = time.monotonic()
        if not self._controller_set_cli.service_is_ready():
            self.get_logger().warn(
                f'Cannot restore FollowPath.vx_min ({reason}): '
                'controller_server set_parameters not ready'
            )
            return
        req = SetParameters.Request()
        req.parameters = [
            Parameter(
                FOLLOW_PATH_VX_MIN_PARAM, Parameter.Type.DOUBLE, prior,
            ).to_parameter_msg()
        ]
        self._controller_set_cli.call_async(req)
        elapsed = time.monotonic() - t0
        self.get_logger().info(
            f'Restored FollowPath.vx_min={prior:.2f} ({reason}) '
            f'in {elapsed:.3f}s (deadline {VX_MIN_RESTORE_DEADLINE_S}s)'
        )

    def _assert_follow_path_vx_min_zero(self) -> bool:
        """Save current FollowPath.vx_min and assert 0.0 for fallback rotate."""
        prior = FOLLOW_PATH_VX_MIN_DEFAULT
        if self._controller_get_cli.service_is_ready():
            get_req = GetParameters.Request()
            get_req.names = [FOLLOW_PATH_VX_MIN_PARAM]
            # Best-effort sync wait bounded — fallback must not hang forever.
            fut = self._controller_get_cli.call_async(get_req)
            t0 = time.monotonic()
            while not fut.done() and (time.monotonic() - t0) < 0.4:
                time.sleep(0.01)
            if fut.done():
                try:
                    resp = fut.result()
                    if resp and resp.values:
                        prior = float(resp.values[0].double_value)
                except Exception:  # noqa: BLE001
                    pass
        self._reorient_saved_vx_min = prior
        if not self._controller_set_cli.service_is_ready():
            self.get_logger().warn(
                'controller_server set_parameters not ready — '
                'cannot assert FollowPath.vx_min=0'
            )
            return False
        req = SetParameters.Request()
        req.parameters = [
            Parameter(
                FOLLOW_PATH_VX_MIN_PARAM, Parameter.Type.DOUBLE, 0.0,
            ).to_parameter_msg()
        ]
        self._controller_set_cli.call_async(req)
        self._reorient_vx_min_asserted = True
        self.get_logger().warn(
            f'Fallback asserted FollowPath.vx_min=0 '
            f'(saved prior={prior:.2f})'
        )
        return True

    def _arm_reorient_forward_only(self, hold_s: float) -> None:
        # Never shorten an existing far-gap clamp (Issue B R4: finish_reorient
        # used to overwrite a 45 s hold with POST_REORIENT 8 s → reverse resumed).
        self._forward_only_until = max(
            self._forward_only_until,
            time.monotonic() + max(hold_s, 8.0),
        )
        self._publish_forward_only()

    def _start_mission_reorient(
        self,
        *,
        route: Optional[RouteInstance],
        path_msg: Optional[Path] = None,
        route_uid: str = '',
        path_id: int = 0,
        expected_generation: Optional[int] = None,
        allow_navfn_bridge: bool = True,
        yaw_err: float,
    ) -> None:
        """PRIMARY Spin (or fallback) — never submit FollowPath while misaligned."""
        if self._reorient_active:
            self.get_logger().info(
                'Reorient already active — skipping duplicate start'
            )
            return
        self._reorient_active = True
        self._reorient_phase = 'spin'
        self._reorient_route = route
        self._reorient_path_msg = path_msg
        self._reorient_path_uid = route_uid
        self._reorient_path_id = path_id
        self._reorient_generation = (
            expected_generation
            if expected_generation is not None
            else self._owner.generation
        )
        self._reorient_allow_navfn = allow_navfn_bridge
        self._reorient_started_at = time.monotonic()
        self._arm_reorient_forward_only(SPIN_TIMEOUT_S + FALLBACK_TIMEOUT_S)
        # Clear stale local lethals before Spin — Path-A barrier keepout made
        # behavior_server exit Spin with "Collision Ahead" in <1 s (Issue B).
        self._clear_local_costmap()
        self.get_logger().info(
            f'Reorient PRIMARY Spin yaw_err={yaw_err:.2f} rad '
            f'(θ_exit={THETA_EXIT_RAD}, θ_reorient={THETA_REORIENT_RAD}) '
            f'path_id={path_id or (route.path_id if route else 0)}'
        )
        if not self._dispatch_spin(yaw_err, timeout_s=SPIN_TIMEOUT_S):
            self.get_logger().warn(
                'Spin unavailable/reject — entering vx_min=0 fallback'
            )
            self._enter_reorient_fallback(yaw_err)

    def _dispatch_spin(self, yaw_err: float, *, timeout_s: float) -> bool:
        if self._spin_client is None:
            return False
        if not self._spin_client.server_is_ready():
            return False
        goal = Spin.Goal()
        goal.target_yaw = float(yaw_err)
        sec = int(timeout_s)
        nanosec = int((timeout_s - sec) * 1e9)
        goal.time_allowance = Duration(sec=sec, nanosec=nanosec)
        gen = self._reorient_generation
        self._spin_action_token += 1
        spin_token = self._spin_action_token
        fut = self._spin_client.send_goal_async(goal)
        fut.add_done_callback(
            lambda f, g=gen, t=spin_token: self._spin_goal_response(f, g, t))
        return True

    def _spin_goal_response(
        self, future, generation: Optional[int], spin_token: Optional[int] = None
    ) -> None:
        if not self._reorient_active:
            return
        if spin_token is not None and spin_token != self._spin_action_token:
            try:
                future.result()
            except Exception:
                pass
            self.get_logger().info(
                f'Ignoring stale Spin acceptance token={spin_token}')
            return
        if generation is not None and generation != self._owner.generation:
            self._abort_reorient('generation_during_spin_accept')
            return
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Spin send failed: {exc}')
            self._on_spin_failed_or_timeout()
            return
        if handle is None or not handle.accepted:
            self.get_logger().warn('Spin goal rejected')
            self._on_spin_failed_or_timeout()
            return
        self._spin_goal_handle = handle
        result_fut = handle.get_result_async()
        result_fut.add_done_callback(
            lambda f, g=generation, t=spin_token: self._spin_goal_result(
                f, g, t))

    def _spin_goal_result(
        self, future, generation: Optional[int], spin_token: Optional[int] = None
    ) -> None:
        if not self._reorient_active:
            return
        if spin_token is not None and spin_token != self._spin_action_token:
            try:
                future.result()
            except Exception:
                pass
            self.get_logger().info(
                f'Ignoring stale Spin result token={spin_token}')
            return
        if generation is not None and generation != self._owner.generation:
            self._abort_reorient('generation_during_spin_result')
            return
        self._spin_goal_handle = None
        try:
            wrapped = future.result()
            status = wrapped.status
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Spin result failed: {exc}')
            self._on_spin_failed_or_timeout()
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f'Spin status={status} — not aligned success')
            self._on_spin_failed_or_timeout()
            return
        self._on_reorient_spin_complete()

    def _current_reorient_yaw_err(self) -> Optional[float]:
        if self._reorient_path_msg is not None:
            tgt = self._path_msg_join_yaw(self._reorient_path_msg)
            return self._mission_yaw_error_for_target(tgt)
        if self._reorient_route is not None:
            tgt = self._route_join_yaw(self._reorient_route)
            return self._mission_yaw_error_for_target(tgt)
        return None

    def _on_reorient_spin_complete(self) -> None:
        err = self._current_reorient_yaw_err()
        if err is not None and yaw_aligned(err):
            self.get_logger().info(
                f'Reorient yaw OK (|err|={abs(err):.2f} < {THETA_EXIT_RAD}) '
                f'→ FollowPath'
            )
            self._finish_reorient_and_submit()
            return
        self.get_logger().warn(
            f'Spin finished but yaw still misaligned '
            f'(err={None if err is None else f"{err:.2f}"})'
        )
        self._on_spin_failed_or_timeout()

    def _on_spin_failed_or_timeout(self) -> None:
        err = self._current_reorient_yaw_err()
        if err is not None and yaw_aligned(err):
            self._finish_reorient_and_submit()
            return
        if self._reorient_phase == 'spin':
            yaw = 0.0 if err is None else float(err)
            self._enter_reorient_fallback(yaw)
            return
        # Fallback also failed — fail-safe abort; NEVER misaligned FollowPath.
        self.get_logger().error(
            'Reorient fallback exhausted with |yaw| ≥ θ_exit — '
            'fail-safe abort (no FollowPath); stuck/join may retry'
        )
        self._abort_reorient('fallback_timeout_misaligned')
        self._stuck_recover_until = time.monotonic() + 2.0

    def _enter_reorient_fallback(self, yaw_err: float) -> None:
        self._reorient_phase = 'fallback'
        self._reorient_started_at = time.monotonic()
        self._arm_reorient_forward_only(FALLBACK_TIMEOUT_S)
        if not self._assert_follow_path_vx_min_zero():
            self.get_logger().error(
                'Fallback cannot assert vx_min=0 — fail-safe abort (no FP)'
            )
            self._abort_reorient('fallback_vx_min_unavailable')
            return
        # R3-N1: prefer second Spin under clamp (no bank Twist publisher).
        if not self._dispatch_spin(yaw_err, timeout_s=FALLBACK_TIMEOUT_S):
            self.get_logger().error(
                'Fallback second Spin unavailable — fail-safe abort (no FP)'
            )
            self._abort_reorient('fallback_spin_unavailable')

    def _finish_reorient_and_submit(self) -> None:
        route = self._reorient_route
        path_msg = self._reorient_path_msg
        uid = self._reorient_path_uid
        path_id = self._reorient_path_id
        gen = self._reorient_generation
        allow_navfn = self._reorient_allow_navfn
        self._restore_follow_path_vx_min(reason='heading_ok')
        self._reorient_active = False
        self._reorient_phase = None
        self._reorient_route = None
        self._reorient_path_msg = None
        self._reorient_generation = None
        self._reorient_allow_fp = True
        self._arm_reorient_forward_only(POST_REORIENT_FORWARD_ONLY_S)
        if path_msg is not None:
            self._send_follow_path_msg(
                path_msg, route_uid=uid, path_id=path_id)
            return
        if route is not None:
            # If still far from the RouteInstance, NavFn will replace geometry —
            # do NOT waive the yaw gate with allow_fp (bridge must re-gate).
            if (
                allow_navfn
                and self._robot_xyyaw is not None
                and route.points
            ):
                rx, ry, _ = self._robot_xyyaw
                nearest = min(
                    math.hypot(rx - x, ry - y)
                    for x, y in route.points[
                        :: max(1, len(route.points) // 100)
                    ]
                )
                if nearest > 5.0:
                    self._reorient_allow_fp = False
            self._send_follow_path(
                route,
                expected_generation=gen,
                allow_navfn_bridge=allow_navfn,
            )

    def _note_follow_path_motion_submit(self) -> bool:
        """Reset the stuck clock on a FollowPath submit that is actually driving.

        StartMission stamps ``_last_progress_t`` before Dijkstra (~60s). The
        same timer tick that submits the first FollowPath then runs
        ``_maybe_owner_stuck_recover``. Without this reset that watchdog
        cancels the new goal as ``no_progress``.

        No-op while ``_reorient_active`` (Spin / fallback still owns the
        robot). Callers must invoke this only after the yaw gate has either
        passed or returned into Spin — never to skip Spin or to send
        FollowPath while misaligned.
        """
        if self._reorient_active:
            return False
        self._last_progress_t = time.monotonic()
        if self._robot_xyyaw is not None:
            self._last_progress_xy = (
                float(self._robot_xyyaw[0]), float(self._robot_xyyaw[1]))
        else:
            self._last_progress_xy = None
        if (self._owner.detail or '') == 'PENDING_SEED':
            # Banner must not stay on the pre-seed string after motion starts.
            self._owner.detail = ''
            self._publish_owner_status()
        return True

    def _send_follow_path_msg(
        self,
        path_msg: Path,
        *,
        route_uid: str,
        path_id: int,
    ) -> None:
        """Low-level FollowPath submit from an arbitrary Path message."""
        if not self._arm_nav_authority():
            return
        if self._fp_client is None:
            return
        if not self._fp_client.server_is_ready():
            return
        # Issue B: mission yaw gate on bridge/splice submits too.
        if (
            self._nav_mode == 'mission'
            and not self._reorient_allow_fp
            and self._robot_xyyaw is not None
        ):
            tgt = self._path_msg_join_yaw(path_msg)
            err = self._mission_yaw_error_for_target(tgt)
            if err is not None and not yaw_aligned(err):
                self._start_mission_reorient(
                    route=None,
                    path_msg=path_msg,
                    route_uid=route_uid,
                    path_id=path_id,
                    expected_generation=self._owner.generation,
                    allow_navfn_bridge=False,
                    yaw_err=err,
                )
                return
        self._reorient_allow_fp = False
        path_msg.header.frame_id = self._map_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for ps in path_msg.poses:
            ps.header = path_msg.header
        # Entry 063 A1: cache exact submitted Goal.path (accept publishes this).
        self._last_submitted_path = path_msg
        goal = FollowPath.Goal()
        goal.path = path_msg
        goal.controller_id = 'FollowPath'
        gen = self._owner.generation
        self._fp_submit_generation = gen
        if self._fp_goal_handle is not None:
            self._fp_cancelled_epochs.add(int(self._fp_goal_epoch))
            self._ignore_fp_results += 1
        self._fp_goal_epoch += 1
        epoch = int(self._fp_goal_epoch)
        # After the yaw-gate return above. Spin completion and the aligned
        # no-Spin path both land here with _reorient_active false, including
        # the timer tick that also runs _maybe_owner_stuck_recover.
        clock_reset = self._note_follow_path_motion_submit()
        self._fp_submit_t = time.monotonic()
        send_future = self._fp_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda f, g=gen, uid=route_uid, e=epoch: self._fp_goal_response(
                f, g, uid, e)
        )
        self.get_logger().info(
            f'FollowPath submitted route_uid={route_uid} path_id={path_id} '
            f'poses={len(path_msg.poses)} (splice_or_msg) gen={gen} epoch={epoch} '
            f'stuck_clock={"reset" if clock_reset else "held"}'
        )

    def _send_follow_path(
        self,
        route: RouteInstance,
        *,
        expected_generation: Optional[int] = None,
        allow_navfn_bridge: bool = True,
        refuse_context: RefuseContext = RefuseContext.CENTRAL_GATE_OTHER,
    ) -> None:
        """Submit frozen RouteInstance to controller FollowPath action."""
        if self._observation_hold:
            self.get_logger().info(
                'Skipping FollowPath submit — observation_hold active'
            )
            return
        if expected_generation is not None and expected_generation != self._owner.generation:
            self.get_logger().info(
                'Skipping FollowPath submit — generation fence mismatch'
            )
            return
        if not self._arm_nav_authority():
            self.get_logger().warn(
                'Refusing FollowPath submit — abort-resend / authority gate '
                f'(phase={self._owner.phase.value}, '
                f'latch={self._owner.operator_stop_latch}, '
                f'auth={self._owner.command_authority.value})'
            )
            return
        if self._fp_client is None:
            self.get_logger().error('FollowPath client not initialised')
            return
        if not self._fp_client.server_is_ready():
            self.get_logger().error('follow_path server not available')
            return

        gen = (
            expected_generation
            if expected_generation is not None
            else self._owner.generation
        )
        # Entry 084 stem/SPAWN refuse is a mission switch/replan contract.
        # Home return (T11) must still submit FollowPath from live pose —
        # do not treat home RouteInstance as Path-B stem-snap.
        home_submit = (
            self._nav_mode == 'home'
            or self._state in (
                MissionNavState.RETURNING_TO_START,
                MissionNavState.RETURNED_TO_START,
            )
            or route is getattr(self, '_home_route', None)
            or int(getattr(route, 'path_id', -1)) == 0
        )
        # Entry 084 R11: central stem gate BEFORE densify / SPAWN full-route.
        # Predicate is ONLY is_spawn_stem_snap (STEM_ROBOT_AWAY inside helper).
        if (
            not home_submit
            and self._robot_xyyaw is not None
            and route.points
            and self._route_is_spawn_stem_snap(route)
        ):
            self._refuse_bare_stem_follow_path(
                route, generation=gen, context=refuse_context)
            return

        # Submit robot-forward slice only — full route from mission start made
        # RPP collision-lookahead abort before the robot moved.
        pts, yaws = self._route_forward_from_robot(route)
        if len(pts) < 2:
            # Entry 084: mid-mission SPAWN full-route fallback forbidden when
            # robot is away from bank start (distance-only; not a 2nd stem gate).
            if (
                not home_submit
                and self._robot_xyyaw is not None
            ):
                rx, ry, _ = self._robot_xyyaw
                sx, sy = self._bank_start_xy(route)
                if math.hypot(rx - sx, ry - sy) >= STEM_ROBOT_AWAY_M:
                    self.get_logger().warn(
                        f'FollowPath refusing SPAWN full-route fallback '
                        f'(robot {math.hypot(rx - sx, ry - sy):.1f}m from '
                        f'bank_start; route_uid={route.route_uid})'
                    )
                    self._refuse_bare_stem_follow_path(
                        route, generation=gen, context=refuse_context)
                    return
            pts, yaws = route.points, route.yaw_rad
        # Path A→B: robot may be >10–20 m from the new corridor. Prefer NavFn
        # over straight densify (Entry 036: stuck recovery densify undid a good
        # switch NavFn bridge → lethal stall). Short gaps still densify.
        #
        # Issue B R2: for far-gap, request NavFn *before* yaw-gating on the
        # RouteInstance join. Reorienting to a 10 m-behind join, then replacing
        # the path with NavFn (different heading), caused Spin thrash
        # (route-yaw ↔ bridge-yaw) and reverse-then-recover. Yaw gate on the
        # bridged Path msg happens in _send_follow_path_msg instead.
        if self._robot_xyyaw is not None and pts:
            rx, ry, ryaw = self._robot_xyyaw
            x0, y0 = pts[0]
            gap = math.hypot(rx - x0, ry - y0)
            # Threshold 5 m (was 2): near-goal 2–3 m gaps thrashed NavFn↔abort
            # and exhausted stuck budget before arrival (Entry 036 block).
            if (
                allow_navfn_bridge
                and gap > 5.0
                and not self._splice_pending
            ):
                # Consume one-shot allow_fp only when we actually submit below;
                # far-gap NavFn must still yaw-gate the bridged path.
                if self._request_switch_navfn_bridge(route, generation=gen):
                    self.get_logger().info(
                        f'FollowPath far-gap {gap:.1f} m → NavFn bridge '
                        f'(route_uid={route.route_uid}; yaw-gate after bridge)'
                    )
                    return
                # Entry 049: if join bridge cannot start, try goal fallback
                # before orphaning FollowPath (prior path left fp_handle=None).
                if self._request_switch_navfn_goal_fallback(
                        route, generation=gen):
                    self.get_logger().warn(
                        f'FollowPath far-gap {gap:.1f} m → NavFn GOAL fallback '
                        f'(route_uid={route.route_uid}; yaw-gate after bridge)'
                    )
                    return
            if gap > 5.0:
                # Long straight densify crosses lethal forest (Entry 036 flake).
                self.get_logger().warn(
                    f'FollowPath refusing densify bridge {gap:.1f} m '
                    f'(route_uid={route.route_uid}); need NavFn'
                )
                return
            if gap > 0.6:
                bridge_pts = densify_polyline(
                    [(rx, ry), (x0, y0)], max_step_m=0.10)
                # Drop duplicate join vertex.
                bridge_pts = bridge_pts[:-1] if len(bridge_pts) > 1 else bridge_pts
                bridge_yaws = along_track_yaw(bridge_pts + [pts[0]])
                bridge_yaws = bridge_yaws[: len(bridge_pts)]
                pts = tuple(bridge_pts) + tuple(pts)
                yaws = tuple(bridge_yaws) + tuple(yaws)
                self.get_logger().info(
                    f'FollowPath densify bridge {gap:.1f} m → '
                    f'route_uid={route.route_uid}'
                )

        # Build Path and yaw-gate inside _send_follow_path_msg (look-ahead chord).
        # Far-gap already returned into NavFn → same msg gate on bridge result.
        path_msg = Path()
        path_msg.header.frame_id = self._map_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for (x, y), yaw in zip(pts, yaws):
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            half = 0.5 * float(yaw)
            ps.pose.orientation.z = math.sin(half)
            ps.pose.orientation.w = math.cos(half)
            path_msg.poses.append(ps)

        self._send_follow_path_msg(
            path_msg, route_uid=route.route_uid, path_id=route.path_id)

    def _fp_goal_response(
        self, future, generation: int, route_uid: str, epoch: int
    ) -> None:
        if generation != self._owner.generation:
            self.get_logger().info(
                f'Ignoring FollowPath accept for stale gen '
                f'(submit={generation}, now={self._owner.generation})'
            )
            return
        if epoch in self._fp_cancelled_epochs:
            self._fp_cancelled_epochs.discard(epoch)
            return
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'FollowPath send failed: {exc}')
            self._handle_follow_path_dispatch_failure(
                generation, epoch, f'send_exception:{type(exc).__name__}')
            return
        if not handle.accepted:
            self.get_logger().warn(
                f'FollowPath goal rejected (route_uid={route_uid})'
            )
            self._handle_follow_path_dispatch_failure(
                generation, epoch, 'goal_rejected')
            return
        self._fp_goal_handle = handle
        # Entry 063 A1: publish exact accepted FollowPath Goal.path (epoch fence).
        if (
            int(epoch) == int(self._fp_goal_epoch)
            and self._last_submitted_path is not None
        ):
            self._publish_follow_path_ref(self._last_submitted_path, epoch=epoch)
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda f, g=generation, e=epoch, uid=route_uid: self._fp_goal_result(
                f, g, e, uid)
        )

    def _handle_follow_path_dispatch_failure(
        self, generation: int, epoch: int, reason: str
    ) -> None:
        """Recover a rejected/failed FollowPath without a silent zero-velocity hang."""
        if int(epoch) != int(self._fp_goal_epoch):
            return
        self._fp_goal_handle = None
        self._clear_active_path()
        if generation != self._owner.generation:
            return
        if self._owner.has_blocking_inhibit() or self._owner.operator_stop_latch:
            return
        active = self._bank.active() if self._bank is not None else None
        live = (
            nav2_costmap_to_grid(self._live_costmap)
            if self._live_costmap is not None else None
        )
        if (
            active is not None
            and live is not None
            and self._state in (
                MissionNavState.FOLLOWING_PATH,
                MissionNavState.SWITCHING_PATH,
            )
        ):
            ok, live_reason = self._active_route_valid(active, live)
            if not ok and self._auto_switch:
                self._mark_path_blocked(
                    active, f'follow_path_dispatch_{reason}:{live_reason}')
                self._set_state(MissionNavState.SWITCHING_PATH)
                self._switch_follow_path(live)
                return
            self._set_state(MissionNavState.FOLLOWING_PATH)
            self._last_progress_t = (
                time.monotonic() - self._stuck_timeout_s - 0.1)
            self._stuck_recover_until = 0.0
            self.get_logger().warn(
                f'FollowPath dispatch failure ({reason}); invoking bounded '
                'owner recovery while live route remains valid'
            )
            self._maybe_owner_stuck_recover()
            return
        self._set_state(MissionNavState.FAILED)
        self._owner.detail = f'FOLLOW_PATH_DISPATCH_FAILED:{reason}'
        self._publish_status()

    def _home_success_fence_ok(
        self, generation: int, epoch: int, route_uid: str
    ) -> bool:
        """D2: all required to call ``_mark_returned_home`` (Wave-1)."""
        if self._nav_mode != 'home':
            return False
        if generation != self._owner.generation:
            return False
        if int(epoch) != int(self._fp_goal_epoch):
            return False
        if self._home_plan_pending:
            return False
        if self._home_route is None:
            return False
        if not route_uid or route_uid != self._home_route.route_uid:
            return False
        if (
            self._home_fp_route_uid is not None
            and route_uid != self._home_fp_route_uid
        ):
            return False
        return True

    def _fp_goal_result(
        self,
        future,
        generation: int,
        epoch: int = 0,
        route_uid: str = '',
    ) -> None:
        try:
            result = future.result()
            status = result.status
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'FollowPath result failed: {exc}')
            if int(epoch) == int(self._fp_goal_epoch):
                self._fp_goal_handle = None
            if epoch in self._fp_cancelled_epochs:
                self._fp_cancelled_epochs.discard(epoch)
                return
            if self._ignore_fp_results > 0:
                self._ignore_fp_results -= 1
                return
            if generation != self._owner.generation:
                return
            self._set_state(MissionNavState.FAILED)
            return
        # Only clear handle when this result belongs to the live epoch.
        # Cancelled epoch N-1 arriving after epoch N accept must not wipe the
        # new handle — that caused Entry 036 "orphaned_handle" stuck recovery
        # while FollowPath was still running, then NavFn thrash.
        if (
            self._fp_goal_handle is not None
            and int(epoch) == int(self._fp_goal_epoch)
        ):
            self._fp_goal_handle = None
        if epoch in self._fp_cancelled_epochs:
            self._fp_cancelled_epochs.discard(epoch)
            if self._ignore_fp_results > 0:
                self._ignore_fp_results -= 1
            self.get_logger().info(
                f'Ignoring cancelled FollowPath result (status={status}, '
                f'epoch={epoch})'
            )
            return
        if self._ignore_fp_results > 0 and status != GoalStatus.STATUS_SUCCEEDED:
            self._ignore_fp_results -= 1
            self.get_logger().info(
                f'Ignoring superseded FollowPath result (status={status})'
            )
            return
        if generation != self._owner.generation:
            self.get_logger().info(
                f'Ignoring FollowPath result for stale gen '
                f'(submit={generation}, now={self._owner.generation})'
            )
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            if self._nav_mode == 'home':
                # D2 fence: never false-RETURNED on mission SUCCEEDED during
                # plan-pending / before home route armed / uid mismatch.
                if self._home_success_fence_ok(generation, epoch, route_uid):
                    self._mark_returned_home('FollowPath home succeeded')
                else:
                    self.get_logger().warn(
                        'Ignoring FollowPath SUCCEEDED — home success fence '
                        f'failed (uid={route_uid!r} pending='
                        f'{self._home_plan_pending} home_route='
                        f'{self._home_route is not None} epoch={epoch}/'
                        f'{self._fp_goal_epoch})'
                    )
                return
            # Corridor splice ends at a checkpoint, not the mission goal —
            # resume frozen RouteInstance if still far from goal.
            if (
                self._splice_used
                and self._route_bank is not None
                and self._route_bank.active() is not None
                and self._goal_xyyaw is not None
                and self._robot_xyyaw is not None
            ):
                gx, gy, _ = self._goal_xyyaw
                rx, ry, _ = self._robot_xyyaw
                if math.hypot(gx - rx, gy - ry) > 1.0:
                    self.get_logger().info(
                        'Corridor splice complete; resuming RouteInstance FollowPath'
                    )
                    self._splice_used = False
                    self._abort_resend_used = False
                    self._set_state(MissionNavState.FOLLOWING_PATH)
                    self._send_follow_path(self._route_bank.active())
                    return
            # Entry 070/071: escape hop is pocket-exit only — never ARRIVED.
            if getattr(self, '_escape_hop_pending_join', False):
                self._escape_hop_pending_join = False
                active = (
                    self._route_bank.active()
                    if self._route_bank is not None else None
                )
                dist_goal = float('nan')
                if (
                    self._goal_xyyaw is not None
                    and self._robot_xyyaw is not None
                ):
                    dist_goal = math.hypot(
                        self._robot_xyyaw[0] - self._goal_xyyaw[0],
                        self._robot_xyyaw[1] - self._goal_xyyaw[1],
                    )
                if active is not None and (
                    math.isnan(dist_goal) or dist_goal > 1.0
                ):
                    # Entry 072: if still far from Path B after hop #1, chain
                    # hop #2 toward the corridor before join NavFn (R10 class
                    # soft-stop when a single south-biased hop left gap≈10 m).
                    nearest_b = float('inf')
                    if self._robot_xyyaw is not None and active.points:
                        rx, ry, _ = self._robot_xyyaw
                        stride = max(1, len(active.points) // 100)
                        nearest_b = min(
                            math.hypot(rx - x, ry - y)
                            for x, y in active.points[::stride]
                        )
                    chained = False
                    if (
                        nearest_b > 5.0
                        and int(getattr(self, '_escape_hop_count', 0))
                        < int(getattr(self, '_escape_hop_budget', 2))
                    ):
                        chained = self._request_switch_escape_hop(
                            active, generation=self._owner.generation)
                    if chained:
                        self.get_logger().info(
                            'Escape hop FollowPath succeeded '
                            f'(dist_goal={dist_goal:.2f}m, path_b_gap='
                            f'{nearest_b:.1f}m) — chaining another escape hop'
                        )
                        self._abort_resend_used = False
                        self._stuck_recoveries = 0
                        self._far_gap_grace_collapsed = False
                        self._set_state(MissionNavState.FOLLOWING_PATH)
                        return
                    self.get_logger().info(
                        'Escape hop FollowPath succeeded '
                        f'(dist_goal={dist_goal:.2f}m) — Path-B join/bridge next'
                    )
                    self._abort_resend_used = False
                    self._stuck_recoveries = 0
                    self._far_gap_grace_collapsed = False
                    self._set_state(MissionNavState.FOLLOWING_PATH)
                    self._send_follow_path(active, allow_navfn_bridge=True)
                    return
                self.get_logger().warn(
                    'Escape hop done but cannot continue join '
                    f'(active={active is not None}, dist_goal={dist_goal})'
                )
            # Geometric fence: never ARRIVED while far from mission goal
            # (DEFAULT_GOAL / StartMission pose). Defends against hop/splice
            # false SUCCEEDED (R9 ARRIVED @ ~10 m class).
            if (
                self._goal_xyyaw is not None
                and self._robot_xyyaw is not None
            ):
                dist_goal = math.hypot(
                    self._robot_xyyaw[0] - self._goal_xyyaw[0],
                    self._robot_xyyaw[1] - self._goal_xyyaw[1],
                )
                if dist_goal > 1.5:
                    self.get_logger().warn(
                        f'Ignoring FollowPath SUCCEEDED — still {dist_goal:.2f}m '
                        'from mission goal (not ARRIVED); continuing FollowPath'
                    )
                    self._set_state(MissionNavState.FOLLOWING_PATH)
                    if (
                        self._route_bank is not None
                        and self._route_bank.active() is not None
                    ):
                        self._abort_resend_used = False
                        self._send_follow_path(
                            self._route_bank.active(), allow_navfn_bridge=True)
                    return
            self._mark_mission_arrived('FollowPath succeeded')
            return
        if self._state in (
            MissionNavState.SWITCHING_PATH,
            MissionNavState.PATH_BLOCKED,
        ):
            return
        if status not in (
            GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_ABORTED,
        ):
            self._set_state(MissionNavState.FAILED)
            return
        # Abort-resend: FollowPath only; hard-gated by abort_resend_forbidden().
        if self._observation_hold:
            self.get_logger().info(
                'Suppressing FollowPath abort-resend (observation_hold)'
            )
            return
        if self._owner.abort_resend_forbidden():
            self.get_logger().warn(
                'Suppressing FollowPath abort-resend (mission-owner gate)'
            )
            return
        if (
            self._state == MissionNavState.RETURNING_TO_START
            and self._home_route is not None
            and not self._abort_resend_used
        ):
            self._abort_resend_used = True
            self.get_logger().warn(
                f'FollowPath ended with status={status} while '
                'RETURNING_TO_START; resending home once'
            )
            self._clear_local_costmap()
            self._send_follow_path(self._home_route)
            return
        if (
            self._state == MissionNavState.RETURNING_TO_START
            and self._abort_resend_used
        ):
            self.get_logger().error(
                f'Return-to-start FollowPath status={status} after one resend; '
                'NO_SAFE_ROUTE'
            )
            self._raise_no_safe_route(f'home_fp_abort_{status}')
            return
        if (
            self._state == MissionNavState.FOLLOWING_PATH
            and self._nav_mode != 'home'
            and self._route_bank is not None
            and self._route_bank.active() is not None
            and not self._abort_resend_used
            and time.monotonic() >= self._switch_grace_until
        ):
            self._abort_resend_used = True
            active_route = self._route_bank.active()
            self.get_logger().warn(
                f'FollowPath ended with status={status} while '
                f'{self._state.value}; resend active route once '
                '(Entry 023 ownership recovery; no local clear)'
            )
            # One NavFn allowed while far from corridor; flag stays True after
            # bridge OK (no re-arm) so a second abort cannot thrash (Entry 036).
            self._send_follow_path(active_route)
            return
        if (
            self._state == MissionNavState.FOLLOWING_PATH
            and time.monotonic() < self._switch_grace_until
        ):
            # Entry 070: far-gap aborts during the long settle mean the active
            # NavFn bridge/GOAL path is unexecutable (MPPI optimizer fail /
            # no-progress). Do not wait out 45 s on a dead path — collapse
            # grace so stuck recovery can rotate joins immediately.
            gap = 0.0
            if (
                self._robot_xyyaw is not None
                and self._route_bank is not None
                and self._route_bank.active() is not None
                and self._route_bank.active().points
            ):
                rx, ry, _ = self._robot_xyyaw
                pts = self._route_bank.active().points
                stride = max(1, len(pts) // 100)
                gap = min(
                    math.hypot(rx - x, ry - y) for x, y in pts[::stride]
                )
            # ABORTED only — intentional cancel is already filtered via
            # _fp_cancelled_epochs. Collapse settle at most once per switch
            # (R7 thrash: collapsing every abort drove 34 m forest loops).
            if (
                gap > 5.0
                and status == GoalStatus.STATUS_ABORTED
                and not getattr(self, '_far_gap_grace_collapsed', False)
            ):
                self._far_gap_grace_collapsed = True
                self.get_logger().warn(
                    f'FollowPath status={status} during far-gap grace '
                    f'(gap={gap:.1f}m) — one-shot settle collapse to rotate join'
                )
                self._switch_grace_until = time.monotonic()
                self._stuck_recover_until = 0.0
                self._abort_resend_used = False
                if int(epoch) == int(self._fp_goal_epoch):
                    self._fp_goal_handle = None
                return
            self.get_logger().info(
                f'FollowPath status={status} during post-switch grace; '
                'owner stuck recovery will re-arm after grace'
            )
            return
        # §H: after one ownership resend, only mark-blocked + switch when the
        # live centreline gate agrees the route is invalid. Transient RPP
        # collision (temp barrier / local noise) must not burn the bank —
        # owner stuck recovery re-arms the same RouteInstance.
        if (
            self._state == MissionNavState.FOLLOWING_PATH
            and self._nav_mode != 'home'
            and self._abort_resend_used
            and self._auto_switch
            and self._bank is not None
            and self._bank.active() is not None
            and self._live_costmap is not None
        ):
            active = self._bank.active()
            live = nav2_costmap_to_grid(self._live_costmap)
            ok, reason = self._active_route_valid(active, live)
            if not ok:
                self.get_logger().warn(
                    f'FollowPath status={status} after ownership recovery; '
                    f'live invalid ({reason}) → mark Path {active.path_id} '
                    'blocked + switch (§H)'
                )
                self._mark_path_blocked(
                    active, f'follow_path_abort_{status}:{reason}')
                self._publish_bank_paths()
                self._set_state(MissionNavState.SWITCHING_PATH)
                self._switch_follow_path(live)
                return
            # Live centreline OK but RPP local collision — NavFn corridor splice
            # (Humble alternative after nudge + failure_tolerance still looped).
            if not self._splice_used and not self._splice_pending:
                if self._request_corridor_splice(self._route_bank.active()):
                    return
            self.get_logger().warn(
                f'FollowPath status={status} after ownership recovery; '
                'live still valid — defer to owner stuck recovery'
            )
            return
        self.get_logger().warn(
            f'FollowPath ended with status={status}; '
            f'leaving state={self._state.value}'
        )

    def _cancel_follow_path(self) -> None:
        """Cancel active FollowPath; ignore that submit's terminal result only."""
        self._fp_cancel_context = None
        self._fp_cancel_deadline = 0.0
        self._switch_action_token += 1
        if self._fp_goal_handle is None:
            return
        try:
            self._fp_cancelled_epochs.add(int(self._fp_goal_epoch))
            self._ignore_fp_results += 1  # legacy counter kept for NTP path
            self._fp_goal_handle.cancel_goal_async()
        except Exception:  # noqa: BLE001
            pass
        self._fp_goal_handle = None

    def _maybe_owner_stuck_recover(self) -> None:
        """BT-inactive stuck recovery: clear local + resubmit active FollowPath.

        Soft-stop (cmd≈0 while FOLLOWING_PATH): treated as no-progress →
        recoverable stuck ticks, then ladder terminal (must not remain
        FOLLOWING_PATH).

        Hang ladder (explicit):
          Mission stuck N → fresh home once → (plan fail | home N | T₂) → §3.2
          Home stuck uses same N + T₂ wall-clock (enabled for nav_mode=='home').
        """
        if not self._use_follow_path:
            return
        if self._observation_hold:
            return
        if self._home_plan_pending:
            return
        # Issue B: do not resubmit FollowPath while Spin/fallback reorient runs.
        if self._reorient_active:
            return
        in_home = (
            self._nav_mode == 'home'
            or self._state == MissionNavState.RETURNING_TO_START
        )
        in_mission_follow = (
            self._state == MissionNavState.FOLLOWING_PATH
            and self._nav_mode != 'home'
        )
        if not in_home and not in_mission_follow:
            return
        if self._owner.abort_resend_forbidden():
            return
        if self._owner.command_authority != CommandAuthority.NAV:
            return
        now = time.monotonic()
        if now < self._switch_grace_until or now < self._stuck_recover_until:
            return
        orphaned = self._fp_goal_handle is None
        # Accept callback can lag; do not thrash-resubmit during that window.
        if orphaned and self._fp_submit_t > 0.0 and (now - self._fp_submit_t) < 2.5:
            return
        # Wave B0 (Entry 047): any in-flight ComputePathToPose bridge/splice
        # must finish before another stuck tick. Previously only orphaned+pending
        # was gated — stalled FollowPath + pending bridge still burned N and
        # reset the join queue, causing planner abort storms → NO_SAFE_ROUTE.
        if self._splice_pending:
            return
        stalled = (
            self._last_progress_t > 0.0
            and (now - self._last_progress_t) >= self._stuck_timeout_s
        )
        if not orphaned and not stalled:
            return

        reason = 'orphaned_handle' if orphaned else 'no_progress'

        # ---- Home mode stuck watchdog (Wave 0) ----
        if in_home:
            elapsed_home = (
                (now - self._home_entered_at) if self._home_entered_at > 0.0
                else 0.0
            )
            t2_eff = self._effective_home_t2_s()
            action = home_stuck_action(
                recoveries=self._stuck_recoveries,
                budget_n=self._stuck_budget_n,
                elapsed_home_s=elapsed_home,
                t2_s=t2_eff,
            )
            if action == 'terminal':
                self.get_logger().error(
                    f'Home stuck bound ({reason}, recoveries='
                    f'{self._stuck_recoveries}, elapsed={elapsed_home:.1f}s) '
                    '→ NO_SAFE_ROUTE'
                )
                self._raise_no_safe_route(f'home_stuck_{reason}')
                return
            route = self._home_route
            if route is None:
                self._raise_no_safe_route('home_stuck_no_route')
                return
            self.get_logger().warn(
                f'Home stuck recovery ({reason}): FollowPath resubmit '
                f'route_uid={route.route_uid}'
            )
            self._stuck_recoveries += 1
            self._stuck_recover_until = now + 4.0
            self._abort_resend_used = False
            self._last_progress_t = now
            if self._robot_xyyaw is not None:
                self._last_progress_xy = (
                    self._robot_xyyaw[0], self._robot_xyyaw[1])
            self._clear_local_costmap()
            self._send_follow_path(route, allow_navfn_bridge=False)
            return

        # ---- Mission FOLLOWING_PATH ----
        if self._route_bank is None or self._route_bank.active() is None:
            return
        route = self._route_bank.active()

        # Far-gap detection (used to avoid burning N / premature home).
        gap = 0.0
        if self._robot_xyyaw is not None and route.points:
            rx, ry, _ = self._robot_xyyaw
            stride = max(1, len(route.points) // 100)
            gap = min(
                math.hypot(rx - x, ry - y)
                for x, y in route.points[::stride]
            )
        far_gap = gap > 5.0
        join_q = getattr(self, '_switch_join_queue', None) or []
        join_attempt = int(getattr(self, '_switch_join_attempt', 0))
        rotating_joins = far_gap and bool(join_q) and join_attempt < len(join_q)

        action = mission_stuck_action(
            recoveries=self._stuck_recoveries,
            budget_n=self._stuck_budget_n,
            home_already_attempted=self._home_attempted_from_stuck,
            home_available=(
                self._return_home_on_all_blocked
                and self._home_xyyaw is not None
            ),
        )
        if action == 'attempt_home':
            # Entry 049: if Path B (active) is still live-valid, home through
            # Path-A keepout often fails → false NO_SAFE_ROUTE. Rebuild join
            # queue / bridge a bounded number of times first.
            live_ok = False
            if (
                self._bank is not None
                and self._bank.active() is not None
                and self._live_costmap is not None
            ):
                live = nav2_costmap_to_grid(self._live_costmap)
                live_ok, _ = self._active_route_valid(self._bank.active(), live)
            # Entry 082: if NavFn join+GOAL already exhausted, do not rebuild
            # the same failing queue while claiming live-valid — go home.
            bridge_exhausted = (
                int(getattr(self, '_far_gap_rebuilds', 0))
                >= max(1, int(getattr(self, '_far_gap_rebuild_budget', 3)))
            )
            if (
                live_ok
                and not bridge_exhausted
                and self._far_gap_rebuilds < self._far_gap_rebuild_budget
            ):
                self._far_gap_rebuilds += 1
                self.get_logger().warn(
                    f'Mission stuck N={self._stuck_budget_n} exhausted but '
                    f'active route still live-valid (gap={gap:.1f}m); '
                    f'rebuild join/bridge '
                    f'({self._far_gap_rebuilds}/{self._far_gap_rebuild_budget}) '
                    'instead of home'
                )
                self._stuck_recoveries = 0
                self._stuck_recover_until = now + 4.0
                self._switch_join_queue = None
                self._switch_join_attempt = 0
                self._switch_goal_fallback_used = False
                self._abort_resend_used = False
                self._last_progress_t = now
                if self._robot_xyyaw is not None:
                    self._last_progress_xy = (
                        self._robot_xyyaw[0], self._robot_xyyaw[1])
                self._clear_local_costmap()
                self._send_follow_path(
                    route,
                    allow_navfn_bridge=True,
                    refuse_context=RefuseContext.CENTRAL_GATE_STUCK,
                )
                return
            self.get_logger().error(
                f'Mission stuck budget N={self._stuck_budget_n} exhausted '
                f'({reason}'
                + ('; bridge_exhausted' if bridge_exhausted else '')
                + '); attempting fresh home once'
            )
            self._home_attempted_from_stuck = True
            self._begin_return_to_start()
            return
        if action == 'terminal':
            self.get_logger().error(
                f'Mission stuck budget exhausted ({reason}); NO_SAFE_ROUTE'
            )
            self._raise_no_safe_route(f'mission_stuck_{reason}')
            return

        # After two same-route stuck clears, escalate only if live gate fails.
        if (
            self._stuck_recoveries >= 2
            and self._auto_switch
            and self._bank is not None
            and self._bank.active() is not None
            and self._live_costmap is not None
        ):
            active = self._bank.active()
            live = nav2_costmap_to_grid(self._live_costmap)
            ok, vreason = self._active_route_valid(active, live)
            if not ok:
                self.get_logger().warn(
                    f'Owner stuck recovery escalate ({reason}): live invalid '
                    f'({vreason}) → mark Path {active.path_id} blocked + switch'
                )
                self._mark_path_blocked(active, f'stuck_{reason}:{vreason}')
                self._publish_bank_paths()
                self._stuck_recoveries += 1
                self._stuck_recover_until = now + 4.0
                self._set_state(MissionNavState.SWITCHING_PATH)
                self._switch_follow_path(live)
                return
        self.get_logger().warn(
            f'Owner stuck recovery ({reason}): FollowPath resubmit '
            f'route_uid={route.route_uid} path_id={route.path_id}'
            + (f' far_gap={gap:.1f}m rotate_joins' if rotating_joins else '')
        )
        # Entry 049: mid-queue join rotation does not burn stuck N — otherwise
        # N=8 expires before all join candidates + goal fallback are tried.
        if not rotating_joins:
            self._stuck_recoveries += 1
        self._stuck_recover_until = now + 4.0
        self._abort_resend_used = False
        self._last_progress_t = now
        if self._robot_xyyaw is not None:
            self._last_progress_xy = (
                self._robot_xyyaw[0], self._robot_xyyaw[1])
        # Far from corridor: NavFn bridge with persistent join queue; else densify.
        # Entry 047: do NOT wipe `_switch_join_queue` on every stuck tick.
        self._clear_local_costmap()
        # Entry 082: FOOTPRINT_ESCAPE only for far-gap / sealed-start escape.
        # Same-path densify stuck (T2 barrier under nose) must NOT clear the
        # LIDAR keepout that should trigger ROUTE_MARKED_BLOCKED + switch.
        # FIRE retained always (obstacle_memory clear_lidar_disk).
        if far_gap or getattr(self, '_sealed_unseal_active', False):
            self._request_footprint_escape_clear()
        allow_navfn = far_gap
        if allow_navfn and not getattr(self, '_switch_join_queue', None):
            self._switch_join_queue = None  # force rebuild in bridge helper
            self._switch_join_attempt = 0
        # Entry 073: re-arm local unseal on sealed far-gap stuck (no hop).
        if far_gap and not self._splice_pending:
            repel = self._path_a_repel_xy()
            locus_d = None
            if repel is not None and self._robot_xyyaw is not None:
                locus_d = math.hypot(
                    self._robot_xyyaw[0] - repel[0],
                    self._robot_xyyaw[1] - repel[1],
                )
            if should_arm_sealed_start_unseal(gap, locus_d):
                self._arm_sealed_start_local_unseal('stuck_far_gap')
                if (
                    not getattr(self, '_drive_escape_used', False)
                    and self._request_drive_on_heading_escape(
                        route, generation=self._owner.generation)
                ):
                    self.get_logger().warn(
                        f'Stuck far_gap={gap:.1f}m → Path-B-tangent '
                        'DriveOnHeading (Entry 073)'
                    )
                    return
        # Entry 049b: after one far-gap stuck tick, prefer mission-goal NavFn
        # fallback before burning more join rotations (soft-stop near Path-A
        # keepout often never reaches distant Path-B joins).
        # Entry 049c: also trigger when within 6 m of goal but soft-stopped
        # (R1v3 fails #6/#8 ended FOLLOWING_PATH at ~3.5 m from goal).
        goal_dist = float('nan')
        if self._robot_xyyaw is not None and self._goal_xyyaw is not None:
            goal_dist = math.hypot(
                self._robot_xyyaw[0] - self._goal_xyyaw[0],
                self._robot_xyyaw[1] - self._goal_xyyaw[1],
            )
        near_goal_soft = (
            not math.isnan(goal_dist) and goal_dist < 6.0 and goal_dist > 0.6
        )
        if (
            (far_gap or near_goal_soft)
            and self._stuck_recoveries >= 1
            and not getattr(self, '_switch_goal_fallback_used', False)
            and not self._splice_pending
        ):
            if self._request_switch_navfn_goal_fallback(
                    route, generation=self._owner.generation):
                why = (
                    f'near_goal={goal_dist:.1f}m' if near_goal_soft
                    else f'far_gap={gap:.1f}m'
                )
                self.get_logger().warn(
                    f'Stuck {why} → early NavFn GOAL fallback'
                )
                return
        self._send_follow_path(
            route,
            allow_navfn_bridge=allow_navfn,
            refuse_context=RefuseContext.CENTRAL_GATE_STUCK,
        )

    def _send_goal(self, x: float, y: float, yaw: float) -> None:
        if self._use_follow_path:
            # Bank FollowPath mode must not submit NTP for mission follow.
            self.get_logger().debug(
                'Ignoring NavigateToPose submit in bank_follow_path mode'
            )
            return
        if not self._arm_nav_authority():
            self.get_logger().warn(
                'Refusing NavigateToPose submit — abort-resend hard gate '
                f'(phase={self._owner.phase.value}, '
                f'latch={self._owner.operator_stop_latch}, '
                f'auth={self._owner.command_authority.value})'
            )
            return
        if not self._nav_client.server_is_ready():
            self.get_logger().error('navigate_to_pose server not available')
            return
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self._map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        half = 0.5 * yaw
        pose.pose.orientation.z = math.sin(half)
        pose.pose.orientation.w = math.cos(half)
        goal.pose = pose
        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response)

    def _goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'goal send failed: {exc}')
            return
        if not handle.accepted:
            self.get_logger().warn('NavigateToPose goal rejected')
            return
        self._goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._goal_result)

    def _goal_result(self, future) -> None:
        try:
            result = future.result()
            status = result.status
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'goal result failed: {exc}')
            if self._ignore_goal_results > 0:
                self._ignore_goal_results -= 1
                return
            if self._use_follow_path:
                return
            self._set_state(MissionNavState.FAILED)
            return
        # FollowPath mode ignores NTP results for mission ownership.
        if self._use_follow_path:
            if self._ignore_goal_results > 0:
                self._ignore_goal_results -= 1
            return
        if self._ignore_goal_results > 0 and status != GoalStatus.STATUS_SUCCEEDED:
            self._ignore_goal_results -= 1
            self.get_logger().info(
                f'Ignoring superseded NavigateToPose result (status={status})'
            )
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            if self._nav_mode == 'home' or self._state == MissionNavState.RETURNING_TO_START:
                self._goal_handle = None
                self._mark_returned_home('NavigateToPose home succeeded')
            else:
                self._mark_mission_arrived('NavigateToPose succeeded')
        elif self._state in (
            MissionNavState.SWITCHING_PATH,
            MissionNavState.PATH_BLOCKED,
        ):
            return
        elif status in (
            GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_ABORTED,
        ):
            # Entry 024: abort while returning home → resend home once.
            if (
                self._state == MissionNavState.RETURNING_TO_START
                and self._home_xyyaw is not None
                and self._ignore_goal_results == 0
                and not self._abort_resend_used
            ):
                if self._owner.abort_resend_forbidden():
                    self.get_logger().warn(
                        'Suppressing home abort-resend (mission-owner gate)'
                    )
                    return
                self._abort_resend_used = True
                self.get_logger().warn(
                    f'NavigateToPose ended with status={status} while '
                    f'RETURNING_TO_START; resending home once '
                    '(Entry 024 ownership recovery)'
                )
                self._send_goal(*self._home_xyyaw)
                return
            if (
                self._state == MissionNavState.RETURNING_TO_START
                and self._abort_resend_used
            ):
                self.get_logger().error(
                    f'Return-to-start NavigateToPose status={status} after '
                    'one resend; NO_SAFE_ROUTE'
                )
                self._raise_no_safe_route(f'home_ntp_abort_{status}')
                return
            # Entry 023: if we own the mission goal and are FOLLOWING_PATH,
            # resend once so an unexpected abort does not leave the robot
            # frozen with no NavigateToPose client.
            # Entry 027: suppress during post_switch_grace so a switch preempt
            # cannot stack a second NavigateToPose restart (stall amplifier).
            if (
                self._state == MissionNavState.FOLLOWING_PATH
                and self._nav_mode != 'home'
                and self._goal_xyyaw is not None
                and self._ignore_goal_results == 0
                and not self._abort_resend_used
                and time.monotonic() >= self._switch_grace_until
            ):
                if self._owner.abort_resend_forbidden():
                    self.get_logger().warn(
                        'Suppressing Entry 023 abort-resend (mission-owner gate)'
                    )
                    return
                self._abort_resend_used = True
                self.get_logger().warn(
                    f'NavigateToPose ended with status={status} while '
                    f'{self._state.value}; resending goal once '
                    '(Entry 023 ownership recovery)'
                )
                self._send_goal(*self._goal_xyyaw)
                return
            if (
                self._state == MissionNavState.FOLLOWING_PATH
                and time.monotonic() < self._switch_grace_until
            ):
                self.get_logger().info(
                    f'NavigateToPose status={status} during post-switch grace; '
                    'suppressing abort-resend (Entry 027)'
                )
                return
            # External client (test/UI) may own the original goal; bank
            # switch preempt is not a mission failure by itself.
            self.get_logger().warn(
                f'NavigateToPose ended with status={status}; '
                f'leaving state={self._state.value}'
            )
        else:
            self._set_state(MissionNavState.FAILED)

    def _clear_local_costmap(self) -> None:
        """Entry 027 post-switch clear (does not mutate command_authority).

        Full §G revision-gated rebuild is ``_revision_gated_local_rebuild`` and
        must not run on every switch (would force gate NONE).
        """
        if not self._clear_local_cli.service_is_ready():
            self.get_logger().warn(
                'local_costmap/clear_entirely_local_costmap not ready; '
                'skipping post-switch clear'
            )
            return
        try:
            self._clear_local_cli.call_async(ClearEntireCostmap.Request())
            self.get_logger().info(
                'Cleared local costmap after path-bank switch (Entry 027)')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Post-switch local clear failed: {exc}')

    def _arm_sealed_start_local_unseal(self, reason: str) -> bool:
        """Disable local LIDAR ObstacleLayer + ClearEntirely (Entry 073).

        FIRE ``fire_obstacle_layer`` stays enabled. Global KeepoutFilter /
        barrier retention untouched. Restore via
        ``_restore_sealed_start_local_unseal`` within
        ``SEALED_UNSEAL_RESTORE_DEADLINE_S`` once clear of the Path-A locus
        or on max-hold / stop.
        """
        if getattr(self, '_sealed_unseal_active', False):
            self._clear_local_costmap()
            self._request_footprint_escape_clear()
            return True
        cli = self._pick_local_costmap_set_cli()
        if cli is None:
            self.get_logger().warn(
                f'Sealed-start unseal ({reason}): local_costmap set_parameters '
                'not ready — clear-only fallback'
            )
            self._clear_local_costmap()
            self._request_footprint_escape_clear()
            # Still mark active so timer tries restore path / clear again.
            self._sealed_unseal_active = True
            self._sealed_unseal_armed_at = time.monotonic()
            self._sealed_unseal_layer_was_enabled = True
            self._sealed_unseal_layer_disabled = False
            return False
        req = SetParameters.Request()
        req.parameters = [
            Parameter(
                LOCAL_OBSTACLE_LAYER_ENABLED_PARAM,
                Parameter.Type.BOOL,
                False,
            ).to_parameter_msg()
        ]
        try:
            cli.call_async(req)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f'Sealed-start unseal set_parameters failed ({reason}): {exc}'
            )
            self._clear_local_costmap()
            return False
        self._sealed_unseal_active = True
        self._sealed_unseal_armed_at = time.monotonic()
        self._sealed_unseal_layer_was_enabled = True
        self._sealed_unseal_layer_disabled = True
        # R12: after unseal the robot can reach GOAL; a single near-goal MPPI
        # reverse sample fails Issue B reverse-then-recover. Hold forward_only
        # through arrival (not a permanent vx_min ban).
        self._sealed_unseal_forward_latch = True
        self._forward_only_until = max(
            self._forward_only_until,
            time.monotonic() + float(SEALED_UNSEAL_MAX_HOLD_S) + 60.0,
        )
        self._publish_forward_only()
        self._clear_local_costmap()
        self._request_footprint_escape_clear()
        self.get_logger().warn(
            f'Sealed-start LOCAL unseal ARMED ({reason}): '
            f'{LOCAL_OBSTACLE_LAYER_ENABLED_PARAM}=false + ClearEntirely '
            f'(FIRE layer retained; restore≤{SEALED_UNSEAL_RESTORE_DEADLINE_S}s '
            f'after min_hold={SEALED_UNSEAL_MIN_HOLD_S}s / '
            f'max_hold={SEALED_UNSEAL_MAX_HOLD_S}s; forward_only latched)'
        )
        return True

    def _pick_local_costmap_set_cli(self):
        """Prefer inner Costmap2DROS set_parameters; fall back to outer name."""
        for attr in ('_local_costmap_set_cli', '_local_costmap_set_cli_alt'):
            cli = getattr(self, attr, None)
            if cli is not None and cli.service_is_ready():
                return cli
        return None

    def _restore_sealed_start_local_unseal(self, *, reason: str) -> None:
        """Re-enable local LIDAR ObstacleLayer within restore deadline."""
        if not getattr(self, '_sealed_unseal_active', False):
            return
        self._sealed_unseal_active = False
        t0 = time.monotonic()
        cli = self._pick_local_costmap_set_cli()
        if (
            cli is not None
            and getattr(self, '_sealed_unseal_layer_disabled', False)
            and getattr(self, '_sealed_unseal_layer_was_enabled', True)
        ):
            req = SetParameters.Request()
            req.parameters = [
                Parameter(
                    LOCAL_OBSTACLE_LAYER_ENABLED_PARAM,
                    Parameter.Type.BOOL,
                    True,
                ).to_parameter_msg()
            ]
            try:
                cli.call_async(req)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(
                    f'Sealed-start unseal restore failed ({reason}): {exc}'
                )
        self._sealed_unseal_layer_disabled = False
        elapsed = time.monotonic() - t0
        self.get_logger().info(
            f'Sealed-start LOCAL unseal RESTORED ({reason}) '
            f'{LOCAL_OBSTACLE_LAYER_ENABLED_PARAM}=true in {elapsed:.3f}s '
            f'(deadline {SEALED_UNSEAL_RESTORE_DEADLINE_S}s)'
        )

    def _maybe_restore_sealed_start_unseal(self) -> None:
        """Timer exit for Entry 073 unseal window (locus clear / max hold)."""
        if not getattr(self, '_sealed_unseal_active', False):
            return
        # Always restore on mission stop / no-safe / arrived.
        if self._owner.phase in (
            MissionPhase.STOPPING,
            MissionPhase.STOPPED,
        ) or self._state in (
            MissionNavState.NO_SAFE_ROUTE,
            MissionNavState.ARRIVED,
            MissionNavState.RETURNED_TO_START,
            MissionNavState.FAILED,
        ):
            self._restore_sealed_start_local_unseal(reason=f'state_{self._state.value}')
            return
        locus_d = None
        repel = self._path_a_repel_xy()
        if repel is not None and self._robot_xyyaw is not None:
            locus_d = math.hypot(
                self._robot_xyyaw[0] - repel[0],
                self._robot_xyyaw[1] - repel[1],
            )
        defer = bool(
            getattr(self, '_reorient_active', False)
            or getattr(self, '_splice_pending', False)
            or getattr(self, '_drive_escape_pending_join', False)
        )
        do_restore, why = should_restore_sealed_start_unseal(
            locus_d,
            float(getattr(self, '_sealed_unseal_armed_at', 0.0)),
            time.monotonic(),
            defer=defer,
        )
        if do_restore:
            self._restore_sealed_start_local_unseal(reason=why or 'timer')

    def _on_memory_revision(self, msg) -> None:
        """Track revision for §G verify only — do not start rebuild here."""
        self._memory_revision = int(msg.data)

    def _on_memory_event(self, msg: String) -> None:
        """§G trigger: typed FIRE create/clear only (not LIDAR_CLEAR churn)."""
        text = (msg.data or '').strip()
        if not text.startswith(('FIRE_MARK', 'FIRE_CLEAR')):
            return
        if not self._use_follow_path:
            return
        if self._rebuild_pending:
            return
        # Auto §G only while mission owns FollowPath motion. Manual/teleop /
        # idle: revision is tracked; StartMission rebuild-recovery is explicit.
        if self._owner.phase not in (
            MissionPhase.NAVIGATING,
            MissionPhase.SWITCHING,
            MissionPhase.RETURNING_TO_START,
            MissionPhase.HOLDING,
        ):
            return
        self.get_logger().info(
            f'§G: FIRE memory event ({text.split()[0]}) rev={self._memory_revision}; '
            'starting revision-gated local rebuild'
        )
        self._revision_gated_local_rebuild()

    def _revision_gated_local_rebuild(self) -> bool:
        """§G: cancel motion → NONE → clear local → verify revision → re-arm.

        Stop during rebuild wins (phase STOPPING/STOPPED): no obsolete resume.
        After clear, defer verification on the next timer ticks via
        ``_rebuild_pending`` so revision can settle.

        D3: snapshot ``_rebuild_resume_mode`` HOME|MISSION; clear home plan
        pending + bump generation so in-flight callbacks cannot arm motion.
        """
        if self._owner.phase in (MissionPhase.STOPPING, MissionPhase.STOPPED):
            self.get_logger().info('§G aborted — Stop during rebuild wins')
            self._rebuild_pending = False
            self._rebuild_resume_mode = None
            return False
        # D3 resume token before mutating phase.
        home_in_progress = (
            self._nav_mode == 'home'
            or self._state == MissionNavState.RETURNING_TO_START
            or self._owner.phase == MissionPhase.RETURNING_TO_START
            or (
                self._home_attempted
                and self._nav_mode == 'home'
                and self._state not in (
                    MissionNavState.NO_SAFE_ROUTE,
                    MissionNavState.RETURNED_TO_START,
                )
            )
        )
        if home_in_progress:
            self._rebuild_resume_mode = 'HOME'
        elif self._owner.phase in (
            MissionPhase.NAVIGATING,
            MissionPhase.SWITCHING,
        ) or self._state in (
            MissionNavState.FOLLOWING_PATH,
            MissionNavState.PATH_BLOCKED,
            MissionNavState.SWITCHING_PATH,
        ):
            self._rebuild_resume_mode = 'MISSION'
        else:
            self._rebuild_resume_mode = None
        self._home_plan_pending = False
        self._owner.bump_generation()
        self._rebuild_rev_snapshot = int(self._memory_revision)
        self._rebuild_retries = getattr(self, '_rebuild_retries', 0)
        self._owner.phase = MissionPhase.COSTMAP_REBUILD
        self._owner.command_authority = CommandAuthority.NONE
        self._publish_authority_now()
        # Cancel FollowPath/NTP so obsolete geometry cannot continue.
        self._cancel_nav()
        self._pending_follow_seed = False
        if not self._clear_local_cli.service_is_ready():
            self._rebuild_retries += 1
            self.get_logger().warn('§G: clear service not ready')
        else:
            try:
                self._clear_local_cli.call_async(ClearEntireCostmap.Request())
                self.get_logger().info(
                    f'§G: cleared local costmap '
                    f'(rev_snapshot={self._rebuild_rev_snapshot} '
                    f'resume={self._rebuild_resume_mode})'
                )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'§G clear failed: {exc}')
                self._rebuild_retries += 1
        self._rebuild_pending = True
        self._rebuild_verify_after = time.monotonic() + 1.0
        return True

    def _maybe_finish_rebuild(self) -> None:
        """Complete §G after settle: revision stable → re-arm per resume token."""
        if not getattr(self, '_rebuild_pending', False):
            return
        if self._owner.phase in (MissionPhase.STOPPING, MissionPhase.STOPPED):
            self.get_logger().info(
                '§G: Stop during rebuild — discard pending resume'
            )
            self._rebuild_pending = False
            self._rebuild_resume_mode = None
            return
        if self._owner.phase != MissionPhase.COSTMAP_REBUILD:
            self._rebuild_pending = False
            return
        if time.monotonic() < getattr(self, '_rebuild_verify_after', 0.0):
            return
        snap = int(getattr(self, '_rebuild_rev_snapshot', self._memory_revision))
        if int(self._memory_revision) != snap:
            self._rebuild_retries = getattr(self, '_rebuild_retries', 0) + 1
            self.get_logger().warn(
                f'§G: memory_revision moved {snap}→{self._memory_revision} '
                f'(retry {self._rebuild_retries}/3)'
            )
            if self._rebuild_retries >= 3:
                saved = self._rebuild_resume_mode
                self._rebuild_pending = False
                self._rebuild_retries = 0
                self._rebuild_resume_mode = None
                # GUI-DB: continuous FIRE_MARK near gap_b thrashes revision so
                # rebuild never settles; HOLD_REBUILD_TIMEOUT must not kill an
                # in-progress home return (Entry 061 D3 HOME resume intent).
                if saved == 'HOME' or (
                    self._home_attempted and self._nav_mode == 'home'
                ):
                    self.get_logger().warn(
                        '§G: rebuild unsettled under HOME — skip '
                        'HOLD_REBUILD_TIMEOUT; re-arm return'
                    )
                    self._resume_home_after_rebuild('unsettled')
                    return
                self.raise_blocking_inhibit(SafetyInhibit.HOLD_REBUILD_TIMEOUT)
                return
            # Reclear under new snapshot (preserve resume mode).
            saved = self._rebuild_resume_mode
            self._revision_gated_local_rebuild()
            if saved is not None:
                self._rebuild_resume_mode = saved
            return
        # Stable revision — may resume only if no other inhibit and route still valid.
        self._rebuild_pending = False
        self._rebuild_retries = 0
        resume = self._rebuild_resume_mode
        self._rebuild_resume_mode = None
        if self._owner.has_blocking_inhibit():
            self.get_logger().warn('§G: rebuild OK but inhibit still active')
            return
        if self._owner.operator_stop_latch:
            self.get_logger().info('§G: rebuild OK but operator latch — no resume')
            self._owner.phase = MissionPhase.STOPPED
            self._publish_authority_now()
            return
        # D3 HOME resume: never NAVIGATING + mission corridor.
        if (
            resume == 'HOME'
            or (self._home_attempted and self._nav_mode == 'home')
        ):
            self._resume_home_after_rebuild('verified')
            return
        if resume == 'MISSION':
            self._owner.command_authority = CommandAuthority.NAV
            self._owner.phase = MissionPhase.NAVIGATING
            self._publish_authority_now()
            resume_states = (
                MissionNavState.FOLLOWING_PATH,
                MissionNavState.PATH_BLOCKED,
                MissionNavState.SWITCHING_PATH,
            )
            if (
                self._use_follow_path
                and self._route_bank is not None
                and self._route_bank.active() is not None
                and self._state in resume_states
                and not self._owner.operator_stop_latch
            ):
                self.get_logger().info(
                    '§G: verified — resuming FollowPath on active RouteInstance'
                )
                self._abort_resend_used = False
                self._set_state(MissionNavState.FOLLOWING_PATH)
                self._send_follow_path(self._route_bank.active())
            else:
                self.get_logger().info(
                    '§G: verified — gate NAV; no FollowPath resume '
                    f'(state={self._state.value})'
                )
            return
        # None: gate NAV only; no FP resume.
        self._owner.command_authority = CommandAuthority.NAV
        self._publish_authority_now()
        self.get_logger().info(
            '§G: verified — gate NAV; no FollowPath resume '
            f'(state={self._state.value} resume=None)'
        )

    def _cancel_nav(self, *, ntp_only: bool = False) -> None:
        # Issue B: restore any temp FollowPath.vx_min on every cancel/exit path.
        self._abort_reorient('cancel_nav')
        # Entry 073: never leave local LIDAR ObstacleLayer disabled after cancel.
        if getattr(self, '_sealed_unseal_active', False):
            self._restore_sealed_start_local_unseal(reason='cancel_nav')
        if self._goal_handle is not None:
            try:
                self._ignore_goal_results += 1
                self._goal_handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
            self._goal_handle = None
        if not ntp_only:
            self._cancel_follow_path()

    # -- publishing --------------------------------------------------------

    def _mark_mission_arrived(self, detail: str) -> None:
        """Goal reached — bank ARRIVED + owner IDLE so StartMission can re-arm.

        Leaving owner NAVIGATING after ARRIVED rejected the next StartMission
        (path_bank_nav block after clean; Entry 035).
        """
        if getattr(self, '_sealed_unseal_active', False):
            self._restore_sealed_start_local_unseal(reason='arrived')
        self._sealed_unseal_forward_latch = False
        self._forward_only_until = 0.0
        self._publish_forward_only()
        self._set_state(MissionNavState.ARRIVED)
        self._observation_hold = False
        self._nav_mode = None
        self._owner.phase = MissionPhase.IDLE
        self._owner.command_authority = CommandAuthority.NONE
        self._owner.detail = detail
        # Entry 063 A3: clear latched FollowPath ref on ARRIVED.
        self._clear_active_path()
        self._publish_authority_now()
        self.get_logger().info(f'Goal reached ({detail}); owner → IDLE')

    def _mark_returned_home(self, detail: str) -> None:
        """Home FollowPath/NTP success — bank RETURNED + owner IDLE + NONE.

        APPROVE watch-out: pin IDLE+NONE (match ARRIVED), not IDLE|STOPPED.
        """
        self._set_state(MissionNavState.RETURNED_TO_START)
        self._observation_hold = False
        self._nav_mode = None
        self._home_plan_pending = False
        self._home_fp_route_uid = None
        self._rebuild_resume_mode = None
        self._last_no_safe_detail = ''
        self._last_home_navfn_pts = []
        self._home_via_phase = None
        self._home_via_generation = -1
        self._home_via_v_star = None
        self._home_via_leg1_pts = []
        self._owner.phase = MissionPhase.IDLE
        self._owner.command_authority = CommandAuthority.NONE
        self._owner.stop_reason = ''
        self._owner.detail = detail
        self._clear_active_path()
        self._publish_authority_now()
        self._publish_owner_status()  # immediate IDLE+NONE (not 0.5s timer)
        home = self._home_xyyaw
        self.get_logger().info(
            'RETURNED_TO_START'
            + (
                f' home=({home[0]:.2f}, {home[1]:.2f})'
                if home is not None else ''
            )
            + f'; switches={self._switch_count}; owner → IDLE+NONE ({detail})'
        )

    def _maybe_terminal_hang_bounds(self) -> None:
        """Wave 0/1 wall-clock bounds T₁ / T₂ / T_seed.

        Ladder (explicit — do not collapse):
          Mission stuck N → fresh home once → (plan fail | home N | T₂) → §3.2
          T₁: return-first once via ``_home_attempted`` latch (A.3), then §3.2
          T_seed: direct terminal only if ``_home_xyyaw is None`` (A.4)
        Soft-stop: no-progress is handled in stuck watchdog; these are hard
        wall clocks so the robot must not remain FOLLOWING_PATH / pending.
        """
        now = time.monotonic()

        # T_seed — deferred StartMission bank seed.
        # A.4 invariant: seed_timeout terminal only if home never recorded.
        # If somehow home is set while seed pending + return_home enabled → home.
        if self._pending_follow_seed and self._use_follow_path:
            if self._pending_follow_seed_since <= 0.0:
                self._pending_follow_seed_since = now
            elif seed_timed_out(
                elapsed_s=now - self._pending_follow_seed_since,
                t_seed_s=self._t_seed_s,
            ):
                if (
                    self._home_xyyaw is not None
                    and self._return_home_on_all_blocked
                    and not self._home_attempted
                    and self._nav_mode != 'home'
                    and not self._owner.has_blocking_inhibit()
                    and not self._owner.operator_stop_latch
                ):
                    self.get_logger().error(
                        f'Deferred seed exceeded T_seed={self._t_seed_s:.0f}s '
                        'but home pose recorded — defensive return-home (A.4)'
                    )
                    self._begin_return_to_start()
                    return
                # A.4 contract: normal seed_timeout terminal only when home
                # was never recorded (or return-home disabled / already tried /
                # inhibit blocks defensive home). Do not "simplify" this ladder
                # to skip home when ``_home_xyyaw is not None``.
                self.get_logger().error(
                    f'Deferred seed exceeded T_seed={self._t_seed_s:.0f}s; '
                    f'NO_SAFE_ROUTE (seed_timeout; home_set='
                    f'{self._home_xyyaw is not None})'
                )
                self._raise_no_safe_route('seed_timeout')
                return
        else:
            if not self._pending_follow_seed:
                self._pending_follow_seed_since = 0.0

        # T₁ — PATH_BLOCKED while command_authority=NONE.
        # Do not terminalize while a valid alternate candidate still exists or
        # a switch is queued for when NAV returns (Entry 059).
        if (
            self._state == MissionNavState.PATH_BLOCKED
            and self._owner.command_authority == CommandAuthority.NONE
        ):
            if self._path_blocked_none_since <= 0.0:
                self._path_blocked_none_since = now
            elif path_blocked_none_timed_out(
                elapsed_s=now - self._path_blocked_none_since,
                t1_s=self._t1_path_blocked_none_s,
            ):
                has_alt = False
                if self._live_costmap is not None and self._bank is not None:
                    live = nav2_costmap_to_grid(self._live_costmap)
                    for cand in self._bank.candidates:
                        if cand.state != CandidateState.AVAILABLE:
                            continue
                        ok, _reason = self._candidate_route_valid(cand, live)
                        if ok:
                            has_alt = True
                            break
                if has_alt or self._switch_pending:
                    self.get_logger().warn(
                        f'PATH_BLOCKED+NONE exceeded T₁='
                        f'{self._t1_path_blocked_none_s:.0f}s but alternate/'
                        f'queued switch exists — not raising NO_SAFE_ROUTE'
                    )
                    self._path_blocked_none_since = now  # re-arm wait window
                    if has_alt:
                        self._switch_pending = True
                    return
                # A.3: return-first once when all-blocked and latch false.
                if (
                    self._return_home_on_all_blocked
                    and not self._home_attempted
                    and self._nav_mode != 'home'
                    and not self._owner.has_blocking_inhibit()
                    and not self._owner.operator_stop_latch
                ):
                    self.get_logger().warn(
                        f'PATH_BLOCKED+NONE exceeded T₁='
                        f'{self._t1_path_blocked_none_s:.0f}s — '
                        'return-home first (A.3 latch)'
                    )
                    self._begin_return_to_start()
                    return
                self.get_logger().error(
                    f'PATH_BLOCKED+NONE exceeded T₁='
                    f'{self._t1_path_blocked_none_s:.0f}s; '
                    'NO_SAFE_ROUTE (path_blocked_none_timeout)'
                )
                self._raise_no_safe_route('path_blocked_none_timeout')
                return
        else:
            self._path_blocked_none_since = 0.0

        # T₂ — home wall-clock. Base param covers RH-POS (~8 m arm). GUI
        # dual-block arms at ~18 m; a progressing return was killed at
        # final_dist≈3.5 m by absolute 60 s (GUI-DB wave2c). Scale by arm
        # distance and grant grace while still closing on home.
        if (
            (
                self._nav_mode == 'home'
                or self._state == MissionNavState.RETURNING_TO_START
            )
            and self._home_entered_at > 0.0
        ):
            elapsed = now - self._home_entered_at
            t2 = self._effective_home_t2_s()
            if elapsed >= t2:
                recently_progressed = (
                    self._last_progress_t > 0.0
                    and (now - self._last_progress_t)
                    < max(10.0, self._stuck_timeout_s)
                )
                hard_cap = max(t2 * 2.0, t2 + 60.0)
                if recently_progressed and elapsed < hard_cap:
                    pass  # still closing — do not NO_SAFE mid-progress
                else:
                    self.get_logger().error(
                        f'Home wall-clock exceeded T₂={t2:.0f}s '
                        f'(base={self._t2_home_stuck_s:.0f}s '
                        f'arm={self._dist_home_at_return_arm:.1f}m '
                        f'elapsed={elapsed:.0f}s); NO_SAFE_ROUTE'
                    )
                    self._raise_no_safe_route('home_wallclock_t2')

        # Via per-leg timeout poll (R4 — same timer path as T₂).
        if self._home_via_phase in ('LEG1_PENDING', 'LEG2_PENDING'):
            started = self._home_via_leg_started_at
            if (
                started > 0.0
                and (now - started) > float(self._home_via_leg_timeout_s)
            ):
                self.get_logger().error(
                    f'Via {self._home_via_phase} leg timeout '
                    f'>{self._home_via_leg_timeout_s:.0f}s → NO_SAFE'
                )
                self._raise_no_safe_route('home_footprint_invalid')

    def _resume_home_after_rebuild(self, why: str) -> None:
        """D3 HOME resume after §G settle or unsettled HOME skip-HOLD."""
        self._nav_mode = 'home'
        self._owner.phase = MissionPhase.RETURNING_TO_START
        self._owner.command_authority = CommandAuthority.NAV
        self._owner.detail = 'RETURNING_TO_START'
        self._set_state(MissionNavState.RETURNING_TO_START)
        self._publish_authority_now()
        # Unsettled rebuild freeze must not consume T₂ driving budget.
        if why == 'unsettled':
            self._home_entered_at = time.monotonic()
        # Mid-via: do not blind-replan; wait for via terminal or cancel+raise.
        if self._home_via_phase is not None:
            self.get_logger().warn(
                f'§G: {why} — via mid-flight phase={self._home_via_phase}; '
                'not requesting fresh home plan'
            )
            return
        if (
            self._home_route is not None
            and self._home_route_c1_oriented_valid(self._home_route)[0]
        ):
            self.get_logger().info(
                f'§G: {why} — HOME resume resubmit home FollowPath'
            )
            self._abort_resend_used = False
            self._last_progress_t = time.monotonic()
            if self._robot_xyyaw is not None:
                self._last_progress_xy = (
                    self._robot_xyyaw[0], self._robot_xyyaw[1])
            self._send_follow_path(
                self._home_route, allow_navfn_bridge=False)
        elif self._home_plan_attempt_count < 2:
            self.get_logger().info(
                f'§G: {why} — HOME resume fresh home plan'
            )
            if not self._request_fresh_home_plan():
                self._raise_no_safe_route('home_plan_unavailable')
        else:
            self._raise_no_safe_route('home_footprint_invalid')

    def _effective_home_t2_s(self) -> float:
        """Home T₂ seconds, scaled for long return arms (GUI dual-block).

        Base ``home_stuck_timeout_s`` (60) fits RH-POS (~8 m). GUI-DB arms at
        ~18 m and was killed mid-progress at final_dist≈3.5 m. Scale with a
        conservative closing rate so progressing returns can finish.
        """
        base = float(self._t2_home_stuck_s)
        arm = float(self._dist_home_at_return_arm or 0.0)
        if arm <= 8.0:
            return base
        # ~0.18 m/s average close + 45 s margin; cap 180 s.
        return max(base, min(180.0, arm / 0.18 + 45.0))

    def _publish_active_route_path(self, route: RouteInstance) -> None:
        """Preview RouteInstance geometry (not authoritative FollowPath mirror).

        Authoritative operator Path is published on FollowPath accept via
        ``_publish_follow_path_ref`` (Entry 063 A1).
        """
        stamp = self.get_clock().now().to_msg()
        path_msg = Path()
        path_msg.header.frame_id = self._map_frame
        path_msg.header.stamp = stamp
        for (x, y), yaw in zip(route.points, route.yaw_rad):
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            half = 0.5 * float(yaw)
            ps.pose.orientation.z = math.sin(half)
            ps.pose.orientation.w = math.cos(half)
            path_msg.poses.append(ps)
        self._active_path_pub.publish(path_msg)

    def _publish_follow_path_ref(
        self, path_msg: Path, *, epoch: Optional[int] = None
    ) -> None:
        """Publish exact accepted FollowPath Goal.path to path_bank/active_path."""
        out = Path()
        out.header.frame_id = path_msg.header.frame_id or self._map_frame
        out.header.stamp = self.get_clock().now().to_msg()
        for ps in path_msg.poses:
            p = PoseStamped()
            p.header = out.header
            p.pose = ps.pose
            out.poses.append(p)
        self._active_path_pub.publish(out)
        if epoch is not None:
            self._follow_path_ref_epoch = int(epoch)

    def _clear_active_path(self) -> None:
        """Publish empty Path + DELETEALL markers (terminal / Stop / ARRIVED)."""
        path_msg = Path()
        path_msg.header.frame_id = self._map_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()
        self._active_path_pub.publish(path_msg)
        self._last_submitted_path = None
        self._follow_path_ref_epoch = -1
        clear = MarkerArray()
        m = Marker()
        m.action = Marker.DELETEALL
        clear.markers.append(m)
        self._marker_pub.publish(clear)

    def _set_state(self, state: MissionNavState) -> None:
        if state == self._state:
            return
        self._state = state
        msg = String()
        msg.data = state.value
        self._state_pub.publish(msg)

    def _publish_status(self) -> None:
        lines: List[str] = [
            f'state={self._state.value}',
            f'switches={self._switch_count}',
            f'nav_mode={self._nav_mode or "none"}',
            f'mission_nav_authority={self._mission_nav_authority}',
        ]
        if self._home_xyyaw is not None:
            lines.append(
                f'home=({self._home_xyyaw[0]:.2f},{self._home_xyyaw[1]:.2f},'
                f'{self._home_xyyaw[2]:.2f})'
            )
        # C3 instrumentation for harness / RH-POS.
        lines.append(f'home_attempted={int(bool(self._home_attempted))}')
        lines.append(f'home_plan_pending={int(bool(self._home_plan_pending))}')
        lines.append(f'home_plan_attempt={self._home_plan_attempt_count}')
        if self._fp_cancel_context is not None:
            lines.append(
                f'follow_path_cancel_pending=1 '
                f'epoch={self._fp_cancel_context["epoch"]} '
                f'retry={self._fp_cancel_context["retry"]}'
            )
        else:
            lines.append('follow_path_cancel_pending=0')
        # H6: always emit so harnesses can assert empty vs concrete token.
        lines.append(f'last_no_safe_detail={self._last_no_safe_detail or ""}')
        if self._last_home_footprint_reason:
            lines.append(
                f'home_footprint_invalid={self._last_home_footprint_reason}'
            )
        if self._dist_home_at_return_arm >= 0.0:
            lines.append(
                f'dist_home_at_return_arm={self._dist_home_at_return_arm:.2f}'
            )
        if self._home_plan_pending:
            lines.append('home_plan_status=PENDING')
        elif self._nav_mode == 'home' and self._home_route is not None:
            lines.append('home_plan_status=ARMED')
        elif self._nav_mode == 'home':
            lines.append('home_plan_status=HOME')
        if self._route_bank is not None:
            lines.append(f'active_uid={self._route_bank.active_uid}')
        if self._bank is not None:
            lines.append(f'active_id={self._bank.active_id}')
            lines.append(f'all_blocked={self._bank.all_blocked()}')
            for c in self._bank.candidates:
                lines.append(c.summary())
        msg = String()
        msg.data = '\n'.join(lines)
        self._status_pub.publish(msg)

    def _publish_bank_paths(self) -> None:
        if self._bank is None:
            return
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        # Clear previous
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for c in self._bank.candidates:
            path_msg = Path()
            path_msg.header.frame_id = self._map_frame
            path_msg.header.stamp = stamp
            for x, y in c.points:
                ps = PoseStamped()
                ps.header = path_msg.header
                ps.pose.position.x = x
                ps.pose.position.y = y
                ps.pose.orientation.w = 1.0
                path_msg.poses.append(ps)
            if 1 <= c.path_id <= len(self._alt_path_pubs):
                self._alt_path_pubs[c.path_id - 1].publish(path_msg)
            # Entry 063 A2: do not overwrite authoritative FollowPath ref with
            # sparse cand.points while FOLLOWING / HOME / switching execute.
            # Candidates stay on candidate_* + markers only during mission.
            executing = self._state in (
                MissionNavState.FOLLOWING_PATH,
                MissionNavState.PATH_BLOCKED,
                MissionNavState.SWITCHING_PATH,
                MissionNavState.RETURNING_TO_START,
            ) or self._nav_mode == 'home' or self._follow_path_ref_epoch >= 0
            if (
                c.state == CandidateState.ACTIVE
                and self._nav_mode != 'home'
                and not executing
            ):
                self._active_path_pub.publish(path_msg)

            m = Marker()
            m.header.frame_id = self._map_frame
            m.header.stamp = stamp
            m.ns = 'path_bank'
            m.id = c.path_id
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.12 if c.state == CandidateState.ACTIVE else 0.06
            if c.state == CandidateState.ACTIVE:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.9, 0.2, 1.0
            elif c.state == CandidateState.BLOCKED:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.9, 0.1, 0.1, 0.6
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.5, 0.9, 0.7
            m.pose.orientation.w = 1.0
            from geometry_msgs.msg import Point as GeoPoint
            for x, y in c.points[:: max(1, len(c.points) // 200)]:
                pt = GeoPoint()
                pt.x, pt.y, pt.z = x, y, 0.15
                m.points.append(pt)
            markers.markers.append(m)
        self._marker_pub.publish(markers)


    # -- mission owner services / authority --------------------------------

    def _publish_authority_now(self) -> None:
        if not self._publish_authority:
            return
        msg = String()
        msg.data = self._owner.command_authority.value
        self._authority_pub.publish(msg)

    def _publish_owner_status(self) -> None:
        status = MissionOwnerStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.state = self._owner.phase.value
        status.operator_stop_latch = self._owner.operator_stop_latch
        status.mission_running = self._owner.phase in (
            MissionPhase.NAVIGATING,
            MissionPhase.SWITCHING,
            MissionPhase.RETURNING_TO_START,
            MissionPhase.MANUAL_NAV,
            MissionPhase.COSTMAP_REBUILD,
        )
        status.stop_reason = self._owner.stop_reason
        status.active_inhibits = self._owner.inhibit_codes()
        status.command_authority = self._owner.command_authority.value
        status.detail = self._owner.detail or self._state.value
        status.mission_id = self._owner.mission_id
        status.generation = self._owner.generation
        self._owner_status_pub.publish(status)
        self._publish_authority_now()
        self._tick_pending_failsafe()

    def _tick_pending_failsafe(self) -> None:
        if SafetyInhibit.THERMAL_FIRE_PENDING not in self._owner.active_inhibits:
            self._pending_fire_since = None
            return
        now = time.monotonic()
        if self._pending_fire_since is None:
            self._pending_fire_since = now
            return
        if now - self._pending_fire_since < self._pending_failsafe_s:
            return
        self._set_state(MissionNavState.FIRE_UNLOCALIZED_HOLD)
        self._owner.raise_inhibit(SafetyInhibit.PENDING_FIRE_UNRESOLVED)
        self._cancel_nav()
        self._owner.command_authority = CommandAuthority.NONE
        self._owner.detail = (
            f'FIRE_UNLOCALIZED_HOLD:{self._pending_failsafe_s:.0f}s')
        self._publish_authority_now()
        self.get_logger().error(
            f'FIRE_UNLOCALIZED_HOLD after '
            f'{self._pending_failsafe_s:.0f}s; operator intervention required'
        )

    def raise_blocking_inhibit(self, code: SafetyInhibit) -> None:
        """§C4 RaiseBlockingInhibit — mode-agnostic."""
        self._owner.raise_inhibit(code)
        # Terminal / hold inhibits must not leave a queued switch to resume later.
        if code in (
            SafetyInhibit.PENDING_FIRE_UNRESOLVED,
            SafetyInhibit.ALL_BLOCKED_NO_HOME,
            SafetyInhibit.SENSOR_AUTHORITY_FAULT,
            SafetyInhibit.HOLD_PERSISTENT_SAFETY,
            SafetyInhibit.THERMAL_FIRE_PENDING,
        ):
            self._clear_switch_pending(f'inhibit:{code.value}')
            self._forward_only_until = 0.0
            self._publish_forward_only()
        self._cancel_nav()
        self._publish_authority_now()
        self.get_logger().warn(
            f'RaiseBlockingInhibit {code.value} → phase={self._owner.phase.value} '
            f'auth={self._owner.command_authority.value}'
        )

    def _srv_start_mission(self, request, response):
        reject = self._owner.start_reject_code()
        # Rebuild-recovery exception.
        if (
            reject == SafetyInhibit.HOLD_REBUILD_TIMEOUT.value
            and self._owner.active_inhibits == {SafetyInhibit.HOLD_REBUILD_TIMEOUT}
        ):
            self._owner.bump_generation()
            self._owner.clear_inhibit(SafetyInhibit.HOLD_REBUILD_TIMEOUT)
            self._owner.phase = MissionPhase.COSTMAP_REBUILD
            self._owner.command_authority = CommandAuthority.NONE
            self._publish_authority_now()
            self._revision_gated_local_rebuild()
            response.success = True
            response.message = 'rebuild-recovery accepted; gate NONE until §G'
            response.mission_id = self._owner.mission_id
            response.reject_code = ''
            return response
        if reject:
            response.success = False
            response.message = f'StartMission rejected: {reject}'
            response.mission_id = self._owner.mission_id
            response.reject_code = reject
            return response
        # Persistent / all-blocked require empty or re-eval — Phase 2: reject if present.
        if self._owner.active_inhibits & {
            SafetyInhibit.HOLD_PERSISTENT_SAFETY,
            SafetyInhibit.ALL_BLOCKED_NO_HOME,
        }:
            response.success = False
            response.message = 'StartMission rejected: unresolved safety inhibit'
            response.mission_id = self._owner.mission_id
            response.reject_code = self._owner.inhibit_codes()[0]
            return response
        self._owner.accept_normal_start()
        self._planning_token += 1
        self._switch_action_token += 1
        self._bank_expansion_pending = False
        if self._planning_future is not None:
            self._planning_future.cancel()
            self._planning_future = None
        self._goal_xyyaw = (float(request.goal_x), float(request.goal_y), float(request.goal_yaw))
        self._set_state(MissionNavState.PLANNING)
        self._bank = None
        self._route_bank = None
        self._block_locus.clear()
        self._ever_blocked_ids.clear()
        self._ever_demo_latched_ids.clear()
        self._block_source.clear()
        self._clear_operator_block_latches(reason='StartMission')
        self._last_submitted_path = None
        self._follow_path_ref_epoch = -1
        self._odom_trail.clear()
        self._home_route = None
        self._home_xyyaw = None
        self._nav_mode = None
        self._switch_count = 0
        self._abort_resend_used = False
        self._stuck_recoveries = 0
        self._stuck_recover_until = 0.0
        self._path_blocked_none_since = 0.0
        self._home_entered_at = 0.0
        self._home_attempted = False
        self._home_attempted_from_stuck = False
        self._home_plan_pending = False
        self._home_plan_attempt_count = 0
        self._home_fp_route_uid = None
        self._rebuild_resume_mode = None
        self._last_home_footprint_reason = ''
        self._last_no_safe_detail = ''
        self._last_home_navfn_pts = []
        self._home_via_phase = None
        self._home_via_generation = -1
        self._home_via_v_star = None
        self._home_via_leg1_pts = []
        self._dist_home_at_return_arm = -1.0
        self._far_gap_rebuilds = 0
        self._stem_snap_bridge_fails = 0
        self._bridge_exhaust_drive_retries = 0
        self._entry083_block_diag_logged = False
        self._splice_used = False
        self._splice_pending = False
        self._last_progress_xy = None
        self._last_progress_t = time.monotonic()
        self._fp_cancelled_epochs.clear()
        self._owner.detail = ''
        self._publish_authority_now()
        if self._use_follow_path:
            # Always accept asynchronously — bank Dijkstra is ~60 s and must
            # not block the StartMission service (15 s client waits caused K1b
            # / Issue B harness false rejects while the mission still seeded).
            # Timer path (_on_timer → _try_start_follow_path_bank) builds when
            # pose+map/costmap are ready (T_seed hang bound still applies).
            self._pending_follow_seed = True
            self._pending_follow_seed_since = time.monotonic()
            self._owner.detail = 'PENDING_SEED'
            self.get_logger().info(
                'StartMission accepted; bank seed scheduled on timer '
                f'(T_seed={self._t_seed_s:.0f}s; Dijkstra not inline)'
            )
            response.success = True
            response.message = (
                'StartMission accepted; bank seed scheduled (async Dijkstra)'
            )
            response.mission_id = self._owner.mission_id
            response.reject_code = ''
            return response
        # Kick NavigateToPose (legacy Phase 2); bank builds on plan.
        self._send_goal(*self._goal_xyyaw)
        response.success = True
        response.message = 'StartMission accepted'
        response.mission_id = self._owner.mission_id
        response.reject_code = ''
        return response

    def _try_start_follow_path_bank(self) -> bool:
        """Start one fenced worker for bank construction."""
        if not self._use_follow_path or self._goal_xyyaw is None:
            return False
        if self._robot_xyyaw is None:
            return False
        if self._prior_map is None and self._live_costmap is None:
            return False
        if self._planning_future is not None and not self._planning_future.done():
            return False
        if self._robot_xyyaw is None:
            self.get_logger().warn(
                f'Discarding planning result token={token}: pose unavailable')
            return False
        start = (self._robot_xyyaw[0], self._robot_xyyaw[1])
        goal = (self._goal_xyyaw[0], self._goal_xyyaw[1])
        if self._prior_map is not None:
            grid = occupancy_msg_to_grid(self._prior_map)
        else:
            grid = nav2_costmap_to_grid(self._live_costmap)
        grid.costs = grid.costs.copy()
        token = self._planning_token = self._planning_token + 1
        self._planning_mission_id = self._owner.mission_id
        self._planning_started_at = time.monotonic()
        self._owner.detail = 'PLANNING_WORKER'
        self._planning_grid = grid
        self._planning_start = start
        self._planning_goal = goal
        self._planning_stage = 'initial'
        self._bank_expansion_pending = True
        # Keep a validated alternative ready before motion starts.  A bank
        # containing only the active route cannot make a bounded handoff when
        # that route is later invalidated.
        # Entry 081: length-primary seed so Path 1 is geometrically shortest.
        initial_cfg = replace(
            length_primary_seed_config(self._cfg),
            max_candidates=min(2, self._cfg.max_candidates),
        )
        self._planning_future = self._planning_executor.submit(
            build_path_bank,
            grid,
            start,
            goal,
            initial_cfg,
            footprint_inflate_m=self._footprint_inflate,
            downsample_factor=6,
        )
        self.get_logger().info(
            f'Planning worker started token={token} '
            f'mission={self._planning_mission_id}'
        )
        return False

    def _try_complete_follow_path_bank(self) -> bool:
        """Apply only the current worker result on the ROS executor thread."""
        future = self._planning_future
        if future is None or not future.done():
            return False
        self._planning_future = None
        token = self._planning_token
        if (
            self._owner.mission_id != self._planning_mission_id
            or self._owner.operator_stop_latch
            or self._owner.command_authority != CommandAuthority.NAV
        ):
            self.get_logger().info(
                f'Discarding stale planning result token={token}')
            return False
        try:
            bank = future.result()
        except Exception as exc:
            self.get_logger().error(f'Path-bank worker failed: {exc}')
            self._raise_no_safe_route('planning_worker_failed')
            return False
        if token != self._planning_token or self._goal_xyyaw is None:
            self.get_logger().info(
                f'Discarding superseded planning result token={token}')
            return False
        start = self._planning_start
        goal = self._planning_goal or (self._goal_xyyaw[0], self._goal_xyyaw[1])
        if self._planning_stage == 'initial':
            self._build_bank(start, goal, prebuilt_bank=bank)
            ok = self._bank is not None and bool(self._bank.candidates)
            if not ok:
                return False
            self._pending_follow_seed = False
            latency = time.monotonic() - self._planning_started_at
            self.get_logger().info(
                f'Planning worker completed token={token} '
                f'stage=initial wall_time={latency:.3f}s')
            self._planning_stage = 'expansion'
            # The first blocked-route handoff only needs one validated
            # alternative.  Building the entire four-route bank here can
            # take minutes on the 1200x1200 live prior map, leaving the
            # cancelled mission with no active route.  Expand to two routes
            # first so Path B becomes selectable; the configured bank capacity
            # remains available for later explicit expansion.
            # Entry 081: first expansion also length-primary (Path 1 shortest).
            expansion_cfg = replace(
                length_primary_seed_config(self._cfg),
                max_candidates=min(2, self._cfg.max_candidates),
            )
            self._planning_future = self._planning_executor.submit(
                build_path_bank,
                self._planning_grid,
                start,
                goal,
                expansion_cfg,
                footprint_inflate_m=self._footprint_inflate,
                downsample_factor=4,
            )
            return True
        self._merge_expanded_bank(bank)
        self._planning_stage = ''
        self._bank_expansion_pending = False
        self.get_logger().info(
            f'Planning worker completed token={token} stage=expansion '
            f'candidates={len(bank.candidates)}')
        return True

    def _merge_expanded_bank(self, expanded: PathBank) -> None:
        """Add background alternatives without preempting the active route."""
        if self._bank is None or self._route_bank is None:
            return
        existing = {c.path_id for c in self._bank.candidates}
        active_id = self._bank.active_id
        for candidate in expanded.candidates:
            if candidate.path_id in existing:
                continue
            candidate.state = CandidateState.PENDING
            self._bank.candidates.append(candidate)
            pts = list(candidate.points)
            nudge_grid = self._planning_grid
            if nudge_grid is not None:
                pts = nudge_polyline_away_from_lethal(
                    pts,
                    nudge_grid,
                    target_clearance_m=self._mission_nudge_target_m(),
                    max_offset_m=max(1.0, self._mission_nudge_target_m() + 0.35),
                )
            self._route_bank.routes.append(make_route_instance(
                candidate.path_id,
                pts,
                label=f'path_{candidate.path_id}',
                mission_salt=self._route_bank.mission_salt,
            ))
        live_grid = (
            nav2_costmap_to_grid(self._live_costmap)
            if self._live_costmap is not None else None
        )
        for candidate in self._bank.candidates:
            if candidate.state != CandidateState.PENDING:
                continue
            if live_grid is None:
                continue
            candidate.state = CandidateState.VALIDATING
            ok, reason = self._candidate_route_valid(candidate, live_grid)
            if ok:
                candidate.state = CandidateState.AVAILABLE
            else:
                candidate.state = CandidateState.BLOCKED
                candidate.block_reason = f'merge_validation:{reason}'
        # Entry 081: only force ACTIVE for active_id. Do NOT override
        # merge-validation BLOCKED → AVAILABLE for new candidates.
        self._bank.active_id = active_id
        for candidate in self._bank.candidates:
            if candidate.path_id == active_id:
                candidate.state = CandidateState.ACTIVE
        self._publish_bank_paths()

    def _srv_stop_mission(self, _request, response):
        was_manual = self._owner.phase == MissionPhase.MANUAL_NAV
        self._owner.begin_operator_stop()
        self._planning_token += 1
        self._switch_action_token += 1
        self._bank_expansion_pending = False
        if self._planning_future is not None:
            self._planning_future.cancel()
            self._planning_future = None
        self._publish_authority_now()
        self._cancel_nav()
        self._pending_follow_seed = False
        self._pending_follow_seed_since = 0.0
        self._home_plan_pending = False
        self._home_plan_attempt_count = 0
        self._home_fp_route_uid = None
        self._rebuild_resume_mode = None
        self._nav_mode = None
        self._path_blocked_none_since = 0.0
        self._home_entered_at = 0.0
        self._clear_switch_pending('StopMission')
        self._sealed_unseal_forward_latch = False
        self._forward_only_until = 0.0
        self._publish_forward_only()
        # Stop during §G: discard rebuild resume of obsolete route.
        self._rebuild_pending = False
        # Entry 062: hygiene clear of demo/operator latches on StopMission (R2 m2).
        self._block_locus.clear()
        self._ever_blocked_ids.clear()
        self._ever_demo_latched_ids.clear()
        self._block_source.clear()
        self._clear_operator_block_latches(reason='StopMission')
        # Phase Two §3.5: StopMission MUST clear ALL_BLOCKED_NO_HOME so Start
        # can rearm after NO_SAFE_ROUTE (previously permanent lockout).
        self._owner.clear_all_blocked_on_stop()
        self._owner.finish_operator_stop()
        self._owner.detail = ''
        self._last_no_safe_detail = ''
        self._last_home_navfn_pts = []
        self._home_via_phase = None
        self._home_via_generation = -1
        self._home_via_v_star = None
        self._home_via_leg1_pts = []
        self._set_state(MissionNavState.IDLE)
        self._clear_active_path()
        # Stop from MANUAL_NAV → deactivate BT so bank stays sole FollowPath owner.
        if was_manual and self._use_follow_path:
            self._bt_change_state(
                Transition.TRANSITION_DEACTIVATE, 'StopMission')
        self._publish_authority_now()
        response.success = True
        response.message = 'StopMission verified'
        response.state = self._owner.phase.value
        response.stop_reason = self._owner.stop_reason
        response.active_inhibits = self._owner.inhibit_codes()
        return response

    def _bt_change_state(self, transition_id: int, label: str) -> bool:
        """ChangeState on bt_navigator (ACTIVATE=3, DEACTIVATE=4)."""
        if not self._use_follow_path:
            return True  # legacy: BT stays active under stock manager
        if not self._bt_change_state_cli.service_is_ready():
            self.get_logger().warn(
                f'{label}: bt_navigator/change_state not ready')
            return False
        try:
            req = ChangeState.Request()
            req.transition = Transition()
            req.transition.id = transition_id
            self._bt_change_state_cli.call_async(req)
            self.get_logger().info(
                f'{label}: bt_navigator transition id={transition_id}')
            return True
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'{label}: bt_navigator ChangeState failed: {exc}')
            return False

    def _srv_enable_manual_nav(self, _request, response):
        if self._owner.has_blocking_inhibit():
            response.success = False
            response.message = 'EnableManualNav rejected: active inhibit'
            response.reject_code = self._owner.inhibit_codes()[0]
            return response
        if self._owner.phase not in (MissionPhase.IDLE, MissionPhase.STOPPED):
            response.success = False
            response.message = f'EnableManualNav rejected: phase={self._owner.phase.value}'
            response.reject_code = f'PHASE_{self._owner.phase.value}'
            return response
        # Activate BT before arming BT_MANUAL (bank_follow_path).
        if self._use_follow_path and not self._bt_change_state(
                Transition.TRANSITION_ACTIVATE, 'EnableManualNav'):
            response.success = False
            response.message = 'EnableManualNav rejected: BT activate unavailable'
            response.reject_code = 'BT_ACTIVATE_UNAVAILABLE'
            return response
        self._owner.accept_manual_nav()
        self._publish_authority_now()
        response.success = True
        response.message = 'EnableManualNav accepted'
        response.reject_code = ''
        return response

    def _srv_disable_manual_nav(self, _request, response):
        self._cancel_nav()
        self._owner.disable_manual_nav()
        self._publish_authority_now()
        if self._use_follow_path:
            self._bt_change_state(
                Transition.TRANSITION_DEACTIVATE, 'DisableManualNav')
        response.success = True
        response.message = 'DisableManualNav accepted'
        response.reject_code = ''
        return response

    def _srv_enable_teleop(self, _request, response):
        if self._owner.has_blocking_inhibit():
            response.success = False
            response.message = 'EnableTeleop rejected: active inhibit'
            response.reject_code = self._owner.inhibit_codes()[0]
            return response
        if self._owner.phase not in (MissionPhase.IDLE, MissionPhase.STOPPED):
            response.success = False
            response.message = f'EnableTeleop rejected: phase={self._owner.phase.value}'
            response.reject_code = f'PHASE_{self._owner.phase.value}'
            return response
        self._owner.accept_teleop()
        self._publish_authority_now()
        response.success = True
        response.message = 'EnableTeleop accepted'
        response.reject_code = ''
        return response

    def _srv_disable_teleop(self, _request, response):
        self._owner.disable_teleop()
        self._publish_authority_now()
        response.success = True
        response.message = 'DisableTeleop accepted'
        response.reject_code = ''
        return response

    def destroy_node(self):
        """Stop the planner worker before releasing ROS resources."""
        self._planning_token += 1
        if self._planning_future is not None:
            self._planning_future.cancel()
        self._planning_executor.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()


def _yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathBankManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
