"""Stable RouteInstance identity, densify+yaw, oriented-footprint validation.

Phase 3 (§E): bank owns frozen FollowPath geometry — no silent NavFn rewrite.
"""

from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .path_bank import NO_INFO, OccupancyGrid2D, Point, path_is_valid

# Husky oriented footprint half-extents (match nav2_params_husky1.yaml) + pad.
FOOTPRINT_HALF_L = 0.55
FOOTPRINT_HALF_W = 0.38
FOOTPRINT_PAD = 0.10


@dataclass(frozen=True)
class RouteInstance:
    """Immutable route identity for one mission bank candidate."""

    route_uid: str
    path_id: int
    label: str
    points: Tuple[Point, ...]
    yaw_rad: Tuple[float, ...]
    geometry_fingerprint: str
    densify_m: float = 0.10

    @property
    def length_m(self) -> float:
        if len(self.points) < 2:
            return 0.0
        total = 0.0
        for i in range(1, len(self.points)):
            x0, y0 = self.points[i - 1]
            x1, y1 = self.points[i]
            total += math.hypot(x1 - x0, y1 - y0)
        return total


def geometry_fingerprint(points: Sequence[Point], *, ndigits: int = 3) -> str:
    """Stable hash of polyline coordinates (rounded)."""
    payload = ';'.join(f'{round(x, ndigits):.{ndigits}f},{round(y, ndigits):.{ndigits}f}'
                       for x, y in points)
    return hashlib.sha1(payload.encode('utf-8')).hexdigest()[:16]


def densify_polyline(
    points: Sequence[Point],
    *,
    max_step_m: float = 0.10,
) -> List[Point]:
    """Insert vertices so consecutive spacing ≤ max_step_m."""
    if len(points) < 2:
        return list(points)
    out: List[Point] = [points[0]]
    step = max(1e-3, float(max_step_m))
    for i in range(1, len(points)):
        x0, y0 = out[-1]
        x1, y1 = points[i]
        dist = math.hypot(x1 - x0, y1 - y0)
        if dist <= step:
            out.append((x1, y1))
            continue
        n = int(math.ceil(dist / step))
        for k in range(1, n + 1):
            t = k / n
            out.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
    return out


def along_track_yaw(points: Sequence[Point]) -> List[float]:
    """Yaw at each pose from forward difference (last copies previous)."""
    if not points:
        return []
    if len(points) == 1:
        return [0.0]
    yaws: List[float] = []
    for i in range(len(points) - 1):
        x0, y0 = points[i]
        x1, y1 = points[i + 1]
        yaws.append(math.atan2(y1 - y0, x1 - x0))
    yaws.append(yaws[-1])
    return yaws


def make_route_instance(
    path_id: int,
    points: Sequence[Point],
    *,
    label: str = '',
    densify_m: float = 0.10,
    mission_salt: str = '',
) -> RouteInstance:
    densified = densify_polyline(points, max_step_m=densify_m)
    yaws = along_track_yaw(densified)
    fp = geometry_fingerprint(densified)
    label = label or f'path_{path_id}'
    uid_src = f'{mission_salt}|{path_id}|{label}|{fp}'
    route_uid = hashlib.sha1(uid_src.encode('utf-8')).hexdigest()[:12]
    return RouteInstance(
        route_uid=route_uid,
        path_id=int(path_id),
        label=label,
        points=tuple(densified),
        yaw_rad=tuple(yaws),
        geometry_fingerprint=fp,
        densify_m=float(densify_m),
    )


def _footprint_corners(
    x: float,
    y: float,
    yaw: float,
    *,
    half_l: float = FOOTPRINT_HALF_L,
    half_w: float = FOOTPRINT_HALF_W,
    pad: float = FOOTPRINT_PAD,
) -> List[Point]:
    hl, hw = half_l + pad, half_w + pad
    local = [(hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)]
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * lx - s * ly, y + s * lx + c * ly) for lx, ly in local]


def _sample_cost(
    grid: OccupancyGrid2D,
    x: float,
    y: float,
) -> Optional[int]:
    idx = grid.world_to_grid(x, y)
    if idx is None:
        return None
    return int(grid.costs[idx[1], idx[0]])


def oriented_footprint_path_valid(
    points: Sequence[Point],
    yaws: Sequence[float],
    grid: OccupancyGrid2D,
    *,
    lethal_threshold: int = 254,
    reject_unknown: bool = True,
    sample_stride: int = 2,
    half_l: float = FOOTPRINT_HALF_L,
    half_w: float = FOOTPRINT_HALF_W,
    pad: float = FOOTPRINT_PAD,
    mid_edge_samples: int = 1,
) -> Tuple[bool, str]:
    """Reject if any oriented footprint sample hits lethal/unknown.

    Samples rectangle corners (+ optional edge midpoints) at densified poses.
    """
    if not points:
        return False, 'empty_path'
    if len(yaws) != len(points):
        return False, 'yaw_length_mismatch'
    stride = max(1, int(sample_stride))
    for i in range(0, len(points), stride):
        x, y = points[i]
        yaw = yaws[i]
        corners = _footprint_corners(
            x, y, yaw, half_l=half_l, half_w=half_w, pad=pad)
        samples = list(corners)
        if mid_edge_samples > 0:
            for a, b in zip(corners, corners[1:] + corners[:1]):
                samples.append(((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5))
        samples.append((x, y))
        for sx, sy in samples:
            c = _sample_cost(grid, sx, sy)
            if c is None:
                return False, f'off_map_at_{i}'
            if c == NO_INFO and reject_unknown:
                return False, f'unknown_at_{i}'
            if c >= lethal_threshold and c != NO_INFO:
                return False, f'lethal_at_{i}_cost_{c}'
    return True, ''


def mask_under_oriented_footprint(
    points: Sequence[Point],
    yaws: Sequence[float],
    *,
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    data: Sequence[int],
    occupied_threshold: int = 50,
    sample_stride: int = 2,
    half_l: float = FOOTPRINT_HALF_L,
    half_w: float = FOOTPRINT_HALF_W,
    pad: float = FOOTPRINT_PAD,
) -> bool:
    """True if OccupancyGrid mask intersects oriented footprint samples."""
    if not points or not data or width <= 0 or height <= 0 or resolution <= 0:
        return False
    if len(yaws) != len(points):
        return True  # treat as blocked if malformed
    stride = max(1, int(sample_stride))
    for i in range(0, len(points), stride):
        x, y = points[i]
        corners = _footprint_corners(
            x, y, yaws[i], half_l=half_l, half_w=half_w, pad=pad)
        for sx, sy in corners + [(x, y)]:
            ix = int((sx - origin_x) / resolution)
            iy = int((sy - origin_y) / resolution)
            if 0 <= ix < width and 0 <= iy < height:
                if int(data[iy * width + ix]) >= occupied_threshold:
                    return True
    return False


def centreline_clear_body_lethal_reject(
    points: Sequence[Point],
    yaws: Sequence[float],
    grid: OccupancyGrid2D,
    *,
    lethal_threshold: int = 254,
) -> Tuple[bool, str]:
    """Acceptance helper: centreline OK but body lethal → must reject.

    Returns (should_reject, reason). should_reject True means oriented check
    catches a hazard that centreline-only path_is_valid misses.
    """
    ok_c, _ = path_is_valid(
        points, grid, lethal_threshold=lethal_threshold, sample_stride=2)
    if not ok_c:
        return False, 'centreline_already_blocked'
    ok_o, reason = oriented_footprint_path_valid(
        points, yaws, grid, lethal_threshold=lethal_threshold)
    if ok_o:
        return False, 'both_clear'
    return True, reason


@dataclass
class RouteBank:
    """Mission-scoped RouteInstances bound once (stable UIDs)."""

    mission_salt: str
    routes: List[RouteInstance] = field(default_factory=list)
    active_uid: Optional[str] = None

    def by_path_id(self, path_id: int) -> Optional[RouteInstance]:
        for r in self.routes:
            if r.path_id == path_id:
                return r
        return None

    def by_uid(self, uid: str) -> Optional[RouteInstance]:
        for r in self.routes:
            if r.route_uid == uid:
                return r
        return None

    def active(self) -> Optional[RouteInstance]:
        if self.active_uid is None:
            return None
        return self.by_uid(self.active_uid)
