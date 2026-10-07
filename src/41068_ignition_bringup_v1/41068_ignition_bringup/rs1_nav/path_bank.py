"""Path-bank generation, scoring, validity, and mission-scoped selection.

This module does NOT replace Nav2. NavFn + the IsPathValid BT remain the
authority for follow/replan timing. The path bank:

1. Builds alternative routes on a known occupancy/cost grid at mission start
2. Ranks them with a simple, documented score
3. Marks a candidate permanently unavailable for the mission when it becomes
   collision-invalid on the live costmap
4. Hands the next still-valid candidate to the manager for a controlled switch

Candidate diversity uses corridor exclusion: after each successful path, a band
around that polyline is treated as lethal on a working copy of the grid and
planning is repeated. That yields geometrically different routes without
hard-coding Path A / Path B.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .geometry import path_length

Point = Tuple[float, float]
GridIndex = Tuple[int, int]

# Nav2 costmap conventions (uint8)
LETHAL = 254
INSCRIBED = 253
NO_INFO = 255


class CandidateState(str, Enum):
    AVAILABLE = 'available'
    ACTIVE = 'active'
    BLOCKED = 'blocked'
    REJECTED = 'rejected'


@dataclass
class PathBankConfig:
    """Tunable path-bank behaviour (ROS parameters map onto these)."""

    max_candidates: int = 4
    exclusion_radius_m: float = 2.5
    min_path_separation_m: float = 2.0
    min_clearance_m: float = 0.30
    # Match Nav2 IsPathValid / lethal semantics: 254 = LETHAL_OBSTACLE.
    # Cost 253 is inscribed soft cost — RPP slows there; it is NOT a blockage.
    lethal_threshold: int = 254
    length_weight: float = 0.45
    clearance_weight: float = 0.35
    cost_weight: float = 0.15
    smoothness_weight: float = 0.05
    clearance_ref_m: float = 2.0
    max_turn_rad: float = math.pi
    validity_lookahead_m: float = 0.0  # 0 = whole remaining path
    sample_stride: int = 2


@dataclass
class PathCandidate:
    path_id: int
    points: List[Point]
    length_m: float
    min_clearance_m: float
    mean_cost: float
    turn_penalty: float
    score: float
    state: CandidateState = CandidateState.AVAILABLE
    block_reason: str = ''

    def summary(self) -> str:
        return (
            f'Path {self.path_id}: score={self.score:.1f} len={self.length_m:.1f}m '
            f'clear={self.min_clearance_m:.2f}m cost={self.mean_cost:.0f} '
            f'state={self.state.value}'
            + (f' ({self.block_reason})' if self.block_reason else '')
        )


@dataclass
class PathBank:
    """Mission-scoped bank of ranked alternative routes."""

    start: Point
    goal: Point
    candidates: List[PathCandidate] = field(default_factory=list)
    active_id: Optional[int] = None

    def available(self) -> List[PathCandidate]:
        return [c for c in self.candidates if c.state == CandidateState.AVAILABLE]

    def active(self) -> Optional[PathCandidate]:
        if self.active_id is None:
            return None
        for c in self.candidates:
            if c.path_id == self.active_id:
                return c
        return None

    def select_best_available(self) -> Optional[PathCandidate]:
        pool = [
            c for c in self.candidates
            if c.state in (CandidateState.AVAILABLE, CandidateState.ACTIVE)
        ]
        if not pool:
            return None
        # Prefer highest score among non-blocked.
        pool = [c for c in self.candidates if c.state == CandidateState.AVAILABLE]
        if not pool:
            return None
        best = max(pool, key=lambda c: c.score)
        for c in self.candidates:
            if c.state == CandidateState.ACTIVE:
                c.state = CandidateState.AVAILABLE
        best.state = CandidateState.ACTIVE
        self.active_id = best.path_id
        return best

    def mark_blocked(self, path_id: int, reason: str) -> None:
        for c in self.candidates:
            if c.path_id == path_id:
                c.state = CandidateState.BLOCKED
                c.block_reason = reason
                if self.active_id == path_id:
                    self.active_id = None
                return

    def all_blocked(self) -> bool:
        return bool(self.candidates) and all(
            c.state in (CandidateState.BLOCKED, CandidateState.REJECTED)
            for c in self.candidates
        )


@dataclass
class OccupancyGrid2D:
    """World-frame occupancy/cost grid used for offline planning."""

    origin_x: float
    origin_y: float
    resolution: float
    width: int
    height: int
    # uint8 costs: 0 free … 254 lethal, 255 unknown. Planning treats unknown
    # as traversable with a mild penalty so a full static map still works when
    # a few cells are unmarked.
    costs: np.ndarray  # shape (height, width), dtype uint8

    def world_to_grid(self, x: float, y: float) -> Optional[GridIndex]:
        ix = int((x - self.origin_x) / self.resolution)
        iy = int((y - self.origin_y) / self.resolution)
        if 0 <= ix < self.width and 0 <= iy < self.height:
            return ix, iy
        return None

    def grid_to_world(self, ix: int, iy: int) -> Point:
        return (
            self.origin_x + (ix + 0.5) * self.resolution,
            self.origin_y + (iy + 0.5) * self.resolution,
        )


def occupancy_msg_to_grid(msg) -> OccupancyGrid2D:
    """Convert nav_msgs/OccupancyGrid (-1/0/100) into Nav2-like uint8 costs."""
    info = msg.info
    data = np.asarray(msg.data, dtype=np.int16).reshape((info.height, info.width))
    costs = np.full(data.shape, NO_INFO, dtype=np.uint8)
    costs[data == 0] = 0
    costs[data >= 50] = LETHAL
    # Unknown stays NO_INFO.
    return OccupancyGrid2D(
        origin_x=float(info.origin.position.x),
        origin_y=float(info.origin.position.y),
        resolution=float(info.resolution),
        width=int(info.width),
        height=int(info.height),
        costs=costs,
    )


def nav2_costmap_to_grid(msg) -> OccupancyGrid2D:
    """Convert nav2_msgs/Costmap into OccupancyGrid2D."""
    meta = msg.metadata
    h, w = int(meta.size_y), int(meta.size_x)
    costs = np.asarray(msg.data, dtype=np.uint8).reshape((h, w))
    return OccupancyGrid2D(
        origin_x=float(meta.origin.position.x),
        origin_y=float(meta.origin.position.y),
        resolution=float(meta.resolution),
        width=w,
        height=h,
        costs=costs,
    )


def inflate_lethal(grid: OccupancyGrid2D, radius_m: float) -> OccupancyGrid2D:
    """Binary dilation of lethal cells by radius_m (footprint clearance)."""
    if radius_m <= 0.0:
        return OccupancyGrid2D(
            grid.origin_x, grid.origin_y, grid.resolution,
            grid.width, grid.height, grid.costs.copy(),
        )
    cells = max(1, int(math.ceil(radius_m / grid.resolution)))
    lethal = grid.costs >= LETHAL
    out = grid.costs.copy()
    ys, xs = np.where(lethal)
    for iy, ix in zip(ys.tolist(), xs.tolist()):
        y0, y1 = max(0, iy - cells), min(grid.height, iy + cells + 1)
        x0, x1 = max(0, ix - cells), min(grid.width, ix + cells + 1)
        # Circular mask
        yy, xx = np.ogrid[y0:y1, x0:x1]
        mask = (yy - iy) ** 2 + (xx - ix) ** 2 <= cells * cells
        region = out[y0:y1, x0:x1]
        region[mask] = np.maximum(region[mask], LETHAL)
    return OccupancyGrid2D(
        grid.origin_x, grid.origin_y, grid.resolution,
        grid.width, grid.height, out,
    )


def _is_traversable(cost: int, allow_unknown: bool) -> bool:
    if cost >= LETHAL and cost != NO_INFO:
        return False
    if cost == NO_INFO:
        return allow_unknown
    return True


def _cell_traversal_cost(cost: int) -> float:
    if cost == NO_INFO:
        return 1.5
    if cost >= LETHAL:
        return 1e6
    # Soft inflation costs raise the path cost so clearance is preferred.
    return 1.0 + (float(cost) / 253.0) * 4.0


def plan_dijkstra(
    grid: OccupancyGrid2D,
    start: Point,
    goal: Point,
    *,
    allow_unknown: bool = True,
) -> Optional[List[Point]]:
    """8-connected Dijkstra on the cost grid. Returns world-frame polyline."""
    s = grid.world_to_grid(*start)
    g = grid.world_to_grid(*goal)
    if s is None or g is None:
        return None
    if not _is_traversable(int(grid.costs[s[1], s[0]]), allow_unknown):
        return None
    if not _is_traversable(int(grid.costs[g[1], g[0]]), allow_unknown):
        return None

    w, h = grid.width, grid.height
    dist = np.full((h, w), np.inf, dtype=np.float64)
    parent = np.full((h, w, 2), -1, dtype=np.int32)
    sx, sy = s
    gx, gy = g
    dist[sy, sx] = 0.0
    heap: List[Tuple[float, int, int]] = [(0.0, sx, sy)]
    neigh = [
        (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
        (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2)),
    ]
    found = False
    while heap:
        d, x, y = heapq.heappop(heap)
        if d > dist[y, x]:
            continue
        if x == gx and y == gy:
            found = True
            break
        for dx, dy, step in neigh:
            nx, ny = x + dx, y + dy
            if not (0 <= nx < w and 0 <= ny < h):
                continue
            c = int(grid.costs[ny, nx])
            if not _is_traversable(c, allow_unknown):
                continue
            nd = d + step * _cell_traversal_cost(c)
            if nd < dist[ny, nx]:
                dist[ny, nx] = nd
                parent[ny, nx] = (x, y)
                heapq.heappush(heap, (nd, nx, ny))

    if not found or not np.isfinite(dist[gy, gx]):
        return None

    # Reconstruct
    cells: List[GridIndex] = []
    cx, cy = gx, gy
    while not (cx == sx and cy == sy):
        cells.append((cx, cy))
        px, py = int(parent[cy, cx, 0]), int(parent[cy, cx, 1])
        if px < 0:
            return None
        cx, cy = px, py
    cells.append((sx, sy))
    cells.reverse()
    return [grid.grid_to_world(ix, iy) for ix, iy in cells]


def exclude_corridor(
    grid: OccupancyGrid2D,
    path: Sequence[Point],
    radius_m: float,
    *,
    fraction: Tuple[float, float] = (0.25, 0.75),
) -> OccupancyGrid2D:
    """Return a copy with a lethal band around part of `path`.

    Only the middle fraction of the polyline is excluded by default. Full-path
    exclusion often disconnects narrow trail maps (single corridor topology);
    middle exclusion forces a genuine detour while keeping start/goal free.
    """
    out = OccupancyGrid2D(
        grid.origin_x, grid.origin_y, grid.resolution,
        grid.width, grid.height, grid.costs.copy(),
    )
    if not path or radius_m <= 0.0:
        return out
    n = len(path)
    i0 = int(max(0.0, min(1.0, fraction[0])) * (n - 1))
    i1 = int(max(0.0, min(1.0, fraction[1])) * (n - 1)) + 1
    segment = path[i0:i1]
    cells = max(1, int(math.ceil(radius_m / grid.resolution)))
    stride = max(1, cells // 2)
    for x, y in segment[::stride]:
        idx = out.world_to_grid(x, y)
        if idx is None:
            continue
        ix, iy = idx
        y0, y1 = max(0, iy - cells), min(out.height, iy + cells + 1)
        x0, x1 = max(0, ix - cells), min(out.width, ix + cells + 1)
        yy, xx = np.ogrid[y0:y1, x0:x1]
        mask = (yy - iy) ** 2 + (xx - ix) ** 2 <= cells * cells
        region = out.costs[y0:y1, x0:x1]
        region[mask] = LETHAL
    return out


def _via_points_around_path(
    path: Sequence[Point],
    grid: OccupancyGrid2D,
    *,
    offsets_m: Sequence[float] = (-4.0, 4.0, -7.0, 7.0),
    at_fractions: Sequence[float] = (0.35, 0.5, 0.65),
) -> List[Point]:
    """Sample free-space vias perpendicular to the primary path."""
    if len(path) < 2:
        return []
    vias: List[Point] = []
    for frac in at_fractions:
        i = int(frac * (len(path) - 1))
        i = max(1, min(len(path) - 2, i))
        ax, ay = path[i - 1]
        bx, by = path[i + 1]
        dx, dy = bx - ax, by - ay
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            continue
        px, py = -dy / norm, dx / norm  # unit perpendicular
        cx, cy = path[i]
        for off in offsets_m:
            vx, vy = cx + px * off, cy + py * off
            idx = grid.world_to_grid(vx, vy)
            if idx is None:
                continue
            if int(grid.costs[idx[1], idx[0]]) >= LETHAL:
                continue
            vias.append((vx, vy))
    return vias


def _plan_via(
    grid: OccupancyGrid2D,
    start: Point,
    via: Point,
    goal: Point,
) -> Optional[List[Point]]:
    a = plan_dijkstra(grid, start, via, allow_unknown=True)
    b = plan_dijkstra(grid, via, goal, allow_unknown=True)
    if a is None or b is None:
        return None
    return a + b[1:]


def _min_clearance(path: Sequence[Point], grid: OccupancyGrid2D) -> float:
    lethal = grid.costs >= LETHAL
    if not lethal.any() or not path:
        return 10.0
    ys, xs = np.where(lethal)
    obs = np.column_stack([
        grid.origin_x + (xs + 0.5) * grid.resolution,
        grid.origin_y + (ys + 0.5) * grid.resolution,
    ])
    best = float('inf')
    stride = max(1, len(path) // 80)
    for x, y in path[::stride]:
        d = np.min(np.hypot(obs[:, 0] - x, obs[:, 1] - y))
        if d < best:
            best = float(d)
    return best if best < float('inf') else 10.0


def _mean_cost(path: Sequence[Point], grid: OccupancyGrid2D) -> float:
    if not path:
        return 0.0
    vals = []
    for x, y in path[:: max(1, len(path) // 100)]:
        idx = grid.world_to_grid(x, y)
        if idx is None:
            continue
        c = int(grid.costs[idx[1], idx[0]])
        if c == NO_INFO:
            continue
        vals.append(float(c))
    return sum(vals) / len(vals) if vals else 0.0


def _turn_penalty(path: Sequence[Point]) -> float:
    if len(path) < 3:
        return 0.0
    total = 0.0
    count = 0
    for a, b, c in zip(path, path[1:], path[2:]):
        v1 = (b[0] - a[0], b[1] - a[1])
        v2 = (c[0] - b[0], c[1] - b[1])
        n1 = math.hypot(*v1)
        n2 = math.hypot(*v2)
        if n1 < 1e-6 or n2 < 1e-6:
            continue
        ang = abs(math.atan2(
            v1[0] * v2[1] - v1[1] * v2[0],
            v1[0] * v2[0] + v1[1] * v2[1],
        ))
        total += ang
        count += 1
    return (total / count) if count else 0.0


def _path_mean_separation(a: Sequence[Point], b: Sequence[Point]) -> float:
    """Average nearest distance from samples of a to polyline b."""
    if not a or not b:
        return 0.0
    dists = []
    stride = max(1, len(a) // 40)
    for p in a[::stride]:
        best = float('inf')
        for q in b[:: max(1, len(b) // 40)]:
            best = min(best, math.hypot(p[0] - q[0], p[1] - q[1]))
        dists.append(best)
    return sum(dists) / len(dists) if dists else 0.0


def score_candidate(
    length_m: float,
    min_clearance_m: float,
    mean_cost: float,
    turn_penalty: float,
    *,
    ref_length_m: float,
    cfg: PathBankConfig,
) -> float:
    """Higher is better. Normalised blend of length, clearance, cost, smoothness.

    score = 100 * (
        w_len   * (1 - length / ref_length) clamped
      + w_clear * (min_clearance / clearance_ref) clamped
      + w_cost  * (1 - mean_cost / 253)
      + w_smooth* (1 - turn_penalty / max_turn)
    )
    """
    ref_len = max(ref_length_m, 1e-3)
    length_term = max(0.0, min(1.0, 1.0 - (length_m - ref_length_m) / ref_len))
    # Prefer not-much-longer than the shortest; still reward absolute shortness.
    length_term = max(0.0, min(1.0, ref_length_m / max(length_m, 1e-3)))
    clear_term = max(0.0, min(1.0, min_clearance_m / max(cfg.clearance_ref_m, 1e-3)))
    cost_term = max(0.0, min(1.0, 1.0 - mean_cost / 253.0))
    smooth_term = max(0.0, min(1.0, 1.0 - turn_penalty / max(cfg.max_turn_rad, 1e-3)))
    raw = (
        cfg.length_weight * length_term
        + cfg.clearance_weight * clear_term
        + cfg.cost_weight * cost_term
        + cfg.smoothness_weight * smooth_term
    )
    return 100.0 * raw


def path_is_valid(
    path: Sequence[Point],
    grid: OccupancyGrid2D,
    *,
    lethal_threshold: int = 253,
    sample_stride: int = 2,
    start_index: int = 0,
) -> Tuple[bool, str]:
    """True iff no sampled cell along the path is at/above lethal_threshold."""
    if not path:
        return False, 'empty_path'
    for i, (x, y) in enumerate(path[start_index:: max(1, sample_stride)]):
        idx = grid.world_to_grid(x, y)
        if idx is None:
            return False, 'off_map'
        c = int(grid.costs[idx[1], idx[0]])
        if c >= lethal_threshold and c != NO_INFO:
            return False, f'lethal_at_index_{start_index + i * sample_stride}_cost_{c}'
    return True, ''


def downsample_grid(grid: OccupancyGrid2D, factor: int) -> OccupancyGrid2D:
    """Max-pool downsample (lethal wins) for faster candidate search."""
    factor = max(1, int(factor))
    if factor == 1:
        return grid
    h, w = grid.height, grid.width
    nh, nw = h // factor, w // factor
    if nh < 2 or nw < 2:
        return grid
    trimmed = grid.costs[: nh * factor, : nw * factor]
    reshaped = trimmed.reshape(nh, factor, nw, factor)
    # Prefer lethal over free when pooling.
    pooled = reshaped.max(axis=(1, 3))
    return OccupancyGrid2D(
        origin_x=grid.origin_x,
        origin_y=grid.origin_y,
        resolution=grid.resolution * factor,
        width=nw,
        height=nh,
        costs=pooled.astype(np.uint8),
    )


def build_path_bank(
    planning_grid: OccupancyGrid2D,
    start: Point,
    goal: Point,
    cfg: Optional[PathBankConfig] = None,
    *,
    footprint_inflate_m: float = 0.38,
    downsample_factor: int = 2,
) -> PathBank:
    """Generate, validate, score and rank up to max_candidates diverse routes."""
    cfg = cfg or PathBankConfig()
    # Plan on a lightly downsampled grid for speed; validate clearance on the
    # full-resolution source grid when available.
    source = planning_grid
    planning_grid = downsample_grid(planning_grid, downsample_factor)
    base = inflate_lethal(planning_grid, footprint_inflate_m)
    bank = PathBank(start=start, goal=goal)
    working = base
    raw_paths: List[List[Point]] = []

    def _try_accept(path: Optional[List[Point]]) -> bool:
        if path is None:
            return False
        if any(
            _path_mean_separation(path, prev) < cfg.min_path_separation_m
            for prev in raw_paths
        ):
            return False
        ok, _reason = path_is_valid(
            path, base, lethal_threshold=cfg.lethal_threshold,
            sample_stride=cfg.sample_stride,
        )
        if not ok:
            return False
        clear = _min_clearance(path, source)
        if clear < cfg.min_clearance_m:
            return False
        raw_paths.append(path)
        return True

    # Phase 1: primary Dijkstra + middle-corridor exclusion.
    for attempt in range(cfg.max_candidates * 2):
        if len(raw_paths) >= cfg.max_candidates:
            break
        path = plan_dijkstra(working, start, goal, allow_unknown=True)
        if path is None:
            break
        accepted = _try_accept(path)
        # Always push the search away from this polyline (or a rejected twin).
        working = exclude_corridor(
            working, path, cfg.exclusion_radius_m,
            fraction=(0.2 + 0.05 * attempt, 0.8 - 0.05 * attempt),
        )
        if not accepted and attempt > cfg.max_candidates:
            break

    # Phase 2: via-point detours if exclusion alone cannot diversify (narrow
    # single-corridor maps like bush_trail often need this).
    if raw_paths and len(raw_paths) < cfg.max_candidates:
        for via in _via_points_around_path(raw_paths[0], base):
            if len(raw_paths) >= cfg.max_candidates:
                break
            _try_accept(_plan_via(base, start, via, goal))

    if not raw_paths:
        return bank

    lengths = [path_length(p) for p in raw_paths]
    ref_len = min(lengths)
    for i, path in enumerate(raw_paths):
        length = lengths[i]
        clear = _min_clearance(path, source)
        mean_c = _mean_cost(path, source)
        turns = _turn_penalty(path)
        sc = score_candidate(
            length, clear, mean_c, turns, ref_length_m=ref_len, cfg=cfg,
        )
        bank.candidates.append(PathCandidate(
            path_id=i + 1,
            points=path,
            length_m=length,
            min_clearance_m=clear,
            mean_cost=mean_c,
            turn_penalty=turns,
            score=sc,
            state=CandidateState.AVAILABLE,
        ))

    bank.candidates.sort(key=lambda c: c.score, reverse=True)
    # Re-number by rank for operator clarity (1 = best).
    for i, c in enumerate(bank.candidates):
        c.path_id = i + 1
    return bank
