"""Sensor-only obstacle memory: typed cells with evidence-based clear rules.

KeepoutFilter overlays an OccupancyGrid mask on the global costmap. This module
owns the pure mark/clear predicates (no ROS). The manager node publishes the
mask; Nav2 KeepoutFilter raises costs. Cells are never freed by message absence,
Gazebo ground truth, or lidar rays through FIRE cells.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

GridIndex = Tuple[int, int]
Point = Tuple[float, float]


class CellSource(str, Enum):
    LIDAR = 'lidar'
    FIRE = 'fire'


class ClearResult(str, Enum):
    ACCEPTED = 'CLEARING_OBSERVATION_ACCEPTED'
    REJECTED = 'CLEARING_OBSERVATION_REJECTED'
    STALE = 'SENSOR_STALE'
    OUT_OF_FOV = 'OUT_OF_FOV'
    WRONG_SOURCE = 'WRONG_SOURCE'
    NOT_MARKED = 'NOT_MARKED'
    HOLD_ACTIVE = 'FIRE_HOLD_ACTIVE'
    NEED_MORE_NEGATIVES = 'FIRE_NEED_MORE_NEGATIVES'
    OCCLUDED = 'FIRE_OCCLUDED'


@dataclass
class MemoryCell:
    ix: int
    iy: int
    source: CellSource
    last_marked_time: float
    fire_clear_confirm_count: int = 0


@dataclass
class ObstacleMemoryConfig:
    """Tunables for mark/clear predicates."""

    max_scan_age_s: float = 0.5
    clear_max_range_m: float = 10.0
    raytrace_max_m: float = 12.0
    range_epsilon_m: float = 0.15
    mark_neighbour_radius_cells: int = 0
    # Skip prior-free cells within this many cells of prior occupied (bank edges).
    prior_edge_margin_cells: int = 4
    obstacle_min_range_m: float = 0.2
    # Memory marks only mid-range hits so distant corridor walls on free PGM
    # cells do not permanently seal remote trail segments.
    obstacle_max_range_m: float = 12.0
    # Entry 023: never mark keepout under/near the robot footprint (Humble
    # KeepoutFilter has no override_lethal_cost to escape footprint traps).
    # ≈ hypot(0.55, 0.38) husky half-extents + small margin.
    mark_exclude_radius_m: float = 0.65
    # Fire: consecutive in-FOV negatives after hold expires.
    fire_clear_confirm_scans: int = 3
    # Entry 027: camera-frame ROI frustum (beer_thermal_camera HFOV=1.047,
    # 320x240 → VFOV≈0.810; detector ROI 0.6×0.7). Clear ⊆ detect FOV.
    # Legacy thermal_cone_half_angle_rad is unused by the frustum gate.
    thermal_hfov_rad: float = 1.047
    thermal_vfov_rad: float = 0.810
    thermal_roi_width_frac: float = 0.6
    thermal_roi_height_frac: float = 0.7
    thermal_near_m: float = 0.1
    thermal_max_range_m: float = 8.0
    # Fallback camera height (map z) when TF z is not supplied.
    thermal_camera_height_m: float = 0.70
    thermal_cone_half_angle_rad: float = 0.5235  # HFOV/2; mark-bridge align only
    # beer_thermal_camera <pose> relative to camera_link (husky.gazebo.xacro).
    # Applied by the manager; unit tests pass 0 unless testing the offset.
    thermal_optical_forward_m: float = 0.45
    thermal_optical_up_m: float = 0.45
    # A LiDAR hit this close to the FIRE cell is the same cluster (0.9 m disc),
    # not a nearer occluder. Only a distinctly nearer return vetoes the clear.
    thermal_occlusion_margin_m: float = 1.0


@dataclass
class MapMeta:
    origin_x: float
    origin_y: float
    resolution: float
    width: int
    height: int
    frame_id: str = 'husky1_map'

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


@dataclass
class ObstacleMemory:
    """Typed cell store with lidar/fire clear evidence rules."""

    meta: MapMeta
    cfg: ObstacleMemoryConfig = field(default_factory=ObstacleMemoryConfig)
    cells: Dict[GridIndex, MemoryCell] = field(default_factory=dict)

    def occupied_count(self) -> int:
        return len(self.cells)

    def has_cell(self, ix: int, iy: int) -> bool:
        return (ix, iy) in self.cells

    def cell_source(self, ix: int, iy: int) -> Optional[CellSource]:
        c = self.cells.get((ix, iy))
        return c.source if c else None

    # -- mark --------------------------------------------------------------

    def _neighbour_indices(self, ix: int, iy: int) -> List[GridIndex]:
        r = max(0, int(self.cfg.mark_neighbour_radius_cells))
        out: List[GridIndex] = []
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                nx, ny = ix + dx, iy + dy
                if 0 <= nx < self.meta.width and 0 <= ny < self.meta.height:
                    out.append((nx, ny))
        return out

    def mark_world(
        self,
        x: float,
        y: float,
        source: CellSource,
        now: float,
        *,
        prior_free: Optional[np.ndarray] = None,
        prior_edge_margin_cells: int = 2,
        robot_xy: Optional[Point] = None,
    ) -> List[GridIndex]:
        """Mark hit cell (+ neighbours).

        If ``prior_free`` is set (H×W bool), only mark cells that are free on
        the static prior *and* at least ``prior_edge_margin_cells`` away from
        prior occupied — skips trail-bank lidar returns that sit on free PGM
        cells next to StaticLayer lethals (would otherwise permanently seal
        corridors). True interior free hits (dynamic barriers) are retained.

        Entry 023: if ``robot_xy`` is set, skip cells within
        ``cfg.mark_exclude_radius_m`` of the robot (footprint keepout trap).
        """
        idx = self.meta.world_to_grid(x, y)
        if idx is None:
            return []
        exclude_r = max(0.0, float(self.cfg.mark_exclude_radius_m))
        marked: List[GridIndex] = []
        for nx, ny in self._neighbour_indices(*idx):
            if prior_free is not None:
                if ny >= prior_free.shape[0] or nx >= prior_free.shape[1]:
                    continue
                if not bool(prior_free[ny, nx]):
                    continue
                if prior_edge_margin_cells > 0 and not _is_interior_free(
                    prior_free, nx, ny, prior_edge_margin_cells
                ):
                    continue
            if robot_xy is not None and exclude_r > 0.0:
                cx, cy = self.meta.grid_to_world(nx, ny)
                if math.hypot(cx - robot_xy[0], cy - robot_xy[1]) <= exclude_r:
                    continue
            existing = self.cells.get((nx, ny))
            # Do not downgrade FIRE → LIDAR; lidar must not own fire cells.
            if existing is not None and existing.source == CellSource.FIRE and source == CellSource.LIDAR:
                continue
            src = CellSource.FIRE if source == CellSource.FIRE else (
                existing.source if existing is not None else source
            )
            if source == CellSource.FIRE:
                src = CellSource.FIRE
            self.cells[(nx, ny)] = MemoryCell(
                ix=nx,
                iy=ny,
                source=src,
                last_marked_time=now,
                fire_clear_confirm_count=0,
            )
            marked.append((nx, ny))
        return marked

    def mark_lidar_hit(
        self,
        x: float,
        y: float,
        now: float,
        *,
        prior_free: Optional[np.ndarray] = None,
        robot_xy: Optional[Point] = None,
    ) -> List[GridIndex]:
        return self.mark_world(
            x, y, CellSource.LIDAR, now,
            prior_free=prior_free,
            prior_edge_margin_cells=self.cfg.prior_edge_margin_cells,
            robot_xy=robot_xy,
        )

    def mark_fire_hit(
        self,
        x: float,
        y: float,
        now: float,
        *,
        prior_free: Optional[np.ndarray] = None,
        robot_xy: Optional[Point] = None,
    ) -> List[GridIndex]:
        # Fire keep-outs may sit near trail edges; allow margin 0 for FIRE.
        return self.mark_world(
            x, y, CellSource.FIRE, now,
            prior_free=prior_free,
            prior_edge_margin_cells=0,
            robot_xy=robot_xy,
        )

    def mark_lidar_scan(
        self,
        ranges: Sequence[float],
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
        origin_xy: Point,
        origin_yaw: float,
        now: float,
        *,
        prior_free: Optional[np.ndarray] = None,
        robot_xy: Optional[Point] = None,
    ) -> int:
        """Mark finite returns in [obstacle_min, obstacle_max]. Returns #cells touched."""
        ox, oy = origin_xy
        n_touched = 0
        angle = angle_min
        obst_min = max(range_min, self.cfg.obstacle_min_range_m)
        obst_max = min(range_max, self.cfg.obstacle_max_range_m)
        # Prefer explicit robot pose; fall back to scan origin (base_scan ≈ body).
        body_xy = robot_xy if robot_xy is not None else origin_xy
        for r in ranges:
            if math.isfinite(r) and obst_min <= r <= obst_max:
                wx = ox + r * math.cos(origin_yaw + angle)
                wy = oy + r * math.sin(origin_yaw + angle)
                n_touched += len(self.mark_lidar_hit(
                    wx, wy, now, prior_free=prior_free, robot_xy=body_xy))
            angle += angle_increment
        return n_touched

    # -- lidar clear -------------------------------------------------------

    def _beam_range_toward(
        self,
        ranges: Sequence[float],
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
        origin_xy: Point,
        origin_yaw: float,
        cell_xy: Point,
    ) -> Tuple[Optional[float], float, float]:
        """Return (measured_range_or_None, bearing_in_scan, dist_to_cell)."""
        ox, oy = origin_xy
        dx = cell_xy[0] - ox
        dy = cell_xy[1] - oy
        dist = math.hypot(dx, dy)
        bearing_world = math.atan2(dy, dx)
        bearing_scan = _angle_diff(bearing_world, origin_yaw)
        if not ranges or angle_increment == 0.0:
            return None, bearing_scan, dist
        # Nearest beam index.
        idx = int(round((bearing_scan - angle_min) / angle_increment))
        if idx < 0 or idx >= len(ranges):
            return None, bearing_scan, dist
        r = float(ranges[idx])
        if not math.isfinite(r) or r < range_min or r > range_max:
            # Free / beyond: treat as ray to clear_max / raytrace_max.
            return None, bearing_scan, dist
        return r, bearing_scan, dist

    def _in_scan_fov(
        self,
        bearing_scan: float,
        angle_min: float,
        angle_max: float,
        dist: float,
    ) -> bool:
        if dist > self.cfg.clear_max_range_m:
            return False
        # Normalise bearing into typical scan span.
        b = bearing_scan
        if b < angle_min - 1e-3 or b > angle_max + 1e-3:
            return False
        return True

    def _hit_is_prior_wall(
        self,
        origin_xy: Point,
        origin_yaw: float,
        bearing_scan: float,
        r_meas: float,
        prior_free: Optional[np.ndarray],
        *,
        margin_m: float = 0.35,
    ) -> bool:
        """True if the measured hit lands on/near static prior occupied.

        Used when a beam reports HIT_AT_CELL on a prior-free memory cell: the
        return is the trail wall beside the dynamic mark, not the obstacle
        (Entry 035 T5 halo — cells at ~1.3 m from barrier never cleared).
        """
        if prior_free is None or r_meas is None or not math.isfinite(r_meas):
            return False
        ox, oy = origin_xy
        ang = origin_yaw + bearing_scan
        hx = ox + float(r_meas) * math.cos(ang)
        hy = oy + float(r_meas) * math.sin(ang)
        hit = self.meta.world_to_grid(hx, hy)
        if hit is None:
            return False
        h, w = prior_free.shape
        r_cells = max(1, int(math.ceil(margin_m / max(1e-6, self.meta.resolution))))
        hx_i, hy_i = hit
        for dy in range(-r_cells, r_cells + 1):
            for dx in range(-r_cells, r_cells + 1):
                nx, ny = hx_i + dx, hy_i + dy
                if 0 <= nx < w and 0 <= ny < h and not bool(prior_free[ny, nx]):
                    return True
        return False

    def _cell_is_prior_free(
        self, ix: int, iy: int, prior_free: Optional[np.ndarray]
    ) -> bool:
        if prior_free is None:
            return False
        h, w = prior_free.shape
        return 0 <= iy < h and 0 <= ix < w and bool(prior_free[iy, ix])

    def try_clear_lidar_cell(
        self,
        ix: int,
        iy: int,
        *,
        ranges: Sequence[float],
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
        origin_xy: Point,
        origin_yaw: float,
        scan_age_s: float,
        tf_ok: bool,
        prior_free: Optional[np.ndarray] = None,
    ) -> ClearResult:
        cell = self.cells.get((ix, iy))
        if cell is None:
            return ClearResult.NOT_MARKED
        if cell.source != CellSource.LIDAR:
            return ClearResult.WRONG_SOURCE
        if not tf_ok or scan_age_s > self.cfg.max_scan_age_s:
            return ClearResult.STALE

        cell_xy = self.meta.grid_to_world(ix, iy)
        angle_max = angle_min + angle_increment * max(0, len(ranges) - 1)
        r_meas, bearing, dist = self._beam_range_toward(
            ranges, angle_min, angle_increment, range_min, range_max,
            origin_xy, origin_yaw, cell_xy,
        )
        if not self._in_scan_fov(bearing, angle_min, angle_max, dist):
            return ClearResult.OUT_OF_FOV

        eps = self.cfg.range_epsilon_m
        clear_horizon = min(self.cfg.clear_max_range_m, self.cfg.raytrace_max_m)

        if r_meas is not None:
            # Occlusion: nearer opaque hit — cannot clear behind it.
            if r_meas < dist - eps:
                return ClearResult.REJECTED
            # Hit still at C: either the dynamic obstacle remains, or the beam
            # latched onto an adjacent static wall (bush_trail knife-edge).
            if r_meas <= dist + eps:
                if (
                    self._cell_is_prior_free(ix, iy, prior_free)
                    and self._hit_is_prior_wall(
                        origin_xy, origin_yaw, bearing, r_meas, prior_free)
                ):
                    del self.cells[(ix, iy)]
                    return ClearResult.ACCEPTED
                # Also try neighbouring beams (±1..2): primary index may sit on
                # the wall while a free beam still passes through C.
                if ranges and angle_increment != 0.0:
                    idx0 = int(round((bearing - angle_min) / angle_increment))
                    for d_idx in (-2, -1, 1, 2):
                        j = idx0 + d_idx
                        if j < 0 or j >= len(ranges):
                            continue
                        rj = float(ranges[j])
                        if not math.isfinite(rj) or rj < range_min:
                            # Free / invalid neighbour beam through C's bearing
                            # neighbourhood — treat as free-through when C is
                            # prior-free (Entry 035 halo).
                            if self._cell_is_prior_free(ix, iy, prior_free):
                                del self.cells[(ix, iy)]
                                return ClearResult.ACCEPTED
                            continue
                        if rj > range_max:
                            continue
                        if rj > dist + eps:
                            del self.cells[(ix, iy)]
                            return ClearResult.ACCEPTED
                return ClearResult.REJECTED
            # Hit behind C: free-space ray passes through C.
            del self.cells[(ix, iy)]
            return ClearResult.ACCEPTED

        # No finite return on beam: free ray out to horizon through C.
        if dist <= clear_horizon + eps:
            del self.cells[(ix, iy)]
            return ClearResult.ACCEPTED
        return ClearResult.OUT_OF_FOV

    def clear_lidar_with_scan(
        self,
        *,
        ranges: Sequence[float],
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
        origin_xy: Point,
        origin_yaw: float,
        scan_age_s: float,
        tf_ok: bool,
        prior_free: Optional[np.ndarray] = None,
    ) -> Tuple[int, int]:
        """Attempt clear on all LIDAR cells. Returns (accepted, rejected)."""
        accepted = 0
        rejected = 0
        for ix, iy in list(self.cells.keys()):
            if self.cells[(ix, iy)].source != CellSource.LIDAR:
                continue
            result = self.try_clear_lidar_cell(
                ix, iy,
                ranges=ranges,
                angle_min=angle_min,
                angle_increment=angle_increment,
                range_min=range_min,
                range_max=range_max,
                origin_xy=origin_xy,
                origin_yaw=origin_yaw,
                scan_age_s=scan_age_s,
                tf_ok=tf_ok,
                prior_free=prior_free,
            )
            if result == ClearResult.ACCEPTED:
                accepted += 1
            elif result == ClearResult.REJECTED:
                rejected += 1
        return accepted, rejected

    # -- fire clear --------------------------------------------------------

    def thermal_roi_half_angles(self) -> Tuple[float, float]:
        """Half H (ROI-shrunk) and half V (full VFOV) for clear gate (Entry 027).

        Horizontal uses detector ``roi_width_frac`` so clear ⊆ detect FOV.
        Vertical uses full VFOV/2 (not ROI height) so level cameras can still
        see ground cells at mid-range without falsely rejecting clears.
        """
        roi_w = max(0.01, min(1.0, float(self.cfg.thermal_roi_width_frac)))
        half_h = 0.5 * float(self.cfg.thermal_hfov_rad) * roi_w
        half_v = 0.5 * float(self.cfg.thermal_vfov_rad)
        return half_h, half_v

    def _cell_in_thermal_frustum(
        self,
        ix: int,
        iy: int,
        origin_xy: Point,
        origin_yaw: float,
        camera_z: Optional[float] = None,
    ) -> bool:
        """True if cell centre projects into the thermal clear frustum.

        ``origin_xy`` / ``origin_yaw`` are the thermal camera pose in map
        (``*_camera_link``). Ground cell assumed at z=0. LiDAR FOV is irrelevant.
        """
        cx, cy = self.meta.grid_to_world(ix, iy)
        dx = cx - origin_xy[0]
        dy = cy - origin_xy[1]
        cos_y = math.cos(origin_yaw)
        sin_y = math.sin(origin_yaw)
        # Camera optical: +X forward, +Y left (ROS base-aligned camera_link).
        x_cam = cos_y * dx + sin_y * dy
        y_cam = -sin_y * dx + cos_y * dy
        cam_z = (
            float(camera_z)
            if camera_z is not None
            else float(self.cfg.thermal_camera_height_m)
        )
        z_cam = 0.0 - cam_z  # ground point relative to camera

        if x_cam < float(self.cfg.thermal_near_m):
            return False
        dist = math.hypot(x_cam, y_cam)
        if dist > float(self.cfg.thermal_max_range_m):
            return False

        half_h, half_v = self.thermal_roi_half_angles()
        az = math.atan2(y_cam, x_cam)
        if abs(az) > half_h:
            return False
        # Elevation vs forward range (same convention as pinhole FOV).
        el = math.atan2(z_cam, x_cam)
        if abs(el) > half_v:
            return False
        return True

    # Back-compat alias for older call sites / docs.
    def _cell_in_thermal_cone(
        self,
        ix: int,
        iy: int,
        origin_xy: Point,
        origin_yaw: float,
        camera_z: Optional[float] = None,
    ) -> bool:
        return self._cell_in_thermal_frustum(
            ix, iy, origin_xy, origin_yaw, camera_z=camera_z)

    def _fire_los_occluded(
        self,
        ix: int,
        iy: int,
        *,
        ranges: Sequence[float],
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
        scan_origin_xy: Point,
        scan_origin_yaw: float,
    ) -> bool:
        """True if a nearer LiDAR return sits between the scanner and the cell.

        Camera LOS is proxied by the 360° scan: a hit closer than the FIRE
        cell means the camera cannot see that ground cell as clear.
        """
        cell_xy = self.meta.grid_to_world(ix, iy)
        r_meas, _bearing, dist = self._beam_range_toward(
            ranges, angle_min, angle_increment, range_min, range_max,
            scan_origin_xy, scan_origin_yaw, cell_xy,
        )
        if r_meas is None:
            return False
        margin = max(self.cfg.range_epsilon_m, self.cfg.thermal_occlusion_margin_m)
        return r_meas < dist - margin

    def try_clear_fire_cell(
        self,
        ix: int,
        iy: int,
        *,
        fire_detected: bool,
        hold_active: bool,
        origin_xy: Point,
        origin_yaw: float,
        observation_fresh: bool,
        camera_z: Optional[float] = None,
        optical_forward_m: float = 0.0,
        optical_up_m: float = 0.0,
        ranges: Optional[Sequence[float]] = None,
        angle_min: float = 0.0,
        angle_increment: float = 0.0,
        range_min: float = 0.0,
        range_max: float = 100.0,
        scan_origin_xy: Optional[Point] = None,
        scan_origin_yaw: float = 0.0,
    ) -> ClearResult:
        cell = self.cells.get((ix, iy))
        if cell is None:
            return ClearResult.NOT_MARKED
        if cell.source != CellSource.FIRE:
            return ClearResult.WRONG_SOURCE
        if not observation_fresh:
            # Stale / missing camera evidence must not accumulate toward clear.
            cell.fire_clear_confirm_count = 0
            return ClearResult.STALE
        ox = origin_xy[0] + float(optical_forward_m) * math.cos(origin_yaw)
        oy = origin_xy[1] + float(optical_forward_m) * math.sin(origin_yaw)
        oz = (
            float(camera_z)
            if camera_z is not None
            else float(self.cfg.thermal_camera_height_m)
        ) + float(optical_up_m)
        if not self._cell_in_thermal_frustum(
            ix, iy, (ox, oy), origin_yaw, camera_z=oz
        ):
            cell.fire_clear_confirm_count = 0
            return ClearResult.OUT_OF_FOV
        if (
            ranges is not None
            and scan_origin_xy is not None
            and self._fire_los_occluded(
                ix, iy,
                ranges=ranges,
                angle_min=angle_min,
                angle_increment=angle_increment,
                range_min=range_min,
                range_max=range_max,
                scan_origin_xy=scan_origin_xy,
                scan_origin_yaw=scan_origin_yaw,
            )
        ):
            cell.fire_clear_confirm_count = 0
            return ClearResult.OCCLUDED
        if fire_detected or hold_active:
            cell.fire_clear_confirm_count = 0
            return ClearResult.HOLD_ACTIVE
        # In-FOV, unoccluded negative.
        cell.fire_clear_confirm_count += 1
        if cell.fire_clear_confirm_count < self.cfg.fire_clear_confirm_scans:
            return ClearResult.NEED_MORE_NEGATIVES
        del self.cells[(ix, iy)]
        return ClearResult.ACCEPTED

    def clear_fire_with_observation_detailed(
        self,
        *,
        fire_detected: bool,
        hold_active: bool,
        origin_xy: Point,
        origin_yaw: float,
        observation_fresh: bool,
        camera_z: Optional[float] = None,
        optical_forward_m: float = 0.0,
        optical_up_m: float = 0.0,
        ranges: Optional[Sequence[float]] = None,
        angle_min: float = 0.0,
        angle_increment: float = 0.0,
        range_min: float = 0.0,
        range_max: float = 100.0,
        scan_origin_xy: Optional[Point] = None,
        scan_origin_yaw: float = 0.0,
    ) -> Dict[ClearResult, int]:
        """Run the FIRE clear pass; return per-result counts (Entry 028 events)."""
        counts: Dict[ClearResult, int] = {}
        extra = dict(
            optical_forward_m=optical_forward_m,
            optical_up_m=optical_up_m,
            ranges=ranges,
            angle_min=angle_min,
            angle_increment=angle_increment,
            range_min=range_min,
            range_max=range_max,
            scan_origin_xy=scan_origin_xy,
            scan_origin_yaw=scan_origin_yaw,
        )
        for ix, iy in list(self.cells.keys()):
            if self.cells[(ix, iy)].source != CellSource.FIRE:
                continue
            result = self.try_clear_fire_cell(
                ix, iy,
                fire_detected=fire_detected,
                hold_active=hold_active,
                origin_xy=origin_xy,
                origin_yaw=origin_yaw,
                observation_fresh=observation_fresh,
                camera_z=camera_z,
                **extra,
            )
            counts[result] = counts.get(result, 0) + 1
        return counts

    def clear_fire_with_observation(
        self,
        *,
        fire_detected: bool,
        hold_active: bool,
        origin_xy: Point,
        origin_yaw: float,
        observation_fresh: bool,
        camera_z: Optional[float] = None,
        optical_forward_m: float = 0.0,
        optical_up_m: float = 0.0,
        ranges: Optional[Sequence[float]] = None,
        angle_min: float = 0.0,
        angle_increment: float = 0.0,
        range_min: float = 0.0,
        range_max: float = 100.0,
        scan_origin_xy: Optional[Point] = None,
        scan_origin_yaw: float = 0.0,
    ) -> int:
        counts = self.clear_fire_with_observation_detailed(
            fire_detected=fire_detected,
            hold_active=hold_active,
            origin_xy=origin_xy,
            origin_yaw=origin_yaw,
            observation_fresh=observation_fresh,
            camera_z=camera_z,
            optical_forward_m=optical_forward_m,
            optical_up_m=optical_up_m,
            ranges=ranges,
            angle_min=angle_min,
            angle_increment=angle_increment,
            range_min=range_min,
            range_max=range_max,
            scan_origin_xy=scan_origin_xy,
            scan_origin_yaw=scan_origin_yaw,
        )
        return counts.get(ClearResult.ACCEPTED, 0)

    # -- mask export -------------------------------------------------------

    def build_occupancy_data(self) -> np.ndarray:
        """Flat int8 OccupancyGrid data: 100 occupied, -1 unknown (never 0-free)."""
        data = np.full(self.meta.width * self.meta.height, -1, dtype=np.int8)
        for (ix, iy) in self.cells:
            data[iy * self.meta.width + ix] = 100
        return data

    def fire_cell_world_points(self) -> List[Point]:
        """Map-frame centres of cells tagged ``CellSource.FIRE`` (Entry 026)."""
        return self.cell_world_points(CellSource.FIRE)

    def lidar_cell_world_points(self) -> List[Point]:
        """Map-frame centres of cells tagged ``CellSource.LIDAR`` (Entry 028 introspection)."""
        return self.cell_world_points(CellSource.LIDAR)

    def cell_world_points(self, source: CellSource) -> List[Point]:
        pts: List[Point] = []
        for (ix, iy), cell in self.cells.items():
            if cell.source == source:
                pts.append(self.meta.grid_to_world(ix, iy))
        return pts

    def cells_near(self, x: float, y: float, radius_m: float,
                   source: Optional[CellSource] = None) -> List[MemoryCell]:
        """Typed cells whose centre lies within ``radius_m`` of (x, y)."""
        out: List[MemoryCell] = []
        for (ix, iy), cell in self.cells.items():
            if source is not None and cell.source != source:
                continue
            cx, cy = self.meta.grid_to_world(ix, iy)
            if math.hypot(cx - x, cy - y) <= radius_m:
                out.append(cell)
        return out

    def path_intersects_memory(
        self,
        path: Sequence[Point],
        sample_stride: int = 2,
    ) -> bool:
        for x, y in path[:: max(1, sample_stride)]:
            idx = self.meta.world_to_grid(x, y)
            if idx is not None and idx in self.cells:
                return True
        return False


def _angle_diff(a: float, b: float) -> float:
    """Signed a - b in (-pi, pi]."""
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d <= -math.pi:
        d += 2.0 * math.pi
    return d


def _is_interior_free(
    prior_free: np.ndarray,
    ix: int,
    iy: int,
    margin: int,
) -> bool:
    """True if (ix,iy) is free and no occupied prior cell within ``margin``."""
    h, w = prior_free.shape
    for dx in range(-margin, margin + 1):
        for dy in range(-margin, margin + 1):
            nx, ny = ix + dx, iy + dy
            if 0 <= nx < w and 0 <= ny < h and not bool(prior_free[ny, nx]):
                return False
    return True
