"""Pure helpers for placing thermal FIRE hazards (Entry 028).

Sensor-authority rule: a FIRE keep-out may be *added* only when the thermal
camera currently reports a fire (debounced positive) **and** a LiDAR return
exists along the camera bearing of the hot blob. The LiDAR supplies range
only; without a thermal positive nothing is marked, and without a LiDAR
return along the blob bearing nothing is marked either (no default range —
a guessed range is how the bridge previously sprayed discs behind the robot
while it turned onto Path B).

Frames: the Husky LiDAR sits on the base_link vertical axis (``scan_joint``
xyz 0 0 0.68) while the thermal optical origin is ``camera_joint`` 0.45 m
plus the sensor ``<pose>`` 0.45 m forward of base_link. Range gating is done
from the camera origin so parallax between the two sensors does not pick the
wrong return at short range.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

Point = Tuple[float, float]

# camera_joint (0.45) + Ignition sensor <pose> x (0.45) in husky.gazebo.xacro.
THERMAL_OPTICAL_FORWARD_M = 0.90
THERMAL_OPTICAL_HEIGHT_M = 0.70   # 0.25 + 0.45 above base_link


@dataclass(frozen=True)
class ThermalBearing:
    """Hot-blob bearing from the detector, camera frame (+left), radians."""

    bearing_rad: float
    half_width_rad: float
    stamp_s: float


def hot_blob_bearing(
    hot_cols: Sequence[int],
    image_width: int,
    hfov_rad: float,
) -> Optional[Tuple[float, float]]:
    """(bearing, half_width) of hot pixels by pinhole column geometry.

    ``hot_cols`` are full-image column indices of pixels above threshold.
    Bearing is positive to the left (ROS camera +Y / base +Y) so the sign
    matches ``atan2`` yaw convention in the base frame.
    """
    if not hot_cols or image_width <= 0 or hfov_rad <= 0.0:
        return None
    fx = (image_width / 2.0) / math.tan(hfov_rad / 2.0)
    cx = image_width / 2.0
    cols = sorted(int(c) for c in hot_cols)
    mean_col = sum(cols) / len(cols)
    bearing = -math.atan((mean_col - cx) / fx)
    left = -math.atan((cols[0] - cx) / fx)
    right = -math.atan((cols[-1] - cx) / fx)
    half_width = max(abs(left - bearing), abs(right - bearing))
    return bearing, half_width


def thermal_hazard_point(
    *,
    fire_detected: bool,
    bearing: Optional[ThermalBearing],
    now_s: float,
    scan_ranges: Optional[Sequence[float]],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    max_range_m: float = 8.0,
    min_range_m: float = 1.5,
    bearing_max_age_s: float = 0.5,
    angular_margin_rad: float = 0.06,
    camera_forward_m: float = THERMAL_OPTICAL_FORWARD_M,
    half_width_cap_rad: float = 0.12,
) -> Optional[Point]:
    """Base-frame XY of the thermal hazard, or ``None`` when evidence is missing.

    Returns None when: no debounced thermal positive; bearing missing or
    older than ``bearing_max_age_s``; no LiDAR return; no finite return
    whose base-frame point lies within ``min(half_width, cap) + margin`` of
    the camera bearing as seen from the camera origin; no such return in
    ``[min_range_m, max_range_m]`` from the camera.

    Range is taken from the return closest to the hot-blob centroid bearing
    (not the nearest range in a wide cone — trail banks otherwise steal the
    mark and spray FIRE discs between the robot and the fire).
    """
    if not fire_detected or bearing is None or not scan_ranges:
        return None
    if now_s - bearing.stamp_s > bearing_max_age_s or now_s < bearing.stamp_s - 1e-3:
        return None
    window = min(max(0.0, bearing.half_width_rad), half_width_cap_rad) + angular_margin_rad
    # (angular_error, dist_cam, point)
    best: Optional[Tuple[float, float, Point]] = None
    for i, r in enumerate(scan_ranges):
        if not (range_min < r < range_max) or math.isinf(r) or math.isnan(r):
            continue
        a = angle_min + i * angle_increment
        px = r * math.cos(a)
        py = r * math.sin(a)
        # Angle of this return as seen from the camera origin.
        dx = px - camera_forward_m
        if dx <= 0.0:
            continue  # behind the camera: cannot be what the camera sees
        ang_cam = math.atan2(py, dx)
        ang_err = abs(_wrap(ang_cam - bearing.bearing_rad))
        if ang_err > window:
            continue
        dist_cam = math.hypot(dx, py)
        if not (min_range_m <= dist_cam <= max_range_m):
            continue
        cand = (ang_err, dist_cam, (px, py))
        if best is None:
            best = cand
            continue
        # Prefer on-bearing; only then nearer range (tie within 0.03 rad).
        if cand[0] + 0.03 < best[0] or (
            abs(cand[0] - best[0]) <= 0.03 and cand[1] < best[1]
        ):
            best = cand
    if best is None:
        return None
    return best[2]


def base_to_map(point_base: Point, base_xy: Point, base_yaw: float) -> Point:
    c, s = math.cos(base_yaw), math.sin(base_yaw)
    return (
        base_xy[0] + c * point_base[0] - s * point_base[1],
        base_xy[1] + s * point_base[0] + c * point_base[1],
    )


class HazardDeduper:
    """One thermal disc per ``fire_detected`` True episode.

    While the detector stays True the robot may yaw (Path A→B U-turn). A new
    disc every tick would spray FIRE along the rotating bearing. Latch the
    first accepted location; unlatch only when the detector goes False so a
    later, independent sighting can mark again.

    Spatial window still rejects near-duplicates across a brief False flicker.
    """

    def __init__(self, min_separation_m: float = 1.5, window_s: float = 120.0):
        self.min_sep = min_separation_m
        self.window_s = window_s
        self._recent: List[Tuple[float, Point]] = []
        self._episode_latched = False
        self._last_detected = False

    def note_detected(self, fire_detected: bool) -> None:
        if self._last_detected and not fire_detected:
            self._episode_latched = False
        self._last_detected = bool(fire_detected)

    def accept(self, p: Point, now_s: float) -> bool:
        if self._episode_latched:
            return False
        self._recent = [(t, q) for t, q in self._recent if now_s - t <= self.window_s]
        for _t, q in self._recent:
            if math.hypot(q[0] - p[0], q[1] - p[1]) < self.min_sep:
                return False
        self._recent.append((now_s, p))
        self._episode_latched = True
        return True


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def near_demo_obstacle(
    xy: Point,
    demos: Sequence[Point],
    radius_m: float,
) -> bool:
    """True when ``xy`` is within ``radius_m`` of any GUI demo-obstacle centre."""
    if not demos or radius_m <= 0.0:
        return False
    r2 = float(radius_m) * float(radius_m)
    x, y = xy
    for dx, dy in demos:
        if (x - dx) * (x - dx) + (y - dy) * (y - dy) <= r2:
            return True
    return False


def bearing_aims_demo_obstacle(
    *,
    robot_xy: Point,
    robot_yaw: float,
    bearing_rad: float,
    demos: Sequence[Point],
    max_range_m: float,
    angular_margin_rad: float = 0.20,
) -> bool:
    """True when a thermal bearing points at a known GUI demo obstacle.

    Used to suppress THERMAL_FIRE_PENDING when the hot blob is the Status GUI
    BLOCK FIRE box (LiDAR may still see it as a normal obstacle; path_bank
    already invalidates via costmap/keepout).
    """
    if not demos or max_range_m <= 0.0:
        return False
    rx, ry = robot_xy
    world_bearing = _wrap(robot_yaw + bearing_rad)
    for dx, dy in demos:
        dist = math.hypot(dx - rx, dy - ry)
        if dist < 0.05 or dist > max_range_m:
            continue
        ang = math.atan2(dy - ry, dx - rx)
        if abs(_wrap(ang - world_bearing)) <= angular_margin_rad:
            return True
    return False
