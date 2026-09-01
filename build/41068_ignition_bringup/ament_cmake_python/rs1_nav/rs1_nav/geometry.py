"""Small geometry helpers shared by the mission demo and the tests."""

import math
from typing import Iterable, Optional, Sequence, Tuple


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Convert a quaternion into a 2D yaw angle in radians."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    """Convert a 2D yaw angle into an (x, y, z, w) quaternion."""
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def path_length(points: Sequence[Tuple[float, float]]) -> float:
    """Total length of a polyline in metres."""
    return sum(
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(points, points[1:])
    )


def path_closest_approach(
    points: Iterable[Tuple[float, float]],
    target: Tuple[float, float],
) -> Optional[float]:
    """Distance from `target` to the nearest point on a polyline.

    This is the measure used to decide whether a replanned path actually routes
    around an obstacle: before an obstacle appears the path passes essentially
    through it, and after a genuine replan the path must keep its distance.
    Segment-wise (not just vertex-wise) so that sparse paths are measured
    correctly.
    """
    points = list(points)
    if not points:
        return None
    if len(points) == 1:
        return math.hypot(points[0][0] - target[0], points[0][1] - target[1])

    tx, ty = target
    best = float('inf')
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        dx, dy = bx - ax, by - ay
        seg_sq = dx * dx + dy * dy
        if seg_sq <= 1e-12:
            t = 0.0
        else:
            t = ((tx - ax) * dx + (ty - ay) * dy) / seg_sq
            t = max(0.0, min(1.0, t))
        cx, cy = ax + t * dx, ay + t * dy
        best = min(best, math.hypot(tx - cx, ty - cy))
    return best


def path_heading(points: Sequence[Tuple[float, float]], index: int = 0) -> float:
    """Heading of the path at `index`, falling back to the next distinct point."""
    if len(points) < 2:
        return 0.0
    index = max(0, min(index, len(points) - 2))
    for start in range(index, len(points) - 1):
        dx = points[start + 1][0] - points[start][0]
        dy = points[start + 1][1] - points[start][1]
        if dx * dx + dy * dy > 1e-6:
            return math.atan2(dy, dx)
    dx = points[-1][0] - points[0][0]
    dy = points[-1][1] - points[0][1]
    return math.atan2(dy, dx)


def point_ahead_on_path(
    points: Sequence[Tuple[float, float]],
    origin: Tuple[float, float],
    look_ahead: float,
) -> Optional[Tuple[Tuple[float, float], float]]:
    """Point `look_ahead` metres along the path past the closest approach to `origin`.

    Returns ((x, y), heading) or None if the path is empty. Used to drop a
    barrier in front of the robot rather than behind it.
    """
    if not points:
        return None
    if len(points) == 1:
        return (points[0], 0.0)

    ox, oy = origin
    best_dist = float('inf')
    best_seg = 0
    best_t = 0.0
    for i, ((ax, ay), (bx, by)) in enumerate(zip(points, points[1:])):
        dx, dy = bx - ax, by - ay
        seg_sq = dx * dx + dy * dy
        if seg_sq <= 1e-12:
            t = 0.0
        else:
            t = ((ox - ax) * dx + (oy - ay) * dy) / seg_sq
            t = max(0.0, min(1.0, t))
        cx, cy = ax + t * dx, ay + t * dy
        dist = math.hypot(ox - cx, oy - cy)
        if dist < best_dist:
            best_dist = dist
            best_seg = i
            best_t = t

    ax, ay = points[best_seg]
    bx, by = points[best_seg + 1]
    remaining = (1.0 - best_t) * math.hypot(bx - ax, by - ay)
    if remaining >= look_ahead:
        heading = math.atan2(by - ay, bx - ax)
        closest_x = ax + best_t * (bx - ax)
        closest_y = ay + best_t * (by - ay)
        return (
            (closest_x + look_ahead * math.cos(heading),
             closest_y + look_ahead * math.sin(heading)),
            heading,
        )

    walked = remaining
    closest_x = ax + best_t * (bx - ax)
    closest_y = ay + best_t * (by - ay)
    last = (closest_x, closest_y)
    heading = path_heading(points, best_seg)
    for i in range(best_seg + 1, len(points) - 1):
        px, py = points[i]
        qx, qy = points[i + 1]
        seg = math.hypot(qx - px, qy - py)
        if walked + seg >= look_ahead:
            heading = math.atan2(qy - py, qx - px)
            need = look_ahead - walked
            frac = 0.0 if seg <= 1e-9 else need / seg
            return ((px + frac * (qx - px), py + frac * (qy - py)), heading)
        walked += seg
        last = (qx, qy)
        heading = math.atan2(qy - py, qx - px)
    return (last, heading)
