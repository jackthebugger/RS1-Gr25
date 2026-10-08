#!/usr/bin/env python3
"""Rebuild bush_trail physical geometry so it matches the intended trail network.

Authority for traversable space is terrain_sources_3/_debug/trail_mask.png
(not the old SLAM-saved bush_trail_world.pgm, which was ~98% free and did not
encode trail corridors).

Produces:
  1. Flat trail floors at constant elevation
  2. Steep, lidar-clearing banks (plateau above Husky lidar AGL)
  3. Minimum trail width for Husky footprint + margin
  4. Occupancy PGM/YAML aligned to Gazebo XY (free=trail, occupied=off-trail)
  5. Optional perimeter wall boxes SDF include for hard lidar returns

Usage:
  python3 scripts/rebuild_bush_trail_nav_geometry.py
  python3 scripts/rebuild_bush_trail_nav_geometry.py --no-walls
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import (
    binary_dilation,
    binary_erosion,
    distance_transform_edt,
    gaussian_filter,
    label,
    zoom,
)

WORLD_SIZE_M = 60.0
SIZE = 1024
GAZEBO_RES = 1025
TRAIL_FLOOR_M = 0.05
# Plateau must clear lidar (~0.845 m AGL on trail) with margin.
BANK_TOP_M = 1.60
# Horizontal distance from trail edge to full bank height — steep cliff face.
WALL_RISE_M = 0.28
# Minimum free corridor width for Husky (±0.38 footprint → 0.76) + margin.
MIN_TRAIL_WIDTH_M = 1.80
LIDAR_AGL_M = 0.845
SPAWN_XY = (-18.0, 3.0)
# Default goal: on-trail east of spawn (old (18,0) was off-trail).
GOAL_XY = (16.0, -0.1)
MAP_RES = 0.05


def _pkg_root() -> Path:
    return Path(__file__).resolve().parents[1]


def softstep(t: np.ndarray) -> np.ndarray:
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def ensure_min_width(trail: np.ndarray, min_width_m: float) -> np.ndarray:
    """Dilate trail until p5 inscribed width meets min_width_m.

    Uses a binary distance closing: dilate by half-width then erode slightly so
    corridors thicken without filling the whole map.
    """
    px_m = WORLD_SIZE_M / (trail.shape[0] - 1)
    need_r = max(1, int(math.ceil((min_width_m * 0.5) / px_m)))
    # Dilate enough that a thin 1-px corridor becomes ~min_width wide.
    out = binary_dilation(trail, iterations=need_r)
    # Light erosion keeps edges from growing unbounded into plateaus.
    erode_r = max(0, need_r - max(1, int(math.ceil(0.35 / px_m))))
    if erode_r > 0:
        out = binary_erosion(out, iterations=erode_r) | trail
    # Final guarantee: if still thin, forced dilate only.
    radius = distance_transform_edt(out)
    widths = radius[out] * 2.0 * px_m if out.any() else np.array([0.0])
    if float(np.percentile(widths, 5)) < min_width_m * 0.9:
        out = binary_dilation(trail, iterations=need_r + 1)
    return out


def build_dem(trail: np.ndarray, water: np.ndarray) -> np.ndarray:
    dist_m = distance_transform_edt(~trail).astype(np.float64)
    dist_m *= WORLD_SIZE_M / (trail.shape[0] - 1)
    wall_t = softstep(dist_m / WALL_RISE_M)
    dem = TRAIL_FLOOR_M + (BANK_TOP_M - TRAIL_FLOOR_M) * wall_t
    if water.any():
        dist_w = distance_transform_edt(~water).astype(np.float64)
        dist_w_m = dist_w * (WORLD_SIZE_M / (trail.shape[0] - 1))
        ww = softstep(1.0 - np.clip(dist_w_m / 2.0, 0.0, 1.0))
        dem = dem * (1.0 - 0.98 * ww) + 0.01 * (0.98 * ww)
    dem[trail & ~water] = TRAIL_FLOOR_M
    # Soften only bank faces, never trail.
    steep = (dist_m > 0.08) & (dist_m < WALL_RISE_M + 0.4) & ~water & ~trail
    blurred = gaussian_filter(dem, sigma=0.6)
    dem = np.where(steep, 0.7 * dem + 0.3 * blurred, dem)
    dem[trail & ~water] = TRAIL_FLOOR_M
    return np.clip(dem, 0.0, BANK_TOP_M + 0.05)


def write_gazebo(dem: np.ndarray, trail: np.ndarray, water: np.ndarray, materials: Path, model_sdf: Path) -> float:
    factor = (GAZEBO_RES - 1) / (dem.shape[0] - 1)
    arr = zoom(dem, factor, order=1)
    if arr.shape != (GAZEBO_RES, GAZEBO_RES):
        fixed = np.zeros((GAZEBO_RES, GAZEBO_RES), dtype=np.float64)
        n = min(GAZEBO_RES, arr.shape[0], arr.shape[1])
        fixed[:n, :n] = arr[:n, :n]
        arr = fixed
    trail_g = zoom(trail.astype(np.float64), factor, order=0) > 0.5
    water_g = zoom(water.astype(np.float64), factor, order=0) > 0.5
    arr[trail_g & ~water_g] = TRAIL_FLOOR_M
    pmin, pmax = float(arr.min()), float(arr.max())
    span = pmax - pmin
    pixels = np.clip(np.rint((arr - pmin) / (span + 1e-9) * 255.0), 0, 255).astype(np.uint8)
    trail_grey = int(np.clip(round((TRAIL_FLOOR_M - pmin) / (span + 1e-9) * 255.0), 0, 255))
    pixels[trail_g & ~water_g] = trail_grey
    materials.mkdir(parents=True, exist_ok=True)
    Image.fromarray(pixels, mode='L').save(materials / 'heights.png')
    text = model_sdf.read_text(encoding='utf-8')
    text2, n = re.subn(r'<size>60 60 [0-9.]+</size>', f'<size>60 60 {span:.3f}</size>', text)
    if n:
        model_sdf.write_text(text2, encoding='utf-8')
    return span


def write_occupancy_pgm(trail: np.ndarray, maps_dir: Path, repo_maps: Path) -> dict:
    """PGM: free=trail, occupied=off-trail. Origin at world SW corner of 60 m map."""
    # Map covers exactly ±30 m at MAP_RES.
    n = int(round(WORLD_SIZE_M / MAP_RES))
    # Resample trail mask to occupancy grid (row0 = south in ROS maps).
    # Heightmap row0 = north (+Y). Occupancy row0 = south (−Y).
    trail_n = zoom(trail.astype(np.float64), n / trail.shape[0], order=0) > 0.5
    if trail_n.shape[0] != n:
        fixed = np.zeros((n, n), dtype=bool)
        m = min(n, trail_n.shape[0])
        fixed[:m, :m] = trail_n[:m, :m]
        trail_n = fixed
    # Flip vertically so row 0 is −Y (ROS map convention).
    trail_ros = np.flipud(trail_n)
    # PGM: 254 free, 0 occupied
    pgm = np.where(trail_ros, 254, 0).astype(np.uint8)
    origin = [-WORLD_SIZE_M * 0.5, -WORLD_SIZE_M * 0.5, 0.0]
    meta = {
        'image': 'bush_trail_world.pgm',
        'mode': 'trinary',
        'resolution': MAP_RES,
        'origin': origin,
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
    }
    for dest in (maps_dir, repo_maps):
        dest.mkdir(parents=True, exist_ok=True)
        Image.fromarray(pgm, mode='L').save(dest / 'bush_trail_world.pgm')
        with open(dest / 'bush_trail_world.yaml', 'w', encoding='utf-8') as f:
            yaml.safe_dump(meta, f, default_flow_style=False, sort_keys=False)
    return {
        'origin': origin,
        'resolution': MAP_RES,
        'width': n,
        'height': n,
        'free_frac': float(trail_ros.mean()),
        'occ_frac': float(1.0 - trail_ros.mean()),
    }


def wall_polyline_boxes(trail: np.ndarray, spacing_m: float = 0.6, wall_h: float = 1.5, wall_t: float = 0.25) -> list:
    """Sample OFF-trail ring just outside the corridor (never inside free space)."""
    # First off-trail ring outside the free corridor — posts sit on the bank,
    # not in the driveway (Entry 019a: edge posts blocked the Husky).
    outer = binary_dilation(trail, iterations=2) & ~trail
    if not outer.any():
        outer = binary_dilation(trail, iterations=1) & ~trail
    px_m = WORLD_SIZE_M / (trail.shape[0] - 1)
    half = WORLD_SIZE_M * 0.5
    ys, xs = np.where(outer)
    step = max(1, int(round(spacing_m / px_m)))
    boxes = []
    for i, (r, c) in enumerate(zip(ys[::step], xs[::step])):
        x = -half + c * px_m
        y = half - r * px_m
        if abs(x) > 28.5 or abs(y) > 28.5:
            continue
        boxes.append({
            'name': f'trail_wall_{i}',
            'x': float(x),
            'y': float(y),
            'z': wall_h * 0.5,
            'sx': wall_t,
            'sy': wall_t,
            'sz': wall_h,
        })
    return boxes


def write_walls_sdf(boxes: list, out_path: Path) -> None:
    parts = [
        '<?xml version="1.0"?>',
        "<sdf version='1.8'>",
        "  <model name='bush_trail_nav_walls'>",
        '    <static>true</static>',
    ]
    for b in boxes:
        parts.append(f"    <link name='{b['name']}'>")
        parts.append(f"      <pose>{b['x']:.3f} {b['y']:.3f} {b['z']:.3f} 0 0 0</pose>")
        parts.append('      <collision name="c"><geometry>'
                     f"<box><size>{b['sx']:.3f} {b['sy']:.3f} {b['sz']:.3f}</size></box>"
                     '</geometry></collision>')
        parts.append('      <visual name="v"><geometry>'
                     f"<box><size>{b['sx']:.3f} {b['sy']:.3f} {b['sz']:.3f}</size></box>"
                     '</geometry>'
                     '<material><ambient>0.35 0.25 0.15 1</ambient>'
                     '<diffuse>0.45 0.32 0.18 1</diffuse></material></visual>')
        parts.append('    </link>')
    parts.append('  </model>')
    parts.append('</sdf>')
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text('\n'.join(parts) + '\n', encoding='utf-8')
    # model.config
    cfg = out_path.parent / 'model.config'
    cfg.write_text(
        '<?xml version="1.0"?>\n<model>\n  <name>bush_trail_nav_walls</name>\n'
        '  <version>1.0</version>\n  <sdf version="1.8">model.sdf</sdf>\n'
        '  <description>Lidar-visible trail boundary posts from trail_mask</description>\n'
        '</model>\n',
        encoding='utf-8',
    )


def pick_goal_on_trail(trail: np.ndarray) -> tuple[float, float]:
    """Prefer GOAL_XY if on trail; else nearest trail cell to GOAL_XY."""
    half = WORLD_SIZE_M * 0.5
    size = trail.shape[0]
    # Prefer eastern trail near y≈0 in spawn component
    lab, _ = label(trail)
    r0 = int(round((half - SPAWN_XY[1]) / WORLD_SIZE_M * (size - 1)))
    c0 = int(round((SPAWN_XY[0] + half) / WORLD_SIZE_M * (size - 1)))
    if not (0 <= r0 < size and 0 <= c0 < size and trail[r0, c0]):
        return GOAL_XY
    comp = lab == lab[r0, c0]
    best = None
    for r in range(size):
        for c in np.where(comp[r])[0]:
            x = -half + c / (size - 1) * WORLD_SIZE_M
            y = half - r / (size - 1) * WORLD_SIZE_M
            d = (x - GOAL_XY[0]) ** 2 + (y - GOAL_XY[1]) ** 2
            if best is None or d < best[0]:
                best = (d, x, y)
    return (best[1], best[2]) if best else GOAL_XY


def patch_world_include(world_sdf: Path, enable_walls: bool) -> None:
    text = world_sdf.read_text(encoding='utf-8')
    include = (
        '\n    <!-- Lidar-visible trail boundary posts (from trail_mask). -->\n'
        '    <include>\n'
        '      <uri>model://bush_trail_nav_walls</uri>\n'
        '      <name>bush_trail_nav_walls</name>\n'
        '      <pose>0 0 0 0 0 0</pose>\n'
        '    </include>\n'
    )
    if 'bush_trail_nav_walls' in text:
        if not enable_walls:
            text = re.sub(
                r'\s*<!-- Lidar-visible trail boundary posts.*?</include>\n',
                '\n',
                text,
                flags=re.S,
            )
            world_sdf.write_text(text, encoding='utf-8')
        return
    if enable_walls:
        # Insert before closing </world>
        text = text.replace('  </world>\n', include + '  </world>\n')
        world_sdf.write_text(text, encoding='utf-8')


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--no-walls', action='store_true', help='Skip discrete wall boxes')
    ap.add_argument('--min-width', type=float, default=MIN_TRAIL_WIDTH_M)
    args = ap.parse_args()

    root = _pkg_root()
    dbg = root / 'models' / 'terrain_sources_3' / '_debug'
    trail_path = dbg / 'trail_mask.png'
    water_path = dbg / 'water_mask.png'
    if not trail_path.is_file():
        raise SystemExit(f'Missing {trail_path}')

    trail0 = np.asarray(Image.open(trail_path).convert('L')) > 127
    water = (
        np.asarray(Image.open(water_path).convert('L')) > 127
        if water_path.is_file()
        else np.zeros_like(trail0)
    )
    trail = ensure_min_width(trail0, args.min_width)
    # Keep water from being marked trail for floor altitude conflicts
    trail = trail & ~water

    dem = build_dem(trail, water)
    np.save(dbg / 'dem_metres.npy', dem)
    Image.fromarray(
        np.clip(np.rint((dem - dem.min()) / (dem.max() - dem.min() + 1e-9) * 255), 0, 255).astype(np.uint8),
        mode='L',
    ).save(root / 'models' / 'terrain_sources_3' / '01_heightmap_greyscale.png')
    # Persist widened mask for debugging / future runs
    Image.fromarray((trail.astype(np.uint8) * 255), mode='L').save(dbg / 'trail_mask_nav.png')

    span = write_gazebo(
        dem, trail, water,
        root / 'models' / 'bush_trail_ground' / 'materials',
        root / 'models' / 'bush_trail_ground' / 'model.sdf',
    )

    maps_pkg = root / 'maps'
    maps_repo = root.parents[2] / 'maps'  # RS1-Gr25/maps
    if not maps_repo.is_dir():
        maps_repo = root.parents[3] / 'maps' if (root.parents[3] / 'maps').is_dir() else maps_pkg
    # Resolve workspace maps: .../RS1-Gr25/maps
    ws_maps = Path('/home/jordan/git/RS1-Gr25/maps')
    occ = write_occupancy_pgm(trail, maps_pkg, ws_maps)

    goal = pick_goal_on_trail(trail)
    enable_walls = not args.no_walls
    boxes = []
    if enable_walls:
        boxes = wall_polyline_boxes(trail, spacing_m=2.0, wall_h=1.50, wall_t=0.40)
        write_walls_sdf(boxes, root / 'models' / 'bush_trail_nav_walls' / 'model.sdf')
    patch_world_include(root / 'worlds' / 'bush_trail_world.sdf', enable_walls)

    # Bank delta check
    dist_m = distance_transform_edt(~trail) * (WORLD_SIZE_M / (SIZE - 1))
    plateau = (dist_m >= WALL_RISE_M) & (dist_m < WALL_RISE_M + 2.0) & ~water
    trail_mean = float(dem[trail].mean()) if trail.any() else 0.0
    plateau_mean = float(dem[plateau].mean()) if plateau.any() else 0.0
    delta = plateau_mean - trail_mean
    radius = distance_transform_edt(trail)
    width_m = radius[trail] * 2.0 * (WORLD_SIZE_M / (SIZE - 1)) if trail.any() else np.array([0.0])

    meta = {
        'trail_floor_m': TRAIL_FLOOR_M,
        'bank_top_m': BANK_TOP_M,
        'wall_rise_m': WALL_RISE_M,
        'trail_to_bank_delta_m': delta,
        'lidar_agl_m': LIDAR_AGL_M,
        'wall_clears_lidar': bool(delta >= LIDAR_AGL_M),
        'sdf_size_z_m': span,
        'min_trail_width_target_m': args.min_width,
        'trail_width_p5_m': float(np.percentile(width_m, 5)),
        'trail_width_p50_m': float(np.percentile(width_m, 50)),
        'spawn_xy': list(SPAWN_XY),
        'goal_xy': list(goal),
        'occupancy': occ,
        'nav_wall_boxes': len(boxes),
        'nav_walls_enabled': enable_walls,
    }
    (dbg / 'nav_geometry_meta.json').write_text(json.dumps(meta, indent=2), encoding='utf-8')
    print('rebuild_bush_trail_nav_geometry')
    for k, v in meta.items():
        print(f'  {k}: {v}')
    if delta < LIDAR_AGL_M:
        print('WARNING: banks may not clear lidar plane', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
