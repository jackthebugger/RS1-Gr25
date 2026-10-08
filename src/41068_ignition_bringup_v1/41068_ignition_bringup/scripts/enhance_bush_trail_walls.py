#!/usr/bin/env python3
"""Deepen trails and raise banks so planar Husky lidar (~0.845 m AGL) sees walls.

Reads terrain_sources_3 trail/water masks + existing DEM, rebuilds elevation so:
  - trail floors stay low, flat, continuous
  - shoulders rise to ~lidar height (delta ≈ 0.90 m) over a short blend
  - off-trail ground sits on that high plateau (more noticeable vs trails)
  - water remains the lowest basins

Updates:
  terrain_sources_3/01_heightmap_greyscale.png (+ dem_metres.npy)
  bush_trail_ground/materials/heights.png (1025 Gazebo install)

Does NOT flatten walls via slope softener — steep trail banks are intentional
for Nav2 costmap sensing.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import (
    binary_dilation,
    distance_transform_edt,
    gaussian_filter,
    zoom,
)

WORLD_SIZE_M = 60.0
SIZE = 1024
GAZEBO_RES = 1025
# Lidar plane ≈ 0.845 m above ground on trail → banks must clear this.
TRAIL_FLOOR_M = 0.05
# Floor→bank delta must exceed Husky lidar AGL (~0.845 m) so scan hits walls.
BANK_TOP_M = 1.05
WALL_RISE_M = 0.70  # horizontal metres to reach full bank height (steep face)
SHOULDER_EXTRA_M = 0.5
WATER_FLOOR_M = 0.01
SPAWN_XY = (-18.0, 3.0)
LIDAR_AGL_M = 0.845


def _pkg_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _m_to_px(m: float, size: int = SIZE) -> float:
    return m / WORLD_SIZE_M * (size - 1)


def softstep(t: np.ndarray) -> np.ndarray:
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def sample_bilinear(field: np.ndarray, x: float, y: float) -> float:
    size = field.shape[0]
    half = WORLD_SIZE_M * 0.5
    col = (x + half) / WORLD_SIZE_M * (size - 1)
    row = (half - y) / WORLD_SIZE_M * (size - 1)
    r0 = int(np.floor(row))
    c0 = int(np.floor(col))
    r1 = min(r0 + 1, size - 1)
    c1 = min(c0 + 1, size - 1)
    r0 = max(0, min(r0, size - 1))
    c0 = max(0, min(c0, size - 1))
    dr = row - r0
    dc = col - c0
    return float(
        (1 - dr) * ((1 - dc) * field[r0, c0] + dc * field[r0, c1])
        + dr * ((1 - dc) * field[r1, c0] + dc * field[r1, c1])
    )


def build_walled_dem(
    trail: np.ndarray,
    water: np.ndarray,
    base_dem: np.ndarray | None,
) -> np.ndarray:
    """Trail canyon + lidar-visible banks from distance to trail."""
    trail = trail.astype(bool)
    water = water.astype(bool)
    # Slightly erode trail so free width stays navigable after wall blend.
    # Dilate then use original as corridor floor.
    dist = distance_transform_edt(~trail).astype(np.float64)  # px
    dist_m = dist * (WORLD_SIZE_M / (trail.shape[0] - 1))

    rise = WALL_RISE_M
    # 0 on trail → 1 at full bank height
    wall_t = softstep(dist_m / rise)
    dem = TRAIL_FLOOR_M + (BANK_TOP_M - TRAIL_FLOOR_M) * wall_t

    # Mild off-trail variation from original DEM (scaled onto bank plateau).
    if base_dem is not None:
        bd = base_dem.astype(np.float64)
        bd = (bd - bd.min()) / (bd.max() - bd.min() + 1e-9)
        # Only modulate off-trail plateau (±0.08 m), never deepen trail floor.
        off = softstep((dist_m - rise) / max(SHOULDER_EXTRA_M, 1e-3))
        dem = dem + off * (bd - 0.5) * 0.16

    # Water basins cut through banks as lowest points with soft shores.
    if water.any():
        dist_w = distance_transform_edt(~water).astype(np.float64)
        dist_w_m = dist_w * (WORLD_SIZE_M / (water.shape[0] - 1))
        ww = softstep(1.0 - np.clip(dist_w_m / 2.0, 0.0, 1.0))
        dem = dem * (1.0 - 0.98 * ww) + WATER_FLOOR_M * (0.98 * ww)

    # Hard-flat trail floors (constant height). Surrounds/banks untouched.
    dem[trail & ~water] = TRAIL_FLOOR_M

    # Spawn pad on trail height at west junction (trail level only).
    dem = flatten_spawn(dem, SPAWN_XY, 3.5, 2.0, TRAIL_FLOOR_M)
    dem[trail & ~water] = TRAIL_FLOOR_M

    # Soften bank faces only — never blur trail pixels.
    steep_zone = (dist_m > 0.15) & (dist_m < rise + SHOULDER_EXTRA_M) & ~water & ~trail
    blurred = gaussian_filter(dem, sigma=0.8)
    dem = np.where(steep_zone, 0.55 * dem + 0.45 * blurred, dem)
    dem[trail & ~water] = TRAIL_FLOOR_M

    dem = np.clip(dem, 0.0, BANK_TOP_M + 0.08)
    return dem.astype(np.float64)


def flatten_spawn(
    dem: np.ndarray,
    spawn_xy: tuple[float, float],
    radius_m: float,
    blend_m: float,
    height_m: float,
) -> np.ndarray:
    size = dem.shape[0]
    half = WORLD_SIZE_M * 0.5
    yy = np.linspace(half, -half, size)
    xx = np.linspace(-half, half, size)
    grid_x, grid_y = np.meshgrid(xx, yy)
    dist = np.hypot(grid_x - spawn_xy[0], grid_y - spawn_xy[1])
    inner, outer = radius_m, radius_m + blend_m
    w = softstep(np.clip((outer - dist) / max(outer - inner, 1e-6), 0.0, 1.0))
    return dem * (1.0 - w) + height_m * w


def dem_to_png(dem: np.ndarray) -> Image.Image:
    dmin, dmax = float(dem.min()), float(dem.max())
    norm = (dem - dmin) / (dmax - dmin + 1e-9)
    pixels = np.clip(np.rint(norm * 255.0), 0, 255).astype(np.uint8)
    return Image.fromarray(pixels, mode='L')


def write_gazebo_heights(dem: np.ndarray, out_png: Path, z_extent: float) -> float:
    """Resample to 1025 and encode full span into PNG; return z_extent used."""
    factor = (GAZEBO_RES - 1) / (dem.shape[0] - 1)
    arr = zoom(dem, factor, order=1)
    if arr.shape[0] != GAZEBO_RES:
        fixed = np.zeros((GAZEBO_RES, GAZEBO_RES), dtype=np.float64)
        n = min(GAZEBO_RES, arr.shape[0])
        fixed[:n, :n] = arr[:n, :n]
        if n < GAZEBO_RES:
            fixed[n:, :] = fixed[n - 1 : n, :]
            fixed[:, n:] = fixed[:, n - 1 : n]
        arr = fixed
    dmin, dmax = float(arr.min()), float(arr.max())
    span = dmax - dmin
    # Encode so PNG 0..255 maps to physical 0..span; SDF size.z must match span.
    norm = (arr - dmin) / (span + 1e-9)
    pixels = np.clip(np.rint(norm * 255.0), 0, 255).astype(np.uint8)
    out_png.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(pixels, mode='L').save(out_png)
    return span


def slope_stats(dem: np.ndarray) -> dict:
    dx = WORLD_SIZE_M / (dem.shape[0] - 1)
    gy, gx = np.gradient(dem, dx, dx)
    slope = np.degrees(np.arctan(np.hypot(gx, gy)))
    trail_stats = {}
    return {
        'p50': float(np.percentile(slope, 50)),
        'p95': float(np.percentile(slope, 95)),
        'max': float(slope.max()),
        'pct_above_15': float((slope > 15.0).mean() * 100.0),
        **trail_stats,
    }


def main() -> int:
    root = _pkg_root()
    src3 = root / 'models' / 'terrain_sources_3'
    dbg = src3 / '_debug'
    materials = root / 'models' / 'bush_trail_ground' / 'materials'

    trail_path = dbg / 'trail_mask.png'
    water_path = dbg / 'water_mask.png'
    dem_npy = dbg / 'dem_metres.npy'

    if not trail_path.is_file():
        raise SystemExit(
            f'Missing {trail_path}. Run generate_terrain_sources_3.py first.'
        )

    trail = np.asarray(Image.open(trail_path).convert('L')) > 127
    water = (
        np.asarray(Image.open(water_path).convert('L')) > 127
        if water_path.is_file()
        else np.zeros_like(trail)
    )
    base = np.load(dem_npy) if dem_npy.is_file() else None

    dem = build_walled_dem(trail, water, base)
    stats = slope_stats(dem)

    # Trail vs full bank plateau (beyond WALL_RISE_M), not mid-ramp average.
    dist_px = distance_transform_edt(~trail)
    dist_m = dist_px * (WORLD_SIZE_M / (SIZE - 1))
    plateau = (dist_m >= WALL_RISE_M) & (dist_m < WALL_RISE_M + 3.0) & ~water
    trail_mean = float(dem[trail].mean()) if trail.any() else 0.0
    plateau_mean = float(dem[plateau].mean()) if plateau.any() else 0.0
    delta = plateau_mean - trail_mean

    # Persist authoritative sources_3 heightmap (physical metres encoded 0..255).
    png = dem_to_png(dem)
    png.save(src3 / '01_heightmap_greyscale.png')
    np.save(dbg / 'dem_metres.npy', dem)

    # Gazebo install: PNG spans full 0..255; SDF size.z must equal physical span.
    span = write_gazebo_heights(dem, materials / 'heights.png', BANK_TOP_M)
    # Re-flatten trails after 1025 resample (bilinear bleed from banks).
    from scipy.ndimage import zoom as _zoom
    h_img = np.asarray(Image.open(materials / 'heights.png'), dtype=np.uint8)
    trail_g = _zoom(trail.astype(np.float64), (GAZEBO_RES - 1) / (SIZE - 1), order=0) > 0.5
    water_g = _zoom(water.astype(np.float64), (GAZEBO_RES - 1) / (SIZE - 1), order=0) > 0.5
    # Decode → force trail metres → re-encode with same span/min.
    # heights.png maps 0..255 → 0..span (write_gazebo_heights uses arr.min()).
    arr_min = float(dem.min())
    phys = arr_min + (h_img.astype(np.float64) / 255.0) * span
    phys[trail_g & ~water_g] = TRAIL_FLOOR_M
    # Recompute encode from phys so surrounds keep absolute metres.
    pmin, pmax = float(phys.min()), float(phys.max())
    span = pmax - pmin
    h_img = np.clip(np.rint((phys - pmin) / (span + 1e-9) * 255.0), 0, 255).astype(np.uint8)
    trail_grey = int(np.clip(round((TRAIL_FLOOR_M - pmin) / (span + 1e-9) * 255.0), 0, 255))
    h_img[trail_g & ~water_g] = trail_grey
    Image.fromarray(h_img, mode='L').save(materials / 'heights.png')
    Image.new('RGB', (64, 64), (72, 110, 58)).save(materials / 'grass_diffuse.png')
    Image.new('RGB', (64, 64), (128, 128, 255)).save(materials / 'flat_normal.png')

    # Update model.sdf size.z to match encoded span.
    model_sdf = root / 'models' / 'bush_trail_ground' / 'model.sdf'
    text = model_sdf.read_text(encoding='utf-8')
    import re

    text2, n = re.subn(
        r'<size>60 60 [0-9.]+</size>',
        f'<size>60 60 {span:.3f}</size>',
        text,
    )
    if n:
        model_sdf.write_text(text2, encoding='utf-8')

    spawn_z = sample_bilinear(dem, *SPAWN_XY)
    husky_z = spawn_z + 0.45

    meta = {
        'trail_mean_m': trail_mean,
        'bank_plateau_mean_m': plateau_mean,
        'trail_to_bank_delta_m': delta,
        'dem_min_m': float(dem.min()),
        'dem_max_m': float(dem.max()),
        'sdf_size_z_m': span,
        'slope': {k: stats[k] for k in ('p50', 'p95', 'max', 'pct_above_15')},
        'spawn_ground_z_m': spawn_z,
        'recommended_husky_z': husky_z,
        'lidar_plane_agl_m': LIDAR_AGL_M,
        'wall_clears_lidar': bool(delta >= LIDAR_AGL_M),
    }
    (dbg / 'wall_enhance_meta.json').write_text(json.dumps(meta, indent=2), encoding='utf-8')

    print('bush trail walls enhanced')
    for k, v in meta.items():
        print(f'  {k}: {v}')
    print(
        f'\nLaunch with:\n'
        f'  world:=bush_trail_world husky_x:=-18 husky_y:=3 '
        f'husky_z:={husky_z:.2f}'
    )
    if delta < LIDAR_AGL_M:
        print(
            f'WARNING: trail→bank delta {delta:.3f} m < lidar AGL {LIDAR_AGL_M} m',
            file=sys.stderr,
        )
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
