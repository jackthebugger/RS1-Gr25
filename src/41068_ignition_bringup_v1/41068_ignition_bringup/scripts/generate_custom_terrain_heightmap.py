#!/usr/bin/env python3
"""Generate Gazebo heightmap for custom_terrain_world from terrain_sources_2.

Authority: models/terrain_sources_2/01_heightmap_greyscale.png (brighter = higher).
02/03 are visual/QA only.

Design targets (from terrain spec):
  - 60 x 60 m heightmap, 60 x 30 m Nav2 arena (map_perimeter unchanged)
  - ~0.76 m total relief (0.7–0.8 m band)
  - Preserve trail-vs-hill contrast; light blur only (sigma ~1.5 px)
  - West-centre spawn (-18, 3) on reference clearing

Outputs: models/custom_terrain/materials/heights.png (+ solid GUI-safe textures)
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import gaussian_filter

WORLD_SIZE_XY_M = 60.0
MAX_HEIGHT_M = 0.76
RESOLUTION = 1025
SPAWN_XY = (-18.0, 3.0)
GAUSSIAN_SIGMA_PX = 1.5
LOW_PERCENTILE = 2.0
HIGH_PERCENTILE = 98.0
GAMMA = 1.05

# Props aligned with custom_world_1 (XY); Z filled from generated height field.
WALL_XY_YAW = [
    (-4.0, 10.0, 1.570796, 'forest_wall_1'),
    (-4.0, -5.0, 1.570796, 'forest_wall_2'),
    (-4.0, -10.0, 1.570796, 'forest_wall_3'),
    (8.0, 13.0, 1.570796, 'forest_wall_4'),
    (8.0, -2.0, 1.570796, 'forest_wall_5'),
    (8.0, -17.0, 1.570796, 'forest_wall_6'),
]
OAK_XY = (3.54698, 5.0, 0.1)


def _pkg_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _resize(im: Image.Image, size: int) -> Image.Image:
    try:
        resample = Image.Resampling.LANCZOS
    except AttributeError:
        resample = Image.LANCZOS
    if im.size != (size, size):
        im = im.resize((size, size), resample)
    return im


def load_luminance(path: Path, size: int) -> np.ndarray:
    im = _resize(Image.open(path).convert('RGB'), size)
    arr = np.asarray(im, dtype=np.float64)
    return 0.299 * arr[:, :, 0] + 0.587 * arr[:, :, 1] + 0.114 * arr[:, :, 2]


def world_to_pixel(x: float, y: float, size: int) -> tuple[float, float]:
    half = WORLD_SIZE_XY_M * 0.5
    col = (x + half) / WORLD_SIZE_XY_M * (size - 1)
    row = (half - y) / WORLD_SIZE_XY_M * (size - 1)
    return row, col


def sample_bilinear(field: np.ndarray, x: float, y: float) -> float:
    size = field.shape[0]
    row, col = world_to_pixel(x, y, size)
    r0 = int(np.floor(row))
    c0 = int(np.floor(col))
    r1 = min(r0 + 1, size - 1)
    c1 = min(c0 + 1, size - 1)
    r0 = max(0, min(r0, size - 1))
    c0 = max(0, min(c0, size - 1))
    dr = row - r0
    dc = col - c0
    v00 = field[r0, c0]
    v01 = field[r0, c1]
    v10 = field[r1, c0]
    v11 = field[r1, c1]
    return (1 - dr) * ((1 - dc) * v00 + dc * v01) + dr * ((1 - dc) * v10 + dc * v11)


def normalise_elevation(field: np.ndarray) -> np.ndarray:
    lo = np.percentile(field, LOW_PERCENTILE)
    hi = np.percentile(field, HIGH_PERCENTILE)
    if hi <= lo + 1e-6:
        return np.zeros_like(field)
    norm = np.clip((field - lo) / (hi - lo), 0.0, 1.0)
    return np.power(norm, GAMMA)


def slope_stats(field_norm: np.ndarray) -> dict:
    h = field_norm * MAX_HEIGHT_M
    spacing = WORLD_SIZE_XY_M / (field_norm.shape[0] - 1)
    gx = np.gradient(h, spacing, axis=1)
    gy = np.gradient(h, spacing, axis=0)
    slope = np.degrees(np.arctan(np.hypot(gx, gy)))
    yy = np.linspace(WORLD_SIZE_XY_M * 0.5, -WORLD_SIZE_XY_M * 0.5, field_norm.shape[0])
    corridor = (yy >= -4) & (yy <= 4)
    sx, sy = SPAWN_XY
    half = WORLD_SIZE_XY_M * 0.5
    spawn_mask = (
        (np.abs(np.linspace(-half, half, field_norm.shape[1])[None, :] - sx) <= 4.0)
        & (np.abs(yy[:, None] - sy) <= 4.0)
    )
    return {
        'slope_mean_deg': float(slope.mean()),
        'slope_p95_deg': float(np.percentile(slope, 95)),
        'slope_max_deg': float(slope.max()),
        'corridor_p95_deg': float(np.percentile(slope[corridor], 95)),
        'spawn_patch_p95_deg': float(np.percentile(slope[spawn_mask], 95)),
        'elevation_min_m': float(h.min()),
        'elevation_max_m': float(h.max()),
        'elevation_range_m': float(h.max() - h.min()),
    }


def write_solid_textures(materials_dir: Path) -> None:
    materials_dir.mkdir(parents=True, exist_ok=True)
    Image.new('RGB', (64, 64), (72, 110, 58)).save(materials_dir / 'grass_diffuse.png')
    Image.new('RGB', (64, 64), (128, 128, 255)).save(materials_dir / 'flat_normal.png')


def generate(source: Path, out_png: Path) -> dict:
    raw = load_luminance(source, RESOLUTION)
    field = gaussian_filter(raw, sigma=GAUSSIAN_SIGMA_PX)
    field = normalise_elevation(field)

    pixels = np.clip(np.rint(field * 255.0), 0, 255).astype(np.uint8)
    out_png.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(pixels, mode='L').save(out_png)

    spawn_norm = float(sample_bilinear(field, *SPAWN_XY))
    spawn_z_m = spawn_norm * MAX_HEIGHT_M
    recommended_husky_z = spawn_z_m + 0.45

    stats = {
        'source': str(source),
        'resolution': RESOLUTION,
        'world_xy_m': WORLD_SIZE_XY_M,
        'max_height_m': MAX_HEIGHT_M,
        'png_min': int(pixels.min()),
        'png_max': int(pixels.max()),
        'spawn_xy': SPAWN_XY,
        'spawn_ground_z_m': spawn_z_m,
        'recommended_husky_z': recommended_husky_z,
        'out_png': str(out_png),
    }
    stats.update(slope_stats(field))

    props = {}
    for x, y, yaw, name in WALL_XY_YAW:
        z = float(sample_bilinear(field, x, y) * MAX_HEIGHT_M)
        props[name] = {'x': x, 'y': y, 'z': round(z, 3), 'yaw': yaw}
    ox, oy, oyaw = OAK_XY
    props['oak8'] = {
        'x': ox, 'y': oy, 'z': round(float(sample_bilinear(field, ox, oy) * MAX_HEIGHT_M), 3),
        'yaw': oyaw,
    }
    stats['prop_poses_z'] = props
    return stats


def main() -> int:
    root = _pkg_root()
    default_src = root / 'models' / 'terrain_sources_2' / '01_heightmap_greyscale.png'
    materials = root / 'models' / 'custom_terrain' / 'materials'
    default_out = materials / 'heights.png'

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=default_src)
    parser.add_argument('--output', type=Path, default=default_out)
    parser.add_argument('--write-stats', type=Path, default=None,
                        help='Optional JSON stats path')
    args = parser.parse_args()

    if not args.source.is_file():
        raise SystemExit(f'Source heightmap not found: {args.source}')

    write_solid_textures(materials)
    stats = generate(args.source, args.output)

    print('custom_terrain heightmap generated')
    for key, value in stats.items():
        if key != 'prop_poses_z':
            print(f'  {key}: {value}')
    print('  prop_poses_z:')
    for name, pose in stats['prop_poses_z'].items():
        print(f'    {name}: z={pose["z"]}')

    if args.write_stats:
        args.write_stats.parent.mkdir(parents=True, exist_ok=True)
        args.write_stats.write_text(json.dumps(stats, indent=2), encoding='utf-8')

    print(
        f"\nLaunch hint:\n"
        f"  world:=custom_terrain_world "
        f"husky_x:={SPAWN_XY[0]} husky_y:={SPAWN_XY[1]} "
        f"husky_z:={stats['recommended_husky_z']:.2f} husky_yaw:=0.0"
    )
    return 0


if __name__ == '__main__':
    sys.exit(main())
