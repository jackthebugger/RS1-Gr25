#!/usr/bin/env python3
"""Install terrain_sources_3 DEM into models/bush_trail_ground for Gazebo.

Authority: models/terrain_sources_3/01_heightmap_greyscale.png
  (already DEM-encoded; black=low, white=high, span 0.74 m in HEIGHT_RULE).

Produces:
  models/bush_trail_ground/materials/heights.png  (1025x1025 L)
  solid GUI-safe diffuse/normal textures
  prints spawn Z and slope QA
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import zoom

WORLD_SIZE_XY_M = 60.0
MAX_HEIGHT_M = 0.74
RESOLUTION = 1025  # Gazebo 2^n+1
SPAWN_XY = (-18.0, 3.0)


def _pkg_root() -> Path:
    return Path(__file__).resolve().parents[1]


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
    return float(
        (1 - dr) * ((1 - dc) * field[r0, c0] + dc * field[r0, c1])
        + dr * ((1 - dc) * field[r1, c0] + dc * field[r1, c1])
    )


def main() -> int:
    root = _pkg_root()
    src = root / 'models' / 'terrain_sources_3' / '01_heightmap_greyscale.png'
    materials = root / 'models' / 'bush_trail_ground' / 'materials'
    out = materials / 'heights.png'

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=src)
    parser.add_argument('--output', type=Path, default=out)
    args = parser.parse_args()

    if not args.source.is_file():
        raise SystemExit(f'Missing source DEM: {args.source}')

    im = Image.open(args.source).convert('L')
    arr = np.asarray(im, dtype=np.float64)
    if arr.shape != (RESOLUTION, RESOLUTION):
        # Preserve geography with bilinear upsample 1024 → 1025 (Gazebo requirement).
        factor = (RESOLUTION - 1) / (arr.shape[0] - 1)
        arr = zoom(arr, factor, order=1)
        # zoom may be off-by-one; crop/pad to exact size
        if arr.shape[0] != RESOLUTION:
            out_a = np.zeros((RESOLUTION, RESOLUTION), dtype=np.float64)
            n = min(RESOLUTION, arr.shape[0])
            out_a[:n, :n] = arr[:n, :n]
            if n < RESOLUTION:
                out_a[n:, :] = out_a[n - 1:n, :]
                out_a[:, n:] = out_a[:, n - 1:n]
            arr = out_a

    # Source is already full-range DEM encoding; keep 0..255 as-is.
    pixels = np.clip(np.rint(arr), 0, 255).astype(np.uint8)
    materials.mkdir(parents=True, exist_ok=True)
    Image.fromarray(pixels, mode='L').save(args.output)
    Image.new('RGB', (64, 64), (72, 110, 58)).save(materials / 'grass_diffuse.png')
    Image.new('RGB', (64, 64), (128, 128, 255)).save(materials / 'flat_normal.png')

    field = pixels.astype(np.float64) / 255.0
    spawn_z = sample_bilinear(field, *SPAWN_XY) * MAX_HEIGHT_M
    husky_z = spawn_z + 0.45

    spacing = WORLD_SIZE_XY_M / (RESOLUTION - 1)
    h = field * MAX_HEIGHT_M
    gx = np.gradient(h, spacing, axis=1)
    gy = np.gradient(h, spacing, axis=0)
    slope = np.degrees(np.arctan(np.hypot(gx, gy)))

    print('bush_trail_ground heightmap installed')
    print(f'  source: {args.source}')
    print(f'  output: {args.output}  {pixels.shape} L')
    print(f'  png size.z: {MAX_HEIGHT_M} m')
    print(f'  png size.xy: {WORLD_SIZE_XY_M} m')
    print(f'  spawn_ground_z: {spawn_z:.3f} m')
    print(f'  recommended_husky_z: {husky_z:.2f}')
    print(f'  slope_p95: {np.percentile(slope, 95):.2f}°  max: {slope.max():.2f}°')
    print(
        f'\nLaunch:\n'
        f'  world:=bush_trail_world '
        f'husky_x:={SPAWN_XY[0]} husky_y:={SPAWN_XY[1]} '
        f'husky_z:={husky_z:.2f} husky_yaw:=0.0'
    )
    return 0


if __name__ == '__main__':
    sys.exit(main())
