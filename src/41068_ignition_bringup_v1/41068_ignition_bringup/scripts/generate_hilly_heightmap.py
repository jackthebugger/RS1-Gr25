#!/usr/bin/env python3
"""Generate Gazebo-ready heightmap PNG for custom_world_hilly.

Authority elevation source:
  models/terrain_sources/01_heightmap_greyscale.png
  (brighter = higher). Images 02/03 are QA / height-rule references only.

Outputs (under models/hilly_ground/materials/):
  heights.png          — 1025x1025 greyscale L, black=low white=high
  grass_diffuse.png    — solid-colour diffuse (GUI-safe)
  flat_normal.png      — flat normal map

Also prints recommended Husky spawn Z for (-18, 3).
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import gaussian_filter


# World / heightmap contract (must match model.sdf + custom_world_hilly.sdf).
WORLD_SIZE_XY_M = 60.0
# Kept modest so planar Nav2 + DiffDrive Husky remain viable on ridges.
MAX_HEIGHT_M = 0.65
RESOLUTION = 1025  # 2^n + 1
SPAWN_XY = (-18.0, 3.0)
SPAWN_PATCH_RADIUS_M = 3.5
SPAWN_PATCH_BLEND_M = 2.0
# Stronger blur removes knife ridges that become lidar "walls" / tip hazards.
GAUSSIAN_SIGMA_PX = 4.5
# Soften the upper percentile so NE ridges stay mild for planar Nav2 / Husky.
HIGH_PERCENTILE = 97.0
LOW_PERCENTILE = 3.0


def _pkg_root() -> Path:
    return Path(__file__).resolve().parents[1]


def load_luminance(path: Path, size: int) -> np.ndarray:
    im = Image.open(path).convert('RGB')
    if im.size != (size, size):
        im = im.resize((size, size), Image.Resampling.LANCZOS)
    arr = np.asarray(im, dtype=np.float64)
    return 0.299 * arr[:, :, 0] + 0.587 * arr[:, :, 1] + 0.114 * arr[:, :, 2]


def world_to_pixel(x: float, y: float, size: int) -> tuple[float, float]:
    """Map Gazebo world XY (origin-centred, +Y north) to image row/col.

    Image top (row 0) = +Y (north). Column 0 = -X (west).
    """
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


def flatten_spawn_patch(field: np.ndarray) -> tuple[np.ndarray, float]:
    """Force a flat lowland disk around SPAWN_XY; return field and spawn height [0,1]."""
    size = field.shape[0]
    sx, sy = SPAWN_XY
    spawn_h = float(sample_bilinear(field, sx, sy))

    yy = np.linspace(WORLD_SIZE_XY_M * 0.5, -WORLD_SIZE_XY_M * 0.5, size)
    xx = np.linspace(-WORLD_SIZE_XY_M * 0.5, WORLD_SIZE_XY_M * 0.5, size)
    grid_x, grid_y = np.meshgrid(xx, yy)
    dist = np.hypot(grid_x - sx, grid_y - sy)

    inner = SPAWN_PATCH_RADIUS_M
    outer = SPAWN_PATCH_RADIUS_M + SPAWN_PATCH_BLEND_M
    weight = np.clip((outer - dist) / (outer - inner), 0.0, 1.0)
    # Cosine smoothstep for a soft rim.
    weight = 0.5 - 0.5 * np.cos(np.pi * weight)

    out = field * (1.0 - weight) + spawn_h * weight
    # Re-sample after flatten (should equal spawn_h).
    spawn_h = float(sample_bilinear(out, sx, sy))
    return out, spawn_h


def normalise_elevation(field: np.ndarray) -> np.ndarray:
    lo = np.percentile(field, LOW_PERCENTILE)
    hi = np.percentile(field, HIGH_PERCENTILE)
    if hi <= lo + 1e-6:
        return np.zeros_like(field)
    norm = np.clip((field - lo) / (hi - lo), 0.0, 1.0)
    # Compress highs so broad plateaus remain but peak slopes soften.
    return np.power(norm, 1.35)


def write_solid_textures(materials_dir: Path) -> None:
    materials_dir.mkdir(parents=True, exist_ok=True)
    # Soft grass green — solid colour, no photo texture (GUI stability).
    green = Image.new('RGB', (64, 64), (72, 110, 58))
    green.save(materials_dir / 'grass_diffuse.png')
    # Flat +Z normal in tangent space (128, 128, 255).
    normal = Image.new('RGB', (64, 64), (128, 128, 255))
    normal.save(materials_dir / 'flat_normal.png')


def generate(source: Path, out_png: Path) -> dict:
    field = load_luminance(source, RESOLUTION)
    field = gaussian_filter(field, sigma=GAUSSIAN_SIGMA_PX)
    field = normalise_elevation(field)
    field, spawn_norm = flatten_spawn_patch(field)

    # Final PNG: 8-bit greyscale L, full 0..255 span → SDF size.z = MAX_HEIGHT_M.
    pixels = np.clip(np.rint(field * 255.0), 0, 255).astype(np.uint8)
    out_png.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(pixels, mode='L').save(out_png)

    spawn_z_m = spawn_norm * MAX_HEIGHT_M
    # Clearance above local ground for DiffDrive settle (matches flat-world ~0.4 habit).
    recommended_husky_z = spawn_z_m + 0.45

    # Slope diagnostic near spawn → east along corridor.
    h0 = sample_bilinear(field, -18.0, 3.0) * MAX_HEIGHT_M
    h1 = sample_bilinear(field, -10.0, 3.0) * MAX_HEIGHT_M
    grade = abs(h1 - h0) / 8.0

    stats = {
        'resolution': RESOLUTION,
        'world_xy_m': WORLD_SIZE_XY_M,
        'max_height_m': MAX_HEIGHT_M,
        'png_min': int(pixels.min()),
        'png_max': int(pixels.max()),
        'png_mean': float(pixels.mean()),
        'spawn_ground_z_m': spawn_z_m,
        'recommended_husky_z': recommended_husky_z,
        'corridor_grade_approx': grade,
        'out_png': str(out_png),
    }
    return stats


def main() -> int:
    root = _pkg_root()
    default_src = root / 'models' / 'terrain_sources' / '01_heightmap_greyscale.png'
    materials = root / 'models' / 'hilly_ground' / 'materials'
    default_out = materials / 'heights.png'

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=default_src)
    parser.add_argument('--output', type=Path, default=default_out)
    args = parser.parse_args()

    if not args.source.is_file():
        raise SystemExit(f'Source heightmap not found: {args.source}')

    write_solid_textures(materials)
    stats = generate(args.source, args.output)

    print('hilly heightmap generated')
    for key, value in stats.items():
        print(f'  {key}: {value}')
    print(
        f"\nLaunch hint:\n"
        f"  world:=custom_world_hilly "
        f"husky_x:={SPAWN_XY[0]} husky_y:={SPAWN_XY[1]} "
        f"husky_z:={stats['recommended_husky_z']:.2f} husky_yaw:=0.0"
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
