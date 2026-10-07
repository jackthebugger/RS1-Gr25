#!/usr/bin/env python3
"""Force bush_trail trail floors to a perfectly constant height.

Does NOT modify off-trail / bank terrain. Fixes Gazebo bumps caused by
bilinear 1024→1025 resampling bleeding bank heights into the trail mask.

Updates:
  terrain_sources_3/_debug/dem_metres.npy
  terrain_sources_3/01_heightmap_greyscale.png  (re-encoded from DEM)
  bush_trail_ground/materials/heights.png
  bush_trail_ground/model.sdf size.z (unchanged span unless needed)
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import binary_erosion, zoom

WORLD_SIZE_M = 60.0
SIZE = 1024
GAZEBO_RES = 1025
# Keep existing trail floor level from wall-enhance (do not raise/lower banks).
TRAIL_FLOOR_M = 0.05


def _pkg_root() -> Path:
    return Path(__file__).resolve().parents[1]


def main() -> int:
    root = _pkg_root()
    src3 = root / 'models' / 'terrain_sources_3'
    dbg = src3 / '_debug'
    materials = root / 'models' / 'bush_trail_ground' / 'materials'
    dem_path = dbg / 'dem_metres.npy'
    trail_path = dbg / 'trail_mask.png'

    if not dem_path.is_file() or not trail_path.is_file():
        raise SystemExit('Need dem_metres.npy and trail_mask.png (run enhance first).')

    dem = np.load(dem_path).astype(np.float64)
    trail = np.asarray(Image.open(trail_path).convert('L')) > 127
    if dem.shape != trail.shape:
        raise SystemExit(f'Shape mismatch dem {dem.shape} trail {trail.shape}')

    # Preserve surrounds exactly: only overwrite trail pixels.
    # Slightly erode so we don't flatten bank toes that sit on the mask rim.
    core = binary_erosion(trail, iterations=1)
    if not core.any():
        core = trail

    before = dem[core].copy()
    dem_out = dem.copy()
    dem_out[core] = TRAIL_FLOOR_M
    # Hard-set full trail mask too (user asked completely flat trails).
    dem_out[trail] = TRAIL_FLOOR_M

    # Water may overlap trail — keep water as the lower of the two.
    water_path = dbg / 'water_mask.png'
    if water_path.is_file():
        water = np.asarray(Image.open(water_path).convert('L')) > 127
        # Do not raise water; only ensure trail∩~water is flat.
        dem_out[trail & ~water] = TRAIL_FLOOR_M

    np.save(dem_path, dem_out)

    # Re-encode 01 without changing relative surround heights more than
    # remapping 0..max — use same global min/max as before when possible.
    dmin, dmax = float(dem_out.min()), float(dem_out.max())
    norm = (dem_out - dmin) / (dmax - dmin + 1e-9)
    Image.fromarray(
        np.clip(np.rint(norm * 255.0), 0, 255).astype(np.uint8), mode='L'
    ).save(src3 / '01_heightmap_greyscale.png')

    # Gazebo 1025: zoom DEM, then re-force trail to exact constant metres.
    factor = (GAZEBO_RES - 1) / (SIZE - 1)
    arr = zoom(dem_out, factor, order=1)
    if arr.shape[0] != GAZEBO_RES:
        fixed = np.zeros((GAZEBO_RES, GAZEBO_RES), dtype=np.float64)
        n = min(GAZEBO_RES, arr.shape[0])
        fixed[:n, :n] = arr[:n, :n]
        arr = fixed
    trail_g = zoom(trail.astype(np.float64), factor, order=0) > 0.5
    if water_path.is_file():
        water_g = zoom(water.astype(np.float64), factor, order=0) > 0.5
        arr[trail_g & ~water_g] = TRAIL_FLOOR_M
    else:
        arr[trail_g] = TRAIL_FLOOR_M

    span = float(arr.max() - arr.min())
    norm_g = (arr - arr.min()) / (span + 1e-9)
    pixels = np.clip(np.rint(norm_g * 255.0), 0, 255).astype(np.uint8)
    # After encoding, trail pixels must share one grey value.
    trail_grey = int(round(((TRAIL_FLOOR_M - float(arr.min())) / (span + 1e-9)) * 255.0))
    trail_grey = int(np.clip(trail_grey, 0, 255))
    if water_path.is_file():
        pixels[trail_g & ~water_g] = trail_grey
    else:
        pixels[trail_g] = trail_grey

    materials.mkdir(parents=True, exist_ok=True)
    Image.fromarray(pixels, mode='L').save(materials / 'heights.png')

    model_sdf = root / 'models' / 'bush_trail_ground' / 'model.sdf'
    text = model_sdf.read_text(encoding='utf-8')
    text2, n = re.subn(
        r'<size>60 60 [0-9.]+</size>',
        f'<size>60 60 {span:.3f}</size>',
        text,
    )
    if n:
        model_sdf.write_text(text2, encoding='utf-8')

    # Verify
    h = pixels.astype(np.float64) / 255.0 * span + float(arr.min())
    # decode: png 0 = arr.min, 255 = arr.max → height = min + (p/255)*span
    h = float(arr.min()) + (pixels.astype(np.float64) / 255.0) * span
    if water_path.is_file():
        tstats = h[trail_g & ~water_g]
    else:
        tstats = h[trail_g]
    print('flatten_bush_trail_floors')
    print(f'  trail pixels forced to {TRAIL_FLOOR_M:.3f} m')
    print(f'  before trail std: {before.std():.6f} m')
    print(f'  after 1025 trail std: {tstats.std():.6e} m  unique greys: {len(np.unique(pixels[trail_g]))}')
    print(f'  surrounds unchanged except global PNG rescale; sdf size.z={span:.3f}')
    print(f'  gazebo heights: {materials / "heights.png"}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
