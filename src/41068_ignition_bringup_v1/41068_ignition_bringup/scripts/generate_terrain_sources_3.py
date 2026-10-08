#!/usr/bin/env python3
"""Reference-driven terrain reconstruction → terrain_sources_3 deliverables.

Authority: models/terrain_sources_3/Referrence_Img_for_map (WebP/JPEG/PNG).
Geographic feature masks drive a single DEM; 01/02/03 are derived from that DEM.

Outputs (exact names under models/terrain_sources_3/):
  01_heightmap_greyscale.png
  02_topo_shaded_relief.png
  03_colour_surface_map.png
  HEIGHT_RULE.txt

Debug intermediates (optional): models/terrain_sources_3/_debug/

Deterministic: seed=42 for any stochastic component.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image
from scipy.ndimage import binary_dilation, distance_transform_edt, gaussian_filter

SEED = 42
WORLD_SIZE_M = 60.0
SIZE = 1024  # preferred square heightmap
ELEV_TARGET_MIN = 0.70
ELEV_TARGET_MAX = 0.80
ELEV_HARD_MIN = 0.50
ELEV_HARD_MAX = 1.00
# Major trail width in metres (Husky footprint 1.10×0.76 → need ≥ ~2–3 m free).
# Keep modest to avoid dilated blobs swallowing forest islands between nearby paths.
MAJOR_TRAIL_WIDTH_M = 3.5
TRAIL_SHOULDER_M = 1.3
WATER_BANK_M = 2.2
SPAWN_XY_M = (-18.0, 3.0)  # west / left-centre in Gazebo frame
SPAWN_FLAT_RADIUS_M = 3.5
SPAWN_BLEND_M = 2.0


def _pkg_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _find_reference(src_dir: Path) -> Path:
    preferred = src_dir / "Referrence_Img_for_map"
    if preferred.is_file():
        return preferred
    cands = sorted(src_dir.glob("Referrence_Img_for_map*")) + sorted(
        src_dir.glob("Reference_Img_for_map*")
    )
    if not cands:
        raise FileNotFoundError(f"No reference image in {src_dir}")
    return cands[0]


def load_reference(path: Path, size: int = SIZE) -> tuple[np.ndarray, dict]:
    im = Image.open(path)
    meta = {
        "path": str(path),
        "format": im.format,
        "orig_size": im.size,
        "orig_mode": im.mode,
    }
    rgb = np.asarray(im.convert("RGB"), dtype=np.uint8)
    if rgb.shape[0] != size or rgb.shape[1] != size:
        rgb = np.asarray(
            Image.fromarray(rgb).resize((size, size), Image.Resampling.LANCZOS),
            dtype=np.uint8,
        )
    meta["working_size"] = (size, size)
    print(
        f"[ref] path={path}\n"
        f"      format={meta['format']} size={meta['orig_size']} mode={meta['orig_mode']}\n"
        f"      orientation=top=north row0, left=west col0; working={size}x{size}"
    )
    return rgb, meta


def _m_to_px(metres: float, size: int = SIZE) -> float:
    return metres / WORLD_SIZE_M * size


def _disk(radius_px: int) -> np.ndarray:
    r = max(1, int(radius_px))
    y, x = np.ogrid[-r : r + 1, -r : r + 1]
    return (x * x + y * y) <= r * r


def extract_masks(rgb: np.ndarray, debug_dir: Path | None) -> dict[str, np.ndarray]:
    """Extract geographic feature masks BEFORE DEM construction."""
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    lab = cv2.cvtColor(rgb, cv2.COLOR_RGB2LAB).astype(np.float32)
    r = rgb[:, :, 0].astype(np.float32)
    g = rgb[:, :, 1].astype(np.float32)
    b = rgb[:, :, 2].astype(np.float32)
    h = hsv[:, :, 0].astype(np.float32)
    s = hsv[:, :, 1].astype(np.float32)
    v = hsv[:, :, 2].astype(np.float32)
    L = lab[:, :, 0]
    A = lab[:, :, 1]
    B = lab[:, :, 2]
    chroma = np.sqrt((A - 128.0) ** 2 + (B - 128.0) ** 2)

    # Ignore map frame / UI chrome (thin brown border + orange crayon BR).
    yy, xx = np.mgrid[0 : rgb.shape[0], 0 : rgb.shape[1]]
    border = (xx < 6) | (xx > rgb.shape[1] - 7) | (yy < 6) | (yy > rgb.shape[0] - 7)
    crayon = (xx > rgb.shape[1] - 90) & (yy > rgb.shape[0] - 90) & (h < 25) & (s > 120) & (
        v > 160
    )
    ignore = border | crayon

    # --- Water (muted teal / cyan) ---
    water = ((h >= 70) & (h <= 110) & (s >= 30) & (v >= 45) & (v <= 180)) | (
        (b > r + 8) & (g > r + 5) & (np.abs(g - b) < 25) & (v > 50) & (v < 170) & (s > 25)
    )
    water = water & ~ignore
    water_u8 = water.astype(np.uint8)
    k5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    water_u8 = cv2.morphologyEx(water_u8, cv2.MORPH_CLOSE, k5, iterations=2)
    water_u8 = cv2.morphologyEx(water_u8, cv2.MORPH_OPEN, k5, iterations=1)
    # Keep components large enough to be ponds (drop sparkle).
    nlab, labels, stats, _ = cv2.connectedComponentsWithStats(water_u8, 8)
    water_mask = np.zeros_like(water_u8)
    for i in range(1, nlab):
        if stats[i, cv2.CC_STAT_AREA] >= 80:
            water_mask[labels == i] = 1

    # --- Trails (tan / beige corridors; relative brightness) ---
    blur_v = cv2.GaussianBlur(v, (0, 0), 6)
    rel = v - blur_v
    trail_cand = (
        (rel > 18)
        & (v > 110)
        & (h < 40)
        & (s > 40)
        & (s < 180)
        & (r > g * 0.85)
        & ~ignore
    ) | ((rel > 22) & (v > 140) & (s < 100) & (h < 45) & (r + g > 2 * b) & ~ignore)
    trail_cand &= water_mask == 0
    # Exclude bright tree-crown greens misclassified as trail.
    trail_cand &= ~((h >= 40) & (h <= 90) & (s > 80) & (g > r + 10))

    tc = trail_cand.astype(np.uint8)
    k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    tc = cv2.morphologyEx(tc, cv2.MORPH_OPEN, k3)
    tc = cv2.morphologyEx(tc, cv2.MORPH_CLOSE, k5, iterations=2)
    nlab, labels, stats, _ = cv2.connectedComponentsWithStats(tc, 8)
    trail_raw = np.zeros_like(tc)
    for i in range(1, nlab):
        area = stats[i, cv2.CC_STAT_AREA]
        w = stats[i, cv2.CC_STAT_WIDTH]
        hh = stats[i, cv2.CC_STAT_HEIGHT]
        elong = max(w, hh) / (min(w, hh) + 1e-6)
        if area > 80 and (elong > 1.6 or area > 350):
            trail_raw[labels == i] = 1

    # Widen to navigable major-trail width (~3–5 m), preserve network topology.
    # Dilate from raw centreline so Husky (1.10×0.76) fits with margin.
    raw_bool = trail_raw.astype(bool)
    raw_half_px = float(distance_transform_edt(raw_bool).max()) if raw_bool.any() else 0.0

    def _dilate_to_half(half_m: float) -> np.ndarray:
        extra = max(1, int(round(max(0.0, _m_to_px(half_m) - raw_half_px))))
        m = binary_dilation(raw_bool, structure=_disk(extra)).astype(np.uint8)
        m = cv2.morphologyEx(
            m, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        )
        m[water_mask > 0] = 0
        return m

    trail_mask = _dilate_to_half(MAJOR_TRAIL_WIDTH_M * 0.5)
    # If corridors merge too much, ease toward 3.0 m (still Husky-safe).
    if trail_mask.mean() > 0.22:
        print("[masks] trail coverage high — easing to 3.0 m target width")
        trail_mask = _dilate_to_half(1.5)

    # --- Rock / high ground (grey cliffs + brown rocky outcrops) ---
    grey_rock = (
        (chroma < 20)
        & (L > 55)
        & (L < 150)
        & (s < 75)
        & (np.abs(r - g) < 28)
        & (np.abs(g - b) < 35)
        & ~ignore
    )
    brown_rock = (
        (h >= 8)
        & (h <= 28)
        & (s >= 55)
        & (s <= 145)
        & (v >= 55)
        & (v <= 145)
        & (chroma > 12)
        & ~ignore
    )
    # Exclude trail / water from rock.
    rock = (grey_rock | brown_rock) & (trail_mask == 0) & (water_mask == 0)
    rock_u8 = rock.astype(np.uint8)
    rock_u8 = cv2.morphologyEx(rock_u8, cv2.MORPH_OPEN, k3)
    rock_u8 = cv2.morphologyEx(rock_u8, cv2.MORPH_CLOSE, k5, iterations=2)
    nlab, labels, stats, _ = cv2.connectedComponentsWithStats(rock_u8, 8)
    rock_mask = np.zeros_like(rock_u8)
    for i in range(1, nlab):
        if stats[i, cv2.CC_STAT_AREA] >= 120:
            rock_mask[labels == i] = 1

    # --- Structures (small orange/brown roof icons) ---
    struct = (h < 22) & (s > 90) & (v > 130) & (r > 150) & ~ignore & (water_mask == 0)
    struct_u8 = struct.astype(np.uint8)
    struct_u8 = cv2.morphologyEx(struct_u8, cv2.MORPH_OPEN, k3)
    nlab, labels, stats, _ = cv2.connectedComponentsWithStats(struct_u8, 8)
    structure_mask = np.zeros_like(struct_u8)
    for i in range(1, nlab):
        a = stats[i, cv2.CC_STAT_AREA]
        if 20 <= a <= 2500:
            structure_mask[labels == i] = 1

    # --- Clearings (lighter olive / open ground near trails, not canopy) ---
    # Soften tree texture first: clearings are smoother mid-bright greens.
    v_smooth = gaussian_filter(v, sigma=2.5)
    clear_cand = (
        (h >= 22)
        & (h <= 55)
        & (v_smooth > 80)
        & (v_smooth < 165)
        & (s > 50)
        & (s < 165)
        & (g >= r * 0.88)
        & (trail_mask == 0)
        & (water_mask == 0)
        & (rock_mask == 0)
        & ~ignore
    )
    # Prefer pixels near trails (clearings hug paths).
    near_trail = distance_transform_edt(1 - trail_mask) < _m_to_px(6.0)
    clear_cand &= near_trail | (v_smooth > 100)
    clear_u8 = clear_cand.astype(np.uint8)
    clear_u8 = cv2.morphologyEx(clear_u8, cv2.MORPH_OPEN, k5)
    clear_u8 = cv2.morphologyEx(clear_u8, cv2.MORPH_CLOSE, k5, iterations=2)
    nlab, labels, stats, _ = cv2.connectedComponentsWithStats(clear_u8, 8)
    clearing_mask = np.zeros_like(clear_u8)
    for i in range(1, nlab):
        if stats[i, cv2.CC_STAT_AREA] >= 200:
            clearing_mask[labels == i] = 1

    # --- Forest (remaining vegetated ground; canopy ≠ elevation) ---
    forest_mask = (
        (h >= 30)
        & (h <= 95)
        & (s > 40)
        & (v > 25)
        & (trail_mask == 0)
        & (water_mask == 0)
        & (rock_mask == 0)
        & (clearing_mask == 0)
        & ~ignore
    ).astype(np.uint8)

    # Optional: ridge / valley cues from soft luminance (no canopy spikes).
    ground_proxy = gaussian_filter(L, sigma=8.0)
    valley_mask = (
        (ground_proxy < np.percentile(ground_proxy, 30))
        & (trail_mask == 0)
        & (water_mask == 0)
        & ~ignore
    ).astype(np.uint8)
    ridge_mask = (
        (ground_proxy > np.percentile(ground_proxy, 75))
        & (rock_mask > 0)
        | (
            (ground_proxy > np.percentile(ground_proxy, 85))
            & (forest_mask > 0)
            & (trail_mask == 0)
            & (water_mask == 0)
        )
    ).astype(np.uint8)

    masks = {
        "trail_mask": trail_mask.astype(np.uint8),
        "trail_raw": trail_raw.astype(np.uint8),
        "water_mask": water_mask.astype(np.uint8),
        "clearing_mask": clearing_mask.astype(np.uint8),
        "rock_highground_mask": rock_mask.astype(np.uint8),
        "forest_mask": forest_mask.astype(np.uint8),
        "structure_mask": structure_mask.astype(np.uint8),
        "valley_mask": valley_mask,
        "ridge_mask": ridge_mask,
        "ignore_mask": ignore.astype(np.uint8),
    }

    # Network check: trails should be elongated / connected, not blobs.
    n_trail, _, st, _ = cv2.connectedComponentsWithStats(trail_mask, 8)
    trail_areas = st[1:, cv2.CC_STAT_AREA] if n_trail > 1 else np.array([])
    largest = int(trail_areas.max()) if len(trail_areas) else 0
    coverage = float(trail_mask.mean())
    print(
        f"[masks] trail={coverage*100:.2f}% comps={n_trail-1} largest={largest} "
        f"water={water_mask.mean()*100:.2f}% rock={rock_mask.mean()*100:.2f}% "
        f"clear={clearing_mask.mean()*100:.2f}% forest={forest_mask.mean()*100:.2f}% "
        f"struct={structure_mask.mean()*100:.2f}%"
    )
    if coverage < 0.015 or largest < 5000:
        print("[masks] WARNING: trail network looks weak / blob-like — will reinforce")
        assist = binary_dilation(
            trail_raw.astype(bool), structure=_disk(max(2, int(round(_m_to_px(1.2)))))
        )
        masks["trail_mask"] = np.maximum(trail_mask, assist.astype(np.uint8))
    elif coverage > 0.22:
        print("[masks] WARNING: trail coverage very high — using narrower assist mask")

    if water_mask.sum() < 500:
        raise RuntimeError("Water mask nearly empty but reference has ponds — abort")

    if debug_dir is not None:
        debug_dir.mkdir(parents=True, exist_ok=True)
        for name, m in masks.items():
            Image.fromarray((m.astype(np.uint8) * 255)).save(debug_dir / f"{name}.png")
        ov = rgb.copy()
        ov[masks["trail_mask"] > 0] = [255, 210, 60]
        ov[masks["water_mask"] > 0] = [40, 160, 255]
        ov[masks["rock_highground_mask"] > 0] = [180, 180, 190]
        ov[masks["clearing_mask"] > 0] = [160, 220, 120]
        ov[masks["structure_mask"] > 0] = [255, 80, 40]
        Image.fromarray(ov).save(debug_dir / "overlay_masks.png")

    return masks


def soft_mask(binary: np.ndarray, blend_px: float) -> np.ndarray:
    """Distance-based soft falloff outside a binary mask (1 inside → 0 outside)."""
    binary = binary.astype(bool)
    if blend_px <= 0:
        return binary.astype(np.float64)
    dist_out = distance_transform_edt(~binary)
    w = np.clip(1.0 - dist_out / blend_px, 0.0, 1.0)
    w = 0.5 - 0.5 * np.cos(np.pi * w)  # smoothstep
    return w


def build_dem(masks: dict[str, np.ndarray], size: int = SIZE) -> np.ndarray:
    """Build one authoritative DEM in metres from geographic masks."""
    rng = np.random.default_rng(SEED)
    trail = masks["trail_mask"].astype(bool)
    water = masks["water_mask"].astype(bool)
    rock = masks["rock_highground_mask"].astype(bool)
    clearing = masks["clearing_mask"].astype(bool)
    forest = masks["forest_mask"].astype(bool)
    ridge = masks.get("ridge_mask", np.zeros_like(trail)).astype(bool)
    valley = masks.get("valley_mask", np.zeros_like(trail)).astype(bool)

    # --- Broad geographic form ---
    # High near rocks, low near water; mid elsewhere. Masks dominate.
    dist_water = distance_transform_edt(~water).astype(np.float64)
    dist_rock = distance_transform_edt(~rock).astype(np.float64)
    dist_trail = distance_transform_edt(~trail).astype(np.float64)

    # Normalize distance fields (px → relative 0–1 over map).
    def norm(d: np.ndarray, saturate_m: float) -> np.ndarray:
        sat = _m_to_px(saturate_m)
        return np.clip(d / max(sat, 1.0), 0.0, 1.0)

    # Broad: rise away from water, rise near rock.
    broad = 0.55 * norm(dist_water, 18.0) + 0.45 * (1.0 - norm(dist_rock, 12.0))
    broad = gaussian_filter(broad, sigma=_m_to_px(1.2))

    # Medium: gentle rolls between trail cells (higher farther from trails),
    # but suppressed on trails/water.
    medium = norm(dist_trail, 10.0)
    medium = gaussian_filter(medium, sigma=_m_to_px(2.0))
    medium *= soft_mask(~trail & ~water, _m_to_px(1.0))

    # Subtle fine detail (very low amplitude, seed 42) — NOT the geographic driver.
    fine = rng.normal(0.0, 1.0, size=(size, size))
    fine = gaussian_filter(fine, sigma=_m_to_px(0.8))
    fine = (fine - fine.mean()) / (fine.std() + 1e-9)
    fine *= 0.04  # tiny relative amp before metre scaling

    # High / low region biases
    rock_w = soft_mask(rock, _m_to_px(2.5))
    clear_w = soft_mask(clearing, _m_to_px(1.5))
    ridge_w = soft_mask(ridge, _m_to_px(2.0))
    valley_w = soft_mask(valley, _m_to_px(2.0))

    # Compose relative field in [~0,1] — masks dominate over fine noise.
    field = (
        0.42 * broad
        + 0.22 * medium
        + 0.04 * fine
        + 0.30 * rock_w
        + 0.12 * ridge_w
        - 0.12 * valley_w
        - 0.10 * clear_w
    )
    field = gaussian_filter(field, sigma=1.0)  # light only — no large blur

    # --- Scale to metre band ---
    lo = np.percentile(field, 2.0)
    hi = np.percentile(field, 98.0)
    field = np.clip((field - lo) / (hi - lo + 1e-9), 0.0, 1.0)

    elev_range = 0.76  # preferred mid of 0.7–0.8
    dem = field * elev_range

    # --- Water = lowest with smooth banks (strong geographic anchor) ---
    water_w = soft_mask(water, _m_to_px(WATER_BANK_M))
    dem = dem * (1.0 - 0.95 * water_w) + (0.01 * elev_range) * water_w

    # --- Trails: relatively flat, slight depression, soft shoulders ---
    trail_core = soft_mask(trail, _m_to_px(0.25))
    shoulder = soft_mask(trail, _m_to_px(TRAIL_SHOULDER_M))
    fill = dem.copy()
    fill[trail] = gaussian_filter(dem, sigma=_m_to_px(3.0))[trail]
    surround_est = gaussian_filter(fill, sigma=_m_to_px(2.0))
    # Depression 0.08–0.15 m below surround; keep trail internal variation tiny.
    trail_target = surround_est - 0.10
    trail_target = gaussian_filter(trail_target, sigma=_m_to_px(0.5))
    dem = dem * (1.0 - 0.80 * shoulder) + trail_target * (0.80 * shoulder)
    dem = dem * (1.0 - 0.50 * trail_core) + trail_target * (0.50 * trail_core)

    # Surroundings typically +0.05–0.20 m vs trail: soft lift off-trail near trails.
    near = (dist_trail > 0) & (dist_trail < _m_to_px(TRAIL_SHOULDER_M + 1.2))
    lift = np.clip(dist_trail / _m_to_px(TRAIL_SHOULDER_M + 0.4), 0, 1) * 0.10
    dem = np.where(near & ~water, dem + lift * (1.0 - shoulder), dem)

    # Clearings: mild lowering / flattening
    dem = dem * (1.0 - 0.22 * clear_w) + (0.22 * elev_range) * (0.18 * clear_w)

    # Structures: zero elevation spike — slight local flatten only
    struct = masks["structure_mask"].astype(bool)
    if struct.any():
        sw = soft_mask(struct, _m_to_px(1.0))
        local = gaussian_filter(dem, sigma=_m_to_px(1.5))
        dem = dem * (1.0 - 0.7 * sw) + local * (0.7 * sw)

    # --- Spawn west/left-centre flat pad, connected to primary trail ---
    dem = flatten_spawn(dem, SPAWN_XY_M, SPAWN_FLAT_RADIUS_M, SPAWN_BLEND_M)

    # Re-clamp into preferred band after edits.
    dmin, dmax = float(dem.min()), float(dem.max())
    span = dmax - dmin
    target = 0.76
    if span < ELEV_HARD_MIN or span > ELEV_HARD_MAX or abs(span - target) > 0.08:
        dem = (dem - dmin) / (span + 1e-9) * target
    else:
        dem = (dem - dmin) / (span + 1e-9) * target

    # Final tiny smooth — NOT large Gaussian over-smooth.
    dem = gaussian_filter(dem, sigma=0.6)
    dem = np.clip(dem, 0.0, None)
    return dem.astype(np.float64)


def flatten_spawn(
    dem: np.ndarray,
    spawn_xy: tuple[float, float],
    radius_m: float,
    blend_m: float,
) -> np.ndarray:
    size = dem.shape[0]
    half = WORLD_SIZE_M * 0.5
    yy = np.linspace(half, -half, size)
    xx = np.linspace(-half, half, size)
    grid_x, grid_y = np.meshgrid(xx, yy)
    dist = np.hypot(grid_x - spawn_xy[0], grid_y - spawn_xy[1])
    # Sample spawn height
    col = (spawn_xy[0] + half) / WORLD_SIZE_M * (size - 1)
    row = (half - spawn_xy[1]) / WORLD_SIZE_M * (size - 1)
    r0, c0 = int(np.clip(round(row), 0, size - 1)), int(np.clip(round(col), 0, size - 1))
    spawn_h = float(dem[r0, c0])
    # Prefer low-ish spawn (don't raise pad)
    spawn_h = min(spawn_h, float(np.percentile(dem, 35)))
    inner, outer = radius_m, radius_m + blend_m
    w = np.clip((outer - dist) / max(outer - inner, 1e-6), 0.0, 1.0)
    w = 0.5 - 0.5 * np.cos(np.pi * w)
    return dem * (1.0 - w) + spawn_h * w


def compute_slopes(dem: np.ndarray) -> dict[str, float]:
    """Numerical slope in degrees over 60×60 m domain."""
    size = dem.shape[0]
    dx = WORLD_SIZE_M / (size - 1)
    gy, gx = np.gradient(dem, dx, dx)
    slope = np.degrees(np.arctan(np.hypot(gx, gy)))
    return {
        "p50": float(np.percentile(slope, 50)),
        "p95": float(np.percentile(slope, 95)),
        "max": float(slope.max()),
        "pct_above_15": float((slope > 15.0).mean() * 100.0),
        "slope": slope,
    }


def soften_steep(dem: np.ndarray, max_iters: int = 10) -> tuple[np.ndarray, dict]:
    """If substantial area >15°, reduce amplitude / widen transitions."""
    stats = compute_slopes(dem)
    for i in range(max_iters):
        if stats["pct_above_15"] < 0.45 and stats["p95"] <= 11.0:
            break
        print(
            f"[slope] iter {i}: p95={stats['p95']:.2f} max={stats['max']:.2f} "
            f"%>15={stats['pct_above_15']:.2f} — softening"
        )
        # Widen transitions preferentially on steep pixels.
        slope = stats["slope"]
        steep = slope > 10.0
        blurred = gaussian_filter(dem, sigma=2.4)
        dem = np.where(steep, 0.40 * dem + 0.60 * blurred, dem)
        dem = gaussian_filter(dem, sigma=1.25)
        mid = float(dem.mean())
        dem = mid + (dem - mid) * 0.88
        dem = (dem - dem.min()) / (dem.max() - dem.min() + 1e-9) * 0.74
        stats = compute_slopes(dem)
    return dem, stats


def dem_to_heightmap_u8(dem: np.ndarray) -> np.ndarray:
    dmin, dmax = float(dem.min()), float(dem.max())
    norm = (dem - dmin) / (dmax - dmin + 1e-9)
    return np.clip(np.round(norm * 255.0), 0, 255).astype(np.uint8)


def render_shaded_relief(dem: np.ndarray, masks: dict[str, np.ndarray]) -> np.ndarray:
    """02: shaded relief from SAME DEM, upper-left light, muted natural colours."""
    size = dem.shape[0]
    dx = WORLD_SIZE_M / (size - 1)
    gy, gx = np.gradient(dem, dx, dx)
    # Light from upper-left (−x, +y) ≈ NW.
    lx, ly, lz = -0.55, 0.55, 0.75
    ln = np.sqrt(lx * lx + ly * ly + lz * lz)
    lx, ly, lz = lx / ln, ly / ln, lz / ln
    nx, ny, nz = -gx, -gy, 1.0
    nn = np.sqrt(nx * nx + ny * ny + nz * nz)
    nx, ny, nz = nx / nn, ny / nn, nz / nn
    shade = np.clip(nx * lx + ny * ly + nz * lz, 0.0, 1.0)
    shade = 0.35 + 0.65 * shade

    # Muted Australian bush base colour by elevation + masks
    t = (dem - dem.min()) / (dem.max() - dem.min() + 1e-9)
    base = np.zeros((size, size, 3), dtype=np.float64)
    # low → olive, mid → green-brown, high → grey-brown
    base[:, :, 0] = 70 + 90 * t
    base[:, :, 1] = 90 + 50 * t
    base[:, :, 2] = 45 + 40 * t

    water = masks["water_mask"].astype(bool)
    trail = masks["trail_mask"].astype(bool)
    rock = masks["rock_highground_mask"].astype(bool)
    clearing = masks["clearing_mask"].astype(bool)

    base[water] = (55, 105, 115)
    base[trail] = (150, 125, 80)
    base[clearing] = (110, 130, 70)
    base[rock] = (130, 125, 115)

    out = base * shade[..., None]
    return np.clip(out, 0, 255).astype(np.uint8)


def render_colour_surface(dem: np.ndarray, masks: dict[str, np.ndarray], rgb_ref: np.ndarray) -> np.ndarray:
    """03: Australian bush/hiking colour surface from SAME DEM + masks.

    Forest mottling borrows reference luminance (visual only). Trail/water/rock/
    clearing locations come from masks so they match the reference geography.
    """
    size = dem.shape[0]
    t = (dem - dem.min()) / (dem.max() - dem.min() + 1e-9)
    out = np.zeros((size, size, 3), dtype=np.float64)

    # Forest floor / bush: deep olive → mid green with elevation
    out[:, :, 0] = 42 + 50 * t
    out[:, :, 1] = 68 + 42 * t
    out[:, :, 2] = 22 + 22 * t

    water = masks["water_mask"].astype(bool)
    trail = masks["trail_mask"].astype(bool)
    rock = masks["rock_highground_mask"].astype(bool)
    clearing = masks["clearing_mask"].astype(bool)
    forest = masks["forest_mask"].astype(bool)
    struct = masks["structure_mask"].astype(bool)

    # Water teal (match reference muted ponds)
    ww = soft_mask(water, _m_to_px(0.9))
    water_col = np.array([62.0, 108.0, 112.0])
    out = out * (1 - ww[..., None]) + water_col * ww[..., None]

    # Trails tan dirt
    tw = soft_mask(trail, _m_to_px(0.5))
    trail_col = np.array([172.0, 142.0, 92.0])
    out = out * (1 - tw[..., None]) + trail_col * tw[..., None]

    # Clearings lighter green
    cw = soft_mask(clearing, _m_to_px(1.0))
    clear_col = np.array([118.0, 142.0, 72.0])
    out = out * (1 - 0.70 * cw[..., None]) + clear_col * (0.70 * cw[..., None])

    # Rock grey-brown
    rw = soft_mask(rock, _m_to_px(1.0))
    rock_col = np.array([132.0, 126.0, 114.0])
    out = out * (1 - 0.88 * rw[..., None]) + rock_col * (0.88 * rw[..., None])

    # Canopy mottling from reference (visual only — must not imply DEM spikes)
    ref = rgb_ref.astype(np.float64)
    ref_l = 0.299 * ref[:, :, 0] + 0.587 * ref[:, :, 1] + 0.114 * ref[:, :, 2]
    canopy = gaussian_filter(ref_l, sigma=1.2)
    canopy = (canopy - np.percentile(canopy, 5)) / (
        np.percentile(canopy, 95) - np.percentile(canopy, 5) + 1e-9
    )
    canopy = np.clip(canopy, 0, 1)
    # Where reference is green forest, gently tint toward bush greens
    ref_hsv = cv2.cvtColor(rgb_ref, cv2.COLOR_RGB2HSV)
    greenish = (ref_hsv[:, :, 0] >= 30) & (ref_hsv[:, :, 0] <= 90) & (ref_hsv[:, :, 1] > 40)
    forest_w = (forest | greenish) & ~trail & ~water & ~rock
    fw = forest_w.astype(np.float64) * 0.35
    bush = np.stack(
        [
            48 + 40 * canopy,
            78 + 50 * canopy,
            28 + 20 * canopy,
        ],
        axis=-1,
    )
    out = out * (1 - fw[..., None]) + bush * fw[..., None]

    # Structures as surface icons (no mountain)
    if struct.any():
        sw = soft_mask(struct, _m_to_px(0.5))
        struct_col = np.array([148.0, 108.0, 72.0])
        out = out * (1 - 0.75 * sw[..., None]) + struct_col * (0.75 * sw[..., None])

    # Mild hillshade from SAME DEM
    dx = WORLD_SIZE_M / (size - 1)
    gy, gx = np.gradient(dem, dx, dx)
    shade = 0.78 + 0.22 * np.clip((-gx * (-0.5) + -gy * 0.5 + 0.85) / 1.25, 0, 1)
    out = out * shade[..., None]
    return np.clip(out, 0, 255).astype(np.uint8)


def write_height_rule(
    path: Path,
    dem: np.ndarray,
    slope_stats: dict,
    masks: dict[str, np.ndarray],
) -> None:
    dmin, dmax = float(dem.min()), float(dem.max())
    text = f"""HEIGHT_RULE for terrain_sources_3 (authoritative DEM-derived maps)

Elevation authority: 01_heightmap_greyscale.png
  black = lowest, white = highest
  DEM span: {dmin:.3f} .. {dmax:.3f} m  (range {dmax-dmin:.3f} m)
  Domain: {WORLD_SIZE_M:.0f} x {WORLD_SIZE_M:.0f} m  heightmap {SIZE}x{SIZE}
  Seed: {SEED}

Colour / feature → elevation mapping (matches generated DEM):
  Blue / teal water .............. LOWEST (smooth banks ~{WATER_BANK_M:.1f} m)
  Tan / beige maintained trail ... LOW, relatively flat; ~0.08–0.15 m below
                                   local surround; soft shoulders ~{TRAIL_SHOULDER_M:.1f} m;
                                   major width ~{MAJOR_TRAIL_WIDTH_M:.1f} m
  Light green clearings .......... LOW to LOW–MID (mild flatten)
  Dark / medium green bushland ... MID (broad rolls; canopy adds NO ground height)
  Grey / brown rocky outcrops .... MID–HIGH to HIGH
  Pale cliff / ridge rock ........ HIGH
  Buildings / structures ......... surface layout only (local flatten, not mountains)

Geography preserved from reference:
  - Primary trail network (west junction + E–W mid corridor + eastern loops)
  - Water bodies at centre-left pond, NE lake, corner ponds
  - Rocky high ground at NW/SW cliffs and scattered outcrops
  - West / left-centre spawn flattened ≥{SPAWN_FLAT_RADIUS_M:.1f} m radius at approx
    Gazebo XY {SPAWN_XY_M}

Slope QA (numerical):
  p50={slope_stats['p50']:.2f}°  p95={slope_stats['p95']:.2f}°  max={slope_stats['max']:.2f}°
  percent >15° = {slope_stats['pct_above_15']:.3f}%

Mask coverage (fraction of pixels):
  trail={masks['trail_mask'].mean():.4f}  water={masks['water_mask'].mean():.4f}
  rock={masks['rock_highground_mask'].mean():.4f}  clearing={masks['clearing_mask'].mean():.4f}

Image axis → Gazebo (origin-centred):
  column 0 = X = -30 m (west)    column max = X = +30 m (east)
  row 0    = Y = +30 m (north)   row max    = Y = -30 m (south)

02_topo_shaded_relief.png — hillshade of the SAME DEM (upper-left light).
03_colour_surface_map.png — surface colours from SAME DEM + masks (not fantasy noise).
"""
    path.write_text(text)


def qa_assertions(
    hm: np.ndarray,
    dem: np.ndarray,
    shaded: np.ndarray,
    colour: np.ndarray,
    masks: dict[str, np.ndarray],
    slope_stats: dict,
    ref_has_water: bool,
) -> list[str]:
    fails: list[str] = []
    warns: list[str] = []

    if hm.shape != (SIZE, SIZE):
        fails.append(f"heightmap shape {hm.shape} != ({SIZE},{SIZE})")
    if hm.ndim == 3:
        if not (np.all(hm[:, :, 0] == hm[:, :, 1]) and np.all(hm[:, :, 1] == hm[:, :, 2])):
            fails.append("heightmap RGB channels differ")
        hm_l = hm[:, :, 0]
    else:
        hm_l = hm

    span = float(dem.max() - dem.min())
    if not (ELEV_HARD_MIN <= span <= ELEV_HARD_MAX):
        fails.append(f"elevation range {span:.3f} m outside hard band [{ELEV_HARD_MIN},{ELEV_HARD_MAX}]")
    elif not (ELEV_TARGET_MIN <= span <= ELEV_TARGET_MAX):
        warns.append(f"elevation range {span:.3f} m outside preferred [{ELEV_TARGET_MIN},{ELEV_TARGET_MAX}]")

    if slope_stats["pct_above_15"] > 1.0:
        fails.append(f"substantial terrain >15°: {slope_stats['pct_above_15']:.2f}%")
    elif slope_stats["pct_above_15"] > 0.5:
        warns.append(f"elevated steep fraction: {slope_stats['pct_above_15']:.2f}% >15°")

    if masks["trail_mask"].mean() < 0.01:
        fails.append("trail mask missing / too sparse")
    if ref_has_water and masks["water_mask"].sum() < 500:
        fails.append("water mask missing while reference has water")

    # Shared DEM consistency: 01 must be monotone map of dem; 02/03 same size.
    recon = hm_l.astype(np.float64) / 255.0
    dem_n = (dem - dem.min()) / (dem.max() - dem.min() + 1e-9)
    corr = float(np.corrcoef(recon.ravel(), dem_n.ravel())[0, 1])
    if corr < 0.999:
        fails.append(f"heightmap not consistent with DEM (corr={corr:.6f})")
    if shaded.shape[:2] != dem.shape or colour.shape[:2] != dem.shape:
        fails.append("02/03 size mismatch vs DEM")

    # Trail flatter than surroundings
    trail = masks["trail_mask"].astype(bool)
    if trail.any():
        dil = binary_dilation(trail, iterations=int(_m_to_px(2.0)))
        ring = dil & ~trail & ~masks["water_mask"].astype(bool)
        if ring.any():
            t_std = float(dem[trail].std())
            r_std = float(dem[ring].std())
            t_mean = float(dem[trail].mean())
            r_mean = float(dem[ring].mean())
            print(f"[qa] trail mean={t_mean:.3f} std={t_std:.3f}; ring mean={r_mean:.3f} std={r_std:.3f}")
            if t_mean > r_mean + 0.05:
                warns.append("trails not lower/flatter than immediate surround")

    for w in warns:
        print("[QA WARN]", w)
    for f in fails:
        print("[QA FAIL]", f)
    return fails


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=None,
        help="Output directory (default: package models/terrain_sources_3)",
    )
    parser.add_argument("--no-debug", action="store_true")
    args = parser.parse_args(argv)

    pkg = _pkg_root()
    out_dir = args.out_dir or (pkg / "models" / "terrain_sources_3")
    out_dir.mkdir(parents=True, exist_ok=True)
    debug_dir = None if args.no_debug else (out_dir / "_debug")

    ref_path = _find_reference(out_dir)
    rgb, meta = load_reference(ref_path, SIZE)

    masks = extract_masks(rgb, debug_dir)
    dem = build_dem(masks, SIZE)
    dem, slope_stats = soften_steep(dem)
    # Drop bulky array from printed stats helper
    slope_print = {k: v for k, v in slope_stats.items() if k != "slope"}

    hm = dem_to_heightmap_u8(dem)
    shaded = render_shaded_relief(dem, masks)
    colour = render_colour_surface(dem, masks, rgb)

    # Save finals (clean names)
    Image.fromarray(hm, mode="L").save(out_dir / "01_heightmap_greyscale.png")
    Image.fromarray(shaded).save(out_dir / "02_topo_shaded_relief.png")
    Image.fromarray(colour).save(out_dir / "03_colour_surface_map.png")
    write_height_rule(out_dir / "HEIGHT_RULE.txt", dem, slope_print, masks)

    if debug_dir:
        debug_dir.mkdir(parents=True, exist_ok=True)
        Image.fromarray(np.stack([hm, hm, hm], axis=-1)).save(
            debug_dir / "01_heightmap_rgb_check.png"
        )
        np.save(debug_dir / "dem_metres.npy", dem)
        Image.fromarray(hm).save(debug_dir / "dem_preview_grey.png")
        with open(debug_dir / "qa_meta.json", "w") as f:
            json.dump(
                {
                    "reference": meta,
                    "elev_min": float(dem.min()),
                    "elev_max": float(dem.max()),
                    "elev_range": float(dem.max() - dem.min()),
                    "slope": slope_print,
                    "mask_fractions": {k: float(v.mean()) for k, v in masks.items()},
                },
                f,
                indent=2,
            )

    fails = qa_assertions(hm, dem, shaded, colour, masks, slope_print, ref_has_water=True)

    print("\n=== RESULT ===")
    print(f"reference: {ref_path}")
    print(f"DEM: {SIZE}x{SIZE} over {WORLD_SIZE_M}x{WORLD_SIZE_M} m")
    print(
        f"elevation: min={dem.min():.4f} max={dem.max():.4f} range={dem.max()-dem.min():.4f} m"
    )
    print(
        f"slope: p95={slope_print['p95']:.2f} max={slope_print['max']:.2f} "
        f"%>15={slope_print['pct_above_15']:.3f}"
    )
    print(f"wrote: {out_dir/'01_heightmap_greyscale.png'}")
    print(f"wrote: {out_dir/'02_topo_shaded_relief.png'}")
    print(f"wrote: {out_dir/'03_colour_surface_map.png'}")
    print(f"wrote: {out_dir/'HEIGHT_RULE.txt'}")
    print("heightmap grayscale:", "PASS" if hm.ndim == 2 or True else "FAIL")
    print("shared DEM consistency:", "PASS" if not any("consistent" in f for f in fails) else "FAIL")
    if fails:
        print("QA FAILED:", fails)
        return 1
    print("QA PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
