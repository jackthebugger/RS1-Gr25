# Husky dimensions (this project)

Project-accurate Clearpath Husky size reference from this repository’s URDF, Gazebo xacro, and Nav2 config. Use when authoring or altering heightmaps, corridors, walls, and spawn patches for `custom_world_1` / `custom_world_hilly`.

**Primary sources**

| File | What it defines |
|------|-----------------|
| `urdf_husky/wheel.urdf.xacro` | Wheel radius, geometric track, wheel width |
| `urdf_husky/husky.urdf.xacro` | Chassis collisions, wheelbase, sensor mounts |
| `urdf_husky/husky.gazebo.xacro` | Lidar/camera plugins, DiffDrive track |
| `config/nav2_params_husky1.yaml` | Planner footprint + inflation |
| `rs1_nav/gazebo_world.py` | `HUSKY_LIDAR_HEIGHT = 0.845` |

---

## Frame / sit height

| Quantity | Value | Source / notes |
|----------|------:|----------------|
| Wheel radius | **0.1651 m** | `wheel.urdf.xacro` |
| Wheel diameter | **0.3302 m** | 2 × radius |
| Wheel axle height in `base_link` | **0.03282 m** | wheel joint origin Z |
| `base_link` above flat ground | **≈ 0.132 m** | axle − radius = 0.03282 − 0.1651 |
| Chassis underside clearance (approx.) | **≈ 0.13 m** | bottom collision vs ground |

**Spawn Z**

- Flat world (`custom_world_1`): `husky_z:=0.4`
- Hilly world (`custom_world_hilly`): local ground + ~0.45 → `husky_z:=0.74`

---

## Plan footprint (length × width)

### Physical (URDF)

| Dimension | Value | Notes |
|-----------|------:|-------|
| Chassis collision length | **0.9874 m** | main collision box |
| Chassis collision width | **0.5709 m** | main collision box |
| Bumper centres | **±0.48 m** along X | front/rear bumper visuals |
| Geometric wheel track | **0.5708 m** | centre-to-centre Y |
| Wheel width | **0.1143 m** | collision cylinder length |
| Overall width over wheels | **≈ 0.685 m** | 0.5708 + 0.1143 |
| Wheelbase (front→rear axle) | **0.512 m** | axles at X = ±0.256 m |
| Effective DiffDrive track | **0.94 m** | calibrated skid-steer (not geometric) |

### Nav2 footprint (what the planner uses)

```text
Footprint rectangle in husky1_base_link:
  ±0.55 m in X  → length 1.10 m
  ±0.38 m in Y  → width  0.76 m

YAML:
  footprint: '[ [0.55, 0.38], [0.55, -0.38], [-0.55, -0.38], [-0.55, 0.38] ]'
```

Slightly larger than the bare hull (margin). With `inflation_radius: 1.60 m`, preferred corridor openings should be several metres wide (forest gaps in `custom_world_1` are ≈ 5.68 m).

---

## Height stack (Z)

Heights relative to `base_link`, and approximate height above flat ground (`base_link` AGL ≈ 0.132 m).

| Element | Z in `base_link` | ≈ height above ground |
|---------|-----------------:|----------------------:|
| Ground contact | −0.132 m | 0 |
| Wheel axle | 0.033 m | ~0.165 m |
| IMU | 0.068 m | ~0.20 m |
| Main chassis collision centre | 0.12 m | ~0.25 m |
| Upper chassis collision | ~0.19 m | ~0.32 m |
| Top plate / arch | ~0.245–0.251 m | ~0.38 m |
| RGB-D `camera_link` | (0.45, 0, **0.25**) | ~0.38 m |
| Thermal sensor pose | (0.45, 0, **0.45**) | ~0.58 m |
| Lidar `base_scan` | (0, 0, **0.68**) | **≈ 0.81 m** (URDF-derived) |
| Project lidar-plane constant | — | **0.845 m** (`HUSKY_LIDAR_HEIGHT`) |
| Lidar visual cylinder | r = 0.09 m, h = 0.04 m | top ≈ **0.83–0.87 m** AGL |

**Terrain / obstacle rule:** features shorter than ~**0.85 m** are often invisible to the planar lidar and therefore to Nav2 costmaps. Barriers meant to block planning should be **≥ ~1.0–1.5 m** tall (obstacle injector default is 1.5 m).

---

## Mass (physics)

| Part | Mass |
|------|-----:|
| Base link | 46.064 kg |
| Each wheel | 2.637 kg |
| Total (base + 4 wheels) | **≈ 56.6 kg** |

---

## Sensors (terrain interaction)

### Lidar (primary for SLAM / Nav2)

| Spec | Value |
|------|------:|
| Type | horizontal `gpu_lidar` only (no vertical FOV) |
| Mount | centred on `base_link`, Z = 0.68 m |
| Range | **0.2 – 40 m** |
| FOV | **360°** |
| Default rate / samples | 10 Hz / 360 |
| Costmap obstacle height band | **0.0 – 2.0 m** |

### Cameras

| Sensor | Mount (`base_link`) | Notes |
|--------|---------------------|--------|
| RGB-D | (0.45, 0, 0.25) | off by default (`enable_camera:=false`) |
| Thermal | pose (0.45, 0, 0.45) | always present; fire keep-outs |

---

## Compact card (paste into map / heightmap prompts)

```text
Clearpath Husky (this project's sim model)
- Nav2 footprint: 1.10 m long × 0.76 m wide (half-extents 0.55 × 0.38)
- Physical overall width over wheels: ~0.69 m
- Physical chassis length: ~0.99 m (bumpers out to ~±0.48 m)
- Wheelbase: 0.512 m; wheel radius: 0.165 m; ground clearance: ~0.13 m
- base_link sits ~0.13 m above flat ground
- Planar lidar plane: ~0.81–0.85 m above ground; 360°; 0.2–40 m
- Obstacles shorter than ~0.85 m often invisible to navigation
- Prefer slopes ≪ ~15°; keep spawn patch flat (≥ ~3–4 m radius)
- Prefer corridor free width ≫ 0.76 m; with 1.6 m inflation, openings of several metres are safer
- World arena used with this robot: 60 × 30 m (perimeter); heightmap mesh may be 60 × 60 m
```

---

## Confirmed vs derived

| Item | Status |
|------|--------|
| Wheel radius, track, wheelbase, collision boxes, lidar/camera mounts, Nav2 footprint | **Confirmed** from URDF / YAML |
| `base_link` AGL ≈ 0.132 m; lidar AGL ≈ 0.81 m | **Derived** from URDF |
| Lidar plane **0.845 m** | **Project constant** — prefer this for obstacle height rules |
| Exact DAE visual mesh AABB | **Not re-measured**; collision + Nav2 numbers are what planning uses |

---

## Related terrain assets

| Path | Role |
|------|------|
| `models/terrain_sources/01_heightmap_greyscale.png` | Elevation authority |
| `models/terrain_sources/HEIGHT_RULE.txt` | Colour → height interpretation |
| `scripts/generate_hilly_heightmap.py` | Regenerates `hilly_ground` PNG |
| `worlds/custom_world_hilly.sdf` | Heightmapped team world |
