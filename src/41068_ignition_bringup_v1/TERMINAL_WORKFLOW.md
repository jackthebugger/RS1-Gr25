# Terminal Workflow Guide — Husky Sim, Mapping, Teleop & Pathfinding

**Package:** `41068_ignition_bringup`  
**Branch context:** use this after checking out this team's branch and rebuilding  
**ROS 2:** Humble · **Simulator:** Ignition Gazebo Fortress  
**Team world / spawn:** `custom_world_1` at `(-18, 3, 0.4)`, yaw `0.0`

This is the practical runbook for every teammate: which terminals to open, what to run in each, and which modes can run together.

---

## 0. One-time setup (every machine)

### 0.1 Clone and locate the workspace

The ROS workspace root is the repo root (the folder that contains `src/` and, after build, `install/`).

```bash
# Example — use YOUR clone path:
export WS="$HOME/git/RS1-Gr25"
cd "$WS"
```

All commands below assume `WS` is set. If you skip the variable, replace `"$WS"` with your real path.

### 0.2 Install dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-dev-tools \
  ros-humble-robot-localization \
  ros-humble-ros-ign \
  ros-humble-ros-ign-interfaces \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-teleop-twist-keyboard \
  python3-numpy
```

Install Ignition Gazebo Fortress if you do not already have it (see package `README.md`).

### 0.3 Build

```bash
export WS="$HOME/git/RS1-Gr25"   # ← your clone
cd "$WS"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select beer_fire_detection 41068_ignition_bringup
```

Rebuild after every `git pull` that touches this package.

### 0.4 Environment block (paste into every new terminal)

```bash
export ROS_LOCALHOST_ONLY=1          # avoids picking up classmates' topics on Wi‑Fi
export WS="$HOME/git/RS1-Gr25"       # ← your clone
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
cd "$WS"
```

**Do not** `colcon build` inside `src/.../41068_ignition_bringup/` — always build from `"$WS"`.

---

## 1. Canonical team launch (use this)

Every teammate uses this exact Terminal 1 command for our world and spawn:

```bash
ros2 launch 41068_ignition_bringup 41068_ignition_husky.launch.py \
  slam:=true nav2:=false rviz:=true gui:=true \
  world:=custom_world_1 \
  husky_x:=-18 husky_y:=3 husky_z:=0.4 husky_yaw:=0.0
```

| Flag | Meaning |
|------|---------|
| `slam:=true` | SLAM Toolbox builds `/husky1/map` |
| `nav2:=false` | Keyboard teleop owns `/husky1/cmd_vel` (no Nav2 fighting you) |
| `rviz:=true` | Live map + laser in RViz |
| `world:=custom_world_1` | Team world (perimeter walls + flat ground) |
| `husky_x/y/z/yaw` | Spawn pose used by the team |

**Rule:** Nav2 and keyboard teleop both publish `/husky1/cmd_vel`. This launch keeps Nav2 off so WASD works. To send an **autonomous** Nav2 goal later, stop this launch and use Workflow B (`nav2:=true`) instead.

---

## 2. Shared facts (namespaces)

| Item | Value |
|------|--------|
| Robot namespace | `husky1` |
| Cmd vel | `/husky1/cmd_vel` |
| Lidar | `/husky1/scan` |
| SLAM map | `/husky1/map` |
| Nav2 action | `/husky1/navigate_to_pose` |
| RViz Fixed Frame | `husky1_map` |
| Team spawn | `husky_x:=-18 husky_y:=3 husky_z:=0.4 husky_yaw:=0.0` |
| Team world | `custom_world_1` |
| Default script goal (`custom_world_1`) | `(-4.5, -4.5, 0)` in `husky1_map` |

---

## 3. Workflow A — Map + drive (WASD), including driving to a goal pose

Use this to build a lidar map and to **drive the Husky yourself** toward a specific place on the map.

### Terminal layout

```text
Terminal 1  →  Gazebo + Husky + SLAM + RViz   (canonical launch)
Terminal 2  →  WASD teleop   (keep this window focused while driving)
Terminal 3  →  Save map when finished (optional)
```

### Terminal 1 — simulation + SLAM + RViz

```bash
export ROS_LOCALHOST_ONLY=1
export WS="$HOME/git/RS1-Gr25"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

ros2 launch 41068_ignition_bringup 41068_ignition_husky.launch.py \
  slam:=true nav2:=false rviz:=true gui:=true \
  world:=custom_world_1 \
  husky_x:=-18 husky_y:=3 husky_z:=0.4 husky_yaw:=0.0
```

Wait ~15 s for SLAM (`nav_start_delay`). In RViz:

- Fixed Frame: `husky1_map`
- Map display: `/husky1/map`
- LaserScan: `/husky1/scan`

### Terminal 2 — WASD teleop (drive / move toward your goal)

```bash
export ROS_LOCALHOST_ONLY=1
export WS="$HOME/git/RS1-Gr25"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

ros2 run 41068_ignition_bringup wasd_teleop.py --ros-args -r cmd_vel:=/husky1/cmd_vel
```

| Key | Action |
|-----|--------|
| `W` / `S` | forward / reverse |
| `A` / `D` | turn left / right |
| `Q` / `E` | slower / faster linear |
| `Z` / `C` | slower / faster turn |
| `Space` | stop |
| `Ctrl-C` | quit |

Motion sticks after a key press until you press another key or Space.

To **move to a specific goal** in this mode: watch the robot pose / map in RViz and drive with WASD until you reach that location (for example toward `(-4.5, -4.5)`).

Fallback (classic keys `i` / `,` / `j` / `l`):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/husky1/cmd_vel
```

### Terminal 3 — save the map

```bash
export ROS_LOCALHOST_ONLY=1
export WS="$HOME/git/RS1-Gr25"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
mkdir -p "$WS/maps"

ros2 run nav2_map_server map_saver_cli \
  -f "$WS/maps/my_map" \
  --ros-args -r map:=/husky1/map -p use_sim_time:=true
```

Writes `$WS/maps/my_map.pgm` and `$WS/maps/my_map.yaml`.

**Common mistake:** `map_saver_cli -f my_map` alone listens on `/map` without sim time and fails with `Failed to spin map subscription`. Always remap to `/husky1/map` and set `use_sim_time:=true`.

---

## 4. Workflow B — Autonomous pathfinding to our goal (Nav2)

Use this when Nav2 should **plan and drive** to the team goal (not WASD).

Same world and spawn as Workflow A — only `nav2` changes to `true`. Do **not** run WASD teleop in this mode.

### Terminal layout

```text
Terminal 1  →  Gazebo + Husky + SLAM + Nav2 + RViz
Terminal 2  →  Pathfinding script  OR  Nav2 Goal in RViz
```

### Terminal 1 — simulation + Nav2 + RViz

```bash
export ROS_LOCALHOST_ONLY=1
export WS="$HOME/git/RS1-Gr25"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

ros2 launch 41068_ignition_bringup 41068_ignition_husky.launch.py \
  slam:=true nav2:=true rviz:=true gui:=true \
  world:=custom_world_1 \
  husky_x:=-18 husky_y:=3 husky_z:=0.4 husky_yaw:=0.0
```

Wait until Nav2 is up (~15–30 s). Check:

```bash
ros2 action list | grep navigate_to_pose
# expect: /husky1/navigate_to_pose
```

### Terminal 2 — send our specific goal

Default team goal for `custom_world_1` is **`(-4.5, -4.5, 0)`** in `husky1_map`.

```bash
export ROS_LOCALHOST_ONLY=1
export WS="$HOME/git/RS1-Gr25"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

ros2 run 41068_ignition_bringup basic_autonomy_demo.py \
  --attach --world custom_world_1 --goal -4.5 -4.5 0
```

Or via launch file:

```bash
ros2 launch 41068_ignition_bringup 41068_autonomy_demo.launch.py \
  robot:=husky1 mission_mode:=single_goal \
  goal_x:=-4.5 goal_y:=-4.5 goal_yaw:=0.0
```

### Alternative — click a goal in RViz (no script)

1. Fixed Frame = `husky1_map`
2. **Nav2 Goal** / **2D Goal Pose** tool
3. Click free space (e.g. near `-4.5, -4.5`)

### Optional — CLI goal

```bash
ros2 action send_goal /husky1/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'husky1_map'}, pose: {position: {x: -4.5, y: -4.5, z: 0.0}, orientation: {w: 1.0}}}}"
```

---

## 5. Workflow C — One-command pathfinding demo (script starts everything)

No separate Terminal 1 needed. Uses the same world / spawn / goal as above.

```bash
export ROS_LOCALHOST_ONLY=1
export WS="$HOME/git/RS1-Gr25"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

ros2 run 41068_ignition_bringup basic_autonomy_demo.py \
  --world custom_world_1 \
  --start -18 3 0 \
  --goal -4.5 -4.5 0 \
  --gui --rviz
```

### Replanning demo (obstacle mid-route)

```bash
ros2 run 41068_ignition_bringup basic_autonomy_demo.py --replan --gui --rviz \
  --world custom_world_1 --start -18 3 0 --goal -4.5 -4.5 0
```

---

## 6. Optional extras (extra terminals)

Only after Terminal 1 (sim) is already running.

### Obstacle injector

```bash
ros2 run 41068_ignition_bringup obstacle_injector.py
```

### Robot status GUI

```bash
ros2 launch 41068_ignition_bringup robot_status_gui.launch.py
```

### Random-walk autonomy (needs Nav2 already up — Workflow B Terminal 1)

```bash
ros2 run 41068_ignition_bringup basic_autonomy_demo.py --attach --mode random_walk
```

---

## 7. Quick-reference cheat sheet

### Every terminal

```bash
export ROS_LOCALHOST_ONLY=1
export WS="$HOME/git/RS1-Gr25"    # your path
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
```

### Mapping / drive with WASD (including manual drive to a goal)

| T | Command |
|---|---------|
| 1 | `ros2 launch 41068_ignition_bringup 41068_ignition_husky.launch.py slam:=true nav2:=false rviz:=true gui:=true world:=custom_world_1 husky_x:=-18 husky_y:=3 husky_z:=0.4 husky_yaw:=0.0` |
| 2 | `ros2 run 41068_ignition_bringup wasd_teleop.py --ros-args -r cmd_vel:=/husky1/cmd_vel` |
| 3 | `map_saver_cli -f "$WS/maps/my_map" --ros-args -r map:=/husky1/map -p use_sim_time:=true` |

### Autonomous pathfinding to `(-4.5, -4.5)`

| T | Command |
|---|---------|
| 1 | `ros2 launch 41068_ignition_bringup 41068_ignition_husky.launch.py slam:=true nav2:=true rviz:=true gui:=true world:=custom_world_1 husky_x:=-18 husky_y:=3 husky_z:=0.4 husky_yaw:=0.0` |
| 2 | `ros2 run 41068_ignition_bringup basic_autonomy_demo.py --attach --world custom_world_1 --goal -4.5 -4.5 0` |

---

## 8. Cleanup / stuck simulation

If RViz flashes or you see `jump back in time` / stale clocks:

```bash
# Stop launches with Ctrl-C in each terminal, then:
pkill -f 'ign gazebo|gz sim|ros2|rviz2' || true
# Confirm nothing leftover:
ps aux | grep -E 'ign gazebo|gz sim|rviz2|async_slam' | grep -v grep
```

Then start again from a fresh sourced terminal.

---

## 9. Troubleshooting

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| `Package '41068_ignition_bringup' not found` | Workspace not sourced | `source "$WS/install/setup.bash"` |
| Teleop does nothing | Wrong terminal focus, or wrong topic | Focus teleop terminal; remap `cmd_vel:=/husky1/cmd_vel` |
| Robot jerks / ignores WASD | `nav2:=true` fighting teleop | Use Workflow A (`nav2:=false`). For Nav2 goals use Workflow B and stop teleop |
| Empty map in RViz | Wrong Fixed Frame / topic | Frame `husky1_map`, topic `/husky1/map` |
| `Failed to spin map subscription` | Wrong topic or no sim time | `-r map:=/husky1/map -p use_sim_time:=true` |
| No `navigate_to_pose` | Still on Workflow A / Nav2 not up | Launch Workflow B (`nav2:=true`); wait ≥15 s |
| Goal rejected / robot stuck | Goal in obstacle or off map | Click free space; check costmaps in RViz |
| Gazebo texture thrash / freeze | Heavy textures + software GL | Stick to `custom_world_1`; avoid `software_gl:=true` on normal Linux GPUs |

---

## 10. Related docs in this package

| File | Contents |
|------|----------|
| `41068_ignition_bringup/README.md` | Install, launch args, multi-robot notes |
| `41068_ignition_bringup/simulation_workflow_and_implementation_overview.md` | Deep implementation map (§6b = mapping detail) |
| `41068_ignition_bringup/scripts/basic_autonomy_demo.py` | Pathfinding / mission entry point |
| `41068_ignition_bringup/scripts/wasd_teleop.py` | WASD keyboard driver |

---

## 11. Suggested first run for a new teammate

1. Set `WS`, install deps, `colcon build`, source (§0).
2. **Workflow A:** run the canonical launch + WASD — confirm the map grows and the Husky moves.
3. Optionally save the map (Terminal 3).
4. Stop Terminal 1 (`Ctrl-C`).
5. **Workflow B:** same world/spawn with `nav2:=true`, then send goal `-4.5 -4.5 0`.
6. Confirm `/husky1/plan` appears in RViz and the robot drives.
