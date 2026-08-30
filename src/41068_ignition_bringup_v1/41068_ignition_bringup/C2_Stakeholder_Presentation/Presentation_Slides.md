# C2 Presentation Slides — Jack Havranek

**Speaker:** Jack Havranek  
**Contribution:** Robot movement and path planning  
**Segment:** Slides 1–3 (movement / navigation within the group presentation)

**Technical scope:** Integrate provided sensor data (LiDAR, odometry, IMU) with groupmates’ thermal sensor data so the Husky can determine the **safest path** to its goal — including when **dynamic blockages** appear mid-route.

**Companion docs:** `pathplanning_and_movement_implementation.md`, `master_robot_movement_pathplanning.md`

---

## Slide 1 — Broader Vision (Where This Can Go)

**Stakeholder hook:** The robot does not just drive — it **finds and updates the escape route**.

### Talking points

- **Beyond 41068:** A lead robot that continuously recomputes the safest evacuation route for a following fire crew using **multi-sensor fusion** (LiDAR + thermal/IR + future soot sensing), not only at mission start
- **~5 years:** Husky-class UGV on a fireground analyses hotspots, assesses hazards, selects paths, and **updates routes** as conditions change; operator supervises via GUI with live paths and risk layers
- **Value:** Less time in smoke and heat; path suitable for the **firetruck** (space, incline, heat margins from our ConOps)
- **41068 stepping stone:** Build the **autonomous navigation core** in simulation — live mapping, global planning, local following, **dynamic replanning** when the route is blocked — before full bushfire thermal fusion and multi-robot stretch goals

### Template questions (answered)

| Question | Short answer |
|---|---|
| What could the system become? | Field-deployed autonomous evacuation guide with continuous safe-route updates |
| Real-world version in ~5 years? | Coordinated bushfire platform; Husky investigates and routes; human survival priority |
| Broader value? | Safer, faster escape for trapped SES / fire crews |
| How does 41068 help? | Proves sensor-driven plan → drive → **replan** loop in simulation |

---

## Slide 2 — What We Will Demonstrate by the End

**Stakeholder hook:** You will **see** the robot plan, drive, and recover when the path changes.

### What the stakeholder will see

- Husky spawned in a forest simulation world
- Operator (or demo script) sets a **safe-house goal**
- Robot **autonomously** builds a map, plans a path, and drives there
- If a **dynamic blockage** appears on the route (wall demo today; **fire/heat zones** once thermal is integrated), the robot **replans** and still reaches the goal

### Integrated accomplishment

```text
Sensors (lidar + thermal [planned]) → costmaps → global plan → follow path
  → blockage detected → replan → continue → GOAL
```

### Success criteria

| Measure | Target |
|---|---|
| Autonomous navigation | MVP R1 — no manual driving to goal |
| Obstacle avoidance | MVP R4 — detour when route blocked |
| Safe route generation | MVP R9 — route avoids known hazards |
| Replan on dynamic blockage | Visible new `/husky1/plan` after obstacle appears |
| Goal reached | Nav2 `SUCCEEDED`; tolerances 0.25 m / 0.35 rad |
| Fire-aware routing (next) | Routes avoid **thermal hazard** regions from groupmates’ sensor pipeline (R2) |

### Demo commands

```bash
# start → goal
python3 scripts/basic_autonomy_demo.py --start 0 0 0 --goal 0 -5 0

# dynamic blockage + replan
python3 scripts/basic_autonomy_demo.py --replan
```

### Template questions (answered)

| Question | Answer |
|---|---|
| What will they see? | Husky autonomously navigating with map and path visible in RViz2 |
| What will the system accomplish? | Sense → plan → drive → replan if blocked → arrive |
| How do we know it worked? | Goal success + no collision with injected obstacle + plan change on replan |
| Key outcomes? | R1, R4, R9; Nadim C2 bar: sensing, moving, avoiding obstacles toward goal |

---

## Slide 3 — Progress So Far

**Stakeholder hook:** Movement is **implemented and tested** — not only proposed.

### Implemented (`2026-08-28` — confirmed in repo)

| Item | Evidence |
|---|---|
| Full Nav2 + SLAM stack | `simple_trees` world; `nav2:=true` lifecycle active |
| Configurable start pose | `husky_x/y/yaw` launch args; demo `--start` |
| Start→goal autonomous demo | `scripts/basic_autonomy_demo.py` |
| Dynamic replan demo | `--replan` + `rs1_nav/PathBlocker` (real Gazebo wall on path) |
| Automated tests | `navigation_test`, `replan_test`, `movement_test` |
| Replan verified | Costmap 254 on barrier; plan divergence ~1.72 m; goal ~32.6 s |

### Sensor → planning chain (today)

- **LiDAR** `/husky1/scan` @ 10 Hz → SLAM map + Nav2 global/local costmaps
- **NavFn** (A*) plans on rolling 40 m global costmap
- **Regulated Pure Pursuit** follows path; BT **replan** when path invalid

### Evidence to show in presentation

- RViz2: map, laser scan, global/local costmaps, planned path
- Gazebo: Husky driving around trees
- Optional live or recorded `--replan` run (path redirects around dropped wall)

### Contrast with C1 Week 3

| Week 3 (`11/08/2026`) | Now (movement implementation) |
|---|---|
| Sim loaded; Nav2 status `unknown` | Nav2 active; `navigate_to_pose` works |
| No autonomous goal demo | Start→goal + replan demos |
| Setup evidence only | Tests + automated mission runner |

### What this taught us

- Provided lidar + Nav2 is the right foundation (course stack, not a custom planner from scratch)
- **Dynamic replanning works** when obstacles are real Gazebo models — same pattern will apply to **moving fire/heat zones**
- Regulated Pure Pursuit outperformed DWB for our replan scenario on WSL2

### Next (Jack’s lane)

- **Integrate groupmates’ thermal sensor data** into hazard representation / costmaps
- “Safest path” then avoids **fire hotspots** as well as trees and walls
- Extend dynamic blockage tests from geometric walls to **heat-driven hazard zones**

### Template questions (answered)

| Question | Answer |
|---|---|
| Implemented / tested? | Nav2 stack, demo script, `rs1_nav/`, replan tests — see table above |
| Evidence? | RViz screenshots/video of map, path, costmaps; replan run |
| What demonstrated? | Autonomous navigation and replanning are feasible within the semester |
| Feasibility confidence? | Closes the gap between C1 “sim loaded” and Nadim’s C2 expectation of sensing + motion + avoidance |

---

## Stakeholder one-liner (30 seconds)

“We simulate a Husky in Gazebo. Its lidar builds a live map. You pick a start and a goal; Nav2 plans a path and drives there. If we drop a wall on that path, the laser sees it, the map updates, and the robot goes around — no one is joysticking it. Next we wire in thermal data so it avoids fire hotspots too.”
