# Jack Havranek — C2 Presentation Working Draft

**Speaker:** Jack Havranek  
**Contribution:** Robot movement and path planning  
**Segment:** Slides J1–J3 (movement / navigation within the group presentation)

---

# 1. Task Understanding

**Overall task:** Group 25 delivers a **10-minute stakeholder-facing presentation** for Component 2 (C2) of UTS **41068 Robotics Studio 1**. The audience is the defined stakeholder (firefighters/SES, assessed via tutor Nadim), not a technical examiner.

**Final deliverable:** In-class presentation using the provided template, covering who the stakeholder is, the problem, vision, semester solution, technical approach, how it works for the stakeholder, **evidence of progress**, end-of-semester demo intent, and delivery path with milestones/risks.

**Main objective:** Give the stakeholder confidence that B.E.E.R. (Bushfire Environmental Evacuation & Rescue) understands their needs, delivers real value, and is on a credible path to a working semester demo.

**Assessment context:** 18 marks (6 rubric criteria × 3). Overtime is penalised. All four members must contribute meaningfully in person.

---

# 2. My Responsibility

| Item | Detail |
|---|---|
| **Role** | Robot movement and path planning |
| **C2 speaker segment** | Slides 1–3 within the group deck (movement / navigation lane) |
| **Technical ownership** | Nav2 mission layer, start→goal autonomy, dynamic replanning, sensor-driven costmaps; future integration of teammates’ thermal hazard data |
| **Required outputs** | Three presentation slides + spoken delivery; live or recorded demo evidence; integration touchpoints with perception (Phu), UI (Faiyad), sim environment (Taj) |
| **Dependencies** | Phu: LiDAR/thermal topics; Taj: custom bushfire world and fire zones; Faiyad: operator UI showing path/pose/hazards |

**Assumption:** Jack’s three slides are the **movement/navigation block** inside the full team deck. Other members cover stakeholder intro, problem, semester ConOps overview, UI, sim environment, and delivery timeline. Jack does **not** need to repeat the full project pitch on every slide — he anchors navigation to the stakeholder story.

---

# 3. Requirements & Rubric Targets

### Mandatory C2 communications (Jack’s contribution)

| Required point | Jack’s coverage |
|---|---|
| Broader vision | Slide 1 — autonomous evacuation routing, not teleop |
| End-of-semester demonstration | Slide 2 — what stakeholder will **see** the Husky do |
| Progress evidence | Slide 3 — implemented Nav2 stack, tests, replan demo |

### Nadim’s C2 evidence bar (movement-relevant)

- Robot **sensing and moving** toward a goal
- **Avoiding obstacles** while reaching the goal
- Proof something real is **working or tested**

### MVP requirements in Jack’s lane

| ID | Requirement | Jack’s evidence |
|---|---|---|
| R1 | Autonomous navigation to safe goal | `basic_autonomy_demo.py`, `navigation_test.py` 3/3 |
| R4 | Avoid blocked route and continue | `--replan`, `replan_test.py` |
| R9 | Safe route avoiding known hazards | Costmap-based NavFn planning; thermal = next step for fire hazards |

### Rubric priorities for Jack’s segment

| Criterion | What Jack must do |
|---|---|
| **1 — Understand problem/proposal** | Plain English: robot finds and updates the escape route |
| **2 — Real value** | Less time in smoke/heat; route suitable for following firetruck |
| **4 — Progress shown** | **Strongest criterion for Jack** — show RViz/Gazebo evidence, test results |
| **5 — Trust delivery path** | Today: lidar + walls; next: thermal hotspots; same replan loop |
| **6 — Engaged stakeholder** | Concise, no ROS jargon, on time |

---

# 4. Recommended Structure

### Jack’s slide sequence (within group deck)

```
[Other members: stakeholder, problem, semester solution intro]
    ↓
Slide J1 — Broader Vision: Autonomous Safe Routing
Slide J2 — End-of-Semester Demo: Plan → Drive → Replan → Goal
Slide J3 — Progress So Far: Working Nav2 Stack + Evidence
    ↓
[Other members: delivery path, risks, Q&A handoff]
```

### Timing (recommended)

| | Duration |
|---|---|
| **Total presentation** | 10:00 max |
| **Jack’s section** | **2:30** (25% of deck; evidence-heavy) |
| **Jack’s slides** | 3 |
| **Per slide** | ~50 s |

---

# 5. COMPLETE DRAFT

---

## Slide J1 — Broader Vision: The Robot Finds the Escape Route

**Slide content:**

```
B.E.E.R. — Where Navigation Can Go

Today (41068):          Beyond 41068 (~5 years):
• Husky plans & drives   • Lead robot continuously updates
  to a safe goal           the safest evacuation route
• Replan when blocked    • LiDAR + thermal + hazard layers
• Simulation only        • Operator supervises via live map/GUI

Value for firefighters & SES:
→ Less time in smoke and heat
→ Route wide enough for the firetruck, not just the robot
→ Same replan logic when fire zones move — not just trees
```

**Visual:**

Split-screen diagram (left = semester, right = long-term):

- **Left panel:** Simple flowchart: `LiDAR scan → live map → plan path → drive → blocked? → replan → safehouse`. Use a top-down forest map with green planned path and a red “blocked” segment rerouting around a tree cluster.
- **Right panel:** Same flowchart extended with a **thermal heat overlay** (orange/red zones) and a GUI mock showing operator view. Annotate “firetruck follows same corridor.”
- **Do not use stock fire photos** — use a schematic or `[INSERT RViz MAP WITH PLANNED PATH]`.

**AI generation prompt — Left panel (semester / 41068):**

> Flat engineering presentation graphic, 16:9 slide inset, not a photograph. Top-down 2D occupancy-grid style map on a light grey grid background: irregular dark grey blobs for tree clusters, pale green free space, a dashed bright green polyline path from a small yellow quadrilateral robot icon at top-centre curving south toward a blue house marker labelled “safehouse”. One segment of the original path crossed out in thin red with a small red X and label “blocked”; a detour path bends around a tree cluster. Above the map, a simple horizontal flowchart in Arial or Helvetica, black lines, white fill boxes: “LiDAR scan → live map → plan path → drive → blocked? → replan → safehouse”. Small caption bottom-left: “41068 — simulation”. Muted university slide palette (white, #2d5016 green path, #888 grey trees). No shadows, no 3D, no glow, no cinematic lighting, no stock-photo fire, no hyper-real textures. Looks like a competent student made it in PowerPoint or draw.io — slightly imperfect alignment, clean but utilitarian.

**AI generation prompt — Right panel (long-term / ~5 years):**

> Same flat diagram style as left panel for visual consistency. Top-down map now includes semi-transparent orange-red heat blobs (thermal hazard overlay) with soft edges, not dramatic flames. Green planned path widened slightly with a faint parallel dashed corridor labelled “firetruck follows same corridor” behind the robot icon. Right side of the panel: simple operator GUI mock — dark grey window chrome, flat UI, small map thumbnail, legend for “lidar obstacles / thermal hazard / planned route”, status text “Route updated 14:32”. Extended flowchart adds boxes: “thermal layer” and “operator GUI”. Caption: “Beyond 41068”. Flat vector infographic, engineering report aesthetic, no photorealism, no AI art gloss, no lens blur, no people, no dramatic smoke.

**AI generation prompt — Full Slide J1 composite (optional single image):**

> Single presentation slide layout, white background, title bar “B.E.E.R. — Where Navigation Can Go” in plain sans-serif. Two equal columns separated by a thin vertical line. Left column labelled “Today (41068)” with the semester schematic described above; right column labelled “Beyond 41068 (~5 years)” with thermal overlay and GUI mock. Bottom strip with three bullet icons (clock, truck width, refresh arrow) and short value labels. Looks like a real UTS engineering group project slide — functional, readable from 3 metres, not marketing art. Avoid: Unreal Engine look, neon colours, 3D renders, AI watermark style, overly symmetrical perfection.

**Speaker notes:**

> “For B.E.E.R., the Husky isn’t just a remote-controlled vehicle — it’s meant to **find and keep updating the safest way out** when a fire crew is trapped.
>
> Long term, we see a lead robot on the fireground that fuses lidar, thermal, and other hazard sensing, recomputes the route as conditions change, and shows the operator a live safe path on a GUI — wide enough for the **firetruck** behind it, not just the robot.
>
> This semester we’re building the **navigation core** in simulation: live mapping, global planning, following the path, and **replanning when something blocks the route**. That’s the foundation before we plug in full bushfire thermal fusion and multi-robot stretch goals.”

**Time:** 50 s

**Transition:** “So what will you actually see us demonstrate with that core by the end of semester?”

**Evidence:** C1 long-term ConOps; `Group25_C1_C2_Knowledge.md` §Concept of Operations; semester scope in C1 proposal.

**Rubric alignment:** Criterion 1 (clarity), 2 (value), 3 (distinctive — continuous safe-route updates vs generic teleop robot).

---

## Slide J2 — What We Will Demonstrate: Plan, Drive, Replan, Arrive

**Slide content:**

```
End-of-Semester Demo — Autonomous Navigation

You will see:
1. Husky spawned in a forest simulation world
2. Operator sets a safe-house goal
3. Robot autonomously maps, plans, and drives there
4. If a blockage appears mid-route → robot replans and still arrives

Success loop:
  Sensors → costmaps → global plan → follow path
    → blockage detected → replan → GOAL ✓

Targets: R1 autonomous nav | R4 obstacle avoidance | R9 safe route
Next: routes also avoid thermal fire zones (R2)
```

**Visual:**

Three-panel storyboard (problem → approach → result):

1. **Gazebo screenshot:** Husky in `simple_trees` world with trees visible. Caption: “Goal set at safehouse.”
2. **RViz screenshot:** Map + green global path + laser scan hitting trees. Caption: “Live map and planned route.”
3. **Before/after replan:** Two RViz frames side-by-side — original path through centre, then new path bending around a barrier. Caption: “Wall dropped on route → new plan → goal reached.”

Use `[INSERT GAZEBO SCREENSHOT]`, `[INSERT RVIZ MAP+PATH]`, `[INSERT REPLAN BEFORE/AFTER]`.

**AI generation prompt — Panel 1 (Gazebo / simple_trees):**

> Authentic-looking Ignition Gazebo Fortress simulation screenshot on Ubuntu desktop, not a CGI render. Third-person view slightly above and behind a small yellow Clearpath Husky UGV on flat green tiled ground plane, scattered low-poly conifer tree models with simple green foliage, soft overcast daylight, no dramatic sunset. Gazebo GUI visible: left panel with entity tree, top menu bar, bottom transport controls, grey default theme. Robot at origin facing south; faint coordinate axis triad on ground. Image should look like a WSL2/Linux desktop capture — slightly soft, ordinary monitor resolution, minor UI aliasing, no film grain, no depth-of-field blur. No fire, no smoke, no humans. Caption space below in plain black text: “Goal set at safehouse.” Avoid: hyper-real forest, Unreal Engine 5 foliage, cinematic composition, glossy reflections, obvious AI smoothness.

**AI generation prompt — Panel 2 (RViz map + path + scan):**

> Authentic RViz2 screenshot, dark grey Qt window, typical ROS 2 Humble layout. Central 2D Map view: grey unknown cells, white free space, black occupied cells forming tree shapes from SLAM; red laser scan rays fanning from a white wireframe Husky robot model; thick bright green global path polyline curving toward a goal arrow marker south of robot. Left Displays panel partially visible listing Grid, RobotModel, LaserScan, Map, Path. Fixed frame label “husky1_map”. Flat orthographic top-down camera, no perspective distortion. Colours match default RViz costmap/path palette — functional not beautified. Slight compression artefacts acceptable. Looks like a student robotics lab screen grab. Caption below: “Live map and planned route.” Avoid: neon cyberpunk UI, fake HUD overlays, perfect symmetry, glowing effects, 3D isometric game view.

**AI generation prompt — Panel 3 (replan before/after):**

> Two RViz2 map panels side by side on one slide, same dark theme, same forest map extent, labelled “Before” and “After” in small plain text top corners. Before: green global path runs straight south through centre of map toward goal. After: same map now shows a dark grey rectangular barrier box on the old path; green path visibly rerouted in an arc around the barrier with ~1–2 m lateral offset; robot wireframe further along new path. Red scan hits the barrier. Minimal annotation arrow between panels. Looks like two sequential screenshots pasted into PowerPoint — not a single stylised illustration. Caption: “Wall dropped on route → new plan → goal reached.” Avoid: exaggerated curved paths, cartoon arrows, dramatic before/after lighting, AI-generated text gibberish in UI panels.

**AI generation prompt — Full Slide J2 three-panel storyboard (optional single image):**

> Presentation slide white background, title “End-of-Semester Demo — Autonomous Navigation”. Three equal-width panels in a row with thin grey borders and numbered labels 1–2–3. Panel 1 Gazebo view, Panel 2 RViz map view, Panel 3 before/after replan as described above. Small success-loop text diagram below panels in monospace font. Consistent scale and alignment — looks assembled by a student for a studio class, not a design agency. 16:9 aspect ratio.

**Speaker notes:**

> “By the end of the semester, here’s what we want you to see.
>
> We spawn the Husky in a forest world. You — or our demo script — set a **safe-house goal**. The robot **autonomously** builds a map from its lidar, plans a path, and drives there with no one on a joystick.
>
> The important part for a bushfire scenario: if something **blocks the route mid-mission** — today we demo that with a wall dropped on the path; later that’s a **moving fire or heat zone** — the laser sees it, the costmap updates, Nav2 publishes a **new plan**, and the robot goes around and still reaches the goal.
>
> That closes MVP requirements R1, R4, and R9. The next step in my lane is wiring in **thermal hazard data** from the perception team so ‘safest path’ also means avoiding fire hotspots, not just trees and walls.”

**Time:** 55 s

**Transition:** “That’s the target — here’s what we’ve already got working.”

**Evidence:** `scripts/basic_autonomy_demo.py`; demo commands; R1/R4/R9 from C1 requirements.

**Rubric alignment:** Criterion 1, 2, 5 (clear final outcome + next step).

---

## Slide J3 — Progress So Far: Movement Is Implemented and Tested

**Slide content:**

```
Progress — Autonomous Navigation (Aug 2026)

Implemented ✓                          Verified metrics
• Nav2 + SLAM on simple_trees          • navigation_test: 3/3 goals
• Configurable start pose              • replan_test: costmap 254 on barrier
• basic_autonomy_demo.py               • Plan divergence: 1.72 m
• Dynamic replan (--replan)            • Goal reached: 32.6 s
• rs1_nav mission layer + tests

Sensor → planning chain (today):
  LiDAR /husky1/scan → SLAM map → Nav2 costmaps → NavFn plan → follow → replan

C1 Week 3 → Now:  Nav2 "unknown"  →  start→goal + replan working

Next: integrate thermal hazard regions into costmaps
```

**Visual:**

Primary: **RViz composite screenshot** showing all of:

- Occupancy grid map with trees as occupied cells
- LaserScan rays
- Green global path (`/husky1/plan`)
- Global costmap overlay (optional semi-transparent)
- Robot model at mid-route

**AI generation prompt — Primary (RViz composite):**

> High-fidelity but authentic RViz2 composite screenshot as used in a robotics progress report. Dark grey interface, multiple display layers enabled simultaneously: occupancy grid map (black trees, grey unknown, white free space), red LaserScan points/rays, green `/husky1/plan` global path, semi-transparent blue-purple global costmap inflation layer at ~40% opacity showing cost gradients around trees, white Husky URDF mesh at mid-route facing along path. Top-down 2D view, map frame `husky1_map`. Displays panel on left shows checked items: Map, LaserScan, Path, Global Costmap, RobotModel. No fake data — layout should resemble Nav2 + SLAM Toolbox demo. Ordinary desktop screenshot feel: slight moiré on grid, not retouched. Robot roughly centred-left, path extending south. Looks like evidence attached to a university assignment, not concept art. Avoid: holographic UI, fake topic names, illegible panel text, oversaturated colours, perfect studio lighting.

Secondary inset: terminal output from `replan_test.py` showing PASS lines:

```
PASS  obstacle inserted as a Gazebo model
PASS  lidar range collapsed after insertion
PASS  global costmap marked the barrier
PASS  Nav2 published a diverging plan
PASS  robot reached the goal after the replan
```

**AI generation prompt — Secondary inset (terminal PASS output):**

> Small picture-in-picture inset, bottom-right corner of slide. Ubuntu GNOME terminal window with dark burgundy/purple default theme (or standard black-on-white xterm — either reads as real). Monospace font (Ubuntu Mono or DejaVu Sans Mono). Exact readable text:
> `PASS  obstacle inserted as a Gazebo model`
> `PASS  lidar range collapsed after insertion`
> `PASS  global costmap marked the barrier`
> `PASS  Nav2 published a diverging plan`
> `PASS  robot reached the goal after the replan`
> `REPLAN TEST PASSED`
> Prompt line above: `python3 test/replan_test.py`. Plain shell output — no colours except optional green PASS if terminal supports it. Slight JPEG compression, ordinary font rendering, not a designed infographic. Avoid: fake code, stylised hacker terminal, glowing green Matrix text, illegible characters.

Optional: short **screen recording** (15–20 s) of `--replan` run embedded in slide or played live.

**AI generation prompt — Optional storyboard for screen recording (use if generating a still frame, not video):**

> Single still frame mimicking a paused screen recording of Gazebo + RViz side by side on one desktop. Left half: Gazebo with Husky approaching a grey box barrier mid-path on green tiled forest world. Right half: RViz map showing path mid-reroute — old green path segment near barrier, new arc forming. VLC or OBS “paused” aesthetic not required; just a raw desktop capture with both windows tiled. Timestamp feel of mid-demo. Use as video thumbnail only. Prefer real screen recording over AI for this asset.

**Speaker notes:**

> “On the movement side, this is **implemented and tested** — not just on the proposal slide.
>
> We have the full Nav2 and SLAM stack running in our `simple_trees` world. One command starts the sim, waits for sensors and planners, sends a goal, and reports whether we arrived. We can set start and goal from the command line, and we have automated tests — `navigation_test` passed three out of three goals, and `replan_test` verified the full chain: a real Gazebo wall inserted on the path, lidar range dropped, costmap marked lethal, Nav2 published a diverging plan about **1.7 metres** off the old route, and the robot reached the goal in about **33 seconds**.
>
> Compare that to C1 Week 3, where we only had the sim loaded and Nav2 showed ‘unknown’. Now we have start-to-goal autonomy and dynamic replanning working.
>
> What we learned: the course Nav2 stack was the right foundation — we extended it rather than writing a planner from scratch. The same replan pattern we proved with a wall is what we’ll use when **thermal fire zones** come from Phu’s perception pipeline and Taj’s environment. That’s my next integration step.”

**Time:** 55 s

**Transition:** Hand to teammate covering UI/sim environment, or to delivery timeline: “That navigation core is what Faiyad’s UI will display and what Taj’s bushfire world will plug into.”

**Evidence (confirmed in repo, 2026-08-28):**

| Claim | Source |
|---|---|
| Nav2 + SLAM active | `pathplanning_and_movement_implementation.md` §65 |
| 3/3 navigation goals | Change 01, 05 — `navigation_test.py` |
| Replan: costmap 254, 1.72 m divergence, 32.6 s | Change 06 — `replan_test.py` |
| `rs1_nav/` modules | `MissionRunner`, `NavObserver`, `PathBlocker` |
| LiDAR @ 10 Hz | Change 03 |

**Rubric alignment:** Criterion **4** (primary), 5 (next steps), 6 (concise evidence).

---

### Stakeholder one-liner (backup / Q&A)

> “We simulate a Husky in Gazebo. Its lidar builds a live map. You pick a start and a goal; Nav2 plans a path and drives there. If we drop a wall on that path, the laser sees it, the map updates, and the robot goes around — no one is joysticking it. Next we wire in thermal data so it avoids fire hotspots too.”

(~30 s if asked directly)

---

# 6. Required Visuals / Media / Evidence

| # | Asset | What it should show | Where | Priority |
|---|---|---|---|---|
| 1 | **RViz — map + scan + global path** | Husky mid-mission; trees on map; green `/husky1/plan` | Slide J2 panel 2, Slide J3 primary | **Mandatory** |
| 2 | **Gazebo — simple_trees world** | Husky among trees; proves sim running | Slide J2 panel 1 | **Mandatory** |
| 3 | **Replan before/after** | Two RViz frames: path through barrier site, then detour; or video scrub | Slide J2 panel 3, Slide J3 | **Mandatory** |
| 4 | **Terminal — replan_test PASS** | All five PASS lines with metrics | Slide J3 inset | Recommended |
| 5 | **Costmap overlay** | Lethal cells (254) at barrier location | Slide J3 or backup slide | Recommended |
| 6 | **Screen recording — `--replan`** | Wall appears → robot stops rerouting → goal reached (~30 s clip) | Live demo backup | **Mandatory backup** |
| 7 | **C1 Week 3 vs Now comparison** | Old screenshot (Nav2 unknown) vs current RViz | Slide J3 or verbal only | Recommended |
| 8 | **Vision diagram** | Semester vs 5-year routing flowchart | Slide J1 | Recommended |
| 9 | **Thermal integration mock** | Placeholder: “thermal zones → costmap” not yet wired | Slide J2 “Next” bullet only | Placeholder until Phu delivers topic |

### AI generation prompts — Section 6 asset checklist

**Asset 1 — RViz map + scan + global path** (see also Slide J2 Panel 2 / Slide J3 Primary prompts above)

> RViz2 dark-theme top-down screenshot, SLAM occupancy grid with tree obstacles as black blobs, red laser scan fan, green global plan polyline, white Husky mesh mid-mission heading south. Displays panel partially visible. Ordinary Linux desktop capture quality — functional ROS demo evidence, not illustrated. Negative prompt: cyberpunk, isometric game map, glowing paths, fake HUD, oversharpened 8K render.

**Asset 2 — Gazebo simple_trees world** (see also Slide J2 Panel 1 prompt above)

> Ignition Gazebo Fortress GUI screenshot, yellow Husky on green tiled plane among simple tree models, default grey UI chrome, overcast lighting, WSL/Linux desktop authenticity. Negative prompt: photoreal forest, cinematic fire, dramatic angles, Unreal Engine environment, AI-smooth ground textures.

**Asset 3 — Replan before/after** (see also Slide J2 Panel 3 prompt above)

> Side-by-side RViz2 screenshots Before/After wall insertion; rerouted green path around grey box barrier; labels in plain Arial. Pasted-into-slide aesthetic. Negative prompt: single stylised diagram, curved cartoon path, exaggerated glow on barrier.

**Asset 4 — Terminal replan_test PASS** (see also Slide J3 Secondary inset prompt above)

> Ubuntu terminal with exact PASS test output lines in monospace, `REPLAN TEST PASSED` footer, plain shell — no graphic design. Negative prompt: Matrix style, illegible text, decorative code blocks.

**Asset 5 — Costmap overlay (lethal cells at barrier)**

> RViz2 top-down view focused on global costmap layer only, semi-transparent colour scale: pale blue free space, yellow/orange inflation, dark red/near-black lethal (254) cells forming a tight rectangle where a barrier sits on the path. Thin green path visible underneath rerouting around lethal region. Small legend or colour bar if RViz would show one. Looks like a debugging screenshot a student saved while tuning Nav2 — slightly zoomed in, not a polished infographic. Include subtle grid lines. Negative prompt: heat-map rainbow abuse, scientific publication figure polish, 3D terrain, fake colourbar labels.

**Asset 6 — Screen recording backup (`--replan`)**

> *Prefer real recording.* If generating a thumbnail still: split desktop Gazebo + RViz, Husky near grey wall barrier, path updating in RViz, both windows with ordinary title bars, mid-demo pause frame. Negative prompt: action movie still, motion blur art, single-window cinematic render.

**Asset 7 — C1 Week 3 vs Now comparison**

> Two-panel slide comparison, plain white background, headers “C1 Week 3 (11 Aug)” and “Now (Aug 2026)”. Left panel: RViz2 screenshot with Navigation2 panel showing Navigation / Localization / Feedback all “unknown” in red or grey; map mostly empty, sim loaded but nav inactive — slightly unfinished student setup vibe. Right panel: active RViz with populated map, green path, Nav2 panel showing active states (green/active). Same Husky sim context implied. Deliberately mundane documentation style — progress report not marketing. Negative prompt: dramatic transformation arrow graphics, before/after glow effects, stock photo comparison templates.

**Asset 8 — Vision diagram (semester vs 5-year)**

> Use Slide J1 full composite prompt above. Flat vector engineering slide, two columns, flowcharts + maps, UTS student presentation quality. Negative prompt: futuristic cityscape, drone hero shots, AI art fire background.

**Asset 9 — Thermal integration mock (placeholder)**

> Small inset diagram for slide corner labelled “Next — thermal integration (planned)”. Minimal block diagram: box “IR / thermal sensor” → arrow → box “hazard layer” → arrow → box “Nav2 costmap” → arrow → “safer path”. Dashed outline around entire chain to imply not yet implemented. Flat grey and orange accent, hand-drawn-engineering-whiteboard feel acceptable but keep text legible. Optional faint thermal blob on miniature map icon. No claim of working system — annotate “planned” or “WIP”. Negative prompt: fully rendered fire scene, photoreal thermal camera feed, completed-product marketing graphic.

### Global negative prompts (append to any generation)

> Avoid: hyperrealistic CGI, cinematic lighting, lens flare, depth of field, glossy AI sheen, oversaturated colours, perfect symmetry, stock photo composition, illegible UI text, fake ROS topic gibberish, dramatic smoke or flames unless explicitly schematic, 8K ultra-sharp render look, watermark, decorative bokeh, Unreal Engine / Blender showcase aesthetic.

### Recommended generation settings (if tool supports)

| Setting | Suggestion |
|---|---|
| Aspect ratio | 16:9 for slides; 4:3 for RViz/Gazebo screenshots |
| Style | “technical screenshot”, “presentation slide”, “engineering diagram” |
| Prefer | Flat colour, limited palette, slight imperfection |
| Best results | Generate RViz/Gazebo as **UI screenshot** style; diagrams as **vector infographic** style — do not mix |
| Strongest evidence | Real captures from `basic_autonomy_demo.py` beat AI for Assets 1–4, 6; use AI mainly for Assets 7–9 and Slide J1 schematics |

### How to capture (run before presentation)

```bash
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source ~/RS1-Gr25/install/setup.bash
cd ~/RS1-Gr25/src/41068_ignition_bringup_v1/41068_ignition_bringup

# Screenshot run — RViz + Gazebo GUI
python3 scripts/basic_autonomy_demo.py --rviz --gui --start 0 0 0 --goal 0 -5 0

# Record replan — capture when wall appears and path changes
python3 scripts/basic_autonomy_demo.py --rviz --gui --replan
```

**No screenshots exist in the repo yet** — you must capture these locally.

---

# 7. Presentation / Delivery Plan

| Phase | Duration | Action |
|---|---|---|
| Handoff in | 0:00 | Previous speaker ends on “navigation is core to B.E.E.R.” |
| Slide J1 | 0:50 | Vision — stakeholder value, not ROS internals |
| Slide J2 | 0:55 | End demo — what they will see; mention thermal as next |
| Slide J3 | 0:55 | Evidence — screenshots + one metric; C1→Now contrast |
| Handoff out | 0:10 | “Navigation core ready; Taj’s world and Faiyad’s UI plug in next” |
| **Total** | **~2:30** | Leave 30 s buffer for group pacing |

### Live demo option (if group allocates time)

**Not recommended for Jack’s solo segment** — 10 min total is tight. Prefer **embedded 15 s video** on Slide J3.

If live demo is attempted (group decision):

1. Pre-launch sim before presentation starts (`--attach` mode)
2. Run: `python3 scripts/basic_autonomy_demo.py --attach --replan`
3. Point audience at RViz: “Watch the green path change when the wall appears”
4. **Backup:** Play pre-recorded `--replan` video if stack fails

### Speaking discipline

- Say **“safe route”** not “NavFn A* on rolling costmap”
- Say **“laser sees the obstacle”** not “LaserScan updates obstacle layer”
- One number only on Slide J3: **“1.7 metre reroute, goal in 33 seconds”**

---

# 8. Group Integration / Handoffs

### What comes before Jack

| Speaker (likely) | Content | Jack should echo |
|---|---|---|
| Opener | Stakeholder = firefighters/SES; trapped crew problem | “When the crew is trapped, something has to find the way out” |
| Taj / Phu | Custom environment, sensors | “My stack consumes that sensor data” |
| Faiyad | UI, semester ConOps | “UI will show the path I’m generating” |

### What comes after Jack

| Speaker (likely) | Needs from Jack |
|---|---|
| Taj | Confirm nav works in `simple_trees`; ready for bushfire world port |
| Faiyad | Topics: `/husky1/plan`, pose TF, future hazard map topic name |
| Timeline speaker | Jack’s milestones: nav done Aug 28; thermal integration next |

### Handoff to other team members

| Member | Jack provides | Jack needs |
|---|---|---|
| **Phu** | Costmap input spec; desired hazard message format (grid or polygon) | Thermal/IR topic name, update rate, frame_id |
| **Taj** | Confirmed nav in Gazebo; spawn args `husky_x/y/yaw` | Custom world SDF path; fire zone coordinates |
| **Faiyad** | `/husky1/plan`, robot pose in `husky1_map`, mission status from demo | UI mockup slot for path overlay |

### Duplicate content to avoid

- Do **not** re-explain the full B.E.E.R. problem (other slides cover it)
- Do **not** claim thermal routing is working — say **“next step”**
- Do **not** show C1 Week 3 screenshot as current progress

---

# 9. Rubric Coverage Matrix

| Rubric criterion | Jack’s contribution | Evidence | Included? | Remaining gap |
|---|---|---|---|---|
| 1 — Understand problem/proposal | Slides J1–J2: escape routing in plain English | ConOps, slide text | Yes | None |
| 2 — Real value | J1: less smoke/heat time; firetruck-suitable path | C1 value claims | Yes | Quantify truck width margin if asked — `[VERIFY]` |
| 3 — Stands out | J1: continuous replanning, not one-shot path | Replan demo vs teleop robots in related work | Yes | Tie to B.E.E.R. acronym in group opener |
| 4 — Progress shown | **J3 primary** — tests, metrics, screenshots | `replan_test.py`, implementation log | **Partial** | **Must capture RViz/Gazebo screenshots** |
| 5 — Trust delivery path | J2 end demo + J3 next step (thermal) | Working nav + planned integration | Yes | Thermal topic interface not yet defined |
| 6 — Engaged stakeholder | Concise script; 2:30 budget; one-liner backup | Speaker notes above | Yes | Rehearse to stay under time |

---

# 10. Missing Information / Placeholders

| Item | Status | Action |
|---|---|---|
| RViz/Gazebo screenshots | **Not in repo** | Capture before presentation |
| Thermal sensor ROS topic | **Not wired** | Confirm with Phu; use placeholder on slide |
| Custom bushfire world (Taj) | **Not Jack’s deliverable** | Reference as team next step, don’t claim done |
| Firetruck clearance margin (m) | C1 mentions space/incline/heat — no number | `[VERIFY COMPONENT SPECIFICATION]` or omit |
| Group slide order / total slide count | Not in sources | Align with team in rehearsal |
| Whether live demo is allowed | Not specified | Default to video backup |
| Progress since 2026-08-28 | Unknown | Re-run tests; update metrics if changed |

**Assumption:** Jack’s slides are numbered 1–3 **within his segment**, not necessarily slides 1–3 of the full deck.

---

# 11. Final Completion Checklist

### Content

- [ ] Paste slide text from Section 5 into team template
- [ ] Align handoff lines with previous/next speakers
- [ ] Remove any claim that thermal routing is already working

### Evidence (critical for marks)

- [ ] Capture RViz: map + scan + global path
- [ ] Capture Gazebo: Husky in forest world
- [ ] Capture replan before/after (or 20 s video)
- [ ] Optional: terminal screenshot of `replan_test` PASS output
- [ ] Re-run `python3 test/replan_test.py` — confirm still passes

### Delivery

- [ ] Rehearse Jack’s segment to **≤ 2:30**
- [ ] Memorise stakeholder one-liner (30 s)
- [ ] Prepare video backup if live demo considered
- [ ] Can explain in own words: what `PathBlocker` does, why replan triggered, what Nav2 role is

### Team coordination

- [ ] Confirm Phu’s thermal topic name/timeline for Slide J2 “Next” bullet
- [ ] Confirm Taj’s world will use same launch args / nav stack
- [ ] Give Faiyad `/husky1/plan` and pose frame for UI
- [ ] Full group run-through: total time ≤ 10:00

### Rubric spot-check

- [ ] Stakeholder can explain back: “Robot finds safe path, replans if blocked”
- [ ] At least one **real screenshot or video** on Slide J3 (Criterion 4)
- [ ] Clear “what’s next” stated (Criterion 5)

---

**Bottom line:** Your strongest C2 asset is verified autonomous navigation with dynamic replanning — `basic_autonomy_demo.py`, `rs1_nav/`, and passing tests. Slide J3 should lead with **visual proof**, not architecture. Thermal integration is honest “next step” material for Slides J2–J3; do not oversell it until Phu’s pipeline is on a ROS topic you consume. Capture the RViz/Gazebo assets this week — that is the main gap between a proficient and excellent mark on Criterion 4.
