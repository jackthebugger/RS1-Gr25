#!/usr/bin/env python3
"""Generate LLM-optimised Markdown from extracted PDF data."""

import json
import os
import re
from datetime import datetime

PDF_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORK_DIR = os.path.dirname(os.path.abspath(__file__))

DOC_CONFIG = {
    "decision_making": {
        "title": "Decision Making",
        "subject": "Robot decision making — TSP, exploration, informative path planning, MCTS, behaviour trees",
        "date": "2025-08-20",
    },
    "path_planning": {
        "title": "Path Planning",
        "subject": "Robot path planning — potential fields, graph search, sampling-based planning, Nav2",
        "date": "2026-08-27",
    },
    "perception_mapping": {
        "title": "Perception and Mapping",
        "subject": "Robot perception, sensors, map representations, occupancy grids, SLAM",
        "date": "2026-08-13",
    },
    "simulation_package": {
        "title": "Simulation Package Overview",
        "subject": "41068 ROS 2 / Gazebo simulation starter package structure and usage",
        "date": "2026-08-06",
    },
}

# Curated visual semantics for image-heavy slides (key, page) -> description
VISUAL_SEMANTICS = {
    ("decision_making", 2): """**Type:** Photo collage / motivation slide

**Visible content:**
- RGB-D scene collected using robot
- Multiple robotics application photographs showing field robots, drones, and exploration scenarios
- Visual motivation for decision-making topics in robotics""",

    ("decision_making", 3): """**Type:** Concept diagram

**Elements:**
- Central equation: $\\text{action} = f(\\text{state})$
- **State** examples: location, map
- **Action** example: "move over there"
- **Planner** block connects state to action

**Relationship:** The planner is a function mapping robot state to the next action.""",

    ("decision_making", 4): """**Type:** Coupled sensing-planning diagram

**Flow:**
1. **Sensing:** Make an observation at this location
2. **Planning:** Move to a new location
3. Cycle repeats — sensing and planning are coupled

**Key concept:** Robot alternates between observing and moving to new locations.""",

    ("decision_making", 5): """**Type:** Section divider

**Label:** Mapping (transition slide between hierarchy overview and definitions)""",

    ("decision_making", 6): """**Type:** System architecture block diagram

**Components (left to right, top to bottom):**

| Component | Inputs | Outputs |
|---|---|---|
| **Map Processor** | SLAM output, other robots' maps | maps, stuck signal; communicates to other robots |
| **Behavior Executive** | stuck, conditions from planners | actions to Global Planner and Local Planner |
| **Global Planner** | maps, actions | path; conditions feedback |
| **Local Planner** | maps, actions, path | trajectory; conditions feedback |
| **Controller** | trajectory | motor commands |

**Visual thumbnails:** Each block shows representative imagery (occupancy map, behaviour tree, global path, local trajectory in 3D environment, physical robot).""",

    ("decision_making", 7): """**Type:** Annotated hierarchy diagram (progression slide 1)

**Annotation:** Map Processor highlighted — labelled **"Previous seminar"**

Same architecture as Slide 6; emphasis on Map Processor as content covered in an earlier seminar.""",

    ("decision_making", 8): """**Type:** Annotated hierarchy diagram (progression slide 2)

**Annotations:**
- Map Processor: **Previous seminar** (blue)
- Behavior Executive: **Today** — "Where do I go next?" (green)
- Global Planner: **Today** (green)

**Focus:** Current seminar covers high-level decision making (behaviour executive) and global planning.""",

    ("decision_making", 9): """**Type:** Annotated hierarchy diagram (progression slide 3)

**Annotations:**
- Map Processor: **Previous seminar**
- Behavior Executive: **Today** — "Where do I go next?"
- Global Planner + Local Planner: **Today** — "How do I get there?"
- Controller: **Future seminar**

**Focus:** Path planning (global + local) is the current topic; low-level control deferred.""",

    ("decision_making", 14): """**Type:** Section divider — Travelling Salesman Problem""",

    ("decision_making", 15): """**Type:** Vehicle routing application photographs

**Examples shown:**
- Mars rover mission planning
- Region of Interest / Extended Mission planning
- Offshore oil rig inspection with robot paths
- Letters / delivery routing
- Orange center inspection regions

**Purpose:** Illustrates real-world variants of routing problems beyond classic TSP.""",

    ("decision_making", 27): """**Type:** Animation / simulation sequence

**Content:** Frontier-based exploration animation showing robot progressively mapping unknown environment by visiting frontier cells at the boundary between known free space and unknown space.

**Stages:** Map expands over time as robot visits frontiers.""",

    ("decision_making", 35): """**Type:** Dual heatmap figure [Hollinger, 2014]

**Left panel:** Estimate of wireless signal strength (spatial field)
**Right panel:** Uncertainty of estimate (higher uncertainty in unobserved regions)

**Purpose:** Illustrates informative path planning — robot should visit high-uncertainty regions.""",

    ("decision_making", 38): """**Type:** Section divider — Monte Carlo Tree Search""",

    ("decision_making", 39): """**Type:** Application photograph

**Content:** MCTS applied to robot information gathering — RGB-D scene collected using robot in structured indoor environment.""",

    ("decision_making", 40): """**Type:** Tree search diagram

**Elements:**
- **Current state** (root)
- **Future actions** (tree branches)
- **Green nodes** = high reward paths
- **Goal:** Find the best action sequence""",

    ("decision_making", 42): """**Type:** MCTS behaviour plot

**X-axis:** exploration ↔ exploitation balance
**Y-axis:** expected reward (high to low)

**Shows:** MCTS balances exploring new branches vs exploiting known high-reward paths.""",

    ("decision_making", 43): """**Type:** MCTS algorithm flowchart

**Pipeline:** Selection → Expansion → Simulation → Backpropagation

**Sub-components:**
- **Tree Policy** (selection/expansion)
- **Default Policy** (simulation/rollout)

**Cycle repeats** to grow search tree and improve action estimates.""",

    ("decision_making", 45): """**Type:** MCTS in action — tree growth visualisation

**Shows:** Search tree expanding over a spatial domain with varying node visit counts and reward estimates.""",

    ("decision_making", 49): """**Type:** Dec-MCTS multi-robot diagram

**(a)** Grow search tree for own actions — performed asynchronously by each robot
**(b)** Decentralised optimisation of probability distributions
**(c)** Communicate distributions with other robots

**Key concept:** Each robot runs MCTS locally and coordinates via communicated probability distributions.""",

    ("decision_making", 50): """**Type:** Section divider — Behaviour Trees""",

    ("decision_making", 51): """**Type:** Multi-task mission photographs

**Context:** DARPA Subterranean Challenge

**Tasks shown:**
- Exploring area
- Avoid hazard
- Drilling
- Scan rocks

**Purpose:** Real missions require switching between multiple concurrent tasks.""",

    ("decision_making", 53): """**Type:** Example behaviour tree + timeline

**Behaviour tree structure (top):**
- Root: Fallback → Sequence
  - Diagnostics
  - Take off → Exploration → Landing
  - Return / Landing / Shutdown branches
  - Coordinated Explore / Roadmap Explore with conditions (Has Unvisited Frontiers, Stuck, Critical Battery, Unreachable Goal)
  - Emergency Land, Return Home, Rewind actions

**Timeline (bottom):** Mission execution over 0–900 seconds showing state transitions.""",

    ("path_planning", 17): """**Type:** Method comparison diagram

**Subtitle:** Control, path finding, trajectory optimization

**Combining methods:**
1. Path Finding
2. Trajectory Optimization ("smoothing")
3. Feedback Control

**Visual paths around U-shaped obstacle:**
- **Blue (Feedback Control):** Straight line into obstacle — fails
- **Magenta (Path Finding):** Jagged path over obstacle — reaches goal
- **Green (Trajectory Optimization):** Smooth curve — optimal execution

**Key points:**
- Many problems solvable with only feedback control (not optimally)
- More problems locally optimal with trajectory optimization alone
- Tricky problems need path finding: global search for valid paths""",

    ("path_planning", 20): """**Type:** Potential field visualisation

**Three panels:**
1. **Attractive Potential** for goals — bowl-shaped field centred on goal
2. **Repulsive Potential** for obstacles — peaks at obstacle locations
3. **Combined Potential Field** — sum of attractive and repulsive: $U(q) = U_{att}(q) + U_{rep}(q)$

**Robot follows gradient downhill** toward goal while avoiding obstacles.""",

    ("path_planning", 21): """**Type:** Local minima demonstration

**Shows:** Robot trapped in local minimum of combined potential field — surrounded by obstacles with no downhill path to goal.

**Label:** "Move Downhill" strategy fails when stuck between obstacle peaks.""",

    ("path_planning", 25): """**Type:** Graph theory definitions

**Graph:** $G = (V, E)$ where $V$ = vertices, $E$ = edges; edge $e = (v, w)$

**Example:** $V = \\{0,1,2,3,4\\}$, $E = \\{(0,1),(0,2),(0,4),(2,3),(3,4),(4,0)\\}$

**Edge types:** Directed or undirected; may have weights

**Diagrams:** Directed unweighted vs directed weighted graph examples""",

    ("path_planning", 31): """**Type:** Wavefront algorithm — step 1

**Grid** with obstacles (shaded), **S** = start, **G** = goal

**Legend:** S = start, G = goal""",

    ("path_planning", 32): """**Type:** Wavefront algorithm — step 2

**Action:** Build the wave starting from goal

**Wavefront values** propagate outward from G; each cell gets distance-to-goal value incrementing by 1 per step (8-connected or 4-connected grid).""",

    ("path_planning", 33): """**Type:** Wavefront algorithm — step 3

**Action:** Get the path in reverse order (follow steepest descent from S toward decreasing wave values)

**Result:** Optimal grid path from S to G avoiding obstacles.""",

    ("path_planning", 48): """**Type:** RRT algorithm — initial tree

**Shows:** Start node with empty/small tree after a few iterations in configuration space with obstacles.""",

    ("path_planning", 49): """**Type:** RRT — random sample step

**Steps shown:**
1. Draw a random sample (random point in C-space)
2. Start node marked""",

    ("path_planning", 50): """**Type:** RRT — nearest node step

**Steps:**
1. Find nearest node in tree to random point
2. nearest point and random point labelled""",

    ("path_planning", 51): """**Type:** RRT — steer step

**Action:** "Steer" from nearest node toward random point (by step size $\\Delta q$)""",

    ("path_planning", 52): """**Type:** RRT — insert node

**Action:** Insert new node into tree (if collision-free)""",

    ("path_planning", 53): """**Type:** RRT — repeat

**Shows:** Full iteration cycle; label "repeat!" — algorithm loops until goal reached or timeout.""",

    ("path_planning", 54): """**Type:** RRT — converged tree

**Shows:** Dense tree after many "repeat!" iterations exploring configuration space.""",

    ("path_planning", 56): """**Type:** RRT vs RRT* comparison plots

**Two side-by-side path plots** in 2D configuration space (-10 to 10 on both axes)

**RRT:** Jagged, suboptimal path
**RRT*:** Smoother, shorter path after rewiring step

**Key difference:** RRT* adds a "rewiring" step to asymptotically converge to optimal paths.""",

    ("path_planning", 61): """**Type:** Screenshot — 41068 package with Nav2

**Windows shown:**
- **Gazebo:** Forest environment with robot navigating
- **RViz:** Navigation 2 panel showing Navigation active, Localization inactive, Feedback active; distance remaining ~17.67 m; 2D Nav Goal tool

**Config path visible:** `41068.rviz`""",

    ("path_planning", 65): """**Type:** Nav2 architecture diagram

**Components (all plugins swappable/customisable):**
- BT Navigator Server
- Controller Server, Planner Server, Behavior Server, Smoother Server
- Waypoint Follower
- Global Costmap, Local Costmap
- Route Server
- Velocity Smoother, Collision Monitor
- Robot Base
- Sensor data feeds into costmaps

**Key message:** All Nav2 plugins can be swapped and customised.""",

    ("perception_mapping", 2): """**Type:** Application photographs

**Three perception tasks:**
1. **Fruit tree modelling** — agricultural robot among trees
2. **Ocean monitoring** — marine/aerial monitoring
3. **Subterranean mapping** — cave/tunnel exploration robot""",

    ("perception_mapping", 10): """**Type:** Autonomous car sensor diagram [machinedesign.com]

**Sensors labelled:**
- Velodyne multi-layer laser scanner
- GPS antenna, INS solution
- Steering actuator, brake actuator, gear shift actuator
- Cameras (multiple)
- Radar
- 2D laser scanner
- DMI (Distance Measurement Instrument)
- APS signal

**Purpose:** Shows complexity of sensor suites on autonomous vehicles.""",

    ("perception_mapping", 11): """**Type:** Mobile robot sensor layout diagram

**Scale:** 100 m robot shown

**Sensors labelled:**
- Ethernet Antenna
- Emergency Stop
- Console
- Door Arc
- Sonar sensors (array along base)
- IR sensors
- Enclosure
- Base / Wheels

**Purpose:** Typical indoor mobile robot sensor configuration.""",

    ("perception_mapping", 31): """**Type:** Problem formulation

**Problem:** Estimate map from sensor data

Given sensor data $z_{1:t}$ and poses $x_{1:t}$ of the sensor, estimate:
$$p(m \\mid z_{1:t}, x_{1:t}) = \\prod_i p(m_i \\mid z_{1:t}, x_{1:t})$$

Each cell $m_i$ is a **binary random variable** → **Binary Bayes filter** (for a static state)""",

    ("perception_mapping", 36): """**Type:** Recursive Bayes filter flowchart

**Pipeline:**
1. Prior belief (occupancy grid)
2. Measurement update (incorporate sensor observation)
3. Posterior belief (updated grid)

**Repeats** as robot moves and collects new observations.""",

    ("simulation_package", 8): """**Type:** ROS system architecture diagram (from slide 8 native text)

**Node/topic graph showing:**
- Physical/Simulated Robot
- Sensor drivers: Lidar, GPS, Camera
- Topics: scan, Image, Motor commands
- Processing nodes: Mapping, Obstacle planning, Object detection, Controller
- Outputs: map, Path, Waypoint, Goal, Object location
- User Interface
- Modules: Sensing, Perception, Planning, Control, Decision Making""",

    ("simulation_package", 12): """**Type:** Expected simulation screenshot

**Three windows:**

1. **Gazebo simulator (left):** Forest world, 2 robots (Husky UGV visible), dynamic objects (beige sphere on ground)

2. **RViz Visualisation: UGV (top right):**
   - Basic autonomy: 2D occupancy map with explored area (white), obstacles (black/red), robot pose arrow
   - Cameras: first-person forest view
   - Config: `41068_husky1.rviz`

3. **RViz Visualisation: UAV (bottom right):**
   - Similar map view from aerial perspective
   - Camera shows UGV and dynamic object from above
   - ROS Time ~173.78, ~27 fps""",

    ("simulation_package", 28): """**Type:** Student project examples from 41068 (2025)

**Shows:** Screenshots/photos of diverse student project adaptations of the simulation package — varied environments, robot configurations, and autonomy implementations.

[Specific project details not legible at extraction resolution]""",
}


def parse_bullets(body_lines):
    """Convert body lines to markdown bullet list."""
    items = []
    i = 0
    while i < len(body_lines):
        line = body_lines[i].strip()
        if not line or re.match(r"^\d+$", line):
            i += 1
            continue
        if line in ("➢", "•", "▪"):
            i += 1
            if i < len(body_lines):
                items.append(body_lines[i].strip())
                i += 1
            continue
        if re.match(r"^\d+\.$", line):
            num = line.rstrip(".")
            i += 1
            parts = []
            while i < len(body_lines):
                nl = body_lines[i].strip()
                if nl in ("➢", "•") or re.match(r"^\d+\.$", nl):
                    break
                if re.match(r"^[a-d]\)$", nl):
                    break
                if re.match(r"^\d+$", nl) and i == len(body_lines) - 1:
                    break
                parts.append(nl)
                i += 1
            items.append(f"{num}. {' '.join(parts)}")
            continue
        if re.match(r"^[a-d]\)$", line):
            letter = line
            i += 1
            parts = []
            while i < len(body_lines):
                nl = body_lines[i].strip()
                if nl in ("➢", "•") or re.match(r"^\d+\.$", nl) or re.match(r"^[a-d]\)$", nl):
                    break
                parts.append(nl)
                i += 1
            items.append(f"   - **{letter}** {' '.join(parts)}")
            continue
        items.append(line)
        i += 1
    return items


def format_native_text(text):
    """Clean and format native slide text."""
    lines = [l.strip() for l in text.split("\n") if l.strip()]
    # Remove footer lines
    filtered = []
    for l in lines:
        if l == "41068 Robotics Studio 1":
            continue
        if re.match(r"^\d{1,2}$", l) and len(lines) > 3:
            continue
        filtered.append(l)
    return filtered


def generate_markdown(stem):
    json_path = os.path.join(WORK_DIR, f"{stem}_extracted.json")
    with open(json_path, encoding="utf-8") as f:
        data = json.load(f)

    cfg = DOC_CONFIG[stem]
    meta = data["metadata"]
    slides = data["slides"]

    lines = []
    lines.append(f"# {cfg['title']}")
    lines.append("")
    lines.append("## Document Overview")
    lines.append("")
    lines.append(f"- **Document type:** PowerPoint seminar slides (PDF export)")
    lines.append(f"- **Author:** {meta.get('author', 'Graeme Best')}")
    lines.append(f"- **Organisation:** University of Technology Sydney — Faculty of Engineering and Information Technology")
    lines.append(f"- **Course:** 41068 Robotics Studio 1")
    lines.append(f"- **Creation date:** {cfg['date']}")
    lines.append(f"- **Pages / slides:** {data['page_count']}")
    lines.append(f"- **Primary subject:** {cfg['subject']}")
    lines.append(f"- **Source file:** `{data['source_file']}`")
    lines.append("")

    # Build contents from slide titles
    lines.append("## Contents")
    lines.append("")
    section_slides = []
    for s in slides:
        title = s["title"] or None
        native = format_native_text(s["all_text_native"])
        if not title and native:
            # Section dividers often have only a title line
            candidates = [l for l in native if l not in ("41068 Robotics Studio 1",) and not re.match(r"^\d+$", l)]
            if len(candidates) == 1:
                title = candidates[0]
        if title and title not in ("Decision Making", "Path Planning", "Perception and Mapping", "Simulation Package"):
            section_slides.append((s["page"], title))
    for page, title in section_slides:
        anchor = re.sub(r"[^a-z0-9]+", "-", title.lower()).strip("-")
        lines.append(f"- [Slide {page} — {title}](#slide-{page})")
    lines.append("")

    # Main content
    lines.append("## Main Content")
    lines.append("")

    visual_count = 0
    text_count = 0

    for s in slides:
        page = s["page"]
        title = s["title"]
        native = format_native_text(s["all_text_native"])

        if not title and native:
            candidates = [l for l in native if not re.match(r"^\d+$", l)]
            if len(candidates) == 1:
                title = candidates[0]

        heading = title if title else f"Slide {page}"
        lines.append(f"### Slide {page} — {heading} {{#slide-{page}}}")
        lines.append("")
        lines.append(f"> **Source:** PDF p. {page}")
        lines.append("")

        # Native text content
        bullets = parse_bullets(s["body_lines"])
        if bullets:
            # Check if structured as bullets or prose
            bullet_items = [b for b in bullets if b.startswith(("1.", "2.", "3.", "4.", "5.", "6.", "7.", "8.", "9.")) or b.startswith("   -")]
            plain_items = [b for b in bullets if b not in bullet_items]

            if plain_items:
                for item in plain_items:
                    if item.startswith("http"):
                        lines.append(f"- {item}")
                    elif item in ("Problem:", "Path Planning:", "Advantages:", "Disadvantages:", "Before", "After"):
                        lines.append(f"**{item.rstrip(':')}:**")
                    else:
                        lines.append(f"- {item}")
                lines.append("")

            if bullet_items:
                for item in bullet_items:
                    lines.append(f"- {item.lstrip('- ')}")
                lines.append("")

        # Visual semantics
        vkey = (stem, page)
        if vkey in VISUAL_SEMANTICS:
            lines.append("#### Visual Content")
            lines.append("")
            lines.append(VISUAL_SEMANTICS[vkey])
            lines.append("")
            visual_count += 1
        elif s["slide_type"] in ("visual_heavy", "mixed") and s["image_count"] > 0:
            lines.append("#### Visual Content")
            lines.append("")
            lines.append(f"**Type:** Image-heavy slide ({s['image_count']} embedded images)")
            lines.append("")
            # Use OCR if it has meaningful content
            ocr = s.get("ocr_text", "")
            ocr_clean = "\n".join(
                l for l in ocr.split("\n")
                if l.strip() and not re.match(r"^[e@°Oo©\.\s:]+$", l.strip())
                and "41068" not in l
            )
            if len(ocr_clean) > 30:
                lines.append("**OCR-extracted labels/text:**")
                lines.append("")
                lines.append("```")
                lines.append(ocr_clean[:1500])
                lines.append("```")
                lines.append("")
            else:
                lines.append("[Visual information primarily conveyed through images; see source PDF p. {}]".format(page))
                lines.append("")
            visual_count += 1
        else:
            text_count += 1

        lines.append("---")
        lines.append("")

    # Extraction metadata
    lines.append("## Extraction Metadata")
    lines.append("")
    lines.append(f"- **Source:** `{data['source_file']}`")
    lines.append(f"- **Pages processed:** {data['page_count']} / {data['page_count']}")
    lines.append(f"- **Extraction date:** {data.get('extraction_date', datetime.now().isoformat())}")
    lines.append(f"- **Slides with curated visual semantics:** {sum(1 for k in VISUAL_SEMANTICS if k[0] == stem)}")
    lines.append(f"- **Slides with OCR/visual fallback:** {visual_count}")
    lines.append(f"- **Text-primary slides:** {text_count}")
    lines.append(f"- **OCR engine:** Tesseract 4.1.1")
    lines.append(f"- **Native text extraction:** PyMuPDF (fitz)")
    lines.append(f"- **Document creator:** {meta.get('creator', 'Microsoft PowerPoint')}")
    lines.append("")

    lines.append("## Extraction Uncertainties")
    lines.append("")
    lines.append("1. Some image-heavy slides rely on curated visual descriptions or OCR; fine diagram details may require referring to the source PDF.")
    lines.append("2. Mathematical notation in slides uses Unicode italics from PowerPoint; LaTeX equivalents are provided where reconstructed.")
    lines.append("3. Photo collage slides (e.g. motivation/examples) contain information not fully transcribed at label level.")
    lines.append("4. Animation slides (e.g. frontier exploration) are represented as sequential descriptions, not frame-by-frame data.")
    lines.append("")

    return "\n".join(lines)


def main():
    for stem in DOC_CONFIG:
        md = generate_markdown(stem)
        out_path = os.path.join(PDF_DIR, f"{stem}_llm_optimised.md")
        with open(out_path, "w", encoding="utf-8") as f:
            f.write(md)
        print(f"Generated: {out_path} ({len(md):,} chars)")


if __name__ == "__main__":
    main()
