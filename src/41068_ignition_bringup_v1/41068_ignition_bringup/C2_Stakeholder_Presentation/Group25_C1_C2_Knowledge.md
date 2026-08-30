# Group 25 — B.E.E.R. Knowledge Base (C1 Proposal + C2 Presentation)

## Overview

- **Project:** Bushfire Environmental Evacuation & Rescue
- **Canonical acronym:** `B.E.E.R.`
- **Course:** `41068` Robotics Studio 1
- **Group:** 25
- **Team:** Phu Huynh, Taj Wilcock, Faiyad Hassan, Jack Havranek
- **Organisation:** University of Technology Sydney (UTS) — inferred from `@student.uts.edu.au` emails; not stated as a standalone field in the C1 PDF
- **Primary platform:** Husky ground robot (provided simulation model)
- **Operating domain for 41068:** Entirely in simulation
- **Purpose of this document:** Single retrieval-oriented source of truth combining the C1 proposal, the C2 assessment brief, C2 team contributions, and Jack Havranek’s assigned speaker slides (movement / path planning)
- **C1 purpose:** Propose project direction, system design, planning, communication approach, and evidence for a forest-management / environmental-monitoring robotics studio project
- **C2 purpose:** Stakeholder-facing presentation that gives the defined stakeholder confidence in need-understanding, solution value, feasibility, and delivery path

## Key Facts

| Category | Fact |
|---|---|
| Project title | Bushfire Environmental Evacuation & Rescue (`B.E.E.R.`) |
| Chosen application feasibility (C1 table) | HIGH |
| Intended users | Firefighters, SES |
| Semester platform | Provided Husky in simulation |
| Stretch aerial platform | UAV / Parrot (models need not be changed; drone is later-stage) |
| Middleware | ROS 2 |
| Simulator | Ignition Gazebo (risks table also names Gazebo/Isaac) |
| Visualisation | RViz2 (C1 diagram text writes `RVis2`) |
| MVP navigation style | Local / reactive hazard avoidance, not full dynamic global route optimisation |
| Stretch environment feature | Dynamic fire spread; vegetation states healthy / burning / burnt |
| C1 due date (Gantt) | Friday 28 August (milestone diamond) |
| C2 delivery | Week 6 Studio class; 10 minutes maximum; all members in person |
| C2 marks | 18 points (6 criteria × 3) |
| Named stakeholder contact in log | Nadim (tutor; teaching staff acting as stakeholder) |
| Repository shown in C1 evidence | `jackthebugger / RS1-Gr25` (Public); description “RS1 Forest Management Project” |
| Latest commit shown in C1 Figure 3 | `9d8c8df` — “Adding sim pakage to repo” — author `TajWilcockUTS` |
| Progress snapshot date | Week 3 (`11/08/2026`) |
| Gantt “today” marker in Figure 1 | Tuesday 18 August |
| Jack Havranek C2 focus | Robot movement and path planning; sensor-driven safe routing to goal, including dynamic replanning |
| Navigation stack (repo, 2026-08-28) | Nav2 + SLAM Toolbox + `basic_autonomy_demo.py` + `rs1_nav/` — start→goal and `--replan` verified in `simple_trees` |
| Thermal in path planning | **Planned / in progress** — integrate groupmates’ thermal sensor data with provided sensors for fire-hazard-aware routing; not present in `rs1_nav/` or costmap config at evidence date |

## Terminology

| Canonical term | Aliases in sources | Notes |
|---|---|---|
| `B.E.E.R.` | BEER; Bush-Fire Environmental Evacuation & Rescue; BEER) | Same project. `BEER)` is a source typo in the stakeholder log. Canonical form is `B.E.E.R.` |
| Husky | husky; robot husky; onboard robot husky | Primary ground platform |
| SES | NSW SES (long-term ConOps) | Intended user / operator organisation |
| ROS 2 | ROS2 | Same middleware |
| RViz2 | RViz; `RVis2` | C1 diagram narrative uses `RVis2`. Figure 2 screenshot shows RViz2. |
| Ignition Gazebo | Gazebo; Gazebo/Isaac | Component plan: Ignition Gazebo. Risk table: Gazebo/Isaac. |
| Safe location | safe house; safehouse; fire trail; safety point; end goal | Semester goal location. Stakeholder feedback selected one designated safehouse over a variable fire-trail goal. |
| Operator UI | GUI; user interface; operator user interface | Human supervisor interface |
| Nav2 | ROS2 Nav2 | Named in skill-gap risk |
| Parrot | parrot | Aerial model mentioned in stakeholder Q&A; need not be changed |
| Chat GPT | genAI; generative AI | Tool named in C1 AI declaration |

**Acronyms defined in sources**

- **B.E.E.R.** — Bushfire Environmental Evacuation & Rescue
- **SES** — expansion not stated in sources (used as organisation name)
- **IR** — used as heat / infrared sensing; expansion not spelled out as a heading
- **LiDAR** — used as sensor type; expansion not spelled out
- **UAV** — used in R10 verification text; expansion not spelled out
- **GUI** — used as operator interface; expansion not spelled out
- **MVP** — used as requirement priority versus Stretch
- **CO** — CO Sensor on system diagram; expansion not spelled out
- **CAD** — used with SolidWorks
- **C1 / C2** — Component 1 (proposal) / Component 2 (stakeholder presentation)

---

## Contents

1. [C2 Assessment](#c2-assessment)
2. [C2 Team Contributions](#c2-team-contributions)
3. [Assigned Speaker Slides — Jack Havranek](#assigned-speaker-slides-jack-havranek)
4. [Team](#team)
5. [Entities](#entities)
6. [Problem and Value](#problem-and-value)
7. [Decisions](#decisions)
8. [Concept of Operations](#concept-of-operations)
9. [Requirements](#requirements)
10. [Constraints, Assumptions, Scope](#constraints-assumptions-scope)
11. [Architecture](#architecture)
12. [Components](#components)
13. [Simulation and Evaluation Environment](#simulation-and-evaluation-environment)
14. [Related Work](#related-work)
15. [Current State](#current-state)
16. [Movement and Path Planning (Jack)](#movement-and-path-planning-jack)
17. [Roles, Workflow, and Delivery Path](#roles-workflow-and-delivery-path)
18. [Risks](#risks)
19. [Timeline](#timeline)
20. [Communication](#communication)
21. [Generative AI](#generative-ai)
22. [Open Issues and Conflicts](#open-issues-and-conflicts)
23. [Visual Assets](#visual-assets)
24. [Source Provenance](#source-provenance)

---

## C2 Assessment

**Source:** `Presentation_Task.md`  
**Assessment name:** Component 2: Stakeholder Engagement Presentation

### Purpose

Give the stakeholder confidence that the team:

- understands stakeholder needs
- has developed a meaningful robotics solution
- is on a credible path toward a useful final outcome

### Assessment stance

- Tutor assesses **from the perspective of the stakeholder the team defines**, not primarily as a technical expert
- Presentation is **stakeholder-facing**
- Must still include enough technical substance, progress, and evidence to show the proposed system is feasible and that meaningful development is underway

### Required communications

Using the provided presentation template, communicate:

1. Who the stakeholder is
2. The problem and how it is currently addressed
3. The broader vision for the project
4. The proposed semester solution
5. The key technical challenges and the proposed approach
6. How the system will work for the stakeholder
7. Evidence of progress so far
8. What the team intends to demonstrate by the end of the semester
9. Path to delivery, including key milestones and risks

### Delivery constraints

| Constraint | Definition |
|---|---|
| Event | In-class presentation during the Week 6 Studio class |
| Time limit | 10 minutes maximum |
| Overtime | Going over time will be penalised |
| Attendance | All team members must be present in person |
| Contribution | All team members must make a meaningful contribution |
| Template | Use the provided presentation template |

### Rubric — Stakeholder Outcomes (“How I feel as the client…”)

Total points: **18**

| # | Outcome | 3 — Excellent | 2 — Proficient | 1 — Marginal | 0 — Not Evident |
|---|---|---|---|---|---|
| 1 | I clearly understand the problem and what you're proposing. | Crystal clear in plain English; I can explain it back. | Mostly clear; a few minor ambiguities. | Vague or jargony; key details unclear. | I don't understand the project. |
| 2 | I believe this will deliver real value for me. | Strong relevance; clear and compelling benefits. | Relevant; benefits are reasonably clear. | Weak link to my needs; benefits unclear. | No convincing value. |
| 3 | This project stands out to me. | Creative, distinctive and memorable; goes beyond the obvious. | Thoughtful, with some distinctive elements. | Mostly conventional or generic. | No clear originality or project identity. |
| 4 | I'm convinced by the progress shown. | Meaningful progress; something real is working or has been tested, with convincing evidence. | Credible evidence; implementation or testing is underway. | Mostly claims, basic setup, or limited evidence. | No credible evidence of progress. |
| 5 | I trust your path to the final demo. | Clear final outcome, next steps and key risks; I believe you can deliver. | Credible plan; next steps and risks mostly clear. | High-level intentions; path or risks are vague. | No credible path to delivery. |
| 6 | I felt engaged as the stakeholder. | Clear, concise, engaging, and on time; convincing explanations. | Generally clear; minor pacing or jargon issues. | Hard to follow or poorly pitched to me. | Not understandable / over time. |

### C2 talking-point map (from C1 facts)

| C2 required point | Canonical location in this document | Highest-signal facts |
|---|---|---|
| Who the stakeholder is | [Stakeholders](#stakeholders) | Firefighters / SES; tutor Nadim acts as stakeholder; teaching staff as stakeholder |
| Problem and current addressing | [Problem](#problem); [Related Work](#related-work) | Trapped/lost firetruck crews; existing RPAs and robots are largely not automated |
| Broader vision | [Long-term ConOps (~5 years)](#long-term-conops-5-years) | Coordinated autonomous bushfire monitoring for NSW SES, fire services, national parks |
| Semester solution | [Semester ConOps (41068)](#semester-conops-41068) | Husky leads SES truck to a designated safe location in simulation |
| Technical challenges and approach | [Risks](#risks); [Requirements](#requirements) | Multi-variable safe routing; sensor distinguishability; ROS 2/Nav2 skill gap; reactive MVP then stretch dynamic fire / drone |
| How it works for the stakeholder | [Semester ConOps (41068)](#semester-conops-41068); [Architecture](#architecture) | Operator initiates and monitors; robot finds/avoids; path must be followable by the firetruck |
| Progress evidence | [Current State](#current-state); [Movement and Path Planning (Jack)](#movement-and-path-planning-jack) | Week 3: sim loaded. **2026-08-28:** Nav2 start→goal + `--replan` demos; thermal integration planned |
| End-of-semester demonstration | [Success criteria](#success-criteria); [MVP vs stretch](#mvp-vs-stretch) | Reach safe location without collisions; detect/avoid fire and obstacles; UI shows pose/path/hazards |
| Path, milestones, risks | [Timeline](#timeline); [Risks](#risks) | Phases 1–5 on Gantt; C1 due 28 Aug; six technical risks with owners |

### C2 evidence bar stated by Nadim (`18/08/26`)

C2 needs proof the project is coming along, including:

- sim screenshots
- some implementation of sensing to find a path
- modification of the sim environment
- screenshots of a custom environment
- the robot sensing and moving accordingly
- avoiding obstacles while trying to reach the goal

---

## C2 Team Contributions

**Type:** Current team responsibilities for C2 (stated by Jack Havranek; navigation details from `pathplanning_and_movement_implementation.md` and `master_robot_movement_pathplanning.md`)

| Member | C2 contribution area | Notes |
|---|---|---|
| Jack Havranek | **Robot movement and path planning** | Integrates provided sensor streams + groupmates’ thermal sensor data so the Husky can determine the safest path to the goal, including when dynamic blockages appear. Owns Nav2 mission layer, replanning demo, and system integration touchpoints. **C2 speaker:** Slides 1–3 (see below). |
| Phu Huynh | Perception (C1 lead) | Simulated sensor integration — LiDAR, camera, IR heat; supplies sensor data consumed by planning |
| Faiyad Hassan | Decision-making / UI (C1 lead) | Path planning and obstacle avoidance logic (C1 role); operator interface |
| Taj Wilcock | Simulation development (C1 lead) | Custom bushfire environment, fire hazard zones, tree states |

### Jack Havranek — movement and path planning (detail)

**Responsibility:** Decision-making layer that turns fused sensor input into safe motion: global route selection, local following, and replanning when the path is blocked.

**Sensor inputs (scope)**

| Source | Sensor / data | Role in planning | Status |
|---|---|---|---|
| Provided package | LiDAR `/husky1/scan` @ 10 Hz | SLAM map + Nav2 global/local costmaps; live obstacle detection | **Confirmed** in repo (`2026-08-28`) |
| Provided package | IMU, odometry, EKF | `odom→base_link`; fused pose for planners | **Confirmed** |
| Groupmates (perception) | Thermal sensor data | Fire / heat hazard regions for safer route selection (avoid hotspots, not just geometric obstacles) | **Planned / in progress** — stated C2 integration work; aligns with Nadim’s suggestion to prioritise thermal for fire projects; not wired into costmaps in repo at `2026-08-28` |
| Future | Carbon soot, additional IR | C1 ConOps names these; consumption by planner not implemented in repo | **Proposed** |

**Planning behaviour (target)**

```text
START → initialise sensors / TF / map → GOAL → global plan (NavFn on costmap)
  → follow path (Regulated Pure Pursuit) → continuous sensor updates
  → costmap updates → path still valid?
      → YES: continue | NO: replan (Nav2 BT is_path_valid) → continue → GOAL
```

**Dynamic blockages:** When a new obstacle appears on the route (e.g. dropped wall via `PathBlocker`, or future moving fire zone from thermal), lidar shortens → costmap marks cells occupied → behaviour tree invalidates path → new global plan → robot follows new route without a second manual goal.

**C1 requirement mapping (Jack’s lane):** R1, R4, R9 (MVP); supports R2/R3 via costmap inputs.

**Companion docs:** `pathplanning_and_movement_implementation.md`, `master_robot_movement_pathplanning.md`

---

## Assigned Speaker Slides — Jack Havranek

**Source:** `Presentation_Slides.md`  
**Speaker:** Jack Havranek  
**C2 segment:** Three slides on movement / path planning within the group presentation  
**Stakeholder angle:** Show that the robot can **find and follow a safe escape route** under sensor-driven autonomy, including when the path changes mid-mission

### Slide 1 — Broader Vision (Where This Can Go)

**Jack’s framing:** Autonomous evacuation routing, not just teleop driving

| Question (template) | Answer (Jack / project facts) |
|---|---|
| If developed beyond 41068, what could the system become? | A field-deployed lead robot that continuously recomputes the safest evacuation route for a following fire crew using multi-sensor fusion (LiDAR + thermal/IR + future soot sensing), not only at mission start |
| Real-world version in ~5 years? | Husky-class UGV on a fireground: analyses hotspots, assesses hazards, selects and updates navigation paths as conditions change; operator supervises via GUI with live paths and risk layers |
| Broader value? | Faster escape for trapped crews; less time in smoke and heat; path suitable for the firetruck (space, incline, heat margins from C1 ConOps) |
| How does 41068 move toward that? | This semester builds the **autonomous navigation core** in simulation: live mapping, global planning, local following, and **dynamic replanning** on blocked paths — foundation before adding full bushfire thermal fusion and multi-robot stretch goals |

### Slide 2 — What We Will Demonstrate by the End

**Jack’s framing:** What the stakeholder will **see** the movement stack do

| Question (template) | Answer |
|---|---|
| What will the stakeholder see? | Husky spawned in a forest world; given a start and a safe-house goal; autonomously drives there while building a map and displaying the planned path in RViz2 |
| What will the integrated system accomplish? | Sense obstacles via lidar → update costmaps → plan route around trees → follow path → if a **dynamic blockage** appears on the route, **replan** and complete the mission without manual re-goal |
| How will we know it worked? | `NavigateToPose` succeeds; robot reaches goal within tolerance (0.25 m / 0.35 rad); visible plan change after blockage; no collision with injected obstacle |
| Key outcomes / measures | MVP movement requirements R1, R4, R9; Nadim C2 bar: robot sensing, moving, avoiding obstacles toward goal; with thermal integration: routes that also respect **fire/heat hazard** regions from groupmates’ sensor pipeline |

**Demo commands (repo)**

```bash
python3 scripts/basic_autonomy_demo.py --start 0 0 0 --goal 0 -5 0
python3 scripts/basic_autonomy_demo.py --replan
```

### Slide 3 — Progress So Far

**Jack’s framing:** Evidence that autonomous path planning is **working**, not only proposed

| Question (template) | Answer |
|---|---|
| Implemented / tested? | **2026-08-28:** Full Nav2 + SLAM stack on `simple_trees`; configurable spawn (`husky_x/y/yaw`); `basic_autonomy_demo.py` start→goal; `--replan` inserts real Gazebo wall mid-path; `rs1_nav/` (`MissionRunner`, `NavObserver`, `PathBlocker`); automated tests (`navigation_test`, `replan_test`) |
| Evidence to show | RViz2: map, scan, global/local costmaps, `/husky1/plan`; Gazebo: Husky motion; optional `--replan` run showing path divergence then recovery |
| What it demonstrated | Provided lidar → SLAM → rolling 40 m global costmap → NavFn A* → Regulated Pure Pursuit → goal; dynamic replan verified (costmap 254 on barrier, plan divergence ~1.72 m, goal in ~32.6 s in `replan_test`) |
| Feasibility confidence | C1 Week 3 only showed sim loaded with Nav2 `unknown`; movement layer now closes the gap Nadim described for C2 |
| Next (Jack’s lane) | Wire **groupmates’ thermal sensor data** into hazard representation / costmap so “safest path” avoids **fire hotspots** as well as trees; extend dynamic blockage tests to heat-driven hazard zones |

**Contrast — C1 Week 3 (`11/08/2026`) vs navigation implementation (`2026-08-28`)**

| Aspect | Week 3 (C1 Figure 2) | Post-implementation |
|---|---|---|
| Sim loads | Yes | Yes |
| Nav2 status | Navigation / Localization / Feedback `unknown` | Lifecycle active; `navigate_to_pose` action available |
| Autonomous goal | Not demonstrated | Start→goal and replan demos |
| Evidence type | Setup screenshot | Tests + demo script + RViz paths/costmaps |

---

## Team

### Members

| Student number | Full name | Email |
|---|---|---|
| 25461649 | Phu Huynh | phu.q.huynh@student.uts.edu.au |
| 25355759 | Taj Wilcock | taj.g.wilcock@student.uts.edu.au |
| 25420973 | Faiyad Hassan | Mohammad.F.Hassan@student.uts.edu.au |
| 25470117 | Jack Havranek | jack.n.havranek@student.uts.edu.au |

### Capabilities and learning goals

| Member | Existing skills (source wording, condensed) | Skills to develop | Intended contribution |
|---|---|---|---|
| Phu | Python, C++, Java; SolidWorks CAD; ROS2; teamwork/communication across software and hardware | ROS2 CORE and virtual simulation (physics, world design, robotic design/implementation) | Mainly physics of robotic movement in the virtual environment; also wherever required |
| Taj | Communication and team management (leader and member) at UTS; Python; C++; some ROS2 | General coding skill; deeper ROS2 understanding | Wherever needed by the team and project |
| Faiyad | Python, C++, C#, Java; ROS 2; RViz; SolidWorks | Team communication/collaboration; ROS 2; RViz; system integration; practical robotics development | Across all areas; particularly software, ROS 2, CAD/mechanical, integration, testing, troubleshooting, documentation |
| Jack | Semi proficient in Python and C++; team collaboration and communication | ROS2 (main focus this semester) | **C2:** Robot movement and path planning — Nav2 mission layer, dynamic replanning, integrating provided sensors + groupmates’ thermal data for safe routing. **C1:** System integration, ROS 2 communication layer, obstacle detection module (lead) |

**Related fact:** All four members identify ROS 2 / simulation as a development area. This is later cited as a delivery risk.

### Operating agreement

**Type:** Team commitment / process

**Expectations**

- Attend weekly meetings
- Complete assigned tasks before internal deadlines
- Respond to team communication within 48 hours
- Contribute meaningfully to all group deliverables

**Accountability**

- Tasks tracked via shared Kanban board
- Missed deadlines must be communicated in advance
- Repeated lack of contribution triggers the conflict process
- If contribution remains insufficient after internal discussion: escalate to the coordinator with documented evidence

**Conflict resolution (ordered)**

1. Internal discussion
2. Documentation of issue
3. Escalation to subject coordinator (if unresolved)

**Academic integrity**

- All work must be original
- No plagiarism
- Each member must be able to explain their subsystem

**Commitment statement (verbatim):** We agree to work collaboratively, professionally, and ethically to deliver a high-quality system.

---

## Entities

### Project: B.E.E.R.

- **Full name:** Bushfire Environmental Evacuation & Rescue
- **Group:** 25
- **Course:** 41068 Robotics Studio 1
- **Title rationale:**
  - Core purpose: guiding trapped or disoriented SES crews to safety during bushfire emergencies
  - “Evacuation & Rescue” communicates the mission-critical function
  - “Bushfire Environmental” grounds the project in domain and operating conditions
  - Acronym chosen as short, memorable, easy for technical and non-technical stakeholders; maps meaningfully to the full name
- **Naming process:** Chat GPT produced an image used to inspire names; team brainstormed, polled top 3, selected `B.E.E.R.`

### Stakeholders

| Entity | Role in sources |
|---|---|
| Firefighters, SES | Intended users of the chosen application |
| NSW SES, fire services, national park authorities | Long-term (~5 year) users of the envisioned platform |
| Teaching staff (tutors and Subject Coordinator) | Act as project stakeholder for the subject |
| Nadim | Named tutor in the communication log; received idea pitch and C1/C2 Q&A |
| Operator | Initiates mission, specifies safe location, monitors UI; does not directly drive in the semester scenario |
| Firefighting crew / vehicle | Assumed capable of following the Husky’s identified route |
| National park rangers, researchers | Users of rejected Habitat Analysis candidate |
| Farmers / agriculture, foresters, silviculturists | Users of rejected Seed Spreader candidate |
| Police, Private Investigator | Users of rejected Missing Human Rescue candidate |

### Platforms and software

| Entity | Relationship |
|---|---|
| Husky | Primary mobile platform; provided; lead Phu |
| Aerial robot / UAV / drone / Parrot | Stretch multi-robot sensing; later introduction; models need not be changed |
| SES truck / firetruck | Follows Husky; path must consider truck space, heat, incline |
| Ignition Gazebo | Provided simulation environment; lead Taj |
| ROS 2 | Existing communication framework; lead Jack |
| Nav2 | Navigation stack named in skill-gap risk and early tests |
| RViz2 | Operator/debug visualisation; C1 testing environment named as custom `RVis2` |
| SolidWorks | CAD for mounting/sensor placement if required |
| Microsoft Teams | Daily communication; decision documentation |
| Microsoft Planner or Trello | Kanban options named |
| GitHub | Named in workflow text as shared repository host |
| `jackthebugger / RS1-Gr25` | Repository shown in Figure 3 (Gitea-style UI) |

---

## Problem and Value

### Problem

When a firetruck team becomes stuck, trapped, or lost in a bushfire — encircled by fire, disoriented by heavy smoke, or similar — the crew needs a safe evacuation path.

### Proposed solution

Deploy an automated robot to find a safe path to a safe location.

### Value claimed for the stakeholder

- Reduce harm risk from prolonged exposure by finding the quickest, least dangerous path
- Provide sensing not usually available to firetruck crews (smoke/fire navigation)
- Ensure the path is suitable for the firetruck as well as the robot
- Variables to keep within an acceptable range: amount of space, heat, surface incline
- Could be added to an existing fleet with minimal modifications
- Extended uses stated: scouting fires in buildings and bushfires; potential fire suppression from within the fire without endangering firefighters
- Long-term: faster/safer situational awareness; reduced need for personnel to enter hazardous areas; more informed bushfire response decisions

### How the problem is currently addressed (from related work)

| Existing approach | Automation status | Implication for B.E.E.R. |
|---|---|---|
| SES Remotely Piloted Aircrafts | Not automated (human operated) | Technology exists in the field; automation is not yet used |
| Ground-based firefighting robots | Not automated; remotely operated | Heat-resistant operation is possible; B.E.E.R. targets autonomy |
| Firefighting co-bots (research) | Research stage; not fully implemented in the real world | Human–robot firefighting collaboration is being seriously considered |

---

## Decisions

| Decision | Status | Rationale / context |
|---|---|---|
| Select Bushfire Environmental Evacuation & Rescue as the application | Confirmed (`11/08/2026`, team + Nadim) | Most interesting of five candidates; most potential for advanced functions; HIGH semester feasibility; team skills plus room for stretch/higher marks |
| Title / acronym `B.E.E.R.` | Confirmed | Memorable; maps to full name; selected after team poll of top 3 names |
| Semester goal: one robot to a set goal with static obstacles first | Confirmed | Show basic sensing/self-navigation before adding a second robot and a dynamically changing environment |
| Design the environment around the robot’s capabilities | Confirmed after Nadim feedback | Better show perception and path-finding than a ring-of-fire plus variable fire-trail goal |
| Multiple obstacles after the ring of fire | Confirmed (Nadim suggestion accepted) | Initial ring-of-fire idea was “on the right track” but did not fully show sensor/path-finding |
| One designated safehouse / spot rather than a variable fire-trail goal | Confirmed (Nadim suggestion accepted) | Replaces variable fire trail as the aim point |
| Two robots not required at the start | Confirmed (Nadim) | Implement basic sensing and self-navigation first, then extend |
| Do not need to change Husky or Parrot models | Confirmed (Nadim) | May stay as provided; may change if the team wants |
| Start thermal camera earlier than relying on package LiDAR for navigation | Recommendation from Nadim | LiDAR navigation in the package “could be difficult”; fire project could start with a thermal camera |
| Internet packages and genAI allowed | Confirmed with constraint | Only if the team understands the code and can adjust and explain it on the fly |
| Entire 41068 system in simulation | Confirmed constraint | Performance limited to provided Husky and available simulated sensors |
| MVP navigation = local/reactive hazard avoidance | Confirmed constraint | Not full dynamic global route optimisation |
| Dynamic fire spread = stretch | Confirmed | May be implemented as stretch; fallback is static pre-placed hazard zones |
| Direct commits to `main` forbidden | Confirmed process | Feature branches; PRs; review by at least one other member |

### Alternatives considered (candidate applications)

C1 guidance: suggested 50 words per use case; five applications aligning with the Project Brief vision; practical or speculative; must respond to a meaningful forest-management or environmental-monitoring need.

| Option | Users | Need / value | Main technical challenge | Feasibility | Outcome |
|---|---|---|---|---|---|
| Habitat Analysis | National park rangers, researchers | Automate search for endangered or invasive plants for faster, more targeted bush regeneration and protection | Identify a wide range of plant species reliably and efficiently | MEDIUM | Not selected |
| Bushfire Environmental Evacuation & Rescue (BEER) | Firefighters, SES | Deployable automated robot identifies escape routes and leads firefighters when trapped; considers fire intensity, firetruck size, terrain difficulty | Assess a wide range of variables simultaneously to identity the safest route efficiently | HIGH | **Selected** |
| Bushfire priority identification | Firefighters, SES | IR heat mapping, LiDAR, carbon soot sensors on drones/huskies to find hottest / highest-spread-chance areas | Sensors for fire detection and accurate priority determination | MEDIUM | Not selected |
| Seed Spreader | Farmers/Agriculture, forester, silviculturists | Increase population of diminishing/rare flora | Healthy soil detection and planting processes | MEDIUM | Not selected |
| Missing Human Rescue | SES, police, Private Investigator | Use fewer resources/officers for “Man hunt” | Find well hidden and deceased bodies | LOW | Not selected |

---

## Concept of Operations

### Long-term ConOps (~5 years)

**Type:** Future plan / vision

- Coordinated autonomous bushfire monitoring platform
- Users: organisations such as NSW SES, fire services, and national park authorities
- Operating environment: forests, mountains, bushland during high-risk fire conditions or active bushfire events
- Platform: Husky equipped with LiDAR, cameras, temperature sensors, and localisation systems, deployed to investigate priority locations in greater detail
- Stated long-term priority: human survival as number one priority; eventually finding the optimal path to rescue trapped civilians
- Autonomous behaviour:
  - analyse sensor data
  - determine location and severity of detected hotspots
  - assess surrounding hazards
  - prioritise areas requiring urgent attention
  - select safe navigation paths
  - continuously update route as environmental conditions change
- Human operator: supervises via central interactive GUI showing robot locations, detected hotspots, risk levels, navigation paths, and live sensor information; may assign new objectives or override autonomous decisions

### Semester ConOps (41068)

**Type:** Proposed semester solution

**Setup**

- SES truck at a simulated bushland location representing last known position of a disoriented/trapped firefighting crew (heavy smoke, encroaching fire)
- Environment mix: clear ground, vegetation, simulated fire hazards
- Mission goal: a defined safe location (e.g. fire trail or safehouse)

**Mission flow**

1. SES truck deploys onboard Husky to lead the truck to the safety point
2. Husky uses simulated sensors (LiDAR, camera, carbon soot, IR heat) to perceive surroundings, including static obstacles (trees, dense bushland) and fire zones
3. Robot continuously scans for obstacles intersecting its planned path and identifies simulated fire regions to avoid
4. System evaluates paths toward the goal and selects a route that avoids detected hazards, applying a predefined clearance margin approximating space required by the following firefighting vehicle
5. If an obstacle or fire region blocks the current path, the robot re-plans a local detour and continues toward the goal
6. Position, planned path, and detected hazards display on a simple operator UI
7. Operator initiates the mission and observes progress (standing in for the crew that would follow in the full-scale application)

**Staged development (team + stakeholder log)**

1. One robot, stagnant/static obstacle environment, static fires, preset obstacles, static safe paths
2. Later: dynamically spreading fires (changing paths)
3. Later: introduce a drone that relays path data from above to the robot/truck; live/moving fires change the robot path in real time

### Success criteria

The scenario ends when the robot reaches the designated safe location after autonomously navigating around all static obstacles and hazards.

Success is demonstrated by:

- completing traversal without collisions
- UI accurately reflecting real-time position and path
- correctly identifying and avoiding all simulated fire hazards placed in the environment

---

## Requirements

**Source:** C1 Section 3.1

### MVP vs stretch

- **MVP:** R1, R2, R3, R4, R5, R9
- **Stretch:** R6, R7, R8, R10

### Requirement catalogue

| ID | Category | Requirement | Why important | Verification / demonstration | Priority |
|---|---|---|---|---|---|
| R1 | Decision making | Husky shall autonomously navigate from start to an operator-defined safe goal without direct manual driving | Fundamental mobility for environmental monitoring and rescue missions | Observe multiple waypoints and assign itself to find the best path. Robot reaches end goal in simulation without direct manual driving. | MVP |
| R2 | Perception | Detect simulated fire hazards within Husky sensing range and report locations to decision-making | Identify hazardous areas; meaningful bushfire-response perception | Place fire truck with Husky in the middle of the forest fire; it finds and guides the fire truck out to safety | MVP |
| R3 | Perception | Detect nearby obstacles using simulated onboard sensors | Prevent collisions with trees, debris, terrain, and environmental obstacles | Place obstacles along the terrain on the robot's route; demonstrate detection by the perception system | MVP |
| R4 | Decision making | When an obstacle blocks the current route, avoid it and continue toward the designated safe location where a traversable route exists | Perception must influence autonomous decision making and mission safety | Introduce an obstacle in the planned route; observe stop or modified motion. Change fire position so an alternate path is required | MVP |
| R5 | User interface | Display robot current position, mission waypoints, and detected fire hotspots | Operator situational awareness; quick interpretation of progress and hazards | Run a mission; verify position, waypoints, and hotspots are visibly represented | MVP |
| R6 | User interface | Display current mission status (idle, navigating, responding to a hazard, or mission complete) and provide real-time data | Operator understands autonomous system state without directly observing the robot | Change the robot between mission states; confirm displayed status updates | Stretch |
| R7 | Simulation | Simulated vegetation shall support healthy, burning, and burnt states, with transitions triggered by the fire simulation | Meaningful dynamic bushfire environment; perception data | Trigger simulated fire; visually demonstrate trees changing colour/appearance across the three states | Stretch |
| R8 | Perception | Generate and update a map of environmental features, robot position, and identified hazardous regions | Richer disaster representation for planning and operator awareness | Demonstrate a map generated or updated as sensor information is received | Stretch |
| R9 | Decision making | Automatically calculate an evacuation or rescue route that avoids known fire hotspots and obstacles | Extends basic navigation to mission-level autonomous bushfire response | Scenario with hazards between robot and destination; demonstrate generation of a safe alternative route | MVP |
| R10 | Multi-robot system | An aerial robot shall survey the environment and communicate detected fire or hazard locations to the ground robot | Aerial sensing covers a larger area; improves ground-robot awareness beyond local sensors | Demonstrate UAV detecting a hazard and the corresponding location appearing in the ground robot’s mission information or map | Stretch |

### Conceptual functions (chosen-application text; not numbered R-IDs)

- Find the quickest, least dangerous path
- Sense through smoke and fire
- Keep path suitable for firetruck and robot
- Check space, heat, and surface incline against an acceptable range

---

## Constraints, Assumptions, Scope

**Authoritative C1 section 2.6 body title:** Scope Boundaries and Assumptions  
**C1 TOC lists 2.6 as:** Key Challenges — see [Open Issues and Conflicts](#open-issues-and-conflicts)

| Type | Statement |
|---|---|
| Scope — will | Autonomously move toward a designated safe location (safe house or fire trail). When an obstacle or fire hazard blocks its path, the Husky locally avoids the hazard and continues toward the designated safe location where a traversable route exists. |
| Scope — will not | Extinguish fires, carry loads, or physically transport/rescue people. |
| Assumption — environment | Terrain is sufficiently traversable for Husky movement. At least one safe path to the designated safe location exists. |
| Assumption — robot/sensors | Simulated sensors reliably detect relevant obstacles and fire hazards within defined sensing range. Localisation data is sufficiently accurate for navigation. |
| Assumption — user/operator | Operator can initiate the mission and specify the designated safe location. Firefighting crew or vehicle is capable of following the Husky’s identified route. |
| Constraint — simulation fidelity | Fire intensity, vegetation/tree density, terrain complexity, and sensor behaviour are simplified versus real bushfire conditions. Dynamic fire spread may be a stretch feature. |
| Constraint — 41068 | System operates entirely in simulation. MVP navigation uses local/reactive hazard avoidance rather than full dynamic global route optimisation. Performance is limited by the provided Husky platform and available simulated sensors. |
| Constraint — C2 delivery | 10 minutes maximum; overtime penalised; all members in person with meaningful contribution. |
| Constraint — genAI / external packages | Allowed if the team understands the code and can adjust and explain it on the fly. |

---

## Architecture

**Source:** C1 Section 3.2, PDF p. 11 / printed p. 10  
**Type:** System architecture block diagram plus Husky hardware illustration

### Narrative (source wording preserved)

The diagram shows integration of real-time environmental sensing with adaptive motion control for autonomous operations. Activation is via an operator GUI. Onboard hardware feeds continuous data to a perception processor for fire fire and obstacle detection, and from that navigate the environment. The decision-making core takes this data and outputs motion control to execute moving functions that navigate to a safe location. Essential system info is stored and displayed through the GUI for testing and debugging. Secondary robots may be included and will communicate with the Husky to navigate faster and more efficiently. All testing will be done in the custom `RVis2` environment with increasingly challenging environments.

**Source artefacts in that paragraph:** duplicate “fire fire”; `RVis2` spelling.

### Data flow

```
Operator → Sensors → Perception → Decision Making → Control → Robot motion
                ↓                      ↓
         Simulation Testing ← Operator Interface (display/debug)
                ↑
    [Future: Drone → Decision Making / Perception]
    [Future: Multi-robot communication with Husky]
```

### Blocks

| Block | Colour in diagram | Inputs | Internal modules | Outputs / relationships |
|---|---|---|---|---|
| Operator Interface | Red | Activation; Safehouse location; Robot speed | — | To Decision Making; receives feedback from Simulation Testing and Validation |
| Sensors | Red | Connected from Operator Interface | LiDAR; Thermal Camera; Infrared Sensors; CO Sensor | Sensor Data Storage and Processing → Perception |
| Perception | Red | Sensor Data Storage and Processing | Fire Detection; Obstacle Detection; Mapping and Nav | To Decision Making |
| Decision Making | Red (central hub) | Perception; Operator Interface | Local Reactive Planning (From Environment); Global Path Finding (From User Input) | Speed, Rotation, Position, Status, Battery → Control and Simulation Testing and Validation |
| Control | Red | Decision Making outputs | Motion Control | Motion Control feeds back to physical environment / robot |
| Simulation Testing and Validation | Red | Decision Making outputs | Scenario Test in Virtual Environment (`RVis2`) | Connected to Operator Interface |
| Drone Implementation for Global Path Identification | Red; dotted (future) | — | — | To Decision Making and Perception; label “Dynamic environment → Adaptive decision making” |
| Future Multi Robot Implementation and Communication | Grey (future) | — | — | Between Perception and Husky illustration |

### Architecture vs component-plan gap

System diagram includes Operator Interface, Decision Making sub-modules, Motion Control, and Simulation Testing blocks that are **not** separate rows in the Component Plan. The Component Plan instead lists Mission Manager, Map, and Obstacle Detection Module as custom modules that partially cover those functions.

**Resolution:** Unresolved in source — both views are preserved.

### Husky hardware illustration labels

**Platform type in illustration:** four-wheeled unmanned ground vehicle (UGV)

| Label | Description in extraction |
|---|---|
| LiDAR array | Top-mounted scanning sensor |
| High-res thermal imager | Forward-facing thermal camera |
| Infrared sensors | IR detection components |
| GPS/Navigation unit | Positioning/navigation module |
| Thermal hotspot | Annotated heat source on illustration |
| System processor | Central compute unit |
| Acoustic sensor | Audio detection |
| Multi-gas sensor suite | Gas detection array |
| Chemical sensor | Chemical detection component |

---

## Components

**Source:** C1 Section 3.3; table continues PDF pp. 11–12 / printed pp. 10–11

| Component | Purpose | Inputs | Outputs | Origin | Initial lead |
|---|---|---|---|---|---|
| Husky | Primary mobile platform to travel through the simulated bushfire environment and investigate hazardous areas | Velocity/motion commands; simulated environment interactions | Odometry, robot pose, sensor data | Provided | Phu |
| Ignition Gazebo Simulation Environment | Simulates bushland environment, robot movement, sensors, obstacles, and fire-related scenarios | World configuration, robot commands, model states | Simulated sensor data and visual environment | Provided | Taj |
| Robot Sensors | Surroundings information for obstacle and hazard detection; may include LiDAR and other Husky-model sensors | Simulated environment | Range measurements, distance from goal, sensor messages | Provided and out sourced | Phu |
| Bushfire Environment / Tree Models | Vegetation and environmental obstacles in the simulated disaster area | Initial world configuration and fire-state commands | Visual and state information for trees and obstacles | Adapted | Taj |
| Dynamic Fire-State System | Controls vegetation transitions among healthy, burning, and burnt | Fire trigger, current tree state, neighbouring fire conditions or timers | Updated tree state and visual appearance | Custom | Taj |
| Obstacle Detection Module | Determines whether trees, debris, or other objects obstruct the planned path | LiDAR/range sensor data | Obstacle locations, distance, hazard flags | Custom | Jack |
| Map | Representation of robot, mission waypoints, obstacles, and detected fire hazards | Robot pose, sensor observations, hotspot locations | Updated environment/hazard map | Custom | Faiyad |
| Mission Manager | Coordinates overall mission; tracks idle, navigating, hazard detected, mission complete | Operator commands, navigation status, hazard detections | Mission state, active goal, system commands | Custom | Faiyad |
| ROS 2 Communication Layer | Modules exchange sensor data, commands, mission states, and detected hazards | ROS 2 topics, services, messages | Inter-node communication | Existing Framework | Jack |

Support members for components are not listed in the component-plan table; see [Roles](#roles).

---

## Simulation and Evaluation Environment

**Intent:** Custom simulation environment that showcases robot abilities and fits the created scenario.

| Topic | Source statement |
|---|---|
| Reuse | Some provided assets may be carried over, particularly tree models and ground textures |
| Custom terrain / scenario | Hills, trenches, dense and sparse forest, fires, dead ends |
| Stretch | Dynamic fire that spreads across the sim environment |
| Primary robot | Husky completes the majority of sensing and navigation |
| Stretch robot | Aerial drone support for more optimised routing by the Husky |
| Provided physics/models | Husky models, sensors, and sim physics utilised but expanded |
| Navigation expansion | More robust navigation that accounts for obstacles and the needs of the following firetruck |
| Additional sensors | Thermal sensors to assist the task and provide additional operator information |
| Testing progression | Custom `RVis2` environment with increasingly challenging environments |

---

## Related Work

| Existing research, product, or system | What it does | Strengths / limitations | How it informs B.E.E.R. |
|---|---|---|---|
| SES Remotely Piloted Aircrafts — https://statements.qld.gov.au/statements/100929 | Search and rescue, disaster assessment, direct communication, emergency resupplies | High efficiency vs walking or driving; health and safety concerns reduced. **Not automated** (human operated) | Technology exists in this field already. Automation is not yet used. |
| Ground Based Firefighting Robots — https://www.hcrot.com/what-robots-help-firefighters | Enter burning buildings with water jets and foam dispensers; target fires with thermal imaging cameras; remotely controlled | Heat resistance — can function in high temperature environments. **Not automated** | A robot operating automatically in a high-temperature environment such as a bushfire is entirely possible. |
| Firefighting Co-bots — https://news.griffith.edu.au/2026/02/16/ai-powered-robot-vehicles-team-up-to-fight-fires/ | Robot–human collaboration; AI-powered robots supervised by a human operator; AI handling some tasks autonomously reduces operator cognitive load and stress | Human–robot collaboration is being investigated as a serious addition to firefighting. Still in the research stage — not fully implemented in the real world | Using robotics for firefighting-related purposes is being seriously considered in related fields, giving validity to the project ideas |

---

## Current State

### C1 proposal snapshot — Week 3 (`11/08/2026`)

**Source:** C1 Section 4.4 / Figure 2–3

- Provided simulated environment loaded and configured
- Team beginning testing and familiarisation with Husky simulated behaviour
- Shared Git repository established (`jackthebugger / RS1-Gr25`)
- RViz2 loaded with lidar/map/path displays; Nav2 panel showed Navigation / Localization / Feedback `unknown`
- Team assessment: groundwork laid; no major technical risks identified at that early stage

### Navigation / path planning — implementation evidence (`2026-08-28`)

**Source:** `pathplanning_and_movement_implementation.md` §63–65  
**Owner lane:** Jack Havranek

**Status:** Autonomous start→goal navigation and dynamic replanning **confirmed** in `simple_trees`.

| Layer | Implementation |
|---|---|
| Simulation | Ignition Gazebo; Husky DiffDrive; `simple_trees` world |
| Sensors (provided) | LiDAR `/husky1/scan` @ 10 Hz → SLAM + Nav2 costmaps |
| Mapping | SLAM Toolbox → `/husky1/map` |
| Planning | NavFn global planner + Regulated Pure Pursuit |
| Replanning | Nav2 BT `is_path_valid`; verified by `replan_test.py` |
| Demo | `basic_autonomy_demo.py`, `rs1_nav/` (`MissionRunner`, `PathBlocker`) |

**Replan test (source):** costmap 254 on barrier → plan divergence ~1.72 m → goal ~32.6 s.

**Not in repo at evidence date:** thermal data in costmaps (Jack’s stated next step); custom bushfire world (Taj); operator UI paths (Faiyad).

### Figure 2 — Simulated environment (C1 Week 3)

- Gazebo: green tiled ground plane; multiple realistic tree models; small yellow Husky
- RViz2 config path: `/home/tajw/git/team25_rs1-Gr25/install/41068_ignition_bringup/share/41068_ignition_bringup/config/41068_husky1.rviz`
- Active displays: Grid, RobotModel, Lidar, Map, Global Path, Local Path, Camera, Depth PointCloud
- Central view: robot perspective with white point cloud (trees and ground)
- Navigation 2 panel: Navigation `unknown`; Localization `unknown`; Feedback `unknown`
- Terminal log prefix: `[rviz2-16] [INFO] [1787088354.657...]`
- Extraction interpretation at Week 3: simulation stack loads; navigation/localisation not yet operational in that screenshot

### Figure 3 — Git repository (C1 Week 3)

| Field | Value |
|---|---|
| Repository | `jackthebugger / RS1-Gr25` (Public) |
| Description | RS1 Forest Management Project |
| Latest commit author | TajWilcockUTS |
| Latest commit message | Adding sim pakage to repo |
| Latest commit hash | `9d8c8df` |

Visible paths:

| Path | Last commit message | Time |
|---|---|---|
| `.vscode` | Adding sim pakage to repo | last week |
| `build` | Adding sim pakage to repo | last week |
| `install` | Adding sim pakage to repo | last week |
| `log` | Adding sim pakage to repo | last week |
| `src/41068_ignition_bringup_v1/...` | Adding sim pakage to repo | last week |
| `README.md` | Initial commit | last week |
| `hello` | hello (first commit OMG!) | last week |

Visible tabs: Code, Issues, Pull requests, Agents, Actions, Projects, Wiki, Security and quality, Insights, Settings

### Historical vs current caution

- C1 progress text is dated Week 3 (`11/08/2026`); navigation implementation evidence is `2026-08-28`
- Do not use Week 3 Nav2 `unknown` screenshot alone as C2 progress evidence for Jack’s movement segment
- C2 is Week 6 Studio; thermal integration and custom bushfire world may advance after `2026-08-28` — not recorded here unless added to source docs

---

## Movement and Path Planning (Jack)

**Owner:** Jack Havranek  
**Sources:** `pathplanning_and_movement_implementation.md`, `master_robot_movement_pathplanning.md`, `scripts/basic_autonomy_demo.py`, `rs1_nav/`

### Role in the B.E.E.R. stack

Jack’s layer sits between **perception** (Phu / groupmates: lidar, thermal, future soot) and **Gazebo motion** (DiffDrive). It consumes sensor-driven occupancy and produces `cmd_vel` via Nav2 to reach the operator’s safe goal, replanning when the route becomes unsafe.

```text
Perception topics (lidar scan; thermal hazard map [planned])
        ↓
SLAM Toolbox (/husky1/map) + Nav2 costmaps (global 40 m rolling, local 10 m)
        ↓
NavFn global planner (/husky1/plan) — avoids lethal / high-cost cells
        ↓
Regulated Pure Pursuit → velocity_smoother → /husky1/cmd_vel → DiffDrive
        ↓
BT navigator: is_path_valid? → replan if blocked
        ↓
MissionRunner (basic_autonomy_demo.py) — bounded start→goal, optional PathBlocker
```

### Implemented stack (`2026-08-28` — **Confirmed**)

| Component | Package / file | Function |
|---|---|---|
| DiffDrive | `urdf_husky/husky.gazebo.xacro` | Executes Nav2 velocity commands in Gazebo |
| LiDAR bridge | `config/gazebo_bridge_husky1.yaml` | `/husky1/scan` @ 10 Hz → SLAM + costmaps |
| EKF | `config/robot_localization_husky1.yaml` | `husky1_odom → husky1_base_link` |
| SLAM | `slam_toolbox`, `config/slam_params_husky1.yaml` | `/husky1/map`, `map→odom` |
| Global costmap | `config/nav2_params_husky1.yaml` | Rolling 40×40 m; obstacle + inflation layers |
| Local costmap | same | Rolling 10×10 m; reactive obstacles |
| Global planner | Nav2 `NavfnPlanner` | A* on costmap; `allow_unknown: true` |
| Controller | `nav2_regulated_pure_pursuit_controller` | Follow global path; collision slowdown |
| Replanning | Nav2 BT + `is_path_valid` | New plan when path blocked by live scan |
| Path blocker | `rs1_nav/gazebo_world.py` `PathBlocker` | Inserts real Gazebo box on route for `--replan` demo |
| Mission orchestration | `rs1_nav/mission.py` `MissionRunner` | Launch/wait/send goal/report/timeouts |
| Observation | `rs1_nav/nav_observer.py` `NavObserver` | Subscribes scan, map, costmaps, plan |
| Demo entry | `scripts/basic_autonomy_demo.py` | One-command autonomous demo |

### Thermal + multi-sensor integration (**Planned / in progress** — Jack stated C2 scope)

**Goal:** Combine provided geometric sensing with **groupmates’ thermal sensor data** so “safest path” means avoiding **fire / heat hazards** as well as trees and walls.

| Integration step | Status | Notes |
|---|---|---|
| LiDAR → costmaps | **Confirmed** | Primary obstacle source today |
| Thermal hazard regions → costmap or planner input | **Planned** | Aligns with R2 (fire hazard detection) and Nadim thermal-camera guidance; perception owned by teammates |
| Dynamic fire zones as moving blockages | **Proposed** | Stretch: Taj dynamic fire + thermal updates → Jack replan loop |
| Carbon soot / additional IR | **Proposed** | Named in C1 ConOps; not in repo |

**Dependency:** Jack’s planner consumes perception outputs via ROS 2 topics/services — interface to be defined with Phu (perception lead) and documented in shared specification (team workflow).

### How to run (demo)

```bash
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source ~/RS1-Gr25/install/setup.bash
cd ~/RS1-Gr25/src/41068_ignition_bringup_v1/41068_ignition_bringup

# start→goal in simple_trees
python3 scripts/basic_autonomy_demo.py --start 0 0 0 --goal 0 -5 0

# dynamic blockage + replan
python3 scripts/basic_autonomy_demo.py --replan

# with RViz / Gazebo GUI
python3 scripts/basic_autonomy_demo.py --rviz --gui --goal 0 -5 0
```

### Stakeholder one-liner (from implementation doc §55)

“We simulate a Husky in Gazebo. Its lidar builds a live map. You pick a start and a goal; Nav2 plans a path and drives there. If we drop a wall on that path, the laser sees it, the map updates, and the robot goes around — no one is joysticking it.”

### C1 risks owned by Jack (movement-related)

| Risk | Mitigation in implementation |
|---|---|
| Path planner fails around dynamic fire + static obstacles | Reactive local costmap + BT replan; fallback stop-and-detour if needed |
| ROS2 Nav2 integration bugs/latency | Modular `rs1_nav` nodes; early integration tests |
| Team ROS2/Nav2 skill gap | Nav2 extension rather than custom planner; documented run workflow |

---

## Roles, Workflow, and Delivery Path

### Roles

| Activity | Lead | Support | Expected contribution / outcome |
|---|---|---|---|
| Perception | Phu | Faiyad | Simulated sensor integration (LiDAR, camera, IR heat) for obstacle and fire detection, tuned and validated in simulation |
| Decision-making | Faiyad | Taj | Path planning and obstacle avoidance enabling Husky to navigate around hazards toward the goal |
| **Movement / path planning (C2)** | **Jack** | Faiyad, Phu | Nav2 start→goal, dynamic replanning, sensor-driven costmaps; integrate groupmates’ thermal hazard data for safest route |
| User interface | Faiyad | Jack | Operator interface displaying robot position, path, and detected hazards in real time |
| Simulation development | Taj | Jack | Custom bushfire environment including tree states (healthy/burning/burnt) and fire hazard zones |
| System integration | Jack | Faiyad | Combine perception, decision-making, and UI into a working end-to-end system |
| Testing and evaluation | Phu | Taj | Test cases for obstacle detection, path planning accuracy, and mission success criteria |
| Repository and software quality | All | All | GitHub repository structure, pull request reviews, coding standards |
| Stakeholder communication | All | All | Liaise with teaching staff acting as stakeholder; maintain communication log |
| Project documentation | All | All | Compile and update proposal and design portfolio documents |
| Project management | Jack | Taj | Coordinate milestones, Kanban board, and meeting schedules |
| Hardware/Sensor configuration | Phu | Faiyad | Configure simulated sensor parameters (range, noise, field of view) to reflect real-world Husky sensor specifications |
| Risk Management | Taj | Phu | Identify, track, and update technical risks and mitigation plans |
| CAD/Mechanical Design | Phu | Taj | Physical mounting or sensor placement design for the Husky platform, modelled in SolidWorks where required |

### Development workflow

**Version control**

- Shared GitHub repository (workflow text)
- Feature branches, e.g. `feature/perception-module`
- Direct commits to `main` not permitted
- Merge via pull requests after testing and review by at least one other member
- Integration agreed during scheduled check-ins

**Task tracking**

- Kanban (Microsoft Planner or Trello)
- Columns: “To Do,” “In Progress,” “Testing,” “Done”
- Microsoft Teams for daily communication; key decisions documented in a shared channel

**Interfaces**

- Software interfaces defined early and documented in a shared specification
- Updates communicated via Teams when changes are proposed

**Integration strategy**

- Subsystems integrated into a shared development branch as soon as test-ready (not end of semester)
- Weekly integration check-ins to test combined functionality, catch issues early, adjust plans

### Immediate next steps after C1 submission

1. Establish shared ROS 2/Gazebo development environment, Git workflow, and integration process
2. Configure Husky platform and sensors
3. Develop custom simulation world
4. Scaffold core perception, decision-making, and mission-management modules
5. Early integration checks for component communication and consistent workflows
6. Begin technical risk testing: sensor reliability, navigation, obstacle detection
7. Foundation for independent module testing, early issue identification, and first working prototype

### Gantt phases (Figure 1)

Visible expanded phase: **Phase 1: FOUNDATION & SETUP**

Collapsed (names truncated in screenshot):

- Phase 2: DEVELOPMENT ENVIRONMENT
- Phase 3: CORE COMPONENT DEVELOP…
- Phase 4: SYSTEM INTEGRATION & END…
- Phase 5: TESTING & VALIDATION

---

## Risks

| Risk | Why it matters | Early test | Mitigation / fallback | Owner |
|---|---|---|---|---|
| Path planner may fail to find a safe route around dynamic fire and static obstacles simultaneously | Without reliable replanning the robot cannot complete the primary mission of guiding the crew to safety | Basic global planner in a simple world with a single static obstacle, before adding dynamic fire | Fall back to simpler reactive obstacle-avoidance (e.g. stop-and-detour) if full dynamic replanning is too complex in the timeframe | Jack |
| Simulated sensors (IR heat, carbon soot, LiDAR) may not distinguish fire hotspots from normal terrain | Inaccurate detection undermines perception and downstream decision-making | Bench test: known fire/hotspot regions; confirm sensor outputs distinguish them from clear terrain | If custom sensor modelling is unreliable, publish a known ground-truth hazard zone from the simulation for perception testing | Phu |
| Integration of perception, decision-making, and control may introduce bugs or latency | Components may work in isolation but fail when passing data between ROS2 nodes in real time | Minimal perception stub connected to the navigation stack to confirm message flow and timing | Modular ROS2 nodes with clearly defined topics/interfaces so faulty components can be isolated and replaced | Jack |
| Custom simulation (dynamic fire, tree state transitions) may be hard to build or too expensive in Gazebo/Isaac | Without a working dynamic environment, several stretch goals and realistic testing are not possible | Prototype a small number of trees transitioning healthy/burning/burnt; check feasibility and performance | If full dynamic fire is infeasible, use static pre-placed hazard zones that still test avoidance | Taj |
| Limited prior ROS2 Nav2 and simulation experience may slow development | All members identified ROS2/simulation as a skill gap — delivery risk | Week 1–2 individual tutorials/small tasks (e.g. each member runs a basic Nav2 demo) | Pair less experienced with more experienced members; extra Gantt buffer for ROS2-related tasks | Jack |
| UI may not update in real time with accurate position, path, and hazard data | Delayed or inaccurate UI undermines operator situational awareness — a key evaluation criterion | Publish dummy position/hazard data to the UI before connecting live sensor data | If continuous streaming is too complex, periodically refresh (e.g. every 1–2 seconds) | Faiyad |

**C2 criterion 5 mapping:** path to final demo must include key risks. The six rows above are the C1 risk set.

---

## Timeline

| Date / window | Event | Outcome |
|---|---|---|
| Due 31/07/26 | Team formation, roles, etc. | Completed (Gantt) |
| Finished ~16 Aug | Define ConOps & requirements | Completed (Gantt) |
| Finished ~16 Aug | Literature & related work review | Completed (Gantt) |
| Finished ~16 Aug | Initial system architecture | Completed (Gantt) |
| 12–18 Aug | Develop high-level system block diagram | In progress at 18 Aug (Gantt) |
| 13–18 Aug | Identify hardware/software components | In progress at 18 Aug (Gantt) |
| 14–18 Aug | Define ROS 2 node architecture | In progress at 18 Aug (Gantt) |
| 15–18 Aug | Define module interfaces/data flow | In progress at 18 Aug (Gantt) |
| 11/08/2026 | Week 3 technical progress write-up | Sim loaded; Git repo established; no major risks identified at that stage |
| 11/08/2026 | Team meeting (all four) | Selected B.E.E.R.; static-then-dynamic staging; roles discussion |
| 11/08/2026 | Stakeholder meeting with Nadim (all four) | Idea selected; environment/obstacle/safehouse feedback accepted |
| 16–27 Aug | Proposal development | Ongoing at Gantt snapshot |
| 17–18 Aug | Consolidate research and requirements | Marked “Today” on 18 Aug Gantt |
| Tuesday 18 August | Gantt current-date marker | Figure 1 snapshot date |
| 18/08/2026 | Team meeting (Taj, Faiyad, Jack) | C1 finalisation and role assignment discussed; **outcome not recorded** |
| 18/08/26 | Stakeholder Q&A (Taj, Faiyad) | Model/C2/genAI/LiDAR-vs-thermal guidance recorded |
| 19–21 Aug | Draft methodology/system architecture | Marked “Tomorrow” relative to 18 Aug |
| 24–25 Aug | Internal team review | Planned |
| 26–27 Aug | Finalise and proofread proposal | Planned |
| Friday 28 Aug | **MILESTONE: Component 1 Proposal Due** | Red diamond on Gantt |
| Week 6 Studio class | **C2 in-class presentation** | 10 minutes maximum |

Gantt visible window: approximately Wed 12 Aug – Sun 30 Aug (weeks 33–35). Tool appearance consistent with Notion or similar (extraction note).

---

## Communication

### Team decision and action log

**Purpose:** Living record of internal discussions, decisions, changes, and actions affecting scope, technical approach, task allocation, or team operation.

| Date | Members | Discussion | Outcome |
|---|---|---|---|
| 11/08/2026 | Taj, Faiyad, Phu, Jack | Team roles; which of 5 ideas is best; how to design the environment to show sensing/navigating; current goal | Current scope: one robot navigates to a set goal with static obstacles. After basic sensing/self-navigating, introduce another robot and a dynamically changing environment. |
| 18/08/2026 | Taj, Faiyad, Jack | Finalisation of C1 document; role assignment | **No outcome recorded in source** |
| — | — | “(add more rows as necessary)” | Template leftover in source |

### Stakeholder communication log

**Purpose:** Record discussions with teaching staff acting as stakeholder (tutors and Subject Coordinator). Regular constructive communication is described as important for negotiating scope, checking assumptions, receiving feedback, and responding to changing needs.

| Date | Members | Discussion | Outcome |
|---|---|---|---|
| 11/08/2026 | Taj, Faiyad, Phu, Jack | Presented 5 ideas to Nadim; selected Bush-Fire Environmental Evacuation & Rescue (BEER). Initial ring-of-fire firetruck-to-fire-trail concept on the right track but not fully showing sensor/path finding. Nadim suggested multiple obstacles after the ring of fire and one designated spot rather than a variable fire-trail goal. | Include multiple obstacles to sense (lidar, heat mapping IR, carbon soot) and circumnavigate to a designated safe house. Design environment around robot capabilities. Initially static fires with preset obstacles (static safe paths); later dynamic spreading fires. Start with stagnant obstacle environment; later introduce a drone relaying path data from above. |
| 18/08/26 | Taj, Faiyad | Questions about C1 and C2: changing Husky or Parrot models; what to have achieved by C2; where to find programming help | Not required to change Husky or Parrot models (can if desired). C2 needs proof of progress (sim screenshots; sensing to find a path; sim environment modification). Nadim: package LiDAR navigation could be difficult; could start with a thermal camera. Internet packages and genAI allowed **if the team understands the code and can adjust and explain it on the fly**. |

### Priority stakeholder questions

C1 guidance: 2–5 substantive questions affecting scope, technical approach, feasibility, assumptions, or next steps. Avoid vague questions such as “Is our idea okay?” Discuss after submitting the proposal; record responses and explain actions in later assessments. Source also says complete responses “in the next relevant assessment”; responses are nonetheless filled in the C1 table.

| Question | Why it mattered | Response received |
|---|---|---|
| Will our idea be sufficient to show off our robots capabilities as per criteria? | If the idea barely shows sensing and self-navigating, marks will be lost | Yes the idea is good, but more obstacles are needed after the ring of fire |
| Do we need to include 2 robots working together right off the bat? | Allows a simple first design before advanced multi-robot work | No — not needed for the first part; after basic sensing and self-navigation, extend with another robot |
| How far along do we need to be for C2? | Avoid falling behind stakeholder/tutor expectations | Need proof the project is coming along: screenshots of a custom environment; robot sensing and moving; avoiding obstacles while reaching the goal |

---

## Generative AI

### Use declared for C1

| Tool | Task | How output was used | How checked |
|---|---|---|---|
| Chat GPT | Help brainstorm names for project ideas | Generated an image used to inspire names suited to the image and project scope | Team brainstormed, polled top 3 names, narrowed to 1: Bushfire Environmental Evacuation & Rescue (`B.E.E.R.`) |

C1 template also states: generative AI may support brainstorming, grammar, clarity, formatting, coding assistance, or technical explanation. Project direction, decisions, evidence, and submitted work must be understood and accepted by the team. Students must be able to explain their work.

### Planned protocol

**Principle:** Use generative AI selectively as a support tool, not as a substitute for the team’s technical development and decision-making.

**Planned uses**

- Debugging assistance and interpretation of error messages for ROS 2 nodes and simulation code
- Boilerplate and scaffolding (launch files, message/service definitions, basic node structures), then reviewed, modified, and tested by the team
- Drafting and refining documentation (this proposal, meeting minutes, later portfolio submissions)
- Explaining unfamiliar concepts, libraries, or ROS 2/Gazebo functionality for learning goals
- Generating test cases or edge-case scenarios for evaluation

**Checks**

- All AI-generated or AI-assisted code reviewed line-by-line by at least one team member before merge
- AI-suggested technical claims, algorithms, or design recommendations independently verified against official documentation (ROS 2, Nav2, Gazebo) or by direct simulation testing
- Every team member must explain, in their own words, any code or design decision they contribute

**Documentation of use**

- Brief note of tool, purpose, and extent of modification in the decision/action log and, where relevant, in code comments or commit messages

**Explicitly inappropriate**

- Fabricating test results, simulation evidence, or stakeholder communications
- Generating entire subsystems without team members understanding their function
- Producing final report content without critical review and originality checks

**External constraint from Nadim:** packages from the internet and genAI may help implement/code the project as long as the team understands the code and can adjust and explain it on the fly.

---

## Open Issues and Conflicts

### Unresolved source conflicts

| Topic | Value A | Value B | Resolution |
|---|---|---|---|
| C1 section 2.6 title | TOC: “Key Challenges” (printed page 5) | Body: “Scope Boundaries and Assumptions” (printed page 7). No standalone Key Challenges body section found. | Unresolved. Treat **Scope Boundaries and Assumptions** as authoritative 2.6 content. |
| Repository host | Workflow text: shared **GitHub** repository | Figure 3: Gitea-style UI for `jackthebugger / RS1-Gr25` | Unresolved. Both descriptions preserved. |
| Simulator naming | Component plan: **Ignition Gazebo** | Risk table: **Gazebo/Isaac** | Unresolved. Both names preserved. |
| Visualisation naming | Diagram narrative: **RVis2** | Figure 2 window: **RViz2**; Faiyad skills: **RViz** | Canonical retrieval term: RViz2. Preserve `RVis2` as source spelling. |
| Safe-location wording | Scope: safe house or fire trail | Stakeholder decision: one designated safehouse / spot rather than variable fire trail | Later stakeholder decision narrows the semester goal; long-term/scope text still mentions fire trail. |
| Architecture inventory | Diagram blocks: Operator Interface, Decision Making, Control, Simulation Testing | Component plan rows: Mission Manager, Map, Obstacle Detection Module, etc. | Unresolved gap; see [Architecture vs component-plan gap](#architecture-vs-component-plan-gap). |

### Incomplete records

1. Team log `18/08/2026`: C1 finalisation and role assignment discussed; **outcome blank**
2. System diagram: some grey connector labels may have additional text not fully legible at extraction resolution; core block names and major flows confirmed
3. Assigned C2 speaker identity not stated in original `Presentation_Slides.md` — **resolved:** Jack Havranek (movement / path planning)
4. Thermal sensor integration into path planning — **planned / in progress** (Jack); not in repo at `2026-08-28`
5. Gantt phases 2–5 are collapsed; full task lists for those phases are not in the sources

### Source wording artefacts (not silently corrected)

- “identity the safest route” (candidate-application challenge)
- “diminising/rare flora”
- “Priorise areas” (long-term ConOps)
- “fire fire” (system-diagram narrative)
- “Adding sim pakage to repo”
- “then further or project with another robot” (priority-questions response)
- “THis will be my main focus” (Jack skills, original PDF)
- Duplicate parenthesis `BEER)`

### Extraction uncertainties (from C1 PDF extraction)

1. TOC 2.6 vs body 2.6 (above)
2. Blank team-log outcome for `18/08/2026`
3. Partial illegibility of some system-diagram connector labels

---

## Visual Assets

| Asset | Type | PDF page | Printed page | Retrieval notes |
|---|---|---|---:|---|
| Cover image | Satellite/aerial photograph | 1 | — | Annotated smoke, active fire (infrared heat signature), burned area; scale bar **200 m**; north arrow **N** |
| System diagram | Block diagram | 11 | 10 | See [Architecture](#architecture) |
| Husky illustration | Annotated UGV drawing | 11 | 10 | Hardware labels listed under Architecture |
| Figure 1 | Gantt chart snapshot | 15 | 14 | Phase 1 Foundation & Setup through C1 due 28 Aug |
| Figure 2 | Gazebo + RViz2 screenshot | 17 | 16 | Caption: “Simulated environment successfully loaded” |
| Figure 3 | Git web UI screenshot | 17 | 16 | Caption: “Screenshot of shared git repository” |

### Cover image contents

| Element | Description |
|---|---|
| Unburned forest | Dense dark green canopy |
| Burned area | Large dark/black charred region (center-left) |
| Active fire front | Glowing red/orange perimeter at the burned-area edge |
| Smoke | Greyish-white haze above canopy (top right) |
| Label: Smoke | Points to hazy areas above canopy |
| Label: Active fire (infrared heat signature) | Points to glowing red/orange perimeter |
| Label: Burned area | Points to dark charred forest |
| Scale | 200 m |
| Orientation | North arrow N |

---

## Source Provenance

### Source files

| File | Role |
|---|---|
| `C2_Stakeholder_Presentation/Presentation_Task.md` | C2 assessment brief, delivery rules, rubric |
| `C2_Stakeholder_Presentation/Presentation_Slides.md` | Jack Havranek’s C2 slides 1–3 (movement / path planning) |
| `pathplanning_and_movement_implementation.md` | Navigation implementation contract and log (`2026-08-28`) |
| `master_robot_movement_pathplanning.md` | Repo reverse-engineering for movement, sensing, Nav2 |
| `scripts/basic_autonomy_demo.py` | Autonomous demo entry point |
| `rs1_nav/` | Mission runner, nav observer, path blocker modules |
| `C2_Stakeholder_Presentation/Group25_C1_Proposal_llm_optimised.md` | Structured extraction of `Group25_C1_Proposal.pdf` |
| `C2_Stakeholder_Presentation/Group25_C1_Proposal.pdf` | Authoritative C1 proposal (22 PDF pages; 21 printed pages in footers) |

### C1 PDF metadata (from extraction)

| Field | Value |
|---|---|
| PDF metadata title | `41068_C1_proposal_document-3` |
| Producer | Skia/PDF m153 Google Docs Renderer |
| PDF pages | 22 |
| Printed pages | 21 (PDF page 22 shows printed page 21) |
| Extraction methods | PyMuPDF, pdfplumber, visual page inspection |
| OCR | Not required for body text |
| Equations | 0 |
| Requirements catalogued | 10 (R1–R10) |
| Technical risks catalogued | 6 |
| Intermediate extraction assets | `/tmp/pdf_extract_group25_c1/` |

### C1 original section map (for PDF lookup)

| C1 heading | PDF pages | Printed pages |
|---|---|---|
| 1 Team and Collaboration | 3–4 | 2–3 |
| 2 Project Direction | 5–8 | 4–7 |
| 3 System Design | 9–14 | 8–13 |
| 4 Planning and Current Evidence | 15–17 | 14–16 |
| 5 Communication | 18–20 | 17–19 |
| 6 Generative AI | 21–22 | 20–21 |
