# Bushfire Environmental Evacuation & Rescue (B.E.E.R.) — Component 1 Proposal Document

## Document Overview

- **Document type:** University coursework proposal (Component 1) for 41068 Robotics Studio 1
- **Course:** 41068 ROBOTICS STUDIO 1
- **Project title:** Bushfire Environmental Evacuation & Rescue (B.E.E.R.)
- **Group number:** 25
- **Author(s):** Phu Huynh, Taj Wilcock, Faiyad Hassan, Jack Havranek (Group 25)
- **Organisation:** University of Technology Sydney (UTS) — inferred from student email domains (`@student.uts.edu.au`)
- **PDF filename:** `Group25_C1_Proposal.pdf`
- **PDF metadata title:** `41068_C1_proposal_document-3`
- **PDF producer:** Skia/PDF m153 Google Docs Renderer
- **PDF pages:** 22
- **Printed document pages:** 21 (footer page numbers on most content pages; PDF page 22 shows printed page 21)
- **Primary subject:** Autonomous ground robot (Clearpath Husky) for bushfire evacuation route finding in simulation
- **Purpose:** Propose project direction, system design, planning, communication approach, and current evidence for a semester robotics studio project responding to forest management / environmental monitoring needs

### Team Members

| Student Number | Full Name | Email |
|---|---|---|
| 25461649 | Phu Huynh | phu.q.huynh@student.uts.edu.au |
| 25355759 | Taj Wilcock | taj.g.wilcock@student.uts.edu.au |
| 25420973 | Faiyad Hassan | Mohammad.F.Hassan@student.uts.edu.au |
| 25470117 | Jack Havranek | jack.n.havranek@student.uts.edu.au |

### Key Topics

- Bushfire evacuation and rescue robotics for SES / firefighting crews
- Autonomous Husky navigation in simulated bushfire environments
- Perception (LiDAR, thermal/IR, carbon soot sensors) and hazard avoidance
- ROS 2, Gazebo/Ignition simulation, custom bushfire world design
- Mission management, operator UI, path planning, and multi-robot stretch goals
- Project planning, risk management, stakeholder communication, and generative AI protocol

---

## Contents

1. [Team and Collaboration](#1-team-and-collaboration)
   - 1.1 [Team Capabilities and Learning Goals](#11-team-capabilities-and-learning-goals)
   - 1.2 [Team Operating Agreement](#12-team-operating-agreement)
2. [Project Direction](#2-project-direction)
   - 2.1 [Candidate Applications](#21-candidate-applications)
   - 2.2 [Chosen Application and Stakeholder Need](#22-chosen-application-and-stakeholder-need)
   - 2.3 [Related Work](#23-related-work)
   - 2.4 [Long-Term Concept of Operations](#24-long-term-concept-of-operations)
   - 2.5 [Concept of Operations – For 41068 Robotics Studio 1](#25-concept-of-operations-for-41068-robotics-studio-1)
   - 2.6 [Scope Boundaries and Assumptions](#26-scope-boundaries-and-assumptions)
   - 2.7 [Project Title](#27-project-title)
3. [System Design](#3-system-design)
   - 3.1 [System Requirements](#31-system-requirements)
   - 3.2 [System Diagram](#32-system-diagram)
   - 3.3 [Component Plan](#33-component-plan)
   - 3.4 [Simulation and Evaluation Environment](#34-simulation-and-evaluation-environment)
   - 3.5 [Technical Risks and Early Tests](#35-technical-risks-and-early-tests)
4. [Planning and Current Evidence](#4-planning-and-current-evidence)
   - 4.1 [Development and Integration Workflow](#41-development-and-integration-workflow)
   - 4.2 [Milestones, Gantt Chart and Immediate Next Steps](#42-milestones-gantt-chart-and-immediate-next-steps)
   - 4.3 [Roles and Responsibilities](#43-roles-and-responsibilities)
   - 4.4 [Current Technical Progress](#44-current-technical-progress)
5. [Communication](#5-communication)
   - 5.1 [Team Decision and Action Log](#51-team-decision-and-action-log)
   - 5.2 [Stakeholder Communication Log](#52-stakeholder-communication-log)
   - 5.3 [Priority Stakeholder Questions](#53-priority-stakeholder-questions)
6. [Generative AI](#6-generative-ai)
   - 6.1 [Generative AI Use Declaration](#61-generative-ai-use-declaration)
   - 6.2 [Planned Generative AI Protocol](#62-planned-generative-ai-protocol)

---

## Figure Index

| Figure | Description | PDF Page | Printed Page |
|---|---|---:|---:|
| Cover image | Satellite/aerial bushfire imagery with annotated smoke, active fire, and burned area | 1 | — |
| Figure 1 | Snapshot of project Gantt chart (Phase 1: Foundation & Setup) | 15 | 14 |
| Figure 2 | Simulated environment successfully loaded (Gazebo + RViz2) | 17 | 16 |
| Figure 3 | Screenshot of shared Git repository (`RS1-Gr25`) | 17 | 16 |
| System diagram (Section 3.2) | Block diagram of sensing, perception, decision-making, control, and future drone integration | 11 | 10 |

## Table Index

| Table | Description | PDF Page | Printed Page |
|---|---|---:|---:|
| Team details | Student numbers, names, emails | 1 | — |
| Team capabilities | Skills, learning goals, intended contributions | 3 | 2 |
| Candidate applications | Five proposed use cases with feasibility ratings | 5 | 4 |
| Related work | Existing systems, strengths/limitations, project relevance | 6–7 | 5–6 |
| Scope boundaries | System will/will not, assumptions, constraints | 8 | 7 |
| System requirements R1–R10 | MVP vs stretch requirements with verification methods | 9–10 | 8–9 |
| Component plan | Nine system components with leads and I/O | 11–12 | 10–11 |
| Technical risks | Six risks with tests, mitigations, owners | 13–14 | 12–13 |
| Roles and responsibilities | Twelve activity areas with lead/support | 16–17 | 15 |
| Team decision log | Internal team decisions (partial) | 18 | 17 |
| Stakeholder communication log | Tutor/stakeholder discussions | 18–19 | 17–18 |
| Priority stakeholder questions | Three questions with responses | 20 | 19 |
| Generative AI declaration | ChatGPT use for project naming | 21 | 20 |

## Visual Content Summary

### Photographs / Satellite Imagery
- **Cover image (PDF p. 1):** Forested landscape with active bushfire front, burned area, and smoke plumes; scale bar 200 m; north arrow present

### Diagrams
- **System architecture diagram (PDF p. 11):** Operator interface, sensors, perception, decision-making, control, simulation validation, future drone and multi-robot paths
- **Husky hardware illustration (PDF p. 11):** Annotated UGV with LiDAR, thermal imager, IR sensors, GPS, processor, acoustic and gas sensors

### Charts / Schedules
- **Gantt chart (Figure 1, PDF p. 15):** Phase 1 Foundation & Setup schedule through Component 1 proposal due date

### Screenshots
- **Figure 2 (PDF p. 17):** Gazebo forest environment with Husky; RViz2 showing LiDAR, map, paths, camera, depth point cloud
- **Figure 3 (PDF p. 17):** Gitea/Git web UI for repository `jackthebugger / RS1-Gr25`

---

## Conflicting Information

### Table of Contents vs actual section 2.6

- **Source A (TOC, PDF p. 2):** Section **2.6 Key Challenges** listed at printed page 5
- **Source B (body, PDF p. 8):** Section **2.6 Scope Boundaries and Assumptions** appears at printed page 7
- **Conflict:** The TOC references "Key Challenges" but no standalone "Key Challenges" section exists in the document body; scope boundaries occupy section 2.6
- **Resolution:** Unresolved in source PDF — treat **Scope Boundaries and Assumptions** as the authoritative section 2.6 content

### Project acronym spelling

- **B.E.E.R.** — used in title and project title section
- **BEER** — used in candidate applications table and stakeholder log
- **BEER)** — typo with extra parenthesis in stakeholder log (PDF p. 18)
- **Resolution:** All refer to the same project; canonical form in title is **B.E.E.R.**

### Simulation tool naming

- Document references **Ignition Gazebo Simulation Environment** and **Gazebo/Isaac** in risks section
- Document references **RVis2** in system diagram narrative — likely intended as **RViz2** (confirmed by Figure 2 screenshot showing RViz2)

---

# 1. Team and Collaboration

> **Source:** PDF pp. 3–4 | Printed pp. 2–3

## 1.1 Team Capabilities and Learning Goals

| Team member | Relevant experience or existing skills | Skills they want to develop | Intended areas of contribution |
|---|---|---|---|
| Phu | Proficient in Python, C++, and Java, with hands-on experience using SolidWorks for CAD design and ROS2 for robotics development. Strong teamwork and communication skills developed through past collaborative technical projects, enabling effective coordination across both software and hardware aspects of a project. | Develop skills in the ROS2 CORE and virtual simulation, which extends to virtual physics, virtual world design, as well as robotic design and implementation. | Mainly the physics behind robotic movement within the virtual environment but also wherever else I am required |
| Taj | Previous experience with communication and team management working in team projects at UTS in previous semesters, as a leader and as a regular member. From previous classes has used Python and C++ as well as some experience with ROS2. | Further development of general coding skill as well as a deeper understanding of ROS2. | Wherever needed by the team and the project. |
| Faiyad | Experience with multiple programming languages, including Python, C++, C# and Java. Familiar with ROS 2 and RViz, as well as CAD design using SolidWorks. | Further develop skills in team communication and collaboration, ROS 2, RViz, system integration, and practical robotics development. | Happy to contribute across all areas of the project, particularly software development, ROS 2, CAD/mechanical design, system integration, testing, troubleshooting and documentation. |
| Jack | Semi proficient in Python and C++ through experience with other coding and robotics projects. Also strengths in team collaboration and communication to achieve a united goal. | Lacks experience with ROS2 which is believed essential in engineering career. This will be main focus area this semester. | Wherever needed. |

**Observation:** All four team members identify ROS 2 / simulation as a development area; this is later cited as a project delivery risk (Section 3.5).

## 1.2 Team Operating Agreement

### Team Expectations

- Attend weekly meetings
- Complete assigned tasks before internal deadlines
- Respond to team communication within 48 hours
- Contribute meaningfully to all group deliverables

### Work Contribution & Accountability

- Tasks tracked via shared Kanban board
- Missed deadlines must be communicated in advance
- Repeated lack of contribution will trigger the conflict process
- *If contribution remains insufficient after internal discussion, the issue will be escalated to the coordinator with documented evidence.*

### Conflict Resolution Process

1. Internal discussion
2. Documentation of issue
3. Escalation to subject coordinator (if unresolved)

### Academic integrity

- All work must be original
- No plagiarism
- Each member must be able to explain their subsystem

### Commitment Statement

We agree to work collaboratively, professionally, and ethically to deliver a high-quality system.

---

# 2. Project Direction

> **Source:** PDF pp. 5–8 | Printed pp. 4–7

## 2.1 Candidate Applications

**Guidance in source:** Suggested length 50 words per use case. Identify 5 possible applications aligning with the Project Vision in the Project Brief. Applications may be practical or speculative; should respond to meaningful forest management or environmental monitoring needs.

| Candidate application | Intended user or stakeholder | Need or potential value | Main technical challenge | Feasibility this semester |
|---|---|---|---|---|
| Habitat Analysis | National park rangers, researchers | Automate search for endangered or invasive plant species for faster and more targeted bush regeneration and protection. | Ability to identify a wide range of plant species reliably and efficiently | MEDIUM |
| Bushfire Environmental Evacuation & Rescue (BEER) | Firefighters, SES | Deployable automated robot to help identify escape routes and lead firefighters when trapped in a bushfire. Considering fire intensity, size of firetruck and terrain difficulty the robot will find an optimal path to safety. | Assessing a wide range of variables simultaneously to identity the safest route efficiently. | HIGH |
| Bushfire priority identification | Firefighters, SES | Uses sensors (IR heat mapping, LiDAR, carbon soot) on drones/huskies to determine which fires/areas are hottest / highest chance of spreading. | Sensors for fire detection and accurate priority determination | MEDIUM |
| Seed Spreader | Farmers/Agriculture, forester, silviculturists | Increase population of diminishing/rare flora; helps repopulate and increase numbers of said flora. | Healthy soil detection and planting processes. | MEDIUM |
| Missing Human Rescue | SES, police, Private Investigator | Allowing use of fewer resources and law enforcement officers for "Man hunt". | Ability to find well hidden and deceased bodies. | LOW |

**Decision:** Bushfire Environmental Evacuation & Rescue selected as chosen application (Section 2.2).

## 2.2 Chosen Application and Stakeholder Need

**Application name:** Bushfire Environmental Evacuation & Rescue (BEER)

### Problem

When a firetruck team becomes stuck, trapped, or lost in a bushfire (encircled by fire, disoriented by heavy smoke, or similar), crews need a safe evacuation path.

### Proposed solution

Deploy an automated robot to find a safe path to a safe location.

### Functional requirements (conceptual)

- Reduce risk from prolonged exposure to hazardous bushfire conditions by finding the quickest, least dangerous path
- Equip robot with sensing equipment not usually available to firetruck crews for navigation through smoke and fire
- Ensure identified path is suitable for the firetruck as well as the robot
- Consider variables: amount of space, heat, surface incline — within acceptable range

### Adoption and extended use

- Could be added to existing firefighting fleet with minimal modifications
- Beyond rescue: scouting fires in buildings and bushfires; potential fire suppression from within the fire without endangering firefighters

### Selection rationale

- Most interesting idea among candidates
- Most potential for additional advanced functions beyond basic functionality
- Feasible for team skills with room for advanced additions for higher marks

## 2.3 Related Work

| Existing research, product or system | What it does | Relevant strengths or limitations | How it informs your project |
|---|---|---|---|
| SES Remotely Piloted Aircrafts — https://statements.qld.gov.au/statements/100929 | Search and rescue, disaster assessment, direct communication, emergency resupplies. | High efficiency compared to walking or driving. Health and safety concerns reduced. **Not automated** (human operated). | Technology exists in this field already. Automation is not yet used. |
| Ground Based Firefighting Robots — https://www.hcrot.com/what-robots-help-firefighters | Safely enter burning buildings with firefighting tools (water jets, foam dispensers); fires targeted with thermal imaging cameras. Controlled remotely by operator. | Heat resistance — can function in high temperature environments. **Not automated.** | A robot operating automatically in a high temperature environment such as a bushfire is entirely possible. |
| Firefighting Co-bots — https://news.griffith.edu.au/2026/02/16/ai-powered-robot-vehicles-team-up-to-fight-fires/ | Robot-human collaboration for firefighting; AI-powered robots supervised by human operator; AI handling some tasks autonomously reduces cognitive load and stress of human operator. | Human–robot collaboration being investigated as serious addition to firefighting practices. Still in research stage — not fully implemented in the real world. | Using robotics for firefighting and related purposes is being seriously considered in related fields, giving validity to project ideas. |

## 2.4 Long-Term Concept of Operations

**Time horizon:** Approximately five years

**Vision:** Coordinated autonomous bushfire monitoring platform used by organisations such as NSW SES, fire services, and national park authorities.

**Operating environment:** Forests, mountains, bushland during high-risk fire conditions or active bushfire events.

**Platform:** Husky robot equipped with LiDAR, cameras, temperature sensors, and localisation systems deployed to investigate priority locations; eventually finding optimal path to rescue trapped civilians with human survival as number one priority.

**Autonomous behaviour:**

- Analyse sensor data
- Determine location and severity of hotspots
- Assess surrounding hazards
- Priorise areas requiring urgent attention
- Select safe navigation paths
- Continuously update route as environmental conditions change

**Human operator role:**

- Supervise via central interface showing robot locations, hotspots, risk levels, navigation paths, live sensor information with interactive GUI
- Assign new objectives or override autonomous decisions when required

**Stakeholder value:**

- Faster and safer situational awareness for emergency responders
- Reduced need for personnel to enter hazardous areas
- More informed bushfire response decisions

## 2.5 Concept of Operations – For 41068 Robotics Studio 1

### Scenario setup

- SES truck positioned at simulated bushland location representing last known position of disoriented/trapped firefighting crew (heavy smoke, encroaching fire)
- Environment: clear ground, vegetation, simulated fire hazards
- Defined safe location (fire trail or safehouse) set as mission goal

### Mission flow

1. SES truck deploys onboard Husky robot to lead truck to safety point
2. Husky uses simulated sensors (LiDAR, camera, carbon soot, IR heat) to perceive surroundings including static obstacles (trees, dense bushland) and fire zones
3. Robot continuously scans for obstacles intersecting planned path and identifies simulated fire regions to avoid
4. System evaluates paths toward goal; selects route avoiding hazards with clearance margin approximating space required by following firefighting vehicle
5. If obstacle or fire blocks path, robot re-plans local detour and continues toward goal
6. Robot position, planned path, and detected hazards displayed on simple operator UI
7. Operator initiates mission and observes progress (representing crew that would follow robot in full-scale application)

### Scenario end / success criteria

- Robot reaches designated safe location
- Autonomously navigates around all static obstacles and hazards
- No collisions during traversal
- UI accurately reflects real-time position and path
- System correctly identifies and avoids all simulated fire hazards in environment

## 2.6 Scope Boundaries and Assumptions

| Category | Current definition |
|---|---|
| **The system will…** | Autonomously move toward a designated safe location, such as a safe house or a fire trail. When an obstacle or fire hazard blocks its path, the Husky will locally avoid the hazard and continue toward the designated safe location where a traversable route exists. |
| **The system will not…** | Extinguish fires, carry loads, or physically transport/rescue people. |
| **Environmental assumptions** | Terrain sufficiently traversable for Husky movement; at least one safe path to designated safe location exists. |
| **Robot or sensor assumptions** | Husky simulated sensors reliably detect relevant obstacles and fire hazards within defined sensing range; localisation data sufficiently accurate for navigation. |
| **User or operator assumptions** | Operator can initiate mission and specify designated safe location. Firefighting crew or vehicle assumed capable of following route identified by Husky. |
| **Simulation simplifications** | Fire intensity, vegetation/tree density, terrain complexity and sensor behaviour simplified compared with real bushfire conditions. Dynamic fire spread may be implemented as stretch feature. |
| **Other important constraints** | System operates entirely in simulation during 41068. MVP navigation uses local/reactive hazard avoidance rather than full dynamic global route optimisation. Performance limited by provided Husky platform and available simulated sensors. |

## 2.7 Project Title

**Title:** Bushfire Environmental Evacuation & Rescue (B.E.E.R.)

**Rationale:**

- Reflects core purpose: guiding trapped or disoriented SES crews to safety during bushfire emergencies
- "Evacuation & Rescue" communicates mission-critical function
- "Bushfire Environmental" grounds project in target domain and operating conditions
- Acronym B.E.E.R. is short, memorable, easy for technical and non-technical stakeholders; maps meaningfully to full name

---

# 3. System Design

> **Source:** PDF pp. 9–14 | Printed pp. 8–13

## 3.1 System Requirements

| ID | Category | Requirement | Why is it important? | How will it be verified or demonstrated? | Priority |
|---|---|---|---|---|---|
| R1 | Decision making | The Husky robot shall autonomously navigate from its starting position to an operator-defined safe goal location without direct manual driving. | Fundamental mobility for environmental monitoring and rescue missions | Observe multiple waypoints and assign itself to find the best path. Robot reaches end goal in simulation without direct manual driving. | MVP |
| R2 | Perception | The system shall detect simulated fire hazards within the sensing range of the Husky and report their locations to the decision-making system. | Enables identification of hazardous areas; meaningful bushfire-response perception | Place fire truck with Husky in middle of forest fire; robot finds and guides fire truck out to safety. | MVP |
| R3 | Perception | The Husky robot shall detect nearby obstacles using its simulated onboard sensors. | Prevents collisions with trees, debris, terrain and environmental obstacles during autonomous operation | Place obstacles along terrain on robot's route; demonstrate detection by perception system. | MVP |
| R4 | Decision making | When an obstacle blocks the current route, the Husky shall avoid the obstacle and continue progressing toward the designated safe location where a traversable route exists. | Demonstrates perception influencing autonomous decision making and mission safety | Introduce obstacle in planned route; observe stop or modified motion to avoid collision. Change fire position so alternate path required. | MVP |
| R5 | User interface | The user interface shall display the robot's current position, mission waypoints and detected fire hotspots. | Operator situational awareness; quick interpretation of progress and hazards | Run mission; verify position, waypoints and hotspots visibly represented on interface. | MVP |
| R6 | User interface | The interface shall display the current mission status, including whether the robot is idle, navigating, responding to a hazard or has completed its mission, while also providing real time data. | Operator understands autonomous system state without directly observing robot | Change robot between mission states; confirm displayed status updates appropriately. | Stretch |
| R7 | Simulation | Simulated vegetation shall support healthy, burning and burnt states, with state transitions triggered by the fire simulation. | Meaningful dynamic bushfire environment; perception data for robotic system | Trigger simulated fire; visually demonstrate trees changing colour/appearance between three states. | Stretch |
| R8 | Perception | The system shall generate and update a map containing environmental features, robot position and identified hazardous regions. | Richer disaster environment representation for planning and operator awareness | Demonstrate map generated or updated as sensor information received during simulated mission. | Stretch |
| R9 | Decision making | The system shall automatically calculate an evacuation or rescue route that avoids known fire hotspots and obstacles. | Extends system from basic navigation to mission-level autonomous bushfire response | Scenario with hazards between robot and destination; demonstrate safe alternative route generation. | MVP |
| R10 | Multi-robot system | An aerial robot shall survey the environment and communicate detected fire or hazard locations to the ground robot. | Aerial sensing covers larger area; improves ground robot situational awareness beyond local sensors | Demonstrate UAV detecting hazard and corresponding location appearing in ground robot mission information or map. | Stretch |

**MVP requirements:** R1, R2, R3, R4, R5, R9  
**Stretch requirements:** R6, R7, R8, R10

## 3.2 System Diagram

> **Source:** PDF p. 11 | Printed p. 10

**Type:** System architecture block diagram with Husky hardware illustration

### Narrative (from source)

This system diagram shows the integration of real time environmental sensing with adaptive motion control for autonomous operations. It will be activated via an operator GUI then the onboard hardware feeds continuous data to a perception processor for fire fire and obstacle detection and from that navigate the environment. The decision making core will take this data and output motion control to physically execute moving functions that will navigate to a safe location. All essential system info will be stored and displayed through the GUI for testing and debugging. Secondary robots may be included and will communicate with the husky to navigate the environment faster and more efficiently. All testing will be done in the custom RVis2 environment with increasingly challenging environments.

**Note:** Source text contains duplicate "fire fire" and "RVis2" (likely RViz2).

### System architecture — elements and relationships

#### Operator Interface (red block)

- **Inputs from operator (grey):** User input: Activation; User input: Safehouse location; Robot speed
- **Outputs to:** Decision Making; receives feedback from Simulation Testing and Validation

#### Sensors (red block)

- Connected from Operator Interface
- **Sub-modules (yellow):** LiDAR; Thermal Camera; Infrared Sensors; CO Sensor
- **Output:** Sensor Data Storage and Processing → Perception

#### Perception (red block)

- **Inputs:** Sensor Data Storage and Processing
- **Sub-modules (yellow):** Fire Detection; Obstacle Detection; Mapping and Nav
- **Outputs to:** Decision Making

#### Decision Making (red block — central hub)

- **Inputs:** Perception; Operator Interface
- **Sub-modules (yellow):**
  - Local Reactive Planning (From Environment)
  - Global Path Finding (From User Input)
- **Outputs (grey labels):** Speed, Rotation, Position, Status, Battery → Control and Simulation Testing and Validation

#### Control (red block)

- **Sub-module (yellow):** Motion Control
- **Feedback:** Motion Control feeds back to physical environment / robot

#### Simulation Testing and Validation (red block)

- **Sub-module (yellow):** Scenario Test in Virtual Environment (RVis2)
- Connected to Operator Interface and Decision Making outputs

#### Future implementations

- **Drone Implementation for Global Path Identification (red):** Dotted connections to Decision Making and Perception; label: "Dynamic environment → Adaptive decision making"
- **Future Multi Robot Implementation and Communication (grey):** Between Perception and Husky illustration

### Husky hardware illustration — visible labels

| Label | Description |
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

**Platform type:** Four-wheeled unmanned ground vehicle (UGV)

### Relationship summary (flow)

```
Operator → Sensors → Perception → Decision Making → Control → Robot motion
                ↓                      ↓
         Simulation Testing ← Operator Interface (display/debug)
                ↑
    [Future: Drone → Decision Making / Perception]
    [Future: Multi-robot communication with Husky]
```

**Gap note:** System diagram includes Operator Interface, Decision Making sub-modules, Motion Control, and Simulation Testing blocks that are **not** listed as separate rows in the Component Plan (Section 3.3). Component Plan lists Mission Manager, Map, and Obstacle Detection Module as custom modules that partially cover these functions.

## 3.3 Component Plan

| Component | Purpose | Inputs | Outputs | Provided, existing, adapted, or custom? | Initial lead and support team members |
|---|---|---|---|---|---|
| Husky | Primary mobile platform to travel through simulated bushfire environment and investigate hazardous areas. | Velocity/motion commands; simulated environment interactions | Odometry, robot pose and sensor data | Provided | Phu |
| Ignition Gazebo Simulation Environment | Simulates bushland environment, robot movement, sensors, obstacles and fire-related scenarios. | World configuration, robot commands, model states | Simulated sensor data and visual environment | Provided | Taj |
| Robot Sensors | Information about surroundings for obstacle and hazard detection; may include LiDAR and other Husky model sensors. | Simulated environment | Range measurements, distance from goal and sensor messages | Provided and out sourced | Phu |
| Bushfire Environment / Tree Models | Vegetation and environmental obstacles within simulated disaster area. | Initial world configuration and fire-state commands | Visual and state information for trees and obstacles | Adapted | Taj |
| Dynamic Fire-State System | Controls transitions of simulated vegetation between healthy, burning and burnt states. | Fire trigger, current tree state, neighbouring fire conditions or timers | Updated tree state and visual appearance | Custom | Taj |
| Obstacle Detection Module | Determines whether trees, debris or other objects obstruct robot's planned path. | LiDAR/range sensor data | Obstacle locations, distance and hazard flags | Custom | Jack |
| Map | Representation of robot, mission waypoints, obstacles and detected fire hazards. | Robot pose, sensor observations, hotspot locations | Updated environment/hazard map | Custom | Faiyad |
| Mission Manager | Coordinates overall mission; tracks states: idle, navigating, hazard detected, mission complete. | Operator commands, navigation status, hazard detections | Mission state, active goal and system commands | Custom | Faiyad |
| ROS 2 Communication Layer | Software modules exchange sensor data, commands, mission states and detected hazards. | ROS 2 topics, services and messages | Inter-node communication | Existing Framework | Jack |

## 3.4 Simulation and Evaluation Environment

**Intent:** Create custom simulation environment showcasing robot abilities and fitting the project scenario.

**Reuse of provided assets:** Tree models and ground textures may be carried over.

**Custom implementation:** Terrain and scenario additions — hills, trenches, dense and sparse forest, fires, dead ends.

**Stretch goal:** Dynamic fire spreading across simulation environment.

**Primary robot:** Husky — majority of sensing and navigation; stretch goal of aerial drone support for optimised routing.

**Provided assets:** Husky models, sensors, sim physics utilised but expanded — more robust navigation accounting for obstacles and following firetruck needs.

**Additional sensors:** Thermal sensors implemented to assist robot task and provide additional operator information.

## 3.5 Technical Risks and Early Tests

| Risk or uncertainty | Why it matters | Early test or evidence needed | Mitigation or fallback plan | Owner |
|---|---|---|---|---|
| Path planning algorithm may fail to reliably find safe route around dynamic fire hazards and static obstacles simultaneously | Without reliable replanning robot cannot complete primary mission of guiding crew to safety | Early test of basic global planner in simple simulated environment with single static obstacle before adding dynamic fire hazards | Fall back to simpler reactive obstacle-avoidance behaviour (e.g. stop-and-detour) if full dynamic replanning too complex within timeframe | Jack |
| Simulated sensors (IR heat, carbon soot, LiDAR) may not provide sufficiently accurate or distinguishable data to reliably detect fire hotspots vs. normal terrain | Inaccurate detection undermines perception and downstream decision-making | Early bench test in simulation: place known fire/hotspot regions; confirm sensor outputs distinguish them from clear terrain | If custom sensor modelling unreliable, simplify to known ground-truth hazard zone published directly by simulation for perception testing | Phu |
| Integration between perception, decision-making and control components may introduce unexpected bugs or latency when combined | Components may work in isolation but fail when passing data between ROS2 nodes in real time | Early integration test connecting minimal perception stub to navigation stack to confirm message flow and timing before full sensor logic | Maintain modular ROS2 nodes with clearly defined topics/interfaces so faulty components can be isolated and replaced | Jack |
| Custom simulation environment (dynamic fire behaviour, tree state transitions) may be difficult to build or too computationally expensive in Gazebo/Isaac | Without working dynamic environment, several stretch goals and realistic testing not possible | Early prototype of small number of trees transitioning between healthy/burning/burnt states to test feasibility and performance impact | If full dynamic fire simulation infeasible, fall back to static pre-placed hazard zones that still test avoidance behaviour | Taj |
| Team's limited prior experience with ROS2 Nav2 and simulation tools may slow development | All members identified ROS2/simulation as skill gap — project delivery risk | Early individual tutorials/small tasks (e.g. each member runs basic Nav2 demo) completed in week 1–2 to confirm baseline competency | Pair less experienced with more experienced members; allocate extra buffer time in Gantt for ROS2-related tasks | Jack |
| User interface may not update in real time with accurate robot position, path and hazard data | Delayed or inaccurate UI undermines operator situational awareness — key evaluation criterion | Early test publishing dummy position/hazard data to UI to confirm real-time display before connecting live sensor data | If real-time updates too complex, fall back to periodically refreshed display (e.g. every 1–2 seconds) rather than continuous streaming | Faiyad |

---

# 4. Planning and Current Evidence

> **Source:** PDF pp. 15–17 | Printed pp. 14–16

## 4.1 Development and Integration Workflow

### Version control

- Shared GitHub repository
- Each member works on feature branch (e.g. `feature/perception-module`)
- Direct commits to `main` not permitted
- Changes merged via pull requests after testing and review by at least one other member
- Integration agreed during scheduled check-ins

### Task tracking

- Kanban board (Microsoft Planner or Trello)
- Columns: "To Do," "In Progress," "Testing," "Done"
- Microsoft Teams for daily communication; key decisions documented in shared channel

### Interface specification

- Software interfaces defined early and documented in shared specification
- Updated and communicated via Teams when changes proposed

### Integration strategy

- Subsystems integrated into shared development branch as soon as test-ready (not end of semester)
- Weekly integration check-ins to test combined functionality, catch issues early, adjust plans

## 4.2 Milestones, Gantt Chart and Immediate Next Steps

### Immediate next steps (post-C1 submission)

1. Establish shared ROS 2/Gazebo development environment, Git workflow and integration process
2. Configure Husky platform and sensors
3. Develop custom simulation world
4. Scaffold core perception, decision-making and mission-management modules
5. Early integration checks for component communication and development workflows
6. Begin technical risk testing: sensor reliability, navigation, obstacle detection
7. Foundation for independent module testing, early issue identification, first working prototype

### Figure 1 — Snapshot of project Gantt Chart

> **Source:** PDF p. 15 | Printed p. 14  
> **Type:** Gantt chart screenshot (project management tool — appearance consistent with Notion or similar)

**Visible phase:** Phase 1: FOUNDATION & SETUP (expanded)

**Timeline window visible:** Approximately Wed 12 Aug – Sun 30 Aug (weeks 33–35)

**Current date marker:** Red vertical line on Tuesday 18 August

#### Phase 1 tasks and schedule

| Task | Subtasks / milestones | Schedule (from visual) | Status |
|---|---|---|---|
| Team formation, roles, etc. | — | Due 31/07/26 | Completed |
| Define ConOps & requirements | — | Finished ~16 Aug | Completed |
| Literature & related work review | — | Finished ~16 Aug | Completed |
| Initial system architecture | — | Finished ~16 Aug | Completed |
| | Develop high-level system block diagram | 12 Aug – 18 Aug | In progress at 18 Aug |
| | Identify hardware/software components | 13 Aug – 18 Aug | In progress at 18 Aug |
| | Define ROS 2 node architecture | 14 Aug – 18 Aug | In progress at 18 Aug |
| | Define module interfaces/data flow | 15 Aug – 18 Aug | In progress at 18 Aug |
| Proposal development | — | 16 Aug – 27 Aug | Ongoing |
| | Consolidate research and requirements | 17 Aug – 18 Aug | Today (18 Aug) |
| | Draft methodology/system architecture | 19 Aug – 21 Aug | Tomorrow |
| | Internal team review | 24 Aug – 25 Aug | Planned |
| | Finalise and proofread proposal | 26 Aug – 27 Aug | Planned |
| **MILESTONE: Component 1 Proposal** | **MILESTONE: Component 1 - Proposal Due** | **Friday 28 Aug** (red diamond) | Milestone |

#### Collapsed future phases (visible but not expanded)

- Phase 2: DEVELOPMENT ENVIRONMENT
- Phase 3: CORE COMPONENT DEVELOP…
- Phase 4: SYSTEM INTEGRATION & END…
- Phase 5: TESTING & VALIDATION

## 4.3 Roles and Responsibilities

| Activity or responsibility | Lead | Support | Expected contribution or outcome |
|---|---|---|---|
| Perception | Phu | Faiyad | Simulated sensor integration (LiDAR, camera, IR heat) for obstacle and fire detection, tuned and validated in simulation environment |
| Decision-making | Faiyad | Taj | Path planning and obstacle avoidance logic enabling Husky to navigate around hazards toward goal location |
| User interface | Faiyad | Jack | Operator interface displaying robot position, path, and detected hazards in real time |
| Simulation development | Taj | Jack | Custom bushfire environment including tree states (healthy/burning/burnt) and fire hazard zones |
| System integration | Jack | Faiyad | Combining perception, decision-making, and UI components into working end-to-end system |
| Testing and evaluation | Phu | Taj | Test cases for obstacle detection, path planning accuracy, and mission success criteria |
| Repository and software quality | All | All | Maintaining GitHub repository structure, pull request reviews, and coding standards |
| Stakeholder communication | All | All | Liaising with teaching staff acting as stakeholder; maintaining communication log |
| Project documentation | All | All | Compiling and updating proposal and design portfolio documents |
| Project management | Jack | Taj | Coordinating milestones, Kanban board, and meeting schedules |
| Hardware/Sensor configuration | Phu | Faiyad | Configuring simulated sensor parameters (range, noise, field of view) to reflect real-world Husky sensor specifications |
| Risk Management | Taj | Phu | Identifying, tracking, and updating technical risks and mitigation plans throughout project |
| CAD/Mechanical Design | Phu | Taj | Physical mounting or sensor placement design for Husky platform, modelled in SolidWorks where required |

## 4.4 Current Technical Progress

> **Source:** PDF p. 17 | Printed p. 16  
> **Date referenced:** Week 3 (11/08/2026)

### Completed work

- Provided simulated environment successfully loaded and configured
- Team familiarising with Husky platform simulated behaviour
- Shared Git repository established for version control and collaborative development

### Status assessment

- Groundwork laid for implementing and testing software changes in simulation
- No major technical risks identified at early stage
- Team will monitor simulation environment as complex features integrated

### Figure 2 — Simulated environment successfully loaded

> **Source:** PDF p. 17 | Printed p. 16  
> **Type:** Screenshot (Gazebo + RViz2 desktop)

**Gazebo window:**

- Green tiled ground plane
- Multiple realistic tree models
- Small yellow robotic vehicle (Husky platform)

**RViz2 window (foreground):**

- **Config path visible:** `/home/tajw/git/team25_rs1-Gr25/install/41068_ignition_bringup/share/41068_ignition_bringup/config/41068_husky1.rviz`
- **Active displays:** Grid, RobotModel, Lidar, Map, Global Path, Local Path, Camera, Depth PointCloud
- **Central view:** Robot perspective with white point cloud data (trees and ground)
- **Navigation 2 panel:** Navigation: unknown; Localization: unknown; Feedback: unknown
- **Terminal log prefix:** `[rviz2-16] [INFO] [1787088354.657...]`

**Interpretation:** Simulation stack loads; navigation/localisation not yet fully operational at time of screenshot.

### Figure 3 — Screenshot of shared Git repository

> **Source:** PDF p. 17 | Printed p. 16  
> **Type:** Web-based Git repository screenshot (Gitea-style UI)

| Field | Value |
|---|---|
| Repository | `jackthebugger / RS1-Gr25` (Public) |
| Description | RS1 Forest Management Project |
| Latest commit author | TajWilcockUTS |
| Latest commit message | Adding sim pakage to repo |
| Latest commit hash | `9d8c8df` |

**Visible directories/files:**

| Path | Last commit message | Time |
|---|---|---|
| `.vscode` | Adding sim pakage to repo | last week |
| `build` | Adding sim pakage to repo | last week |
| `install` | Adding sim pakage to repo | last week |
| `log` | Adding sim pakage to repo | last week |
| `src/41068_ignition_bringup_v1/...` | Adding sim pakage to repo | last week |
| `README.md` | Initial commit | last week |
| `hello` | hello (first commit OMG!) | last week |

**Navigation tabs visible:** Code, Issues, Pull requests, Agents, Actions, Projects, Wiki, Security and quality, Insights, Settings

---

# 5. Communication

> **Source:** PDF pp. 18–20 | Printed pp. 17–19

## 5.1 Team Decision and Action Log

**Purpose (from source):** Concise living record of internal discussions, decisions, changes, and actions affecting scope, technical approach, task allocation, or team operation.

| Date | Team members | Decision, issue, or discussion | Summary / outcome |
|---|---|---|---|
| 11/08/2026 | Taj, Faiyad, Phu, Jack | Team roles; which of 5 ideas is best; how to design environment to show sensing/navigating capabilities; current goal | Current scope goal: one robot navigates to set goal with static obstacles. Once basic sensing/self-navigating capabilities shown, introduce another robot and dynamically changing environment. |
| 18/08/2026 | Taj, Faiyad, Jack | Finalisation of C1 document; role assignment | [No outcome recorded in source] |
| — | — | (add more rows as necessary) | — |

## 5.2 Stakeholder Communication Log

**Purpose (from source):** Record discussions with teaching staff acting as project stakeholder (tutors and Subject Coordinator).

**Stakeholder named in log:** Nadim (tutor)

| Date | Team members | Decision, issue, or discussion | Summary / outcome |
|---|---|---|---|
| 11/08/2026 | Taj, Faiyad, Phu, Jack | Presented 5 ideas to Nadim; selected Bush-Fire Environmental Evacuation & Rescue (BEER). Initial ring-of-fire firetruck-to-fire-trail concept on right track but not fully showing sensor/path finding capabilities. Nadim suggested multiple obstacles after ring of fire and one designated safehouse goal instead of variable fire trail. | Multiple obstacles for robot to sense (LiDAR, heat mapping IR, carbon soot) and circumnavigate to designated safe house. Design environment around robot capabilities. Initially static fires with preset obstacles (static safe paths); later dynamic spreading fires (changing paths). Start with robot navigating stagnant obstacle environment; later introduce drone relaying path data from above (live/moving fires change path in real time). |
| 18/08/26 | Taj, Faiyad | Questions about C1 and C2: need to change Husky or Parrot models; C2 expectations; programming help | Not required to change Husky or Parrot models (can change if desired). C2 needs proof of progress including sim screenshots requiring sensing/path finding and sim environment modification. Nadim: LiDAR navigation could be difficult; suggest starting with thermal camera for fire project. Allowed to use internet packages and genAI to help implement/code — **as long as team understands code and can adjust and explain on the fly**. |

## 5.3 Priority Stakeholder Questions

**Guidance:** 2–5 substantive questions materially affecting scope, technical approach, feasibility, assumptions, or next steps.

| Question or Uncertainty | Why is this important to clarify? | Response received |
|---|---|---|
| Will our idea be sufficient to show off our robots capabilities as per criteria? | If idea barely shows sensing and self-navigating aspects, marks will be lost | Yes, idea is good, but need more obstacles for robot to sense and navigate after the ring of fire |
| Do we need to include 2 robots working together right off the bat? | Allows simpler initial design before advanced multi-robot implementation | No — not needed for first part; after basic sensing and self-navigation, can extend with another robot |
| How far along do we need to be for C2? | Ensure not falling behind stakeholder/tutor expectations | Need proof project is coming along: screenshots of custom environment, robot sensing and moving, avoiding obstacles while reaching goal |

**Note in source:** "Complete this in the next relevant assessment" — responses recorded above for questions discussed after proposal submission.

---

# 6. Generative AI

> **Source:** PDF pp. 21–22 | Printed pp. 20–21

## 6.1 Generative AI Use Declaration

| Tool | Task supported | How the output was used | How the output was checked or verified |
|---|---|---|---|
| Chat GPT | Help brainstorm names for project ideas | Gen AI outputted an image to inspire possible names suited to image and project scope | Team brainstormed list, ran poll on top 3 preferred names, narrowed to 1: 'Bushfire Environmental Evacuation & Rescue (B.E.E.R.)' |

## 6.2 Planned Generative AI Protocol

**Principle:** Generative AI used selectively as support tool, not substitute for team's technical development and decision-making.

### Planned uses

- Debugging assistance and interpretation of error messages when developing ROS 2 nodes and simulation code
- Boilerplate and scaffolding code (launch files, message/service definitions, basic node structures) — reviewed, modified, and tested by team
- Drafting and refining documentation (proposal, meeting minutes, portfolio submissions)
- Explaining unfamiliar concepts, libraries, or ROS 2/Gazebo functionality for learning goals
- Generating test cases or edge-case scenarios for component evaluation

### Verification checks

- All AI-generated or AI-assisted code reviewed line-by-line by at least one team member before merge
- AI-suggested technical claims, algorithms, or design recommendations verified against official documentation (ROS 2, Nav2, Gazebo) or direct simulation testing
- Every team member must explain in own words any code or design decision they contribute

### Documentation of AI use

- Brief note of tool used, purpose, and extent of modification recorded in decision/action log and code comments or commit messages as relevant

### Inappropriate uses (explicitly stated)

- Fabricating test results, simulation evidence, or stakeholder communications
- Generating entire subsystems without team members understanding their function
- Producing final report content without critical review and originality checks

---

## Cover Image — Bushfire Satellite/Aerial Imagery

> **Source:** PDF p. 1

**Type:** Satellite or aerial photograph of active bushfire in forested terrain

### Visible elements

| Element | Description |
|---|---|
| Unburned forest | Dense dark green canopy |
| Burned area | Large dark/black charred region (center-left) |
| Active fire front | Glowing red/orange perimeter at edge of burned area |
| Smoke | Greyish-white haze above forest canopy (top right) |

### Annotations (labels in image)

| Label | Points to |
|---|---|
| Smoke | Greyish-white hazy areas above canopy |
| Active fire (infrared heat signature) | Glowing red/orange fire perimeter |
| Burned area | Dark charred forest section |

### Map elements

- Scale bar: **200 m**
- North arrow: **N**

---

## Extraction Metadata

| Field | Value |
|---|---|
| **Source PDF** | `C2_Stakeholder_Presentation/Group25_C1_Proposal.pdf` |
| **Output file** | `C2_Stakeholder_Presentation/Group25_C1_Proposal_llm_optimised.md` |
| **Pages processed** | 22 / 22 |
| **PDF type** | Digitally generated text PDF (Google Docs Renderer); not scanned |
| **OCR required** | No for body text; visual semantic extraction used for figures/diagrams |
| **Tables extracted** | 14 meaningful tables (21 pdfplumber table regions; some are partial/continuation rows) |
| **Figures / visuals extracted** | 5 (cover image, system diagram, Gantt chart, 2 progress screenshots) |
| **Equations identified** | 0 |
| **Requirements catalogued** | 10 (R1–R10) |
| **Technical risks catalogued** | 6 |
| **Extraction methods** | PyMuPDF (text, structure, page render, image extract), pdfplumber (tables), visual page inspection |
| **Known extraction uncertainties** | 3 (see below) |
| **Intermediate assets** | `/tmp/pdf_extract_group25_c1/` (page renders, tables JSON, structured blocks) |

## Extraction Uncertainties

1. **PDF p. 2 (TOC) vs PDF p. 8 (body)** — Section 2.6 listed as "Key Challenges" in TOC but document body contains "Scope Boundaries and Assumptions"; no "Key Challenges" section body text found.
2. **PDF p. 18** — Team decision log entry for 18/08/2026 has discussion items but **no recorded outcome** in source.
3. **PDF p. 11 (system diagram)** — Some grey connector labels on diagram may have additional text not fully legible at extraction resolution; core block names and major flows confirmed via visual inspection and narrative text.

---

*End of extracted document. Original PDF unchanged.*
