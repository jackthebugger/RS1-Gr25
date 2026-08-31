# Decision Making

## Document Overview

- **Document type:** PowerPoint seminar slides (PDF export)
- **Author:** Graeme Best
- **Organisation:** University of Technology Sydney — Faculty of Engineering and Information Technology
- **Course:** 41068 Robotics Studio 1
- **Creation date:** 2025-08-20
- **Pages / slides:** 57
- **Primary subject:** Robot decision making — TSP, exploration, informative path planning, MCTS, behaviour trees
- **Source file:** `decision_making.pdf`

## Contents

- [Slide 3 — 𝑎𝑐𝑡𝑖𝑜𝑛=  𝑓(𝑠𝑡𝑎𝑡𝑒)](#slide-3)
- [Slide 4 — Coupled Sensing and Planning](#slide-4)
- [Slide 6 — Example Decision Making Hierarchy](#slide-6)
- [Slide 7 — Example Decision Making Hierarchy](#slide-7)
- [Slide 8 — Example Decision Making Hierarchy](#slide-8)
- [Slide 9 — Example Decision Making Hierarchy](#slide-9)
- [Slide 10 — What is Decision Making?](#slide-10)
- [Slide 11 — Why is Decision Making Challenging?](#slide-11)
- [Slide 12 — Questions for your 41068 Project](#slide-12)
- [Slide 13 — Outline](#slide-13)
- [Slide 14 — Travelling Salesman Problem](#slide-14)
- [Slide 15 — Vehicle Routing Problems](#slide-15)
- [Slide 16 — Travelling Salesman Problem](#slide-16)
- [Slide 17 — Solution: Brute-Force Search](#slide-17)
- [Slide 18 — Solution: Nearest Neighbour](#slide-18)
- [Slide 19 — Solution: 2-Opt](#slide-19)
- [Slide 20 — Off-the-Shelf Packages](#slide-20)
- [Slide 21 — Orienteering Problem](#slide-21)
- [Slide 22 — Exploration](#slide-22)
- [Slide 23 — Robot Exploration](#slide-23)
- [Slide 24 — Problem Definition](#slide-24)
- [Slide 25 — Frontier-Based Exploration](#slide-25)
- [Slide 26 — Frontier Selection Strategies](#slide-26)
- [Slide 27 — Animation: Frontier-Based Exploration](#slide-27)
- [Slide 28 — Advanced Exploration](#slide-28)
- [Slide 29 — Off-the-Shelf Packages](#slide-29)
- [Slide 30 — Informative Path Planning](#slide-30)
- [Slide 31 — Informative Path Planning](#slide-31)
- [Slide 32 — Uncertainty: Example](#slide-32)
- [Slide 33 — Uncertainty: Entropy](#slide-33)
- [Slide 34 — Information-Based Exploration](#slide-34)
- [Slide 35 — Estimate a Field](#slide-35)
- [Slide 36 — Gaussian Processes](#slide-36)
- [Slide 37 — Algorithms](#slide-37)
- [Slide 38 — Monte Carlo Tree Search](#slide-38)
- [Slide 39 — MCTS for Robot Information Gathering](#slide-39)
- [Slide 40 — Tree Search Problem](#slide-40)
- [Slide 41 — Monte Carlo Tree Search (MCTS)](#slide-41)
- [Slide 42 — MCTS Behaviour](#slide-42)
- [Slide 43 — MCTS Algorithm](#slide-43)
- [Slide 44 — Upper-Confidence Bound (UCB)](#slide-44)
- [Slide 45 — MCTS in Action](#slide-45)
- [Slide 46 — How To Implement MCTS for My Problem?](#slide-46)
- [Slide 47 — Tuning Parameters](#slide-47)
- [Slide 48 — Debugging Tips](#slide-48)
- [Slide 49 — Dec-MCTS for Multi-Robot Coordination](#slide-49)
- [Slide 50 — Behaviour Trees](#slide-50)
- [Slide 51 — Multi-Task Missions](#slide-51)
- [Slide 52 — Behaviour Tree Definitions](#slide-52)
- [Slide 53 — Example Mission](#slide-53)
- [Slide 54 — Off-the-Shelf Packages](#slide-54)
- [Slide 55 — Reflection](#slide-55)
- [Slide 56 — Conclusions](#slide-56)
- [Slide 57 — Questions for your 41068 Project](#slide-57)

## Main Content

### Slide 1 — Decision Making

> **Source:** PDF p. 1

- Graeme Best
- Faculty of Engineering and Information Technology
- University of Technology Sydney

---

### Slide 2 — Slide 2

> **Source:** PDF p. 2

#### Visual Content

**Type:** Photo collage / motivation slide

**Visible content:**
- RGB-D scene collected using robot
- Multiple robotics application photographs showing field robots, drones, and exploration scenarios
- Visual motivation for decision-making topics in robotics

---

### Slide 3 — 𝑎𝑐𝑡𝑖𝑜𝑛=  𝑓(𝑠𝑡𝑎𝑡𝑒)

> **Source:** PDF p. 3

- Robot Decision Making
- e.g. “move over there”
- e.g. location, map
- “planner”

#### Visual Content

**Type:** Concept diagram

**Elements:**
- Central equation: $\text{action} = f(\text{state})$
- **State** examples: location, map
- **Action** example: "move over there"
- **Planner** block connects state to action

**Relationship:** The planner is a function mapping robot state to the next action.

---

### Slide 4 — Coupled Sensing and Planning

> **Source:** PDF p. 4

- Make an observation at this location
- Sensing
- Planning
- Move to a new location

#### Visual Content

**Type:** Coupled sensing-planning diagram

**Flow:**
1. **Sensing:** Make an observation at this location
2. **Planning:** Move to a new location
3. Cycle repeats — sensing and planning are coupled

**Key concept:** Robot alternates between observing and moving to new locations.

---

### Slide 5 — Slide 5

> **Source:** PDF p. 5

#### Visual Content

**Type:** Section divider

**Label:** Mapping (transition slide between hierarchy overview and definitions)

---

### Slide 6 — Example Decision Making Hierarchy

> **Source:** PDF p. 6

#### Visual Content

**Type:** System architecture block diagram

**Components (left to right, top to bottom):**

| Component | Inputs | Outputs |
|---|---|---|
| **Map Processor** | SLAM output, other robots' maps | maps, stuck signal; communicates to other robots |
| **Behavior Executive** | stuck, conditions from planners | actions to Global Planner and Local Planner |
| **Global Planner** | maps, actions | path; conditions feedback |
| **Local Planner** | maps, actions, path | trajectory; conditions feedback |
| **Controller** | trajectory | motor commands |

**Visual thumbnails:** Each block shows representative imagery (occupancy map, behaviour tree, global path, local trajectory in 3D environment, physical robot).

---

### Slide 7 — Example Decision Making Hierarchy

> **Source:** PDF p. 7

- Previous
- seminar

#### Visual Content

**Type:** Annotated hierarchy diagram (progression slide 1)

**Annotation:** Map Processor highlighted — labelled **"Previous seminar"**

Same architecture as Slide 6; emphasis on Map Processor as content covered in an earlier seminar.

---

### Slide 8 — Example Decision Making Hierarchy

> **Source:** PDF p. 8

- Previous
- seminar
- Today
- Where do I
- go next?
- Today

#### Visual Content

**Type:** Annotated hierarchy diagram (progression slide 2)

**Annotations:**
- Map Processor: **Previous seminar** (blue)
- Behavior Executive: **Today** — "Where do I go next?" (green)
- Global Planner: **Today** (green)

**Focus:** Current seminar covers high-level decision making (behaviour executive) and global planning.

---

### Slide 9 — Example Decision Making Hierarchy

> **Source:** PDF p. 9

- Previous
- seminar
- Today
- Where do I
- go next?
- How do I
- Future
- get there?
- seminar
- Today

#### Visual Content

**Type:** Annotated hierarchy diagram (progression slide 3)

**Annotations:**
- Map Processor: **Previous seminar**
- Behavior Executive: **Today** — "Where do I go next?"
- Global Planner + Local Planner: **Today** — "How do I get there?"
- Controller: **Future seminar**

**Focus:** Path planning (global + local) is the current topic; low-level control deferred.

---

### Slide 10 — What is Decision Making?

> **Source:** PDF p. 10

- Robot decision making
- is the process of choosing
- what the robot should
- do next
- based on its current knowledge, goals, and environment.
- It includes selecting the next
- task, goal, or behaviour
- , rather than just how
- to execute a specific movement.
- Good decision-making helps a robot act
- autonomously
- , adapting to
- changing conditions, and make the best use of its time, sensors, and
- capabilities.

---

### Slide 11 — Why is Decision Making Challenging?

> **Source:** PDF p. 11

- Defining the right objective for the application
- Incomplete and uncertain information – usually from noisy sensors
- Interact with lower-level planners and controllers
- Computationally difficult – many options to pick from
- Choosing the right algorithm for the task
- Tuning algorithm parameters
- Multi-robot coordination

---

### Slide 12 — Questions for your 41068 Project

> **Source:** PDF p. 12

- Ideas for different types of decision making
- Your project must have some decision making
- How will your robot decide where to go next?
- What is the objective?
- What algorithms will solve this problem?
- How will these algorithms connect to the rest of your system?

---

### Slide 13 — Outline

> **Source:** PDF p. 13

- Travelling salesman problem
- Exploration
- Informative path planning
- Monte Carlo tree search
- Behaviour trees

---

### Slide 14 — Travelling Salesman Problem

> **Source:** PDF p. 14

- Travelling Salesman Problem

#### Visual Content

**Type:** Section divider — Travelling Salesman Problem

---

### Slide 15 — Vehicle Routing Problems

> **Source:** PDF p. 15

#### Visual Content

**Type:** Vehicle routing application photographs

**Examples shown:**
- Mars rover mission planning
- Region of Interest / Extended Mission planning
- Offshore oil rig inspection with robot paths
- Letters / delivery routing
- Orange center inspection regions

**Purpose:** Illustrates real-world variants of routing problems beyond classic TSP.

---

### Slide 16 — Travelling Salesman Problem

> **Source:** PDF p. 16

- Widely studied benchmark problem in Computer Science
- Useful approximation to some robotics problems
- Many solutions exist
- Many variants exist
**Problem:**
- Given a list of locations and the distances
- between locations,
- what is the shortest possible route
- that visits each location exactly once and
- returns to the origin?

---

### Slide 17 — Solution: Brute-Force Search

> **Source:** PDF p. 17

- Optimal algorithm, very slow
- 𝑂(𝑛!)
- time
- 1,2,3, … , 𝑛
- 2,1,3, … , 𝑛
- 1,3,2, … , 𝑛
- 2,3,1, … , 𝑛
- …

- 1. Arbitrarily label all of the nodes
- 2. Try all possible permutations
- 3. Pick the permutation with the lowest cost

---

### Slide 18 — Solution: Nearest Neighbour

> **Source:** PDF p. 18

- A “greedy” algorithm
- Making a locally-optimal choice at each stage

- 1. Pick a starting node
- 2. Move to the nearest neighbour that has not yet been visited
- 3. Repeat step 2

---

### Slide 19 — Solution: 2-Opt

> **Source:** PDF p. 19

- 1. Start with a nearest-neighbour solution
- 2. Pick two random edges in the existing solution
- 3. Replace them with a better pair of edges
- 4. Repeat 2 and 3

---

### Slide 20 — Off-the-Shelf Packages

> **Source:** PDF p. 20

- You could implement solutions yourself
- Many off-the-shelf packages exist,
- e.g. python_tsp:
- https://pypi.org/project/python_tsp/
- Define the distance between each
- pair of points.
- Infinity means disconnected.
- Visit the locations in this order.
- Also see coverage planning:
- https://fields2cover.github.io/

---

### Slide 21 — Orienteering Problem

> **Source:** PDF p. 21

**Problem:**
- Given a list of locations, the
- distances between locations,
- and a maximum route length,
- find a route that visits as many
- locations as possible?

---

### Slide 22 — Exploration

> **Source:** PDF p. 22

- Exploration

#### Visual Content

**Type:** Image-heavy slide (2 embedded images)

**OCR-extracted labels/text:**

```
0 . ° ° oO e e O Oo e e 5 ° 5 5 oO e@ e Oo
OO 0 e e e ° : . ° [> © © e e e e 2 :
e O OD e @ O . 0 . O o @ OO) C @ @ O : :
e Oo Oo e e O ° 6 . ° ° e 5 O e e O ° °
Exploration
```

---

### Slide 23 — Robot Exploration

> **Source:** PDF p. 23

- Aim: Construct a
- map
- of the world
- Classic robot planning problem
- Lots of practical uses

---

### Slide 24 — Problem Definition

> **Source:** PDF p. 24

- Unknown
**Problem:**
- Occupied
- Given the current map of the world
- and a sensor model,
- where should a robot move to quickly
- build a full map of the world?
- Free

---

### Slide 25 — Frontier-Based Exploration

> **Source:** PDF p. 25

- Key idea:
- To gain the most new information, move to the boundary between free space
- and unknown space, i.e. “frontiers”
- Current map
- Frontier cells
- Frontier clusters

---

### Slide 26 — Frontier Selection Strategies

> **Source:** PDF p. 26

- Which frontier to go to first?
- Closest first
- Other quantities to consider:
- Path distance
- Safety
- ?
- Frontier size
- Information beyond the frontier
- ?
- ?
- Coordination value
- …
- Combinations of the above
- Then use path planning to selected frontier

---

### Slide 27 — Animation: Frontier-Based Exploration

> **Source:** PDF p. 27

#### Visual Content

**Type:** Animation / simulation sequence

**Content:** Frontier-based exploration animation showing robot progressively mapping unknown environment by visiting frontier cells at the boundary between known free space and unknown space.

**Stages:** Map expands over time as robot visits frontiers.

---

### Slide 28 — Advanced Exploration

> **Source:** PDF p. 28

- 1. Frontier generation
- **a)** Vision (coverage)
- **b)** Range (exploration)
- 2. Viewpoint extraction
- 3. Viewpoint ranking
- **a)** Distance
- **b)** Momentum change
- **c)** Coordination value
- **d)** …
- 4. RRT-Connect planner

---

### Slide 29 — Off-the-Shelf Packages

> **Source:** PDF p. 29

- You could implement solutions yourself
- This would be challenging, but achievable for a 41068 project
- While there are no “default” solutions, there are many experimental packages:
- I recommend trying this one first
- https://github.com/robo-friends/m-explore-ros2
- , which is
- referenced in the following instructions:
- https://husarion.com/tutorials/ros2-tutorials/10-exploration/
- Lots of other variations on github, such as:
- https://github.com/AniArka/Autonomous-Explorer-and-Mapper-ros2-nav2
- https://github.com/SeanReg/nav2_wavefront_frontier_exploration?tab=readme-ov-file
- It might take quite a bit of work to get these packages to do what you want!

---

### Slide 30 — Informative Path Planning

> **Source:** PDF p. 30

- Informative Path Planning

#### Visual Content

**Type:** Image-heavy slide (2 embedded images)

**OCR-extracted labels/text:**

```
S . ° ° Oo e@ e O O e e cS . : 9 OC e@ e oO
e O OD e @ O : : : Oo e OO) C e @ O : .
e Oo OQ e e O ° ° . ° ° e O O e e © ° °
Informative Path Planning
```

---

### Slide 31 — Informative Path Planning

> **Source:** PDF p. 31

- We have a
- quantity of interest
- that we wish to observe/estimate
- A map
- The location of an object
- The type of an object
- A temperature map of the ocean
- Soil pH levels
- Maintain a probabilistic belief of the quantity of interest
**Problem:**
- Given the current belief of a quantity of interest,
- where should a robot move to quickly
- reduce the uncertainty
- of this belief?

---

### Slide 32 — Uncertainty: Example

> **Source:** PDF p. 32

- Define a random variable
- 𝑋
- Let
- 𝑝(𝑥
- )
- be the probability that
- 𝑋= 𝑥
- 𝑖
- 𝑖
- Assume
- 𝑋
- is a Bernoulli variable: i.e. can be either “0” or “1”
- Which has the highest uncertainty?

---

### Slide 33 — Uncertainty: Entropy

> **Source:** PDF p. 33

- For a Bernoulli (0 or 1) distribution
- High
- uncertainty
- Entropy
- We need a method to quantify “uncertainty”
- Entropy defined as:
- Low
- Low
- uncertainty
- uncertainty
- Sum over all possible outcomes for
- 𝑋

---

### Slide 34 — Information-Based Exploration

> **Source:** PDF p. 34

- Alternative to frontier-based exploration
- Mathematically correct
- Much slower to compute
- Choose the next action that reduces “uncertainty”
- Use a probabilistic occupancy grid
- High
- uncertainty
- “Uncertainty” often defined as entropy
- Entropy
- Low
- Low
- uncertainty
- uncertainty
- Probability of being occupied

---

### Slide 35 — Estimate a Field

> **Source:** PDF p. 35

- Estimate of wireless
- signal strength
- Uncertainty of
- estimate
- [Hollinger, 2014]

#### Visual Content

**Type:** Dual heatmap figure [Hollinger, 2014]

**Left panel:** Estimate of wireless signal strength (spatial field)
**Right panel:** Uncertainty of estimate (higher uncertainty in unobserved regions)

**Purpose:** Illustrates informative path planning — robot should visit high-uncertainty regions.

---

### Slide 36 — Gaussian Processes

> **Source:** PDF p. 36

- Gaussian Processes are probability
- distributions over functions
- Infinite dimensional Gaussian
- distribution
- A covariance function defines the
- relationship between points
- Quantity of
- interest
- 3 possible
- functions
- Off-the-shelf package:
- Observations
- https://scikit-
- Uncertainty
- learn.org/stable/modules/gaussian_pr
- ocess.html
- Location

---

### Slide 37 — Algorithms

> **Source:** PDF p. 37

- Greedy:
- Move to the location that has the highest uncertainty
- Branch and bound:
- Tree search that iteratively bounds the solution quality
- Sampling-based methods:
- RRT over the information field
- Monte Carlo tree search:
- Next section!

---

### Slide 38 — Monte Carlo Tree Search

> **Source:** PDF p. 38

- Monte Carlo Tree Search

#### Visual Content

**Type:** Section divider — Monte Carlo Tree Search

---

### Slide 39 — MCTS for Robot Information Gathering

> **Source:** PDF p. 39

#### Visual Content

**Type:** Application photograph

**Content:** MCTS applied to robot information gathering — RGB-D scene collected using robot in structured indoor environment.

---

### Slide 40 — Tree Search Problem

> **Source:** PDF p. 40

- Current state
- Future actions
- Green =
- high reward
- Find the best action sequence

#### Visual Content

**Type:** Tree search diagram

**Elements:**
- **Current state** (root)
- **Future actions** (tree branches)
- **Green nodes** = high reward paths
- **Goal:** Find the best action sequence

---

### Slide 41 — Monte Carlo Tree Search (MCTS)

> **Source:** PDF p. 41

- Biased
- random sampling
- Exploits “smoothness” of search space
- Any-time
- Only requires evaluation of
- full
- paths
- Can incorporate problem-specific heuristics
- Has theoretical convergence guarantees
- Recently become popular in robotics
- Computer Go
- [Silver, 2017]

---

### Slide 42 — MCTS Behaviour

> **Source:** PDF p. 42

- expected reward
- high
- low

#### Visual Content

**Type:** MCTS behaviour plot

**X-axis:** exploration ↔ exploitation balance
**Y-axis:** expected reward (high to low)

**Shows:** MCTS balances exploring new branches vs exploiting known high-reward paths.

---

### Slide 43 — MCTS Algorithm

> **Source:** PDF p. 43

#### Visual Content

**Type:** MCTS algorithm flowchart

**Pipeline:** Selection → Expansion → Simulation → Backpropagation

**Sub-components:**
- **Tree Policy** (selection/expansion)
- **Default Policy** (simulation/rollout)

**Cycle repeats** to grow search tree and improve action estimates.

---

### Slide 44 — Upper-Confidence Bound (UCB)

> **Source:** PDF p. 44

- Average rollout score
- Parent #visits
- Child #visits
- Upper confidence
- bound (UCB1)
- Multi-armed bandit
- Exploitation
- Exploration

---

### Slide 45 — MCTS in Action

> **Source:** PDF p. 45

#### Visual Content

**Type:** MCTS in action — tree growth visualisation

**Shows:** Search tree expanding over a spatial domain with varying node visit counts and reward estimates.

---

### Slide 46 — How To Implement MCTS for My Problem?

> **Source:** PDF p. 46

- What is a
- state
- ?
- For each
- state
- , what
- actions
- can I do?
- For each
- action
- , what is my
- next state
- ?
- For each complete sequence of actions, what is its
- reward
- ?
- Scaled between roughly 0 and 1

---

### Slide 47 — Tuning Parameters

> **Source:** PDF p. 47

- Planning horizon (size of tree)
- Rollout policy
- Random
- Greedy
- Exploration-exploitation parameter
- 2

---

### Slide 48 — Debugging Tips

> **Source:** PDF p. 48

- Observe the solutions
- Plot the tree
- Show empirically that it is converging
- Optimisation: Reward evaluation is the slowest part
- Use a profiler
- Caching where possible

---

### Slide 49 — Dec-MCTS for Multi-Robot Coordination

> **Source:** PDF p. 49

- (a) Grow search tree for
- own actions
- Performed asynchronously
- by each robot
- (c) Communicate distributions
- (b) Decentralised optimisation of
- with other robots
- probability distributions

#### Visual Content

**Type:** Dec-MCTS multi-robot diagram

**(a)** Grow search tree for own actions — performed asynchronously by each robot
**(b)** Decentralised optimisation of probability distributions
**(c)** Communicate distributions with other robots

**Key concept:** Each robot runs MCTS locally and coordinates via communicated probability distributions.

---

### Slide 50 — Behaviour Trees

> **Source:** PDF p. 50

- Behaviour Trees

#### Visual Content

**Type:** Section divider — Behaviour Trees

---

### Slide 51 — Multi-Task Missions

> **Source:** PDF p. 51

- Subterranean Challenge
- Exploring area
- Avoid hazard
- Drilling
- Scan rocks

#### Visual Content

**Type:** Multi-task mission photographs

**Context:** DARPA Subterranean Challenge

**Tasks shown:**
- Exploring area
- Avoid hazard
- Drilling
- Scan rocks

**Purpose:** Real missions require switching between multiple concurrent tasks.

---

### Slide 52 — Behaviour Tree Definitions

> **Source:** PDF p. 52

- Activate recursively,
- Sequence:
- from left to right
- Up to first failure
- Fallback:
- Up to first success
- Conditions:
- Success or failure
- Actions:
- … and other
- Success or failure
- types of nodes

---

### Slide 53 — Example Mission

> **Source:** PDF p. 53

#### Visual Content

**Type:** Example behaviour tree + timeline

**Behaviour tree structure (top):**
- Root: Fallback → Sequence
  - Diagnostics
  - Take off → Exploration → Landing
  - Return / Landing / Shutdown branches
  - Coordinated Explore / Roadmap Explore with conditions (Has Unvisited Frontiers, Stuck, Critical Battery, Unreachable Goal)
  - Emergency Land, Return Home, Rewind actions

**Timeline (bottom):** Mission execution over 0–900 seconds showing state transitions.

---

### Slide 54 — Off-the-Shelf Packages

> **Source:** PDF p. 54

- The Nav2 navigation stack uses a specific Behaviour Tree underneath:
- https://docs.nav2.org/behavior_trees/overview/detailed_behavior_tree_walkthrough.html
- Many implementations exist:
- https://www.behaviortree.dev/docs/ros2_integration/
- https://github.com/BehaviorTree/BehaviorTree.ROS2?tab=readme-ov-file
- It will take quite a bit of work to understand
- Nav2 behaviour tree
- and integrate with these packages
- Also see: Finite State Machines

---

### Slide 55 — Reflection

> **Source:** PDF p. 55

- Reflection

#### Visual Content

**Type:** Image-heavy slide (2 embedded images)

**OCR-extracted labels/text:**

```
0 . ° ° oO e e O Oo e e 5 ° 5 5 oO e@ e Oo
OO 0 e e e ° 0 ° ° 9 OO C e e e ° :
e O OD e @ O . 0 . O o @ OO) C e @ O : :
e 0 O e e@ O ° ° ° O° ° e O 0 e e@ O o °
Reflection
```

---

### Slide 56 — Conclusions

> **Source:** PDF p. 56

- Robot decision making
- is the process of choosing
- what the robot should do next
- based on
- its current knowledge, goals, and environment.
- Travelling salesman problem
- aims to visit a set of locations as quickly as possible
- Exploration
- aims to build a map of an environment by visiting unobserved locations
- Informative path planning
- gathers the most information, as defined by an application
- Monte Carlo tree search
- is an algorithm for decision making
- Behaviour trees
- is a way of encoding high-level task switching

---

### Slide 57 — Questions for your 41068 Project

> **Source:** PDF p. 57

- Ideas for different types of decision making
- Your project must have some decision making
- How will your robot decide where to go next?
- What is the objective?
- What algorithms will solve this problem?
- How will these algorithms connect to the rest of your system?

---

## Extraction Metadata

- **Source:** `decision_making.pdf`
- **Pages processed:** 57 / 57
- **Extraction date:** 2026-08-28T15:36:19.526542
- **Slides with curated visual semantics:** 22
- **Slides with OCR/visual fallback:** 25
- **Text-primary slides:** 32
- **OCR engine:** Tesseract 4.1.1
- **Native text extraction:** PyMuPDF (fitz)
- **Document creator:** Microsoft® PowerPoint® for Microsoft 365

## Extraction Uncertainties

1. Some image-heavy slides rely on curated visual descriptions or OCR; fine diagram details may require referring to the source PDF.
2. Mathematical notation in slides uses Unicode italics from PowerPoint; LaTeX equivalents are provided where reconstructed.
3. Photo collage slides (e.g. motivation/examples) contain information not fully transcribed at label level.
4. Animation slides (e.g. frontier exploration) are represented as sequential descriptions, not frame-by-frame data.
