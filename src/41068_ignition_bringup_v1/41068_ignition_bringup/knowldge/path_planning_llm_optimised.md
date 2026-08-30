# Path Planning

## Document Overview

- **Document type:** PowerPoint seminar slides (PDF export)
- **Author:** Graeme Best
- **Organisation:** University of Technology Sydney — Faculty of Engineering and Information Technology
- **Course:** 41068 Robotics Studio 1
- **Creation date:** 2026-08-27
- **Pages / slides:** 69
- **Primary subject:** Robot path planning — potential fields, graph search, sampling-based planning, Nav2
- **Source file:** `path_planning.pdf`

## Contents

- [Slide 3 — 𝑎𝑐𝑡𝑖𝑜𝑛=  𝑓(𝑠𝑡𝑎𝑡𝑒)](#slide-3)
- [Slide 4 — Example Decision Making Hierarchy](#slide-4)
- [Slide 5 — Example Decision Making Hierarchy](#slide-5)
- [Slide 6 — Example Decision Making Hierarchy](#slide-6)
- [Slide 7 — Definition: Path Planning](#slide-7)
- [Slide 8 — What is Path Planning?](#slide-8)
- [Slide 9 — Why is Path Planning Challenging?](#slide-9)
- [Slide 10 — Questions for your 41068 Project](#slide-10)
- [Slide 11 — Outline](#slide-11)
- [Slide 12 — Quick Definitions](#slide-12)
- [Slide 13 — Configuration Space](#slide-13)
- [Slide 14 — Example: Robot Arms](#slide-14)
- [Slide 15 — Example: 2D World, 2 Degrees of Freedom, Circle](#slide-15)
- [Slide 16 — Motion Models](#slide-16)
- [Slide 17 — Method Variations](#slide-17)
- [Slide 18 — Artificial Potential Fields](#slide-18)
- [Slide 19 — Artificial Potential Fields](#slide-19)
- [Slide 20 — Example](#slide-20)
- [Slide 21 — Many Obstacles: Move Downhill](#slide-21)
- [Slide 22 — When does it go wrong?](#slide-22)
- [Slide 23 — Graph Search](#slide-23)
- [Slide 24 — Graph Search Method](#slide-24)
- [Slide 25 — Graph Data Structure](#slide-25)
- [Slide 26 — Graph Data Structure](#slide-26)
- [Slide 27 — Graph Data Structure](#slide-27)
- [Slide 28 — Graph Data Structure](#slide-28)
- [Slide 29 — C-free Space Discretisation: Cell Decomposition](#slide-29)
- [Slide 30 — Wavefront Algorithm](#slide-30)
- [Slide 31 — Wavefront Algorithm](#slide-31)
- [Slide 32 — Wavefront Algorithm](#slide-32)
- [Slide 33 — Wavefront Algorithm](#slide-33)
- [Slide 34 — Wavefront Algorithm](#slide-34)
- [Slide 35 — A General Shortest Path Problem](#slide-35)
- [Slide 36 — Dijkstra’s Algorithm](#slide-36)
- [Slide 37 — Dijkstra’s Algorithm](#slide-37)
- [Slide 38 — A* (“A star”) Algorithm](#slide-38)
- [Slide 39 — A* cost heuristic](#slide-39)
- [Slide 40 — Dijkstra’s vs A*](#slide-40)
- [Slide 41 — Sampling-Based Motion Planning](#slide-41)
- [Slide 42 — Randomised Algorithms (“Monte Carlo Algorithms”)](#slide-42)
- [Slide 43 — Probabilistic Roadmaps (PRM)](#slide-43)
- [Slide 44 — PRM Example](#slide-44)
- [Slide 45 — PRM Example](#slide-45)
- [Slide 46 — PRM](#slide-46)
- [Slide 47 — RRT (Rapidly-Exploring Random Trees)](#slide-47)
- [Slide 48 — RRT Algorithm](#slide-48)
- [Slide 49 — RRT Algorithm](#slide-49)
- [Slide 50 — RRT Algorithm](#slide-50)
- [Slide 51 — RRT Algorithm](#slide-51)
- [Slide 52 — RRT Algorithm](#slide-52)
- [Slide 53 — RRT Algorithm](#slide-53)
- [Slide 54 — RRT Algorithm](#slide-54)
- [Slide 55 — Examples](#slide-55)
- [Slide 56 — RRT vs RRT* (adds a “rewiring” step)](#slide-56)
- [Slide 57 — RRT vs PRM](#slide-57)
- [Slide 58 — X](#slide-58)
- [Slide 59 — Nav2 Navigation Stack](#slide-59)
- [Slide 60 — Nav2](#slide-60)
- [Slide 61 — Example: 41068 package](#slide-61)
- [Slide 62 — Example: Turtlebot](#slide-62)
- [Slide 63 — Nav2 Concept](#slide-63)
- [Slide 64 — Documentation](#slide-64)
- [Slide 65 — Overview](#slide-65)
- [Slide 66 — Commander API](#slide-66)
- [Slide 67 — Reflection](#slide-67)
- [Slide 68 — Conclusions](#slide-68)
- [Slide 69 — Questions for your 41068 Project](#slide-69)

## Main Content

### Slide 1 — Path Planning

> **Source:** PDF p. 1

- Graeme Best
- Faculty of Engineering and Information Technology
- University of Technology Sydney

---

### Slide 2 — Slide 2

> **Source:** PDF p. 2

#### Visual Content

**Type:** Image-heavy slide (13 embedded images)

**OCR-extracted labels/text:**

```
wei ae NE 7 Ee tom Jv Fg ae ee PS ae
=e = o. oe a En ae a ae
eh “= A : iy ‘| 5 Bo ACES ce ee A
hel ae M4 : = “d p=! , ? ee Se
—— Via —_ | e ee ; te oe ea ee N
E X CR ey Rte Sat
h \ ep ee TM
i i.’ YS es
ieee — Lee ceases Meee
ees ed Sa lL i as cae
pea Pee einee e ak _ aos N cel = . aor
ees Ce Oe sig cs — - a ~—a sn ve 6 a4 > ¥
a. ete Son : ’
e ie 4 See " , ia
, So ; : a al)
pow as 9 Set
im. s : a KAS
ee Og ws
Pa a a ‘ : Ba
hee TG) pee er SN am + AR
mis ~*~ hs rea ss Y
a LL Sky 5 tle EB et ’
2
```

---

### Slide 3 — 𝑎𝑐𝑡𝑖𝑜𝑛=  𝑓(𝑠𝑡𝑎𝑡𝑒)

> **Source:** PDF p. 3

- Robot Decision Making
- e.g. “move over there”
- e.g. location, map
- “planner”

#### Visual Content

**Type:** Image-heavy slide (2 embedded images)

**OCR-extracted labels/text:**

```
Robot Decision Making x
action = f (state)
e.g. “move over there” | e.g. location, map
“planner”
```

---

### Slide 4 — Example Decision Making Hierarchy

> **Source:** PDF p. 4

#### Visual Content

**Type:** Image-heavy slide (3 embedded images)

**OCR-extracted labels/text:**

```
Example Decision Making Hierarchy
SLAM |. Map Processor Behavior Executive
ontput 4 2 y stuck ie
| ae ee Quan
other |S: e ay Ral / | \ =,
robots’ | § . Fei.. a 6-@= wets Fite
maps a
maps i conditions
| ———_—_
| I i
communicate Global Planner Local Planner Controller
to other ae aloe
robots o \ trajectory commands
Le>™ Zr pat oy . ;
OARS 7
ea
4
```

---

### Slide 5 — Example Decision Making Hierarchy

> **Source:** PDF p. 5

- Previous
- Where do I
- seminar
- go next?

#### Visual Content

**Type:** Image-heavy slide (3 embedded images)

**OCR-extracted labels/text:**

```
Example Decision Making Hierarchy
SLAM Map Processor Behavior Executive Previous Where do |
output | BP dad seminar go next?
3 g _ i
ir Pasa ee i =
other |S: ce i ox T\ ——
robots’ S 3 be a é-@= fae w= Pome]
maps =
maps | | conditions
| ———_—_
= i
- |
communicate Global Planner Local Planner Controller
to other ¥ motor
robots SS ' Z trajectory commands
Py Mn $
BN SP Tran 103 oe
PONS * 44 3
hey A Se
5
```

---

### Slide 6 — Example Decision Making Hierarchy

> **Source:** PDF p. 6

- Previous
- Where do I
- seminar
- go next?
- How do I
- Current
- get there?
- seminar

#### Visual Content

**Type:** Image-heavy slide (3 embedded images)

**OCR-extracted labels/text:**

```
Example Decision Making Hierarchy
sLam |_ Map Processor Behavior Executive Previous Where do |
output |e a seminar go next?
3 ~ od - 4 — Prk _
a (ieee ae ee
other |S: a i ox A ——
robots’ Lie é-@= eee
maps
= conditions How dol
maps | Current get there?
[| -——"— seminar
: a =
commintiieate Global Planner Local Planner Controller
to other : maine
at Ah Hi g 8 hjectory commands
PM nee \
er if He ath Wao Io
Pr |e" | :
Me x &
6
```

---

### Slide 7 — Definition: Path Planning

> **Source:** PDF p. 7

**Path Planning:**
- Given the map, the current robot
- location, and the goal
- Find a trajectory that will cause the
- robot to reach the goal location when
- executed

---

### Slide 8 — What is Path Planning?

> **Source:** PDF p. 8

- Robot path planning
- is the process of figuring out
- how the robot moves
- from its current state to a target
- given its map, constraints, uncertainties.
- It includes computing feasible, safe paths/trajectories; handling obstacles
- and dynamic objects; respecting kinematic/dynamic limits and non-
- holonomic constraints; and replanning as the world changes
- Good path planning enables smooth, efficient, and safe motion, making real-
- time progress toward goals while balancing time, energy, and risk.

---

### Slide 9 — Why is Path Planning Challenging?

> **Source:** PDF p. 9

- Defining the right objective: time, speed, safety, comfort
- Incomplete and uncertain information – usually from noisy sensors
- Interact with high-level decision making
- Interact with lower-level controllers
- Computationally difficult – many paths to pick from
- Choosing the right algorithm for the task
- Tuning algorithm parameters
- Multi-robot collision avoidance

---

### Slide 10 — Questions for your 41068 Project

> **Source:** PDF p. 10

- Introduce standard path planning techniques
- Your project must have some decision making / path planning
- How will your robot navigate around the environment?
- You are probably using the Nav2 stack
- But how does this work?
- How can you modify it?
- Or implement it yourself?

---

### Slide 11 — Outline

> **Source:** PDF p. 11

- Quick definitions
- Artificial potential fields
- Graph search methods
- Sampling-based motion planning
- Nav2 navigation stack

---

### Slide 12 — Quick Definitions

> **Source:** PDF p. 12

- Quick Definitions

#### Visual Content

**Type:** Image-heavy slide (2 embedded images)

**OCR-extracted labels/text:**

```
S . ° ° oO e e O O ° e cS . : 5 OC e@ e oO
OO 0 e e e ° : . ° : e © e e e e 2 :
e O OD e @ O . ° . O : e OO) C e @ O . :
e 0 O e e@ O ° ° ° O° ° e O 0 e e@ O o °
Quick Definitions
```

---

### Slide 13 — Configuration Space

> **Source:** PDF p. 13

- Although the motion planning problem is defined in the regular world (the “workspace”),
- it lives in another space:
- the configuration space
- A robot configuration
- 𝑞
- is a specification of the position of all robot points relative to a
- fixed coordinate system
- Usually a configuration is expressed as a
- vector of positions and orientations
- The configuration space (“C-space”) is the
- space of all possible configurations
- The topology of C-space is usually
- not the Cartesian space

---

### Slide 14 — Example: Robot Arms

> **Source:** PDF p. 14

- 3 different configurations
- Configuration space

#### Visual Content

**Type:** Image-heavy slide (4 embedded images)

**OCR-extracted labels/text:**

```
Example: Robot Arms
3 different configurations Configuration space
i nh
14
```

---

### Slide 15 — Example: 2D World, 2 Degrees of Freedom, Circle

> **Source:** PDF p. 15

- Workspace
- Configuration Space

#### Visual Content

**Type:** Image-heavy slide (3 embedded images)

**OCR-extracted labels/text:**

```
Example: 2D World, 2 Degrees of Freedom, Circle x
Where can | move Where can | move
this robot in the this point in the
vicinity of this vicinity of this
obstacle? __is —
equivalent oe
S to... S
: i | Li
Workspace Configuration Space
```

---

### Slide 16 — Motion Models

> **Source:** PDF p. 16

- Industrial robot arm
- Omnidirectional
- Dubins car
- Unicycle

#### Visual Content

**Type:** Image-heavy slide (8 embedded images)

**OCR-extracted labels/text:**

```
Motion Models x
&
Cee
Omnidirectional Dubins car Industrial robot arm
QO ooo oN. vo, 5
Y.. ‘+, Po Og z
3 "SX \ : \ | Ke °
. C . y. , Z \ oY JOINT 3.9... (a &
%, Whiz ~ ee S Fo = sf SS)
>, 3 ’ \ in \ Ss 5, “A JOINT 6
| ‘. , } , \ . ; Jo; /) uy
Unicycle Ne Se ONG cape .
Yo Lp 7 a ei
Vb.2 ' . ' 1 ° ‘
fr 0 \ vo} '
Woz SU 2
```

---

### Slide 17 — Method Variations

> **Source:** PDF p. 17

#### Visual Content

**Type:** Method comparison diagram

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
- Tricky problems need path finding: global search for valid paths

---

### Slide 18 — Artificial Potential Fields

> **Source:** PDF p. 18

- Artificial Potential Fields

#### Visual Content

**Type:** Image-heavy slide (2 embedded images)

**OCR-extracted labels/text:**

```
S . ° ° Oo e@ e O O e e cS . : 9 OC e@ e oO
OO © O e e e 0 : . . : OO e e e e ° :
e O O e @ Oo : ° . 0 7 e O} CU e e O : 0
e Oo OQ e e O ° ° . ° ° e O O e e © ° °
Artificial Potential Fields
```

---

### Slide 19 — Artificial Potential Fields

> **Source:** PDF p. 19

- Attempts to guide the robot from the initial configuration to the goal configuration while
- staying in C-free
- Analogous to an electric potential field
- Electric potential field
- Potential field composed of:
- Attraction force at goal
- Repulsion force at obstacles
- The robot follows a virtual force defined by
- the gradient of the potential field

---

### Slide 20 — Example

> **Source:** PDF p. 20

#### Visual Content

**Type:** Potential field visualisation

**Three panels:**
1. **Attractive Potential** for goals — bowl-shaped field centred on goal
2. **Repulsive Potential** for obstacles — peaks at obstacle locations
3. **Combined Potential Field** — sum of attractive and repulsive: $U(q) = U_{att}(q) + U_{rep}(q)$

**Robot follows gradient downhill** toward goal while avoiding obstacles.

---

### Slide 21 — Many Obstacles: Move Downhill

> **Source:** PDF p. 21

#### Visual Content

**Type:** Local minima demonstration

**Shows:** Robot trapped in local minimum of combined potential field — surrounded by obstacles with no downhill path to goal.

**Label:** "Move Downhill" strategy fails when stuck between obstacle peaks.

---

### Slide 22 — When does it go wrong?

> **Source:** PDF p. 22

- Stuck in local minima
- How to fix this?
- Random walk when stuck
- Define potential fields without local minima (?)
- Combine with global path planning

---

### Slide 23 — Graph Search

> **Source:** PDF p. 23

- Graph Search

#### Visual Content

**Type:** Image-heavy slide (2 embedded images)

**OCR-extracted labels/text:**

```
S . ° ° oO e e O O e e cS . : 9 OC e@ e oO
OO 0 e e e ° : . ° [> © © e e e e 2 :
e O OD e @ O . 0 . O o @ OO) C @ @ O : :
e 0 O e e@ O ° ° ° O° ° e O 0 e e@ O o °
Graph Search
```

---

### Slide 24 — Graph Search Method

> **Source:** PDF p. 24

- Grid cells
- Visibility graph
- Probabilistic roadmap

- 1. Discretise C-Space into a graph
- 2. Perform a graph search
- 3. Extract the shortest path

---

### Slide 25 — Graph Data Structure

> **Source:** PDF p. 25

#### Visual Content

**Type:** Graph theory definitions

**Graph:** $G = (V, E)$ where $V$ = vertices, $E$ = edges; edge $e = (v, w)$

**Example:** $V = \{0,1,2,3,4\}$, $E = \{(0,1),(0,2),(0,4),(2,3),(3,4),(4,0)\}$

**Edge types:** Directed or undirected; may have weights

**Diagrams:** Directed unweighted vs directed weighted graph examples

---

### Slide 26 — Graph Data Structure

> **Source:** PDF p. 26

#### Visual Content

**Type:** Image-heavy slide (3 embedded images)

**OCR-extracted labels/text:**

```
+
Graph Data Structure +
&%
Ce
¢ A path is a sequence of vertices connected by edges, and represented as a sequence in 2 ways:
© (VQ, 1, Vj. >, Vos--sVp_]> Eps Vy) -- alternating vertices and edges
© (Vos V1+ V2ssVp-1+ Vy) -- vertices only
¢ A graph is connected if, for any vertices v and w, there is a path from v to w.
An unconnected graph
```

---

### Slide 27 — Graph Data Structure

> **Source:** PDF p. 27

#### Visual Content

**Type:** Image-heavy slide (3 embedded images)

**OCR-extracted labels/text:**

```
Graph Data Structure x
'°
Cee
2. Graph Representations ~
¢ Adjacency matrix
© n by n matrix, where n is number of vertices
© A(i,j) = 1 iff (i,j) is an edge, or 0 otherwise
© For weighted graph: A(i,j) = w (weight of edge), or positive infinity otherwise
(0) (1) o1odo0o1
ooiii
(2) A=l/o0000
ooi0o0
4)—-+>@ oo0010
2
(0) (1) 5 o2oo
oo 516
1 1 (2) A= : oO Oo © ]
on fF wm wo
3 ooo 1]
Ve 3)
```

---

### Slide 28 — Graph Data Structure

> **Source:** PDF p. 28

#### Visual Content

**Type:** Image-heavy slide (4 embedded images)

**OCR-extracted labels/text:**

```
+
Graph Data Structure +
e Adjacency list
© Each vertex has a linked list of edges
© Edge stores destination and label
© Better when adjacency matrix is sparse
pL Tet
WY | bah
2 = aT
Y-@ 67
OD. = oe
bea hora
‘| A | @ |
(4) GY 3 | L237 |
' [am
```

---

### Slide 29 — C-free Space Discretisation: Cell Decomposition

> **Source:** PDF p. 29

- 1. Create a grid
- 2. Avoid any cells that intersect with an obstacle
- 3. Connect a graph over the cells
- 4. Plan shortest path
- 5. If no plan exists, double the resolution and try again!

---

### Slide 30 — Wavefront Algorithm

> **Source:** PDF p. 30

- Motivated by water waves

- 1. Create a discretised map using cell decomposition
- 2. Add in start and goal locations
- 3. Fill in the “wavefront table”
- 4. Get the obstacle-free path from the wavefront table (Many ways to implement this)

---

### Slide 31 — Wavefront Algorithm

> **Source:** PDF p. 31

#### Visual Content

**Type:** Wavefront algorithm — step 1

**Grid** with obstacles (shaded), **S** = start, **G** = goal

**Legend:** S = start, G = goal

---

### Slide 32 — Wavefront Algorithm

> **Source:** PDF p. 32

#### Visual Content

**Type:** Wavefront algorithm — step 2

**Action:** Build the wave starting from goal

**Wavefront values** propagate outward from G; each cell gets distance-to-goal value incrementing by 1 per step (8-connected or 4-connected grid).

---

### Slide 33 — Wavefront Algorithm

> **Source:** PDF p. 33

#### Visual Content

**Type:** Wavefront algorithm — step 3

**Action:** Get the path in reverse order (follow steepest descent from S toward decreasing wave values)

**Result:** Optimal grid path from S to G avoiding obstacles.

---

### Slide 34 — Wavefront Algorithm

> **Source:** PDF p. 34

- What assumption is being made
- about the graph edges here?

#### Visual Content

**Type:** Image-heavy slide (3 embedded images)

**OCR-extracted labels/text:**

```
Wavefront Algorithm What assumption is being made
about the graph edges here?
S— start, G-- goal Get the path (reverse order)
8 7 65. 5
—
33)
6 6
```

---

### Slide 35 — A General Shortest Path Problem

> **Source:** PDF p. 35

- Given a weighted graph and two vertices s and g
- Find a shortest path from s to g (i.e. path with least sum of edge costs)
- s
- g

---

### Slide 36 — Dijkstra’s Algorithm

> **Source:** PDF p. 36

- An algorithm that solves the shortest path problem
- Traverse the graph by visiting the next vertex that
- has the lowest path cost
- Maintain two sets:
- Visited set (red)
- Unvisited set (blue)

---

### Slide 37 — Dijkstra’s Algorithm

> **Source:** PDF p. 37

- Each node has a “path cost” and a “parent pointer”
- If in
- visited set
- : do nothing
- If in
- unvisited set
- : update the cost and parent if this path has lower cost
- Otherwise: add it to the
- unvisited set
- with appropriate cost and parent

- 1. Empty all parent pointers and path costs
- 2. Create empty unvisited and visited sets
- 3. Add start node to unvisited set with cost 0
- 4. Select the node in the unvisited set with minimum path cost
- 5. Move this selected node to the visited set
- 6. For each neighbour of the selected node:
- 7. Repeat from step 4 until the unvisited set is empty or the goal is found

---

### Slide 38 — A* (“A star”) Algorithm

> **Source:** PDF p. 38

- An algorithm for the general shortest path problem that is faster than Dijkstra’s
- algorithm in typical scenarios
- Relies on a user-defined heuristic estimate
- One change to the algorithm: Define the node costs
- f
- ,
- g
- , and
- h
- as:
- 𝑓𝑥= 𝑔𝑥+ ℎ(𝑥)
- Estimated
- path cost from
- start node to
- to goal
- 𝑥
- Path-cost: cost of the shortest
- Heuristic: Estimate of the
- path from start node to
- distance to the goal
- 𝑥
- Heuristic must be admissible (not overestimate) and consistent (monotone increasing)
- Equivalent to Dijkstra’s algorithm when
- ℎ𝑥= 0

---

### Slide 39 — A* cost heuristic

> **Source:** PDF p. 39

- 𝑓𝑥= 𝑔𝑥+ ℎ(𝑥)
- Estimated
- path cost from
- start node to
- to goal
- 𝑥
- Path-cost: cost of the shortest
- Heuristic: Estimate of the
- path from start node to
- distance to the goal
- 𝑥
- s
- x
- ℎ𝑥
- g
- 𝑔𝑥

---

### Slide 40 — Dijkstra’s vs A*

> **Source:** PDF p. 40

#### Visual Content

**Type:** Image-heavy slide (4 embedded images)

**OCR-extracted labels/text:**

```
Dijkst A*
we
Be ee ee ee
eee ae see ee se tee se ee eee
as te ee ee se tee se ee eee
fee ee ee es Pe a a bee ee ee
fee ee ee es Po a a tee ee ee
Pe a bee ee ee fet ee ee he se ee eee
He see ee fet ee ee he se ee eee
Pa Pa
```

---

### Slide 41 — Sampling-Based Motion Planning

> **Source:** PDF p. 41

- Sampling-Based Motion Planning

#### Visual Content

**Type:** Image-heavy slide (2 embedded images)

**OCR-extracted labels/text:**

```
S . ° ° Oo e@ e O O e e cS . : 9 OC e@ e oO
O © oO e e e ° . . ° — OO C e e e ° :
e O OD e @ O : ° . O : e OO) C e @ O . :
e Oo OQ e e O ° ° . ° ° e O O e e © ° °
Sampling-Based Motion Planning
```

---

### Slide 42 — Randomised Algorithms (“Monte Carlo Algorithms”)

> **Source:** PDF p. 42

- General method for approaching computationally-
- difficult problems
- Replace parts of your algorithm that do an exhaustive
- search with random sampling
- Common approach in robotics:
- Particle filter localisation
- Probabilistic roadmaps (PRM)
- (today)
- Randomly-exploring random trees (RRT)
- (today)
- Monte Carlo tree search (MCTS)
- (previous seminar)

---

### Slide 43 — Probabilistic Roadmaps (PRM)

> **Source:** PDF p. 43

- Generate random graphs using random sampling

- 1. Sample the C-space by generating a large number of samples
- 2. For every sample that is collision free, create a graph node at that location
- 3. Connect pairs of nodes that have a short, local path
- 4. After generating the graph, it can be searched with Dijkstra’s or A* From 49274 Space Robotics

---

### Slide 44 — PRM Example

> **Source:** PDF p. 44

#### Visual Content

**Type:** Image-heavy slide (3 embedded images)

**OCR-extracted labels/text:**

```
PRM Example
1 wy °e e > °
- ai
fo} . —_ : o . :
B e : > e ml e@ 7 > e
. => ° wy °
° ° 4 ° ) ° ° 4 ° e
```

---

### Slide 45 — PRM Example

> **Source:** PDF p. 45

#### Visual Content

**Type:** Image-heavy slide (3 embedded images)

[Visual information primarily conveyed through images; see source PDF p. 45]

---

### Slide 46 — PRM

> **Source:** PDF p. 46

**Advantages:**
- Probabilistically complete
- Do not construct C-space
- Apply easily to high dimensional spaces
- Solved previously unsolved problems
**Disadvantages:**
- Does not work well at all problems
- Not optimal, not complete

---

### Slide 47 — RRT (Rapidly-Exploring Random Trees)

> **Source:** PDF p. 47

- A different type of randomised algorithm for path planning
- Conceptually simple: Incrementally grow a tree that explores the space
- Works well in high-dimensional state space
- Easily incorporate differential constraints
- Applicable to a broad class of problems
- Lots of variants, e.g. RRT* for asymptotic optimality

---

### Slide 48 — RRT Algorithm

> **Source:** PDF p. 48

- tree after a few iterations
- start

#### Visual Content

**Type:** RRT algorithm — initial tree

**Shows:** Start node with empty/small tree after a few iterations in configuration space with obstacles.

---

### Slide 49 — RRT Algorithm

> **Source:** PDF p. 49

- draw a random sample
- random
- point
- start

#### Visual Content

**Type:** RRT — random sample step

**Steps shown:**
1. Draw a random sample (random point in C-space)
2. Start node marked

---

### Slide 50 — RRT Algorithm

> **Source:** PDF p. 50

- find the nearest node
- nearest
- point
- random
- point
- start

#### Visual Content

**Type:** RRT — nearest node step

**Steps:**
1. Find nearest node in tree to random point
2. nearest point and random point labelled

---

### Slide 51 — RRT Algorithm

> **Source:** PDF p. 51

- “steer” towards the random point
- nearest
- point
- random
- point
- start

#### Visual Content

**Type:** RRT — steer step

**Action:** "Steer" from nearest node toward random point (by step size $\Delta q$)

---

### Slide 52 — RRT Algorithm

> **Source:** PDF p. 52

- insert new node into tree
- new node
- start

#### Visual Content

**Type:** RRT — insert node

**Action:** Insert new node into tree (if collision-free)

---

### Slide 53 — RRT Algorithm

> **Source:** PDF p. 53

- start
- nearest
- point
- new node
- repeat!
- random
- point

#### Visual Content

**Type:** RRT — repeat

**Shows:** Full iteration cycle; label "repeat!" — algorithm loops until goal reached or timeout.

---

### Slide 54 — RRT Algorithm

> **Source:** PDF p. 54

- repeat!
- repeat!
- start
- repeat!
- repeat!
- repeat!

#### Visual Content

**Type:** RRT — converged tree

**Shows:** Dense tree after many "repeat!" iterations exploring configuration space.

---

### Slide 55 — Examples

> **Source:** PDF p. 55

#### Visual Content

**Type:** Image-heavy slide (5 embedded images)

**OCR-extracted labels/text:**

```
Examples ~
: wy
_
— , Lom
a 31S e
ye NAD _.
— KO
```

---

### Slide 56 — RRT vs RRT* (adds a “rewiring” step)

> **Source:** PDF p. 56

#### Visual Content

**Type:** RRT vs RRT* comparison plots

**Two side-by-side path plots** in 2D configuration space (-10 to 10 on both axes)

**RRT:** Jagged, suboptimal path
**RRT*:** Smoother, shorter path after rewiring step

**Key difference:** RRT* adds a "rewiring" step to asymptotically converge to optimal paths.

---

### Slide 57 — RRT vs PRM

> **Source:** PDF p. 57

#### Visual Content

**Type:** Image-heavy slide (4 embedded images)

[Visual information primarily conveyed through images; see source PDF p. 57]

---

### Slide 58 — X

> **Source:** PDF p. 58

- Path Smoothing
- Randomised path planners tend to find paths that are not so great for execution:
- Very jagged
- Often longer than necessary
- In practice, smooth the path before using it
- Short-cutting:
- Attempt to connect pairs of nodes along the path while skipping intermediate nodes
- Non-linear optimisation
- Define a “smoothness” objective and optimise to update the path

---

### Slide 59 — Nav2 Navigation Stack

> **Source:** PDF p. 59

- Nav2 Navigation Stack

#### Visual Content

**Type:** Image-heavy slide (2 embedded images)

**OCR-extracted labels/text:**

```
S . ° ° Oo e@ e O O e e cS . : 9 OC e@ e oO
O O © e e e ° : . e: : OO C e e e ° :
e O O e @ Oo : ° . 0 7 e O} CU e e O : 0
e Oo OQ e e O ° ° . ° ° e O O e e © ° °
Nav2 Navigation Stack
```

---

### Slide 60 — Nav2

> **Source:** PDF p. 60

- Default navigation tool for ROS2 – very extensively used across research and industry

---

### Slide 61 — Example: 41068 package

> **Source:** PDF p. 61

#### Visual Content

**Type:** Screenshot — 41068 package with Nav2

**Windows shown:**
- **Gazebo:** Forest environment with robot navigating
- **RViz:** Navigation 2 panel showing Navigation active, Localization inactive, Feedback active; distance remaining ~17.67 m; 2D Nav Goal tool

**Config path visible:** `41068.rviz`

---

### Slide 62 — Example: Turtlebot

> **Source:** PDF p. 62

- https://docs.nav2.org/getting_started/index.html

#### Visual Content

**Type:** Image-heavy slide (3 embedded images)

**OCR-extracted labels/text:**

```
Example: Turtlebot
— MT no a =] Bee Lee
UIs 62
```

---

### Slide 63 — Nav2 Concept

> **Source:** PDF p. 63

- Nav2 enables mobile robots to navigate through complex environments to complete
- user-defined application tasks with nearly any class of robot kinematics
- Move from Point A to Point B, sequences of points, object following, coverage, etc.
- Integrates perception, planning, control, localisation, visualisation
- Compute an environmental model from sensor and semantic data, dynamically path
- plan, compute velocities for motors, avoid obstacles, and structure higher-level robot
- behaviours
- Highly customisable through plugins and parameters
- See config/nav_params.yaml in 41068 package

---

### Slide 64 — Documentation

> **Source:** PDF p. 64

- The documentation is very extensive, go read it!
- https://docs.nav2.org/
- Overview from the developer:
- https://www.youtube.com/watch?v=QB7lOKp3ZDQ
- Configuration guide:
- https://docs.nav2.org/configuration/index.html
- Original academic paper:
- https://arxiv.org/pdf/2003.00368

---

### Slide 65 — Overview

> **Source:** PDF p. 65

- All of the plugins can be
- swapped and customised

#### Visual Content

**Type:** Nav2 architecture diagram

**Components (all plugins swappable/customisable):**
- BT Navigator Server
- Controller Server, Planner Server, Behavior Server, Smoother Server
- Waypoint Follower
- Global Costmap, Local Costmap
- Route Server
- Velocity Smoother, Collision Monitor
- Robot Base
- Sensor data feeds into costmaps

**Key message:** All Nav2 plugins can be swapped and customised.

---

### Slide 66 — Commander API

> **Source:** PDF p. 66

- From your code, you can interact with Nav2 using
- “actions”
- Actions are like topics, but have a request and response, and
- goals and results:
- https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-
- Actions/Understanding-ROS2-Actions.html
- 41068 package provides example code
- The “Commander API” simplifies how to call and track
- the actions
- Overview:
- https://docs.nav2.org/commander_api/index.html
- Code with examples:
- https://github.com/ros-navigation/navigation2/tree/main/nav2_simple_commander

---

### Slide 67 — Reflection

> **Source:** PDF p. 67

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

### Slide 68 — Conclusions

> **Source:** PDF p. 68

- Robot path planning
- is the process of figuring out
- how the robot moves from its current
- state to a target
- given its map, constraints, uncertainties.
- Artificial potential fields
- is a simple path planning approach, but often gets stuck
- Graph search methods
- , including Dijkstra’s and A*, plan a path globally over a graph
- representing the environment
- Probabilistic Roadmap (PRM)
- constructs a graph for A* using random sampling
- Rapidly-Exploring Random Tree (RRT)
- randomly expands a tree over the configuration
- space to find a path
- Nav2
- is the default ROS2 package for navigation

---

### Slide 69 — Questions for your 41068 Project

> **Source:** PDF p. 69

- Introduce standard path planning techniques
- Your project must have some decision making / path planning
- How will your robot navigate around the environment?
- You are probably using the Nav2 stack
- But how does this work?
- How can you modify it?
- Or implement it yourself?

---

## Extraction Metadata

- **Source:** `path_planning.pdf`
- **Pages processed:** 69 / 69
- **Extraction date:** 2026-08-28T15:36:50.550641
- **Slides with curated visual semantics:** 17
- **Slides with OCR/visual fallback:** 41
- **Text-primary slides:** 28
- **OCR engine:** Tesseract 4.1.1
- **Native text extraction:** PyMuPDF (fitz)
- **Document creator:** Microsoft® PowerPoint® for Microsoft 365

## Extraction Uncertainties

1. Some image-heavy slides rely on curated visual descriptions or OCR; fine diagram details may require referring to the source PDF.
2. Mathematical notation in slides uses Unicode italics from PowerPoint; LaTeX equivalents are provided where reconstructed.
3. Photo collage slides (e.g. motivation/examples) contain information not fully transcribed at label level.
4. Animation slides (e.g. frontier exploration) are represented as sequential descriptions, not frame-by-frame data.
