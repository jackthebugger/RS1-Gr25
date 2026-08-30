# Perception and Mapping

## Document Overview

- **Document type:** PowerPoint seminar slides (PDF export)
- **Author:** Graeme Best
- **Organisation:** University of Technology Sydney — Faculty of Engineering and Information Technology
- **Course:** 41068 Robotics Studio 1
- **Creation date:** 2026-08-13
- **Pages / slides:** 43
- **Primary subject:** Robot perception, sensors, map representations, occupancy grids, SLAM
- **Source file:** `perception_mapping.pdf`

## Contents

- [Slide 2 — Robot Perception Tasks](#slide-2)
- [Slide 3 — Maps for Subterranean Exploration (earlier seminar)](#slide-3)
- [Slide 6 — Questions for your 41068 Project](#slide-6)
- [Slide 7 — Outline](#slide-7)
- [Slide 8 — Sensors for Mapping](#slide-8)
- [Slide 9 — Sensors](#slide-9)
- [Slide 10 — Sensors on Autonomous Cars](#slide-10)
- [Slide 11 — Sensors on Mobile Robots](#slide-11)
- [Slide 12 — Sensors on Mars Rover Perseverance](#slide-12)
- [Slide 13 — LIDAR: LI ght D etection A nd R anging](#slide-13)
- [Slide 14 — Map Representations](#slide-14)
- [Slide 15 — What is Mapping?](#slide-15)
- [Slide 16 — Why is Mapping Challenging?](#slide-16)
- [Slide 17 — Map Categories](#slide-17)
- [Slide 18 — Occupancy Grids](#slide-18)
- [Slide 19 — Terrain Map](#slide-19)
- [Slide 20 — Distance Function Maps](#slide-20)
- [Slide 21 — Feature Maps](#slide-21)
- [Slide 22 — Trajectory Tracking](#slide-22)
- [Slide 23 — Topological Maps](#slide-23)
- [Slide 24 — Semantic Maps](#slide-24)
- [Slide 25 — Environmental Phenomena](#slide-25)
- [Slide 26 — Which are most meaningful for your project?](#slide-26)
- [Slide 27 — Occupancy Grids](#slide-27)
- [Slide 28 — Occupancy Grid](#slide-28)
- [Slide 29 — Occupancy Probability](#slide-29)
- [Slide 30 — Occupancy Grid Map](#slide-30)
- [Slide 31 — Problem: Estimate Map from Sensor Data](#slide-31)
- [Slide 32 — Bayesian Filtering](#slide-32)
- [Slide 33 — Incorporating an Observation](#slide-33)
- [Slide 34 — Noisy Observations](#slide-34)
- [Slide 35 — Applying Bayes’ Theorem](#slide-35)
- [Slide 36 — Recursive Bayes’ Filter](#slide-36)
- [Slide 37 — Recursive Bayes’ Filter](#slide-37)
- [Slide 38 — ROS2 Packages](#slide-38)
- [Slide 39 — Hierarchical Data Structures](#slide-39)
- [Slide 40 — Simultaneous Localisation and Mapping (SLAM)](#slide-40)
- [Slide 41 — Reflection](#slide-41)
- [Slide 42 — Conclusions](#slide-42)
- [Slide 43 — Questions for your 41068 Project](#slide-43)

## Main Content

### Slide 1 — Perception and Mapping

> **Source:** PDF p. 1

- Graeme Best
- Faculty of Engineering and Information Technology
- University of Technology Sydney

---

### Slide 2 — Robot Perception Tasks

> **Source:** PDF p. 2

- Fruit tree modelling
- Ocean monitoring
- Subterranean mapping

#### Visual Content

**Type:** Application photographs

**Three perception tasks:**
1. **Fruit tree modelling** — agricultural robot among trees
2. **Ocean monitoring** — marine/aerial monitoring
3. **Subterranean mapping** — cave/tunnel exploration robot

---

### Slide 3 — Maps for Subterranean Exploration (earlier seminar)

> **Source:** PDF p. 3

- Exploration maps
- Navigation roadmap
- Local planner map
- OpenVDB structure
- Connect visited locations
- Distance transform
- LiDAR- and camera-
- Repairs for changes
- Dust filtering
- observed labels
- Shared across team
- Computed on GPU
- ➢
- Shared across team

---

### Slide 4 — Slide 4

> **Source:** PDF p. 4

#### Visual Content

**Type:** Image-heavy slide (3 embedded images)

**OCR-extracted labels/text:**

```
— Sa
Six. so, ao) 5
. - : & =—>.~
— - — ~~,
om ? Se
° Bs : *s,
| aa .
, a — aa -< m “4 >
my 7 SS.
= | os
bk | _
Da ie »
4
```

---

### Slide 5 — Slide 5

> **Source:** PDF p. 5

#### Visual Content

**Type:** Image-heavy slide (3 embedded images)

**OCR-extracted labels/text:**

```
le aa
Mapping
Pe |
3 4 a a >
ms —— |
: -
```

---

### Slide 6 — Questions for your 41068 Project

> **Source:** PDF p. 6

- Ideas for different types of maps
- Your project must have some perception/mapping
- Your project must have a user interface
- What types of maps will your robot produce?
- For primary task
- For autonomy
- What information will these maps contain?
- How will you show these maps in your User Interface?

---

### Slide 7 — Outline

> **Source:** PDF p. 7

- Sensors for mapping
- Map representations – many examples
- Occupancy grids
- Reflection

---

### Slide 8 — Sensors for Mapping

> **Source:** PDF p. 8

- Sensors for Mapping

#### Visual Content

**Type:** Image-heavy slide (2 embedded images)

**OCR-extracted labels/text:**

```
S . ° ° Oo e@ e O O e e cS . : 9 OC e@ e oO
@ e@ © e e e 0 : . ° : OO C e e e ° :
@ C) O) e@ e@ O . : . O ° e@ @ CO @ @ OC . :
e Oo OQ e e O ° ° . ° ° e O O e e © ° °
Sensors for Mapping
```

---

### Slide 9 — Sensors

> **Source:** PDF p. 9

- A device for measuring some physical quantity
- The sensor usually converts from the measurement space to an electrical
- signal
- Temperature
- Distance
- Force
- SENSOR
- Electrical Signal
- Speed
- Sound
- Light

---

### Slide 10 — Sensors on Autonomous Cars

> **Source:** PDF p. 10

- machinedesign.com

#### Visual Content

**Type:** Autonomous car sensor diagram [machinedesign.com]

**Sensors labelled:**
- Velodyne multi-layer laser scanner
- GPS antenna, INS solution
- Steering actuator, brake actuator, gear shift actuator
- Cameras (multiple)
- Radar
- 2D laser scanner
- DMI (Distance Measurement Instrument)
- APS signal

**Purpose:** Shows complexity of sensor suites on autonomous vehicles.

---

### Slide 11 — Sensors on Mobile Robots

> **Source:** PDF p. 11

- 100m

#### Visual Content

**Type:** Mobile robot sensor layout diagram

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

**Purpose:** Typical indoor mobile robot sensor configuration.

---

### Slide 12 — Sensors on Mars Rover Perseverance

> **Source:** PDF p. 12

- See
- https://science.nasa.gov/mission/mars-2020-perseverance/science-instruments/
- https://science.nasa.gov/mission/mars-2020-perseverance/rover-components/

---

### Slide 13 — LIDAR: LI ght D etection A nd R anging

> **Source:** PDF p. 13

- Measure the time of flight of laser light pulses
- The pulsed laser beam is deflected by an internal
- rotating mirror
- The measurement data is available in real time via an
- interface

---

### Slide 14 — Map Representations

> **Source:** PDF p. 14

- Map Representations

#### Visual Content

**Type:** Image-heavy slide (2 embedded images)

**OCR-extracted labels/text:**

```
o ° ° ° Oo e e Oo © e e Co ° 5 o Oo e e Oo
@ e@ © e e e 0 : . ° : OO C e e e ° :
@ C) O e@ e O . : . O ° e@ CO) CO @ @ OC . :
e Oo OQ e e O ° ° . ° ° e O O e e © ° °
Map Representations
```

---

### Slide 15 — What is Mapping?

> **Source:** PDF p. 15

- Robot mapping
- is the process by which a robot builds a representation of
- its environment using data from its sensors (and prior information).
- This representation – often called a
- map
- – captures spatial information
- about the surroundings.
- Useful for:
- Enabling autonomy
- : Understand location of obstacles, landmarks, traversability,
- navigable areas.
- Primary tasks
- : Provide information to humans and other robots that is specific to
- the task. May be displayed in a
- user interface
- .

---

### Slide 16 — Why is Mapping Challenging?

> **Source:** PDF p. 16

- Perception challenge: Relying on sensors to understand the world
- Noisy sensors
- Local coordinates to global coordinates (transforms)
- Motion involved
- May change over time
- Many different representations, each with advantages/disadvantages

---

### Slide 17 — Map Categories

> **Source:** PDF p. 17

- Typical maps are loosely categorised as:
- Metric:
- Accurately represents distance
- Topological:
- Accurately represents connectivity
- Semantic:
- Additionally associates “descriptive meaning” with locations
- Some maps contain all of the above

---

### Slide 18 — Occupancy Grids

> **Source:** PDF p. 18

- Good for navigation
- Discretise the world into cells (2D) or
- voxels (3D)
- Grid structure is rigid
- Each cell is assumed to be occupied or
- free space
- Requires substantial memory resources
- Created from LIDAR data using ray casting
- Standard ROS packages
- Will discuss further in next section

---

### Slide 19 — Terrain Map

> **Source:** PDF p. 19

- Good for navigating complex terrain
- Similar to occupancy grid, but describes properties of the terrain, such as elevation or
- surface type
- Good for modelling complex 3D terrains
- Elevation often represented as a contour map

---

### Slide 20 — Distance Function Maps

> **Source:** PDF p. 20

- Defines the
- distance
- to occupied space
- Useful for safe navigation
- Useful for fast ray tracing
- Dense representation, grid (2D) or voxels (3D)
- Expensive to compute

---

### Slide 21 — Feature Maps

> **Source:** PDF p. 21

- Good for localisation
- Landmark Based
- Kalman filter or optimisation systems (SLAM)
- Map of trees in a park:
- jillhubley.com/project/prospectparktrees/
- Compact representation
- Multiple feature observations to improve position
- estimates
- Landmarks on
- Mars
- Victoria Park dataset

---

### Slide 22 — Trajectory Tracking

> **Source:** PDF p. 22

- Movements of objects of interest
- Similar to feature map, but the
- landmarks are moving over time
- GPS tracks of whales in
- Atlantic Ocean
- Tracking pedestrians in a
- train station

---

### Slide 23 — Topological Maps

> **Source:** PDF p. 23

- Explains connectivity between locations
- Sydney train network
- May have descriptions associated with places or paths
- Represented as a
- graph
- of vertices and edges
- Doesn’t
- necessarily
- preserve distances, scale, or directions
- Sydney Botanic Gardens
- Robot navigation graphs,
- e.g. Probabilistic Roadmap (PRM)
- Royal National Park

---

### Slide 24 — Semantic Maps

> **Source:** PDF p. 24

- Adds descriptions to a map that are “meaningful”
- High level robot commands, e.g. “go to office”
- Human-robot interaction
- Map of trees in a park:
- jillhubley.com/project/prospectparktrees/
- Royal national park

---

### Slide 25 — Environmental Phenomena

> **Source:** PDF p. 25

- Gaussian Process regression
- Quantities described as a continuous field:
- Temperature
- 3 possible
- Quantity of
- Elevation
- functions
- interest
- Soil type
- Observations
- Sound volume
- Uncertainty
- Communication bandwidth
- Location
- Ground water
- salinity
- Wireless signal strength
- Soil pH level

---

### Slide 26 — Which are most meaningful for your project?

> **Source:** PDF p. 26

#### Visual Content

**Type:** Image-heavy slide (9 embedded images)

**OCR-extracted labels/text:**

```
Which are most meaningful for your project?
Wao Pe
fon Se fa i i) -
a 4 eee Cee
a a ag is .
f "] ree ys a
: disc 1 : i at pe
Naa a ad i
2°83 e) Wwe — reer
Fy SS . fi ‘Ca Rw a i legend
we ie ; wnt ee ae Yee fo, S, "Staton Ie a 3
oS \ ae a 9
a 7; Ea 7 FANE, wt & SMTA Onmann Orn ‘
oe - ih ° reer, al Sits, - en . ’ ‘i © Sheer O Woiters eS = iy %, a:
S Tome) ae 4 DG # a | 7
f : pl 4 f Le ; Ne © campsite La Patking een a Es
as Crea y) Mes ys 4 Yea a Bpociontethtng ue . ae
Lae YW, Samm! 4 Gt a sm Dither walking route *2 Cycling rou a
LEG ae oA & 24 aft i - ss i * “ see Gases =
26
```

---

### Slide 27 — Occupancy Grids

> **Source:** PDF p. 27

- Occupancy Grids

#### Visual Content

**Type:** Image-heavy slide (2 embedded images)

**OCR-extracted labels/text:**

```
S . ° ° oO e e O O ° e cS . : 5 OC e@ e oO
OO 0 e e e ° : . ° [> © © e e e e 2 :
e O OD e @ O : : . O : e OO) C e @ O . :
e 0 O e e@ O ° ° ° O° ° e O 0 e e@ O o °
Occupancy Grids
```

---

### Slide 28 — Occupancy Grid

> **Source:** PDF p. 28

- Originally developed for very noisy sensors (sonar)
- Create on-the-fly as a robot moves through an environment
- Need a way to model uncertainty

---

### Slide 29 — Occupancy Probability

> **Source:** PDF p. 29

- Each cell is a binary random variable that models occupancy
- Cell is occupied
- p(m) = 1
- Cell is not occupied
- p(m) = 0
- No knowledge
- p(m) = 0.5

---

### Slide 30 — Occupancy Grid Map

> **Source:** PDF p. 30

- Discrete array of cells, each with an occupancy variable
- m
- - > {free, occupied} ->
- {0,1}
- x
- y
- m
- m
- 1,1
- 1,2
- m
- m
- 2,1
- 2,2

---

### Slide 31 — Problem: Estimate Map from Sensor Data

> **Source:** PDF p. 31

#### Visual Content

**Type:** Problem formulation

**Problem:** Estimate map from sensor data

Given sensor data $z_{1:t}$ and poses $x_{1:t}$ of the sensor, estimate:
$$p(m \mid z_{1:t}, x_{1:t}) = \prod_i p(m_i \mid z_{1:t}, x_{1:t})$$

Each cell $m_i$ is a **binary random variable** → **Binary Bayes filter** (for a static state)

---

### Slide 32 — Bayesian Filtering

> **Source:** PDF p. 32

- Recursively update
- p(m)
- for each cell
- y
- p(m
- )
- 1,1
- p(m
- 1,2
- )
- p(m
- )
- p(m
- )
- p(m
- )
- x,y
- 2,1
- 2,2
- 0  1
- p(m
- x,y
- )

---

### Slide 33 — Incorporating an Observation

> **Source:** PDF p. 33

- Let’s say our range-finder reports a reading of 5m
**Before:**
**After:**
- there isn’t
- something here
- there is
- something
- somewhere
- 5 m
- 5 m
- around here
- unoccupied
- no information
- occupied

---

### Slide 34 — Noisy Observations

> **Source:** PDF p. 34

- Measurement model for a sonar

#### Visual Content

**Type:** Image-heavy slide (4 embedded images)

**OCR-extracted labels/text:**

```
Noisy Observations >
44
Cee
~~
>» Measurement model for a sonar ~~»
I
Occupancy probability ——
08
z+d, zd,
0.6
P prior
04 Zz z+d,
o> z-d,
measured dist.
0
0 0.5 1 1.5 2 2.5 3
distance between the cell and the sensor
```

---

### Slide 35 — Applying Bayes’ Theorem

> **Source:** PDF p. 35

- Posterior map
- Measurement model
- Prior map
- p( z | m )
- p( m | z )
- p( m )
- p( z | m ) p( m )
- p( m | z )  =
- Bayes’ rule
- p( z )

---

### Slide 36 — Recursive Bayes’ Filter

> **Source:** PDF p. 36

#### Visual Content

**Type:** Recursive Bayes filter flowchart

**Pipeline:**
1. Prior belief (occupancy grid)
2. Measurement update (incorporate sensor observation)
3. Posterior belief (updated grid)

**Repeats** as robot moves and collects new observations.

---

### Slide 37 — Recursive Bayes’ Filter

> **Source:** PDF p. 37

#### Visual Content

**Type:** Image-heavy slide (3 embedded images)

**OCR-extracted labels/text:**

```
. J . @
Recursive Bayes’ Filter +
%
Ce
~~
* * *
2
* * *
* * * ~
```

---

### Slide 38 — ROS2 Packages

> **Source:** PDF p. 38

- Occupancy grids are very common on robots, especially for enabling path planning
- Many common packages, for example see the nav2 package:
- https://docs.nav2.org/setup_guides/sensors/mapping_localization.html
- This is used in the 41068 starter package
- This enables basic occupancy grid mapping
- But does
- not
- provide other types of maps we
- discussed earlier – for these you will need to
- look elsewhere, or create them yourself!

---

### Slide 39 — Hierarchical Data Structures

> **Source:** PDF p. 39

- OctoMap 3D mapping library
- Tree data structure
- Multi-resolution queries

#### Visual Content

**Type:** Image-heavy slide (5 embedded images)

**OCR-extracted labels/text:**

```
Hierarchical Data Structures
a er.
> OctoMap 3D mapping library Ee roa
Tree data structure Ge 4 42
L Vo
ae es" * zi ie
<p et a ae Pie
Multi-resolution queries ae ae oR a oe oa
7 . = Bo Bet rae ra
Soo *" S
US 39
```

---

### Slide 40 — Simultaneous Localisation and Mapping (SLAM)

> **Source:** PDF p. 40

- If we have a map:
- We can localise!
- NOT THAT SIMPLE!
- If we can localise:
- We can make a map!

---

### Slide 41 — Reflection

> **Source:** PDF p. 41

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

### Slide 42 — Conclusions

> **Source:** PDF p. 42

- Robots typically have many different sensors
- Robot mapping
- is the process by which a robot builds a representation of its environment
- using data from its sensors (and prior information).
- Sensors and maps are useful for:
- Enabling autonomy
- : Understand location of obstacles, landmarks, traversability, navigable areas
- Primary tasks
- : Provide information to humans and other robots that is specific to the task. May be
- provided in a
- user interface
- .
- Many different types of maps for different purposes:
- Occupancy, terrain, feature, topological, semantic, environmental, … maps
- Occupancy grid mapping fuses data from multiple noisy observations

---

### Slide 43 — Questions for your 41068 Project

> **Source:** PDF p. 43

- Ideas for different types of maps
- Your project must have some perception/mapping
- Your project must have a user interface
- What types of maps will your robot produce?
- For primary task
- For autonomy
- What information will these maps contain?
- How will you show these maps in your User Interface?

---

## Extraction Metadata

- **Source:** `perception_mapping.pdf`
- **Pages processed:** 43 / 43
- **Extraction date:** 2026-08-28T15:37:13.640400
- **Slides with curated visual semantics:** 5
- **Slides with OCR/visual fallback:** 15
- **Text-primary slides:** 28
- **OCR engine:** Tesseract 4.1.1
- **Native text extraction:** PyMuPDF (fitz)
- **Document creator:** Microsoft® PowerPoint® for Microsoft 365

## Extraction Uncertainties

1. Some image-heavy slides rely on curated visual descriptions or OCR; fine diagram details may require referring to the source PDF.
2. Mathematical notation in slides uses Unicode italics from PowerPoint; LaTeX equivalents are provided where reconstructed.
3. Photo collage slides (e.g. motivation/examples) contain information not fully transcribed at label level.
4. Animation slides (e.g. frontier exploration) are represented as sequential descriptions, not frame-by-frame data.
