# Simulation Package Overview

## Document Overview

- **Document type:** PowerPoint seminar slides (PDF export)
- **Author:** Graeme Best
- **Organisation:** University of Technology Sydney — Faculty of Engineering and Information Technology
- **Course:** 41068 Robotics Studio 1
- **Creation date:** 2026-08-06
- **Pages / slides:** 33
- **Primary subject:** 41068 ROS 2 / Gazebo simulation starter package structure and usage
- **Source file:** `simulation_package.pdf`

## Contents

- [Slide 2 — The 41068 Simulation Starter Package](#slide-2)
- [Slide 3 — Questions for your 41068 Project](#slide-3)
- [Slide 4 — Outline](#slide-4)
- [Slide 5 — ROS and Gazebo refresher](#slide-5)
- [Slide 6 — What is the Robot Operating System (ROS)?](#slide-6)
- [Slide 7 — Key ROS Concepts](#slide-7)
- [Slide 8 — Relatively Simple Example](#slide-8)
- [Slide 9 — ROS, Gazebo, and RViz](#slide-9)
- [Slide 10 — ROS Learning Resources](#slide-10)
- [Slide 11 — Understanding the Supplied Package](#slide-11)
- [Slide 12 — What You Should See](#slide-12)
- [Slide 13 — Package Structure](#slide-13)
- [Slide 14 — Where to Get Started: README.md](#slide-14)
- [Slide 15 — Launch Files](#slide-15)
- [Slide 16 — What the Main Launch File Starts](#slide-16)
- [Slide 17 — Other Launch Files](#slide-17)
- [Slide 18 — Namespaces and TF Frames](#slide-18)
- [Slide 19 — Config Files](#slide-19)
- [Slide 20 — Two Supplied Robots](#slide-20)
- [Slide 21 — Robot Models](#slide-21)
- [Slide 22 — World Files](#slide-22)
- [Slide 23 — Models, Textures, and Gazebo Fuel](#slide-23)
- [Slide 24 — Provided Script 1: Dynamic World Demo](#slide-24)
- [Slide 25 — Provided Script 2: Basic Autonomy Demo](#slide-25)
- [Slide 26 — Ideas for Extending the Package](#slide-26)
- [Slide 27 — Dimensions You Could Change](#slide-27)
- [Slide 28 — Examples from 41068 in 2025](#slide-28)
- [Slide 29 — Procedurally-Generated World Files](#slide-29)
- [Slide 30 — Further Resources](#slide-30)
- [Slide 31 — Reflection](#slide-31)
- [Slide 32 — Reminder of Expectations](#slide-32)
- [Slide 33 — Questions for your 41068 Project](#slide-33)

## Main Content

### Slide 1 — Simulation Package

> **Source:** PDF p. 1

- Overview
- Graeme Best
- Faculty of Engineering and Information Technology
- University of Technology Sydney

---

### Slide 2 — The 41068 Simulation Starter Package

> **Source:** PDF p. 2

- ROS 2 package
- Gazebo world of a forest environment
- Ground and aerial robots with sensors
- Simple autonomy stack
- Example autonomy code
- Example dynamic world features
- Just enough to get started!
- You should adapt the simulation to
- meet the needs of your project:
- Expanded world
- New objects
- Modified robots
- …

---

### Slide 3 — Questions for your 41068 Project

> **Source:** PDF p. 3

- How will you adapt the simulation environment to support your project?
- How will you adapt the robots and sensors?
- How will you adapt and develop the autonomy?
- How will you test and evaluate your system?

---

### Slide 4 — Outline

> **Source:** PDF p. 4

- Very brief ROS 2 refresher
- Relationship between ROS, Gazebo and RViz
- Package structure
- Launch files
- Namespaces
- Gazebo-ROS bridges
- Config files
- Robot descriptions
- Worlds and models
- Supplied code
- Ideas for adapting the simulation

---

### Slide 5 — ROS and Gazebo refresher

> **Source:** PDF p. 5

- ROS and Gazebo refresher

#### Visual Content

**Type:** Image-heavy slide (2 embedded images)

**OCR-extracted labels/text:**

```
S . ° ° Oo e@ e O O e e cS . : 9 OC e@ e oO
©) C) O e @ e © : ° e . C) C) C) e e e 6 .
e@ CO) O) e@ e CO . : . ® ° e@ C) e e@ e@ C) . :
e Oo OQ e e O ° ° . ° ° e O O e e © ° °
ROS and Gazebo refresher
```

---

### Slide 6 — What is the Robot Operating System (ROS)?

> **Source:** PDF p. 6

- ROS 2 is a collection of software libraries, communication tools and
- conventions for developing robot systems
- A ROS system is normally composed of many separate software
- components
- These components exchange data rather than being combined into one
- large program
- ROS provides interfaces and communication

---

### Slide 7 — Key ROS Concepts

> **Source:** PDF p. 7

- Nodes
- : running software components
- Topics
- : continuous streams of data
- Messages
- : the structure of data sent through topics
- Parameters
- : the values used to configure nodes
- Services
- : request-and-response interactions
- Actions
- : longer-running goals with feedback and results
- TF
- : coordinate frames and transformations
- Launch files
- : start and configure collections of nodes

---

### Slide 8 — Relatively Simple Example

> **Source:** PDF p. 8

- node
- topic
- Physical or
- Raw sensor data
- Motor commands
- Simulated
- Sensing
- Robot
- Lidar
- scan
- Lidar
- driver
- Waypoint
- Path
- Mapping
- Obstacle
- planning
- Location
- map
- Controller
- GPS
- Goal
- driver
- User
- User
- Perception
- Interface
- interface
- Control
- Planning
- Image
- Object
- Camera
- Object
- location
- driver
- detection
- Decision
- Making

#### Visual Content

**Type:** ROS system architecture diagram (from slide 8 native text)

**Node/topic graph showing:**
- Physical/Simulated Robot
- Sensor drivers: Lidar, GPS, Camera
- Topics: scan, Image, Motor commands
- Processing nodes: Mapping, Obstacle planning, Object detection, Controller
- Outputs: map, Path, Waypoint, Goal, Object location
- User Interface
- Modules: Sensing, Perception, Planning, Control, Decision Making

---

### Slide 9 — ROS, Gazebo, and RViz

> **Source:** PDF p. 9

- Ignition Gazebo
- ROS
- RViz
- Visualises:
- Simulates:
- Provides:
- The environment
- Communication between
- Robot models
- software components
- Robot geometry
- Sensor data
- Topics, actions, services,
- Collision and physics
- Maps
- and parameters
- Paths
- Joints and actuators
- Autonomy: Mapping,
- Cameras, lidar and IMUs
- localisation, planning,
- TF frames
- control, [your code], …
- Navigation goals
- …

---

### Slide 10 — ROS Learning Resources

> **Source:** PDF p. 10

- This seminar is not a replacement for the ROS tutorials
- The best way to learn is to try it yourself!
- Use the 41068 package README and working examples
- Revisit 41012 PFMS and related subjects
- Official ROS 2 tutorials
- https://docs.ros.org/en/humble/Tutorials.html

---

### Slide 11 — Understanding the Supplied Package

> **Source:** PDF p. 11

- Understanding the Supplied Package

#### Visual Content

**Type:** Image-heavy slide (2 embedded images)

**OCR-extracted labels/text:**

```
S . ° ° Oo e@ e O O e e cS . : 9 OC e@ e oO
OO 0 e e e 2 . . ° : OO e e e e ° :
e O OD e @ O : ° . O : e OO) C e @ O . :
e Oo OQ e e O ° ° . ° ° e O O e e © ° °
Understanding the Supplied Package
```

---

### Slide 12 — What You Should See

> **Source:** PDF p. 12

- RViz
- Basic
- Visualisation:
- autonomy
- UGV
- Gazebo
- simulator
- Forest
- world
- Cameras
- RViz
- Visualisation:
- Dynamic
- UAV
- objects
- 2 robots

#### Visual Content

**Type:** Expected simulation screenshot

**Three windows:**

1. **Gazebo simulator (left):** Forest world, 2 robots (Husky UGV visible), dynamic objects (beige sphere on ground)

2. **RViz Visualisation: UGV (top right):**
   - Basic autonomy: 2D occupancy map with explored area (white), obstacles (black/red), robot pose arrow
   - Cameras: first-person forest view
   - Config: `41068_husky1.rviz`

3. **RViz Visualisation: UAV (bottom right):**
   - Similar map view from aerial perspective
   - Camera shows UGV and dynamic object from above
   - ROS Time ~173.78, ~27 fps

---

### Slide 13 — Package Structure

> **Source:** PDF p. 13

- 41068_ignition_bringup/
- ├── config/
- configuration and parameter files
- ├── launch/
- launch files: start different combinations of the simulation
- ├── models/
- custom reusable Gazebo environment models
- ├── scripts/
- two ROS Python examples
- ├── urdf_husky/
- Husky UGV robot, sensors and Gazebo plugins
- ├── urdf_parrot/
- Parrot UAV robot, sensors and Gazebo plugins
- ├── worlds/
- Gazebo world files
- ├── CMakeLists.txt
- installs the package resources and executable scripts
- ├── package.xml
- dependencies and package metadata
- └── README.md
- installation and launch instructions and tips

---

### Slide 14 — Where to Get Started: README.md

> **Source:** PDF p. 14

- Follow the README installation instructions
- Build the workspace
- Launch a simpler configuration first
- Confirm it runs without error
- Drive the robot around
- Try the more complicated configurations
- Preserve this state in Git
- Start making changes

---

### Slide 15 — Launch Files

> **Source:** PDF p. 15

- The canonical launch file is:
- 41068_ignition.launch.py
- It supports these main options:
- husky:=true/false
- parrot:=true/false
- slam:=true/false
- nav2:=true/false
- rviz:=true/false
- world:=simple_trees/large_demo
- 41068_ignition_husky.launch.py
- calls the main launch file with Husky only
- calls the main launch file with Parrot only
- 41068_ignition_parrot.launch.py

---

### Slide 16 — What the Main Launch File Starts

> **Source:** PDF p. 16

- The canonical launch file performs the following sequence:

- 1. Starts the Gazebo world and bridges the simulation clock
- 2. Generates and spawns each enabled robot
- 3. Starts robot state publishing, localisation, and the robot-specific Gazebo bridge
- 4. Optionally starts namespaced SLAM and Nav2 instances for each robot
- 5. Optionally starts one namespaced RViz process for each robot

---

### Slide 17 — Other Launch Files

> **Source:** PDF p. 17

- 41068_navigation.launch.py
- Starts namespaced SLAM and Nav2
- Helper launch file – shouldn’t usually be called directly
- 41068_dynamic_world_demo.launch.py
- Starts only the dynamic-world node (described later)
- Assumes the main launch file is already running
- 41068_autonomy_demo.launch.py
- Starts only the example autonomy node (described later)
- Assumes the main launch file is already running

---

### Slide 18 — Namespaces and TF Frames

> **Source:** PDF p. 18

- The supplied robots are always
- namespaced
- , even when running only one robot
- Namespaces allow separate copies of nodes/topics for each robot
- Multiple robots are an option – start by getting one robot working first!
- Husky
- Parrot
- ROS namespace:   /husky1
- ROS namespace:   /parrot1
- ROS topics:      /husky1/scan
- ROS topics:      /parrot1/scan
- /husky1/camera/image
- /parrot1/camera/image
- /husky1/cmd_vel
- /parrot1/cmd_vel
- TF frame prefix: husky1_
- TF frame prefix: parrot1_
- TF frames:       husky1_map
- TF frames:       parrot1_map
- husky1_odom
- parrot1_odom
- husky1_camera_link
- parrot1_camera_link
- Gazebo model:    husky1
- Gazebo model:    parrot1

---

### Slide 19 — Config Files

> **Source:** PDF p. 19

- Gazebo-ROS bridges
- Global clock
- Husky/Parrot topics
- Robot localisation
- Parameters for the localisation node
- SLAM toolbox
- Parameters for the SLAM node
- Nav2
- Parameters for navigation, planning, and control
- Rviz
- RViz visualisation configuration
- Best edited directly in rviz then saved

---

### Slide 20 — Two Supplied Robots

> **Source:** PDF p. 20

- Husky:
- Parrot:
- Four-wheeled ground robot
- Visually resembles a quadrotor
- Differential-drive control
- Deliberately simplified
- Simulated odometry
- Uses a Husky-like planar movement
- IMU
- Gravity is disabled
- GPU lidar
- Operates at a fixed altitude
- RGB-D camera
- Collisions are disabled so it can navigate
- through leaves

---

### Slide 21 — Robot Models

> **Source:** PDF p. 21

- URDF.xacro files describe:
- Links and joints
- Visual and collision geometry
- Inertial properties
- Robot and sensor frames
- Reusable variables and macros
- Gazebo.xacro files add:
- Simulated sensors and their parameters
- Gazebo topic and model names
- Drive and controller plugins

---

### Slide 22 — World Files

> **Source:** PDF p. 22

- Lighting
- The package contains two provided world files
- simple_trees.sdf
- Simple grass plane
- Physics
- Small number of trees
- Check the basic simulation works
- large_demo.sdf
- Custom forest ground texture
- Forest boundary walls
- Terrain
- Collection of trees, rocks, other objects
- Dynamic demo objects (see later slide)
- Richer project starting environment
- Trees,
- rocks…

---

### Slide 23 — Models, Textures, and Gazebo Fuel

> **Source:** PDF p. 23

- Worlds contain objects from Gazebo Fuel:
- https://app.gazebosim.org/fuel/models
- The package also contains custom models and
- textures in the
- directory
- models
- Supports other formats, like 3D meshes
- You can try adding other Fuel objects
- or create your own!
- Incrementally add complexity – complex models
- can significantly slow down the simulation

---

### Slide 24 — Provided Script 1: Dynamic World Demo

> **Source:** PDF p. 24

- It’s possible to change parameters of a Gazebo world dynamically from your code!
- Position, colour, visibility, lighting, …
- dynamic_world_demo.py
- demonstrates how Python code can alter Gazebo:
- Moves a small animal marker randomly around the forest
- Cycles a tree between healthy, fire and burnt appearances
- See the code to find out how this works
- Adapt the code to develop other dynamic features!

---

### Slide 25 — Provided Script 2: Basic Autonomy Demo

> **Source:** PDF p. 25

- basic_autonomy_demo.py
- provides skeleton autonomy code:
- connects perception, mapping and navigation
- Demonstrates how several ROS interfaces can be connected inside one node
- Is deliberately not intended to be a good perception/planning algorithm
- The node does the following:
- Subscribes to a map and an RGB image
- Estimates the brightness of each image
- Uses TF to find the robot pose in the map
- Sample waypoint in known free space. Brighter image = distant goal; darker image = closer goal
- Sends the waypoint to Nav2
- When goal is reached, chooses another goal

---

### Slide 26 — Ideas for Extending the Package

> **Source:** PDF p. 26

- Ideas for Extending the Package

#### Visual Content

**Type:** Image-heavy slide (2 embedded images)

**OCR-extracted labels/text:**

```
S . ° ° Oo e@ e O O e e cS . : 9 OC e@ e oO
e © O e e e 0 : . . : OO e e e e ° :
e © Oo Cn) Oo : : : 0 ° e C) C ee e | . .
e Oo OQ e e O ° ° . ° ° e O O e e © ° °
Ideas for Extending the Package
```

---

### Slide 27 — Dimensions You Could Change

> **Source:** PDF p. 27

- The starter package is only a starting point!
- There are many things you can and should change
- Environment
- Dynamic features
- Robot
- Sensors
- Autonomy (perception, decision making, user interface)
- Multi-robot coordination and communication
- …
- Choose changes that support your project. More complexity is not automatically better.

---

### Slide 28 — Examples from 41068 in 2025

> **Source:** PDF p. 28

#### Visual Content

**Type:** Student project examples from 41068 (2025)

**Shows:** Screenshots/photos of diverse student project adaptations of the simulation package — varied environments, robot configurations, and autonomy implementations.

[Specific project details not legible at extraction resolution]

---

### Slide 29 — Procedurally-Generated World Files

> **Source:** PDF p. 29

- World files follow the “SDF” (Simulation Description Format) format described here:
- https://sdformat.org/spec/
- SDF files are a type of XML file with predefined tags relevant to robotics simulation:
- World, model, link, joint, pose, visual, collision, material, geometry, sensor, plugin, light, physics, …
- Modify world files in software:
- Read in a functioning world file using any
- XML parser
- Modify/copy/remove XML objects using code
- Export the modified XML file
- Open new world file in Gazebo, check functionality
- Get creative – can even create randomised worlds!

---

### Slide 30 — Further Resources

> **Source:** PDF p. 30

- Gazebo Fuel:
- https://app.gazebosim.org/fuel/models
- SDF file format:
- https://sdformat.org/spec/
- An XML parser (there are many others):
- https://docs.python.org/3/library/xml.etree.elementtree.html
- ROS 2 tutorials:
- https://docs.ros.org/en/humble/Tutorials.html
- Gazebo ignition:
- https://gazebosim.org/home
- Nav2 autonomy:
- https://docs.nav2.org/
- Turtlebot autonomy stack:
- https://turtlebot.github.io/turtlebot4-user-manual/
- 41068 seminars will spark ideas for other technical concepts

---

### Slide 31 — Reflection

> **Source:** PDF p. 31

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

### Slide 32 — Reminder of Expectations

> **Source:** PDF p. 32

- The supplied package gives you a working starting point
- You are not expected to use every supplied component
- E.g., Start with 1 robot – it is not compulsory to have more than 1 robot
- You should change the simulation in any way relevant to your application:
- Modify the supplied environment
- Add/remove robots and sensors
- Upgrade the autonomy stack
- Create new ROS nodes
- Use a completely different simulation setup
- Your goal is to create a simulation and software system that supports your proposed project
- Don’t make it unnecessarily complicated, since it will slow down your simulation speed

---

### Slide 33 — Questions for your 41068 Project

> **Source:** PDF p. 33

- How will you adapt the simulation environment to support your project?
- How will you adapt the robots and sensors?
- How will you adapt and develop the autonomy?
- How will you test and evaluate your system?

---

## Extraction Metadata

- **Source:** `simulation_package.pdf`
- **Pages processed:** 33 / 33
- **Extraction date:** 2026-08-28T15:37:30.179586
- **Slides with curated visual semantics:** 3
- **Slides with OCR/visual fallback:** 7
- **Text-primary slides:** 26
- **OCR engine:** Tesseract 4.1.1
- **Native text extraction:** PyMuPDF (fitz)
- **Document creator:** Microsoft® PowerPoint® for Microsoft 365

## Extraction Uncertainties

1. Some image-heavy slides rely on curated visual descriptions or OCR; fine diagram details may require referring to the source PDF.
2. Mathematical notation in slides uses Unicode italics from PowerPoint; LaTeX equivalents are provided where reconstructed.
3. Photo collage slides (e.g. motivation/examples) contain information not fully transcribed at label level.
4. Animation slides (e.g. frontier exploration) are represented as sequential descriptions, not frame-by-frame data.
