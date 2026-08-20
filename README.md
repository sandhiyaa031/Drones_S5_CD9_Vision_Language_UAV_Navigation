<p align="center">
  <img src="amrita.png" alt="Logo" width="400"/>
</p>

# Vision-Language UAV Navigation for GPS-Denied Environments

### Vision-Language Navigation | UAV | Computer Vision | GPS-Denied Navigation | ROS 2 | PX4 | Gazebo

</div>

---

## Team Members

| S. No. | Name | Roll Number | Email |
|---|---|---|---|
| 1 | JENISHAA BHARATHI M | CB.SC.U4AIE24221 | cb.sc.u4aie24221@cb.students.amrita.edu |
| 2 | NAVEEN K | CB.SC.U4AIE24235 | cb.sc.u4aie24235@cb.students.amrita.edu |
| 3 | POOJA N | CB.SC.U4AIE24242 | cb.sc.u4aie24242@cb.students.amrita.edu |
| 4 | PRANESH M | CB.SC.U4AIE24345 | cb.sc.u4aie24345@cb.students.amrita.edu |
| 5 | SANDHIYA D | CB.SC.U4AIE24353 | cb.sc.u4aie24353@cb.students.amrita.edu |

---

# Abstract

Unmanned Aerial Vehicles (UAVs) traditionally rely on predefined GPS waypoints or precise manual control, limiting their flexibility in dynamic environments. This project introduces a **Vision-Language Navigation framework** that enables autonomous drones to understand and execute natural language navigation commands. By combining computer vision with natural language processing, the system interprets instructions such as **"Navigate past the blue sign and stop near the door"**, processes live camera input, identifies relevant visual targets, and dynamically generates navigation commands.

The framework provides a modular foundation for developing, simulating, and testing intelligent aerial navigation systems in GPS-denied environments.

---

# Introduction

### Evolving Aerial Autonomy
UAVs are transitioning from manually operated aircraft to autonomous systems capable of navigating complex and unpredictable environments.

### Visual Perception Challenge
Real-time interpretation of camera data and rapid flight adjustment remain major challenges in autonomous aerial navigation.

### Integrated ROS 2 Architecture
This project combines computer vision with a modular **ROS 2 flight-control architecture** for visual target tracking and autonomous navigation.

### Simulation
**Gazebo and PyBullet** provide realistic physics, sensor data, and camera simulation for safe testing without physical hardware.

### PX4 Integration
**PX4** provides flight stabilization, telemetry, waypoint management, and low-level control required for autonomous UAV operation.

---

# Methodology

The system combines the following core modules:

- **PyBullet + gym-pybullet-drones:** UAV, camera, physics, and target-tracking simulation.
- **OpenCV:** Camera processing and visual target detection.
- **Visual Servoing + PID:** Converts image-space target errors into UAV motion commands.
- **ROS 2:** Communication layer connecting perception, localization, and flight control.
- **PX4 SITL + Gazebo:** Autonomous flight simulation and low-level control.
- **PX4 State Bridge:** Publishes UAV odometry and state to ROS 2.
- **NED–ENU + TF/Odometry:** Maintains consistent UAV coordinate frames.
- **PX4 Offboard Control:** Sends position setpoints to the UAV.
- **QGroundControl:** Provides telemetry and flight-state monitoring.

### Flight Sequence

```text
ARM → TAKEOFF → HOVER → WAYPOINT → RETURN → LAND
```

---

# System Architecture

```mermaid
flowchart TD
    A["Natural Language Command"] --> B["Language Processing"]
    B --> C["Visual Target Detection"]

    D["UAV Camera"] --> C
    C --> E["Visual Servoing + PID"]
    E --> F["Position Setpoint"]

    G["IMU / Odometry"] --> H["Localization"]
    H --> E

    F --> I["ROS 2"]
    I --> J["PX4 Offboard Control"]
    J --> K["PX4 SITL"]
    K --> L["Gazebo UAV"]

    L --> D
    K --> M["QGroundControl"]
```

---

# Toy Example

### Command

> **"Find the red building near the road and fly to it."**

### Language Interpretation

```text
Object    = Building
Attribute = Red
Relation  = Near Road
Action    = Navigate
```

### Target Detection

The camera image and language instruction are used to identify the target:

$$
B = Ground(I,L)
$$

where:

- $I$ = camera image
- $L$ = language instruction
- $B$ = detected target region

### Position Error

Suppose:

```math
P_{UAV}=(0,0,5)\;m
```

```math
P_{target}=(10,8,5)\;m
```

The position error is:

```math
e_p=P_{target}-P_{UAV}
```

Therefore:

```math
e_p=
\begin{bmatrix}
10\\
8\\
0
\end{bmatrix}
\;m
```

The distance to the target is:

```math
d=\sqrt{(10-0)^2+(8-0)^2+(5-5)^2}
```

Therefore:

```math
d=\sqrt{164}\approx12.81\;m
```

### Navigation

The UAV generates intermediate position setpoints:

```text
(0,0,5)
   ↓
(3,2,5)
   ↓
(6,5,5)
   ↓
(10,8,5)
```

A position setpoint is represented as:

```math
W_i=[x_i,y_i,z_i,\psi_i]
```

The UAV considers the target reached when:

```math
d<\epsilon
```

where $\epsilon$ is the allowed position error.


Uploading Screencast from 2026-08-18 21-46-16.mp4…



## Base Paper

This project is inspired by the UAV Vision-Language Navigation problem studied in:

> **Towards Realistic UAV Vision-Language Navigation: Platform, Benchmark, and Methodology**

**Paper:**  
https://arxiv.org/pdf/2410.07087

**TravelUAV Repository:**  
https://github.com/buaa-colalab/TravelUAV

The paper provides the research foundation for the Vision-Language Navigation aspect of this project, while this repository develops an independent simulation and ROS 2–PX4 based implementation.

---

<div align="center">

### Amrita Vishwa Vidyapeetham

**Vision-Language UAV Navigation for GPS-Denied Environments**

</div>
