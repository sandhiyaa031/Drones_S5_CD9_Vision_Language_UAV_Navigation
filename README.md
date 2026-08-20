# Vision-Language UAV Navigation for GPS-Denied Environments

<p align="center">

<img src="Images/logo-branding-amrita-universiy-2024.jpg" alt="Amrita Vishwa Vidyapeetham" width="900">

</p>

[![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E.svg)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Simulation-orange.svg)](https://gazebosim.org/)
[![PX4](https://img.shields.io/badge/PX4-SITL-blue.svg)](https://px4.io/)
[![QGroundControl](https://img.shields.io/badge/QGroundControl-GCS-green.svg)](https://qgroundcontrol.com/)
[![Python](https://img.shields.io/badge/Python-3.10+-blue.svg)](https://www.python.org/)
[![OpenCV](https://img.shields.io/badge/OpenCV-Computer%20Vision-red.svg)](https://opencv.org/)
[![PyTorch](https://img.shields.io/badge/PyTorch-Deep%20Learning-red.svg)](https://pytorch.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

**Autonomous Aerial Robotics | Vision-Language Navigation | Semantic Grounding | SLAM | GPS-Denied Navigation | ROS 2 | PX4 | Gazebo**

---

# Team Members

| S. No. | Name                | Roll Number      | Email                                                                                     |
| ------ | ------------------- | ---------------- | ----------------------------------------------------------------------------------------- |
| 1      | JENISHAA BHARATHI M | CB.SC.U4AIE24221 | [cb.sc.u4aie24221@cb.students.amrita.edu](mailto:cb.sc.u4aie24221@cb.students.amrita.edu) |
| 2      | NAVEEN K            | CB.SC.U4AIE24235 | [cb.sc.u4aie24235@cb.students.amrita.edu](mailto:cb.sc.u4aie24235@cb.students.amrita.edu) |
| 3      | POOJA N             | CB.SC.U4AIE24242 | [cb.sc.u4aie24242@cb.students.amrita.edu](mailto:cb.sc.u4aie24242@cb.students.amrita.edu) |
| 4      | PRANESH M           | CB.SC.U4AIE24345 | [cb.sc.u4aie24345@cb.students.amrita.edu](mailto:cb.sc.u4aie24345@cb.students.amrita.edu) |
| 5      | SANDHIYA D          | CB.SC.U4AIE24353 | [cb.sc.u4aie24353@cb.students.amrita.edu](mailto:cb.sc.u4aie24353@cb.students.amrita.edu) |

---
# Adaptive Semantic Memory for Vision-Language Autonomous UAV Navigation in GPS-Denied Environments


[![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E.svg)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Simulation-orange.svg)](https://gazebosim.org/)
[![ArduPilot](https://img.shields.io/badge/ArduPilot-SITL-red.svg)](https://ardupilot.org/)
[![Python](https://img.shields.io/badge/Python-3.10+-blue.svg)](https://www.python.org/)
[![PyTorch](https://img.shields.io/badge/PyTorch-2.x-red.svg)](https://pytorch.org/)
[![ROS2](https://img.shields.io/badge/ROS2-Robotics-22314E.svg)](https://www.ros.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

**Autonomous Aerial Robotics | Vision-Language Navigation | GPS-Denied Navigation | Semantic Memory**

</div>

---

## Team Members


| Name                | Roll Number      |
| ------------------- | ---------------- |
| JENISHAA BHARATHI M | CB.SC.U4AIE24221 |
| NAVEEN K            | CB.SC.U4AIE24235 |
| POOJA N             | CB.SC.U4AIE24242 |
| PRANESH M           | CB.SC.U4AIE24345 |
| SANDHIYA D          | CB.SC.U4AIE24353 |
---
<p align="center">
  <img src="Images/logo-branding-amrita-universiy-2024.jpg"
       alt="Amrita Vishwa Vidyapeetham"
       width="900">
</p>

# Vision-Language UAV Navigation

## Team Members

| S. No. | Name | Roll Number | Email | Role |
|---|---|---|---|---|
| 1 | Pranesh M | [CB.SC.U4AI24345] | [Email] | Vision-Language Navigation |
| 2 | Sandhiya D | [CB.SC.U4AIE24353] |  | Simulation & Flight Stack |
| 3 | Jenishaa Bharathi M | [CB.SC.U4AIE24221] | [Email] | ROS 2 & System Integration |
| 4 | Naveen K | [CB.SC.U4AIE24235] | [Email] | GPS-Denied Navigation / SLAM |
| 5 | Pooja N | [ CB.SC.U4AIE24242] | [Email] | Semantic Memory & Evaluation |

---

# Abstract

This project develops a vision-language based autonomous UAV navigation system that enables a drone to interpret natural-language instructions, understand its visual environment, identify semantic targets, estimate its position, plan a feasible trajectory, and execute autonomous flight in a simulated environment.

The proposed system integrates **Vision-Language Models (VLMs), semantic object grounding, visual perception, localization/SLAM, trajectory planning, ROS 2, PX4, Gazebo, and QGroundControl** into a unified navigation pipeline.

For example, given an instruction such as:

> "Find the red building near the road and fly to it."

the system interprets the instruction, identifies the corresponding object from the UAV's camera observations, estimates the target location, generates a navigation trajectory, and sends appropriate flight setpoints to the PX4 flight controller.

The simulation environment uses an **X500 UAV in Gazebo with PX4 SITL**, while QGroundControl is used for flight monitoring and visualization.

The project is inspired by the **TravelUAV** vision-language navigation framework and focuses on developing a modular simulation architecture that can progressively support language understanding, visual grounding, GPS-denied localization, planning, and autonomous flight.

---

# Introduction

Unmanned Aerial Vehicles (UAVs) are increasingly used for surveillance, inspection, search and rescue, mapping, delivery, and autonomous exploration. Traditional autonomous UAV systems generally rely on predefined waypoints or manually specified coordinates. Such interfaces require the user to know the environment and provide precise numerical navigation goals.

Vision-language navigation introduces a more intuitive alternative: the UAV can receive instructions in natural language and use visual information from its environment to determine what the instruction refers to and where it should navigate.

For example, instead of specifying:

$$
(x,y,z)=(10,5,5)
$$

a user can provide:

> "Fly to the red building near the road."

The UAV must then solve several interconnected problems:

1. Understand the natural-language instruction.
2. Interpret the visual scene.
3. Ground the described object in the image.
4. Estimate the UAV's position.
5. Determine the target's location.
6. Plan a feasible route.
7. Generate flight setpoints.
8. Execute the trajectory through the flight controller.

Therefore, the project combines concepts from **artificial intelligence, computer vision, robotics, control systems, SLAM, and UAV simulation**.

---

# Problem Statement

The objective is to develop an autonomous UAV navigation pipeline capable of transforming a natural-language instruction into executable UAV motion.

The system can be represented as:

$$
\text{Language}
\rightarrow
\text{Vision-Language Understanding}
\rightarrow
\text{Target Grounding}
\rightarrow
\text{Localization}
\rightarrow
\text{Planning}
\rightarrow
\text{Flight Control}
$$

The main challenge is the integration of high-level semantic reasoning with low-level physical flight control.

The VLM should determine **what the user wants**, while the navigation and control systems determine **where and how the UAV should fly**.

---

# Objectives

The major objectives of the project are:

- Develop a natural-language interface for UAV navigation.
- Integrate vision-language reasoning into the UAV navigation pipeline.
- Ground language-described targets in camera observations.
- Estimate UAV position using available localization methods.
- Develop a navigation and waypoint-generation system.
- Integrate the navigation system with ROS 2.
- Interface the navigation system with PX4.
- Simulate autonomous UAV flight using Gazebo.
- Monitor the UAV using QGroundControl.
- Establish a framework for future GPS-denied navigation using SLAM.
- Evaluate navigation performance using measurable metrics.

---

# Base Paper

The primary research direction is inspired by:

**"Towards Realistic UAV Vision-Language Navigation: Platform, Benchmark, and Methodology"**

The paper introduces **TravelUAV**, a platform and benchmark for studying vision-language navigation for UAVs.

The project does not claim to reproduce the TravelUAV implementation exactly. Instead, it adapts the vision-language navigation problem to our available **PX4 + Gazebo + ROS 2** simulation environment.

### References

- TravelUAV paper: *Towards Realistic UAV Vision-Language Navigation: Platform, Benchmark, and Methodology*
- TravelUAV project: https://github.com/buaa-colalab/TravelUAV

---

# Proposed System Architecture

```mermaid
flowchart TD

A["Human Natural-Language Command"]
--> B["Vision-Language Model"]

B
--> C["Semantic Target / Intent"]

C
--> D["Object Grounding"]

E["UAV Camera"]
--> D

D
--> F["Target Location"]

E
--> G["Visual Perception"]

H["IMU / UAV State"]
--> I["Localization / SLAM"]

G
--> I

I
--> J["UAV Pose"]

F
--> K["Navigation Goal"]

J
--> K

K
--> L["Path / Trajectory Planner"]

L
--> M["Position / Velocity Setpoints"]

M
--> N["ROS 2 Integration"]

N
--> O["PX4 Offboard Control"]

O
--> P["Gazebo X500 UAV"]

P
--> E

P
--> Q["QGroundControl"]

Q
--> R["Telemetry / Monitoring"]
