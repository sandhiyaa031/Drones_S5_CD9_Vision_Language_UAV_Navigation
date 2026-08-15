# Adaptive Semantic Memory for Vision-Language Autonomous UAV Navigation in GPS-Denied Environments

<div align="center">
</div>

<div align="center">

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

| Name | Roll Number |
|---|---|
| PRANESH M | CB.SC.U4AIE24345 |
| SANDHIYA D | CB.SC.U4AIE24353 |
| JENISHAA BHARATHI M | CB.SC.U4AIE24221 |
| POOJA N | CB.SC.U4AIE24242 |
| NAVEEN K | CB.SC.U4AIE24235 |

---

# Table of Contents

1. [Introduction & Motivation](#1-introduction--motivation)
2. [Problem Statement](#2-problem-statement)
3. [Research Gap](#3-research-gap)
4. [Project Objectives](#4-project-objectives)
5. [Core Research Question](#5-core-research-question)
6. [System Overview](#6-system-overview)
7. [Quadrotor Fundamentals](#7-quadrotor-fundamentals)
8. [Coordinate Frames & State Representation](#8-coordinate-frames--state-representation)
9. [Quaternion-Based Attitude Representation](#9-quaternion-based-attitude-representation)
10. [Quadrotor Dynamics](#10-quadrotor-dynamics)
11. [GPS-Denied Navigation](#11-gps-denied-navigation)
12. [Visual Odometry & SLAM](#12-visual-odometry--slam)
13. [State Estimation](#13-state-estimation)
14. [Semantic Mapping](#14-semantic-mapping)
15. [Vision-Language Navigation](#15-vision-language-navigation)
16. [Adaptive Semantic Memory](#16-adaptive-semantic-memory)
17. [Proposed Architecture](#17-proposed-architecture)
18. [Simulation Environment](#18-simulation-environment)
19. [Software Stack](#19-software-stack)
20. [Hardware Architecture](#20-hardware-architecture)
21. [Raspberry Pi Deployment](#21-raspberry-pi-deployment)
22. [Simulation-to-Real Transfer](#22-simulation-to-real-transfer)
23. [5–7 Month Project Roadmap](#23-57-month-project-roadmap)
24. [Experimental Methodology](#24-experimental-methodology)
25. [Baselines](#25-baselines)
26. [Evaluation Metrics](#26-evaluation-metrics)
27. [Ablation Studies](#27-ablation-studies)
28. [Expected Results](#28-expected-results)
29. [Real-World Demonstration](#29-real-world-demonstration)
30. [Limitations](#30-limitations)
31. [Future Work](#31-future-work)
32. [Repository Structure](#32-repository-structure)
33. [Execution & Platform Information](#33-execution--platform-information)
34. [Reproducibility](#34-reproducibility)
35. [References](#35-references)
36. [Conclusion](#36-conclusion)

---

# 1. Introduction & Motivation

Autonomous drones have progressed from remotely controlled aerial platforms to increasingly capable robotic systems that can perceive, localize, plan, and act in complex environments.

However, many autonomous UAV systems still rely heavily on:

* GPS or external localization infrastructure
* predefined waypoints
* geometric maps
* fixed object classes
* short-horizon planning
* stateless perception

These assumptions break down in environments such as:

* disaster zones
* collapsed buildings
* warehouses
* underground environments
* tunnels
* mines
* forests
* large indoor facilities

A human search-and-rescue operator does not navigate such environments using latitude and longitude alone.

Humans combine:

**visual perception + spatial understanding + memory + language + reasoning + action.**

This project investigates whether a UAV can be given a similar high-level capability.

The proposed system combines:

```text
GPS-Denied Localization
        +
Semantic Mapping
        +
Vision-Language Understanding
        +
Adaptive Semantic Memory
        +
Autonomous Planning
        +
Low-Level Flight Control
