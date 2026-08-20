# Vision-Language UAV Navigation for GPS-Denied Environments

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

## Team Members

| Name | Roll Number | Email |
|---|---|---|
| JENISHAA BHARATHI M | CB.SC.U4AIE24221 | cb.sc.u4aie24221@cb.students.amrita.edu |
| NAVEEN K | CB.SC.U4AIE24235 | cb.sc.u4aie24235@cb.students.amrita.edu |
| POOJA N | CB.SC.U4AIE24242 | cb.sc.u4aie24242@cb.students.amrita.edu |
| PRANESH M | CB.SC.U4AIE24345 | cb.sc.u4aie24345@cb.students.amrita.edu |
| SANDHIYA D | CB.SC.U4AIE24353 | cb.sc.u4aie24353@cb.students.amrita.edu |

---

# Abstract

This project develops a **Vision-Language UAV Navigation system** for autonomous navigation in GPS-denied environments. The system converts natural-language instructions into executable UAV navigation goals by combining vision-language reasoning, semantic grounding, localization/SLAM, path planning, ROS 2, PX4 and Gazebo.

A typical instruction such as **"Find the red building near the road and fly to it"** is interpreted, grounded in the UAV's visual observations, converted into a spatial navigation goal, planned as a trajectory and executed by a simulated PX4-controlled UAV.

The system is developed and evaluated using software-in-the-loop simulation with **PX4, Gazebo and QGroundControl**.

---

# 1. Introduction

Conventional UAV navigation usually requires GPS coordinates or predefined waypoints:

```text
Takeoff
   ↓
Waypoint 1
   ↓
Waypoint 2
   ↓
Land
