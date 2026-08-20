# GPS-Denied Autonomous UAV Navigation Stack (Member 4)

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy%20%7C%20Humble%20%7C%20Iron-blue.svg)](https://docs.ros.org/)
[![Python](https://img.shields.io/badge/Python-3.10%2B-brightgreen.svg)](https://www.python.org/)
[![OpenCV](https://img.shields.io/badge/OpenCV-4.x-orange.svg)](https://opencv.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

Autonomous, real-time 6-DoF localization, visual-inertial state estimation, mapping, and flight controller bridging in **GPS-denied environments** (indoor rooms, underground tunnels, urban canyons, dense forests).

---

## 🎯 Member 4 Role & Responsibilities

| Role | Primary Responsibilities | Software / Tools | Deliverables |
| :--- | :--- | :--- | :--- |
| **GPS-Denied Navigation Engineer** (Member 4) | Visual SLAM, Localization, Mapping, Coordinate Frames, Pose Estimation, Multi-Sensor Fusion | ORB-SLAM3, RTAB-Map, OpenCV, ROS 2, EKF | Real-time localization & 2D/3D map generation without GPS, MAVROS Vision Pose Bridge |

---

## 🚀 Key Features

1. **High-Speed Visual Odometry (`gps_denied_nav/vo/`)**:
   - Pyramidal Lucas-Kanade (KLT) optical flow with forward-backward bidirectional outlier rejection.
   - FAST / ORB / Shi-Tomasi feature detectors with **Spatial Grid Bucketing** to prevent feature clustering.
   - 5-point Essential Matrix estimation with RANSAC (`cv2.findEssentialMat`, `cv2.recoverPose`).
   - Landmark 3D triangulation and metric scale recovery using altitude/rangefinder constraints.

2. **15-State Error-State Extended Kalman Filter (`gps_denied_nav/fusion/`)**:
   - Continuous IMU mechanization predicting at 100 Hz with gravity compensation.
   - States: Position (3), Velocity (3), Orientation Quaternion (4), Accel Biases (3), Gyro Biases (3).
   - Asynchronous measurement updates fusing Visual Odometry (30 Hz) + Rangefinder/Barometer (20 Hz).
   - Low-latency, jitter-free 50 Hz `/odom/filtered` output with Joseph-form covariance stability.

3. **Coordinate Transformation & MAVROS Bridge (`gps_denied_nav/bridge/`)**:
   - Complete standard ROS REP-103 / REP-105 TF2 tree (`map` $\rightarrow$ `odom` $\rightarrow$ `base_link` $\rightarrow$ `camera_link` $\rightarrow$ `camera_optical_frame`).
   - Dynamic transformation from ROS ENU (East-North-Up) to Flight Controller NED (North-East-Down).
   - Feeds `/mavros/vision_pose/pose` and `/mavros/odometry/out` with validated covariance matrices for ArduPilot / PX4 EKF2.

4. **Occupancy Grid Mapping & Point Cloud Filtering (`gps_denied_nav/mapping/`)**:
   - Converts 3D sparse/dense visual point clouds into 2D `nav_msgs/OccupancyGrid` on `/map` and `/costmap/costmap`.
   - Voxel grid downsampling, Statistical Outlier Removal (SOR), and RANSAC ground plane segmentation.

5. **Built-in Synthetic Simulator & Benchmark Evaluator (`gps_denied_nav/eval/`)**:
   - Generates synthetic 3D indoor GPS-denied environments, camera feeds, and noisy IMU streams.
   - Evaluates Absolute Trajectory Error (ATE RMSE) and Relative Pose Error (RPE) against Ground Truth.

---

## 📂 Repository Structure

```
drones/
├── package.xml                        # ROS 2 package manifest
├── setup.py / setup.cfg               # Python package setup
├── CMakeLists.txt                     # Dual build configuration
├── ARCHITECTURE.md                    # Detailed architecture & latency budgets
├── README.md                          # Main project guide
├── config/
│   ├── camera_calib.yaml              # Camera intrinsics & distortion model
│   ├── imu_noise_params.yaml          # IMU noise densities & random walk
│   ├── ekf_params.yaml                # 15-state EKF process/measurement noise
│   ├── rtabmap_params.yaml            # RTAB-Map 3D visual SLAM configuration
│   ├── orb_slam3_mono_inertial.yaml   # ORB-SLAM3 configuration file
│   └── nav_view.rviz                  # RViz2 multi-display layout
├── gps_denied_nav/
│   ├── vo/                            # Visual Odometry & Feature Tracking
│   │   ├── feature_tracker.py
│   │   ├── visual_odometer.py
│   │   └── vo_node.py
│   ├── fusion/                        # 15-State Multi-Sensor EKF
│   │   ├── ekf_core.py
│   │   └── ekf_fusion_node.py
│   ├── bridge/                        # TF2 Tree & MAVROS Bridge
│   │   ├── tf_broadcaster.py
│   │   └── mavros_vision_bridge.py
│   ├── mapping/                       # Occupancy Grid & Point Cloud Processor
│   │   ├── pointcloud_processor.py
│   │   └── occupancy_grid_node.py
│   └── eval/                          # Diagnostics, Simulator & Benchmarks
│       ├── synthetic_data_generator.py
│       ├── trajectory_evaluator.py
│       └── diagnostics_node.py
├── launch/
│   ├── gps_denied_master.launch.py    # Master launch file
│   ├── vo_fusion.launch.py            # Lightweight VO + EKF launch
│   ├── rtabmap_slam.launch.py         # RTAB-Map 3D SLAM launch
│   └── standalone_demo.launch.py      # Standalone synthetic simulation launch
├── scripts/
│   ├── run_synthetic_demo.py          # Interactive standalone demo & live HUD
│   ├── calibrate_camera.py            # OpenCV checkerboard calibration tool
│   └── evaluate_trajectory.py         # TUM/EuRoC trajectory benchmark script
└── docs/
    ├── COORDINATE_FRAMES.md           # REP-103/105, ENU vs NED, Optical frames
    ├── MATHEMATICAL_FORMULATION.md    # Epipolar math, EKF derivation, Quaternions
    ├── INTEGRATION_GUIDE.md           # Interface contracts with Members 1, 2, 3, 5
    └── ORB_SLAM3_RTABMAP_GUIDE.md     # Setup guide for ORB-SLAM3 and RTAB-Map
```

---

## ⚡ Quick Start

### Option 1: Run Standalone Demo (No ROS 2 installation required)
You can test and verify the entire localization, sensor fusion, mapping, and benchmark pipeline immediately on Windows or Linux:

```bash
python scripts/run_synthetic_demo.py
```
This will:
- Simulate a 3D UAV figure-8 flight in a GPS-denied room.
- Run KLT/FAST visual tracking and 15-state EKF sensor fusion in real-time.
- Build a real-time 2D obstacle occupancy grid.
- Compute ATE RMSE, RPE, and drift rate %.
- Save live dashboard snapshots and publication-grade evaluation plots (`evaluation_benchmark_plot.png`).

---

### Option 2: Run in ROS 2 (Jazzy / Humble)

```bash
# 1. Source ROS 2 environment
source /opt/ros/$ROS_DISTRO/setup.bash

# 2. Build the workspace
colcon build --packages-select gps_denied_nav --symlink-install
source install/setup.bash

# 3. Launch Master Navigation Stack
ros2 launch gps_denied_nav gps_denied_master.launch.py \
    camera_topic:=/camera/image_raw \
    imu_topic:=/imu/data \
    rangefinder_topic:=/rangefinder/range

# 4. View in RViz2
rviz2 -d $(ros2 pkg prefix gps_denied_nav)/share/gps_denied_nav/config/nav_view.rviz
```

---

## 📡 ROS 2 Published Topics

| Topic | Message Type | Rate | Description |
| :--- | :--- | :--- | :--- |
| `/odom/filtered` | `nav_msgs/Odometry` | 50 Hz | High-rate filtered 6-DoF pose & velocity in ENU |
| `/odom/filtered_path` | `nav_msgs/Path` | 10 Hz | Filtered drone trajectory |
| `/vo/odom` | `nav_msgs/Odometry` | 30 Hz | Raw Visual Odometry estimate |
| `/vo/debug_image` | `sensor_msgs/Image` | 30 Hz | Camera feed with tracked features & status HUD |
| `/vo/sparse_map` | `sensor_msgs/PointCloud2` | 5 Hz | Triangulated 3D visual landmarks |
| `/mavros/vision_pose/pose` | `geometry_msgs/PoseStamped` | 30 Hz | Vision pose transformed to **NED** for ArduPilot/PX4 |
| `/map` | `nav_msgs/OccupancyGrid` | 1 Hz | 2D global obstacle grid map |
| `/costmap/costmap` | `nav_msgs/OccupancyGrid` | 5 Hz | Local rolling obstacle costmap |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 2 Hz | SLAM tracking health & covariance status |

---

## 🤝 Cross-Member Interface Summary

- **Member 1 (VLM Navigation)**: Subscribes to `/odom/filtered` and `/costmap/costmap` to plan collision-free waypoint goals from visual-language commands.
- **Member 2 (Simulation & Flight Stack)**: Provides `/camera/image_raw`, `/imu/data`, and receives `/mavros/vision_pose/pose` to close the control loop in ArduPilot/PX4 EKF2.
- **Member 3 (System Integration)**: Orchestrates system launch files, monitors `/diagnostics`, and manages MAVROS topic routing.
- **Member 5 (Semantic Memory)**: Uses `/odom/filtered` keyframe poses as spatial anchors for memory storage and retrieval.

---

## 📄 Documentation Links
- [Coordinate Frames & Transformations](docs/COORDINATE_FRAMES.md)
- [Mathematical Formulation & Derivations](docs/MATHEMATICAL_FORMULATION.md)
- [Team Integration Guide](docs/INTEGRATION_GUIDE.md)
- [ORB-SLAM3 & RTAB-Map Setup](docs/ORB_SLAM3_RTABMAP_GUIDE.md)
- [System Architecture](ARCHITECTURE.md)
