# ORB-SLAM3 and RTAB-Map ROS 2 Integration Guide

## 1. RTAB-Map (Real-Time Appearance-Based Mapping) in ROS 2

RTAB-Map provides robust 3D/2D visual SLAM with loop closure detection, memory management, and OctoMap generation.

### 1.1 Installation (Ubuntu / ROS 2 Jazzy & Humble)
```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-rtabmap-ros ros-${ROS_DISTRO}-rtabmap-viz
```

### 1.2 Launching with Member 4 Stack
```bash
# Launch Member 4 VO & EKF + RTAB-Map
ros2 launch gps_denied_nav rtabmap_slam.launch.py \
    rgb_topic:=/camera/image_raw \
    depth_topic:=/camera/depth/image_raw \
    camera_info_topic:=/camera/camera_info \
    odom_topic:=/odom/filtered
```

### 1.3 Exporting 3D Point Cloud & 2D Grid
RTAB-Map publishes:
- `/map`: 2D Occupancy Grid (`nav_msgs/OccupancyGrid`)
- `/rtabmap/cloud_map`: Dense 3D Point Cloud (`sensor_msgs/PointCloud2`)
- `/rtabmap/octomap_grid`: OctoMap 3D grid representation

---

## 2. ORB-SLAM3 ROS 2 Integration

ORB-SLAM3 is an industry standard for Monocular, Stereo, and Visual-Inertial SLAM with bundle adjustment.

### 2.1 Building ORB-SLAM3 with ROS 2 Wrapper
```bash
# 1. Clone ORB-SLAM3
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
cd ORB_SLAM3
chmod +x build.sh
./build.sh

# 2. Build ROS 2 Wrapper
chmod +x build_ros2.sh
./build_ros2.sh
```

### 2.2 Running Monocular-Inertial Mode
```bash
ros2 run orb_slam3_ros2 mono_inertial \
    Vocabulary/ORBvoc.txt \
    $(ros2 pkg prefix gps_denied_nav)/share/gps_denied_nav/config/orb_slam3_mono_inertial.yaml \
    /camera/image_raw:=/camera/image_raw \
    /imu:=/imu/data
```

### 2.3 Key Features Supported:
1. **Multi-Map Atlas System**: Creates new submaps during visual tracking failure and automatically merges them upon loop closure.
2. **Inertial Initialization**: Fast (< 2 seconds) gravity direction and IMU gyro/accel bias convergence.
3. **Low Latency**: Keyframe tracking runs at camera rate (30+ FPS) on standard CPU.
