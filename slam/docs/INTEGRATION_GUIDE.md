# GPS-Denied Navigation Integration Guide (Cross-Member Interface)

## 1. System Integration Architecture

```mermaid
graph LR
    subgraph Member 4 (GPS-Denied Navigation)
        VO["Visual Odometry Node"]
        EKF["15-State EKF Fusion"]
        TF["TF2 Broadcaster"]
        MB["MAVROS Vision Bridge"]
        OG["Occupancy Grid Node"]
    end

    subgraph Member 1 (VLM Navigation)
        VLM["TravelUAV / VLM Goal Planner"]
    end

    subgraph Member 2 (Simulation & Flight)
        SITL["ArduPilot / Gazebo SITL"]
    end

    subgraph Member 3 (System Integration)
        MAVROS["MAVROS Node"]
    end

    subgraph Member 5 (Semantic Memory)
        MEM["Adaptive Memory DB"]
    end

    SITL -->|/camera/image_raw, /imu/data| VO
    SITL -->|/imu/data, /rangefinder/range| EKF
    VO -->|/vo/odom| EKF
    VO -->|/vo/sparse_map| OG
    EKF -->|/odom/filtered| MB
    EKF -->|/odom/filtered| TF
    MB -->|/mavros/vision_pose/pose| MAVROS
    MAVROS -->|MAVLink VISION_POSITION_ESTIMATE| SITL
    EKF -->|/odom/filtered| VLM
    OG -->|/map, /costmap/costmap| VLM
    EKF -->|/odom/filtered (6-DoF Keyframe Poses)| MEM
```

---

## 2. Topic & Interface Contracts

### 2.1 Interface with Member 1 (Vision-Language Navigation Engineer)
| Topic Name | Message Type | Rate | Description |
| :--- | :--- | :--- | :--- |
| `/odom/filtered` | `nav_msgs/Odometry` | 50 Hz | Current estimated UAV 6-DoF pose and velocities in ENU world frame |
| `/map` | `nav_msgs/OccupancyGrid` | 1 Hz | 2D global obstacle grid map for path planning and waypoint validation |
| `/costmap/costmap` | `nav_msgs/OccupancyGrid` | 5 Hz | Local rolling obstacle costmap for obstacle avoidance during flight |
| `/vo/path` | `nav_msgs/Path` | 10 Hz | Historical flight path for trajectory grounding |

### 2.2 Interface with Member 2 (Simulation & Flight Stack Engineer)
| Required Input Topic | Message Type | Target Rate | Source |
| :--- | :--- | :--- | :--- |
| `/camera/image_raw` | `sensor_msgs/Image` | 30 Hz | Gazebo Harmonic forward/downward camera |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Latched | Camera calibration matrix $K$ and distortion $D$ |
| `/imu/data` | `sensor_msgs/Imu` | 100 Hz | High-rate IMU linear acceleration & angular velocity |
| `/rangefinder/range` | `sensor_msgs/Range` | 20 Hz | Downward sonar/LiDAR distance to ground |

### 2.3 Interface with Member 3 (ROS 2 & System Integration Engineer)
| Topic Name | Message Type | Rate | Target |
| :--- | :--- | :--- | :--- |
| `/mavros/vision_pose/pose` | `geometry_msgs/PoseStamped` | 30 Hz | Feeds external vision pose to ArduPilot / PX4 EKF2 in NED frame |
| `/mavros/odometry/out` | `nav_msgs/Odometry` | 30 Hz | High-accuracy vision odometry with covariance |
| `/tf` / `/tf_static` | `tf2_msgs/TFMessage` | Dynamic | Complete coordinate tree (`map` $\rightarrow$ `odom` $\rightarrow$ `base_link` $\rightarrow$ sensors) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 2 Hz | Visual tracking state, EKF health, and error covariance |

### 2.4 Interface with Member 5 (Adaptive Semantic Memory & Evaluation Engineer)
| Topic Name | Message Type | Rate | Purpose |
| :--- | :--- | :--- | :--- |
| `/odom/filtered` | `nav_msgs/Odometry` | On Keyframe | Timestamped 6-DoF anchor poses for storing landmark embeddings |
| `/vo/sparse_map` | `sensor_msgs/PointCloud2` | 2 Hz | 3D sparse landmark points to associate with semantic objects in SQLite |

---

## 3. ArduPilot / PX4 Parameter Configuration (For Member 2 & 3)

To enable external vision navigation in ArduPilot SITL / PX4 without GPS:

### ArduPilot Parameters:
```ini
AHRS_EKF_TYPE = 3           # Use EKF3
EK3_ENABLE = 1              # Enable EKF3
EK3_SRC1_POSXY = 6          # Source 1 XY Position = ExternalNav
EK3_SRC1_VELXY = 6          # Source 1 XY Velocity = ExternalNav
EK3_SRC1_POSZ = 6           # Source 1 Z Position = ExternalNav (or 1 for Rangefinder)
EK3_SRC1_YAW = 6            # Source 1 Yaw = ExternalNav
VISO_TYPE = 1               # MAVLink Vision Position Estimate
ARMING_CHECK = -9           # Disable GPS arming check for indoor testing
```

### PX4 Parameters:
```ini
EKF2_AID_MASK = 24          # Vision position & yaw fusion (bits 3 & 4)
EKF2_HGT_MODE = 3           # Vision height (or 2 for Range sensor)
EKF2_EV_DELAY = 10          # Delay compensation in ms
```
