# ROS 2 nodes, topics and messages

Every row below was extracted from the `create_publisher` / `create_subscription` calls in
`src/uav_autonomy/uav_autonomy/*_node.py` (and the node modules named below), not copied from older
documents. Debug/marker topics are listed once at the end. Executables are those registered in
`src/uav_autonomy/setup.py`; there is **no single launch file** for the integrated system — the shell scripts
(`scripts/run_flight_trial.sh`, `scripts/mission/run_mission.sh`, `scripts/mission/run_instruction.sh`,
`scripts/disaster/run_demo.sh`) start each node with `ros2 run uav_autonomy <executable>`.
(`launch/p2_perception.launch.py` belongs to an older image-plane debris prototype and is not used by the
integrated mission.)

## Vocabulary

| Term | Meaning here |
|---|---|
| **Node** | One ROS 2 process, e.g. `flight_controller`. |
| **Topic** | A named, typed, many-to-many stream, e.g. `/uav/flight_status`. Publishers and subscribers do not know each other. |
| **Message** | The data type carried by a topic, e.g. `uav_interfaces/msg/FlightStatus`. |
| **Service / action** | Request–reply / long-running goal with feedback. **This project defines none**: every interface below is a topic. |
| **PX4 uORB topic** | PX4's *internal* publish/subscribe bus. The Micro XRCE-DDS agent mirrors selected uORB topics to ROS 2 as `/fmu/out/*` (PX4 → ROS) and accepts `/fmu/in/*` (ROS → PX4). |

## Nodes

| Executable (`ros2 run uav_autonomy …`) | Logic module (no ROS) | Role |
|---|---|---|
| `flight_controller` | `flight_safety.py`, `avoidance_interface.py`, `search_interface.py`, `mission_interface.py` | Offboard state machine; **only** publisher on `/fmu/in/*` |
| `debris_depth_detector` | `debris_detection.py` | Upward depth image → 3-D debris detections |
| `debris_tracker` | `debris_tracking.py` | Kalman filter + Hungarian association |
| `debris_predictor` | `debris_prediction.py` | Ballistic prediction, 0.1–2.0 s |
| `debris_risk` | `debris_risk.py` | Closest approach, risk state |
| `debris_avoidance` | `debris_avoidance.py` | Candidate sidesteps → proposal |
| `survivor_perception` | `survivor_detection.py`, `survivor_confirmation.py`, `vlm_client.py` | Candidate detector, temporal confirmation, VLM |
| `survivor_localization` | `survivor_localization.py` | Pixel → 3-D point, EKF per candidate |
| `search_planner` | `search_planning.py` | Lawnmower coverage + inspection detours |
| `navigator` | `navigation.py` | A* on the SLAM map |
| `slam_bridge` | `slam_geometry.py` | Levels/filters lidar scans, TF `odom→base_level` |
| `mission_manager` | `mission_logic.py` | Preflight gate, START/RETURN_HOME, report |
| `mission_viz` | – | Republishes existing data as RViz paths/markers |
| `state_bridge` | – | PX4 odometry → `nav_msgs/Odometry` on `/odom` (older tool) |
| *(external)* `slam_toolbox` (`online_async_launch.py`) | – | 2-D pose-graph SLAM (ROS package, not project code) |
| *(external)* `ros_gz_bridge parameter_bridge` | – | Gazebo sensor topics → ROS 2 |
| *(external)* `MicroXRCEAgent udp4 -p 8888` | – | PX4 uORB ⇄ ROS 2 |

Not run in the integrated mission: `mission_language.py` (a library run on the ground by
`scripts/mission/interpret_mission.py`, not a node), `rescue_mission.py`, `debris_perception`,
`continuous_debris_spawner` (a test tool).

## Main interfaces

QoS for `/fmu/*`: best-effort, transient-local, keep-last 5 (see `flight_controller.py`).

### PX4 ⇄ ROS 2

| Topic | Publisher | Subscriber(s) | Type | Purpose |
|---|---|---|---|---|
| `/fmu/in/offboard_control_mode` | `flight_controller` | PX4 | `px4_msgs/OffboardControlMode` | Heartbeat declaring "position control" (`position=true`) |
| `/fmu/in/trajectory_setpoint` | `flight_controller` | PX4 | `px4_msgs/TrajectorySetpoint` | Position setpoint (NED), yaw = NaN (hold) |
| `/fmu/in/vehicle_command` | `flight_controller` | PX4 | `px4_msgs/VehicleCommand` | Mode switch, arm, `NAV_LAND` |
| `/fmu/out/vehicle_local_position_v1` | PX4 | `flight_controller`, `debris_risk`, `debris_avoidance`, `mission_manager` | `px4_msgs/VehicleLocalPosition` | Position/velocity/acceleration estimate (EKF2) |
| `/fmu/out/vehicle_odometry` | PX4 | detector, risk, avoidance, `survivor_localization`, `search_planner`, `navigator`, `slam_bridge`, `state_bridge` | `px4_msgs/VehicleOdometry` | Pose + attitude quaternion, 100 Hz |
| `/fmu/out/vehicle_status_v4` | PX4 | `flight_controller` | `px4_msgs/VehicleStatus` | Arming, nav state, preflight flag |
| `/fmu/out/vehicle_command_ack_v1` | PX4 | `flight_controller` | `px4_msgs/VehicleCommandAck` | Command accepted/denied |
| `/fmu/out/vehicle_land_detected` | PX4 | `flight_controller` | `px4_msgs/VehicleLandDetected` | Landed flag |
| `/fmu/out/battery_status_v1` (or `battery_status`) | PX4 | `mission_manager` | `px4_msgs/BatteryStatus` | Battery fraction (simulated battery) |

### Sensors (Gazebo → ROS 2 via `ros_gz_bridge`; configs in `src/uav_autonomy/config/`)

| Topic | Subscriber(s) | Type | Purpose |
|---|---|---|---|
| `/camera/image_raw`, `/camera/camera_info` | `survivor_perception`, `survivor_localization`, `search_planner` (info) | `sensor_msgs/Image`, `CameraInfo` | Downward RGB camera, 1280×960 |
| `/depth_up/image_raw`, `/depth_up/camera_info` | `debris_depth_detector`, `debris_tracker` (info) | `sensor_msgs/Image` (32FC1, m), `CameraInfo` | Upward depth camera, 640×480 |
| `/uav/scan_raw` | `slam_bridge` | `sensor_msgs/LaserScan` | Planar lidar, 720 rays, 10 Hz (village/urban worlds) |

### Perception and mapping

| Topic | Publisher | Subscriber(s) | Type | Purpose |
|---|---|---|---|---|
| `/perception/survivor/detections` | `survivor_perception` | `survivor_localization`, `mission_manager` | `uav_interfaces/SurvivorDetectionArray` | Image-plane candidates + semantic state |
| `/perception/survivor/locations` | `survivor_localization` | `search_planner`, `mission_manager`, `mission_viz` | `uav_interfaces/SurvivorLocationArray` | 3-D position, covariance, VLM flags (frame `px4_local_ned`) |
| `/perception/debris/detections`, `…/detections_local` | `debris_depth_detector` | `debris_tracker` | `vision_msgs/Detection3DArray` | Detections in sensor / PX4 local frame |
| `/perception/debris/tracks` | `debris_tracker` | `debris_predictor`, `debris_risk`, `debris_avoidance` | `uav_interfaces/DebrisTrackArray` | Persistent identity, position, velocity |
| `/perception/debris/predictions` | `debris_predictor` | `debris_risk`, `debris_avoidance` | `uav_interfaces/DebrisPredictionArray` | Predicted states at 7 horizons |
| `/perception/debris/risk` | `debris_risk` | `debris_avoidance` | `uav_interfaces/DebrisRiskArray` | SAFE/WATCH/WARNING/CRITICAL per track |
| `/uav/scan` | `slam_bridge` | `slam_toolbox`, `navigator` | `sensor_msgs/LaserScan` | Levelled, filtered scan |
| `/map`, `/pose`, TF `map→odom` | `slam_toolbox` | `navigator` | `nav_msgs/OccupancyGrid`, `PoseWithCovarianceStamped`, `tf2_msgs/TFMessage` | SLAM map and pose |

### Planning, mission and flight status (all high-level; only proposals)

| Topic | Publisher | Subscriber(s) | Type | Purpose |
|---|---|---|---|---|
| `/planning/search/goal_raw` | `search_planner` (its `/planning/search/goal` output is remapped to `goal_raw` by `scripts/mission/run_mission.sh` line 65 for the village/urban worlds) | `navigator` | `uav_interfaces/SearchGoal` | Raw coverage goal |
| `/planning/search/goal` | `navigator` (village/urban) **or** `search_planner` (small world) | `flight_controller`, `mission_manager` | `uav_interfaces/SearchGoal` | Validated-by-controller horizontal goal + speed cap |
| `/planning/navigator/status` | `navigator` | `search_planner`, `mission_viz` | `std_msgs/String` | `UNREACHABLE`, `LOCALIZATION_LOST`, … |
| `/perception/debris/avoidance_command` | `debris_avoidance` | `flight_controller`, `mission_manager` | `uav_interfaces/AvoidanceCommand` | Avoidance setpoint **proposal** (0.5 s validity) |
| `/mission/command` | `mission_manager` | `flight_controller`, `navigator` | `uav_interfaces/MissionCommand` | `START`, `RETURN_HOME`, `LAND_NOW` |
| `/mission/status` (2 Hz), `/mission/report` | `mission_manager` | `mission_viz`; operator | `uav_interfaces/MissionStatus`, `std_msgs/String` (JSON) | Phase, final report |
| `/uav/flight_status` | `flight_controller` | `search_planner`, `mission_manager`, `navigator`, `debris_risk`, `debris_avoidance` | `uav_interfaces/FlightStatus` | Controller state, active setpoint, avoidance counters |

### Debug / visualisation (not part of the control path)

`/perception/survivor/{debug_image,status,localization_status,location_markers}`,
`/perception/debris/{debug_image,status,track_markers,tracks_debug,prediction_markers,predictions_debug,risk_markers,risk_debug,avoidance_markers,avoidance_debug}`,
`/planning/search/{status,markers}`, `/planning/navigator/path`, `/slam/{odometry_pose,bridge_status}`,
`/viz/{flown_path,slam_path,mission}`, `/odom`.
`survivor_perception` and `search_planner` also exchange `std_msgs/String` status
(`/perception/survivor/status`, `/planning/search/status`).

## Custom messages (`src/uav_interfaces/msg/`)

`AvoidanceCommand`, `DebrisPrediction(Array)`, `DebrisRisk(Array)`, `DebrisTrack(Array)`, `FlightStatus`,
`MissionCommand`, `MissionStatus`, `PredictedState`, `SearchGoal`, `SurvivorDetection(Array)`,
`SurvivorLocation(Array)`. Field-level comments are in the `.msg` files.

## Checking a live system

```bash
ros2 topic hz /fmu/out/vehicle_odometry
ros2 topic echo /uav/flight_status --field state
ros2 topic echo /perception/survivor/locations
ros2 topic echo /mission/report --once
ros2 node list
```
