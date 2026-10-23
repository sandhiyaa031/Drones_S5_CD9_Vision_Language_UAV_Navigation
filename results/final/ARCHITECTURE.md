# Architecture (as built, Phases 0–7)

## One rule above all

`flight_controller` is the **only** node that publishes to PX4
(`/fmu/in/*`). Perception, tracking, prediction, risk, avoidance planning
and survivor perception never command the vehicle. The avoidance planner
*proposes* a setpoint; the controller validates it and decides.

No node that runs in flight reads simulator ground truth. Ground truth is
recorded separately and used only by offline scoring scripts.

## System diagram

```
                              Gazebo Harmonic (collapsed_building_rescue world)
                 ┌───────────────────────────────────────────────────────────────┐
                 │  x500_rescue UAV   upward depth cam 640x480   downward RGB cam │
                 │                    100 deg, 30 Hz, 35 m       1280x960, 30 Hz  │
                 └───────┬────────────────────┬──────────────────────┬───────────┘
        lockstep sensors │                    │ ros_gz_bridge        │ ros_gz_bridge
                         ▼                    ▼                      ▼
                 ┌──────────────┐   /depth_up/image_raw        /camera/image_raw
                 │  PX4 SITL    │   /depth_up/camera_info      /camera/camera_info
                 │  EKF2, MC    │           │                        │
                 │  controllers │           ▼                        ▼
                 └──────┬───────┘   debris_depth_detector     survivor_perception ──HTTP──► local VLM
       Micro XRCE-DDS   │           (3D detections)           (candidates, tracks,          (Ollama,
        /fmu/out/*  ◄───┤                   │                  semantic state)               gemma3:4b)
        /fmu/in/*   ───►│                   ▼                        │
                        │           debris_tracker                   ▼
                        │           (Kalman + Hungarian)      /perception/survivor/detections
                        │                   │                 /perception/survivor/debug_image
                        │                   ▼
                        │           debris_predictor  (ballistic, 0.1 … 2.0 s)
                        │                   │
                        │                   ▼
                        │           debris_risk  (SAFE / WATCH / WARNING / CRITICAL)   advisory
                        │                   │
                        │                   ▼
                        │           debris_avoidance  (19 candidates, constraints, cost)
                        │                   │  /perception/debris/avoidance_command   (a proposal)
                        │                   ▼
                        └────────►  flight_controller  ◄── /uav/flight_status ──► planner
                                    state machine + flight_safety monitor
                                    validates the proposal, sole publisher to /fmu/in/*
```

Vehicle odometry (`/fmu/out/vehicle_odometry`, 100 Hz) feeds the detector,
the risk estimator and the planner.

## Nodes

| Node | Source (`src/uav_autonomy/uav_autonomy/`) | Phase | Role |
|---|---|---|---|
| `flight_controller` | `flight_controller.py`, `flight_safety.py`, `avoidance_interface.py` | 0, 6 | Offboard state machine: arm, take off, hold or fly waypoints, land. Safety monitor. Validates avoidance proposals and speed caps. |
| `debris_depth_detector` | `debris_depth_detector.py`, `debris_detection.py` | 2 | Depth image → 3D detections in the sensor frame and in PX4 local NED |
| `debris_tracker` | `debris_tracker.py`, `debris_tracking.py` | 3 | Kalman filter with gravity, Mahalanobis gating, Hungarian assignment |
| `debris_predictor` | `debris_predictor.py`, `debris_prediction.py` | 4 | Ballistic prediction at 7 horizons with covariance |
| `debris_risk` | `debris_risk_node.py`, `debris_risk.py` | 5 | Closest approach against the UAV, risk state per track |
| `debris_avoidance` | `debris_avoidance_node.py`, `debris_avoidance.py` | 6, 6.1 | Candidate setpoints, constraints, cost, avoidance state machine, perception health |
| `survivor_perception` | `survivor_perception_node.py`, `survivor_detection.py`, `survivor_confirmation.py`, `vlm_client.py` | 7 | Candidate detector, temporal confirmation, asynchronous VLM verification |

Each node is a thin ROS wrapper around a pure-Python module without ROS
imports; the unit tests exercise those modules directly.

Message package `src/uav_interfaces/msg/`: `DebrisTrack(Array)`,
`PredictedState`, `DebrisPrediction(Array)`, `DebrisRisk(Array)`,
`AvoidanceCommand`, `FlightStatus`, `SurvivorDetection(Array)`.

## Topics

| Topic | Type | From → to |
|---|---|---|
| `/fmu/out/vehicle_odometry`, `vehicle_local_position_v1`, `vehicle_status_v4` | `px4_msgs` | PX4 → controller, perception, planner |
| `/fmu/in/offboard_control_mode`, `trajectory_setpoint`, `vehicle_command` | `px4_msgs` | **controller only** → PX4 |
| `/depth_up/image_raw`, `/depth_up/camera_info` | `sensor_msgs/Image` (32FC1), `CameraInfo` | bridge → detector |
| `/camera/image_raw`, `/camera/camera_info` | `sensor_msgs/Image` (rgb8), `CameraInfo` | bridge → survivor perception |
| `/perception/debris/detections`, `detections_local` | `vision_msgs/Detection3DArray` | detector → tracker |
| `/perception/debris/tracks` | `uav_interfaces/DebrisTrackArray` | tracker → predictor, risk, planner |
| `/perception/debris/predictions` | `uav_interfaces/DebrisPredictionArray` | predictor → risk, planner |
| `/perception/debris/risk` | `uav_interfaces/DebrisRiskArray` | risk → planner |
| `/perception/debris/avoidance_command` | `uav_interfaces/AvoidanceCommand` | planner → controller (proposal) |
| `/uav/flight_status` | `uav_interfaces/FlightStatus` | controller → planner, risk |
| `/perception/survivor/detections` | `uav_interfaces/SurvivorDetectionArray` | survivor perception → (future phases) |
| `…/debug_image`, `…/track_markers`, `…/risk_markers`, `…/avoidance_markers` | `Image`, `MarkerArray` | visualisation |

## Flight-control architecture

```
WAIT_FOR_DATA → PRESTREAM → REQUEST_OFFBOARD → WAIT_PREFLIGHT → REQUEST_ARM → TAKEOFF
   → HOLD (timed)            or   WAYPOINT_1 → WAYPOINT_2 → RETURN_HOME
   → REQUEST_LAND → LANDING → COMPLETE
   any failure, stale telemetry, safety violation → FAILSAFE → PX4 AUTO LAND
```

* Position setpoints only (`OffboardControlMode.position = true`).
* During an accepted avoidance command the setpoint sent to PX4 is the
  avoidance target; the mission target is never modified, so the mission
  resumes by itself when the command ends.
* A command is accepted only in airborne mission states, if it is newer than
  0.5 s, finite, within 6 m of the vehicle and between 1.5 m and 15 m above
  the takeoff point.
* No PX4 source, safety parameter or preflight check is modified.

## Time

PX4 is started with `UXRCE_DDS_SYNCT=0` for each run (and its stored
parameters are restored afterwards), so PX4 timestamps equal Gazebo
simulation time exactly (measured error 0 ms). Sensor frames are paired with
odometry by `timestamp_sample`. This is a simulation time base, not a
hardware clock-synchronisation method.

## Not in the architecture yet

3D survivor localisation, a search planner, a mission state machine and a
natural-language mission interface (Phases 8–11) do not exist. The debris
chain and the survivor chain have been validated separately and have not
been flown together.
