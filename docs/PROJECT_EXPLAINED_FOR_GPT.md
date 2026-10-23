# Autonomous UAV disaster-rescue project: complete explanation

(Written to be pasted into another AI assistant with the request: "Explain this project to me step by step, in simple language, and quiz me on it." Everything here describes what actually exists and was measured; limits are stated plainly. All of it runs in SIMULATION only. Nothing has flown on a real drone.)

## 1. What the project is

A semester-5 engineering project: a quadrotor drone that, in a simulated earthquake-collapse site, can
1. take a human instruction in plain English ("Search the collapsed apartment block for survivors, avoid falling debris, report survivor locations, and return to base"),
2. turn it into a validated mission,
3. take off and fly a search pattern over the named area,
4. detect survivors (a mannequin in orange clothes) in its downward camera,
5. estimate each survivor's 3D position and report coordinates,
6. detect and dodge falling debris with an upward depth camera,
7. build a map with SLAM and plan routes around walls on that map,
8. return to base and land, with safety monitors and failsafes throughout.

It runs in two separate simulators: **Gazebo + PX4** (the main, most complete system) and **MuJoCo** (a second, simpler, independent simulator).

## 2. Tools and vocabulary

- **PX4**: open-source autopilot firmware. It runs the low-level flight control (stabilises the drone, holds position, lands). Here it runs as "SITL" (software in the loop): the real firmware compiled for a PC, fed by a simulator instead of real sensors. Unmodified.
- **Gazebo Harmonic**: a physics + rendering simulator. Provides the world, the drone body, cameras, lidar.
- **ROS 2 Jazzy**: a message-passing framework. Programs ("nodes") exchange messages on named "topics". Each project function is a node.
- **Micro XRCE-DDS agent**: the bridge that makes PX4's internal messages (uORB) appear as ROS 2 topics (`/fmu/out/...` from PX4, `/fmu/in/...` to PX4).
- **Offboard mode**: PX4 follows position setpoints streamed by an external program (our flight controller).
- **EKF2**: PX4's state estimator; fuses IMU, barometer, magnetometer and GNSS into position/velocity/attitude.
- **SLAM**: Simultaneous Localization And Mapping: building a map from a range sensor while estimating where you are in it.
- **VLM**: vision-language model (gemma3:4b via Ollama): looks at a cropped image and answers questions, used to confirm "is this a person?". Needs the NVIDIA GPU.
- **Frames**: Gazebo world = ENU (x east, y north, z up). PX4 local = NED (north, east, down) with origin at takeoff; for this project the base pad is at the world origin so NED = (world y, world x, -world z). The SLAM "map" frame is its own frame (rotated a few degrees relative to PX4's, because PX4's heading is still settling when SLAM starts).
- **Lockstep**: PX4 and the simulator advance time together step by step.
- **RTF (real-time factor)**: simulated time per real second; about 0.5-1.0 here.

## 3. The one rule that shapes the design

**Only `flight_controller.py` ever sends commands to PX4** (`/fmu/in/*`). Every other node only reads sensors or publishes *proposals* (a goal, an avoidance sidestep). The flight controller validates every proposal (range limits, age, altitude, speed) before acting. This keeps one place responsible for safety. Also: simulator ground truth (exact positions of objects) is never used by the online system; it is read only by offline scoring scripts.

## 4. System architecture (data flow)

```
Human sentence
  -> mission_language (interpreter) -> validated mission file (area, altitude, time budget, rules)
  -> mission_manager (preflight checks, START / RETURN_HOME commands, final report)
       -> flight_controller (state machine; the ONLY PX4 commander)  <->  PX4 SITL  <->  Gazebo (physics, sensors)

Sensors (Gazebo -> ROS topics via ros_gz_bridge):
  downward RGB camera  -> survivor_perception -> survivor_localization (EKF) -> search_planner, mission_manager
  upward depth camera  -> debris_depth_detector -> debris_tracker -> debris_predictor -> debris_risk -> debris_avoidance -> flight_controller (proposal)
  planar lidar         -> slam_bridge -> slam_toolbox -> /map + TF map->odom + /pose -> navigator
  search_planner goals -> navigator (A* on the SLAM map) -> flight_controller
```

## 5. Component by component

### 5.1 flight_controller (`uav_autonomy/flight_controller.py`)
A state machine: WAIT_FOR_DATA -> PRESTREAM (stream hold setpoints so PX4 accepts offboard) -> REQUEST_OFFBOARD -> WAIT_PREFLIGHT -> REQUEST_ARM -> TAKEOFF -> (HOLD | WAYPOINTS | SEARCH) -> RETURN_HOME -> REQUEST_LAND -> LANDING -> COMPLETE, with FAILSAFE reachable from almost anywhere. In SEARCH it follows goals from the planner/navigator. It checks telemetry freshness (position must be <1 s old; vehicle-status has a separate configurable limit, 3 s in the urban world because PX4 publishes status at only ~2 Hz of simulated time), arming state, mode, altitude excursions and an **impact detector** (acceleration > 30 m/s^2 -> FAILSAFE, AUTO LAND). It applies speed caps and the avoidance override (when a valid avoidance proposal arrives it temporarily replaces the mission target, then recovers). Parameters added later: `return_altitude` (optional climb-first return), `map_return` (a mission RETURN request keeps it in SEARCH while the navigator routes home on the map), `status_timeout_s`, `arrival_max_speed` (a waypoint/home counts as reached only when slow enough).
Safety logic is in `flight_safety.py` (pure Python, unit tested).

### 5.2 Debris chain (falling objects), Phases 2-6
Upward-looking depth camera sees boxes falling from above.
1. `debris_depth_detector`: finds moving objects in the depth image, converts to 3D points.
2. `debris_tracker`: Kalman filter with gravity, associates detections over time (Hungarian algorithm), gives position+velocity per object.
3. `debris_predictor`: ballistic prediction (position under gravity) a couple of seconds ahead.
4. `debris_risk`: decides if a predicted path comes within a safety radius of the drone (levels: none, caution, critical), with a lead time.
5. `debris_avoidance`: picks a sidestep candidate (e.g. "south_east_4" = 4 m SE) that keeps the safe distance; publishes it as a proposal. Not map-aware.
Measured: needs about 0.9 s of warning; direct hits on a hovering drone avoided 15/15; in the urban world 5 manoeuvres for 6 boxes, 0 collisions.

### 5.3 Survivor perception (Phase 7)
`survivor_detection.py`: a hand-built detector on the downward camera image: finds saturated orange clothing (HSV colour), then checks for a skin-tone head nearby, a long thin body shape and blue legs; gives each candidate a confidence. It is a baseline tuned for the simulated mannequin, **not a general person detector**. Colour-matching look-alikes (orange barrels, cone, tarp, crate) can fool it. `survivor_confirmation.py` requires temporal consistency (seen in e.g. 5 of 8 frames), then optionally asks the VLM in two steps ("is there a person?" then details). VLM verdicts: SURVIVOR_CONFIRMED, REJECTED_BY_VLM, LOW_CONFIDENCE. Without a VLM everything stays UNVERIFIED_CANDIDATE.

### 5.4 Survivor localisation (Phase 8) (`survivor_localization.py`)
Turns "a survivor appears at pixel (u,v)" into a 3D point. One camera gives only a viewing ray, so each survivor is a static 3D point tracked by an **extended Kalman filter**: first measurement intersects the ray with a plane at an assumed height (coarse, large uncertainty along the ray); as the drone moves, rays from different positions intersect (triangulation) and the uncertainty shrinks. Inputs: pixel, PX4 pose, camera model. Output: position in PX4 local NED (+ latitude/longitude), 1-sigma uncertainty, height, observation count. **It uses PX4's pose estimate, not SLAM and not ground truth.** Typical error 0.1-0.3 m.

### 5.5 Search planner (Phase 9) (`search_planning.py`)
Given a rectangle (north/east range) it generates a **lawnmower path**: parallel lanes spaced from the camera footprint at the flight altitude (with overlap). Keeps a coverage grid of what the camera has seen. When a located candidate appears it inserts a short detour to view it from two sides (inspection viewpoints) so the localiser gets a baseline and the VLM gets an oblique view. It only proposes horizontal goals; altitude, arming, landing belong to the flight controller. It can skip a waypoint the navigator reports unreachable.

### 5.6 Mission manager (Phase 10) (`mission_logic.py`, `mission_manager_node.py`)
Runs on the ground side of the mission: preflight gate (checks every required node is alive before START; refuses the mission otherwise), sends START, watches phases (TAKEOFF, SEARCH, RETURN, LANDING), decides when the search ends (area covered, time budget used, battery low, objective met), sends RETURN_HOME, and writes the **mission report** JSON (outcome, coverage, each located survivor with coordinates/uncertainty/status, avoidance count, timeline).

### 5.7 Natural-language missions (Phase 11) (`mission_language.py`)
Takes the sentence and a catalogue of named places (`missions/places_disaster.json`: area rectangles, altitude limits, time budgets). A rules interpreter (keywords) or an LLM interpreter turns it into a mission; strict checks refuse anything outside limits or ambiguous (grounding checks: numbers and places in the output must appear in the instruction). It understands "avoid falling debris" (debris protection on), "only report confirmed" (needs VLM), "prioritize X" (start search nearest X). The operator confirms the read-back before flight.

### 5.8 SLAM (village/urban worlds)
- A planar **gpu_lidar** (720 rays, 10 Hz) on the drone.
- `slam_bridge`: levels each scan using the attitude, drops returns outside the flight plane, only forwards scans when the drone is at the mapping altitude (6 m), publishes the TF odom->base_level from PX4 odometry (converted to ENU), optionally adds synthetic odometry drift for testing.
- `slam_toolbox` (ROS package, async online mode): 2D pose-graph SLAM with scan matching and loop closure; publishes `/map` (occupancy grid, 0.25 m, ~1 Hz), TF `map->odom`, `/pose`. Urban config (`slam_toolbox_urban.yaml`) uses stricter match acceptance because repeated window patterns caused a false match that rotated the map ~25 degrees once.
- Honest facts: the map is a 2D **slice at 6 m**. PX4 (with simulated GNSS) still flies the drone, so this is **not GPS-denied flight**. SLAM's input odometry is PX4's estimate. Measured error vs truth: village 0.1-0.6 m (and it corrected deliberately degraded odometry 1.1 m -> 0.16 m, 6.7 m -> 0.21 m); urban world 0.5-2.8 m RMSE, worse than the odometry it is given.

### 5.9 Navigator (`navigation.py`, `navigator_node.py`)
Sits between the search planner and the flight controller. Takes the planner's raw goal, plans an **A\* route on the SLAM occupancy grid** (0.5 m planning cells, obstacles inflated by 1.75 m, unknown space passable but costlier), follows a look-ahead point along the route, replans every second or when blocked, and publishes the goal to the controller. Converts goals between PX4 and map frames using both pose estimates. Reports a target as UNREACHABLE (planner skips it) if no route exists for 4 s or the vehicle stands at the nearest reachable point. Declares LOCALIZATION LOST (stops publishing goals; controller returns home) if the SLAM transform/scan is stale 6 s or the SLAM pose jumps relative to PX4. Return home is also routed on the map; if no route home exists it retraces its recorded breadcrumbs (flown path) instead. Honest limit: it plans only in the 2D slice, so it cannot see overhead or slabs just below the plane.

### 5.10 Visualisation
`mission_viz` republishes paths/markers; RViz config `village.rviz` shows the SLAM map, lidar scan, planned route, flown path, search lanes, survivor markers, debris tracks, mission text and the camera image. Only live topics are drawn.

## 6. The worlds

- **Original small world** (`collapsed_building_rescue.sdf`, `survivor_variants/`): one collapsed building, used for Phases 0-16 (baseline, still works as fallback).
- **Village world** (`make_village.py`): 125 m x 130 m, houses/roads/trees and a collapsed building; built for SLAM.
- **Urban disaster zone** (`scripts/disaster/make_disaster_zone.py`): dark overcast earthquake site. Procedurally generated (no downloaded assets): damaged multi-storey concrete frames (facades from a cell grid with window openings and jagged broken tops), drooping floor slabs, rubble piles from random convex chunks plus steel beams, wrecked cars, a U-shaped courtyard, a street blocked by fallen panels with side walls forcing a detour. Six scenarios: A (none), B (one survivor), B2 (courtyard), C (three survivors + five look-alikes), D (survivors under leaning slabs), F (look-alikes only, false-positive control). Performance tricks: visuals merged into one mesh per material and all static collisions in one body (took real-time factor from 0.6 to ~0.9). Truth files exist only for offline scoring. Dust/haze deliberately not simulated (particles would pollute the depth/lidar). Flight altitude fixed at 6 m so the lidar slice hits solid wall bands.
- Scope rule: **searches are exterior only**; the interior of standing buildings is out of scope (two flights hit slabs inside).

## 7. MuJoCo simulation (`simulation/mujoco/`)
A completely separate simulator, needing no ROS/Gazebo/PX4 for the standalone mode (own Python venv with `mujoco`).
- Vehicle: x500 mass/inertia/rotor layout, thrust and motor lag; thrust and torque are applied by four motor actuators, so it moves only through physics.
- `controllers/quadrotor.py`: our own cascaded controller (position -> velocity -> attitude -> motor thrusts). **Not PX4.**
- Scenes: generated from the same shapes exported by the Gazebo generator (same layout, simpler geometry, same textures).
- `run_flight.py`: hover/waypoints/obstacle-navigation tests. `run_mission.py`: the rescue workflow reusing project modules: interpreter -> search planner -> lidar occupancy map with **known pose (not SLAM)** + the project's A* -> rendered downward camera -> detector -> localisation EKF -> report -> return -> land. No VLM, no SLAM, no debris avoidance, no mission manager/failsafes; vehicle state is simulator truth.
- **PX4 in the loop** (`px4_hil_bridge.py`, `run_px4_mujoco_trial.sh`): MuJoCo plays the simulator for PX4 over MAVLink HIL: the bridge listens on TCP 4560 (PX4 connects as client), sends noisy IMU/magnetometer/barometer and GPS, receives PX4's motor outputs. The unchanged flight_controller then flies hover/waypoints through PX4. Needed fixes: sensor noise (PX4's validator marks constant values stale), a magnetic field matching PX4's world model, and the fact PX4 blocks at start-up until the simulator connects. It uses an iris-like vehicle model (airframe 10016). Only hover and a 3-waypoint pattern; no camera/lidar in that loop.

## 8. Testing and evaluation

- ROS unit tests (`src/uav_autonomy/test`): ~550 pass (mission language, mission logic, navigation, search planning, safety, tracking...). Two style-lint tests (flake8, pep257) fail.
- MuJoCo tests: 12 pass (scenes load, free fall under gravity with motors off, hover, landing, waypoints, lidar mapping, collision-free navigation).
- Flight verification: after each Gazebo flight `verify_ulog.py` reads PX4's own log and checks armed, offboard, takeoff reached, altitude hold, landed, no excursion, no impact.
- Offline scoring: `summarize_runs.py`, `evaluate_slam.py` (SLAM trajectory and map vs ground truth), `validate_disaster.py` (world geometry and route checks), planner benchmark, RTF probe, gap monitor (diagnoses telemetry stalls).
- Batch scripts: `run_disaster_batch.sh ext`, `run_mujoco_batch.sh`, separation check scripts.

## 9. Key measured results (all simulation)

- Gazebo urban exterior batch: 10/10 flights complete and landed, 0 failsafes, all PX4-log checks passed; survivors found 8 of 9 in searched areas (median error 0.21 m); 5 avoidance manoeuvres for 6 boxes, 0 collisions.
- Faculty demo flight: survivor reported at 54.95 N, 16.94 E vs truth 55.0 N, 17.0 E (0.10 m); landed; PX4 checks pass.
- MuJoCo missions: 6/6 complete, 0 collisions, survivors found 0.21-0.46 m error. MuJoCo faculty mission: 0.30 m error.
- PX4-in-the-loop MuJoCo: 18 hover/waypoint flights complete with all PX4-log checks; hover hold RMSE ~4.4 cm; one lockstep hang unresolved.
- SLAM urban: RMSE 0.5-2.8 m (faculty run 1.33 m; 0.69 m after best alignment).

## 10. Problems found along the way (good to understand)

- **False "telemetry stale" failsafes**: PX4 publishes vehicle status at ~2 Hz of simulated time; at low real-time factor that exceeded the controller's 1 s wall-clock limit. Fixed by making the world cheaper and adding a separate status timeout.
- **Impacts in the collapsed building**: a 2D lidar slice can't see overhead; the old climb-first return and flights into facade openings crashed. Fixed with map-routed return, breadcrumb backtracking, exterior-only searches.
- **SLAM false match** in repetitive facades; stricter settings; a too-narrow search window made slam_toolbox crash (kept as evidence).
- **Navigator replanning flood** in MuJoCo when called every control tick instead of 10 Hz.
- **Process interference**: running MuJoCo next to a Gazebo flight spoiled that flight; a refused launch once killed the other simulation (fixed); launchers now refuse when the other's PX4 stack is running.

## 11. Honest limitations

Simulation only. Survivor detector is a colour/shape baseline for one mannequin; VLM confirmation needs the NVIDIA GPU (driver not loaded on the current kernel 7.0.0-34; kernel 7.0.0-30 works after a reboot). SLAM is 2D, worse than PX4's own estimate in the urban world, and not used to fly. Debris avoidance needs ~0.9 s warning and isn't map-aware. MuJoCo version lacks VLM, SLAM, debris avoidance, mission manager and failsafes. Search areas are exterior only. Real-time factor about 0.5 on Intel graphics.

## 12. Where things are (repo `~/Desktop/drones/ros2_ws`)

- `src/uav_autonomy/uav_autonomy/` all nodes and pure-Python logic; `src/uav_interfaces/` custom messages.
- `scripts/` run scripts (`run_flight_trial.sh` starts Gazebo, PX4, agent and nodes; `mission/run_mission.sh`, `mission/run_instruction.sh`), `disaster/` (urban generator, demo, validators, batch), `slam/` (scoring, plots), `village/`.
- `src/uav_autonomy/worlds/`, `models_slam/`, `models_disaster/`, `config/` (bridge, SLAM, RViz).
- `missions/` places catalogue and generated missions; `results/` all evidence (kept, including failures).
- `simulation/mujoco/` the MuJoCo simulation (README inside).
- `docs/` per-phase docs, `GAZEBO_READINESS_AUDIT.md`, `MUJOCO_PORT_PLAN.md`, `MULTI_SIM_COMPARISON.md`, `FACULTY_DEMO_RUNBOOK.md`; `~/Desktop/drones/MULTI_SIM_PROGRESS.md` full history; `UAV_DISASTER_RESCUE_COMPLETE_STUDY_GUIDE.md` the long study guide.

## 13. History in one paragraph

Phases 0-1: flight foundation and sensors (takeoff/hover/landing, cameras). 2-6: debris perception, tracking, prediction, risk, avoidance. 7: survivor detection + VLM. 8-10: localisation, search planner, mission manager. 11: natural-language missions. 12-16: integrated missions, repeats, fault injection, live VLM runs. Then: village world + real SLAM + map-aware navigation; urban disaster zone; MuJoCo simulation and PX4-in-the-loop; final audits and faculty demo.
