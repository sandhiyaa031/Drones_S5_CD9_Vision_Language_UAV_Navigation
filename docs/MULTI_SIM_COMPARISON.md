# Gazebo + PX4 versus MuJoCo: what each simulation is and is not

Two separate simulations of the same urban earthquake-collapse zone. They share
the layout, the textures and several pure-Python autonomy modules. They are
**not equivalent**: the MuJoCo one is simpler on purpose, and results from one
must not be quoted as results of the other.

Measured 2026-10-20 on the same machine (i7-14650HX, Intel graphics; the NVIDIA
driver is not loaded on kernel 7.0.0-34). Evidence: `ros2_ws/results/disaster_gazebo/`,
`simulation/mujoco/results/`, `MULTI_SIM_PROGRESS.md`.

| | Simulation A: Gazebo + PX4 + ROS 2 | Simulation B: MuJoCo |
|---|---|---|
| **Launch (GUI)** | `ros2_ws/scripts/disaster/run_demo.sh` | `simulation/mujoco/scripts/run_flight.py nav --gui`, `run_mission.py ... --gui` |
| **Launch (headless / test)** | `scripts/disaster/run_disaster_batch.sh ext` | `scripts/run_mujoco_batch.sh`, `python -m pytest tests` |
| **Needs** | ROS 2 Jazzy, PX4 SITL, Gazebo Harmonic, XRCE agent | only the `.venv` with `mujoco` (no ROS, no Gazebo, no PX4) |
| **World** | 6 SDF worlds; textured procedural meshes, 39k triangles, shadows | same layout as MJCF boxes/cylinders with the same textures; rubble as stacked cylinders; no window meshes beyond the shape cuts |
| **Vehicle physics** | Gazebo, x500 (2.0 kg + rotors, inertia, thrust 8.55 N/rotor, motor lag) | MuJoCo, x500 values (standalone) or an iris-like model matching PX4 airframe 10016 (PX4 loop) |
| **Autopilot / controller** | PX4 SITL (unmodified) commanded by `flight_controller.py` | standalone cascaded controller (**not PX4**); in the PX4 loop: PX4 + the **unchanged** `flight_controller.py` |
| **Vehicle state used by the controller** | PX4 EKF2 with simulated GNSS | standalone: **simulator ground truth, no noise**; PX4 loop: PX4 EKF2 on simulated noisy IMU/mag/baro/GNSS |
| **Camera / depth** | Gazebo RGB and upward depth, ROS topics | rendered RGB (detector and localiser run on it); no depth camera yet |
| **Lidar** | Gazebo gpu_lidar, 720 rays, 10 Hz | MuJoCo rays, 360 per scan, 10 Hz |
| **Mapping** | slam_toolbox 2D SLAM with loop closure (real SLAM) | occupancy grid from lidar rays with the **known pose** (**not SLAM**) |
| **Survivor perception** | project detector + two-step VLM (VLM needs the NVIDIA GPU; absent today) | same detector module on MuJoCo frames; **no VLM** |
| **Survivor localisation** | project EKF, PX4 pose | same EKF, simulator pose (ground truth, labelled) |
| **Search planning, navigation** | same `search_planning`, A* `navigation` inside ROS nodes | same modules, called directly |
| **Mission from a sentence** | rules or LLM interpreter, mission manager, preflight gate | rules interpreter; no mission manager / preflight gate / failsafe monitor |
| **Obstacle (debris) avoidance** | detector, tracker, predictor, risk, planner: 5 manoeuvres for 6 boxes, 0 collisions | **not integrated**; debris pool exists in the scenes, no avoidance chain |
| **Failsafes** | PX4 + controller (stale telemetry, safety monitor, map-routed return); fault injection tested | none beyond collision logging |
| **Return home** | along the SLAM map at 6 m, climb only near home | along the lidar map at 6 m |

## Measured results

| | Gazebo + PX4 (exterior areas, final configuration) | MuJoCo (standalone controller) |
|---|---|---|
| Mission flights | 10 of 10 complete and landed, 0 failsafes; PX4-log checks 10 of 10 | 6 of 6 complete and landed, 0 collisions |
| Survivors in the searched areas found | 8 of 9 (error median 0.21 m, max 1.07 m) | 6 of 6 (error 0.21-0.46 m) |
| Look-alike candidates reported (no VLM) | yes (F: 2 of 2 candidates were look-alikes) | yes (F: 2, C: 1-4) |
| Debris | 6 boxes, 5 manoeuvres, 0 collisions, closest box 1.34 m | not tested |
| Mapping error against truth | SLAM RMSE 0.5-2.8 m, map precision 0.31-0.61 | not applicable (known pose) |
| Hover accuracy | PX4 log: hold RMSE 4.3-4.5 cm (PX4 loop in MuJoCo) | 9.4 mm RMSE (ground-truth state) |
| Waypoint tracking | PX4 loop: 3 waypoints to 0.4 / 17.6 / 0.4 cm | square at 3 m/s: cross-track RMSE 0.53 m, max 1.37 m |
| Obstacle-aware flight | search among rubble and walls, collision-free in the final configuration | 111 m detour around the blocked street, 0 collisions, min lidar range 1.69 m |
| Speed | real-time factor about 0.55 with GUI (Intel) | 1.0-1.25x with rendering (nav 123 s in 99 s); PX4 loop 2.6x without cameras |
| Repeatability | repeats differ (PX4, SLAM, timing) | bit-identical repeats (no noise) |
| Tests | 552 unit tests pass, 6 skipped; 2 style-lint tests fail (280 flake8 findings) | 12 headless tests pass (22 s); `check_isolation.sh` and `check_separation.sh` pass |

PX4 in the MuJoCo loop (re-audited 2026-10-21: 18 hover/waypoint flights complete with
every PX4-log check passing, 1 hung in lockstep, unresolved) shows the autopilot and the
project's controller working against MuJoCo physics (hover hold RMSE 4.4 cm; all three
waypoints reached; landing offset 0.35 m with the arrival-speed gate, 2.1-2.2 m without
it). It uses an iris-like model and a GNSS-aided EKF, with no camera, lidar, VLM or
avoidance in the loop: it is **PX4-in-the-loop flight, not the rescue mission**.

## Known limitations

Gazebo + PX4
- Intel graphics only today (no NVIDIA driver on the running kernel): low
  real-time factor (about 0.5-0.6), no live VLM.
- SLAM is a 2D slice at 6 m; error 0.5-2.8 m in this world; PX4 flies the
  vehicle, SLAM supplies the obstacle map (not GPS-denied flight).
- Interiors of standing structures are out of scope (2D lidar cannot see
  overhead): two flights in the first batches ended in impacts there.
- Coverage of larger areas is limited by the time budget (the headline mission
  found 1 of 2 survivors).
- Debris avoidance is bounded (about 0.9 s warning) and is not checked against
  the map.
- The controller's `status_timeout_s` is 3.0 s for urban runs (default 1.0 s,
  position telemetry still 1.0 s): measured need, documented.

MuJoCo
- No VLM, no SLAM, no debris avoidance, no mission manager or failsafes in the
  standalone mission.
- Ground-truth vehicle state, no sensor noise, no wind.
- Simplified rubble and no window meshes.
- PX4 loop: iris-like vehicle, hover and a 3-waypoint flight only; one lockstep hang in
  19 trials (unresolved); landing offset 2.1 m without the arrival-speed gate (0.35 m
  with it, now the script default).
- GUI needs an X11 display (GLFW started with `WAYLAND_DISPLAY` unset here).
