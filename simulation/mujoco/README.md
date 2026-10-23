# MuJoCo quadrotor simulation (standalone)

An independent simulator of the UAV disaster-rescue project. It needs **no
ROS, no PX4, no Gazebo** and no running daemon. Closing or never starting the
Gazebo stack changes nothing here, and nothing here writes into the Gazebo
results (`../../results/`).

What it is, and what it is not:

| | MuJoCo simulation (this folder) | Gazebo stack (`ros2_ws/`) |
|---|---|---|
| Physics | MuJoCo 3.16: x500 mass/inertia/rotor layout/thrust limits from the PX4 model, motor lag, rotor drag, contact | Gazebo Harmonic, same vehicle |
| Autopilot | **own cascaded controller (not PX4)** | PX4 SITL (unmodified) |
| Vehicle state | **simulator ground truth, no noise, no estimator** | PX4 EKF2 with simulated GNSS |
| Obstacle map | lidar rays + **known pose** (mapping, **not SLAM**) | slam_toolbox 2D SLAM |
| Survivor perception | project detector on rendered MuJoCo frames; no VLM | detector + VLM (VLM needs the GPU) |
| Survivor localisation | project EKF on the rendered detections | same module |
| Navigation / search planning | the project's A*, search planner, mission interpreter | same modules inside ROS nodes |
| Debris avoidance | **not integrated** (boxes can be released; no avoidance chain) | detector, tracker, predictor, risk, avoidance |
| PX4 in the loop | **hover x3 and 3 waypoints x1 demonstrated** (separate script, iris-like model; section below) | native |

The two are **not equivalent**: this one is simpler on purpose (see the
comparison in `docs/MULTI_SIM_COMPARISON.md`).

## Setup (once, user space, no sudo)

```bash
cd ~/Desktop/drones/ros2_ws/simulation/mujoco && ./setup_env.sh
```

Creates `.venv` with `mujoco` only; numpy, OpenCV, SciPy, matplotlib and pytest
come from the system Python.

## Generate the scenes

The scenes come from the shapes exported by the Gazebo world generator (same
urban layout). They are already generated; to rebuild:

```bash
cd ~/Desktop/drones/ros2_ws && python3 scripts/disaster/make_disaster_zone.py --shapes-only && cp results/disaster_gazebo/worlds_shapes/*.json simulation/mujoco/worlds/shapes/
```
```bash
cd ~/Desktop/drones/ros2_ws/simulation/mujoco && .venv/bin/python scripts/make_mujoco_scene.py
```

Scenes: `worlds/mj_urban_{A,B,B2,C,D,F}.xml` (A none, B one survivor on the
collapse, B2 one in the courtyard, C three survivors + five look-alikes,
D two partly under slabs, F look-alikes only).

## Run (GUI window)

```bash
cd ~/Desktop/drones/ros2_ws/simulation/mujoco && MUJOCO_GL=glfw .venv/bin/python scripts/run_flight.py nav --gui
```
Other scenarios: `hover`, `waypoints`, `nav --goal EAST,NORTH`. The window
follows the vehicle (MuJoCo viewer); close it to end the run.

The rescue mission from a sentence, with the window:

```bash
cd ~/Desktop/drones/ros2_ws/simulation/mujoco && MUJOCO_GL=glfw .venv/bin/python scripts/run_mission.py "Search the collapsed apartment block for survivors, report survivor locations, and return to base." --scene urban_B --gui
```

## Run headless and test

```bash
cd ~/Desktop/drones/ros2_ws/simulation/mujoco && MUJOCO_GL=egl .venv/bin/python scripts/run_flight.py waypoints
```
```bash
cd ~/Desktop/drones/ros2_ws/simulation/mujoco && MUJOCO_GL=egl .venv/bin/python -m pytest tests -q
```

`MUJOCO_GL=egl` renders offscreen (works on the Intel GPU too); `glfw` needs a
display. Each run writes `report.json`, `trajectory.csv` and `flight.png`
(or `mission_report.json`) to `results/<scenario>_<scene>_<time>/` or `--out`.

## PX4 in the loop (hardware-in-the-loop bridge)

PX4 SITL (airframe `10016_none_iris`, the MAVLink-simulator airframe) flies the
MuJoCo vehicle; the project's **unchanged** `flight_controller.py` commands PX4
through the Micro XRCE-DDS agent. MuJoCo plays the simulator over PX4's
MAVLink HIL interface: it listens on TCP 4560 (PX4 connects as client), sends
`HIL_SENSOR` (IMU, magnetometer, barometer, with small Gaussian noise) and
`HIL_GPS` at 250/10 Hz and applies PX4's `HIL_ACTUATOR_CONTROLS` to the four
rotors. Needs ROS 2 sourced and `ros2_ws` built; start nothing else before it
(no Gazebo flight at the same time).

```bash
cd ~/Desktop/drones/ros2_ws/simulation/mujoco && scripts/run_px4_mujoco_trial.sh my_hover 3.0
```
```bash
cd ~/Desktop/drones/ros2_ws/simulation/mujoco && MODE=waypoint scripts/run_px4_mujoco_trial.sh my_waypoints 3.0
```
Results: `results/px4_hil/<tag>/` (`verification.json` = PX4-log checks,
`flight_controller.log`, `bridge.log`, `poses.csv`, `flight.ulg`). PX4's stored
parameters are saved and restored by the script. Measured (2026-10-20): three
hover flights and one 3-waypoint flight, all complete, all 8 PX4-log checks
pass; hover hold RMSE 4.3-4.5 cm, landing offset 0.2-1.4 cm; waypoints reached
to 0.4 / 17.6 / 0.4 cm; without the controller's arrival-speed gate that flight landed 2.1-2.2 m from the start (explained: "home reached" fires while flying past home), with it (`arrival_max_speed:=0.3`, now the default in waypoint mode) 0.35 m. The
vehicle is an iris-like model (mass 1.5 kg) matching the airframe's parameters,
the EKF uses simulated GNSS, there is no camera or lidar in this loop.

## Checks you can re-run

```bash
cd ~/Desktop/drones/ros2_ws/simulation/mujoco && MUJOCO_GL=egl .venv/bin/python -m pytest tests -q
```
```bash
cd ~/Desktop/drones/ros2_ws/simulation/mujoco && tests/check_isolation.sh
```
```bash
cd ~/Desktop/drones/ros2_ws/simulation/mujoco && tests/check_separation.sh
```
`check_separation.sh` (about 12 min, ROS workspace built, nothing else running) starts a
Gazebo mission, tries to start the PX4 trial (must refuse), then the reverse. The PX4 trial
refuses while any PX4 / XRCE / Gazebo process is running, and a refused launch leaves the
running simulation untouched. If the bridge gets no reply from PX4 for 8 s it reports
`STALL` (one unresolved lockstep hang was seen in 19 trials).

## Files

- `models/x500_body.xml` quadrotor (4 site-motors: thrust + reaction torque)
- `controllers/quadrotor.py` plant helper and cascaded controller
- `controllers/flight_sim.py` stepping, collision log, lidar, occupancy map
- `controllers/behaviours.py` takeoff, goto, land
- `controllers/perception.py` rendered camera, detector glue, tracker
- `models/iris_hil_body.xml` vehicle for the PX4 loop (airframe 10016 values)
- `scripts/` scene builder, flight and mission runners, batch, checks,
  `px4_hil_bridge.py` and `run_px4_mujoco_trial.sh` (PX4 loop)
- `tests/test_mujoco_sim.py` headless tests
- results: `results/`

## Reused project modules (pure Python, loaded from `ros2_ws/src/uav_autonomy`)

`mission_language`, `search_planning`, `navigation`, `survivor_detection`,
`survivor_localization`. They are imported as files; no ROS node is started.

## Measured results (2026-10-20)

Flights `results/final/`, missions `results/batch/` (`results/summary.json`),
tests `tests/` (12 passed), isolation `tests/check_isolation.sh` (OK).

| | |
|---|---|
| hover at 6 m | altitude RMSE 9.4 mm, touchdown 0.34 m/s, landed on the pad |
| square, 3 m/s | 6 of 6 waypoints, cross-track RMSE 0.53 m (max 1.37 m), speed overshoot to 4.4 m/s |
| detour around the blocked street | 111 m (straight 96 m), 0 collisions, min lidar range 1.69 m, landed 0.11 m from the pad |
| 6 rescue missions (sentence to landing) | 6 of 6 complete, 0 collisions, 6 of 6 searched-area survivors found, error 0.21-0.46 m; look-alikes reported as unverified candidates (F: 2, C: 1-4) |
| a mission that entered the collapsed block (`results/dev/mission_B_interior`, kept) | **collided** with a drooping floor tile at 139 s and did not get home: the 2D lidar cannot see slabs just below its plane. Searches are therefore exterior only |

The runs are deterministic (identical repeats: no noise in the standalone
controller).

## Limits

No SLAM (the map uses the known pose), no VLM, no debris-avoidance chain, no
mission manager or failsafes in the standalone mission, no wind, simplified
rubble (stacked cylinders), the standalone controller sees ground-truth state
and is not PX4. PX4 in the loop: hover and one 3-waypoint flight only. Results
here must not be quoted as results of the PX4 / Gazebo rescue system.
