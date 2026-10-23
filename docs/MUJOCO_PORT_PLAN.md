# MuJoCo port plan

Status: **plan only. Nothing runs in MuJoCo yet.** MuJoCo is not installed on
this machine (`import mujoco` fails). A `mujoco 3.16.0` wheel for Python 3.12
is available from PyPI. This plan does not change the Gazebo + PX4 setup; the
two simulators are meant to live side by side.

> **Status update (2026-10-20 evening):** this was written as a plan. Since
> then the port was built and tested; see `simulation/mujoco/README.md`,
> `MULTI_SIM_PROGRESS.md` and `MULTI_SIM_COMPARISON.md`. What exists:
> (M0-M2) isolated venv, x500 MuJoCo model, standalone controller, hover,
> waypoint, landing and **PX4 in the loop** (hover x3, 3 waypoints x1: all
> PX4-log checks pass, unchanged `flight_controller.py`); (M3-M5, standalone
> only) rendered camera, detector and localiser on MuJoCo frames, debris pool
> without avoidance; (M7 partly) lidar occupancy map with known pose and the
> project's A* (not SLAM). What does **not** exist: the debris avoidance chain,
> SLAM, VLM, depth camera and the mission manager in MuJoCo, and camera/lidar
> inside the PX4 loop. Plan items below that differ from what was done: the
> bridge is a TCP *server* (see the correction in section 1); the PX4 vehicle is
> an iris-like model matching airframe 10016 (4001_gz_x500 enables the Gazebo
> bridge); PX4's data validator needs noisy sensors (constant values are flagged
> STALE); the field must match PX4's world model (declination 3.4 deg).

> **Audit result (2026-10-21, re-measured; evidence `simulation/mujoco/results/audit/`):**
> - **PX4-in-the-loop flight is demonstrated, full rescue autonomy in MuJoCo is NOT.**
>   PX4 SITL + the unchanged `flight_controller.py` + MuJoCo physics: hover x3 and
>   3-waypoint x3 flights, all PX4-log checks pass (hover hold RMSE 4.4 cm).
>   No camera, lidar, VLM, SLAM, debris avoidance or mission manager is inside that loop.
> - The standalone MuJoCo mission (sentence -> planner -> rendered camera -> detector ->
>   localiser -> map-routed return -> landing) runs with ground-truth vehicle state, a
>   lidar map with known pose and NO VLM/SLAM/avoidance/failsafes: 6 of 6 missions
>   complete, 0 collisions. It is a different system from the Gazebo one, not a port of
>   all of it.
> - The 2.1 m landing offset after the waypoint flight is explained (arrival-speed gate
>   off; with the gate 0.35 m). One PX4-loop trial of 19 that ran to a conclusion hung in
>   lockstep (unresolved, now reported by a watchdog).
> - Directory: the MuJoCo code lives in `ros2_ws/simulation/mujoco/` (inside the repository).

## 1. The central decision: keep PX4, replace only Gazebo

The autonomy stack talks to the vehicle only through PX4's uORB topics over
XRCE-DDS: `/fmu/in/{offboard_control_mode,trajectory_setpoint,vehicle_command}`
and `/fmu/out/{vehicle_local_position_v1,vehicle_odometry,vehicle_status_v4,
vehicle_land_detected,vehicle_command_ack_v1,battery_status_v1}`. It never
talks to Gazebo, apart from these:

- camera images and the lidar scan, through `ros_gz_bridge`
- `/clock`
- test tooling: debris spawning (`gz service`), the ground-truth pose
  recorder, camera-view presets

PX4 (commit d983b64, already built) contains `simulator_mavlink`, the HIL
interface that jMAVSim and Gazebo-classic used. **Correction (2026-10-20,
read from `SimulatorMavlink.cpp` and `px4-rc.mavlinksim`): PX4 is the TCP
*client* here** (`simulator_mavlink start -c 4560` logs "Waiting for simulator
to accept connection on TCP port 4560"), so the *simulator side must listen* on
port 4560; the first version of this plan had it the other way round. The
simulator sends `HIL_SENSOR` (IMU, magnetometer, barometer; its `id 0` message
advances PX4's lockstep clock) and `HIL_GPS`. PX4 answers with
`HIL_ACTUATOR_CONTROLS` (motor commands). It is started by airframe
`10016_none_iris` (`ROMFS/.../px4-rc.mavlinksim`); airframe `4001_gz_x500`
cannot be used for this because it enables the Gazebo bridge
(`SIM_GZ_EN`). `pymavlink` is already installed in `~/.local`. **Not yet
demonstrated:** see `MULTI_SIM_PROGRESS.md` for the state of the bridge.

**So: MuJoCo simulates the physics and sensors, a Python bridge speaks MAVLink
HIL to the unchanged PX4, and the whole ROS 2 stack runs as it does today.**
`flight_controller.py` stays the only node that commands PX4. PX4 source and
safety parameters stay as they are.

Rejected alternatives:

| Option | Why not |
|---|---|
| MuJoCo with its own controller, no PX4 | Throws away the validated flight controller, PX4 failsafes, the log verification and every flight result. It becomes a different system, not a port |
| PX4 SIH (built-in physics) + MuJoCo only for drawing and cameras | Quick, but MuJoCo would not simulate the vehicle, so debris contact and collisions would not be physical |
| PX4 `rotorpy` interface | Ties the port to another simulator's message format; HIL MAVLink is the documented, general one |

## 2. MuJoCo quadrotor model (MJCF)

The x500 values PX4 is tuned for (from `Tools/simulation/gz/models/x500*`
and airframe `4001_gz_x500`):

| Quantity | Value |
|---|---|
| Body mass | 2.0 kg (+ 4 rotors of 0.016 kg) |
| Inertia (body) | ixx = iyy = 0.02167, izz = 0.04 kg·m² |
| Rotor positions (FRD, m) | (0.13, 0.22), (−0.13, −0.20), (0.13, −0.22), (−0.13, 0.20) |
| Spin directions | CA_ROTOR*_KM: +0.05, +0.05, −0.05, −0.05 |
| Thrust | F = 8.54858e-6 · ω² N, ω_max = 1000 rad/s (8.55 N per motor) |
| Yaw moment | M = 0.016 · F |
| Motor time constant | 0.0125 s up, 0.025 s down |
| Rotor drag | 8.06428e-5 (velocity-proportional, per rotor) |

How to model it:

- One free-joint body with box/cylinder geoms for collision.
- Four `site`s at the rotor positions. Each step, the bridge sets
  `xfrc_applied` (thrust along body z, yaw moment), or uses four `motor`
  actuators with `gear` = [0 0 1 0 0 ±0.016].
- The first-order motor lag and ω = command · ω_max are done in the bridge,
  because PX4 sends normalised commands in [0, 1].
- Rotor drag is applied as a force opposite the body's horizontal velocity.
- Physics step 1 ms (`timestep 0.001`, RK4). Sensors go out at PX4's rates:
  IMU 250 Hz, GPS 10 Hz, barometer 50 Hz.

Sensor bodies, same mounts as in Gazebo:

- downward RGB camera (`camera_link`)
- upward depth camera (`depth_up_link`, z 0.39 m)
- planar lidar at model z 0.16 m

## 3. Environment

`scripts/village/make_village.py` already builds every world from a few
primitives (boxes, cylinders, spheres, manikins). Add an MJCF writer next to
the SDF writer, so one generator produces both and the truth files stay
identical. The original single-building worlds (`base`, `multi`, ...) are
hand-written SDF; port them by a small SDF→MJCF converter for the primitive
shapes, or rebuild them in the generator.

Watch out for:

- **Survivor appearance.** The survivor detector is a colour/shape baseline
  tuned on Gazebo renders. MuJoCo's renderer shades differently (simpler
  lighting, no PBR). Colours must be matched and the detector re-measured on
  MuJoCo frames, **not silently retuned**.
- **The VLM** was validated on Gazebo images; its accuracy on MuJoCo renders
  is unknown until measured.
- **Shadows and textures** are simpler. Do not compare image-based results
  across simulators without saying so.

## 4. How each signal reaches the stack

| Signal | Gazebo today | MuJoCo port |
|---|---|---|
| Vehicle pose, velocity (to PX4) | gz plugins → PX4 `gz_bridge` | bridge computes IMU (body acceleration minus gravity, angular rate) + noise, magnetometer (Earth field rotated into the body frame), barometer (ISA pressure from height), GPS (from the home lat/lon) → `HIL_SENSOR`, `HIL_GPS` |
| Vehicle state (to the ROS nodes) | PX4 `/fmu/out/*` | **unchanged**: PX4 EKF2 estimates from the HIL sensors, published over XRCE-DDS |
| RGB camera | gz camera → ros_gz_bridge `/camera/image_raw`, `/camera/camera_info` | `mujoco.Renderer` offscreen on the camera's MJCF camera; published by rclpy on the **same topics, encodings and intrinsics** |
| Depth camera | gz depth → `/depth_up/image_raw` (32FC1) | `Renderer` with depth rendering on; convert MuJoCo's depth buffer to metres; same topic and encoding |
| Lidar | gz gpu_lidar → `/uav/scan_raw` | `mujoco.mj_multiRay` from the lidar site, 720 rays, 10 Hz, same `LaserScan` fields; `slam_bridge` unchanged |
| Clock | gz `/clock` | the bridge publishes `/clock` from MuJoCo time; nodes keep `use_sim_time` |
| Ground truth (offline scoring only) | `scripts/sensor_study/record.py` → `poses.csv` | the bridge writes `poses.csv` in the **same columns**, so every scorer is reused |
| Falling debris | `spawn_study.py` creates boxes through `gz service` | MuJoCo cannot add bodies at runtime: pre-declare N free bodies parked far away, "release" one by setting its `qpos`/`qvel`; `releases.json` in the same format |

## 5. Commands and flight

Nothing changes on the command side:

```
mission_manager → flight_controller → /fmu/in/* → PX4 (offboard) → HIL_ACTUATOR_CONTROLS → bridge → MuJoCo
```

Takeoff, hover, waypoints, debris avoidance, return and landing are what
they are today: the controller's set-points flown by PX4's position
controller. The bridge only turns motor commands into forces.

Launch: start PX4 with `PX4_SYS_AUTOSTART=10016` (MAVLink simulator) and set
the x500 geometry parameters (`CA_ROTOR*`, `MPC_THR_HOVER`) at start-up through
the parameter protocol, from the new trial script. This is the same kind of
parameter handling as `UXRCE_DDS_SYNCT` today, with backup and restore.
**No PX4 source or airframe file is edited.**

Two things that will need care:

1. **Magnetometer.** The bridge must use the field PX4 expects at the home
   location (lat 47.397971, lon 8.546163: inclination ≈ 63°, declination
   ≈ 3°). Otherwise preflight fails with "strong magnetic interference",
   exactly as it did when the village world first lacked
   `<spherical_coordinates>`.
2. **Lockstep timing.** PX4 waits for every `HIL_SENSOR`. If the Python
   bridge plus rendering cannot keep up, the simulation slows; it does not
   break, but the controller's 1.0 s wall-clock telemetry timeout applies
   (risk R1 in `GAZEBO_READINESS_AUDIT.md`). Render cameras on their own
   schedule (e.g. 15–30 Hz of sim time), not every physics step.

## 6. What is reused, adapted, replaced

| Unchanged | Needs a MuJoCo adapter | Replaced / new |
|---|---|---|
| `flight_controller.py`, `mission_manager`, `mission_logic`, `mission_language`, interpreters | trial script: new `scripts/mujoco/run_mujoco_trial.sh`, modelled on `run_flight_trial.sh` (start bridge instead of gz, same logs) | MJCF vehicle model |
| debris depth detector, tracker, predictor, risk, avoidance planner | `make_village.py`: MJCF writer | `mujoco_bridge.py`: physics stepping, HIL MAVLink, sensors, ROS camera/lidar/clock publishing, `poses.csv` |
| survivor perception, confirmation, VLM client, localisation | debris release: `qpos`-based version of `spawn_study.py` | camera view presets (replacing `view.sh`) |
| search planner, navigator, `slam_bridge`, `slam_toolbox` | | |
| `verify_ulog.py`, all scorers (`evaluate_slam.py`, `summarize_runs.py`, `evaluate_mission.py`, ...) | | |
| XRCE agent, GCS heartbeat, PX4 build | | |

## 7. Testing the port against the Gazebo baseline

Run the same mission files with the same seeds, and score both with the same
scripts. A port step passes only when its number is inside the band:

| Test | Gazebo baseline (evidence) | Accept in MuJoCo |
|---|---|---|
| Hover 5 m, 30 s | altitude RMSE 0.070–0.096 m, all log checks (`phase0`) | all `verify_ulog.py` checks pass; RMSE ≤ 0.15 m |
| Waypoint square | Phase 1 waypoint checks | same checks |
| Takeoff time, landing offset | per-run `verification.json` | within ±30 % / ≤ 0.5 m |
| Camera rates | 30 Hz (NVIDIA) | ≥ 15 Hz sim time |
| Debris direct hits on a hovering UAV | 15/15 avoided (`phase6`) | measure; report |
| Survivor detector | recall 1.00, precision 0.72 (`phase7`) | measure on MuJoCo frames; report, do not tune silently |
| Survivor localisation | 0.05–0.2 m | ≤ 0.5 m |
| Integrated mission (small world) | 12/12 complete (`phase13`) | 5/5 complete |
| SLAM (village) | search RMSE 0.11–0.57 m | measure with `evaluate_slam.py` |

Differences are reported as differences. They are not "fixed" by changing
thresholds in the autonomy code.

## 8. Installation (user space only, no sudo, no driver changes)

```bash
python3 -m venv --system-site-packages ~/venvs/mujoco
```
```bash
~/venvs/mujoco/bin/pip install mujoco==3.16.0
```

`--system-site-packages` keeps rclpy (ROS 2 Jazzy) and pymavlink visible.
Rendering:

- `MUJOCO_GL=egl` needs the NVIDIA driver, i.e. kernel 7.0.0-30.
- On Intel, `MUJOCO_GL=glfw` with a display, or `osmesa`. `osmesa` would need
  the `libosmesa6` package, which is not checked here and would need the
  owner's sudo.

Size: the wheel is 30.6 MB (downloaded to the session scratch folder during this audit to check that it exists; not installed). Nothing else is
needed: numpy, OpenCV, pymavlink, rclpy and px4_msgs are already present.

## 9. Implementation order

| Step | Work | Done when |
|---|---|---|
| M0 | venv + `mujoco`; render one offscreen frame (EGL or GLFW) | a PNG from a test scene |
| M1 | MJCF x500 + small-world geometry; open-loop thrust test without PX4 | hover thrust ≈ 59 % of maximum, matching ω_hover ≈ 770 rad/s |
| M2 | `mujoco_bridge.py` HIL link; PX4 `10016` + x500 parameters; arm, takeoff, hover, land with the **unchanged** flight controller | `verify_ulog.py` passes 3/3 hover flights |
| M3 | `/clock`, RGB + depth cameras on the existing topics; `poses.csv` | camera rates and intrinsics match; scorers run |
| M4 | survivor manikin + world conversion; survivor chain on MuJoCo frames | detector numbers measured and reported |
| M5 | debris bodies + release; debris chain | direct-drop avoidance measured |
| M6 | integrated small-world mission from a sentence | 5 repeats scored against Phase 13 |
| M7 | lidar via `mj_multiRay`; village MJCF; SLAM + navigator | SLAM and mission scores against the village baseline |

M0–M2 is the critical path. If M2 does not fly reliably, stop there and
report; do not build M3–M7 on top of it.

## 10. Known limitations and risks

- Python bridge speed in lockstep (1 kHz physics, 250 Hz HIL, camera
  rendering) is the main technical risk. If it is too slow: render at a
  lower rate, use `mj_step` substeps, or move the HIL loop to C++ later.
- MuJoCo rendering differs from Gazebo, so perception results are not
  comparable one to one (section 3).
- No built-in sensor-noise or wind plugins: noise is written in the bridge.
- Contact physics differs (soft contacts), which matters for landing and box
  impacts.
- Runtime body creation is not possible (debris pool).
- The VLM on Intel/CPU is too slow for live use. That holds in both
  simulators, so live VLM needs the NVIDIA kernel.
- A faculty demo in MuJoCo is realistic only after M6. Before that, the
  honest demo is "PX4 flying in MuJoCo through the same controller" (M2–M3).
