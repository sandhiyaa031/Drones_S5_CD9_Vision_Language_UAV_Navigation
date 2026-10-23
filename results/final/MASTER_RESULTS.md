# Master results: Phases 0–7

Status date: 2026-10-06. Paths are relative to `ros2_ws/` unless they start
with `./` (this folder). Every number is taken from the phase documents and
result files named beside it; nothing was re-measured for this document.

---

## 1. Project title

**Vision-Language UAV Navigation — autonomous UAV disaster-rescue system in
simulation** (Semester 5 project, repository
`Drones_S5_CD9_Vision_Language_UAV_Navigation`).

Note on the repository's subtitle "GPS-denied": the system built so far uses
PX4's EKF2 with a simulated GNSS receiver. GPS-denied navigation is not
implemented and is not claimed.

## 2. Project objective

One integrated UAV system that, given a mission in natural language, takes
off, flies to a collapsed building, searches it, finds and confirms a
survivor, reports the survivor's location, and stays safe from falling debris
throughout.

**State today:** the flight foundation, the debris-safety chain (see → track
→ predict → assess → avoid) and survivor perception with VLM verification
are built and validated in simulation (Phases 0–7). Survivor localisation,
search, mission logic, the natural-language interface and the integrated
mission (Phases 8–15) are **not started**.

## 3. Problem statement

After a structural collapse a small UAV can search where people cannot go,
but two things stand in the way of autonomy: the environment is still
dangerous (debris keeps falling), and "is that a survivor?" is a semantic
question that simple detectors answer badly. The project addresses both
with (a) an onboard perception-to-avoidance chain for dynamic debris, and
(b) a deterministic visual candidate detector checked by a vision-language
model, under one flight-control authority.

## 4. Overall system architecture

See [`ARCHITECTURE.md`](ARCHITECTURE.md) for the diagram, node table and
topic table. In short:

```
upward depth camera → detector → tracker → predictor → risk → avoidance planner ─┐ (proposal)
downward RGB camera → survivor detector → temporal confirmation → VLM            ▼
PX4 odometry ───────────────────────────────────────────────────────► flight_controller → PX4
```

Design rules that hold everywhere:

* only `flight_controller` commands PX4;
* no node in flight reads simulator ground truth;
* PX4 source, safety parameters and preflight checks are untouched;
* every capability is demonstrated live in Gazebo and scored offline against
  ground truth; failures are kept as evidence.

## 5. Software / hardware stack

Ubuntu 24.04.4, ROS 2 Jazzy, Gazebo Sim (Harmonic) 8.11.0, PX4-Autopilot
`d983b64` (SITL), px4_msgs `f719702`, Micro XRCE-DDS agent, Python 3.12
(numpy, scipy, OpenCV), Ollama 0.35.1 with `gemma3:4b`. Laptop: Core
i7-14650HX, 15 GB RAM, RTX 4050 (6 GB) + Intel iGPU. Full list:
[`REPRODUCTION.md`](REPRODUCTION.md).

## 6. Simulation environment

World `collapsed_building_rescue` (project-owned SDF, all static): three
8 m long, 3 m high outer walls and an interior wall, four collapsed roof slabs at about 3 m
leaving an opening of roughly 3.1 × 2 m, fallen concrete and rubble blocks,
and a survivor manikin standing at (1.2, 0) m inside. The UAV starts inside,
under the opening. Debris is spawned as physical boxes (0.25–0.5 m), given
one impulse and never moved afterwards. Phase 7 adds generated world
variants (occluding panel, rubble, distractor objects, outdoor survivor, no
survivor).

## 7. UAV model

`x500_rescue`: PX4's X500 quadrotor with two project-mounted cameras. It is
spawned by the trial script and PX4 attaches to it with the stock `gz_x500`
airframe (`PX4_GZ_STANDALONE=1`). No PX4 model file is modified.

## 8. Sensors and camera configuration

| Sensor | Resolution, rate | Field of view | Purpose |
|---|---|---|---|
| Downward RGB camera | 1280 × 960, 30 Hz | 99.7° horizontal | survivor search |
| Upward depth camera (zenith) | 640 × 480, 30 Hz, 32-bit metres, 0.2–35 m | 100° horizontal | falling debris |
| PX4 IMU, barometer, magnetometer, GNSS (simulated) | – | – | state estimation (EKF2) |

The upward depth camera was chosen from a measured sensor study
(`results/sensor_study/`): a downward camera cannot see debris in time.

## 9. PX4 + ROS 2 + Gazebo architecture

Gazebo runs the physics and sensors in lockstep with PX4 SITL. PX4 talks to
ROS 2 through the Micro XRCE-DDS agent (`/fmu/out/*`, `/fmu/in/*`). Camera
images reach ROS through `ros_gz_bridge`. A GCS heartbeat script satisfies
PX4's data-link check so that arming is allowed without changing a
parameter. The project's own Gazebo server configuration omits PX4's
`GstCameraSystem` plugin, which leaked memory at 107 MiB/s.

## 10. Time synchronisation approach

Measured problem: with PX4's default (`UXRCE_DDS_SYNCT=1`) PX4 timestamps on
ROS are wall-clock based and drift from simulation time by seconds (6.0 s
over a 48 s hover). Adopted: `UXRCE_DDS_SYNCT=0` per run, restored
afterwards. Result: PX4 time equals Gazebo time, error 0 ms at
p50/p95/p99 during hover. Sensor frames are paired with
odometry interpolated at `timestamp_sample`. This is a simulation time base,
not a real-hardware clock-synchronisation method.
(`docs/sensor_integration.md`, `results/time_sync/`.)

## 11. Flight-control architecture

`flight_controller.py`: an offboard state machine (WAIT_FOR_DATA → PRESTREAM
→ REQUEST_OFFBOARD → WAIT_PREFLIGHT → REQUEST_ARM → TAKEOFF → HOLD or
WAYPOINT_1/2/RETURN_HOME → REQUEST_LAND → LANDING → COMPLETE; any failure →
FAILSAFE → PX4 AUTO LAND). Position setpoints only. A flight safety monitor
checks every position sample. Avoidance proposals and speed caps are
validated here. Details in [`ARCHITECTURE.md`](ARCHITECTURE.md).

---

## 12. Phase 0 — Flight foundation

| | |
|---|---|
| **Objective** | A repeatable autonomous flight: cold start, arm, take off, hold, land, with proof from the PX4 log |
| **Problem solved** | Arming was blocked (no GCS link); the old world ran at real-time factor 0.41; flights were not verified by anything but the controller's own log |
| **Implementation** | One-command trial runner; GCS heartbeat; PX4-log verifier; flight safety monitor (additive, 39 lines in the controller) |
| **Source** | `flight_controller.py`, `flight_safety.py`, `scripts/run_flight_trial.sh`, `scripts/gcs_heartbeat.py`, `scripts/verify_ulog.py` |
| **Topics** | in: `/fmu/out/vehicle_local_position_v1`, `vehicle_status_v4`, `vehicle_command_ack_v1`; out: `/fmu/in/offboard_control_mode`, `trajectory_setpoint`, `vehicle_command` |
| **Algorithm** | state machine with explicit timeouts; safety checks (impact > 30 m/s², position/velocity discontinuity, speed envelope, estimator reset, stale telemetry > 1 s) → FAILSAFE → AUTO LAND |
| **Tests** | 26 unit tests, including replays of recorded logs at 62/42/25/12 Hz |
| **Live validation** | 5/5 hover flights pass every log check (final configuration) |
| **Metrics** | takeoff 6.69–7.27 s; hover altitude RMSE 0.070–0.096 m; drift 0.042–0.059 m; landing offset 0.015–0.059 m |
| **Successful** | hover at 3 m, 30 s hold, landing at the takeoff point |
| **Failure** | the stock waypoint route hit the roof 8.66 s after arming (kept as evidence); the monitor later caught the same collision live at 8.6 s and landed at 8.7 s |
| **Limits** | detection is after contact; a crashed vehicle may stay armed (no auto-disarm, by decision) |
| **Status** | **DONE** |
| **Evidence** | `results/phase0/`, `results/safety/`, `docs/runbook.md`, `docs/flight_safety.md` |

## 13. Phase 1 — Environment and sensors

| | |
|---|---|
| **Objective** | A simulation world and sensor set that run in real time and give trustworthy data |
| **Problem solved** | slow, leaking simulation; no depth sensing; unknown camera geometry; PX4 and sensor clocks disagreeing |
| **Implementation** | clean static world; project Gazebo server config; sensor study; `x500_rescue` model with the upward depth camera; bridge config; time-base fix |
| **Source** | `worlds/collapsed_building_rescue.sdf`, `models/x500_rescue`, `models/depth_cam_up`, `config/gz_server.config`, `config/camera_bridge_rescue.yaml`, `scripts/measure_sim.py`, `scripts/benchmark_two_camera.sh`, `scripts/sensor_study/`, `scripts/sensor_integration/` |
| **Topics** | `/camera/image_raw`, `/camera/camera_info`, `/depth_up/image_raw`, `/depth_up/camera_info` |
| **Algorithm / equations** | pinhole model `u = cx − fx·Y/X`, `v = cy − fy·Z/X`; axial depth; back-projection ([`MATHEMATICAL_METHODS.md`](MATHEMATICAL_METHODS.md) §2) |
| **Live validation** | 320 s benchmark on NVIDIA: real-time factor 0.9986, both cameras 30.2–30.3 Hz, memory +3.6 MiB, timestamp error 0 ms |
| **Metrics** | depth error on flat targets ≤ 1e-5 m (no sensor noise is simulated); falling-box centre error 13–22 mm median, 41 mm max; 21 of 24 falling boxes reconstructed; no rotor or body pixels in 501 hover frames |
| **Sensor study** | 3 flights, 20 of 24 releases analysed → decision: keep the downward camera for survivors, add an upward ~100° depth camera for debris |
| **Failure / limits** | performance target needs the NVIDIA driver (Intel GPU 0.90–0.94); depth has no noise model; the zenith camera does not see low, lateral threats |
| **Status** | **DONE** |
| **Evidence** | `results/phase1/`, `results/sensor_study/`, `results/sensor_integration/`, `results/time_sync/`, `docs/sensor_integration.md` |

## 14. Phase 2 — 3D debris perception

| | |
|---|---|
| **Objective** | Detect falling debris in 3D from the upward depth camera, in the PX4 local frame |
| **Implementation** | connected components on the valid-depth mask, depth-jump split, size gates, back-projection, transform to local NED with pose interpolated at the image time, static-structure filter |
| **Source** | `debris_detection.py`, `debris_depth_detector.py` |
| **Inputs** | `/depth_up/image_raw`, `/depth_up/camera_info`, `/fmu/out/vehicle_odometry` |
| **Outputs** | `/perception/debris/detections` (sensor frame), `detections_local` (NED), `debug_image`, `status` |
| **Equations** | [`MATHEMATICAL_METHODS.md`](MATHEMATICAL_METHODS.md) §2–4 |
| **Tests** | 21 unit tests on synthetic depth scenes |
| **Live validation** | 4 hover flights, 26–30 boxes each from 6–13 m with up to 3 m/s, several airborne together |
| **Metrics** | 86/86 objects detected; frame recall 98.7 % (1713/1736); 0 unexplained detections of 1852; local-frame surface error p95 ≤ 0.160 m, max 0.244 m; 2.5–7.7 ms per frame; 24 Hz |
| **Failures** | overlapping boxes at the same range merge (all missed frames; one first detection 0.33 s late); 113 border-cut detections with biased position; a low lateral throw that approaches below the field of view is never detected; a box struck the UAV in one run (nothing avoided debris yet) |
| **Limits** | no sensor noise in simulation; visible surface, not centre (≈ 0.16 m); depth stream gaps up to 0.53 s |
| **Status** | **DONE** |
| **Evidence** | `results/phase2/`, `docs/debris_perception.md`, video `results/demo/detector_view_seed11.mp4` |

## 15. Phase 3 — Debris tracking

| | |
|---|---|
| **Objective** | Persistent identity and velocity for every falling object |
| **Implementation** | 6-state Kalman filter with gravity as a known input; Mahalanobis gate (16.27) plus a reach gate; Hungarian assignment; TENTATIVE / CONFIRMED / COASTING life cycle; special handling of border-cut and merged measurements |
| **Source** | `debris_tracking.py`, `debris_tracker.py`; message `DebrisTrack(Array)` |
| **Inputs / outputs** | in `/perception/debris/detections_local`; out `/perception/debris/tracks`, `track_markers`, `tracks_debug` |
| **Equations** | state `x = [p, v]`, `p' = p + v dt + ½ g dt² e_z`; KF predict/update; `cost = yᵀS⁻¹y` ([`MATHEMATICAL_METHODS.md`](MATHEMATICAL_METHODS.md) §5) |
| **Tests** | 42 unit tests |
| **Live validation** | 7 flights (single objects, heights/speeds, lateral throws, three seeded runs of 26–29 objects, UAV climbing) |
| **Metrics** | position RMSE to true surface 0.000–0.073 m (to centre 0.170–0.308 m); velocity RMSE 0.07–0.43 m/s; vertical bias −0.04…+0.03 m/s (without gravity: −1.1 m/s); **0 ID switches**; confirmation delay median 0.07–0.18 s; ≤ 2.7 ms per frame |
| **Failures** | 3 duplicate confirmed tracks from merged objects; border-only objects stay tentative (4 of 8 lateral); coasting after impact gives 6–15 m/s velocity error for up to 6 frames |
| **Limits** | no drag/bounce; covariance conservative, not calibrated |
| **Status** | **DONE** |
| **Evidence** | `results/phase3/`, `docs/debris_tracking.md` |

## 16. Phase 4 — Trajectory prediction

| | |
|---|---|
| **Objective** | Where will each tracked object be over the next 2 s, with uncertainty |
| **Implementation** | three models compared on identical recorded data; ballistic chosen; predictions at 0.1, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0 s with propagated covariance; only for confirmed or freshly coasting tracks |
| **Source** | `debris_prediction.py`, `debris_predictor.py`; messages `PredictedState`, `DebrisPrediction(Array)` |
| **Inputs / outputs** | in `/perception/debris/tracks`; out `/perception/debris/predictions`, markers, debug |
| **Equations** | `p(t+h) = p + v h + ½ a h²`, `a = (0, 0, 9.8)`; `P(h) = F P Fᵀ + Q` |
| **Tests** | 24 unit tests |
| **Live validation** | 6 hover flights, 2346 predictions scored; one flight with the UAV climbing |
| **Metrics** | RMSE to true surface: 0.028 m @0.1 s, 0.056 m @0.5 s, 0.110 m @1.0 s, 0.160 m @1.5 s, 0.414 m @2.0 s. Constant velocity: 4.44 m @1.0 s; constant acceleration: 2.36 m @1.0 s. Velocity bias within ±0.03 m/s. 0.12 ms per frame |
| **Failures / limits** | only 41 of 2108 predictions at 2.0 s could be scored (the object had already hit something): the predictor has no map and predicts through structure; free fall only |
| **Status** | **DONE** |
| **Evidence** | `results/phase4/`, `docs/debris_prediction.md`, `./figures/06_…`, `./figures/07_…` |

## 17. Phase 5 — Collision-risk estimation

| | |
|---|---|
| **Objective** | For each tracked object: will it enter the UAV's safety volume, when, and how sure |
| **Implementation** | dense closest-approach search along the predicted path against the UAV's motion; contact and safety radius; capped uncertainty margin; states SAFE / WATCH / WARNING / CRITICAL; POTENTIAL_THREAT for unconfirmed fast objects. Advisory only |
| **Source** | `debris_risk.py`, `debris_risk_node.py`; message `DebrisRisk(Array)` |
| **Inputs / outputs** | in tracks, predictions, odometry; out `/perception/debris/risk`, `risk_markers`, `risk_debug` |
| **Equations** | contact radius 0.40 + 0.71·size; safety = contact + 0.30 m; margin = min(σ_r, 0.10 + 0.40 t); reaction time 0.75 s ([`MATHEMATICAL_METHODS.md`](MATHEMATICAL_METHODS.md) §7) |
| **Tests** | 35 unit tests (32 from Phase 5, 3 added in 6.1 for the motion models) |
| **Live validation** | scenarios: safe pass, near miss, direct hit, lateral hit, multiple, UAV climbing |
| **Metrics** | offline, 281 objects, free fall: precision 0.61, recall 0.954; 10 of 10 collisions CRITICAL. Whole life: precision 0.67, recall 0.79. Live A–D: precision 0.81, recall 0.87. Lead time median 0.80 s (5th percentile 0.45 s) |
| **Failures** | 15 of 18 whole-life misses were boxes deflected by structure; the lateral hit reached only POTENTIAL_THREAT and struck the UAV; ~18 % of safe objects flagged (passing 0.70–0.90 m) |
| **Limits** | constant-velocity UAV assumption; sphere safety volume; uncertainty is an empirical envelope |
| **Status** | **DONE** |
| **Evidence** | `results/phase5/`, `docs/debris_risk.md`, `./figures/08_collision_risk_traces.png` |

## 18. Phase 6 — Autonomous debris avoidance

| | |
|---|---|
| **Objective** | Move the UAV out of the way of a threatening object and resume the mission, without giving up the single PX4 authority |
| **Implementation** | planner proposes a position setpoint; 19 candidates evaluated on a measured vehicle-response model; hard constraints (altitude, speed, structure exclusion, clearance against every track); cost function; state machine NORMAL_FLIGHT / PREPARE_AVOIDANCE / AVOIDING / CLEARING / RESUME_MISSION / AVOIDANCE_FAILSAFE; controller validates and executes |
| **Source** | `debris_avoidance.py`, `debris_avoidance_node.py`, `avoidance_interface.py`, additive changes in `flight_controller.py`; messages `AvoidanceCommand`, `FlightStatus`; `scripts/avoidance/` |
| **Topics** | `/perception/debris/avoidance_command`, `/uav/flight_status`, `avoidance_markers`, `avoidance_debug` |
| **Equations** | response model, constraints, `J = 10·risk + 1.0·deviation + 0.5·effort + 0.5·change + 0.5·altitude` ([`MATHEMATICAL_METHODS.md`](MATHEMATICAL_METHODS.md) §8) |
| **Tests** | 75 unit tests (planner, constraints, state machine, controller rules) |
| **Live validation** | 14 scenarios, each flown without and with avoidance on identical releases; ablations; repeats |
| **Metrics** | runs with a collision 11/14 → 5/14; missions completed 3/14 → 10/14; **15/15 direct hits on a hovering UAV avoided** (min separation 0.91 m); 0/22 non-threats hit; 1 unnecessary manoeuvre of 25. Lead 1.30 s median; start latency 0.068 s; planner 8.8 ms; deviation 3.1 m; recovery 1.9 s |
| **Successful** | no debris, safe debris, near miss (no reaction), direct threats, multiple threats, climbing UAV, search-speed legs, two of three seeded runs |
| **Failures** | lateral throw (lead 0.62 s); transit under hard acceleration; late release (0.59 s lead); inside the roof opening (no feasible candidate, by design); one direct hit lost to a 0.76 s depth gap |
| **Limits** | needs ≈ 0.9 s of warning; deviations larger than necessary |
| **Status** | **BOUNDED / PARTIAL** — works inside the measured envelope |
| **Evidence** | `results/phase6/` (README, `comparison.json`, plots, `superseded/`), `docs/debris_avoidance.md`, `./figures/09…12` |

## 19. Phase 6.1 — Robustness and diagnostics

| | |
|---|---|
| **Objective** | Address three blockers: fast transit, short lateral throws, stale perception |
| **Implementation** | optional UAV motion models in risk (constant acceleration, response model); response model refitted with PX4's measured acceleration; perception health FRESH / DEGRADED / STALE with a held risk frame and a DEFENSIVE state; speed caps applied by the controller; depth-gap injector for testing |
| **Source** | additions in `debris_risk.py`, `debris_risk_node.py`, `debris_avoidance.py`, `avoidance_interface.py`, `flight_controller.py`; `scripts/avoidance/depth_gate.py`, `replay_risk_models.py`, `fit_response_states.py` |
| **Live validation** | 40 recorded flights: acceptance re-run, cruise at four speeds, lateral series, injected gaps, motion-model comparison |
| **Metrics** | no regression (seeded runs now 0 collisions in 24 releases). **Cruise 2.9–4.4 m/s: 0 collisions in 8 flights**, min separation 1.91 m. Depth gap hiding a new object: fine to 250 ms, marginal at 500 ms (0.64 m), collision at 750 ms. Lateral: avoided at 1.1 s and 1.3 s of flight, not at 1.0 s or 0.93 s. Minimum warning ≈ 0.9 s. Response-model error 0.80 m → 0.25 m |
| **Negative results (reported)** | acceleration-aware UAV motion gives **no** earlier warning (0.73–0.80 s vs 0.71–0.75 s vs 0.47–0.73 s); hard-acceleration transit still collides (1 of 2); defensive preparation does not help the 0.93 s throw |
| **Estimator diagnostic** | PX4 relabels each simulated GPS fix as 110 ms old while its real latency is 0 ± 5 ms. One paired flight with the delay set to 0: position lead 86.6 → 10.0 ms, but tilt error (3.4°), sensor-to-object vector error (0.45 vs 0.46 m), warning lead (0.764 vs 0.788 s) and the collision outcome unchanged. Stopped there; configuration left at default |
| **Status** | **BOUNDED** — the envelope is measured; hard acceleration remains unsolved |
| **Evidence** | `results/phase6_1/`, `results/phase6_1_estimator_diagnostic/`, `./figures/13…14` |

## 20. Phase 7 — Survivor perception and VLM verification

| | |
|---|---|
| **Objective** | Detect a possible survivor with the downward camera and have a VLM decide whether it is one; states NO_TARGET / CANDIDATE / SURVIVOR_CONFIRMED / UNCERTAIN |
| **Problem solved** | the pre-existing "VLM" files contained no VLM (a colour threshold logged as a VLM alert), blocked the camera callback on HTTP, and published image coordinates as a 3D pose; all replaced |
| **Implementation** | deterministic colour/shape candidate detector (simulation baseline); temporal confirmation (5 of 8 frames); asynchronous VLM worker (one crop at a time, two requests: neutral description, then text-only decision; strict answer validation); honest VLM_AVAILABLE / VLM_UNAVAILABLE / VLM_ERROR; proper ROS messages; debug image |
| **Source** | `survivor_detection.py`, `survivor_confirmation.py`, `vlm_client.py`, `survivor_perception_node.py`; messages `SurvivorDetection(Array)`; `scripts/survivor/` |
| **Inputs** | `/camera/image_raw`, `/camera/camera_info` only |
| **Outputs** | `/perception/survivor/detections`, `debug_image`, `status` |
| **Equations** | HSV cue masks; `confidence = 0.30 + 0.35·head + 0.20·shape + 0.15·legs` ([`MATHEMATICAL_METHODS.md`](MATHEMATICAL_METHODS.md) §10). No camera-ray geometry online |
| **Tests** | 92 unit tests, including an audit that the node has no ground-truth input and a test that the camera callback never waits for the VLM |
| **Live validation** | 11 scenarios (clear, occluded, rubble, altitude 3–12 m, moving at 2.0 and 2.9 m/s, no survivor, falling debris, distractors, multiple), each flown with and without the VLM |
| **Detector metrics** | TP 2016, FN 0, FP 773 → precision 0.7228, **recall 1.0000**, F1 0.8391. All FP on one designed distractor cluster; 0 FP on rubble, roof, shadows, debris, empty scenes. Reliable down to 45 px |
| **VLM metrics (held-out, 175 crops)** | non-survivors: 98/100 rejected, **0 confirmed**; clear survivors: 24/45 confirmed, 20 rejected (oblique view 24/32, straight-down 0/13) |
| **Live with VLM** | 4 of 8 survivor instances SURVIVOR_CONFIRMED (2.5–4.2 s after entering view); 1146 confirmed frames on a survivor, 0 on a non-survivor; VLM latency p50 2.36 s; callback p50 2.1–3.9 ms |
| **Without VLM** | 0 confirmations, status VLM_UNAVAILABLE throughout (11 flights) |
| **Failures** | VLM rejects straight-down views; orange object touching a skin-coloured object scores as a strong candidate; 2 of 11 first-batch flights failed safe from GPU contention; CPU-only VLM 44–82 s |
| **Status** | **ACCEPTED SIMULATION BASELINE** — "Simulation survivor-perception baseline validated; real-world generalization is future work." |
| **Evidence** | `results/phase7/`, `docs/survivor_perception.md`, `./figures/15…18` |

---

## 21. Quantitative results

All tables with sources: [`METRICS_SUMMARY.md`](METRICS_SUMMARY.md).
Headline numbers:

| Capability | Result |
|---|---|
| Autonomous hover | 5/5 flights; altitude RMSE 0.070–0.096 m |
| Simulation | real-time factor 0.9986, cameras 30 Hz, time error 0 ms |
| Debris detection | 86/86 objects, 98.7 % frame recall, 0 false detections |
| Tracking | 0 ID switches, velocity bias < 0.05 m/s |
| Prediction | 0.11 m RMSE at 1 s (constant velocity: 4.44 m) |
| Risk | 10/10 true collisions flagged CRITICAL; lead time median 0.80 s |
| Avoidance | collision runs 11/14 → 5/14; 15/15 direct hits avoided when hovering |
| Survivor detector | recall 1.00, precision 0.72 |
| VLM | 0 false confirmations; 53 % of clear survivors confirmed |
| Tests | 315 passing |

## 22. Mathematical formulation

[`MATHEMATICAL_METHODS.md`](MATHEMATICAL_METHODS.md): frames, camera model,
back-projection, pose interpolation, Kalman filter with gravity, Mahalanobis
gating and Hungarian assignment, ballistic prediction, closest approach and
risk radii, vehicle-response model, avoidance constraints and cost, survivor
cue score. Each equation is marked implemented / not default / evaluation
only.

## 23. Failure cases

[`LIMITATIONS_AND_FAILURES.md`](LIMITATIONS_AND_FAILURES.md) lists every
failure with cause, fix and remaining status. The ones that matter most:

1. Avoidance needs ≈ 0.9 s of warning; below 0.7 s it failed every time.
2. Hard-acceleration transit is unsolved (measurement drift under
   acceleration; estimator change does not help).
3. Laterally thrown debris with under ~1 s of flight is not avoided.
4. Inside the roof opening there is no escape manoeuvre.
5. The VLM rejects survivors seen from straight above.
6. The survivor detector is specific to the manikin's colours.

## 24. Safety mechanisms

* Single PX4 command authority (`flight_controller`); planner output is a
  validated proposal with a 0.5 s timeout.
* Flight safety monitor: impact, estimator jump, speed envelope, stale
  telemetry → FAILSAFE → PX4 AUTO LAND. Never weakened in any phase.
* PX4 safety parameters and preflight checks unchanged.
* Structure exclusion zones in the planner; altitude and speed limits.
* Perception health: stale perception is not treated as "no debris".
* Survivor confirmation requires temporal stability **and** a VLM
  confirmation; without a VLM nothing is ever confirmed.
* Per-run PX4 parameter overrides are saved, verified and restored.

## 25. Current limitations

Simulation only; noise-free depth; GNSS-aided state estimation;
manikin-specific survivor detector; small local VLM; the two perception
chains not yet flown together; no localisation, search or mission logic.
Full list in [`LIMITATIONS_AND_FAILURES.md`](LIMITATIONS_AND_FAILURES.md).

## 26. Current project completion status

8 of 16 phases complete (0–7, with 6/6.1 bounded and 7 a simulation
baseline). The integrated rescue mission does **not** exist yet.
[`PHASE_STATUS.md`](PHASE_STATUS.md).

## 27. Remaining phases

| Phase | Work |
|---|---|
| 8 | Survivor 3D localisation from pixel detections, camera geometry and vehicle pose |
| 9 | Search planner over the building (with an oblique viewing geometry, which Phase 7 showed the VLM needs, and without hard accelerations, which Phase 6.1 showed avoidance needs) |
| 10 | Mission state machine tying flight, search, avoidance and confirmation together |
| 11 | Natural-language mission interpretation |
| 12 | End-to-end integrated mission |
| 13 | System evaluation |
| 14 | Failure testing |
| 15 | Demonstration and final documentation |

## 28. Reproduction instructions

[`REPRODUCTION.md`](REPRODUCTION.md) (versions, build, environment
variables, launch, tests) and [`GAZEBO_DEMO.md`](GAZEBO_DEMO.md) (exact demo
commands, per terminal).

## 29. Evidence inventory

| Phase | Design document | Results folder | Contents |
|---|---|---|---|
| 0 | `docs/runbook.md`, `docs/flight_safety.md` | `results/phase0/`, `results/safety/` | run folders with PX4 log, node logs, `verification.json` |
| 1 | `docs/sensor_integration.md` | `results/phase1/`, `results/sensor_study/`, `results/sensor_integration/`, `results/time_sync/` | measurements, benchmark, depth validation, sensor-study tables, plots and videos |
| 2 | `docs/debris_perception.md` | `results/phase2/` | detector output, evaluation JSON, debug frames |
| 3 | `docs/debris_tracking.md` | `results/phase3/` | tracker output, evaluation JSON |
| 4 | `docs/debris_prediction.md` | `results/phase4/` | predictions, model comparison, plots |
| 5 | `docs/debris_risk.md` | `results/phase5/` | risk output, offline and live scoring, plot |
| 6 | `docs/debris_avoidance.md` | `results/phase6/` | baseline and avoidance runs, comparison, plots, superseded batches |
| 6.1 | `docs/debris_avoidance.md` §3.12, `docs/debris_risk.md` | `results/phase6_1/`, `results/phase6_1_estimator_diagnostic/` | 38 flights, replay, model fit, A/B diagnostic |
| 7 | `docs/survivor_perception.md` | `results/phase7/` | live runs with and without VLM, VLM datasets and evaluation, size study, superseded batches |
| Package | – | `results/project_progress/` | this document set, `figures/`, `demo_check/` (two flights of 2026-10-06) |

Counts on disk (2026-10-06): 228 PX4 flight logs under `results/`
(`.ulg`; 226 before today plus the two demo checks); 315 unit tests;
9 current design documents in `docs/`; 4 recorded
videos (`results/demo/detector_view_seed11.mp4`,
`results/sensor_study/*/analysis/real_camera_annotated.mp4`).
