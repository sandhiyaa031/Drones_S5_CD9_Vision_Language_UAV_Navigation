# Metrics summary (Phases 0–7)

Every number is copied from an existing result file or phase document; the
source is given in the last column (paths relative to `ros2_ws/`). All
results are from PX4 SITL + Gazebo simulation. Ground truth was used only
offline, for scoring. Nothing here was re-measured or recomputed for this
summary except the unit-test count.

## Flight foundation (Phase 0)

| Metric | Value | Source |
|---|---|---|
| Autonomous hover flights passing every log check | 5 of 5 (final configuration) | `results/phase0/final_config/hover_0*/verification.json` |
| Takeoff time to 3 m | 6.69 – 7.27 s | same |
| Hover altitude RMSE | 0.070 – 0.096 m | same |
| Max horizontal drift during the 30 s hold | 0.042 – 0.059 m | same |
| Landing offset from the takeoff point | 0.015 – 0.059 m | same |
| Time inside the altitude band | 31.4 – 32.2 s | same |
| Stock waypoint route in this world | **failed**: roof collision 8.66 s after arming | `results/phase0/final_config/waypoint_01` |
| Safety monitor: roof collision detected | impact at 8.6 s, PX4 AUTO LAND at 8.7 s | `results/safety/waypoint_with_monitor`, `docs/flight_safety.md` |

## Environment and sensors (Phase 1, sensor study, sensor integration)

| Metric | Value | Source |
|---|---|---|
| Real-time factor, old world | 0.41 (rejected) | `results/phase0/hover_01`, `docs/runbook.md` |
| Real-time factor, final two-camera configuration (NVIDIA, 320 s) | 0.9986 (0.997 – 1.000 per minute) | `results/sensor_integration/v09_benchmark_nvidia/benchmark.json` |
| Gazebo camera rate, depth and mono | 30.2 – 30.3 Hz each | same |
| ROS image rate in that benchmark, depth / mono | 17.8 – 21.1 Hz / 14.7 – 16.0 Hz | same |
| Gazebo server memory growth over the benchmark | 3.6 MiB | same |
| Real-time factor on the Intel GPU (no NVIDIA driver) | 0.90 – 0.94, cameras 27.3 – 28.3 Hz (below target) | `docs/sensor_integration.md` |
| Downward camera | 1280 × 960, `rgb8`, 99.7° HFOV | `docs/sensor_integration.md` |
| Upward depth camera | 640 × 480, 100° HFOV, f = 268.51 px, far clip 35 m | same |
| Depth error on flat targets at 2 / 5 / 10 / 15 m (median) | 1e-6 / 2e-6 / 5e-6 / 8e-6 m (the simulated sensor has no noise) | same |
| Falling boxes reconstructed from depth, box-centre error (median / max) | 13 – 22 mm / 41 mm | same |
| Sensor study: releases analysed | 20 of 24 | `results/sensor_study/README.md` |

## Time synchronisation

| Metric | `UXRCE_DDS_SYNCT=1` (PX4 default) | `UXRCE_DDS_SYNCT=0` (adopted) | Source |
|---|---|---|---|
| PX4 stamp − Gazebo time | ≈ 1.79e9 s, drifting | 0.000 s | `docs/sensor_integration.md`, `results/time_sync/` |
| Drift over a 48 s hover | 6.0 s | 0 | same |
| Timing error p50 / p95 / p99 (hover) | 2.1 / 3.9 / 3.9 s | 0 / 0 / 0 ms | same |
| Nearest odometry sample to a depth frame | – | p50 4 ms, max 8 – 12 ms | same, `benchmark.json` |

## Debris detection (Phase 2)

| Metric | Value | Source |
|---|---|---|
| Objects detected at least once | 86 of 86 that entered the field of view | `docs/debris_perception.md`, `results/phase2/summary.json` |
| Frame-level recall | 98.7 % (1713 of 1736 object-frames); per run 0.982 – 1.000 | same |
| Detections with no true object within 0.35 m | 0 of 1852 | same |
| Surface error, local frame, p95 / max | 0.000 – 0.160 m / 0.076 – 0.244 m per run | same |
| Detection point to box centre (it is the visible surface) | median 0.16 m | same |
| Processing time per frame, median / max | 2.5 – 7.7 ms / 13 – 28 ms | same |
| Detector frame rate (simulation time) | 23.9 – 24.5 Hz | same |

## Debris tracking (Phase 3)

| Metric | Value | Source |
|---|---|---|
| Position RMSE to the true surface | 0.000 – 0.073 m (7 live runs) | `docs/debris_tracking.md`, `results/phase3/summary_live.json` |
| Position RMSE to the true centre | 0.170 – 0.308 m | same |
| Velocity RMSE | 0.07 – 0.43 m/s | same |
| Vertical velocity bias (median) | −0.04 to +0.03 m/s | same |
| Vertical velocity bias without gravity in the model | −1.11 to −1.14 m/s | same |
| ID switches (CLEAR MOT) | 0 in all 7 runs | same |
| False confirmed tracks | 0, 0, 0, 1, 2, 0, 0 (duplicates of merged objects) | same |
| Objects confirmed / seen | 8/8, 7/8, 4/8 (lateral), 29/29, 25/26, 26/26, 10/10 | same |
| Confirmation delay | median 0.07 – 0.18 s, worst 0.50 s | same |
| Processing time per frame | median 0.12 – 0.73 ms, max 2.7 ms | same |

## Trajectory prediction (Phase 4)

Position RMSE to the true surface, objects still in free fall
(`docs/debris_prediction.md`, `results/phase4/`):

| Horizon | n (live) | Ballistic, live | Constant velocity | Constant acceleration |
|---|---|---|---|---|
| 0.1 s | 1956 | 0.028 m | 0.027 m | 0.030 m |
| 0.25 s | 1700 | 0.036 m | 0.057 m | 0.114 m |
| 0.5 s | 1281 | 0.056 m | 0.841 m | 0.614 m |
| 0.75 s | 831 | 0.064 m | 2.34 m | 1.42 m |
| 1.0 s | 511 | 0.110 m | 4.44 m | 2.36 m |
| 1.5 s | 213 | 0.160 m | 10.5 m | 4.68 m |
| 2.0 s | 41 | 0.414 m | 19.1 m | 10.8 m |

(The two right-hand columns are the offline replay of the same recorded
detections.) Vertical velocity bias of the ballistic model: within
±0.03 m/s at every horizon. Processing: median 0.12 ms per frame.

## Collision risk (Phase 5)

| Metric | Free fall | Whole life | Source |
|---|---|---|---|
| Offline, 16 recorded flights, 281 objects: precision | 0.61 | 0.67 | `docs/debris_risk.md`, `results/phase5/offline_recorded_hover.json` |
| recall | 0.954 | 0.79 | same |
| false-positive rate | 0.181 | 0.169 | same |
| false-negative rate | 0.046 | 0.209 | same |
| True collisions reaching CRITICAL | 10 of 10 | 13 of 18 | same |
| Live scenarios A–D (43 objects, whole life) | – | precision 0.81, recall 0.87 | `results/phase5/live_all_hover.json` |

Warning lead time (first WARNING/CRITICAL → true closest approach, free
fall, 62 objects): median 0.80 s, 5th percentile 0.45 s, minimum 0.43 s,
maximum 2.04 s. Collisions only: median 1.11 s, minimum 0.78 s. Processing:
median 0.32 – 0.40 ms per frame.

## Debris avoidance (Phase 6, final batch of 14 scenarios)

| Metric | Without avoidance | With avoidance | Source |
|---|---|---|---|
| Runs with a collision | 11 of 14 | 5 of 14 | `results/phase6/README.md`, `comparison.json` |
| Missions completed | 3 of 14 | 10 of 14 | same |
| Collisions / releases | 15 / 42 (a flight ends at its first hit) | 6 / 52 | same |
| Direct hits on a hovering UAV avoided | – | 15 of 15, smallest separation 0.91 m | same |
| Non-threatening boxes hit because of a manoeuvre | – | 0 of 22 | same |
| Unnecessary manoeuvres | – | 1 of 25 | same |

| Manoeuvre measurement (25 manoeuvres) | Median | Range | Source |
|---|---|---|---|
| Warning lead time | 1.30 s | 0.50 – 1.46 s | `results/phase6/README.md` |
| Start latency (first alert frame → controller active) | 0.068 s | 0.036 – 0.104 s | same |
| Manoeuvre duration | 1.18 s | 0.48 – 1.94 s | same |
| Recovery to the mission setpoint | 1.94 s | 0.02 – 2.45 s | same |
| Deviation from the mission setpoint (hover) | 3.08 m | 0.20 – 3.43 m | same |
| Max speed / max acceleration (no impact) | 3.0 m/s / 6.1 m/s² | up to 4.38 m/s / 6.7 m/s² | same |
| Planner compute time per cycle | 8.8 ms | worst 20.1 ms | same |
| Command rate | 44.2 Hz | 40.5 – 45.5 Hz | same |

Every collision in that batch had a lead time ≤ 0.70 s; every threat with
≥ 0.94 s was avoided.

## Phase 6.1 (bounded results)

| Metric | Value | Source |
|---|---|---|
| Steady cruise at 2.9 / 3.5 / 4.0 / 4.4 m/s, boxes aimed at the UAV | 0 collisions in 8 flights, 19 releases; min separation 1.91 m; lead 1.26 – 1.40 s | `results/phase6_1/README.md` |
| Hard-acceleration leg (6 m setpoint step, ≈ 6 m/s²) | **not solved**: collision in 1 of 2 flights (Phase 6: 3 of 5) | same |
| Warning lead with constant-velocity / constant-acceleration / response-model UAV motion (replay) | 0.73 – 0.80 s / 0.71 – 0.75 s / 0.47 – 0.73 s (no improvement) | same |
| Lateral throws avoided, by flight time | 1.3 s: 4/4; 1.1 s: 4/4; 1.0 s: 0/3; 0.93 s: 0/1 (0/5 over both phases) | same |
| Minimum warning lead for success (44 hover manoeuvres) | ≈ 0.9 s (no collision at ≥ 0.92 s; always at ≤ 0.71 s) | same |
| Injected depth gap from the moment of release | 100 ms, 250 ms: avoided 3/3; 500 ms: 0 collisions, min 0.64 m; 750 ms: collision | same |
| Injected depth gap after the manoeuvre started (500, 750 ms) | avoided 3/3 each, ≥ 1.82 m | same |
| Seeded random runs (24 releases) | 0 collisions (Phase 6: 1) | same |
| Response model error up to 1.2 s ahead | 0.24 m hover / 0.28 m transit (before refit: 0.80 m) | `results/phase6_1/model/fit_summary.txt` |

## Estimator diagnostic (one paired flight)

| Metric | GPS delay 110 ms (PX4 default) | GPS delay 0 ms | Source |
|---|---|---|---|
| Actual latency of the simulated GPS data | 0 ± 5 ms (9 flights) | – | `results/phase6_1_estimator_diagnostic/README.md` |
| Position estimate lead | 86.6 ms | 10.0 ms | same |
| Horizontal position error, median / max | 0.296 / 0.400 m | 0.086 / 0.162 m | same |
| Tilt error on the leg, p95 / max | 3.40 / 3.46° | 3.40 / 3.44° | same |
| Sensor-to-object vector error, max | 0.452 m | 0.461 m | same |
| False debris lateral velocity, max | 1.231 m/s | 1.081 m/s | same |
| Warning lead time | 0.764 s | 0.788 s | same |
| Outcome (planner disabled) | collision | collision | same |

Conclusion: the GPS-delay setting explains the position error only; it does
not change the warning time or the outcome. Configuration left unchanged.

## Survivor perception + VLM (Phase 7, final run, 11 flights)

| Metric | Value | Source |
|---|---|---|
| Candidate detector, live: TP / FN / FP | 2016 / 0 / 773 over 5983 frames | `results/phase7/README.md`, `summary_vlm.json` |
| Candidate detector precision / recall / F1 | 0.7228 / 1.0000 / 0.8391 | same |
| Same, replay on recorded frames | 0.7120 / 1.0000 / 0.8318 | same |
| False positives outside the two distractor scenarios | 0 (rubble, roof, shadows, debris, empty scene) | same |
| False-positive rate per frame / on frames without a survivor | 0.129 / 0.228 | same |
| Detector hit rate by survivor size | ≥ 45 px: 100 %; 30–45 px: 47 %; 20–30 px: 12 %; < 20 px: 0 % | `results/phase7/size_study_detector.json` |
| Temporal confirmation delay | median 0.13 – 0.30 s | `results/phase7/README.md` |
| VLM alone, held-out crops: clear survivors confirmed | 24 of 45 (20 rejected, 1 uncertain) | `results/phase7/vlm_eval_heldout_gemma3_4b.json` |
| VLM alone: non-survivors rejected | 98 of 100 (0 confirmed, 2 uncertain) | same |
| VLM alone: oblique view / straight-down view confirmed | 24 of 32 / 0 of 13 | same |
| Survivors reaching SURVIVOR_CONFIRMED in live flights | 4 of 8 | `results/phase7/README.md` |
| Frames in SURVIVOR_CONFIRMED on a survivor / on a non-survivor | 1146 / 0 | same |
| Published output (CANDIDATE or CONFIRMED) precision / recall / F1 | 0.8952 / 0.7882 / 0.8383 | same |
| VLM latency, live on GPU (65 requests, 0 errors) | p50 2.36 s, p95 2.55 s, max 2.84 s | same |
| VLM latency, CPU only | 44 s and 82 s in two runs | same |
| First in view → SURVIVOR_CONFIRMED | 2.5 – 4.2 s | same |
| Camera callback time | p50 2.1 – 3.9 ms, max 15 ms | same |
| End-to-end perception latency (camera stamp → message received) | p50 14 – 19 ms, worst p95 28 ms | same |
| Without a VLM: confirmations | 0 in 11 flights (status VLM_UNAVAILABLE throughout) | `results/phase7/summary_no_vlm.json` |

## Tests

| Test file | Tests |
|---|---|
| `test_flight_safety.py` | 26 |
| `test_debris_detection.py` | 21 |
| `test_debris_tracking.py` | 42 |
| `test_debris_prediction.py` | 24 |
| `test_debris_risk.py` | 35 |
| `test_debris_avoidance.py` | 75 |
| `test_survivor_perception.py` | 92 |
| **Total** | **315, all passing** (re-run on 2026-10-06) |

## Not measured

No metric exists yet for survivor 3D localisation, search coverage, mission
completion, or natural-language mission interpretation: Phases 8–15 have not
been started.
