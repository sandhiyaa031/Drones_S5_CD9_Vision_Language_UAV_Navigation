# Project progress package — Phases 0 to 7

**Vision-Language UAV Navigation: autonomous UAV disaster-rescue system
(simulation).** Status date: 2026-10-06.

> **Read this first.** Phases 0–7 of 16 are complete. The integrated,
> end-to-end rescue mission does **not** exist yet. Everything here is PX4
> SITL + Gazebo simulation. Every number comes from a recorded flight or a
> result file in this repository, and failures are reported next to
> successes.

## Objective

A UAV that is given a mission in natural language, flies to a collapsed
building, searches it, finds and confirms a survivor, reports where the
survivor is, and stays safe from falling debris throughout.

## What works today

```
 upward depth camera ─► detect ─► track ─► predict ─► risk ─► avoidance planner ─┐ proposal
 (falling debris)       Ph.2      Ph.3     Ph.4       Ph.5    Ph.6 / 6.1         │
                                                                                 ▼
 downward RGB camera ─► survivor candidate ─► temporal check ─► VLM     flight_controller ─► PX4 ─► Gazebo
 (survivor)             Ph.7                                    (Ollama)  Ph.0 (only node that commands PX4)
                                                                                 ▲
 PX4 state estimate (EKF2, simulated IMU / baro / mag / GNSS) ───────────────────┘
```

| Phase | What | Status |
|---|---|---|
| 0 | Autonomous takeoff, hover, landing; flight safety monitor | DONE |
| 1 | Collapsed-building world, two cameras, time base | DONE |
| 2 | 3D debris detection (upward depth camera) | DONE |
| 3 | Debris tracking | DONE |
| 4 | Trajectory prediction | DONE |
| 5 | Collision-risk estimation | DONE |
| 6 / 6.1 | Autonomous debris avoidance | BOUNDED / PARTIAL |
| 7 | Survivor perception + VLM verification | ACCEPTED SIMULATION BASELINE |
| 8–15 | Survivor localisation, search, mission logic, language interface, integration, evaluation, failure testing, final demo | NOT STARTED |

## Headline results

| Capability | Result |
|---|---|
| Autonomous hover | 5 of 5 flights pass every log check; altitude RMSE 0.070–0.096 m |
| Simulation | real-time factor 0.9986, cameras 30 Hz (NVIDIA GPU); PX4-to-simulation time error 0 ms |
| Debris detection | 86 of 86 objects; frame recall 98.7 %; 0 false detections in 1852 |
| Tracking | 0 ID switches in 7 runs |
| Prediction | 0.110 m RMSE at 1 s (constant-velocity model: 4.44 m) |
| Collision risk | 10 of 10 true collisions flagged CRITICAL; median lead time 0.80 s |
| Avoidance | runs with a collision 11/14 → 5/14; 15 of 15 direct hits on a hovering UAV avoided; steady cruise to 4.4 m/s, 0 collisions in 8 flights |
| Survivor detector | recall 1.00, precision 0.72 (all false positives on one designed distractor) |
| VLM verification | 0 false confirmations; 24 of 45 clear survivors confirmed; 4 of 8 confirmed in live flights |
| Unit tests | 315 passing |

## Main limitations

* Avoidance needs about 0.9 s of warning. It still fails for
  hard-acceleration transit, lateral throws with ≤ 1 s of flight, and inside
  the roof opening (no escape manoeuvre).
* The survivor detector is specific to the simulated manikin; the VLM
  rejects straight-down views.
* The debris chain and the survivor chain have not been flown together.
* State estimation uses simulated GNSS. GPS-denied navigation is not
  implemented.
* The depth sensor has no noise model. No real-hardware test.

## What can be shown live

Two short Gazebo flights, commands in [`GAZEBO_DEMO.md`](GAZEBO_DEMO.md):
debris avoidance, and survivor detection (with VLM confirmation when the
NVIDIA GPU is available). Both were flown on this machine on 2026-10-06 on
the Intel GPU; see [`demo_check/`](demo_check/).

## Documents

| Document | Content |
|---|---|
| [`MASTER_RESULTS.md`](MASTER_RESULTS.md) | The full report, 29 sections, every phase in detail |
| [`RESULTS.md`](RESULTS.md) | Two-page results overview |
| [`PHASE_STATUS.md`](PHASE_STATUS.md) | Status table of all 16 phases |
| [`METRICS_SUMMARY.md`](METRICS_SUMMARY.md) | Every metric with its source file |
| [`ARCHITECTURE.md`](ARCHITECTURE.md) | Nodes, topics, authority rules |
| [`MATHEMATICAL_METHODS.md`](MATHEMATICAL_METHODS.md) | Equations actually implemented |
| [`LIMITATIONS_AND_FAILURES.md`](LIMITATIONS_AND_FAILURES.md) | Every failure, cause and status |
| [`GAZEBO_DEMO.md`](GAZEBO_DEMO.md) | Live demo commands |
| [`REPRODUCTION.md`](REPRODUCTION.md) | Versions, build, run, tests |

## Per phase

* [`phase0_flight_foundation/`](phase0_flight_foundation/README.md)
* [`phase1_environment_sensors/`](phase1_environment_sensors/README.md)
* [`phase2_debris_perception/`](phase2_debris_perception/README.md)
* [`phase3_debris_tracking/`](phase3_debris_tracking/README.md)
* [`phase4_trajectory_prediction/`](phase4_trajectory_prediction/README.md)
* [`phase5_collision_risk/`](phase5_collision_risk/README.md)
* [`phase6_debris_avoidance/`](phase6_debris_avoidance/README.md) (includes 6.1 and the estimator diagnostic)
* [`phase7_survivor_vlm/`](phase7_survivor_vlm/README.md)

## Evidence

The original evidence is not duplicated here; it stays in
`ros2_ws/results/phase*/`, `sensor_study/`, `sensor_integration/`,
`time_sync/`, `safety/` and `demo/`, with the design documents in
`ros2_ws/docs/`. This folder adds: the documents above, 20 figures copied
into [`figures/`](figures/), and the two demo-check flights of today.
228 PX4 flight logs are on disk under `results/`.

## Roadmap

Phase 8 survivor 3D localisation → 9 search planner → 10 mission state
machine → 11 natural-language mission → 12 integrated mission → 13
evaluation → 14 failure testing → 15 demonstration.
