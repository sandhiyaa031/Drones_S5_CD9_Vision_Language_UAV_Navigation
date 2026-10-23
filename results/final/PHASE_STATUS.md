# Phase status

Status as of 2026-10-06. **The project is not complete.** Phases 0–7 are
done to the level stated below; Phases 8–15 have not been started, so there
is no integrated end-to-end rescue mission yet.

Status words: **DONE** = validated live in Gazebo against the phase's
criteria. **BOUNDED** = works inside a measured envelope, with documented
failures outside it. **ACCEPTED SIMULATION BASELINE** = accepted for the
simulated manikin only. Evidence paths are relative to `ros2_ws/`.

| Phase | Description | Status | Key result | Evidence | Remaining work |
|---|---|---|---|---|---|
| 0 | Flight foundation: PX4 SITL offboard takeoff, hover, landing; flight safety monitor | **DONE** | 5/5 hover flights pass every log check; altitude RMSE 0.070–0.096 m; roof collision of the stock waypoint route detected and landed | `results/phase0/`, `results/safety/`, `docs/runbook.md`, `docs/flight_safety.md` | A crashed vehicle may stay armed (no auto-disarm, by decision) |
| 1 | Environment + sensors: clean collapsed-building world, downward RGB + upward depth camera, time base | **DONE** | Real-time factor 0.9986, both cameras 30.2–30.3 Hz (NVIDIA); PX4-to-simulation time error 0 ms | `results/phase1/`, `results/sensor_study/`, `results/sensor_integration/`, `results/time_sync/`, `docs/sensor_integration.md` | Performance target needs the NVIDIA driver (Intel GPU: 0.90–0.94); depth sensor has no noise model |
| 2 | 3D debris perception from the upward depth camera | **DONE** | 86/86 objects detected; frame recall 98.7 %; 0 unexplained detections in 1852 | `results/phase2/`, `docs/debris_perception.md` | Overlapping objects merge; only sees the upward cone |
| 3 | Debris tracking (Kalman filter with gravity, Hungarian association) | **DONE** | 0 ID switches in 7 live runs; position RMSE ≤ 0.073 m to the true surface; velocity RMSE 0.07–0.43 m/s | `results/phase3/`, `docs/debris_tracking.md` | Edge-only objects stay tentative; nothing valid after impact |
| 4 | Trajectory prediction (ballistic) | **DONE** | RMSE 0.110 m at 1.0 s, 0.414 m at 2.0 s (constant velocity: 4.44 m at 1.0 s) | `results/phase4/`, `docs/debris_prediction.md` | No drag, bounce or map |
| 5 | Collision-risk estimation (advisory) | **DONE** | Free fall: recall 0.954, precision 0.61; all 10 true collisions CRITICAL; lead time median 0.80 s | `results/phase5/`, `docs/debris_risk.md` | Deflections off structure not predicted; ~18 % conservative false alarms |
| 6 | Autonomous debris avoidance | **BOUNDED / PARTIAL** | Collision runs 11/14 → 5/14; 15/15 direct hits on a hovering UAV avoided; needs ≈ 0.9 s of warning | `results/phase6/`, `docs/debris_avoidance.md` | Fails for hard-acceleration transit, lateral throws under ~1 s, no-escape geometry |
| 6.1 | Robustness: perception health, speed caps, UAV motion models, estimator diagnostic | **BOUNDED** | Steady cruise to 4.4 m/s: 0 collisions in 8 flights; depth gaps to 500 ms tolerated; minimum lead ≈ 0.9 s measured | `results/phase6_1/`, `results/phase6_1_estimator_diagnostic/` | Hard acceleration **not solved**; estimator change does not help (diagnosed) |
| 7 | Survivor perception + VLM verification | **ACCEPTED SIMULATION BASELINE** | Detector recall 1.00, precision 0.72 (all false positives on one designed distractor); VLM: 0 false confirmations, 24/45 clear survivors confirmed; 4/8 survivors confirmed live | `results/phase7/`, `docs/survivor_perception.md` | Colour/shape detector for the manikin only; VLM rejects straight-down views |
| 8 | Survivor 3D localisation | **NOT STARTED** | – | – | everything |
| 9 | Search planner | **NOT STARTED** | – | – | everything |
| 10 | Mission state machine | **NOT STARTED** | – | – | everything |
| 11 | Natural-language mission interpretation | **NOT STARTED** | – | – | everything |
| 12 | End-to-end integrated mission | **NOT STARTED** | – | – | everything |
| 13 | System evaluation | **NOT STARTED** | – | – | everything |
| 14 | Failure testing | **NOT STARTED** | – | – | everything |
| 15 | Demonstration and final documentation | **NOT STARTED** | – | – | everything |

## What this means in one paragraph

The UAV can take off, fly and land by itself in simulation, see falling
debris with an upward depth camera, track it, predict where it will be,
decide whether it is a threat, and side-step it when it has about 0.9 s of
warning. Separately, it can see the simulated survivor with its downward
camera and have a local vision-language model confirm it. These two chains
have not yet been flown together, the survivor is not yet located in 3D, and
there is no search behaviour or mission logic tying it into one rescue
mission.

## Software status

* 315 unit tests pass (Phases 0–7).
* Git: branch `claude-development` at `b39741a` (Phase 0/1, safety monitor
  and sensor study). Everything after that is in the working tree,
  **uncommitted**, by the project owner's instruction.
