# Survivor 3D localisation (Phase 8)

Status: implemented, unit-tested (38 tests) and validated in eight live
Gazebo flights on 2026-10-06. Evidence: `results/phase8/` (one folder per
flight under `live/`, one score file `live_<scenario>.json` each,
`batch.log`).

## What it does

Turns the Phase 7 pixel detections of a survivor candidate into a position in
the PX4 local frame (north, east, down) with an uncertainty, and keeps one
entry per physical candidate for the whole flight.

| | |
|---|---|
| Library | `src/uav_autonomy/uav_autonomy/survivor_localization.py` (numpy only) |
| Node | `survivor_localization` (`survivor_localization_node.py`) |
| Messages | `uav_interfaces/SurvivorLocation`, `SurvivorLocationArray` |
| Inputs | `/perception/survivor/detections`, `/camera/camera_info`, `/fmu/out/vehicle_odometry` |
| Outputs | `/perception/survivor/locations` (frame `px4_local_ned`), `/perception/survivor/location_markers`, `/perception/survivor/localization_status` |
| Tests | `test/test_survivor_localization.py`, 36 tests |
| Offline scoring | `scripts/survivor/evaluate_localization.py` (the only place ground truth is used) |
| Recording | `scripts/perception/log_locations.py` |

The node does not command PX4, does not read simulator ground truth and does
not change the flight path. It is not connected to the flight controller.

## Method

The downward camera is monocular: one frame gives a direction, not a
distance.

1. **Viewing ray.** Pixel `(u, v)` → body-frame ray
   `r_b = normalise(−(v−cy)/fy, (u−cx)/fx, 1)` (image right = body right,
   image up = body forward, optical axis down). Camera position
   `c = p + R·(0, 0, 0.14)`, ray `r = R·r_b`, with the vehicle pose `(p, R)`
   interpolated to the image time.
2. **First estimate.** The ray is cut by a horizontal plane 0.9 m above the
   ground (mid-way between a person lying and the top of a standing person).
   The ground level is measured from the vehicle's own altitude estimate
   before takeoff. Covariance: small across the ray, large along it
   (height prior σ = 0.6 m).
3. **Refinement.** Each candidate is a static 3D point in an extended Kalman
   filter; the measurement is the pixel position,
   `u = cx + fx·y_b/z_b`, `v = cy − fy·x_b/z_b`, with the 2×3 Jacobian. When
   the vehicle moves, rays from different places intersect and the filter
   triangulates, so the height assumption stops mattering.
4. **Measurement noise.** `σ² = σ_px² + (f·σ_att)² + (f·σ_pos/range)²` with
   σ_px = 15 % of the box size, σ_att = 1.5°, σ_pos = 0.15 m. Frames closer
   than 0.5 s share their errors and are down-weighted accordingly.
5. **Reported uncertainty.** Filter covariance plus a 0.2 m systematic floor
   (the vehicle's own position error and the offset between box centre and
   the person do not average out).
6. **Skipped frames.** Box cut by the image border, detection not measured
   in this frame, no pose at the image time, vehicle tilt above 35°, ray
   flatter than about 20°, pixel innovation outside the 99.9 % gate.
7. **Identity.** A new image track joins an existing entry if its pixel
   position agrees with that entry's projection; entries that converge to
   within 0.5 m are merged. "VLM confirmed" is sticky per entry and is never
   inherited from a neighbouring object.
   An entry seen from one direction only (long uncertainty along its
   viewing ray) is also merged into another entry it is consistent with,
   up to 1.5 m away (added in Phase 9, after side views produced duplicate
   entries; re-checked on the eight Phase 8 flights: no regression, and the
   duplicate in the occluded case disappears, 0.41 m final error).
8. **States.** COARSE (single viewing direction, height assumed) and
   LOCALIZED (reported horizontal 1σ ≤ 0.5 m and ≥ 10 frames). Fewer than 5
   frames: not published.

## Results

Horizontal error = distance between the estimate and the true position of
the object, both in the local frame. All eight flights ran on the Intel GPU
without the VLM (the NVIDIA driver was not available), so nothing is
VLM-confirmed in them. All eight completed their mission.

### Live flights (node running in flight)

| Scenario | Object | Error at first report | Final error | Reported 1σ | Frames | Camera baseline |
|---|---|---|---|---|---|---|
| A_clear: hover 5 m | survivor | 0.35 m | 0.14 m | 0.26 m | 291 | 1.6 m (climb) |
| D_altitude: hover 12 m | survivor | 0.33 m | 0.34 m | 0.21 m | 253 | 8.7 m (climb) |
| E_moving_20: route at 2.0 m/s | survivor | 0.34 m | 0.04 m | 0.21 m | 96 | 6.3 m |
| E_moving_29: route at 2.9 m/s | survivor | 0.41 m | 0.04 m | 0.22 m | 58 | 6.0 m |
| H_multi: two survivors + distractors | survivor (inside) | 0.32 m | 0.10 m | 0.21 m | 152 | 3.4 m |
| | survivor (outside) | 0.39 m | 0.14 m | 0.22 m | 49 | 5.9 m |
| | orange cylinder | 0.18 m | 0.03 m | 0.20 m | 180 | 7.3 m |
| | orange plank | 0.29 m | 0.04 m | 0.21 m | 122 | 8.6 m |
| C_rubble: survivor in rubble | survivor | 0.64 m | 0.46 m | 0.28 m | 288 | 1.6 m |
| B_occluded: survivor partly hidden | survivor, entry 1 | 0.14 m | 0.29 m | 0.23 m | 120 | 1.5 m |
| | survivor, entry 2 (duplicate) | 0.65 m | **0.57 m** | 0.26 m | 253 | 1.1 m |
| F_none: no survivor, route | – | no entry published | | | | |

Summary: 8 of 8 survivor instances located; final error 0.04–0.14 m when the
vehicle flew past, 0.14–0.34 m from a hover, 0.46–0.57 m with rubble or an
occluder. One duplicate entry (occluded case). No entry in the empty world.
Every final error is within the reported 2σ except the occluded duplicate
(2.2σ).

### Earlier replay (files no longer on disk)

Before the live flights, the same library was replayed on 23 recorded
Phase 7 flights (recorded pixel detections + PX4's own pose from the flight
log). Those recordings were removed from the workspace on 2026-10-06, so
these numbers are quoted from that session's output and cannot be
re-derived: clear views 0.05–0.33 m; rubble 0.43–0.46 m; partly hidden
0.47–0.60 m (outside 2σ); no entries in four no-survivor flights; in the two
flights with VLM confirmation and distractors, survivor entries were marked
confirmed and no distractor entry was.

## Failures and limits

* **Partial occlusion biases the position and can split one person into
  two entries** (live: 0.29 m and 0.57 m, the second at 2.2σ): the visible
  part's centre is not the person's centre and it changes with the view.
  The reported uncertainty does not cover this.
* **Not yet shown with a live VLM**: the "confirmed" flag was exercised only
  in the earlier replay and in unit tests.
* **Hover only gives a direction.** With no vehicle motion the position
  depends on the assumed 0.9 m height; the error is the height error times
  the tangent of the off-nadir angle.
* **Accuracy is bounded by the vehicle's own position estimate**
  (0.1–0.3 m against the simulator in these flights).
* **Objects about 1 m apart are the separation limit**; an earlier version
  merged a survivor with a cylinder 1.07 m away and wrongly marked the
  cylinder confirmed. Fixed by projection-based association; covered by a
  unit test.
* Distractor objects are located too, as unconfirmed candidates.
* Static targets only. Flat ground assumed for the first estimate.
* Simulation only; the camera has no lens distortion or noise.
* Found during validation: the trial script did not stop the new node, so a
  node from the previous flight kept publishing. Fixed in
  `scripts/run_flight_trial.sh`; the two affected flights were repeated.

## Reproduce

Live flight with the node, recorded and scored (Intel GPU, no VLM):

```bash
cd ~/Desktop/drones/ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 scripts/survivor/make_worlds.py && NO_VLM=1 LOCALIZATION=1 NVIDIA_OFFLOAD=0 RESULTS_ROOT=$PWD/results/phase8/live scripts/survivor/run_scenario.sh E_moving_20 && python3 scripts/survivor/evaluate_localization.py results/phase8/live/E_moving_20
```

Scenarios: `A_clear`, `D_altitude`, `E_moving_20`, `E_moving_29`, `H_multi`,
`B_occluded`, `C_rubble`. Unit tests:

```bash
cd ~/Desktop/drones/ros2_ws/src/uav_autonomy && python3 -m pytest test/test_survivor_localization.py -q
```
