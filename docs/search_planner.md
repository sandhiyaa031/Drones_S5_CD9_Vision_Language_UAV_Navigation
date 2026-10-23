# Search planner (Phase 9)

Status: implemented, unit-tested (43 tests) and flown in seven live Gazebo
missions on 2026-10-06. Evidence: `results/phase9/` (one folder per flight
under `live/`, one score file `live_<name>.json` each).

All flights ran on the Intel GPU **without the VLM** (the NVIDIA driver was
not available), so no candidate was ever VLM-confirmed and the planner could
not use a VLM verdict to skip distractors.

## What it does

After takeoff the UAV covers a rectangular search area with parallel lanes,
keeps a map of what the camera has seen, makes a short detour to look at
each located survivor candidate from the side, then returns and lands.

| | |
|---|---|
| Library | `src/uav_autonomy/uav_autonomy/search_planning.py` (numpy only) |
| Node | `search_planner` (`search_planner_node.py`) |
| Controller rules | `search_interface.py`, new `SEARCH` state in `flight_controller.py` |
| Message | `uav_interfaces/SearchGoal` |
| Inputs | `/uav/flight_status`, `/fmu/out/vehicle_odometry`, `/camera/camera_info`, `/perception/survivor/locations` |
| Outputs | `/planning/search/goal` (10 Hz proposal), `/planning/search/status`, `/planning/search/markers` |
| Tests | `test/test_search_planning.py` |
| Run / score | `scripts/search/run_search.sh`, `scripts/search/evaluate_search.py` (the only place ground truth is used) |

## Authority

The planner proposes a horizontal goal and a speed cap. `flight_controller`
remains the only node that commands PX4 and applies these rules each cycle:

* a goal older than 1.0 s, not finite, or more than 25 m from the takeoff
  point is refused; the previous setpoint stays in force
* altitude is never taken from the planner
* the speed is always capped: the goal's value limited to 0.5–3.0 m/s,
  2.0 m/s if none is given
* no valid goal for 5 s, the planner reporting completion, or 300 s in
  SEARCH → return to the takeoff point and land
* SEARCH is a safety-monitored state, and debris-avoidance commands are
  accepted in it exactly as in the other flight states

The search is off by default (`search_mission:=false`); nothing changes for
the earlier flight modes.

## Method

1. **Footprint.** At height `h` above the plane of interest the usable
   image covers `±0.8·h·(H/2)/fy` forward and `±0.8·h·(W/2)/fx` sideways
   (0.8 = margin, because detections cut by the image border are not used).
   At 5 m: about ±2.9 m by ±3.9 m.
2. **Lanes.** Parallel lanes along the longer side of the area, spacing =
   swath width × (1 − 0.5): neighbouring swaths overlap by half, so every
   point is seen from two lanes, i.e. from the side as well as from above.
   The path starts at the lane end nearest to the vehicle.
3. **Speed.** 2.0 m/s on lanes, 1.5 m/s on inspection detours: inside the
   speeds validated for detection (Phase 7) and for debris avoidance in
   steady flight (Phase 6.1). No altitude changes.
4. **Coverage map.** A 0.5 m grid; each cycle the camera footprint computed
   from the vehicle's own pose marks the cells it contains. A cell also
   counts as "seen obliquely" when viewed at least 15° off nadir.
5. **Inspection.** When Phase 8 reports a candidate (at least 15 frames,
   known for 3 s, not rejected by the VLM, not at an already inspected
   place), the planner flies to two viewpoints on a circle around it, 90°
   apart, at about 35° off nadir, and holds 4 s at each. It ends early if
   the candidate becomes VLM-confirmed. At most 6 inspections, 25 s each.
   Then the lane is resumed.

## Results

| Flight | World | Outcome | Search time | Path flown | True coverage | Survivors located | Inspections |
|---|---|---|---|---|---|---|---|
| `none` | no survivor | complete | 26.9 s | 44.9 m | 100 % | 0 of 0 | 0 |
| `base` | building, 1 survivor | complete | 28.2 s | 23.4 m | 100 % | 1 of 1 (0.04 m) | 1 |
| `outdoor` | 1 survivor outside | complete | 45.1 s | 57.5 m | 100 % | 1 of 1 (0.09 m) | 1 |
| `multi` | 2 survivors + distractors | complete | 90.0 s | 81.2 m | 100 % | 2 of 2 (0.13, 0.05 m) | 4 (2 survivors, 2 distractors) |
| `multi_debris_light` | same + 3 falling boxes | complete | 97.7 s | 85.1 m | 100 % | 2 of 2 (0.22, 0.11 m) | 5 (2 survivors, 3 distractors) |
| `multi_debris` | same + 3 falling boxes | **failsafe** (see below) | – | 81.9 m | 100 % | 2 of 2 (0.17, 0.02 m) | 5 |
| `multi_planner_absent` | planner not running | landed safely | – | – | – | – | – |

* Every completed flight passed all PX4-log checks; landing 0.18–0.28 m
  from the takeoff point.
* Survivors were viewed at up to 37–59° off nadir (hover-only flights in
  Phase 7/8: about 13°).
* Motion: speed ≤ 2.06 m/s on lanes; horizontal acceleration p95
  2.3–2.8 m/s², maximum 3.0–4.2 m/s² (6.6 m/s² during an avoidance
  manoeuvre).
* The planner's own coverage estimate equals the true coverage in every
  flight (100 %, 100 % oblique).
* **Debris during the search:** 3 boxes released above the UAV in each of
  the two debris flights; 2 avoidance manoeuvres accepted in SEARCH each
  time; no box came closer than 1.36 m (centre to centre); the search
  resumed afterwards.

## Failures and limits

* **Failsafe under machine load (`multi_debris`).** With every node, five
  recorders and camera-frame recording running on the Intel GPU, the flight
  controller lost PX4 telemetry for more than 1 s, 127 s into the flight,
  and landed (safety guard working as designed). Repeated with lighter
  recording (`LIGHT=1`): complete. Same cause as the Phase 7 GPU-contention
  failures; not a planner fault, but the full system is at the limit of
  this machine without the NVIDIA GPU.
* **Distractors are inspected too.** Without a VLM verdict every candidate
  looks the same: 2–3 of the 4–5 detours went to orange objects. Each
  detour costs about 14 s.
* **Coverage is geometric.** A cell counts as seen when it is inside the
  camera footprint; walls and roof slabs that hide it are not modelled.
  100 % coverage does not mean the inside of the building was seen.
* **Duplicate entries from side views** (found in the first search flight,
  `multi_first_version`: 10 entries for 6 objects, 6 inspections). Fixed in
  the Phase 8 library (entries seen from one direction are merged into a
  localised entry they are consistent with; association allows for the
  systematic error) and by making the planner wait for 15 frames. After the
  fix: 6–7 entries; an object on the roof (3.2 m high) can still produce a
  second entry because its height is far from the assumed 0.9 m.
* **Planner parameter types** (found in the first attempt,
  `multi_planner_absent`): the planner did not start because an area bound
  was passed as an integer. The controller held, then returned and landed
  after 5 s without a goal. Fixed in `run_search.sh`; kept as evidence of
  the lost-planner behaviour.
* Rectangular, obstacle-free area at one altitude; constant heading; the
  area is given by hand. No re-planning for wind, battery or no-fly zones.
* Not validated with a live VLM, at other altitudes, or in larger areas.
* The release plan used for the debris flights was recreated
  (`results/phase6/plans/README.md`); it is not the original Phase 6 plan.

## Reproduce

```bash
cd ~/Desktop/drones/ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 scripts/survivor/make_worlds.py && scripts/search/run_search.sh multi && python3 scripts/search/evaluate_search.py results/phase9/live/multi
```

With falling debris and avoidance (lighter recording):

```bash
cd ~/Desktop/drones/ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && DEBRIS=1 LIGHT=1 scripts/search/run_search.sh multi debris_light
```

Add `HEADLESS=0` to watch in Gazebo. Unit tests:

```bash
cd ~/Desktop/drones/ros2_ws/src/uav_autonomy && python3 -m pytest test/test_search_planning.py -q
```
