# Multi-simulation progress (Gazebo urban disaster zone + MuJoCo)

Resume rule: read this file first. Only items marked **VERIFIED** were executed
and checked; everything else is not done. No commits or pushes are made by the
assistant. Existing evidence is never deleted.

Machine state (2026-10-20): kernel 7.0.0-34-generic, `nvidia-smi` fails ->
Intel rendering, no live VLM. Not changed (would need a reboot: asked first).

## Stage log

### Stage 0 - Inspection and rendering check (2026-10-20)
- Inspected: `ros2_ws/src/uav_autonomy/worlds/` (collapsed_building_rescue.sdf,
  survivor_variants/, village/, village_fast/), `scripts/village/make_village.py`
  (World/part/model helpers, `sdf_header`, scenarios, truth JSON in
  `results/phase7/worlds/`), `scripts/mission/run_mission.sh` (world selection:
  `base` / survivor_variants/<name>.sdf / `village_*` branch -> worlds/village[_fast],
  models_slam, bridge_village.yaml, SLAM + navigator), detector colour rules
  (`survivor_detection.py`: orange clothing H5-22 S>=170, skin, blue legs).
- Executed: textured OBJ mesh test world in Gazebo + `scripts/disaster/gz_snapshot.py`
  -> image saved. **VERIFIED**: textured meshes render (MTL map_Kd and SDF PBR
  albedo both). Evidence: `results/disaster_gazebo/checks/mesh_render_test.png`.
- Fact: PX4 base `.px4_param_backup` not present (clean).

### Stage 1 - Generator and world geometry (2026-10-20)
- Created `ros2_ws/scripts/disaster/make_disaster_zone.py` (procedural textures,
  damaged facade meshes from a cell grid with window rows kept out of the
  5.5-7 m band, tilted/jittered slabs, rubble piles from convex chunks, beams,
  collision boxes inside the visuals, truth JSON), `render_views.sh`
  (injects static cameras into a temp copy of the world), `validate_disaster.py`.
- Executed: `python3 scripts/disaster/make_disaster_zone.py` -> 6 scenarios
  (urban_A, B, B2, C, D, F), about 257 static models and 73k triangles each.
  Output: `src/uav_autonomy/models_disaster/dz_assets/` (20 MB),
  `worlds/disaster/` (shadows), `worlds/disaster_fast/` (no shadows),
  truth in `results/phase7/worlds/urban_*.json`.
- Executed: `scripts/disaster/render_views.sh urban_C` (Intel rendering, headless
  server) -> 8 PNGs in `results/disaster_gazebo/screenshots/urban_C/`.
  **VERIFIED by viewing**: grey overcast scene, damaged concrete frames, drooping
  floors, rubble with beams, survivors (orange) visible. First render was too
  boxy/intact and stains looked like camouflage -> reworked (partial floor
  tiles, more facade damage, bigger piles, subtler textures).
- Executed: `scripts/disaster/validate_disaster.py --scenario urban_C` ->
  **PASS** (heights <= 15 m, rubble <= 4.4 m, 363 structures cross the 6 m
  slice, needed routes exist with the navigator's planner, street block forces
  a 1.32x detour). The first run FAILED (detour 1.06x because the street was
  not enclosed) -> street-side walls added. Evidence:
  `results/disaster_gazebo/checks/validate_urban_C.json`.
- Changed existing files: `scripts/mission/run_mission.sh` (case
  `village_*|urban_*`, VDIR disaster/disaster_fast, models_disaster on the
  resource path; backup `results/disaster_gazebo/backups/run_mission.sh.before_urban`).
- New: `missions/places_disaster.json`, `missions/urban_collapsed_block.json`.

### Stage 2 - First flights in the urban world (2026-10-20, Intel rendering)
Every flight is kept in `ros2_ws/results/disaster_gazebo/live/` (failed ones too).
- `urban_B_first`, `_second`, `_gaps`, `_gaps2`, `_gaps3`, `_lighter`, `_batched`:
  **all FAILSAFE** ("PX4 position/status telemetry became stale or invalid"),
  from disarmed up to 1.5 s after the search started.
- Diagnosis (measured): a separate monitor (`scripts/disaster/gap_monitor.py`)
  saw no stall in `/clock` or PX4 position, but PX4's `vehicle_status` reached
  the controller only about once per second (gaps 0.83-1.14 s). PX4 publishes
  it at ~2 Hz of SIMULATION time, so at a real-time factor near 0.5 the gap
  touches the controller's 1.0 s wall-clock timeout. Takeoff-phase RTF:
  urban 0.50-0.56, village on Intel 0.79-0.84, small world 0.91.
  The 1.0 s timeout was NOT changed.
- Cause of the low RTF (measured with `scripts/disaster/rtf_probe.sh`,
  vehicle + three sensors, no PX4): 252 separate models / 245 collision shapes
  vs 73 / 65 in the village. Fixes in the generator: (1) static batching - one
  visual mesh per material (render), (2) all static collisions in ONE body
  (physics). Probe RTF 0.60 -> about 0.8-1.0 with all sensors rendering.
  Facade faces are also merged into rectangles (73k -> 39k triangles).
- `urban_B_onebody` (after both fixes): **no more stale-telemetry failsafe**;
  full search flown with SLAM + navigator for 180 s, then FAILSAFE from
  "SAFETY: impact 74.3 m/s^2" during the return climb. Cause: the navigator
  flew to the nearest reachable point of a goal inside the collapsed building
  (through a large facade opening), then no route home existed from that
  pocket, the navigator handed over to the controller, whose 18 m climb hit a
  floor slab. Found by the safety monitor, not hidden.
- Fix: breadcrumb backtracking in `navigation.py` / `navigator_node.py`
  (retrace the flown path when no route home exists), 2 new unit tests (43
  navigation tests pass).
- New tools: `gap_monitor.py`, `rtf_probe.sh`, `gz_sink.py` (all in
  `ros2_ws/scripts/disaster/`); `EXTRA_HOOK` option in `run_mission.sh`.
- SDF validation: `gz sdf -k` on all 12 world files: Valid
  (`results/disaster_gazebo/checks/sdf_validation.txt`).

### Stage 3 - First complete missions and SLAM tuning (2026-10-20)
- `urban_B_backtrack`: **COMPLETE**, landed, all PX4-log checks pass, survivor
  located 0.09 m (UNVERIFIED: no VLM on this kernel), coverage 0.63, time budget
  hit. But **SLAM failed**: RMSE 12.5 m, map rotated ~25 deg by a false match in
  the collapse zone (the navigator's jump check did not see it: the correction
  pivots around the vehicle). Mission safety was unaffected (flight uses PX4).
- Fix attempt 1 (`config/slam_toolbox_urban.yaml`, stricter acceptance AND a
  narrower search): `urban_B_strictslam` - **slam_toolbox crashed** ("Mapper
  FATAL ERROR - unable to get pointer in probability search"); navigator
  declared LOCALIZATION LOST after 6 s, controller returned home, landed
  (intended failure path worked). Kept as evidence.
- Fix attempt 2 (stricter thresholds only; search window as in the village):
  `urban_B_strictslam2` - COMPLETE, landed, survivor 0.15 m, coverage 0.83,
  **SLAM RMSE 1.10 m (max 1.72 m, 0.58 % of 272 m path), rigid-fit RMSE 0.67 m,
  map precision 0.43 (0.57 rigid-fit), recall 0.58**. Worse than the village
  (0.11-0.57 m): repetitive facades + clutter. Still one 14 m map->odom step
  (rotation about the vehicle), yaw error up to 11 deg. NOT claimed as solved.
- `run_mission.sh` now exports SLAM_PARAMS=config/slam_toolbox_urban.yaml for urban_*.

### Stage 4 - Debris avoidance and the status-timeout change (2026-10-20)
- `urban_B_debris` (release at sim ~100 s, UAV moving): COMPLETE, landed, boxes
  missed by 1.31 m with no manoeuvre needed; survivor NOT found (coverage 0.73,
  time budget). Not an avoidance test.
- `urban_B_debris_hover` (release at ~33-45 s): **FAILSAFE (stale telemetry)**
  again, with the debris detector adding load. Boxes also fell before the debris
  chain was active. Kept.
- **Controller change (flight_controller.py, backup in
  `results/disaster_gazebo/backups/flight_controller.py.before_status_timeout`)**:
  new parameter `status_timeout_s`, default = 1.0 s = unchanged behaviour. Urban
  runs pass 3.0 s (`run_mission.sh`, STATUS_TIMEOUT_S). Reason (measured): PX4's
  vehicle_status arrives at ~2 Hz of sim time, 0.83-1.14 s apart in wall time at
  RTF ~0.5, which trips the 1.0 s limit with no link loss. Position telemetry
  keeps the strict 1.0 s limit, so a real link loss is still caught. Full test
  suite after the change: 552 passed, 5 skipped (style-lint tests deselected).
- `urban_B_debris_hover2` (release 33 s): COMPLETE, but boxes fell before the
  avoidance node was active (first state change at 57 s): not an avoidance test.
- `urban_B_debris_lead` (plan `results/disaster_gazebo/plans/urban_lead.json`, boxes
  aimed 1.4 s ahead of the UAV via `lead_s`, release at ~85-94 s in the B1
  collapse zone): **COMPLETE, landed, 3 boxes, 2 avoidance manoeuvres accepted
  and completed, no impact, closest box 1.91 m, survivor located 0.01 m
  (unverified), coverage 0.77, log checks pass**. SLAM RMSE 1.11 m.

### Stage 5 - Return safety, demo and screenshots (2026-10-20)
- `urban_B_onebody` and `urban_C_demo1` both ended with **impact failsafes in the
  return climb** (74 m/s2 and 97 m/s2): the drone was inside / in the plane of a
  large facade opening of the collapsed block, and the 18 m climb-first return
  hit a slab / lintel above it. The 2D lidar slice cannot see overhead. Kept.
- Fix (controller param `map_return`, navigator subscribes `/mission/command`):
  a mission RETURN_HOME no longer starts a climb; the controller stays in SEARCH,
  the navigator routes home on the SLAM map at 6 m (breadcrumb backtracking if
  no route), and the controller starts its (short, near-home) return when the
  navigator says "complete". 150 s fallback to the old return. Enabled for urban
  worlds only (`run_mission.sh`). Tests: 552 passed, 5 skipped.
- `urban_C_demo2` (**Gazebo window + RViz + debris + capture, Intel, via
  `scripts/disaster/run_demo.sh`**): COMPLETE, landed, all PX4-log checks pass,
  3 avoidance manoeuvres, closest box 1.62 m, max altitude 6.15 m, map-routed return
  (about 85 s), coverage 0.71, candidate = a look-alike (no VLM), real survivor
  not reached within the 300 s budget. Evidence:
  `results/disaster_gazebo/demo/search_collapsed_block_urban_C_demo2/`
  (`screenshots_windows/` RViz + Gazebo window captures via `xwd -id`; OS
  desktop screenshots are blocked, not worked around).
- `capture_views.py` (cameras added to the running world): topic bug fixed
  (`/dz_view/dz_cam_<name>`), verified on an idle world (9 frames).

### Stage 6 - Scenario batch (in progress) and MuJoCo build (2026-10-20)
- Gazebo batch `scripts/disaster/run_disaster_batch.sh all` started 18:42, log
  `ros2_ws/results/disaster_gazebo/batch.log`, flights in
  `ros2_ws/results/disaster_gazebo/live/*_b1` and `*_headline`.
- **Contamination note (my mistake):** I ran MuJoCo flights/missions on the same
  machine while the batch was flying. `urban_C_headline` ended in a
  stale-telemetry failsafe (coverage 0.40) exactly while a MuJoCo mission was
  running, so that flight is NOT a valid result; it will be re-flown alone.
  MuJoCo job stopped at 19:20; no more concurrent runs until the batch ends.
- Batch so far (reports on disk): A COMPLETE (cov 0.78, no entries), B2
  COMPLETE (cov 0.56, courtyard survivor not found in the budget), headline C
  FAILSAFE (contaminated), D COMPLETE (cov 0.83).
- MuJoCo (all in `ros2_ws/simulation/mujoco/`), **VERIFIED by running** (headless, EGL):
  - env: `setup_env.sh` -> `.venv` with mujoco 3.16.0 only.
  - `scripts/hover_check.py`: hover z 4.997 m, RMS 6.7 mm.
  - `run_flight.py hover` (scene urban_C): altitude RMSE 9.4 mm, touchdown
    0.34 m/s, landed on the pad.
  - `run_flight.py waypoints`: 6/6 waypoints, cross-track RMSE 0.53 m (max
    1.37 m), speed overshoot 4.4 m/s vs 3 m/s command, landing offset 0.06 m.
  - `run_flight.py nav` (goal 33,90 beyond the blocked street): goal reached,
    flown path 110.9 m vs 95.9 m straight, **0 collisions**, min lidar range
    1.71 m, returned home, landed 0.11 m from the pad. Uses the project's A*
    navigator on a lidar map with the KNOWN pose (not SLAM).
  - `run_mission.py` (sentence -> search -> render -> detect -> localise ->
    return -> land): written; first run interrupted by me; NOT yet verified.
  - tests written (`tests/test_mujoco_sim.py`), NOT yet run.

### Stage 7 - Gazebo batch results, interior hazard, MuJoCo mission (2026-10-20)
- Gazebo batch 1 (`run_disaster_batch.sh all`, areas that INCLUDE building
  interiors; reports in `ros2_ws/results/disaster_gazebo/live/*_b1|*_b2|*kill*`):
  A COMPLETE; B b2 COMPLETE survivor 0.13 m; B2 courtyard COMPLETE, survivor
  not found (cov 0.56); D COMPLETE, found 0.14 m; F COMPLETE (cov 0.79);
  kill_slam and kill_navigator COMPLETE and landed (fail-safe path works);
  headline C FAILSAFE: spoiled by my concurrent MuJoCo job; re-flown alone as
  `urban_C_headline2`: COMPLETE, landed, log checks pass, 3 avoidance
  manoeuvres, closest box 1.48 m, but coverage 0.47 and no survivor found (the
  candidate it inspected was the tarp look-alike; no VLM).
- **Interior hazard (found in both simulators):** a 2D lidar slice cannot see
  overhead or slabs just below its plane. Gazebo: 2 impacts during the return
  climb after entering the collapsed block (onebody, demo1). MuJoCo: collision
  with a drooping floor tile inside the block at t = 139 s, mission INCOMPLETE
  (`ros2_ws/simulation/mujoco/results/dev/mission_B_interior`, kept).
  **Decision (mine, to be reviewed):** search areas are now EXTERIOR only
  (`missions/places_disaster.json`: collapsed_block east 14..27, courtyard
  floor, disaster_zone east 14..43, commercial_block plaza); the old
  rectangles are kept as `*_full` places and marked not flight-safe; the
  interior of standing structures is declared OUT OF SCOPE. Backups of the old
  places/missions: `ros2_ws/results/disaster_gazebo/backups/*before_exterior`.
- MuJoCo, all headless, nothing else running:
  - tests: `tests/test_mujoco_sim.py` **12 passed in 22 s** (scenes compile,
    free fall under g with motors off, hover RMSE < 5 cm, landing, waypoint
    arrival, lidar mapping, collision-free A* navigation).
  - bug found by profiling: navigator called every 4 ms tick replans forever
    when a goal has no route -> mission 20x too slow; now 10 Hz like the ROS
    node.
  - missions (sentence -> interpreter -> planner -> render -> detector ->
    localiser -> map-routed return -> land), `results/batch/`, summary
    `results/summary.json`: block_B, courtyard_B2, block_D_occluded,
    block_F_lookalikes, courtyard_C, zone_C_headline: **6/6 COMPLETE, landed,
    0 collisions, 6/6 survivors in the searched areas found, errors 0.21-0.46 m**;
    look-alikes appear as unverified candidates (no VLM): F 2, C-courtyard 1,
    C-zone 4. The controller uses ground-truth state; the map is lidar with known
    pose (not SLAM).
- Gazebo batch 2 `run_disaster_batch.sh ext` (exterior areas) started 21:19.

### Stage 8 - Final Gazebo batch (exterior areas), demo, MuJoCo PX4-in-the-loop (2026-10-20)
Gazebo, final configuration (Intel graphics, headless unless noted, nothing else
running; folders `ros2_ws/results/disaster_gazebo/live/*_x*`, scores in
`summary_ext.json`, `summary_slam.json`, `evaluate.log`):
- `run_disaster_batch.sh ext`: **10 of 10 flights COMPLETE and landed, 0
  failsafes, all 10 PX4-log verifications pass**. Objective (area covered) met
  in 7 (the other 3: the headline mission hit its 420 s budget at 0.795
  coverage, and the 2 fault flights end early by design).
  - A recon: covered, no candidate reported. B x2 repeats: survivor 0.11 and
    0.14 m. B2 courtyard: 0.23 m. D (under a slab): 0.18 m. F (look-alikes
    only): 2 look-alike candidates reported UNVERIFIED (no VLM).
  - headline (disaster zone, priority collapsed block, debris): 1 of 2
    survivors found (0.08 m), 2 avoidance manoeuvres, closest box 1.34 m.
  - debris flight (B + aimed boxes): survivor 0.06 m, 3 avoidance manoeuvres,
    closest box 1.88 m, no impact.
  - faults at 60 s: SLAM killed -> LOCALIZATION LOST -> return -> landed;
    navigator killed -> return -> landed.
  - totals: survivors reported 8 of 9 in the searched areas, error median
    0.21 m (mean 0.32, max 1.07), 5 avoidance manoeuvres for 6 boxes, 0 box
    collisions, closest box surface 1.34 m.
  - SLAM (offline, vs truth): RMSE 0.50-2.80 m, rigid-fit 0.17-1.92 m; map
    precision 0.31-0.61, recall 0.32-0.76. WORSE than the village (0.1-0.6 m).
  - route planning offline on urban maps: median 8-13 ms, p95 54-380 ms, max
    0.53 s per route (`planner_benchmark.json`).
- GUI demo (`scripts/disaster/run_demo.sh`, Gazebo window + RViz + debris):
  `demo/search_collapsed_block_urban_C_demo3`: COMPLETE, landed, log checks
  pass, survivor 0.17 m, coverage 1.0, 3 avoidance manoeuvres, closest box 1.19 m
  (centre), 36 in-world camera frames and 21 window captures. The Gazebo GUI
  showed a real-time factor of about 56 %.
- All earlier flights stay on disk and in `summary_missions.json`: 37 urban
  flights with a report in total, **26 COMPLETE and 11 FAILSAFE**. The 11:
  8 stale-telemetry failsafes before the `status_timeout_s` fix, 2 impacts in
  return climbs (before `map_return`), 1 stale-telemetry failsafe during my
  concurrent MuJoCo job. The final configuration (10 + the demo + the
  clean headline re-fly) had none.

MuJoCo (all in `ros2_ws/simulation/mujoco/`; headless EGL unless noted):
- GUI: `run_flight.py nav --gui` opens the MuJoCo viewer ("MuJoCo : mj_urban_C")
  and flew in real time with the same numbers as headless (window capture:
  `results/screens/mujoco_gui_nav_*.png`; needs X11, so GLFW is started with
  WAYLAND_DISPLAY unset).
- final flights `results/final/`: hover RMSE 9.4 mm, touchdown 0.34 m/s;
  square: 6/6 waypoints, cross-track RMSE 0.53 m; nav around the blocked
  street: 111 m, 0 collisions, min lidar range 1.69 m, landed. Deterministic
  (identical repeats, no noise).
- missions `results/batch/`: 6/6 COMPLETE, 0 collisions, 6/6 searched-area
  survivors found, errors 0.21-0.46 m (`results/summary.json`).
- isolation: `tests/check_isolation.sh` -> ISOLATION OK (no Gazebo result file
  or PX4 parameter file changed, 0 sim/ROS processes).
- **PX4 in the loop: DEMONSTRATED** (`results/px4_hil/`): PX4 SITL (airframe
  10016_none_iris via MAVLink HIL, bridge = TCP server on 4560) + the UNCHANGED
  `flight_controller.py` over the XRCE-DDS agent + MuJoCo physics.
  hil2, hil3, hil4 (hover 3 m, 30 s hold): COMPLETE, all 8 PX4-log checks pass,
  hold RMSE 4.3-4.5 cm, landing offset 0.2-1.4 cm. hilwp1 (3 waypoints): COMPLETE,
  all checks pass, waypoints reached to 0.4/17.6/0.4 cm, but landed 2.1 m from
  the start. Needed three fixes found by experiment: sensor noise (PX4's data
  validator flagged constant simulated sensors STALE after ~100 samples ->
  no heading), the magnetic field consistent with PX4's world model, and the
  fact that PX4 blocks in start-up until the simulator connects.
  Limits: iris-like vehicle model (not the x500), GNSS-aided EKF, no cameras or
  lidar in this loop, hover and one 3-waypoint flight only (n = 4 + 1).

### Stage 9 - Final evidence audit (2026-10-20/21) - every number below was re-measured in this stage
Nothing else was running during these checks except where stated. Evidence:
`ros2_ws/simulation/mujoco/results/audit/` and `ros2_ws/results/audit/gazebo/`.

1. **File layout** (checked): MuJoCo is at `ros2_ws/simulation/mujoco/` (inside the git
   repository; my first attempt put it one level too deep and outside the repo, fixed;
   the repo's own `simulation/object-track` is untouched). Venv `.venv` (git-ignored by
   `simulation/mujoco/.gitignore`), setup `setup_env.sh`. Gazebo generator
   `ros2_ws/scripts/disaster/make_disaster_zone.py`, worlds
   `ros2_ws/src/uav_autonomy/worlds/disaster{,_fast}/`, assets
   `.../models_disaster/dz_assets/` (11 meshes, 8 textures). No stale paths, no
   unignored large files except results (138 MB MuJoCo, 2.8 GB Gazebo results: the owner
   decides what to commit). `MULTI_SIM_PROGRESS.md` lives at the project root
   `~/Desktop/drones/`, which is OUTSIDE the git repository (a snapshot copy is in
   `ros2_ws/docs/`).
2. **MuJoCo from its final location**: `setup_env.sh` OK; `pytest tests` 12 passed;
   hover/waypoints/nav flights and the 6 missions are bit-identical to the earlier runs
   (deterministic); GUI: the documented command starts and keeps running
   (`results/audit/gui_documented.log`), window captured under X11
   (`results/audit/mujoco_gui_audit.png`).
   **PX4 in the loop** (`results/px4_hil/`): 3 new hover flights complete, all 8 PX4-log
   checks pass, hold RMSE 4.43-4.46 cm (earlier 4.3-4.5 cm), landing offsets 1.0-2.4 cm.
   Waypoint flights: all 3 waypoints reached in every run (closest approach <= 0.13 m).
   **2.1 m landing offset EXPLAINED and fixed**: without the controller's arrival-speed gate
   (`arrival_max_speed`, default 0 = off) "home reached" fires while flying past home and
   PX4 AUTO LAND drifts: 2.14 m (hilwp1), 2.23 m (audit_wp_default). With the gate (0.3 m/s, as
   the Gazebo missions use): 0.37 m (audit_wp_gate), 0.345 m (audit_wp_newdefault, the new
   script default). Remaining 0.35 m: not investigated.
   ROS 2 suite: **552 passed, 6 skipped, 2 failed** (the two style-lint tests, flake8 280
   findings and pep257: failing before this work too, 271 findings at the start of the
   session).
3. **Gazebo from scratch**: world loads and renders (9 stills,
   `results/audit/gazebo/screenshots/`, inspected: grey overcast light, damaged concrete
   frames, drooping slabs, rubble with beams, orange survivors on ledges, look-alikes, the
   walled street with its blockage; NOT photoreal: grid-like window openings, small
   rubble). GUI demo (`results/audit/gazebo/demo/..._audit_demo`): COMPLETE, landed,
   PX4 log checks pass, survivor 0.20 m, 3 avoidance manoeuvres, closest box 0.88 m, the
   second candidate was the tarp look-alike (no VLM), 39 in-world frames + 18 window
   captures, Gazebo GUI real-time factor 53 %.
   Strongest mission (`results/audit/gazebo/live/..._audit_headline3`; headline sentence,
   whole exterior zone, debris, alone on the machine): COMPLETE, landed, PX4 checks pass,
   coverage 0.893 in the 420 s budget, **1 of 2 survivors located (0.01 m)**, 5 candidates
   reported (3 are look-alikes, unverified), 3 avoidance manoeuvres, closest box 1.9 m,
   landing offset 0.22 m.
   Fallback small-world mission (`.../fallback/..._audit_fallback`): COMPLETE, 2/2
   survivors (0.09, 0.03 m), 2 manoeuvres, landed, PX4 checks pass.
4. **Separation** (`tests/check_separation.sh`, run twice): A: a Gazebo mission flying, the
   MuJoCo PX4 trial started -> refused ("REFUSING TO START"), Gazebo flight untouched and
   COMPLETE with PX4 checks True. B: a MuJoCo PX4 trial flying, a Gazebo mission started ->
   refused, MuJoCo trial COMPLETE with PX4 checks True. Standalone MuJoCo opens no socket;
   the PX4-in-the-loop path and Gazebo share PX4's UDP ports (8888 XRCE, 14540/14550/18570/
   14580), the PX4 rootfs (logs, parameters) and process names, so they must not run
   together; both scripts refuse. A standalone MuJoCo job next to a Gazebo flight is not
   blocked and spoils the flight's timing (seen once).
   **Bugs found by this test and fixed**: (1) my MuJoCo trial installed its cleanup trap
   BEFORE the refusal check, so a refused launch KILLED the running Gazebo flight (found:
   first attempt, `results/audit/gazebo/spoiled_by_script_bug/NOTE.txt`); (2) the trial
   reused result folders, which overwrote the logs of the stalled trial below.
   **UNRESOLVED**: in the first run of check B the MuJoCo trial hung (the bridge got no
   reply from PX4 at ~95 s simulated, hold stuck at 12 s of 30). A controlled set afterwards
   (3 trials with the refused Gazebo launch, 3 without, + 1 more) was 7/7 clean, so I
   cannot attribute the hang to the refused launch. Over the PX4-loop trials of this
   stage: 18 completed with every PX4-log check passing, 1 stalled, 1 aborted by me.
   A stall watchdog (8 s) now makes the bridge and trial report `BRIDGE_STALL` instead of
   waiting 300 s.

**Not verified / not done**: live VLM (NVIDIA driver not loaded on kernel 7.0.0-34; reboot
is the owner's action); MuJoCo debris avoidance, SLAM, VLM, depth camera, mission manager;
camera/lidar inside the PX4 loop; PX4 loop beyond hover and one 3-waypoint pattern;
MuJoCo capture under Wayland (GUI starts, capture needs X11); the stall cause.

## Next action
For a reliable faculty demonstration: (1) reboot into kernel 7.0.0-30-generic (GRUB, your
action) so the NVIDIA GPU and the live VLM work; (2) rehearse `scripts/disaster/run_demo.sh`
three times on that kernel; (3) keep the MuJoCo GUI run (`run_flight.py nav --gui`) as the
second, independent demo; (4) do not run anything else during a Gazebo flight.
