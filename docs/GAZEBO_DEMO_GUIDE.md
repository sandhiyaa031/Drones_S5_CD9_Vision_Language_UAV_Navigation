# Gazebo demo guide: urban earthquake-collapse disaster zone

Simulation A of the project: PX4 SITL + Gazebo Harmonic + the ROS 2 rescue
system in a dark urban earthquake-collapse zone (damaged concrete multi-storey
blocks, exposed and drooping floors, rubble piles, a rubble-blocked street, a
U-shaped courtyard). Everything below runs from `~/Desktop/drones/ros2_ws`.
The old small world and the village world are untouched and still work
(`docs/GAZEBO_READINESS_AUDIT.md` has their commands).

Machine state when this was written: kernel 7.0.0-34, `nvidia-smi` fails, so
Gazebo renders on the Intel GPU and there is no live VLM. Every number below
was measured in that state. Run `uname -r && nvidia-smi` first; with the
NVIDIA kernel (7.0.0-30, a reboot you choose) everything gets faster, and the
VLM can confirm survivors.

## 0. Prepare

```bash
cd ~/Desktop/drones/ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash
```

## 1. Build and check the world (no simulator needed)

```bash
python3 scripts/disaster/make_disaster_zone.py
```
Writes six scenarios (`urban_A` none, `urban_B` one survivor on the collapse,
`urban_B2` one in the courtyard, `urban_C` three + five look-alikes,
`urban_D` two under slabs, `urban_F` look-alikes only) to
`src/uav_autonomy/worlds/disaster/` (shadows) and `disaster_fast/` (no
shadows), assets to `src/uav_autonomy/models_disaster/dz_assets/`, truth for
OFFLINE scoring to `results/phase7/worlds/urban_*.json`.

```bash
python3 scripts/disaster/validate_disaster.py --scenario urban_C
```
Checks heights (<= 15 m), rubble (<= 4.5 m), that the 6 m lidar slice meets
363 structures, that the routes the mission needs exist (the navigator's own
planner, 1.75 m keep-out) and that the blocked street forces a detour.

```bash
scripts/disaster/render_views.sh urban_C
```
Eight stills from cameras injected into a temporary copy of the world
(`results/disaster_gazebo/screenshots/urban_C/`).

## 2. The one-command demo (Gazebo window + RViz)

```bash
scripts/disaster/run_demo.sh
```
Asks you to confirm the interpreted instruction ("Search the collapsed
apartment block for survivors, avoid falling debris, report survivor
locations, and return to base."), then starts PX4, Gazebo with its window,
the lidar UAV, SLAM, the navigator, the debris chain with avoidance, survivor
perception and localisation, the search planner, the mission manager, RViz and
the screenshot recorders. The default mission (rubble field and alley of the
collapsed block, about 4 minutes of search) takes 5-6 minutes of wall time on
Intel graphics. Another sentence or scene:

```bash
scripts/disaster/run_demo.sh "Search the courtyard for survivors." urban_B2 courtyard_demo
```

While it flies, in another terminal (view presets of the Gazebo window):

```bash
scripts/disaster/view.sh collapse
```
(`overview`, `collapse`, `courtyard`, `street`, `base`, `follow`, `free`.)

Output: `results/disaster_gazebo/demo/search_<area>_<scene>_<tag>/` with
`mission_report.json` (the report), `verification.json` (PX4-log checks),
`flight.ulg`, SLAM files, `screenshots_windows/` (RViz and Gazebo window
captures every 25 s) and `screenshots/` (views from cameras added to the
running world every 20 s). The operating system blocks desktop screenshots in
this session, so window pictures are made per window (`xwd -id`); a window
drawn over another one shows up in the other's picture.

## 3. Headless missions, batch and scoring

```bash
RESULTS_ROOT=$PWD/results/disaster_gazebo/live NVIDIA_OFFLOAD=0 LIGHT=1 scripts/mission/run_mission.sh $PWD/missions/urban_collapsed_block.json urban_B mytag
```
With falling debris (boxes aimed 1.4 s ahead of the UAV, released after the
debris chain is running):

```bash
DEBRIS=1 DEBRIS_SETTLE=70 PLAN=$PWD/results/disaster_gazebo/plans/urban_lead.json RESULTS_ROOT=$PWD/results/disaster_gazebo/live NVIDIA_OFFLOAD=0 LIGHT=1 scripts/mission/run_mission.sh $PWD/missions/urban_collapsed_block_protected.json urban_B mydebris
```
The whole evaluation batch (8 flights, about 1 hour) and its scoring:

```bash
scripts/disaster/run_disaster_batch.sh all
```
```bash
scripts/disaster/evaluate_disaster.sh
```
**Do not run other heavy jobs (for example the MuJoCo simulation) while a
Gazebo flight is running**: a flight in this project was spoiled that way
(see `MULTI_SIM_PROGRESS.md`).

## 3b. Scope: exterior searches only

The searched areas (`missions/places_disaster.json`) are the **exterior**: the
rubble field and alley of the collapsed block, the courtyard floor, the street
corridor and the plaza. **The interior of standing structures is out of scope.**
The 2D lidar slice (6 m) cannot see overhead or slabs just below its plane;
flights that went into the collapsed block through a facade opening ended in
impacts in both simulators (Gazebo: `search_*_B_onebody`, `*_C_demo1`; MuJoCo:
`simulation/mujoco/results/dev/mission_B_interior`). The old rectangles that
include interiors are kept as `collapsed_block_full` and `disaster_zone_full`
and are marked not flight-safe.

## 3c. Measured results of the final configuration (Intel graphics)

Source: `results/disaster_gazebo/summary_ext.json`, `summary_slam.json`, the run
folders `live/search_*_x*` and `demo/search_collapsed_block_urban_C_demo3`.

| Flight | Result |
|---|---|
| recon, no survivors (A) | covered, nothing reported |
| one survivor, 2 repeats (B) | found 0.11 m and 0.14 m, coverage 100 % |
| courtyard (B2) | found 0.23 m |
| survivor under a leaning slab (D) | found 0.18 m |
| look-alikes only (F) | 2 look-alike candidates reported unverified (no VLM) |
| falling debris, 3 boxes (B) | survivor 0.06 m, 3 avoidance manoeuvres, closest box 1.88 m |
| headline sentence, 2 survivors, debris (C) | 1 of 2 found (0.08 m), 2 manoeuvres, closest box 1.34 m, 420 s budget used at 79.5 % |
| SLAM killed at 60 s | localisation lost, returned, landed |
| navigator killed at 60 s | returned, landed |
| GUI demo with RViz (C, collapsed block) | survivor 0.17 m, 3 manoeuvres, closest box 1.19 m |

10 of 10 batch flights completed, landed and passed every PX4-log check, no
failsafe; 0 box collisions. SLAM error against ground truth was 0.5-2.8 m RMSE
and the map precision 0.31-0.61 (worse than the village world, see section 5).
Earlier development flights (11 failsafes of 37) are kept; see
`MULTI_SIM_PROGRESS.md`.

## 4. What is different from the village and the old world

- `run_mission.sh` treats worlds named `urban_*` like the village ones
  (lidar UAV, SLAM, navigator) and additionally: the stricter SLAM settings
  `config/slam_toolbox_urban.yaml`, the controller parameters
  `status_timeout_s:=3.0` and `map_return:=true` (see section 5).
- Every return is routed on the SLAM map at the search altitude; the climb to
  18 m only happens near the base.

## 5. Things you must know (measured, not guessed)

- PX4 publishes its vehicle status at about 2 Hz of simulation time. At the
  real-time factor of this world on Intel graphics (about 0.5-0.6) two messages
  are about 1 s apart in wall time, which tripped the controller's 1.0 s
  freshness limit without any link loss (8 flights ended in a failsafe before
  this was understood). `status_timeout_s` (default 1.0 = unchanged, 3.0 here)
  applies to the status only; position telemetry keeps the 1.0 s limit.
- The 2D lidar slice cannot see what is above the vehicle. Two flights ended in
  an impact during an 18 m return climb inside / under a collapsed structure.
  Hence the map-routed return.
- The SLAM map can be pulled out of shape by repetitive facades. The stricter
  settings reduced the position error from 12.5 m to about 1.1 m over a 270 m
  flight; it is still worse than in the village world. PX4's own estimate flies
  the vehicle; SLAM supplies the obstacle map.
- Dust and haze are NOT simulated: Gazebo's particle effects also appear in the
  depth camera and lidar and would create false debris and wall returns.
- Without the VLM every candidate is reported as unverified, including
  look-alikes (barrel, cone, tarp, crate).


## 6. Final audit (2026-10-21), re-measured

Evidence `results/audit/gazebo/`. GUI demo (Intel graphics, no VLM): complete, landed,
PX4-log checks pass, survivor located 0.20 m, 3 avoidance manoeuvres, a look-alike also
reported as unverified candidate. Headline mission (whole exterior zone, debris): complete,
landed, 1 of 2 survivors located (0.01 m), 3 look-alikes reported, 3 manoeuvres. The old
small-world mission still works (2 of 2 survivors). **Rehearse before showing: the
operating system forbids desktop screenshots here, so window captures come from `xwd`.**

Exact commands (from `~/Desktop/drones/ros2_ws`, after `source /opt/ros/jazzy/setup.bash && source install/setup.bash`):

| What | Command |
|---|---|
| Urban GUI demo (Gazebo + RViz) | `scripts/disaster/run_demo.sh` |
| Urban mission, headless | `RESULTS_ROOT=$PWD/results/disaster_gazebo/live NVIDIA_OFFLOAD=0 LIGHT=1 scripts/mission/run_mission.sh $PWD/missions/urban_collapsed_block.json urban_B mytag` |
| Whole evaluation batch | `scripts/disaster/run_disaster_batch.sh ext` then `scripts/disaster/evaluate_disaster.sh` |
| Fallback: original small world | `INTERPRETER=rules HEADLESS=0 LIGHT=1 scripts/mission/run_instruction.sh "Search the whole site for survivors and avoid falling debris." multi demo_site` |
| MuJoCo + PX4 hover | `simulation/mujoco/scripts/run_px4_mujoco_trial.sh my_hover 3.0` |

Never run a Gazebo flight and a MuJoCo PX4 trial together: both launchers refuse
(`simulation/mujoco/tests/check_separation.sh` checks both directions). Do not run anything
heavy (for example the standalone MuJoCo simulation) next to a Gazebo flight either.
