# Faculty demo runbook (measured 2026-10-21, Intel graphics, no VLM)

Evidence: `results/faculty/` (Gazebo run folder, SLAM figure) and
`simulation/mujoco/results/faculty/`. Never run the two simulators at the same time.

## 1. Gazebo: fly to a survivor, detect, localise, report
```bash
cd ~/Desktop/drones/ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && scripts/disaster/run_demo.sh "Search the collapsed apartment block for survivors, report survivor locations, and return to base." urban_B
```
(asks you to confirm the interpreted mission; about 6 minutes). Fallback if anything fails:
```bash
cd ~/Desktop/drones/ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && INTERPRETER=rules HEADLESS=0 LIGHT=1 scripts/mission/run_instruction.sh "Search the whole site for survivors and avoid falling debris." multi demo_site
```
**Measured** (`results/faculty/gazebo/search_collapsed_block_urban_B_faculty_gz`): mission complete,
landed (offset 0.14 m), PX4 log checks pass. The downward camera produced detector candidates in 333
of 1989 frames, the first at t = 52.0 s; the first location estimate came at 53.1 s (coarse, sigma
0.61 m) and converged to sigma 0.21 m. **Reported survivor: 54.95 m north, 16.94 m east of takeoff
(lat 47.3984651, lon 8.5463886), height 2.66 m; true position 55.0 N, 17.0 E: error 0.10 m.**
Status in the report: UNVERIFIED_CANDIDATE (no VLM on this machine). Report: `mission_report.json`.
The estimate comes from the camera detections and PX4's pose (not from SLAM, not from simulator truth).

## 2. MuJoCo: same workflow, independent simulator
```bash
cd ~/Desktop/drones/ros2_ws/simulation/mujoco && env -u WAYLAND_DISPLAY MUJOCO_GL=glfw .venv/bin/python scripts/run_mission.py "Search the collapsed apartment block for survivors, report survivor locations, and return to base." --scene urban_B --gui
```
(headless: `MUJOCO_GL=egl` and no `--gui`; about 2 minutes of simulated time, runs in real time with the window.)
**Measured** (`simulation/mujoco/results/faculty/mujoco_B`, `..._B_gui`): complete, landed,
0 collisions, coverage 100 %; **reported survivor at 16.74 m east, 55.14 m north (sigma 0.20 m); true
17.0 / 55.0: error 0.30 m**; annotated camera frames `detection_*.png`.
What it is: own controller (not PX4) flying on simulator state, detector and localiser on rendered
camera frames, map from lidar with known pose. No VLM, no SLAM, no avoidance.
PX4-in-the-loop MuJoCo (hover and waypoints only): `simulation/mujoco/scripts/run_px4_mujoco_trial.sh my_hover 3.0`.

## 3. SLAM (verified in the Gazebo run above)
- Implementation: `slam_toolbox` (2D lidar pose-graph SLAM with loop closure, async) fed by
  `slam_bridge` (levelled scans + PX4 odometry as the odometry input).
- Live during the flight: `/map` 1.0 Hz (301 x 282 cells, 0.25 m), `/pose` 1.0 Hz, TF `map->odom`,
  `/uav/scan` ~3 Hz after filtering to the 6 m slice; nodes `slam_toolbox`, `slam_bridge`, `navigator`.
- Navigation uses it: the navigator subscribes to `/map`, `/tf`, `/pose`, `/uav/scan`, plans A* routes on
  the SLAM occupancy grid (1.75 m keep-out), converts goals between the PX4 and map frames through the two
  pose estimates, and declares LOCALIZATION LOST (then the controller returns home) if SLAM dies
  (tested: SLAM killed mid-flight -> returned and landed).
- Measured error against ground truth (offline only): position RMSE **1.33 m** (max 2.2 m, 0.65 % of the
  205 m path), **0.69 m after best rigid alignment**, yaw error up to 11.8 deg; map precision 0.57 and
  recall 0.71 after alignment (0.25 / 0.38 with the first-sample alignment). Figure:
  `results/faculty/slam_map_vs_truth.png`; RViz capture with the live map, mission state and camera:
  `.../screenshots_windows/rviz_0004.png`.
- Be honest about: SLAM is **not** what flies the drone (PX4 with simulated GNSS does; this is not
  GPS-denied flight) and here it is *less* accurate than the odometry it is given (rigid-fit 0.69 m vs
  0.20 m): repetitive facades and clutter hurt scan matching. It was useful in the village world, where
  it corrected deliberately degraded odometry (1.1 m and 6.7 m of odometry error down to 0.16 and 0.21 m).
  Survivor coordinates are in the PX4 local frame, not the SLAM map frame.

## 4. Talking points / limits
Searches are exterior only (rubble field, alley, courtyard, street); look-alikes are reported as
unverified because the VLM needs the NVIDIA driver (kernel 7.0.0-30, a reboot you choose).
