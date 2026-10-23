# Gazebo demonstration guide (what can be shown today)

## What exists and what does not

```
CAN BE DEMONSTRATED (validated, Phases 0–7)
  autonomous takeoff → hover / waypoint flight → landing            (PX4 SITL, X500)
  collapsed-building world, downward RGB camera, upward depth camera
  falling debris → 3D detection → tracking → ballistic prediction → collision risk
  → autonomous avoidance manoeuvre → return to the mission setpoint
  survivor manikin → visual candidate → temporal tracking → local VLM check → SURVIVOR_CONFIRMED

NOT YET BUILT (Phases 8–15, not started)
  3D survivor localisation → search planner → mission state machine
  → natural-language mission → one integrated end-to-end rescue mission
```

Two things to say plainly in the demo:

* The debris-avoidance chain (Phases 2–6) and the survivor/VLM chain
  (Phase 7) were validated **in separate flights**. Running both in one
  flight has not been tested; it is integration work for a later phase.
* The survivor detector is a colour/shape baseline for the simulated
  manikin, not a general person detector.

## Before the demo: check the machine (2 minutes)

```bash
uname -r
```
```bash
nvidia-smi
```

| State | Meaning for the demo |
|---|---|
| kernel `7.0.0-30-generic`, `nvidia-smi` shows the RTX 4050 | validated configuration: real-time factor ≈ 1.0, VLM answers in ≈ 2.4 s |
| kernel `7.0.0-34-generic`, `nvidia-smi` fails (**the state found on 2026-10-06**) | Gazebo renders on the Intel GPU. Both demos below were flown in this state today and worked, at lower camera rates. The VLM would run on the CPU (44–82 s per answer), too slow to confirm in a short hover. |

To get the NVIDIA state back: reboot and pick kernel `7.0.0-30-generic` in
the GRUB "Advanced options" menu (machine owner's action; nothing is
installed). Details: `docs/runbook.md`.

Every command below is run from `~/Desktop/drones/ros2_ws`. Each script
cold-starts Gazebo, PX4, the XRCE agent, the GCS heartbeat, the camera
bridge and the nodes, flies, tears everything down and verifies the PX4 log.
Wait for it to finish before starting the next one.

## Demo 1 — debris avoidance (≈ 1.5 min)

Three boxes are dropped from 12 m straight onto the UAV hovering at 5 m,
one at a time. Without avoidance the first one hits (baseline flights).

Intel GPU (works without NVIDIA; **flown today**, headless: 3 of 3 avoided
by 1.83–2.09 m, mission completed — `demo_check/avoidance_direct_intel_gpu`):

```bash
D='python3 $WS_DIR/scripts/sensor_study/record.py --out-dir $RUN_DIR --duration 80 --poses-only & python3 $WS_DIR/scripts/perception/log_detections.py --out-dir $RUN_DIR --duration 80 --save-debug-every 4 & python3 $WS_DIR/scripts/sensor_study/spawn_study.py --plan $WS_DIR/results/phase6/plans/C_direct.json --out $RUN_DIR/releases.json --start-altitude 4.8 --settle 1.5 --max-duration 23; wait'; HEADLESS=0 SETTLE_SECONDS=20 PERCEPTION=1 NVIDIA_OFFLOAD=0 CAMERA_BRIDGE=1 AVOIDANCE=on DURING_HOOK="$D" CONTROLLER_ARGS="-p target_altitude:=5.0" VERIFY_ARGS="--target-alt 5.0 --hold-seconds 30" FLIGHT_TIMEOUT=180 scripts/run_flight_trial.sh results/project_progress/demo_check/live_avoidance hover
```

With NVIDIA (the validated Phase 6 command; add `HEADLESS=0` in front to
open the Gazebo window):

```bash
RESULTS_ROOT=$PWD/results/project_progress/demo_check HEADLESS=0 SETTLE_SECONDS=20 scripts/avoidance/run_scenario.sh C_direct avoid demo
```

The same scenario without avoidance, to show the collision
(replace `avoid` by `baseline`):

```bash
RESULTS_ROOT=$PWD/results/project_progress/demo_check HEADLESS=0 SETTLE_SECONDS=20 scripts/avoidance/run_scenario.sh C_direct baseline demo
```

`HEADLESS=0` opens the Gazebo GUI. The two commands above with the GUI open
were not rehearsed today (today's checks were headless); the GUI adds load at
start-up, which is why `SETTLE_SECONDS=20` is set. If the controller reports
"telemetry became stale" during start-up, close everything and run it again.

What to point at while it runs (Terminal 6 commands below):
`/perception/debris/risk` going SAFE → CRITICAL, the controller log line
"AVOIDANCE accepted", the UAV side-stepping about 3 m in the Gazebo window,
then "AVOIDANCE recovery complete".

Score it afterwards:

```bash
PYTHONPATH=src/uav_autonomy python3 scripts/avoidance/evaluate_avoidance.py results/project_progress/demo_check/live_avoidance
```

## Demo 2 — survivor detection and VLM confirmation (≈ 1.5 min)

The UAV climbs through the roof opening to 5 m and hovers; the manikin is
1.2 m to the side, seen obliquely.

Start the local VLM first (Terminal 5), and leave it running:

```bash
OLLAMA_HOST=127.0.0.1:11434 OLLAMA_KEEP_ALIVE=60m ~/ollama/bin/ollama serve
```

Warm the model once in another terminal (the first load takes 2–4 minutes;
later answers take ≈ 2.4 s on the GPU):

```bash
~/ollama/bin/ollama run gemma3:4b "ready"
```

With NVIDIA (the validated Phase 7 command):

```bash
VLM_ENDPOINT=http://127.0.0.1:11434/v1 VLM_MODEL=gemma3:4b HEADLESS=0 SETTLE_SECONDS=20 scripts/survivor/run_scenario.sh A_clear demo
```

Expected: `CANDIDATE` within 0.3 s of the manikin entering the view,
`SURVIVOR_CONFIRMED` about 2.5 s later. The run is written to
`results/phase7/live/A_clear_demo/` (a new folder; existing results are not
touched).

Intel GPU, without the VLM (**flown today**, headless: CANDIDATE in 349 of
370 airborne frames, 0 false positives, status VLM_UNAVAILABLE —
`demo_check/survivor_clear_intel_gpu_no_vlm`):

```bash
D='python3 $WS_DIR/scripts/sensor_study/record.py --out-dir $RUN_DIR --duration 95 --frame-every 3 & python3 $WS_DIR/scripts/perception/log_survivor.py --out-dir $RUN_DIR --duration 95 & wait'; HEADLESS=0 SETTLE_SECONDS=20 NVIDIA_OFFLOAD=0 CAMERA_BRIDGE=1 SURVIVOR=1 DURING_HOOK="$D" CONTROLLER_ARGS="-p target_altitude:=5.0 -p hold_seconds:=20.0" VERIFY_ARGS="--target-alt 5.0 --hold-seconds 20" FLIGHT_TIMEOUT=170 scripts/run_flight_trial.sh results/project_progress/demo_check/live_survivor hover
```

This shows the honest fallback: the survivor is reported as `CANDIDATE` with
`VLM_UNAVAILABLE`, never as confirmed.

Other Phase 7 scenarios that can be shown the same way (replace `A_clear`):
`G_distractors` (orange boxes and a skin-coloured ball: no confirmation),
`F_none` (no survivor: NO_TARGET), `C_rubble`, `B_occluded`.

Watch the node's own view while it flies:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run rqt_image_view rqt_image_view /perception/survivor/debug_image
```

## Monitoring commands (Terminal 6)

Source ROS first in that terminal:

```bash
source /opt/ros/jazzy/setup.bash && source ~/Desktop/drones/ros2_ws/install/setup.bash
```

```bash
ros2 topic echo /uav/flight_status --field state
```
```bash
ros2 topic echo /perception/debris/risk --field highest_state
```
```bash
ros2 topic echo /perception/debris/avoidance_command --field reason
```
```bash
ros2 topic echo /perception/survivor/detections --field state
```
```bash
ros2 topic hz /depth_up/image_raw
```
```bash
ros2 run rqt_image_view rqt_image_view /perception/debris/debug_image
```
```bash
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

Risk states: 0 SAFE, 1 WATCH, 2 WARNING, 3 CRITICAL. Survivor states:
0 NO_TARGET, 1 CANDIDATE, 2 SURVIVOR_CONFIRMED, 3 UNCERTAIN.
RViz markers (fixed frame `px4_local_ned`): `/perception/debris/track_markers`,
`/perception/debris/risk_markers`, `/perception/debris/avoidance_markers`.

## The same thing, one terminal per process

The scripts above are the validated path. The sequence below is what
`scripts/run_flight_trial.sh` runs, split by terminal; it was not separately
rehearsed as six manual terminals. Start order: **3, 1, 2, 4, 5, 6**
(Gazebo must be up before PX4 attaches).

Every terminal first:

```bash
source /opt/ros/jazzy/setup.bash && source ~/Desktop/drones/ros2_ws/install/setup.bash && cd ~/Desktop/drones/ros2_ws
```

**Terminal 3 — Gazebo** (server; omit the three `__NV…` variables on the
Intel GPU):

```bash
export GZ_SIM_RESOURCE_PATH=$PWD/src/uav_autonomy/models && source ../PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh && export GZ_SIM_SERVER_CONFIG_PATH=$PWD/src/uav_autonomy/config/gz_server.config && export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json && gz sim --verbose=3 -r -s src/uav_autonomy/worlds/collapsed_building_rescue.sdf
```

GUI, in a second tab:

```bash
gz sim -g
```

Spawn the project UAV (downward mono + upward depth camera), in a third tab:

```bash
gz service -s /world/collapsed_building_rescue/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 --req "name: \"x500_rescue_0\", allow_renaming: false, sdf_filename: \"$PWD/src/uav_autonomy/models/x500_rescue/model.sdf\""
```

**Terminal 1 — PX4 SITL**, attached to that model. `PX4_PARAM_UXRCE_DDS_SYNCT=0`
makes PX4 time equal simulation time; PX4 stores it, so copy
`parameters.bson` and `parameters_backup.bson` aside first and put them back
afterwards (the script does this automatically):

```bash
cd ~/Desktop/drones/PX4-Autopilot/build/px4_sitl_default/rootfs && PX4_PARAM_UXRCE_DDS_SYNCT=0 PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=collapsed_building_rescue PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_NAME=x500_rescue_0 ../bin/px4
```

**Terminal 2 — Micro XRCE-DDS agent**, and the GCS heartbeat in a second tab
(PX4 will not arm without it):

```bash
MicroXRCEAgent udp4 -p 8888
```
```bash
python3 scripts/gcs_heartbeat.py
```

**Terminal 5 — VLM** (only for the survivor demo): the `ollama serve`
command above.

**Terminal 4 — ROS bridge, perception, flight controller.** Wait about 10 s
after PX4 has started. Start what the demo needs, each in its own tab or in
the background:

```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$PWD/src/uav_autonomy/config/camera_bridge_rescue.yaml
```
```bash
ros2 run uav_autonomy debris_depth_detector
```
```bash
ros2 run uav_autonomy debris_tracker
```
```bash
ros2 run uav_autonomy debris_predictor
```
```bash
ros2 run uav_autonomy debris_risk
```
```bash
ros2 run uav_autonomy debris_avoidance --ros-args -p enabled:=true
```
```bash
VLM_ENDPOINT=http://127.0.0.1:11434/v1 VLM_MODEL=gemma3:4b ros2 run uav_autonomy survivor_perception
```

Last, the flight controller (the only node that commands PX4):

```bash
ros2 run uav_autonomy flight_controller --ros-args -p waypoint_mission:=false -p target_altitude:=5.0
```

Falling debris for the avoidance demo, once the UAV hovers:

```bash
python3 scripts/sensor_study/spawn_study.py --plan results/phase6/plans/C_direct.json --out /tmp/releases.json --uav x500_rescue_0 --start-altitude 4.8
```

After `FLIGHT COMPLETE` stop everything and cold-start again before another
flight (PX4 will not re-arm in the same session).

## If nothing can be run live: recorded evidence

| What | File (relative to `ros2_ws/`) |
|---|---|
| Downward camera view of the collapsed building and manikin | `results/project_progress/figures/01_downward_camera_collapsed_building.jpg` |
| Detector view with falling debris (video) | `results/demo/detector_view_seed11.mp4` |
| Onboard camera with falling debris annotated (videos) | `results/sensor_study/A_vertical_offsets/analysis/real_camera_annotated.mp4` (also `B_heights_speeds`, `C_lateral_threats`) |
| Predicted trajectories | `figures/06_trajectory_prediction_view.png`, `07_prediction_error_vs_horizon.png` |
| Risk traces: safe pass, near miss, direct hit, lateral hit | `figures/08_collision_risk_traces.png` |
| Avoidance manoeuvres from the planner's own logs | `figures/09_…direct_threats.png`, `10_…multiple_threats.png`, `13_…cruise_4p4_m_s.png` |
| Avoidance failures | `figures/11_…hard_acceleration_transit.png`, `12_…lateral_throw.png` |
| Today's avoidance check | `figures/19_demo_check_2026-10-06_avoidance.png` |
| Survivor confirmed by the VLM (node's debug view) | `figures/15_survivor_confirmed_by_vlm.jpg` |
| Distractor scene, outdoor pass | `figures/16_survivor_distractor_scene.jpg`, `17_survivor_outdoor_pass.jpg` |
| Today's survivor check | `figures/20_demo_check_2026-10-06_survivor_candidate.jpg` |
| Every flight: PX4 log, node logs, scored JSON | the run folders under `results/phase*/` |

(`figures/` means `results/project_progress/figures/`.)

## Suggested 10-minute order

1. One slide: the boundary box at the top of this page.
2. Demo 1 (avoidance), then show `figures/09` and the baseline-vs-avoidance
   table in `METRICS_SUMMARY.md`.
3. Demo 2 (survivor + VLM), then `figures/15` and `figures/16`.
4. Two failures, honestly: `figures/11` (hard acceleration) and the VLM
   rejecting the straight-down view (`LIMITATIONS_AND_FAILURES.md`).
5. `PHASE_STATUS.md`: what remains.
