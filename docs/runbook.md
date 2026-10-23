# Runbook: cold start, flight trial, verification

Verified on 2026-10-03 with PX4 `d983b64`, px4_msgs `f719702`, ROS 2 Jazzy,
Gazebo Sim 8.11.0, Ubuntu 24.04, RTX 4050 (driver 595.84, PRIME on-demand).

All paths are relative to `ros2_ws/` unless stated. `PX4-Autopilot/` must sit
next to `ros2_ws/` and already be built (`build/px4_sitl_default/bin/px4`).

## One-command trial (recommended)

```bash
cd ~/Desktop/drones/ros2_ws
NVIDIA_OFFLOAD=1 CAMERA_BRIDGE=1 scripts/run_flight_trial.sh results/my_run hover
```

This cold-starts everything in order, flies, tears everything down, copies the
PX4 ULog into the run folder and verifies it. Exit code 0 means every check in
`verification.json` passed. Add `HEADLESS=0` to open the Gazebo GUI.

Modes: `hover` (3 m, 30 s hold, land), `waypoint`, `none` (simulation only).
Do not use `waypoint` in this world: see "Known limits".

## Manual cold start (one terminal each, in this order)

Every terminal first needs ROS sourced, otherwise the `gz` command is not found:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Desktop/drones/ros2_ws/install/setup.bash
```

1. Gazebo server with the project world and project plugin list:

```bash
cd ~/Desktop/drones/ros2_ws
source ../PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh
export GZ_SIM_SERVER_CONFIG_PATH=$PWD/src/uav_autonomy/config/gz_server.config
export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
gz sim -r -s src/uav_autonomy/worlds/collapsed_building_rescue.sdf
```

2. (optional) Gazebo GUI: `gz sim -g`

3. PX4 SITL attached to the running world. `PX4_GZ_WORLD` is required: the
   X500 airframe otherwise defaults it to `default` and PX4 waits forever.

```bash
cd ~/Desktop/drones/PX4-Autopilot/build/px4_sitl_default/rootfs
PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=collapsed_building_rescue \
  PX4_SIM_MODEL=gz_x500_mono_cam_down ../bin/px4
```

4. XRCE-DDS agent: `MicroXRCEAgent udp4 -p 8888`

5. GCS heartbeat (required to arm; no PX4 parameter is changed):

```bash
python3 ~/Desktop/drones/ros2_ws/scripts/gcs_heartbeat.py
```

6. Camera bridge (image and camera_info):

```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:=$HOME/Desktop/drones/ros2_ws/src/uav_autonomy/config/camera_bridge.yaml
```

7. Wait about 10 s for the EKF, then the flight controller. It is the only
   node that publishes to `/fmu/in/*`. Never start `rescue_mission.py`.

```bash
ros2 run uav_autonomy flight_controller --ros-args -p waypoint_mission:=false
```

After `FLIGHT COMPLETE`, stop everything and cold-start again before the next
flight: PX4 stays in Offboard with the setpoint stream lost, and preflight will
not pass until SITL is restarted.

## Verifying a flight from the PX4 log

```bash
python3 scripts/verify_ulog.py results/my_run/flight.ulg
python3 scripts/verify_ulog.py --mode waypoint "--waypoints=-4,-4;-4,-5.5;0,0" flight.ulg
```

Checks: armed, Offboard while armed, reached target altitude, held the altitude
band for the hold time (hover), each waypoint reached (waypoint), landed,
disarmed, no excursion, and no airborne collision impact (horizontal
acceleration above 30 m/s^2 while above 0.5 m). Uses only PX4's own log.

## Measuring simulator and camera performance

With the simulation running:

```bash
python3 scripts/measure_sim.py --duration 20
```

Reports real-time factor, Gazebo-native camera rate, ROS image and camera_info
rates, PX4 status/position rates and GPU utilisation over one common window.

## UAV model (since the sensor integration)

`run_flight_trial.sh` now spawns the project model `x500_rescue` (downward
mono + upward depth) and lets PX4 attach to it; the camera bridge then uses
`config/camera_bridge_rescue.yaml`. Set `PROJECT_MODEL=none` to get the
Phase 0/1 configuration (PX4's own `gz_x500_mono_cam_down`). The manual steps
above describe that Phase 0/1 configuration; for the project model replace
step 3 by the spawn and attach commands in `docs/sensor_integration.md`.

## Time base (simulation only)

`run_flight_trial.sh` starts PX4 with `UXRCE_DDS_SYNCT=0` by default
(`PX4_PARAM_UXRCE_DDS_SYNCT=0` in PX4's environment for that run). PX4's
stored parameter files are copied aside before the run and put back at
teardown, also after an interrupted run, so the PX4 parameter store is not
changed permanently. `UXRCE_SYNCT=px4` leaves PX4's own setting.

Effect: PX4 timestamps on ROS equal Gazebo simulation time (measured error
0 ms at p50/p95/p99, see `docs/sensor_integration.md`). Pair sensor frames
with `vehicle_odometry.timestamp_sample`, never with `.timestamp`.

This gives a common simulation-time basis in lockstep simulation. It is not
a clock-synchronisation solution for real hardware.

For a manual start add `PX4_PARAM_UXRCE_DDS_SYNCT=0` in front of the PX4
command in step 3, and restore `parameters.bson` / `parameters_backup.bson`
in `PX4-Autopilot/build/px4_sitl_default/rootfs/` afterwards.

## NVIDIA driver state (required for the performance targets)

Required: `nvidia-smi` works, i.e. the NVIDIA kernel module is loaded for the
running kernel. Known-good state (2026-10-03): kernel `7.0.0-30-generic`,
packages `nvidia-driver-595-open 595.84` and
`linux-modules-nvidia-595-open-7.0.0-30-generic`, PRIME mode `on-demand`.

State found on 2026-10-04: the machine booted kernel `7.0.0-34-generic`. The
prebuilt NVIDIA modules are installed only for `7.0.0-30`
(`/lib/modules/7.0.0-30-generic/kernel/nvidia-595-open/`), there is none for
`7.0.0-34`, DKMS is not installed and Secure Boot is enabled. So the module
cannot load and Gazebo renders on the Intel GPU (real-time factor 0.90 to
0.94 with both cameras, below target).

Two ways back, both to be done by the machine owner:

* boot kernel `7.0.0-30-generic` from the GRUB "Advanced options" menu
  (nothing to install), or
* install the matching module package for the new kernel, expected name
  `linux-modules-nvidia-595-open-7.0.0-34-generic`, then reboot.

Check: `nvidia-smi` prints the RTX 4050, and after a run
`grep GL_RENDERER results/<run>/ogre2.log` shows "NVIDIA GeForce RTX 4050".

## Acceptance benchmark (final two-camera configuration)

```bash
cd ~/Desktop/drones/ros2_ws
scripts/benchmark_two_camera.sh results/sensor_integration/v09_benchmark_nvidia 320
```

Runs the simulation for the given seconds with both cameras, NVIDIA offload,
the ROS bridge and `UXRCE_DDS_SYNCT=0`; samples rates, real-time factor, GPU
and memory every minute; records and analyses timestamps; writes
`benchmark.json`. Exit code 0 only if every check passes: rendered on NVIDIA,
real-time factor >= 0.95, both Gazebo streams >= 28 Hz, Gazebo alive to the
end, memory growth <= 50 MiB, timestamp error p99 <= 1 ms. It refuses to run
when `nvidia-smi` fails.

## Debris avoidance flights (Phase 6)

```bash
scripts/avoidance/run_scenario.sh C_direct baseline   # planner disabled
scripts/avoidance/run_scenario.sh C_direct avoid      # planner acting
scripts/avoidance/run_all.sh                          # every scenario, both
PYTHONPATH=src/uav_autonomy python3 scripts/avoidance/compare.py
```

`run_flight_trial.sh` takes `AVOIDANCE=on|baseline|off`, `AVOIDANCE_ARGS`,
`CONTROLLER_ARGS` (e.g. `-p target_altitude:=5.0`) and `VERIFY_ARGS`.
The planner only proposes setpoints on
`/perception/debris/avoidance_command`; `flight_controller` validates them
and remains the only node publishing to `/fmu/in/*`. See
`docs/debris_avoidance.md`.

Do not edit files under `src/uav_autonomy/uav_autonomy/` while flights are
running: the workspace is installed with `--symlink-install`, so nodes
started later would pick up the edited code.

## Known problems and their causes (all measured, see results/)

| Symptom | Cause | Handling |
|---|---|---|
| Arming blocked: "No connection to the GCS" | X500 airframe sets `NAV_DLL_ACT=2` | run `gcs_heartbeat.py`; parameter untouched |
| Real-time factor about 0.4, camera 3 to 13 Hz, controller reports stale telemetry | old `collapsed_building_complex` world included a non-static `rescue_building` model whose links interpenetrate static geometry | use `collapsed_building_rescue.sdf` |
| Gazebo server dies after about 100 s with a camera model | PX4's `custom::GstCameraSystem` plugin grows memory about 107 MiB/s | project `gz_server.config` does not load it |
| `gz` command not found, PX4 "Timed out waiting for Gazebo world" | ROS environment not sourced, or `PX4_GZ_WORLD` missing in standalone mode | see steps above |
| Second flight will not arm | Offboard signal lost after the previous flight | cold-start again |
| A previous run left processes behind | interrupted session | `run_flight_trial.sh` refuses to start and lists them |
| Rendering on the Intel iGPU | PRIME is on-demand | `NVIDIA_OFFLOAD=1` (the three `__NV/__GLX/__EGL` variables) |

## Known limits

* The world is collision-safe only for vertical flight through the roof opening
  above the spawn point. Leaving the opening at 3 m hits the roof slabs
  (measured: `results/phase0/final_config/waypoint_01`). The hard-coded
  waypoint route must not be flown here.
* `flight_controller.py` does not detect a collision or an estimator jump; it
  reported `FLIGHT COMPLETE` on the flight above. Only the ULog check caught it.
* The laptop was in the `power-saver` power profile during all measurements.
