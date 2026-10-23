# Sensor integration: downward mono + upward depth

All values below were measured on 2026-10-03/04 (see
`results/sensor_integration/`). PX4-Autopilot is not modified.

## Model hierarchy (project-owned, `src/uav_autonomy/models/`)

```
x500_rescue
├── include model://x500            (PX4, unmodified; base_link at model z 0.24)
├── include model://mono_cam        (PX4, unmodified) -> camera_link
│     pose in model frame (0, 0, 0.10), rpy (0, 1.5707, 0)   [same as PX4's
│     x500_mono_cam_down]; fixed joint CameraJoint base_link -> camera_link
└── include model://depth_cam_up    (project) -> depth_up_link
      pose in model frame (0, 0, 0.39), rpy (0, -1.5707963, 0)
      = base_link (0, 0, +0.15); fixed joint DepthUpJoint
```

Include poses are in the model frame, not the base_link frame. The rotor plane
is at model z 0.30, so the depth camera sits 0.09 m above it.

The script spawns `x500_rescue_0` through Gazebo's create service, then starts
PX4 with `PX4_GZ_STANDALONE=1 PX4_SIM_MODEL=gz_x500
PX4_GZ_MODEL_NAME=x500_rescue_0`. PX4 selects the stock X500 airframe (4001)
and attaches to the existing model.

## Frames

Gazebo sensor frame: +X optical axis, +Y image left, +Z image up.

| Sensor | optical axis | image left (u decreasing) | image up (v decreasing) |
|---|---|---|---|
| Upward depth (`depth_up_link`) | body +Z (up) | body +Y | body -X |
| Downward mono (`camera_link`) | body -Z (down) | body +Y | body +X |

Pixel model (both): `u = cx - fx * Y/X`, `v = cy - fy * Z/X`.
Back-projection for depth `d`: `p = (d, -(u-cx) d/fx, -(v-cy) d/fy)`.
Confirmed with rendered targets: a box toward body +X appears below the image
centre, a box toward body +Y appears left of it
(`v02_static_depth/static/orientation_check.png`).

## Topics

| Gazebo topic | Gazebo type | ROS topic | ROS type |
|---|---|---|---|
| `/uav/depth_up/image` | `gz.msgs.Image`, R_FLOAT32 | `/depth_up/image_raw` | `sensor_msgs/msg/Image`, `32FC1` |
| `/uav/depth_up/camera_info` | `gz.msgs.CameraInfo` | `/depth_up/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/uav/depth_up/image/points` | `gz.msgs.PointCloudPacked` | not bridged | |
| `/world/collapsed_building_rescue/model/x500_rescue_0/link/camera_link/sensor/camera/image` | `gz.msgs.Image`, RGB | `/camera/image_raw` | `sensor_msgs/msg/Image`, `rgb8` |
| `.../sensor/camera/camera_info` | `gz.msgs.CameraInfo` | `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` |

Bridge configuration: `config/camera_bridge_rescue.yaml` (Gazebo to ROS only).

## camera_info (upward depth)

640 x 480, frame `depth_up_link`, fx = fy = 268.5119, cx = 320, cy = 240,
distortion all zero. Expected from a 100 deg field of view: 268.512.

## Depth semantics

* Value: **axial** depth (distance along the optical axis), float32 metres.
  A flat target read the same value at the centre and all corners.
* Nothing in range (open sky or beyond far clip): `+inf`.
* Closer than the near clip (0.2 m): `-inf`. No NaN was observed.
* The far clip acts on the **ray length**, not the axial depth. With the
  first setting (20 m) a flat target at 15 m lost 42.7 % of its pixels,
  exactly those whose ray is longer than 20 m. The far clip is therefore
  **35 m**: flat targets are fully valid at 2, 5, 10 and 15 m (0 % invalid,
  centre and corners) and 99.7 % valid at 20 m (0.285 % invalid, the extreme
  corner pixels whose ray is 35.0 m). Errors stay below 1.4e-5 m.
* The simulated sensor has **no noise model**: errors are at float precision.
  A real depth camera would be far noisier; this is a simulation limit.

## Results

The table below was measured with the 20 m far clip on the NVIDIA GPU. After
a restart on 2026-10-04 the NVIDIA kernel module no longer loads (new kernel
7.0.0-34), so later runs render on the Intel GPU: with both cameras that gives
real-time factor 0.90 to 0.94 and 27.3 to 28.3 Hz, for the 20 m and the 35 m
clip alike (`v05_farclip_ab_*`, `v07_stability_5min_far35_intel`; memory flat
at 514 MiB over 5 minutes). The performance targets are not met on the Intel
GPU; the clip distance makes no measurable difference.

| Check | Result | Target |
|---|---|---|
| Static flat targets 2 / 5 / 10 / 15 m, median error | 1e-6 / 2e-6 / 5e-6 / 8e-6 m | < 0.10 m |
| Static, max error | 1e-6 / 4e-6 / 8e-6 / 1e-5 m | |
| Static, invalid pixels | 0 / 0 / 0 / 42.7 % (far clip, see above) | |
| Falling boxes: point-to-true-surface error, median | 0.5 mm (A, B), 3.5 mm (C) | < 0.25 m |
| Falling boxes: worst single point | 21 mm | |
| Falling boxes: box-centre error, median / max | 13 to 22 mm / 41 mm | < 0.25 m |
| Objects reconstructed | 21 of 24 (494 depth frames), speeds to 12.2 m/s | |
| Rotors or body in view | 0 of 501 hover frames without debris, 0 on the ground | |
| Real-time factor (5 min) | 0.965 to 0.992 | >= 0.95 |
| Gazebo depth / mono rate | 29.3 to 30.0 Hz each | >= 28 Hz |
| ROS depth / mono image rate | 24.3 to 25.2 Hz / 17.2 to 17.6 Hz | measured only |
| Gazebo server memory over 5 min | 840 -> 844 MiB | stable |
| GPU (RTX 4050, NVIDIA offload) | about 30 % mean, 44 % peak | |
| Hover flights with this model | 3 of 3 pass ULog verification | |

The three objects not seen by the depth camera approached from the side at a
shallow angle (below the field of view) or missed by 5.6 m. A zenith camera
does not see low-elevation threats.

Falling-box reconstruction used the ground-truth sensor pose so that it tests
the depth sensor alone. Error from PX4's own pose estimate is not included.

## Time synchronisation

Measured by matching each PX4 `sensor_combined` gyro sample to the Gazebo IMU
samples it was computed from (PX4 publishes the mean of two consecutive
Gazebo samples, identical to float32 precision; 99.99 % of samples matched).
For a matched pair, PX4 stamp minus Gazebo stamp is the true clock offset of
that sample with no transport delay in it. Data: `results/time_sync/`.

| | `UXRCE_DDS_SYNCT=1` (PX4 default) | `UXRCE_DDS_SYNCT=0` |
|---|---|---|
| PX4 stamp minus Gazebo time | about 1.79e9 s (wall-clock epoch) | 0.000 s |
| Drift, 5 min static | 21 s (first 10 s vs last 10 s) | 0 |
| Drift, hover flight (48 s) | 6.0 s | 0 |
| Timing error assuming a constant offset, p50 / p95 / p99, static | 5.6 s / 10.6 s / 12.3 s | 0 / 0 / 0 ms |
| Same, hover flight | 2.1 s / 3.9 s / 3.9 s | 0 / 0 / 0 ms |
| Matched samples (static / hover) | 74 223 / 12 097 | 74 730 / 11 849 |
| Real-time factor during capture | 0.93 / 0.87 | 0.93 / 0.85 |

With time synchronisation on, PX4 converts its stamps to the agent's wall
clock; since the simulator runs slower than real time the stamps drift away
from Gazebo time by `1 - real-time factor`. With it off, PX4's stamps are
PX4's own clock, which in lockstep simulation is exactly Gazebo's simulation
time, sample for sample.

With `UXRCE_DDS_SYNCT=0`:

* depth image stamp and PX4 `timestamp_sample` are on the same time base, so a
  depth frame at stamp `t` pairs with odometry interpolated at `t`;
* the nearest `vehicle_odometry` sample (100 Hz) is at most 8 to 12 ms from a
  depth frame (p50 4 ms), so interpolation is required, not nearest-sample;
* `vehicle_odometry.timestamp` is 0 to 8 ms later than `timestamp_sample`;
  use `timestamp_sample`;
* messages sent to PX4 still carry ROS wall-clock stamps. PX4's generated
  receive code clamps an incoming stamp to its own current time
  (`min(stamp - offset, now)`), so they are treated as "just arrived"; the
  offboard-loss and stale-setpoint checks keep working on arrival time. The
  unchanged `flight_controller.py` armed, flew and landed normally;
* not covered: this is a property of lockstep simulation. On real hardware
  PX4 and the companion clock are different clocks and need synchronising.

Adopted for simulation (2026-10-04): `run_flight_trial.sh` applies
`UXRCE_DDS_SYNCT=0` to every run by default and restores PX4's stored
parameter files at teardown, so the PX4 parameter store is never changed
permanently. Synchronise on `vehicle_odometry.timestamp_sample`, not
`.timestamp`. This is a common time base for lockstep simulation only; it is
not a real-hardware clock-synchronisation solution.

Both hover flights of this experiment (sync on and off) failed the ULog
"30 s altitude hold" check with 27.0 s and 26.4 s. Cause: the controller
holds for 30 s of wall time, and the simulator ran at 0.85 to 0.87 real-time
factor on the Intel GPU, so only 26 to 27 s of simulated time elapsed. All
other checks passed in both.

## Ground truth

`scripts/sensor_study/record.py` writes Gazebo `pose/info` (50 Hz, simulation
time) to CSV for offline evaluation. No perception or control node subscribes
to it.
