# Phase 3: 3D debris tracking

Code: `uav_autonomy/debris_tracking.py` (pure Python core),
`uav_autonomy/debris_tracker.py` (ROS node), messages in `uav_interfaces`.
Tests: `test/test_debris_tracking.py` (42). Results: `results/phase3/`.

Tracking only. No prediction horizon, risk, avoidance or commands.

## Interfaces

| | Topic | Type |
|---|---|---|
| in | `/perception/debris/detections_local` | `vision_msgs/Detection3DArray`, PX4 local NED |
| in | `/perception/debris/detections` | `vision_msgs/Detection3DArray`, sensor frame (border test only) |
| in | `/depth_up/camera_info` | `sensor_msgs/CameraInfo` (border test only) |
| out | `/perception/debris/tracks` | `uav_interfaces/DebrisTrackArray` |
| out | `/perception/debris/track_markers` | `visualization_msgs/MarkerArray` |
| out | `/perception/debris/tracks_debug` | `std_msgs/String` JSON (innovation, dt, covariance, timing) |

`DebrisTrack`: `id`, `stamp`, `position`, `velocity`, `covariance` (6x6),
`age`, `time_since_update`, `hits`, `consecutive_misses`, `size`,
`confidence`, `status` (TENTATIVE / CONFIRMED / COASTING).

Ego-motion contract (verified): the detector publishes local-frame positions
already transformed with the UAV pose interpolated to the image stamp
(`vehicle_odometry.timestamp_sample`). The tracker therefore works in the
local frame and needs no UAV state. It reads no Gazebo topic.

## Measurement model

`z = H x + w`, `H = [I3 0]`. `z` is the centroid of the **visible surface**
of the object as reported by the detector, not the box centre. The tracked
state is that surface point. Against ground truth it lies on the true box
surface (error about 0) and 0.16 to 0.31 m from the box centre. No
centre correction is applied.

Measurement noise: 0.10 m per axis (covers PX4 pose error and the change of
the visible face). Two cases use a larger value:

* partial view (region cut by the image border): 0.30 m; the centroid of
  the visible part is biased by up to the object size;
* merged region (size at least 1.4 times the track's size): half the region
  size; two touching objects are reported as one region.

## State and motion model

`x = [p_x, p_y, p_z, v_x, v_y, v_z]`, PX4 local NED.

    p' = p + v dt + 0.5 g dt^2 e_z        v' = v + g dt e_z
    Q_axis = sigma_a^2 [[dt^4/4, dt^3/2], [dt^3/2, dt^2]]

`dt` is the difference of sensor timestamps of consecutive processed frames;
nothing assumes a frame rate. `sigma_a = 3 m/s^2` on every axis. Initial
velocity 0 with standard deviation 8 m/s.

**Why gravity is in the model.** The same recorded detections were replayed
through a plain constant-velocity filter (`g = 0`, vertical `sigma_a = 12`)
and through the model above:

| Recorded run | CV: vertical velocity bias (median) | CV: velocity RMSE | with g: bias | with g: RMSE |
|---|---|---|---|---|
| seed 11 | -1.11 m/s | 1.13 m/s | -0.03 m/s | 0.17 m/s |
| seed 22 | -1.11 m/s | 1.11 m/s | -0.02 m/s | 0.14 m/s |
| seed 33 | -1.13 m/s | 1.25 m/s | -0.04 m/s | 0.50 m/s |
| lateral | -1.14 m/s | 1.32 m/s | +0.03 m/s | 0.42 m/s |

A constant-velocity filter systematically under-estimates the fall speed of
an accelerating body by about 1.1 m/s. Gravity is a known input, not an
estimated quantity; it removes the bias. `gravity_m_s2 = 0` restores the
constant-velocity filter. Drag is not modelled.

## Data association

1. Predict every track to the frame stamp.
2. Cost = squared Mahalanobis distance of the innovation; a pair is allowed
   only if it is below the chi-square gate (3 dof, 99.9 %: 16.27) and the
   innovation is within reach: `25 m/s * time_since_update + 0.6 m`.
3. Hungarian assignment (`scipy.optimize.linear_sum_assignment`).
4. Unassigned detections start tentative tracks.

## Track life cycle

* TENTATIVE -> CONFIRMED after 3 measurements of the fully visible object
  whose positions span at least 0.10 m. Static structure that enters the
  view (roof edges while climbing through the opening) is seen for a moment
  before the detector's static filter removes it; it does not move in the
  local frame, or is cut by the image border, so it is never confirmed.
* CONFIRMED -> COASTING when a frame has no measurement for it.
* Deleted: tentative after 2 missed frames; confirmed after 6 missed frames
  or 1.0 s without a measurement.
* A confirmed track whose prediction lies inside a merged region claimed by
  another track is held (no miss counted) for up to 0.6 s.

## Time handling

* Frames with a stamp equal to or older than the last processed one are
  dropped and counted (duplicate / out-of-order). There is no reprocessing of
  late measurements.
* A stream gap is one large `dt`: covariance grows through `Q`, the reach
  gate widens, the track is re-associated on the next frame. Deletion is
  evaluated only when a frame arrives, so a gap by itself deletes nothing.
* A gap longer than 2.0 s drops all tracks.

## Results

Provisional targets were set before the runs: persistent IDs, stable
multi-object tracking, position RMSE <= 0.25 m, physically reasonable
velocity, no systematic velocity bias, <= 1 ID switch per seeded run, stable
with 100 ms gaps, no divergence across a 0.5 s gap.

Live flights (final tracker node, NVIDIA rendering, `UXRCE_DDS_SYNCT=0`),
scored afterwards against Gazebo truth. Errors are for measurement-updated
states; coasting states are listed separately.

| Run | Objects confirmed / seen | Position RMSE to true surface | to true centre | Velocity RMSE | Vertical bias (median) | ID switches | False confirmed |
|---|---|---|---|---|---|---|---|
| single objects | 8 / 8 | 0.000 m | 0.170 m | 0.07 m/s | -0.03 m/s | 0 | 0 |
| heights and speeds | 7 / 8 | 0.000 m | 0.186 m | 0.11 m/s | -0.03 m/s | 0 | 0 |
| lateral throws | 4 / 8 | 0.073 m | 0.308 m | 0.43 m/s | +0.02 m/s | 0 | 0 |
| seed 11 (29 objects) | 29 / 29 | 0.017 m | 0.220 m | 0.24 m/s | -0.03 m/s | 0 | 1 |
| seed 44 (26 objects) | 25 / 26 | 0.008 m | 0.202 m | 0.12 m/s | -0.04 m/s | 0 | 2 |
| seed 55 (26 objects) | 26 / 26 | 0.005 m | 0.180 m | 0.16 m/s | -0.03 m/s | 0 | 0 |
| UAV climbing (10 objects) | 10 / 10 | 0.013 m | 0.239 m | 0.08 m/s | -0.02 m/s | 0 | 0 |

* ID switches use the CLEAR MOT convention (a truth-to-track pairing is kept
  while it stays within 0.6 m). Re-solving the pairing from scratch every
  frame gives 4, 1 and 1 for seeds 11, 44 and 55: two boxes falling within
  0.5 m of each other have tracks that are each within 0.6 m of both.
* Track continuity (confirmed frames / frames from first association to
  impact): mean 0.55 to 0.86. It includes the 2 to 4 frames before
  confirmation and the time after the object has left the field of view.
* Confirmation delay: median 0.07 to 0.18 s, worst 0.50 s.
* Frame gaps while tracking: up to 0.46 s. After gaps of 90 ms or more the
  position error stayed at or below 0.19 m and the velocity error at or
  below 2.0 m/s (seed 11); for the longest gaps (0.30 to 0.46 s) 0.00 m and
  0.03 to 0.20 m/s.
* Processing: median 0.12 to 0.73 ms, maximum 2.7 ms per frame.
* Innovation consistency (NIS): mean 0.03 to 0.13 against an expected 3. The
  stated covariance is much larger than the actual error: the simulated
  measurements are almost noise-free while 0.10 m is assumed.

### Worst cases

* Coasting: once an object passes below the UAV it is no longer seen. The
  track keeps falling ballistically for up to 6 frames while the real object
  may already have hit the ground: velocity errors of 6 to 15 m/s and
  position errors up to 0.38 m in those frames.
* Two boxes falling within about 0.5 m of each other are one merged
  detection for part of the fall. IDs were kept, but 3 duplicate confirmed
  tracks appeared in two runs.
* Objects only ever seen cut by the image border are followed as TENTATIVE
  tracks and not confirmed: 4 of 8 in the lateral run, 1 in each of two other
  runs. Their state is published with status TENTATIVE.
* In run seed 44 a box struck the hovering UAV (safety monitor triggered,
  PX4 landed). Tracking is evaluated up to that point.

## Limits

* Measurement = visible surface, not the centre (about 0.2 m).
* No aerodynamic drag, no bounce model; a track is meaningless after impact.
* Late (out-of-order) measurements are discarded, not fused.
* Side-entering objects that never become fully visible stay tentative.
* Covariance is conservative, not calibrated.
* Validated in simulation with a noise-free depth sensor.
