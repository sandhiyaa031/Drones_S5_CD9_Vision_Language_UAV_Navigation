# Phase 2: 3D debris perception (upward depth camera)

Code: `uav_autonomy/debris_detection.py` (pure Python core),
`uav_autonomy/debris_depth_detector.py` (ROS node).
Tests: `test/test_debris_detection.py`. Results: `results/phase2/`.

## Inputs and outputs

| | Topic | Type |
|---|---|---|
| in | `/depth_up/image_raw` | `sensor_msgs/Image`, `32FC1`, metres |
| in | `/depth_up/camera_info` | `sensor_msgs/CameraInfo` |
| in | `/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` (own pose estimate) |
| out | `/perception/debris/detections` | `vision_msgs/Detection3DArray`, frame `depth_up_link` |
| out | `/perception/debris/detections_local` | `vision_msgs/Detection3DArray`, frame `px4_local_ned` |
| out | `/perception/debris/debug_image` | `sensor_msgs/Image`, `bgr8`, 10 Hz |
| out | `/perception/debris/status` | `std_msgs/String`, JSON, every frame |

Each detection carries: position (centroid of the visible surface), size
(extent along the frame axes; "approximate size" = largest lateral extent),
confidence (`results[0].hypothesis.score`, class `debris`), and the array
header gives the timestamp and source frame. The status JSON additionally has
pixel count, pixel bounding box, range, border flag and rejection counts.

The node does not subscribe to any Gazebo pose topic, does not know release
positions, and does not publish to `/fmu/in/*`.

## Method

1. **Valid mask**: finite depth within `[0.5 m, 35 m]`. `+inf` (sky / beyond
   far clip) and `-inf` or `< 0.5 m` (closer than the near clip; UAV body or
   rotors) are dropped.
2. **Regions**: 8-connected components of the mask; a component is split
   where its sorted depths jump by more than 0.30 m, so two objects that
   overlap in the image at different ranges separate.
3. **3D**: every pixel is back-projected with the camera intrinsics,
   `p_s = (d, -(u-cx) d/fx, -(v-cy) d/fy)`; centroid and extents follow.
4. **Gates**: fewer than 12 pixels -> noise; lateral extent under 0.08 m ->
   too small; over 2.5 m -> building or ground surface.
5. **Local frame**: the vehicle pose is interpolated from odometry to the
   image stamp (`timestamp_sample`, see `sensor_integration.md`), then
   `p_ned = R(q) * FRD(p_s) + p_uav`. If no pose brackets the stamp within
   50 ms (30 ms extrapolation), only the sensor-frame array is published.
6. **Static filter** (local frame): a detection is dropped when an earlier
   detection at least 0.75 s old lies within 0.20 m of it and 60 % of the
   frames since then also had one there. Structure seen from a moving UAV is
   static in the local frame; falling debris is not.

Confidence is a heuristic in [0, 1], not a calibrated probability:
`0.30 + 0.35*min(1, pixels/150) + 0.20*(1 - depth_std/0.30) + 0.15` (the last
term only when the region does not touch the image border).

## Tests

21 unit tests on synthetic depth scenes: open sky, one and two boxes,
overlapping boxes at different depths, near returns, large surface, speckle,
out-of-range, NaN, border flag, back-projection, frame transforms (level and
yawed), pose interpolation and gaps, static filter (with dropped frames, and
under ego-motion), no-pose operation.

## Live validation (2026-10-04)

Four hover flights at 3 m in `collapsed_building_rescue`, NVIDIA rendering,
`UXRCE_DDS_SYNCT=0`, detector running live. Three used seeded random release
plans (seeds 11, 22, 33: 26 to 30 boxes each, 0.25 to 0.5 m, released from 6
to 13 m with up to 3 m/s initial speed, 0.25 to 1.2 s apart so several are
airborne together), one the lateral-throw plan. Boxes are physical Gazebo
bodies given a one-step impulse and never moved afterwards. Ground truth was
recorded separately and compared only afterwards
(`scripts/perception/evaluate_detections.py`).

| | seed 11 | seed 22 | seed 33 | lateral |
|---|---|---|---|---|
| Objects that entered the field of view while falling | 29 | 26 | 26 | 5 of 8 |
| Detected at least once | 29 | 26 | 26 | 5 |
| Object-frames expected / detected | 597 / 591 | 560 / 550 | 483 / 476 | 96 / 96 |
| Frame-level recall | 0.990 | 0.982 | 0.986 | 1.000 |
| Most objects in view at once | 3 | 3 | 2 | 1 |
| Detections with no ground-truth object within 0.35 m | 0 | 0 | 0 | 0 |
| Surface error, sensor frame, max (m) | 0.041 | 0.176 | 0.067 | 0.002 |
| Surface error, local frame, p95 / max (m) | 0.043 / 0.076 | 0.026 / 0.134 | 0.000 / 0.092 | 0.160 / 0.244 |
| Processing time per frame, median / max (ms) | 7.7 / 26 | 7.3 / 28 | 7.4 / 25 | 2.5 / 13 |
| Detector frame rate (Hz, simulation time) | 24.4 | 24.1 | 24.5 | 23.9 |
| Odometry pose available at image time | 99.9 % | 100 % | 100 % | 100 % |

Totals: 86 of 86 objects detected, 1713 of 1736 object-frames (98.7 %), 1852
detections, none unexplained. First detection came in the first frame the
object was fully in view for 85 of 86 objects.

"Surface error" is the distance from the detection point to the true box
surface (0 when it lies on it). The detection point is the centroid of the
visible face, so it sits about half a box height below the box centre (median
0.16 m). The local-frame figure also contains PX4's own pose error.
Surface error does not grow with object speed (correlation -0.06 to 0.07 up to
13 m/s), which is what a correct image timestamp predicts.

### Misses and false detections

* All missed object-frames examined were two boxes overlapping in the image at
  nearly the same range; they are merged into one region (one detection
  instead of two). One object was first detected 0.33 s late for this reason.
* 113 additional detections were of boxes only partly inside the image
  (flagged `touches_border`); their position is the centroid of the visible
  part, biased toward the image centre.
* On the ground before takeoff the roof slabs fill part of the view. They are
  rejected as `too_large` in every frame; no detection was published there.
* No detection came from the UAV body or rotors.

## Limits

* The simulated depth has no noise; real sensor noise would need the
  pixel-count and size gates retuned and would create false regions.
* The ROS depth stream delivers about 24 of Gazebo's 30 frames per second,
  with occasional gaps (95th percentile 68 to 100 ms, worst 0.26 to 0.53 s).
  The detector is not the bottleneck (7 ms per frame); the tracker must
  tolerate these gaps.
* Objects are seen only inside the 100 x 84 degree upward cone. Three boxes
  of the lateral plan approached too low to be seen while falling.
* Two objects overlapping at the same range are merged.
* A static object is published for its first 0.75 s; an object that hangs
  motionless for longer is dropped (by design: only moving debris).
* Position is the visible surface, not the object centre.
* In run `live_seed_33` a falling box struck the hovering UAV 32 s after
  arming; the safety monitor detected the impact and PX4 landed. Nothing
  avoids debris yet (that is Phase 6).

## Earlier Codex P2 files

`debris_perception.py` (image-plane frame differencing on the mono camera) is
not used by this pipeline: it gives no metric position and the downward camera
cannot see debris before it passes (see `results/sensor_study`). It is left in
place, unreferenced. `survivor_perception.py` is left for the survivor phase.
