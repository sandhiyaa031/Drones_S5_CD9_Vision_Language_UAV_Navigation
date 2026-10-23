# P2 camera perception

## Node graph

`debris_perception` consumes `/camera/image_raw` with sensor-data QoS and
publishes:

- `/perception/debris/tracks` (`std_msgs/msg/String`, compact JSON in
  `image_pixels` coordinates)
- `/perception/debris/debug_image` (`sensor_msgs/msg/Image`)

Track JSON includes `object_id`, image timestamp, pixel center and bounding box,
pixel velocity, confidence, hit count, and constant-velocity predictions at
0.5, 1.0, and 2.0 seconds. `estimated_position_3d_m` is `null`: the mono camera
does not provide metric depth. The current confidence is a deterministic
appearance/shape score and is not a calibrated probability.

`survivor_perception` independently publishes orange-color candidates on
`/perception/survivor/candidates`. It sends candidate crops to
`/perception/survivor/verification` through the optional asynchronous VLM path
when `vlm_endpoint` is configured. Candidate coordinates are pixels, not
physical positions.

Launch both nodes with:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch uav_autonomy p2_perception.launch.py
```

The launch argument `camera_topic` defaults to `/camera/image_raw`;
`vlm_endpoint` defaults to an empty value (candidate detection remains active,
VLM verification does not).

## Runtime validation record

Date: 2026-10-02. The proven flight-controller source was not changed, and the
flight controller was not launched during these checks.

PX4/Gazebo command:

```bash
PX4_GZ_WORLD=collapsed_building_complex make px4_sitl gz_x500_mono_cam_down
```

The live Gazebo camera topic was:

```text
/world/collapsed_building_complex/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image
```

Bridge command (ROS remap to the project's camera topic):

```bash
ros2 run ros_gz_bridge parameter_bridge \
  '/world/collapsed_building_complex/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image' \
  --ros-args -r __node:=camera_bridge \
  -r /world/collapsed_building_complex/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image:=/camera/image_raw
```

Observed ROS interface: `/camera/image_raw`, `sensor_msgs/msg/Image`,
1280x960, `rgb8`, frame `camera_link`. Measured rate in this runtime was
approximately 2.5-3 Hz, below the earlier nominal camera rate.

Seeded debris command:

```bash
ros2 run uav_autonomy continuous_debris_spawner \
  --count 6 --seed 42 --interval-min 1.0 --interval-max 1.0 \
  --height-min 4.5 --height-max 5.5 --lifetime 8.0
```

The spawner reported six objects spawned and six removed, with zero remaining;
reported wall duration was 58.4 seconds. Its per-object log confirmed an
`APPLIED` physics impulse. This confirms scenario execution and cleanup, but
does not by itself confirm that the camera saw the falling objects.

One observed `/perception/debris/tracks` record was:

```json
{"object_id":26,"timestamp_ns":41748000000,"pixel_position":{"x":1231.6,"y":348.12},"bbox_xywh_px":[1222,334,22,26],"velocity_px_s":{"x":0.83,"y":-0.66},"confidence":0.916,"hits":2}
```

The record was near the image edge and was not verified as a debris object.
The captured camera view was obstructed/poorly framed. The survivor candidate
topic returned an empty candidate list during the sample. No VLM endpoint was
configured, so no VLM response was obtained.

Offline synthetic smoke checks passed for a translating rectangular region
(stable ID, positive pixel velocity, all three prediction horizons) and an
orange human-shaped blob (one candidate bounding box). These checks validate
the basic algorithm plumbing only; they do not replace camera-scene validation.

PX4 telemetry at the time showed `pre_flight_checks_pass=false` and
`gcs_connection_lost=false`; PX4 startup output reported `system power
unavailable`. The aircraft was not armed or flown. Consequently, seeded debris
tracking, survivor visibility, and VLM verification remain unvalidated in the
live scene. Do not use these nodes for avoidance or flight decisions.
