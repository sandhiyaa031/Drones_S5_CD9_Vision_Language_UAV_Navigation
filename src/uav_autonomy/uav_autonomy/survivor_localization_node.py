#!/usr/bin/env python3
"""ROS 2 node: 3D position of survivor candidates (Phase 8).

Subscribes (the vehicle's own data only)
    /perception/survivor/detections   uav_interfaces/SurvivorDetectionArray
                                      (pixels, Phase 7)
    /camera/camera_info               sensor_msgs/CameraInfo
    /fmu/out/vehicle_odometry         px4_msgs/VehicleOdometry

Publishes
    /perception/survivor/locations            uav_interfaces/
                                              SurvivorLocationArray,
                                              frame px4_local_ned
    /perception/survivor/location_markers     visualization_msgs/MarkerArray
    /perception/survivor/localization_status  std_msgs/String, JSON

The stamp of every output is the camera frame's own stamp. The vehicle pose
is interpolated to that time (UXRCE_DDS_SYNCT=0 puts PX4 timestamp_sample on
the same clock in lockstep simulation).

This node never publishes to /fmu/in/*, never reads Gazebo ground truth and
does not change the flight path. Method: survivor_localization.py.
"""

import json
import math

import numpy as np
from px4_msgs.msg import VehicleOdometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
    qos_profile_sensor_data)
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from uav_interfaces.msg import (
    SurvivorDetectionArray, SurvivorLocation, SurvivorLocationArray)
from visualization_msgs.msg import Marker, MarkerArray

from uav_autonomy.debris_detection import PoseBuffer, is_finite_pose
from uav_autonomy.survivor_localization import (
    CameraModel,
    LOC_LOCALIZED,
    LocalizerParams,
    SurvivorLocalizer,
)

LOCAL_FRAME = 'px4_local_ned'


def to_time(stamp: float):
    """Float seconds -> builtin_interfaces/Time fields."""
    sec = int(math.floor(stamp))
    return sec, int(round((stamp - sec) * 1e9)) % 1000000000


class SurvivorLocalizationNode(Node):
    """Wrap SurvivorLocalizer with ROS interfaces."""

    def __init__(self):
        """Declare parameters and create publishers and subscribers."""
        super().__init__('survivor_localization')
        defaults = LocalizerParams()
        values = {}
        for name in defaults.__dataclass_fields__:
            self.declare_parameter('loc.' + name, getattr(defaults, name))
            values[name] = self.get_parameter('loc.' + name).value
        self.params = LocalizerParams(**values)
        # camera position in the body frame (FRD) [m]
        self.declare_parameter('camera_offset_body', [0.0, 0.0, 0.14])
        # height of the body origin above the ground when landed [m]
        self.declare_parameter('body_height_on_ground_m', 0.23)
        # local z of the ground; NaN = measure it before takeoff
        self.declare_parameter('ground_down', float('nan'))
        self.offset = [float(v) for v in self.get_parameter(
            'camera_offset_body').value]
        self.body_height = float(self.get_parameter(
            'body_height_on_ground_m').value)
        self.ground_down = float(self.get_parameter('ground_down').value)
        self.ground_samples = []
        self.ground_fallback_warned = False

        self.localizer = None
        self.poses = PoseBuffer()
        self.frames = 0

        self.pub = self.create_publisher(
            SurvivorLocationArray, '/perception/survivor/locations', 10)
        self.pub_markers = self.create_publisher(
            MarkerArray, '/perception/survivor/location_markers', 2)
        self.pub_status = self.create_publisher(
            String, '/perception/survivor/localization_status', 10)

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=20)
        self.create_subscription(
            CameraInfo, '/camera/camera_info', self.on_info,
            qos_profile_sensor_data)
        self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.on_odometry,
            px4_qos)
        self.create_subscription(
            SurvivorDetectionArray, '/perception/survivor/detections',
            self.on_detections, 10)
        self.get_logger().info(
            'Survivor localization ready; waiting for camera_info.')

    def on_info(self, msg):
        """Build the localizer once intrinsics are known."""
        if self.localizer is not None:
            return
        cam = CameraModel(fx=msg.k[0], fy=msg.k[4], cx=msg.k[2], cy=msg.k[5],
                          width=msg.width, height=msg.height,
                          offset_body=tuple(self.offset))
        self.localizer = SurvivorLocalizer(cam, self.params, 0.0)
        self.get_logger().info(
            f'Intrinsics: fx={cam.fx:.2f} fy={cam.fy:.2f} cx={cam.cx:.1f} '
            f'cy={cam.cy:.1f} {cam.width}x{cam.height}')

    def on_odometry(self, msg):
        """Store the vehicle pose; measure the ground level while landed."""
        if msg.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            return
        if not is_finite_pose(msg.position, msg.q):
            return
        position = [float(v) for v in msg.position]
        self.poses.add(msg.timestamp_sample * 1e-6, position,
                       [float(v) for v in msg.q])
        if math.isnan(self.ground_down) and len(self.ground_samples) < 50:
            speed = math.sqrt(sum(float(v) ** 2 for v in msg.velocity))
            if speed < 0.15:
                self.ground_samples.append(position[2])
                if len(self.ground_samples) == 50:
                    self.ground_down = float(np.median(
                        self.ground_samples)) + self.body_height
                    self.get_logger().info(
                        f'Ground level measured before takeoff: local z = '
                        f'{self.ground_down:.2f} m')
            elif not self.ground_samples and \
                    not self.ground_fallback_warned:
                # started while already moving: the local origin is where
                # the estimator was initialised, on the ground
                self.ground_fallback_warned = True
                self.ground_down = self.body_height
                self.get_logger().warn(
                    'Vehicle already moving at start: ground level assumed '
                    f'at local z = {self.ground_down:.2f} m')

    def on_detections(self, msg):
        """Update the estimates with one camera frame and publish."""
        if self.localizer is None:
            return
        if math.isnan(self.ground_down):
            return          # still measuring the ground level
        self.localizer.ground_down = self.ground_down
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose = self.poses.at(stamp)
        detections = [{
            'track_id': d.track_id, 'center_u': d.center_u,
            'center_v': d.center_v, 'bbox_width': d.bbox_width,
            'bbox_height': d.bbox_height,
            'touches_border': d.touches_border, 'measured': d.measured,
            'semantic_state': d.semantic_state} for d in msg.detections]
        updated = {e.entity_id for e in self.localizer.update(
            stamp, detections, pose)}
        self.frames += 1

        out = SurvivorLocationArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = LOCAL_FRAME
        out.ground_down = float(self.ground_down)
        out.pose_available = pose is not None
        summary = []
        for entity in self.localizer.reported():
            loc = SurvivorLocation()
            sec, nanosec = to_time(entity.last_stamp)
            loc.header.stamp.sec, loc.header.stamp.nanosec = sec, nanosec
            loc.header.frame_id = LOCAL_FRAME
            loc.survivor_id = entity.entity_id
            loc.track_id = entity.last_track_id
            loc.position.x, loc.position.y, loc.position.z = (
                float(v) for v in entity.position)
            loc.covariance = [float(v) for v in self.localizer
                              .reported_covariance(entity).ravel()]
            loc.horizontal_sigma = self.localizer.reported_sigma(entity)
            loc.height_above_ground = float(
                self.ground_down - entity.position[2])
            loc.localization_state = self.localizer.state_of(entity)
            loc.observations = entity.observations
            loc.baseline = float(entity.baseline_m)
            loc.min_range = float(entity.min_range_m)
            sec, nanosec = to_time(entity.first_stamp)
            loc.first_seen.sec, loc.first_seen.nanosec = sec, nanosec
            loc.updated = entity.entity_id in updated
            loc.semantic_state = entity.semantic_state
            loc.vlm_confirmed = entity.confirmed
            loc.vlm_rejected = entity.vlm_rejected
            out.locations.append(loc)
            summary.append({
                'id': entity.entity_id,
                'ned': [round(float(v), 3) for v in entity.position],
                'sigma_h': round(loc.horizontal_sigma, 3),
                'state': int(loc.localization_state),
                'obs': entity.observations,
                'baseline': round(entity.baseline_m, 2),
                'confirmed': entity.confirmed})
        self.pub.publish(out)
        self.pub_status.publish(String(data=json.dumps({
            'stamp': round(stamp, 3), 'pose': pose is not None,
            'ground_down': round(self.ground_down, 3),
            'locations': summary, 'skipped': self.localizer.skipped},
            separators=(',', ':'))))
        if self.frames % 5 == 0:
            self.publish_markers(out)

    def publish_markers(self, array):
        """Spheres at the estimates, scaled by the horizontal 1-sigma."""
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for loc in array.locations:
            marker = Marker()
            marker.header = array.header
            marker.ns = 'survivor_location'
            marker.id = int(loc.survivor_id)
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = loc.position
            marker.pose.orientation.w = 1.0
            size = max(0.3, 2.0 * float(loc.horizontal_sigma))
            marker.scale.x = marker.scale.y = size
            marker.scale.z = 0.3
            marker.color.a = 0.8
            if loc.vlm_confirmed:
                marker.color.g = 1.0
            elif loc.localization_state == LOC_LOCALIZED:
                marker.color.r, marker.color.g = 1.0, 0.8
            else:
                marker.color.r, marker.color.g = 1.0, 0.4
            markers.markers.append(marker)
        self.pub_markers.publish(markers)


def main(args=None):
    """Run the node."""
    rclpy.init(args=args)
    node = SurvivorLocalizationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
