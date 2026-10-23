#!/usr/bin/env python3
"""ROS 2 node: what slam_toolbox needs from the vehicle (village missions).

Subscribes
    /fmu/out/vehicle_odometry   px4_msgs/VehicleOdometry (NED, FRD)
    /uav/scan_raw               sensor_msgs/LaserScan, body plane

Publishes
    TF odom -> base_level       the vehicle's odometry as a planar pose
                                (ENU x east, y north; yaw only)
    /uav/scan                   sensor_msgs/LaserScan in frame base_level:
                                the raw scan with roll and pitch removed and
                                returns away from the flight plane dropped
    /slam/odometry_pose         geometry_msgs/PoseStamped, the same planar
                                pose (for recording)
    /slam/bridge_status         std_msgs/String, JSON

Scans are forwarded only while the vehicle is within `altitude_band_m` of
`mapping_altitude_m`, so that the map is one horizontal slice of the world.

`odom_scale_error` and `odom_yaw_drift_deg_per_min` deliberately degrade the
odometry handed to SLAM (experiments only; both zero by default). PX4's own
estimate, which flies the vehicle, is never altered.

This node does not command PX4 and reads no simulator ground truth.
"""

import json
import math

from geometry_msgs.msg import PoseStamped, TransformStamped
from px4_msgs.msg import VehicleOdometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
    qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from uav_autonomy.debris_detection import PoseBuffer
from uav_autonomy.slam_geometry import (
    DriftModel, enu_flu_rotation, level_scan, ned_to_enu_xy, yaw_enu)


class SlamBridge(Node):
    """Odometry TF and levelled scans for slam_toolbox."""

    def __init__(self):
        """Declare parameters and create publishers and subscribers."""
        super().__init__('slam_bridge')
        self.declare_parameter('mapping_altitude_m', 6.0)
        self.declare_parameter('altitude_band_m', 0.7)
        self.declare_parameter('max_plane_offset_m', 0.8)
        self.declare_parameter('odom_scale_error', 0.0)
        self.declare_parameter('odom_yaw_drift_deg_per_min', 0.0)
        self.altitude = float(self.get_parameter('mapping_altitude_m').value)
        self.band = float(self.get_parameter('altitude_band_m').value)
        self.offset = float(self.get_parameter('max_plane_offset_m').value)
        self.drift = DriftModel(
            float(self.get_parameter('odom_scale_error').value),
            math.radians(float(self.get_parameter(
                'odom_yaw_drift_deg_per_min').value)) / 60.0)
        self.poses = PoseBuffer()
        self.ground_z = None
        self.scans_in = self.scans_out = self.dropped = 0
        self.tf = TransformBroadcaster(self)
        self.pub_scan = self.create_publisher(LaserScan, '/uav/scan', 10)
        self.pub_pose = self.create_publisher(
            PoseStamped, '/slam/odometry_pose', 10)
        self.pub_status = self.create_publisher(
            String, '/slam/bridge_status', 10)
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=20)
        self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.on_odometry,
            px4_qos)
        self.create_subscription(
            LaserScan, '/uav/scan_raw', self.on_scan, qos_profile_sensor_data)
        self.create_timer(1.0, self.on_timer)
        self.get_logger().info(
            f'SLAM bridge ready; mapping slice at {self.altitude:.1f} m '
            f'+/- {self.band:.1f} m; odometry degradation: scale '
            f'{self.drift.scale - 1.0:+.3f}, yaw '
            f'{math.degrees(self.drift.rate) * 60:.2f} deg/min')

    def on_odometry(self, msg):
        """Publish the planar odometry pose and its transform."""
        if msg.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            return
        position = [float(v) for v in msg.position]
        q = [float(v) for v in msg.q]
        if not all(math.isfinite(v) for v in position + q):
            return
        stamp = msg.timestamp_sample * 1e-6
        self.poses.add(stamp, position, q)
        if self.ground_z is None:
            self.ground_z = position[2]      # the node starts on the ground
        x, y = ned_to_enu_xy(position)
        yaw = yaw_enu(enu_flu_rotation(q))
        x, y, yaw = self.drift.update(stamp, x, y, yaw)
        sec = int(stamp)
        nanosec = int((stamp - sec) * 1e9)
        tf = TransformStamped()
        tf.header.stamp.sec, tf.header.stamp.nanosec = sec, nanosec
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_level'
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.z = math.sin(yaw / 2.0)
        tf.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf.sendTransform(tf)
        pose = PoseStamped()
        pose.header = tf.header
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.orientation = tf.transform.rotation
        self.pub_pose.publish(pose)

    def on_scan(self, msg):
        """Level one scan and forward it if the vehicle is in the slice."""
        self.scans_in += 1
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose = self.poses.at(stamp)
        if pose is None or self.ground_z is None:
            return
        height = self.ground_z - float(pose[0][2])
        if abs(height - self.altitude) > self.band:
            return
        ranges, dropped = level_scan(
            msg.ranges, msg.angle_min, msg.angle_increment,
            enu_flu_rotation(pose[1]), height, msg.range_min, msg.range_max,
            max_plane_offset_m=self.offset)
        self.dropped += dropped
        out = LaserScan()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'base_level'
        out.angle_min, out.angle_max = msg.angle_min, msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment, out.scan_time = 0.0, msg.scan_time
        out.range_min, out.range_max = msg.range_min, msg.range_max
        out.ranges = [float(v) for v in ranges]
        self.pub_scan.publish(out)
        self.scans_out += 1

    def on_timer(self):
        """Status for recording."""
        self.pub_status.publish(String(data=json.dumps({
            'scans_in': self.scans_in, 'scans_out': self.scans_out,
            'returns_dropped': self.dropped})))


def main(args=None):
    """Run the node."""
    rclpy.init(args=args)
    node = SlamBridge()
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
