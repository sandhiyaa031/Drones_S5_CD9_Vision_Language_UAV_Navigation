#!/usr/bin/env python3
"""ROS 2 node: 3D debris detections from the upward depth camera.

Subscribes (sensor data only)
    /depth_up/image_raw     sensor_msgs/Image, 32FC1 metres
    /depth_up/camera_info   sensor_msgs/CameraInfo
    /fmu/out/vehicle_odometry   px4_msgs/VehicleOdometry (own pose estimate)

Publishes
    /perception/debris/detections        vision_msgs/Detection3DArray,
                                         frame depth_up_link
    /perception/debris/detections_local  vision_msgs/Detection3DArray,
                                         frame px4_local_ned (when the pose
                                         at the image time is available)
    /perception/debris/debug_image       sensor_msgs/Image, bgr8
    /perception/debris/status            std_msgs/String, JSON per frame

All output stamps are the depth image's own stamp. With UXRCE_DDS_SYNCT=0 in
lockstep simulation, PX4 timestamp_sample is on the same clock, so the pose
is interpolated to exactly that time.

This node never publishes to /fmu/in/* and never reads Gazebo ground truth.
"""

import json
import time

import numpy as np
from px4_msgs.msg import VehicleOdometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
    qos_profile_sensor_data)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from vision_msgs.msg import (
    Detection3D, Detection3DArray, ObjectHypothesisWithPose)

from uav_autonomy.debris_detection import (
    DebrisDetector,
    DetectorParams,
    Intrinsics,
    PoseBuffer,
    depth_to_debug_image,
    is_finite_pose,
)

SENSOR_FRAME = 'depth_up_link'
LOCAL_FRAME = 'px4_local_ned'


class DebrisDepthDetector(Node):
    """Wrap DebrisDetector with ROS interfaces."""

    def __init__(self):
        """Declare parameters and create publishers and subscribers."""
        super().__init__('debris_depth_detector')
        defaults = DetectorParams()
        names = ('min_range_m', 'max_range_m', 'depth_jump_m', 'min_pixels',
                 'min_size_m', 'max_size_m', 'static_radius_m',
                 'static_time_s', 'static_min_fraction', 'history_s')
        values = {}
        for name in names:
            self.declare_parameter(name, getattr(defaults, name))
            values[name] = self.get_parameter(name).value
        self.params = DetectorParams(**values)
        self.declare_parameter('debug_rate_hz', 10.0)
        self.debug_period = 1.0 / max(
            0.1, float(self.get_parameter('debug_rate_hz').value))

        self.detector = None
        self.poses = PoseBuffer()
        self.last_debug = 0.0
        self.frames = 0

        self.pub_sensor = self.create_publisher(
            Detection3DArray, '/perception/debris/detections', 10)
        self.pub_local = self.create_publisher(
            Detection3DArray, '/perception/debris/detections_local', 10)
        self.pub_debug = self.create_publisher(
            Image, '/perception/debris/debug_image', 2)
        self.pub_status = self.create_publisher(
            String, '/perception/debris/status', 10)

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=20)
        self.create_subscription(
            CameraInfo, '/depth_up/camera_info', self.on_info,
            qos_profile_sensor_data)
        self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.on_odometry,
            px4_qos)
        self.create_subscription(
            Image, '/depth_up/image_raw', self.on_depth,
            qos_profile_sensor_data)
        self.get_logger().info(
            'Debris depth detector ready; waiting for camera_info.')

    def on_info(self, msg):
        """Build the detector once intrinsics are known."""
        if self.detector is not None:
            return
        self.detector = DebrisDetector(
            Intrinsics(fx=msg.k[0], fy=msg.k[4], cx=msg.k[2], cy=msg.k[5],
                       width=msg.width, height=msg.height), self.params)
        self.get_logger().info(
            f'Intrinsics: fx={msg.k[0]:.2f} fy={msg.k[4]:.2f} '
            f'cx={msg.k[2]:.1f} cy={msg.k[5]:.1f} {msg.width}x{msg.height}')

    def on_odometry(self, msg):
        """Store the vehicle's pose, keyed by timestamp_sample."""
        if msg.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            return
        if not is_finite_pose(msg.position, msg.q):
            return
        self.poses.add(msg.timestamp_sample * 1e-6,
                       [float(v) for v in msg.position],
                       [float(v) for v in msg.q])

    def on_depth(self, msg):
        """Detect debris in one depth frame and publish the results."""
        if self.detector is None:
            return
        started = time.perf_counter()
        if msg.encoding != '32FC1':
            self.get_logger().error(
                f'Unsupported depth encoding {msg.encoding!r}',
                throttle_duration_sec=5.0)
            return
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(
            msg.height, msg.width)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose = self.poses.at(stamp)
        result = self.detector.process(depth, stamp, pose)

        sensor_array = Detection3DArray()
        sensor_array.header.stamp = msg.header.stamp
        sensor_array.header.frame_id = SENSOR_FRAME
        local_array = Detection3DArray()
        local_array.header.stamp = msg.header.stamp
        local_array.header.frame_id = LOCAL_FRAME
        for index, det in enumerate(result.detections):
            sensor_array.detections.append(self._to_msg(
                sensor_array.header, index, det.position_sensor,
                det.size_sensor, det.confidence))
            if det.position_local is not None:
                size = np.full(3, det.size_m)
                local_array.detections.append(self._to_msg(
                    local_array.header, index, det.position_local, size,
                    det.confidence))
        self.pub_sensor.publish(sensor_array)
        if pose is not None:
            self.pub_local.publish(local_array)

        elapsed_ms = (time.perf_counter() - started) * 1e3
        latest = self.poses.latest_time()
        status = String()
        status.data = json.dumps({
            'stamp': round(stamp, 6),
            'detections': len(result.detections),
            'rejected': result.rejected,
            'valid_pixels': result.valid_pixels,
            'near_pixels': result.near_pixels,
            'pose_available': pose is not None,
            'pose_age_s': (round(stamp - latest, 4)
                           if latest is not None else None),
            'processing_ms': round(elapsed_ms, 2),
            'items': [{
                'p_sensor': [round(float(v), 4) for v in d.position_sensor],
                'p_local': ([round(float(v), 4) for v in d.position_local]
                            if d.position_local is not None else None),
                'size_m': round(d.size_m, 3),
                'size_sensor': [round(float(v), 3) for v in d.size_sensor],
                'pixels': d.pixels, 'bbox_uv': list(d.bbox_uv),
                'range_m': round(d.range_m, 3),
                'touches_border': d.touches_border,
                'confidence': round(d.confidence, 3)}
                for d in result.detections]}, separators=(',', ':'))
        self.pub_status.publish(status)

        now = time.monotonic()
        if now - self.last_debug >= self.debug_period:
            self.last_debug = now
            image = depth_to_debug_image(depth, result,
                                         self.params.max_range_m)
            debug = Image()
            debug.header = msg.header
            debug.height, debug.width = image.shape[:2]
            debug.encoding = 'bgr8'
            debug.step = image.shape[1] * 3
            debug.data = image.tobytes()
            self.pub_debug.publish(debug)
        self.frames += 1

    @staticmethod
    def _to_msg(header, index, position, size, confidence):
        det = Detection3D()
        det.header = header
        det.id = str(index)
        det.bbox.center.position.x = float(position[0])
        det.bbox.center.position.y = float(position[1])
        det.bbox.center.position.z = float(position[2])
        det.bbox.center.orientation.w = 1.0
        det.bbox.size.x = float(size[0])
        det.bbox.size.y = float(size[1])
        det.bbox.size.z = float(size[2])
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = 'debris'
        hypothesis.hypothesis.score = float(confidence)
        hypothesis.pose.pose = det.bbox.center
        det.results.append(hypothesis)
        return det


def main(args=None):
    """Run the debris depth detector node."""
    rclpy.init(args=args)
    node = DebrisDepthDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
