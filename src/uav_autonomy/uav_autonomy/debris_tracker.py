#!/usr/bin/env python3
"""ROS 2 node: persistent 3D debris tracks from the detector's output.

Subscribes
    /perception/debris/detections_local   vision_msgs/Detection3DArray
        (PX4 local NED; the detector already applied the interpolated UAV
        pose, so UAV motion is accounted for before tracking)
    /perception/debris/detections         vision_msgs/Detection3DArray
    /depth_up/camera_info                 sensor_msgs/CameraInfo
        (sensor-frame boxes and intrinsics, used only to tell whether a
        region was cut by the image border, i.e. a partial view)

Publishes
    /perception/debris/tracks         uav_interfaces/DebrisTrackArray
    /perception/debris/track_markers  visualization_msgs/MarkerArray
        (measurement, track ID, velocity arrow, track history)
    /perception/debris/tracks_debug   std_msgs/String, JSON per frame
        (state, covariance diagonal, innovation, dt, processing time)

Time: every step uses the stamp of the detection array, which is the depth
image's own stamp. ROS arrival time is never used.

Tracking only: no prediction horizon, no risk, no commands. This node does
not publish to /fmu/in/* and does not read Gazebo ground truth.
"""

import json
import time

from geometry_msgs.msg import Point
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from uav_interfaces.msg import DebrisTrack, DebrisTrackArray
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray

from uav_autonomy.debris_tracking import (
    COASTING,
    DebrisTracker,
    Measurement,
    TENTATIVE,
    TrackerParams,
)


class DebrisTrackerNode(Node):
    """Wrap DebrisTracker with ROS interfaces."""

    def __init__(self):
        """Declare parameters and create publishers and the subscriber."""
        super().__init__('debris_tracker')
        defaults = TrackerParams()
        values = {}
        for name in TrackerParams.__dataclass_fields__:
            self.declare_parameter(name, getattr(defaults, name))
            values[name] = self.get_parameter(name).value
        self.tracker = DebrisTracker(TrackerParams(**values))
        self.pub_tracks = self.create_publisher(
            DebrisTrackArray, '/perception/debris/tracks', 20)
        self.pub_markers = self.create_publisher(
            MarkerArray, '/perception/debris/track_markers', 5)
        self.pub_debug = self.create_publisher(
            String, '/perception/debris/tracks_debug', 50)
        self.intrinsics = None
        self.sensor_frames = {}     # stamp (ns) -> {detection id: partial}
        self.create_subscription(
            CameraInfo, '/depth_up/camera_info', self.on_info,
            qos_profile_sensor_data)
        self.create_subscription(
            Detection3DArray, '/perception/debris/detections',
            self.on_sensor_detections, 50)
        self.create_subscription(
            Detection3DArray, '/perception/debris/detections_local',
            self.on_detections, 50)
        self.get_logger().info(
            'Debris tracker ready: gravity='
            f'{self.tracker.params.gravity_m_s2} m/s^2')

    def on_info(self, msg):
        """Remember the depth camera intrinsics."""
        self.intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5],
                           msg.width, msg.height)

    def on_sensor_detections(self, msg):
        """Note which sensor-frame regions touch the image border."""
        if self.intrinsics is None:
            return
        key = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.sensor_frames[key] = {
            det.id: region_touches_border(
                det.bbox.center.position, det.bbox.size, self.intrinsics)
            for det in msg.detections}
        for old in sorted(self.sensor_frames)[:-30]:
            del self.sensor_frames[old]

    def on_detections(self, msg):
        """Advance the tracker by one detection frame."""
        started = time.perf_counter()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        key = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        partial = self.sensor_frames.get(key, {})
        measurements = []
        for det in msg.detections:
            p = det.bbox.center.position
            score = (det.results[0].hypothesis.score if det.results
                     else 1.0)
            measurements.append(Measurement(
                np.array([p.x, p.y, p.z]), float(det.bbox.size.x),
                float(score), bool(partial.get(det.id, False))))
        tracks = self.tracker.step(stamp, measurements)
        if tracks is None:      # out-of-order or duplicate stamp
            self.get_logger().warning(
                f'Dropped detection frame with stamp {stamp:.3f} '
                f'({self.tracker.dropped_frames})',
                throttle_duration_sec=2.0)
            return

        array = DebrisTrackArray()
        array.header = msg.header
        for tr in tracks:
            out = DebrisTrack()
            out.id = tr.id
            out.stamp = msg.header.stamp
            out.position.x, out.position.y, out.position.z = (
                float(v) for v in tr.x[:3])
            out.velocity.x, out.velocity.y, out.velocity.z = (
                float(v) for v in tr.x[3:])
            out.covariance = [float(v) for v in tr.P.reshape(-1)]
            out.age = float(tr.age)
            out.time_since_update = float(tr.time_since_update)
            out.hits = int(tr.hits)
            out.consecutive_misses = int(tr.misses)
            out.size = float(tr.size_m)
            out.confidence = float(tr.confidence)
            out.status = int(tr.status)
            array.tracks.append(out)
        self.pub_tracks.publish(array)
        elapsed_ms = (time.perf_counter() - started) * 1e3

        debug = String()
        debug.data = json.dumps({
            'stamp': round(stamp, 6), 'n_meas': len(measurements),
            'processing_ms': round(elapsed_ms, 3),
            'dropped': self.tracker.dropped_frames,
            'tracks': [{
                'id': tr.id, 'status': tr.status, 'hits': tr.hits,
                'p': [round(float(v), 4) for v in tr.x[:3]],
                'v': [round(float(v), 4) for v in tr.x[3:]],
                'P': [round(float(v), 5) for v in np.diag(tr.P)],
                'tsu': round(tr.time_since_update, 4),
                'age': round(tr.age, 4), 'dt': round(tr.last_dt, 4),
                'nis': (round(tr.last_nis, 4)
                        if tr.last_nis is not None else None),
                'z': ([round(float(v), 4) for v in tr.last_measurement]
                      if tr.last_measurement is not None else None),
                'innovation': ([round(float(v), 4)
                                for v in tr.last_innovation]
                               if tr.last_innovation is not None else None),
                'confidence': round(tr.confidence, 3)}
                for tr in tracks]}, separators=(',', ':'))
        self.pub_debug.publish(debug)
        self.pub_markers.publish(self._markers(msg.header, tracks))

    @staticmethod
    def _markers(header, tracks):
        """Visualisation: measurement, ID text, velocity arrow, history."""
        array = MarkerArray()
        clear = Marker()
        clear.header = header
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        for tr in tracks:
            if tr.status == TENTATIVE:
                continue
            coasting = tr.status == COASTING
            base = tr.id * 10

            def make(kind, offset, ns):
                m = Marker()
                m.header = header
                m.ns, m.id, m.type = ns, base + offset, kind
                m.action = Marker.ADD
                m.pose.orientation.w = 1.0
                m.color.a = 1.0
                m.color.r, m.color.g, m.color.b = (
                    (0.6, 0.6, 0.6) if coasting else (1.0, 0.55, 0.0))
                return m

            if tr.last_measurement is not None:
                m = make(Marker.SPHERE, 0, 'measurement')
                (m.pose.position.x, m.pose.position.y,
                 m.pose.position.z) = (float(v) for v in tr.last_measurement)
                m.scale.x = m.scale.y = m.scale.z = max(0.1, tr.size_m)
                array.markers.append(m)
            text = make(Marker.TEXT_VIEW_FACING, 1, 'id')
            text.pose.position.x = float(tr.x[0])
            text.pose.position.y = float(tr.x[1])
            text.pose.position.z = float(tr.x[2]) - 0.4
            text.scale.z = 0.3
            text.text = f'#{tr.id} {np.linalg.norm(tr.x[3:]):.1f} m/s'
            array.markers.append(text)
            arrow = make(Marker.ARROW, 2, 'velocity')
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.05, 0.1, 0.1
            start = Point(x=float(tr.x[0]), y=float(tr.x[1]),
                          z=float(tr.x[2]))
            end = Point(x=float(tr.x[0] + 0.2 * tr.x[3]),
                        y=float(tr.x[1] + 0.2 * tr.x[4]),
                        z=float(tr.x[2] + 0.2 * tr.x[5]))
            arrow.points = [start, end]
            array.markers.append(arrow)
            line = make(Marker.LINE_STRIP, 3, 'history')
            line.scale.x = 0.03
            line.points = [Point(x=float(h[0]), y=float(h[1]), z=float(h[2]))
                           for h in tr.history]
            array.markers.append(line)
        return array


def region_touches_border(centre, size, intrinsics, margin_px=3.0):
    """True if a sensor-frame box reaches the edge of the depth image.

    Sensor frame: +X optical axis, +Y image left, +Z image up. The box is
    projected with its near face; the detector's size already includes one
    pixel footprint, so a region on the border projects onto or past it.
    """
    fx, fy, cx, cy, width, height = intrinsics
    near = max(centre.x - 0.5 * size.x, 1e-3)
    u_min = cx - fx * (centre.y + 0.5 * size.y) / near
    u_max = cx - fx * (centre.y - 0.5 * size.y) / near
    v_min = cy - fy * (centre.z + 0.5 * size.z) / near
    v_max = cy - fy * (centre.z - 0.5 * size.z) / near
    return bool(u_min <= margin_px or v_min <= margin_px
                or u_max >= width - 1 - margin_px
                or v_max >= height - 1 - margin_px)


def main(args=None):
    """Run the debris tracker node."""
    rclpy.init(args=args)
    node = DebrisTrackerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
