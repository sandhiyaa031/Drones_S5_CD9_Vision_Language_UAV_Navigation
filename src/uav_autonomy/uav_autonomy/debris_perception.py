#!/usr/bin/env python3
# flake8: noqa: I100,I201
"""Detect and track moving debris in the onboard camera image plane.

This node deliberately consumes camera pixels only. It does not use Gazebo
poses, PX4 state, or any flight-control interface.
"""

import json
import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class DebrisPerception(Node):
    """Moving-region detector, image-plane tracker, and predictor."""

    def __init__(self):
        """Initialize ROS interfaces and image-plane track state."""
        super().__init__('debris_perception')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('min_area_px', 45)
        self.declare_parameter('max_area_fraction', 0.18)
        self.declare_parameter('motion_threshold', 22)
        self.declare_parameter('processing_scale', 0.5)
        self.declare_parameter('max_track_age_s', 0.8)
        self.declare_parameter('association_gate_px', 100.0)
        self.declare_parameter('debug_rate_hz', 8.0)

        self.bridge = CvBridge()
        self.previous_gray = None
        self.previous_stamp_ns = None
        self.tracks = {}
        self.next_id = 1
        self.last_debug_publish = 0.0
        self.processing_scale = float(
            self.get_parameter('processing_scale').value)
        if not 0.1 <= self.processing_scale <= 1.0:
            raise ValueError('processing_scale must be in [0.1, 1.0]')

        self.track_pub = self.create_publisher(
            String, '/perception/debris/tracks', 10)
        self.debug_pub = self.create_publisher(
            Image, '/perception/debris/debug_image', 2)
        self.image_sub = self.create_subscription(
            Image,
            str(self.get_parameter('image_topic').value),
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            'Debris perception ready: pixel-only motion detection and tracking; '
            f"input={self.get_parameter('image_topic').value}"
        )

    @staticmethod
    def _stamp_ns(msg):
        value = msg.header.stamp
        result = int(value.sec) * 1_000_000_000 + int(value.nanosec)
        return result if result > 0 else time.time_ns()

    def _to_bgr(self, msg):
        if msg.encoding in ('rgb8', 'bgr8', 'mono8'):
            return self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
        raise ValueError(f'unsupported image encoding {msg.encoding!r}')

    def _motion_mask(self, gray):
        if (self.previous_gray is None or
                self.previous_gray.shape != gray.shape):
            self.previous_gray = gray.copy()
            return np.zeros_like(gray), np.eye(2, 3, dtype=np.float32)

        previous = self.previous_gray
        transform = None
        # Estimate camera/background motion from sparse corners. Moving debris
        # is an outlier and is rejected by RANSAC.
        old_pts = cv2.goodFeaturesToTrack(
            previous, maxCorners=350, qualityLevel=0.01, minDistance=12)
        if old_pts is not None and len(old_pts) >= 8:
            new_pts, status, _ = cv2.calcOpticalFlowPyrLK(
                previous, gray, old_pts, None,
                winSize=(21, 21), maxLevel=3,
                criteria=(cv2.TERM_CRITERIA_EPS |
                          cv2.TERM_CRITERIA_COUNT, 25, 0.03))
            if new_pts is not None and status is not None:
                good_old = old_pts[status.ravel() == 1]
                good_new = new_pts[status.ravel() == 1]
                if len(good_old) >= 8:
                    transform, inliers = cv2.estimateAffinePartial2D(
                        good_old, good_new, method=cv2.RANSAC,
                        ransacReprojThreshold=3.0, maxIters=500,
                        confidence=0.98)
                    if transform is not None:
                        inlier_ratio = (
                            float(np.mean(inliers))
                            if inliers is not None else 0.0)
                        if inlier_ratio < 0.30:
                            transform = None
        if transform is None:
            # If the background transform cannot be estimated, do not turn
            # camera motion into false moving-object detections.
            self.previous_gray = gray.copy()
            return np.zeros_like(gray), np.eye(2, 3, dtype=np.float32)
        aligned = cv2.warpAffine(
            previous, transform, (gray.shape[1], gray.shape[0]),
            flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        delta = cv2.absdiff(gray, aligned)
        threshold = int(self.get_parameter('motion_threshold').value)
        _, mask = cv2.threshold(delta, threshold, 255, cv2.THRESH_BINARY)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=1)
        self.previous_gray = gray.copy()
        return mask, transform

    def _detect(self, bgr, mask):
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        min_area = int(self.get_parameter('min_area_px').value)
        max_area = int(mask.shape[0] * mask.shape[1] *
                       float(self.get_parameter('max_area_fraction').value))
        detections = []
        for label in range(1, count):
            x, y, width, height, area = (int(v) for v in stats[label])
            if area < min_area or area > max_area or width < 5 or height < 5:
                continue
            region = labels[y:y + height, x:x + width] == label
            fill = float(area) / max(1, width * height)
            if fill < 0.035:
                continue
            crop = bgr[y:y + height, x:x + width]
            # Debris is commonly grey/tan/brown in this scenario. Color only
            # adjusts confidence; motion and geometry remain the detector.
            hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
            saturation = hsv[:, :, 1][region]
            neutral_fraction = (
                float(np.mean(saturation < 115))
                if saturation.size else 0.0)
            earth_fraction = (
                float(np.mean((hsv[:, :, 0][region] < 30) &
                              (saturation < 190)))
                if saturation.size else 0.0)
            contrast_score = min(1.0, fill * 2.0)
            confidence = float(np.clip(
                0.48 + 0.22 * neutral_fraction +
                0.16 * earth_fraction + 0.14 * contrast_score,
                0.0, 0.99))
            scale = 1.0 / self.processing_scale
            full_x, full_y = x * scale, y * scale
            full_width, full_height = width * scale, height * scale
            detections.append({
                'bbox': [int(full_x), int(full_y),
                         int(full_width), int(full_height)],
                'center': [full_x + full_width / 2.0,
                           full_y + full_height / 2.0],
                'area_px': int(area * scale * scale),
                'confidence': confidence,
            })
        return detections

    @staticmethod
    def _iou(box_a, box_b):
        ax, ay, aw, ah = box_a
        bx, by, bw, bh = box_b
        left, top = max(ax, bx), max(ay, by)
        right, bottom = min(ax + aw, bx + bw), min(ay + ah, by + bh)
        intersection = max(0, right - left) * max(0, bottom - top)
        union = aw * ah + bw * bh - intersection
        return intersection / union if union > 0 else 0.0

    def _associate(self, detections, stamp_ns):
        dt_by_id = {}
        for track_id, track in self.tracks.items():
            dt = max(1e-3, (stamp_ns - track['stamp_ns']) / 1e9)
            dt_by_id[track_id] = dt
            track['predicted_center'] = [
                track['center'][0] + track['velocity'][0] * dt,
                track['center'][1] + track['velocity'][1] * dt,
            ]

        pairs = []
        gate_base = float(self.get_parameter('association_gate_px').value)
        for track_id, track in self.tracks.items():
            dt = dt_by_id[track_id]
            gate = min(
                gate_base * 2.0,
                gate_base + math.hypot(*track['velocity']) * dt * 0.5)
            for index, detection in enumerate(detections):
                dx = detection['center'][0] - track['predicted_center'][0]
                dy = detection['center'][1] - track['predicted_center'][1]
                distance = math.hypot(dx, dy)
                overlap = self._iou(track['bbox'], detection['bbox'])
                if distance <= gate or overlap >= 0.02:
                    pairs.append((distance - min(overlap, 1.0) * 25.0,
                                  track_id, index))
        pairs.sort()
        used_tracks, used_detections = set(), set()
        for _, track_id, index in pairs:
            if track_id in used_tracks or index in used_detections:
                continue
            track = self.tracks[track_id]
            detection = detections[index]
            dt = dt_by_id[track_id]
            # Alpha-beta update smooths center and pixel velocity while
            # retaining the current frame timestamp.
            alpha, beta = 0.72, 0.18
            residual = [
                detection['center'][axis] - track['predicted_center'][axis]
                for axis in (0, 1)]
            track['center'] = [
                track['predicted_center'][axis] + alpha * residual[axis]
                for axis in (0, 1)]
            track['velocity'] = [track['velocity'][axis] +
                                 beta * residual[axis] / dt for axis in (0, 1)]
            track['bbox'] = detection['bbox']
            track['stamp_ns'] = stamp_ns
            track['confidence'] = (
                0.65 * track['confidence'] +
                0.35 * detection['confidence'])
            track['hits'] += 1
            track['misses'] = 0
            track['confirmed'] = track['hits'] >= 2
            used_tracks.add(track_id)
            used_detections.add(index)
        for index, detection in enumerate(detections):
            if index in used_detections:
                continue
            track_id = self.next_id
            self.next_id += 1
            self.tracks[track_id] = {
                'center': detection['center'], 'velocity': [0.0, 0.0],
                'bbox': detection['bbox'], 'stamp_ns': stamp_ns,
                'confidence': detection['confidence'], 'hits': 1,
                'misses': 0, 'confirmed': False,
            }
        for track in self.tracks.values():
            if track['stamp_ns'] < stamp_ns:
                track['misses'] += 1
        max_age = float(self.get_parameter('max_track_age_s').value)
        for track_id in list(self.tracks):
            if stamp_ns - self.tracks[track_id]['stamp_ns'] > max_age * 1e9:
                del self.tracks[track_id]

    def _tracks_document(self, stamp_ns, width, height):
        states = []
        for track_id, track in sorted(self.tracks.items()):
            if not track['confirmed']:
                continue
            px, py = track['center']
            vx, vy = track['velocity']
            predictions = {}
            for horizon in (0.5, 1.0, 2.0):
                predictions[f'{horizon:.1f}s'] = {
                    'x_px': round(px + vx * horizon, 2),
                    'y_px': round(py + vy * horizon, 2),
                }
            states.append({
                'object_id': track_id,
                'timestamp_ns': stamp_ns,
                'pixel_position': {'x': round(px, 2), 'y': round(py, 2)},
                'bbox_xywh_px': track['bbox'],
                'image_size_px': {'width': width, 'height': height},
                'estimated_position_3d_m': None,
                'velocity_px_s': {'x': round(vx, 2), 'y': round(vy, 2)},
                'predicted_pixel_positions': predictions,
                'confidence': round(track['confidence'], 3),
                'hits': track['hits'],
            })
        return {
            'header': {
                'timestamp_ns': stamp_ns,
                'frame_id': 'camera_link',
                'coordinate_space': 'image_pixels'},
            'tracks': states}

    def image_callback(self, msg):
        """Detect, associate, publish tracks, and produce the debug view."""
        try:
            bgr = self._to_bgr(msg)
            scale = self.processing_scale
            small_width = max(1, int(msg.width * scale))
            small_height = max(1, int(msg.height * scale))
            small_bgr = cv2.resize(
                bgr, (small_width, small_height),
                interpolation=cv2.INTER_AREA)
            gray = cv2.cvtColor(small_bgr, cv2.COLOR_BGR2GRAY)
            stamp_ns = self._stamp_ns(msg)
            mask, _ = self._motion_mask(gray)
            detections = self._detect(small_bgr, mask)
            self._associate(detections, stamp_ns)
            document = self._tracks_document(stamp_ns, msg.width, msg.height)
            output = String()
            output.data = json.dumps(document, separators=(',', ':'))
            self.track_pub.publish(output)

            now = time.monotonic()
            rate = float(self.get_parameter('debug_rate_hz').value)
            if rate > 0 and now - self.last_debug_publish >= 1.0 / rate:
                annotated = bgr.copy()
                for track_id, track in self.tracks.items():
                    if not track['confirmed']:
                        continue
                    x, y, w, h = track['bbox']
                    cv2.rectangle(
                        annotated, (x, y), (x + w, y + h),
                        (0, 220, 255), 2)
                    label = f"debris {track_id} {track['confidence']:.2f}"
                    cv2.putText(
                        annotated, label, (x, max(18, y - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 255), 2)
                debug = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                debug.header = msg.header
                self.debug_pub.publish(debug)
                self.last_debug_publish = now
        except Exception as exc:  # Keep malformed frames from killing the node.
            self.get_logger().error(
                f'Image processing failed: {exc}', throttle_duration_sec=2.0)


def main(args=None):
    """Run the debris perception node."""
    rclpy.init(args=args)
    node = DebrisPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        finally:
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
