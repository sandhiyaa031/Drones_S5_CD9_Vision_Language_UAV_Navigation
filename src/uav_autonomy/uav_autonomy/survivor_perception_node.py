#!/usr/bin/env python3
"""ROS 2 node: survivor candidates from the downward camera, VLM-verified.

Subscribes
    /camera/image_raw       sensor_msgs/Image (rgb8 / bgr8), the real stream
    /camera/camera_info     sensor_msgs/CameraInfo (frame and size check)

Publishes
    /perception/survivor/detections   uav_interfaces/SurvivorDetectionArray
    /perception/survivor/debug_image  sensor_msgs/Image (bgr8, reduced)
    /perception/survivor/status       std_msgs/String, JSON per frame
                                      (logging only; not the interface)

Pipeline (docs/survivor_perception.md)
    camera frame -> candidate detector (deterministic, simulation baseline)
                 -> temporal confirmation (stable tracks only)
                 -> crop of a stable track -> VLM (background thread)
                 -> semantic state per track

The camera callback never waits for the VLM: crops are handed to a worker
thread and answers are picked up on later frames. If no VLM endpoint is
configured or reachable the node keeps publishing candidates and says
VLM_UNAVAILABLE; it never reports a confirmation the VLM did not give.

This node publishes image coordinates only (no 3D position), commands
nothing, and reads no simulator ground truth: its only inputs are the two
camera topics above.
"""

import json
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from uav_interfaces.msg import SurvivorDetection, SurvivorDetectionArray

from uav_autonomy.survivor_confirmation import (
    ConfirmationParams,
    NO_TARGET,
    PENDING,
    STATE_NAMES,
    SurvivorMonitor,
    VERIFY_NAMES,
    VlmAnswer,
    semantic_state,
)
from uav_autonomy.survivor_detection import (
    DetectorParams, SOURCE, detect, to_bgr)
from uav_autonomy.vlm_client import STATUS_NAMES, VlmConfig, VlmWorker

VISIBILITY = {'': 0, 'clear': 1, 'partial': 2, 'poor': 3}
STATE_COLOURS = {1: (0, 200, 255), 2: (0, 220, 0), 3: (0, 140, 255),
                 4: (90, 90, 90)}
STATE_TITLES = {1: 'SURVIVOR CANDIDATE', 2: 'SURVIVOR CONFIRMED',
                3: 'UNCERTAIN', 4: 'REJECTED'}


def _declare_dataclass(node, defaults, prefix):
    """Declare every field as the ROS parameter ``<prefix>.<field>``."""
    values = {}
    for name in defaults.__dataclass_fields__:
        default = getattr(defaults, name)
        key = f'{prefix}.{name}'
        if isinstance(default, tuple):
            node.declare_parameter(key, list(default))
            values[name] = tuple(node.get_parameter(key).value)
        else:
            node.declare_parameter(key, default)
            values[name] = node.get_parameter(key).value
    return type(defaults)(**values)


class SurvivorPerceptionNode(Node):
    """Wrap detector, temporal confirmation and VLM worker."""

    def __init__(self):
        """Declare parameters and create publishers and subscribers."""
        super().__init__('survivor_perception')
        self.detector = _declare_dataclass(self, DetectorParams(),
                                           'detector')
        self.monitor = SurvivorMonitor(
            _declare_dataclass(self, ConfirmationParams(), 'confirmation'))
        # The endpoint and model are parameters (defaults from the
        # environment); the API key is read from the environment only and
        # is never logged or published.
        self.declare_parameter('vlm_endpoint',
                               os.environ.get('VLM_ENDPOINT', ''))
        self.declare_parameter('vlm_model', os.environ.get('VLM_MODEL', ''))
        self.declare_parameter('vlm_timeout_s', 30.0)
        self.declare_parameter('debug_every', 3)      # frames; 0 = off
        self.declare_parameter('debug_scale', 0.5)
        self.vlm_config = VlmConfig(
            endpoint=str(self.get_parameter('vlm_endpoint').value).strip(),
            model=str(self.get_parameter('vlm_model').value).strip(),
            api_key=os.environ.get('VLM_API_KEY', ''),
            timeout_s=float(self.get_parameter('vlm_timeout_s').value))
        self.worker = VlmWorker(self.vlm_config)
        self.debug_every = int(self.get_parameter('debug_every').value)
        self.debug_scale = float(self.get_parameter('debug_scale').value)

        self.frames = 0
        self.bad_frames = 0
        self.camera_frame = ''
        self.info_size = None
        self.vlm_events = []

        self.pub = self.create_publisher(
            SurvivorDetectionArray, '/perception/survivor/detections', 10)
        self.pub_debug = self.create_publisher(
            Image, '/perception/survivor/debug_image', 2)
        self.pub_status = self.create_publisher(
            String, '/perception/survivor/status', 50)
        self.create_subscription(Image, '/camera/image_raw', self.on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/camera_info',
                                 self.on_info, qos_profile_sensor_data)
        self.create_timer(0.1, self.collect_vlm)
        endpoint = self.vlm_config.endpoint or '<none>'
        self.get_logger().info(
            'Survivor perception ready (simulation baseline detector). '
            f'VLM endpoint: {endpoint}, model: '
            f'{self.vlm_config.model or "<none>"}')
        if not self.vlm_config.endpoint or not self.vlm_config.model:
            self.get_logger().warning(
                'No VLM endpoint/model configured: candidates will be '
                'published, verification status will be VLM_UNAVAILABLE, '
                'nothing will be reported as confirmed.')

    # -- inputs ----------------------------------------------------------
    def on_info(self, msg):
        """Remember the camera frame and image size."""
        self.camera_frame = msg.header.frame_id
        self.info_size = (int(msg.width), int(msg.height))

    def collect_vlm(self):
        """Apply finished VLM requests (called from a timer and per frame)."""
        for out in self.worker.poll():
            ctx = out.context or {}
            track_id = ctx.get('track_id')
            event = {'job': out.job_id, 'track': track_id, 'kind': out.kind,
                     'latency_s': round(out.latency_s, 3),
                     'frame_stamp': ctx.get('stamp'),
                     'crop_px': ctx.get('crop_px'),
                     'wall': round(time.time(), 3)}
            if out.kind == 'answer':
                self.monitor.apply_answer(track_id, VlmAnswer(
                    decision=out.decision, confidence=out.confidence,
                    visibility=out.visibility, label=out.target,
                    frame_stamp=float(ctx.get('stamp', 0.0)),
                    latency_s=out.latency_s, model=out.model),
                    float(ctx.get('area', 0.0)))
                event.update(decision=out.decision, target=out.target,
                             confidence=round(out.confidence, 3),
                             visibility=out.visibility,
                             description=out.description)
                self.get_logger().info(
                    f'VLM answer for track {track_id}: {out.decision} '
                    f'"{out.target}" confidence {out.confidence:.2f} '
                    f'visibility {out.visibility} '
                    f'({out.latency_s:.2f} s)')
            else:
                self.monitor.apply_failure(track_id,
                                           out.kind == 'unavailable')
                event.update(error=out.error)
                self.get_logger().warning(
                    f'VLM request for track {track_id} failed '
                    f'({out.kind}): {out.error}',
                    throttle_duration_sec=5.0)
            self.vlm_events.append(event)

    def on_image(self, msg):
        """Detect, follow, request verification, publish. Never blocks."""
        started = time.perf_counter()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= 0.0:
            self.bad_frames += 1
            self.get_logger().warning(
                'camera frame without a timestamp ignored',
                throttle_duration_sec=5.0)
            return
        try:
            bgr = to_bgr(msg.data, msg.height, msg.width, msg.encoding,
                         msg.step)
        except ValueError as exc:
            self.bad_frames += 1
            self.get_logger().error(f'camera frame rejected: {exc}',
                                    throttle_duration_sec=5.0)
            return
        if self.info_size and self.info_size != (msg.width, msg.height):
            self.get_logger().warning(
                f'image size {msg.width}x{msg.height} differs from '
                f'camera_info {self.info_size}', throttle_duration_sec=10.0)
        self.frames += 1
        t_detect = time.perf_counter()
        candidates = detect(bgr, self.detector)
        detect_ms = (time.perf_counter() - t_detect) * 1e3

        self.collect_vlm()
        stable = self.monitor.update(stamp, candidates)
        request = self.monitor.next_request(time.monotonic(),
                                            self.worker.usable)
        if request is not None:
            c = request.candidate
            job = self.worker.submit(
                bgr, (c.x, c.y, c.width, c.height),
                {'track_id': request.track_id, 'stamp': stamp,
                 'area': request.area,
                 'crop_px': [int(c.width), int(c.height)]})
            self.vlm_events.append({
                'job': job, 'track': request.track_id, 'kind': 'request',
                'frame_stamp': stamp, 'wall': round(time.time(), 3)})

        frame_id = msg.header.frame_id or self.camera_frame
        array = SurvivorDetectionArray()
        array.header.stamp = msg.header.stamp
        array.header.frame_id = frame_id
        array.state = self.monitor.frame_state()
        array.vlm_status = self.worker.status
        array.vlm_model = self.vlm_config.model
        rows = []
        for track in stable:
            det, row = self._to_msg(track, msg, frame_id, stamp)
            array.detections.append(det)
            rows.append(row)
        elapsed_ms = (time.perf_counter() - started) * 1e3
        array.processing_ms = float(elapsed_ms)
        self.pub.publish(array)

        status = String()
        status.data = json.dumps({
            'stamp': round(stamp, 6), 'wall': round(time.time(), 4),
            'frame': self.frames, 'encoding': msg.encoding,
            'size': [msg.width, msg.height],
            'processing_ms': round(elapsed_ms, 3),
            'detect_ms': round(detect_ms, 3),
            'raw_candidates': [
                [c.x, c.y, c.width, c.height, c.confidence,
                 int(c.head), int(c.shape), int(c.legs),
                 int(c.touches_border)] for c in candidates],
            'state': STATE_NAMES[array.state],
            'vlm_status': STATUS_NAMES[array.vlm_status],
            'vlm_busy': self.worker.busy,
            'tracks': rows, 'vlm_events': self.vlm_events},
            separators=(',', ':'))
        self.vlm_events = []
        self.pub_status.publish(status)

        if self.debug_every > 0 and self.frames % self.debug_every == 0:
            self.pub_debug.publish(self._debug_image(
                bgr, msg, stable, array, stamp))

    # -- outputs ---------------------------------------------------------
    def _to_msg(self, track, image_msg, frame_id, stamp):
        c = track.candidate
        state = semantic_state(track, self.monitor.params)
        det = SurvivorDetection()
        det.header.stamp = image_msg.header.stamp
        det.header.frame_id = frame_id
        det.track_id = track.track_id
        det.bbox_x, det.bbox_y = int(c.x), int(c.y)
        det.bbox_width, det.bbox_height = int(c.width), int(c.height)
        det.center_u, det.center_v = (float(v) for v in c.center)
        det.image_width = int(image_msg.width)
        det.image_height = int(image_msg.height)
        det.candidate_confidence = float(c.confidence)
        det.head_cue, det.legs_cue = bool(c.head), bool(c.legs)
        det.touches_border = bool(c.touches_border)
        det.hits = int(track.hits)
        det.age = float(stamp - track.first_stamp)
        det.measured = bool(track.measured)
        det.semantic_state = state
        det.verification_status = track.verification
        det.vlm_confidence = -1.0
        det.vlm_requests = int(track.requests)
        det.source = SOURCE
        row = {'id': track.track_id,
               'bbox': [int(c.x), int(c.y), int(c.width), int(c.height)],
               'conf': round(float(c.confidence), 3), 'head': int(c.head),
               'legs': int(c.legs), 'border': int(c.touches_border),
               'hits': track.hits, 'age': round(det.age, 3),
               'measured': int(track.measured),
               'state': STATE_NAMES[state],
               'verify': VERIFY_NAMES[track.verification],
               'requests': track.requests}
        if track.answers:
            last = track.answers[-1]
            det.vlm_confidence = float(last.confidence)
            det.vlm_visibility = VISIBILITY.get(last.visibility, 0)
            det.vlm_label = last.label
            sec = int(last.frame_stamp)
            det.vlm_stamp.sec = sec
            det.vlm_stamp.nanosec = int(round((last.frame_stamp - sec)
                                              * 1e9))
            det.vlm_latency = float(last.latency_s)
            det.source = f'{SOURCE}+vlm:{last.model}'
            row.update(vlm_decision=last.decision,
                       vlm_conf=round(last.confidence, 3),
                       vlm_vis=last.visibility, vlm_label=last.label,
                       vlm_latency=round(last.latency_s, 3),
                       vlm_stamp=round(last.frame_stamp, 6))
        return det, row

    def _debug_image(self, bgr, image_msg, stable, array, stamp):
        """Reduced camera image with this node's own outputs drawn on it."""
        s = self.debug_scale
        view = cv2.resize(bgr, None, fx=s, fy=s,
                          interpolation=cv2.INTER_AREA)
        font = cv2.FONT_HERSHEY_SIMPLEX
        for track in stable:
            c = track.candidate
            state = semantic_state(track, self.monitor.params)
            colour = STATE_COLOURS[state]
            p0 = (int(c.x * s), int(c.y * s))
            p1 = (int((c.x + c.width) * s), int((c.y + c.height) * s))
            cv2.rectangle(view, p0, p1, colour, 2 if track.measured else 1)
            verify = VERIFY_NAMES[track.verification].replace('VLM_', '')
            lines = [f'#{track.track_id} {STATE_TITLES[state]}',
                     f'cue {c.confidence:.2f}  VLM: {verify}']
            if track.answers:
                last = track.answers[-1]
                lines.append(f'VLM conf {last.confidence:.2f} '
                             f'({last.visibility}) "{last.label[:22]}"')
            elif track.verification == PENDING:
                lines.append('VLM: waiting for answer')
            y = p1[1] + 16 if p1[1] + 52 < view.shape[0] else max(
                14, p0[1] - 40)
            for i, text in enumerate(lines):
                org = (max(2, p0[0]), y + 16 * i)
                cv2.putText(view, text, org, font, 0.45, (0, 0, 0), 3)
                cv2.putText(view, text, org, font, 0.45, colour, 1)
        header = (f't={stamp:.3f} s  {STATE_NAMES[array.state]}  '
                  f'{STATUS_NAMES[array.vlm_status]}')
        cv2.putText(view, header, (6, 18), font, 0.5, (0, 0, 0), 3)
        cv2.putText(view, header, (6, 18), font, 0.5, (255, 255, 255), 1)
        out = Image()
        out.header = image_msg.header
        out.height, out.width = view.shape[:2]
        out.encoding = 'bgr8'
        out.step = view.shape[1] * 3
        out.data = np.ascontiguousarray(view).tobytes()
        return out

    def destroy_node(self):
        """Stop the VLM worker thread."""
        self.worker.stop()
        return super().destroy_node()


def main(args=None):
    """Run the survivor perception node."""
    rclpy.init(args=args)
    node = SurvivorPerceptionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
