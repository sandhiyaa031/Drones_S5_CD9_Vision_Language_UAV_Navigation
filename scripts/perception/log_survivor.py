#!/usr/bin/env python3
"""Log the survivor perception node's output (evaluation tooling).

Records
    survivor.jsonl        the node's per-frame status (JSON)
    survivor_msgs.jsonl   one line per SurvivorDetectionArray received on the
                          proper interface: its stamp, state, detections, and
                          the simulation time at which it arrived (from PX4
                          odometry, same time base as the camera stamps), so
                          that the end-to-end perception latency can be
                          measured
    debug_frames/         every Nth debug image, as published by the node
    survivor_cpu.json     CPU use of the node process, sampled once a second
"""

import argparse
import json
import os
import signal
import subprocess
import time

import cv2
import numpy as np
from px4_msgs.msg import VehicleOdometry
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)
from sensor_msgs.msg import Image
from std_msgs.msg import String
from uav_interfaces.msg import SurvivorDetectionArray


def node_cpu_ticks():
    """(utime + stime) of the survivor_perception process, or None."""
    out = subprocess.run(['pgrep', '-f', 'survivor_perception'],
                         capture_output=True, text=True).stdout.split()
    total, found = 0, False
    for pid in out:
        try:
            fields = open(f'/proc/{pid}/stat').read().rsplit(')', 1)[1].split()
            if 'log_survivor' in open(f'/proc/{pid}/cmdline').read():
                continue
            total += int(fields[11]) + int(fields[12])
            found = True
        except (OSError, IndexError, ValueError):
            continue
    return total if found else None


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--out-dir', required=True)
    parser.add_argument('--duration', type=float, default=80.0)
    parser.add_argument('--save-debug-every', type=int, default=5)
    args = parser.parse_args()
    os.makedirs(os.path.join(args.out_dir, 'debug_frames'), exist_ok=True)
    rclpy.init()
    node = Node('log_survivor')
    status_out = open(os.path.join(args.out_dir, 'survivor.jsonl'), 'w')
    msgs_out = open(os.path.join(args.out_dir, 'survivor_msgs.jsonl'), 'w')
    state = {'odom': None, 'debug': 0, 'status': 0, 'msgs': 0}

    def on_status(msg):
        status_out.write(msg.data + '\n')
        state['status'] += 1

    def on_odometry(msg):
        state['odom'] = (msg.timestamp_sample * 1e-6, time.monotonic())

    def sim_now():
        if state['odom'] is None:
            return None
        t, wall = state['odom']
        return t + (time.monotonic() - wall)

    def on_array(msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = sim_now()
        msgs_out.write(json.dumps({
            'stamp': round(stamp, 6),
            'sim_time_received': None if now is None else round(now, 4),
            'frame_id': msg.header.frame_id, 'state': int(msg.state),
            'vlm_status': int(msg.vlm_status), 'vlm_model': msg.vlm_model,
            'processing_ms': round(float(msg.processing_ms), 3),
            'detections': [{
                'id': int(d.track_id), 'frame_id': d.header.frame_id,
                'bbox': [d.bbox_x, d.bbox_y, d.bbox_width, d.bbox_height],
                'center': [round(d.center_u, 1), round(d.center_v, 1)],
                'image': [d.image_width, d.image_height],
                'candidate_confidence': round(d.candidate_confidence, 3),
                'semantic_state': int(d.semantic_state),
                'verification_status': int(d.verification_status),
                'vlm_confidence': round(d.vlm_confidence, 3),
                'vlm_label': d.vlm_label, 'source': d.source,
                'measured': bool(d.measured)} for d in msg.detections]},
            separators=(',', ':')) + '\n')
        state['msgs'] += 1

    def on_debug(msg):
        state['debug'] += 1
        if (not args.save_debug_every
                or state['debug'] % args.save_debug_every):
            return
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        cv2.imwrite(os.path.join(args.out_dir, 'debug_frames',
                                 f'{stamp:010.3f}.jpg'), image,
                    [cv2.IMWRITE_JPEG_QUALITY, 88])

    px4_qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST, depth=5)
    node.create_subscription(String, '/perception/survivor/status',
                             on_status, 100)
    node.create_subscription(SurvivorDetectionArray,
                             '/perception/survivor/detections', on_array, 100)
    node.create_subscription(Image, '/perception/survivor/debug_image',
                             on_debug, 5)
    node.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry',
                             on_odometry, px4_qos)
    stop = {'now': False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(now=True))
    signal.signal(signal.SIGTERM, lambda *_: stop.update(now=True))
    cpu, ticks_per_s = [], os.sysconf('SC_CLK_TCK')
    last_ticks, last_wall = node_cpu_ticks(), time.monotonic()
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.duration and not stop['now']:
        try:
            rclpy.spin_once(node, timeout_sec=0.05)
        except Exception:        # context shut down underneath us
            break
        if time.monotonic() - last_wall >= 1.0:
            ticks, wall = node_cpu_ticks(), time.monotonic()
            if ticks is not None and last_ticks is not None:
                cpu.append(round(100.0 * (ticks - last_ticks) / ticks_per_s
                                 / (wall - last_wall), 1))
            last_ticks, last_wall = ticks, wall
    status_out.close()
    msgs_out.close()
    json.dump({'cpu_percent_of_one_core': cpu},
              open(os.path.join(args.out_dir, 'survivor_cpu.json'), 'w'))
    print(f"status frames={state['status']} detection arrays="
          f"{state['msgs']} debug images={state['debug']}", flush=True)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
