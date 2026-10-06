#!/usr/bin/env python3
"""Measure the offset between PX4 timestamps and Gazebo simulation time.

For every PX4 VehicleLocalPosition message received over ROS, record
  offset = msg.timestamp_sample [PX4 time]  -  Gazebo /clock sim time at arrival
The arrival-based offset includes transport latency, so its *minimum* over a
window is the best estimate of the true clock offset and its trend over the
run is the drift. Also records sensor image stamps against /clock to show that
image headers are in Gazebo simulation time. Read-only.
"""

import argparse
import json
import os
import sys
import threading
import time

from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.image_pb2 import Image as GzImage
from gz.transport13 import Node as GzNode
import numpy as np
from px4_msgs.msg import VehicleLocalPosition, VehicleOdometry
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--world', default=os.environ.get(
        'WORLD', 'collapsed_building_rescue'))
    parser.add_argument('--duration', type=float, default=300.0)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()

    sim = {'t': None}
    lock = threading.Lock()
    image_lag = []

    def on_clock(msg):
        sim['t'] = msg.sim.sec + msg.sim.nsec * 1e-9

    def on_depth(msg):
        if sim['t'] is not None:
            stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
            with lock:
                image_lag.append(sim['t'] - stamp)

    gz_node = GzNode()
    gz_node.subscribe(Clock, f'/world/{args.world}/clock', on_clock)
    gz_node.subscribe(GzImage, '/uav/depth_up/image', on_depth)

    rclpy.init()
    node = Node('time_sync')
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=10)
    samples = {'position': [], 'odometry': []}

    def make(name, field):
        def callback(msg):
            if sim['t'] is not None:
                samples[name].append(
                    (sim['t'], getattr(msg, field) * 1e-6 - sim['t'],
                     msg.timestamp * 1e-6 - sim['t'],
                     time.time() - getattr(msg, field) * 1e-6))
        return callback

    node.create_subscription(
        VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
        make('position', 'timestamp_sample'), qos)
    node.create_subscription(
        VehicleOdometry, '/fmu/out/vehicle_odometry',
        make('odometry', 'timestamp_sample'), qos)

    t0 = time.monotonic()
    while time.monotonic() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.05)

    report = {'duration_wall_s': round(time.monotonic() - t0, 1)}
    for name, rows in samples.items():
        if len(rows) < 50:
            report[name] = {'samples': len(rows)}
            continue
        a = np.array(rows)
        t, off_sample, off_pub = a[:, 0], a[:, 1], a[:, 2]
        # windowed maximum of (px4 - sim_at_arrival): arrival is always late,
        # so the largest value in a window is the least-delayed sample.
        edges = np.linspace(t[0], t[-1], 11)
        window_best = [float(off_sample[(t >= lo) & (t < hi)].max())
                       for lo, hi in zip(edges[:-1], edges[1:])
                       if ((t >= lo) & (t < hi)).any()]
        centres = 0.5 * (edges[:-1] + edges[1:])[:len(window_best)]
        slope = float(np.polyfit(centres, window_best, 1)[0])
        report[name] = {
            'samples': len(rows),
            'rate_hz_sim_time': round((len(rows) - 1) / (t[-1] - t[0]), 2),
            'sim_time_span_s': round(float(t[-1] - t[0]), 1),
            'offset_px4_minus_gazebo_best_s': round(max(window_best), 6),
            'offset_per_window_s': [round(v, 6) for v in window_best],
            'drift_s_per_s': slope,
            'drift_ms_over_run': round(slope * (t[-1] - t[0]) * 1e3, 3),
            'arrival_latency_median_ms': round(float(np.median(
                max(window_best) - off_sample)) * 1e3, 2),
            'arrival_latency_p99_ms': round(float(np.percentile(
                max(window_best) - off_sample, 99)) * 1e3, 2),
            'publish_minus_sample_median_ms': round(float(np.median(
                off_pub - off_sample)) * 1e3, 3),
            # PX4 stamps compared with this machine's wall clock on arrival
            'wall_arrival_minus_px4_stamp_ms': {
                'min': round(float(a[:, 3].min()) * 1e3, 2),
                'median': round(float(np.median(a[:, 3])) * 1e3, 2),
                'p99': round(float(np.percentile(a[:, 3], 99)) * 1e3, 2),
                'max': round(float(a[:, 3].max()) * 1e3, 2)}}
    with lock:
        lag = np.array(image_lag)
    if len(lag):
        report['depth_image_stamp_vs_clock'] = {
            'frames': int(len(lag)),
            'clock_minus_stamp_median_ms': round(float(np.median(lag)) * 1e3,
                                                 2),
            'clock_minus_stamp_min_ms': round(float(lag.min()) * 1e3, 2),
            'clock_minus_stamp_max_ms': round(float(lag.max()) * 1e3, 2)}
    text = json.dumps(report, indent=1)
    with open(args.output, 'w') as handle:
        handle.write(text + '\n')
    print(text, flush=True)
    sys.stdout.flush()
    os._exit(0)


if __name__ == '__main__':
    main()
