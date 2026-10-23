#!/usr/bin/env python3
"""Log wall-clock gaps in /clock and in PX4's local position (telemetry).

Writes one JSON line per gap longer than --gap seconds to OUT (and a summary
at the end). Used to see whether a "telemetry stale" failsafe comes from the
simulator stalling or from the PX4 link. Read-only: publishes nothing.

Usage: gap_monitor.py --out FILE [--duration 200] [--gap 0.3]
"""

import argparse
import json
import time

import rclpy
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rosgraph_msgs.msg import Clock


class Monitor(Node):
    def __init__(self, out, gap):
        super().__init__('gap_monitor')
        self.out, self.gap = open(out, 'w'), gap
        self.last = {'clock': None, 'px4': None, 'status': None}
        self.sim = 0.0
        self.worst = {'clock': 0.0, 'px4': 0.0, 'status': 0.0}
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Clock, '/clock', self.on_clock, qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v4',
                                 self.on_status, qos)
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position_v1',
                                 self.on_px4, qos)

    def tick(self, key):
        now = time.monotonic()
        if self.last[key] is not None:
            dt = now - self.last[key]
            self.worst[key] = max(self.worst[key], dt)
            if dt > self.gap:
                self.out.write(json.dumps({
                    'source': key, 'gap_s': round(dt, 3),
                    'sim_s': round(self.sim, 2),
                    'wall': round(time.time(), 2)}) + '\n')
                self.out.flush()
        self.last[key] = now

    def on_clock(self, msg):
        self.sim = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.tick('clock')

    def on_status(self, _msg):
        self.tick('status')

    def on_px4(self, _msg):
        self.tick('px4')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--out', required=True)
    parser.add_argument('--duration', type=float, default=200.0)
    parser.add_argument('--gap', type=float, default=0.3)
    args = parser.parse_args()
    rclpy.init()
    node = Monitor(args.out, args.gap)
    end = time.monotonic() + args.duration
    while time.monotonic() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
    node.out.write(json.dumps({'summary_worst_gap_s': node.worst}) + '\n')
    node.out.close()


if __name__ == '__main__':
    main()
