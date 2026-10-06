#!/usr/bin/env python3
"""Test tool: relay the depth stream and drop frames on purpose.

Relays /depth_up/image_raw to /depth_up/image_gated unchanged (serialized
bytes). After every release recorded in <run_dir>/releases.json it drops the
frames whose sensor time lies in
    [release time + delay, release time + delay + gap]
so that the perception chain sees a depth gap of a known length. The
detector is pointed at the gated topic by a ROS remap; nothing else changes.
Dropped stamps are written to <run_dir>/depth_gaps.json.
"""

import argparse
import json
import os
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--run-dir', required=True)
    parser.add_argument('--gap', type=float, required=True, help='seconds')
    parser.add_argument('--delay', type=float, default=0.0,
                        help='seconds after the release')
    parser.add_argument('--duration', type=float, default=120.0)
    args = parser.parse_args()
    rclpy.init()
    node = Node('depth_gate')
    pub = node.create_publisher(Image, '/depth_up/image_gated',
                                qos_profile_sensor_data)
    releases_path = os.path.join(args.run_dir, 'releases.json')
    state = {'windows': [], 'seen': 0, 'checked': 0.0, 'dropped': [],
             'relayed': 0}

    def poll_releases():
        now = time.monotonic()
        if now - state['checked'] < 0.01:
            return
        state['checked'] = now
        try:
            releases = json.load(open(releases_path))
        except (OSError, ValueError):
            return
        for rel in releases[state['seen']:]:
            t0 = rel.get('sim_time_at_request')
            if t0 is not None:
                state['windows'].append(
                    (t0 + args.delay, t0 + args.delay + args.gap))
        state['seen'] = len(releases)

    def on_image(raw):
        poll_releases()
        # CDR: 4-byte encapsulation header, then header.stamp (int32 sec,
        # uint32 nanosec).
        sec, nanosec = struct.unpack_from('<iI', raw, 4)
        stamp = sec + nanosec * 1e-9
        if any(a <= stamp <= b for a, b in state['windows']):
            state['dropped'].append(round(stamp, 4))
            return
        pub.publish(raw)
        state['relayed'] += 1

    node.create_subscription(Image, '/depth_up/image_raw', on_image,
                             qos_profile_sensor_data, raw=True)
    end = time.monotonic() + args.duration
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)
            poll_releases()
    except Exception:       # context shut down at teardown
        pass
    json.dump({'gap_s': args.gap, 'delay_s': args.delay,
               'windows': state['windows'], 'dropped': state['dropped'],
               'relayed': state['relayed']},
              open(os.path.join(args.run_dir, 'depth_gaps.json'), 'w'),
              indent=1)


if __name__ == '__main__':
    main()
