#!/usr/bin/env python3
"""Record the survivor localisation output of one flight (Phase 8).

Writes into --out-dir:
    locations.jsonl             one line per SurvivorLocationArray
    localization_status.jsonl   the node's JSON status per frame

Recording only: nothing here is fed back to any node.
"""

import argparse
import json
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from uav_interfaces.msg import SurvivorLocationArray


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--out-dir', required=True)
    parser.add_argument('--duration', type=float, default=80.0)
    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    rclpy.init()
    node = Node('log_locations')
    out = open(os.path.join(args.out_dir, 'locations.jsonl'), 'w')
    status_out = open(os.path.join(
        args.out_dir, 'localization_status.jsonl'), 'w')
    count = {'msgs': 0}

    def on_array(msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        out.write(json.dumps({
            'stamp': round(stamp, 6), 'frame_id': msg.header.frame_id,
            'ground_down': round(float(msg.ground_down), 4),
            'pose': bool(msg.pose_available),
            'locations': [{
                'id': int(m.survivor_id), 'track': int(m.track_id),
                'ned': [round(m.position.x, 4), round(m.position.y, 4),
                        round(m.position.z, 4)],
                'cov': [round(float(v), 5) for v in m.covariance],
                'sigma_h': round(float(m.horizontal_sigma), 4),
                'height': round(float(m.height_above_ground), 3),
                'state': int(m.localization_state),
                'obs': int(m.observations),
                'baseline': round(float(m.baseline), 3),
                'min_range': round(float(m.min_range), 3),
                'updated': bool(m.updated),
                'semantic_state': int(m.semantic_state),
                'confirmed': bool(m.vlm_confirmed),
                'rejected': bool(m.vlm_rejected)} for m in msg.locations]},
            separators=(',', ':')) + '\n')
        count['msgs'] += 1

    node.create_subscription(
        SurvivorLocationArray, '/perception/survivor/locations', on_array, 50)
    node.create_subscription(
        String, '/perception/survivor/localization_status',
        lambda m: status_out.write(m.data + '\n'), 50)
    end = time.monotonic() + args.duration
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    out.close()
    status_out.close()
    print(f'log_locations: {count["msgs"]} messages')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
