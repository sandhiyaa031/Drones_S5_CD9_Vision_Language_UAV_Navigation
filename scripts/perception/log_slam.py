#!/usr/bin/env python3
"""Record SLAM output of one flight (village missions).

Writes into --out-dir:
    slam_poses.jsonl   10 Hz: stamp, odometry pose handed to SLAM (x, y,
                       yaw in the odom frame) and the SLAM pose (the same
                       composed with slam_toolbox's map->odom correction)
    slam_map.npz       the last occupancy grid (data, resolution, origin)
    slam_map_info.json when the map was received, its size and resolution
    slam_bridge.jsonl  the bridge's status

Recording only: nothing here is fed back to any node.
"""

import argparse
import json
import math
import os
import time

import numpy as np
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--out-dir', required=True)
    parser.add_argument('--duration', type=float, default=300.0)
    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    rclpy.init()
    node = Node('log_slam')
    out = open(os.path.join(args.out_dir, 'slam_poses.jsonl'), 'w')
    bridge = open(os.path.join(args.out_dir, 'slam_bridge.jsonl'), 'w')
    state = {'map_odom': None, 'last': 0.0, 'map': None, 'maps': 0,
             'corrections': 0}

    def on_tf(msg):
        for tf in msg.transforms:
            t = tf.transform
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            if tf.header.frame_id == 'map' and tf.child_frame_id == 'odom':
                new = (t.translation.x, t.translation.y, yaw_of(t.rotation))
                if state['map_odom'] is not None and (
                        abs(new[0] - state['map_odom'][0]) > 1e-4
                        or abs(new[1] - state['map_odom'][1]) > 1e-4):
                    state['corrections'] += 1
                state['map_odom'] = new
            elif tf.header.frame_id == 'odom' \
                    and tf.child_frame_id == 'base_level':
                if stamp - state['last'] < 0.1:
                    continue
                state['last'] = stamp
                ox, oy, oyaw = (t.translation.x, t.translation.y,
                                yaw_of(t.rotation))
                row = {'stamp': round(stamp, 4),
                       'odom': [round(ox, 4), round(oy, 4), round(oyaw, 5)]}
                if state['map_odom'] is not None:
                    mx, my, myaw = state['map_odom']
                    c, s = math.cos(myaw), math.sin(myaw)
                    row['slam'] = [round(mx + c * ox - s * oy, 4),
                                   round(my + s * ox + c * oy, 4),
                                   round(myaw + oyaw, 5)]
                    row['map_odom'] = [round(mx, 4), round(my, 4),
                                       round(myaw, 5)]
                out.write(json.dumps(row, separators=(',', ':')) + '\n')

    def on_map(msg):
        state['map'] = msg
        state['maps'] += 1

    node.create_subscription(TFMessage, '/tf', on_tf, 200)
    node.create_subscription(
        OccupancyGrid, '/map', on_map, QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(String, '/slam/bridge_status',
                             lambda m: bridge.write(m.data + '\n'), 10)
    def save_map():
        """Write the latest map (called every 10 s: the recorder is
        usually stopped with the flight, before its own duration ends)."""
        info = {'maps_received': state['maps'],
                'map_odom_corrections': state['corrections']}
        grid = state['map']
        if grid is not None:
            data = np.array(grid.data, dtype=np.int8).reshape(
                grid.info.height, grid.info.width)
            tmp = os.path.join(args.out_dir, 'slam_map_tmp.npz')
            np.savez_compressed(
                tmp, data=data, resolution=grid.info.resolution,
                origin=[grid.info.origin.position.x,
                        grid.info.origin.position.y])
            os.replace(tmp, os.path.join(args.out_dir, 'slam_map.npz'))
            info.update({
                'width': grid.info.width, 'height': grid.info.height,
                'resolution': grid.info.resolution,
                'origin': [grid.info.origin.position.x,
                           grid.info.origin.position.y],
                'occupied_cells': int((data > 50).sum()),
                'free_cells': int((data == 0).sum()),
                'unknown_cells': int((data < 0).sum())})
        with open(os.path.join(args.out_dir, 'slam_map_info.json'),
                  'w') as f:
            json.dump(info, f, indent=1)
        out.flush()
        bridge.flush()
        return info

    end = time.monotonic() + args.duration
    next_save = time.monotonic() + 10.0
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if time.monotonic() >= next_save:
            next_save = time.monotonic() + 10.0
            save_map()
    info = save_map()
    out.close()
    bridge.close()
    print(f'log_slam: {info}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
