#!/usr/bin/env python3
"""Save onboard camera frames once the UAV is airborne (evidence capture).

Waits until PX4 local position shows the vehicle at least --min-altitude above
its starting height, then writes --count frames from /camera/image_raw as PNG
together with a JSON sidecar (altitude, position, image stamp). Read-only.
"""

import argparse
import json
import os
import time

import cv2
from cv_bridge import CvBridge
from px4_msgs.msg import VehicleLocalPosition
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
    qos_profile_sensor_data)
from sensor_msgs.msg import CameraInfo, Image


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--out-dir', required=True)
    parser.add_argument('--min-altitude', type=float, default=2.8)
    parser.add_argument('--count', type=int, default=3)
    parser.add_argument('--interval', type=float, default=5.0)
    parser.add_argument('--timeout', type=float, default=120.0)
    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    rclpy.init()
    node = Node('capture_airborne_frame')
    bridge = CvBridge()
    state = {'ground_z': None, 'pos': None, 'saved': 0, 'last': 0.0,
             'info_saved': False}

    def on_position(msg):
        if state['ground_z'] is None and msg.z_valid:
            state['ground_z'] = float(msg.z)
        state['pos'] = msg

    def on_info(msg):
        if state['info_saved']:
            return
        with open(os.path.join(args.out_dir, 'camera_info.json'), 'w') as f:
            json.dump({'width': msg.width, 'height': msg.height,
                       'frame_id': msg.header.frame_id,
                       'distortion_model': msg.distortion_model,
                       'd': list(msg.d), 'k': list(msg.k),
                       'p': list(msg.p)}, f, indent=2)
        state['info_saved'] = True

    def on_image(msg):
        pos = state['pos']
        if pos is None or state['ground_z'] is None:
            return
        altitude = state['ground_z'] - float(pos.z)
        now = time.monotonic()
        if altitude < args.min_altitude or now - state['last'] < args.interval:
            return
        index = state['saved']
        bgr = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        name = f'airborne_{index:02d}'
        cv2.imwrite(os.path.join(args.out_dir, name + '.png'), bgr)
        with open(os.path.join(args.out_dir, name + '.json'), 'w') as f:
            json.dump({
                'altitude_above_start_m': round(altitude, 3),
                'ned_position_m': [round(float(pos.x), 3),
                                   round(float(pos.y), 3),
                                   round(float(pos.z), 3)],
                'heading_rad': round(float(pos.heading), 4),
                'image_stamp': {'sec': msg.header.stamp.sec,
                                'nanosec': msg.header.stamp.nanosec},
                'width': msg.width, 'height': msg.height,
                'encoding': msg.encoding,
                'frame_id': msg.header.frame_id}, f, indent=2)
        state['saved'] += 1
        state['last'] = now
        print(f'saved {name}.png at {altitude:.2f} m', flush=True)

    px4_qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST, depth=5)
    node.create_subscription(
        VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
        on_position, px4_qos)
    node.create_subscription(
        Image, '/camera/image_raw', on_image, qos_profile_sensor_data)
    node.create_subscription(
        CameraInfo, '/camera/camera_info', on_info, qos_profile_sensor_data)

    deadline = time.monotonic() + args.timeout
    while (rclpy.ok() and state['saved'] < args.count
           and time.monotonic() < deadline):
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.try_shutdown()
    raise SystemExit(0 if state['saved'] >= args.count else 1)


if __name__ == '__main__':
    main()
