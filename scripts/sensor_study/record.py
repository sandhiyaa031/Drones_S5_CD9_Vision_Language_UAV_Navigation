#!/usr/bin/env python3
"""Sensor-study recorder: camera frames plus Gazebo ground-truth poses.

Records, directly from Gazebo transport (no ROS bridge in the path):
  * every onboard camera frame as JPEG, named by its simulation timestamp
  * ground-truth poses of the UAV, its camera link and all debris models

Ground truth is recorded for evaluation only. Nothing here feeds perception
or control, and nothing is published.
"""

import argparse
import csv
import os
import signal
import threading
import time

import cv2
from gz.msgs10.image_pb2 import Image
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--out-dir', required=True)
    parser.add_argument('--world', default=os.environ.get(
        'WORLD', 'collapsed_building_rescue'))
    parser.add_argument('--model', default=os.environ.get(
        'UAV_NAME', 'x500_mono_cam_down_0'))
    parser.add_argument('--depth', action='store_true',
                        help='also record /uav/depth_up/image as 16-bit '
                             'PNG in millimetres (0 = closer than near '
                             'clip, 65535 = nothing in range)')
    parser.add_argument('--no-mono', action='store_true')
    parser.add_argument('--poses-only', action='store_true',
                        help='record ground-truth poses only (no images)')
    parser.add_argument('--debris-prefix', default='p1_debris_')
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--jpeg-quality', type=int, default=85)
    parser.add_argument('--frame-every', type=int, default=1,
                        help='save every Nth camera frame (default: all)')
    args = parser.parse_args()

    frames_dir = os.path.join(args.out_dir, 'frames_raw')
    os.makedirs(frames_dir, exist_ok=True)
    pose_file = open(os.path.join(args.out_dir, 'poses.csv'), 'w', newline='')
    poses = csv.writer(pose_file)
    poses.writerow(['t_sim', 'name', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
    frame_file = open(os.path.join(args.out_dir, 'frames.csv'), 'w',
                      newline='')
    frames = csv.writer(frame_file)
    frames.writerow(['t_sim', 't_wall', 'file', 'width', 'height'])
    lock = threading.Lock()
    counts = {'frames': 0, 'poses': 0}
    wanted = (args.model, 'camera_link', 'base_link', 'depth_up_link')

    def on_pose(msg):
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        rows = []
        for p in msg.pose:
            if p.name in wanted or p.name.startswith(args.debris_prefix):
                rows.append([f'{t:.6f}', p.name,
                             p.position.x, p.position.y, p.position.z,
                             p.orientation.x, p.orientation.y,
                             p.orientation.z, p.orientation.w])
        with lock:
            poses.writerows(rows)
            counts['poses'] += 1

    def on_image(msg):
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        counts['seen'] = counts.get('seen', 0) + 1
        if counts['seen'] % args.frame_every:
            return
        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)
        name = f'{t:010.3f}.jpg'
        cv2.imwrite(os.path.join(frames_dir, name),
                    cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                    [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality])
        with lock:
            frames.writerow([f'{t:.6f}', f'{time.time():.3f}', name,
                             msg.width, msg.height])
            counts['frames'] += 1

    depth_dir = os.path.join(args.out_dir, 'frames_raw', 'depth')
    if args.depth:
        os.makedirs(depth_dir, exist_ok=True)
        depth_file = open(os.path.join(args.out_dir, 'depth_frames.csv'),
                          'w', newline='')
        depth_rows = csv.writer(depth_file)
        depth_rows.writerow(['t_sim', 't_wall', 'file', 'finite_pixels'])

    def on_depth(msg):
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(
            msg.height, msg.width)
        mm = np.full(depth.shape, 65535, dtype=np.uint16)
        finite = np.isfinite(depth)
        mm[finite] = np.clip(np.rint(depth[finite] * 1000.0), 1, 65534)
        mm[np.isneginf(depth)] = 0
        name = f'{t:010.3f}.png'
        cv2.imwrite(os.path.join(depth_dir, name), mm)
        with lock:
            depth_rows.writerow([f'{t:.6f}', f'{time.time():.3f}', name,
                                 int(finite.sum())])
            counts['depth'] = counts.get('depth', 0) + 1

    node = Node()
    node.subscribe(Pose_V, f'/world/{args.world}/pose/info', on_pose)
    if args.poses_only:
        args.no_mono, args.depth = True, False
    if not args.no_mono:
        node.subscribe(
            Image,
            f'/world/{args.world}/model/{args.model}/link/camera_link/'
            'sensor/camera/image', on_image)
    if args.depth:
        node.subscribe(Image, '/uav/depth_up/image', on_depth)
    print(f'recording to {args.out_dir} for {args.duration:.0f} s',
          flush=True)
    # Finish cleanly on SIGINT/SIGTERM so a teardown does not lose data.
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    stop.wait(args.duration)
    with lock:
        pose_file.close()
        frame_file.close()
        if args.depth:
            depth_file.close()
            print(f"depth_frames={counts.get('depth', 0)}", flush=True)
        print(f"frames={counts['frames']} pose_messages={counts['poses']}",
              flush=True)
    os._exit(0)  # gz-transport's Python node aborts in its destructor


if __name__ == '__main__':
    main()
