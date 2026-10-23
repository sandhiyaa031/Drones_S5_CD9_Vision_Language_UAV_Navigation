#!/usr/bin/env python3
"""Keep the Gazebo window's camera behind the drone (display only).

Gazebo's /gui/follow is accepted but does not move the camera in this setup,
so this reads the drone's pose from the world's pose topic and moves the
window camera (/gui/move_to/pose) a few times per second. It only changes
what the window shows; nothing here is used by the flight software.

Usage: chase_cam.py [--model x500_rescue_0] [--back 6] [--up 4] [--rate 5]
       [--duration 0 (= until Ctrl+C)]
"""

import argparse
import math
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.gui_camera_pb2 import GUICamera
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--model', default='x500_rescue_0')
    ap.add_argument('--world', default='collapsed_building_rescue')
    ap.add_argument('--back', type=float, default=6.0, help='metres behind (south-west of) the drone')
    ap.add_argument('--up', type=float, default=4.0, help='metres above the drone')
    ap.add_argument('--rate', type=float, default=5.0)
    ap.add_argument('--duration', type=float, default=0.0)
    a = ap.parse_args()

    node = Node()
    latest = {}

    def on_poses(msg):
        for p in msg.pose:
            if p.name == a.model:
                latest['p'] = (p.position.x, p.position.y, p.position.z)

    node.subscribe(Pose_V, f'/world/{a.world}/pose/info', on_poses)
    node.subscribe(Pose_V, f'/world/{a.world}/dynamic_pose/info', on_poses)
    # camera looks along +x +y (north-east) from behind and above: the collapse is north-east of the base
    d = a.back / math.sqrt(2.0)
    yaw = math.atan2(1.0, 1.0)
    pitch = math.atan2(a.up, a.back)
    cp, sp, cy, sy = math.cos(pitch / 2), math.sin(pitch / 2), math.cos(yaw / 2), math.sin(yaw / 2)
    q = (-sp * sy, sp * cy, cp * sy, cp * cy)
    cam = None
    t_end = time.time() + a.duration if a.duration > 0 else float('inf')
    warned = False
    while time.time() < t_end:
        p = latest.get('p')
        if p is None:
            if not warned:
                print(f'chase_cam: waiting for {a.model} ...', flush=True)
                warned = True
            time.sleep(0.5)
            continue
        target = (p[0] - d, p[1] - d, p[2] + a.up)
        # smooth the camera so it glides instead of jumping
        cam = target if cam is None else tuple(c + 0.5 * (t - c) for c, t in zip(cam, target))
        req = GUICamera()
        req.pose.position.x, req.pose.position.y, req.pose.position.z = cam
        req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z, req.pose.orientation.w = q
        node.request('/gui/move_to/pose', req, GUICamera, Boolean, 1000)
        time.sleep(1.0 / a.rate)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
