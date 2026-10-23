#!/usr/bin/env python3
"""Screenshots of a RUNNING Gazebo simulation, from cameras added to the world.

The operating system does not allow desktop screenshots in this session, so
static camera models are created inside the running world (entity factory,
the same call the debris spawner uses) and their frames are saved as PNG at
a fixed interval. The images show what really happens in the simulation:
the UAV, falling debris, the collapse. It reads no ground truth and changes
nothing but adding the cameras.

Usage: capture_views.py --out-dir DIR [--duration 300] [--interval 20]
                        [--delay 10] [--world collapsed_building_rescue]
"""

import argparse
import os
import threading
import time

import cv2
import numpy as np
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node

# name, x, y, z, roll, pitch, yaw (pitch > 0 looks down; yaw 0 = east)
VIEWS = [
    ('overview', -45.0, -40.0, 46.0, 0.0, 0.55, 0.78),
    ('collapse', -34.0, 30.0, 26.0, 0.0, 0.50, 0.30),
    ('top_site', 8.0, 52.0, 52.0, 0.0, 1.45, 0.0),
]


def camera_sdf(name, pose):
    x, y, z, r, p, yw = pose
    return (
        f'<sdf version="1.9"><model name="{name}"><static>true</static>'
        f'<pose>{x} {y} {z} {r} {p} {yw}</pose><link name="l"><sensor '
        'name="c" type="camera"><update_rate>2</update_rate>'
        f'<topic>/dz_view/{name}</topic><camera><horizontal_fov>1.15'
        '</horizontal_fov><image><width>960</width><height>600</height>'
        '</image><clip><near>0.2</near><far>400</far></clip></camera>'
        '</sensor></link></model></sdf>')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--out-dir', required=True)
    parser.add_argument('--duration', type=float, default=300.0)
    parser.add_argument('--interval', type=float, default=20.0)
    parser.add_argument('--delay', type=float, default=10.0)
    parser.add_argument('--world', default='collapsed_building_rescue')
    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    node = Node()
    latest = {}
    lock = threading.Lock()

    def make_cb(name):
        def cb(msg):
            with lock:
                latest[name] = msg
        return cb
    time.sleep(args.delay)
    for name, *pose in VIEWS:
        req = EntityFactory()
        req.sdf = camera_sdf(f'dz_cam_{name}', pose)
        req.name = f'dz_cam_{name}'
        req.allow_renaming = False
        ok, reply = node.request(f'/world/{args.world}/create', req,
                                 EntityFactory, Boolean, 5000)
        print(f'camera {name}: {"created" if ok and reply.data else "FAILED"}',
              flush=True)
        node.subscribe(Image, f'/dz_view/dz_cam_{name}', make_cb(name))
    start = time.monotonic()
    shots = 0
    while time.monotonic() - start < args.duration:
        time.sleep(args.interval)
        stamp = int(time.monotonic() - start)
        with lock:
            frames = dict(latest)
        for name, msg in frames.items():
            img = np.frombuffer(msg.data, np.uint8).reshape(
                msg.height, msg.width, 3)[:, :, ::-1]
            cv2.imwrite(os.path.join(
                args.out_dir, f'{name}_t{stamp:04d}.png'), img)
            shots += 1
    print(f'saved {shots} frames', flush=True)


if __name__ == '__main__':
    main()
