#!/usr/bin/env python3
"""Save frames of a Gazebo camera topic as PNG files (gz-transport, no ROS).

Usage: gz_snapshot.py TOPIC OUT.png [--count 1] [--timeout 20]
With --count N > 1 the files are OUT_0.png ... OUT_{N-1}.png.
Works for RGB (R8G8B8) and depth (R_FLOAT32, saved as an 8-bit image scaled
to 0..30 m) camera images.
"""

import argparse
import sys
import threading

import cv2
import numpy as np
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('topic')
    parser.add_argument('out')
    parser.add_argument('--count', type=int, default=1)
    parser.add_argument('--timeout', type=float, default=20.0)
    args = parser.parse_args()
    frames, done = [], threading.Event()

    def on_image(msg):
        if len(frames) < args.count:
            frames.append(msg)
        if len(frames) >= args.count:
            done.set()

    node = Node()
    node.subscribe(Image, args.topic, on_image)
    if not done.wait(args.timeout):
        print(f'only {len(frames)} frame(s) from {args.topic}', file=sys.stderr)
    for i, msg in enumerate(frames):
        if msg.pixel_format_type == 13 or msg.step == msg.width * 4 and \
                len(msg.data) == msg.width * msg.height * 4 and \
                msg.pixel_format_type != 3:
            depth = np.frombuffer(msg.data, np.float32).reshape(
                msg.height, msg.width)
            img = np.clip(np.nan_to_num(depth, posinf=30.0) / 30.0 * 255,
                          0, 255).astype(np.uint8)
        else:
            img = np.frombuffer(msg.data, np.uint8).reshape(
                msg.height, msg.width, 3)[:, :, ::-1]
        name = args.out if args.count == 1 else \
            args.out.replace('.png', f'_{i}.png')
        cv2.imwrite(name, img)
        print(name)
    return 0 if len(frames) == args.count else 1


if __name__ == '__main__':
    sys.exit(main())
