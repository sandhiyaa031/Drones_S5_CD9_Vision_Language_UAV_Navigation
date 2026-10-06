#!/usr/bin/env python3
"""Log the debris detector's output to JSON lines (evaluation tooling).

Records /perception/debris/status (which carries every detection of every
frame) and /perception/debris/tracks_debug (every track of every frame) with the wall-clock arrival time, and optionally saves debug images.
"""

import argparse
import os
import signal
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--out-dir', required=True)
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--save-debug-every', type=int, default=3,
                        help='save every Nth debug image that shows a '
                             'detection (0 = none)')
    args = parser.parse_args()
    os.makedirs(os.path.join(args.out_dir, 'debug_frames'), exist_ok=True)
    rclpy.init()
    node = Node('log_detections')
    out = open(os.path.join(args.out_dir, 'detections.jsonl'), 'w')
    state = {'frames': 0, 'with_det': 0, 'debug': 0, 'last_count': 0}

    def on_status(msg):
        out.write(f'{{"t_wall":{time.time():.4f},"d":{msg.data}}}\n')
        state['frames'] += 1
        state['last_count'] = msg.data.count('"p_sensor"')
        state['with_det'] += state['last_count'] > 0

    def on_debug(msg):
        if not args.save_debug_every or state['last_count'] == 0:
            return
        state['debug'] += 1
        if state['debug'] % args.save_debug_every:
            return
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        cv2.imwrite(os.path.join(args.out_dir, 'debug_frames',
                                 f'{stamp:010.3f}.jpg'), image,
                    [cv2.IMWRITE_JPEG_QUALITY, 85])

    node.create_subscription(String, '/perception/debris/status', on_status,
                             50)
    tracks_out = open(os.path.join(args.out_dir, 'tracks.jsonl'), 'w')

    def on_tracks(msg):
        tracks_out.write(msg.data + '\n')
        state['track_frames'] = state.get('track_frames', 0) + 1

    node.create_subscription(String, '/perception/debris/tracks_debug',
                             on_tracks, 50)
    predictions_out = open(os.path.join(args.out_dir, 'predictions.jsonl'),
                           'w')

    def on_predictions(msg):
        predictions_out.write(msg.data + '\n')
        state['prediction_frames'] = state.get('prediction_frames', 0) + 1

    node.create_subscription(
        String, '/perception/debris/predictions_debug', on_predictions, 50)
    risk_out = open(os.path.join(args.out_dir, 'risk.jsonl'), 'w')

    def on_risk(msg):
        risk_out.write(msg.data + '\n')
        state['risk_frames'] = state.get('risk_frames', 0) + 1

    node.create_subscription(String, '/perception/debris/risk_debug',
                             on_risk, 50)
    avoidance_out = open(os.path.join(args.out_dir, 'avoidance.jsonl'), 'w')

    def on_avoidance(msg):
        avoidance_out.write(msg.data + '\n')

    node.create_subscription(String, '/perception/debris/avoidance_debug',
                             on_avoidance, 100)
    node.create_subscription(Image, '/perception/debris/debug_image',
                             on_debug, 5)
    stop = {'now': False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(now=True))
    signal.signal(signal.SIGTERM, lambda *_: stop.update(now=True))
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.duration and not stop['now']:
        try:
            rclpy.spin_once(node, timeout_sec=0.05)
        except Exception:   # context shut down underneath us at teardown
            break
    out.close()
    tracks_out.close()
    predictions_out.close()
    risk_out.close()
    avoidance_out.close()
    print(f"risk frames={state.get('risk_frames', 0)}", flush=True)
    print(f"prediction frames={state.get('prediction_frames', 0)}",
          flush=True)
    print(f"track frames={state.get('track_frames', 0)}", flush=True)
    print(f"status frames={state['frames']} with detections="
          f"{state['with_det']}", flush=True)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
