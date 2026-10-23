#!/usr/bin/env python3
"""Subscribe to Gazebo sensor topics and discard the data (forces rendering).

Usage: gz_sink.py SECONDS cam|depth|lidar [...]
Used by rtf_probe.sh: Gazebo renders a sensor only while someone subscribes.
"""
import sys
import time

from gz.msgs10.image_pb2 import Image
from gz.msgs10.laserscan_pb2 import LaserScan
from gz.transport13 import Node

TOPICS = {
    'cam': ('/world/collapsed_building_rescue/model/x500_rescue_0/link/'
            'camera_link/sensor/camera/image', Image),
    'depth': ('/uav/depth_up/image', Image),
    'lidar': ('/uav/scan', LaserScan)}


def main():
    secs, names = float(sys.argv[1]), sys.argv[2:]
    node, counts = Node(), {n: 0 for n in names}

    def make(n):
        def cb(_msg):
            counts[n] += 1
        return cb
    for n in names:
        topic, msg_type = TOPICS[n]
        node.subscribe(msg_type, topic, make(n))
    time.sleep(secs)
    print({n: round(c / secs, 1) for n, c in counts.items()}, 'Hz')


if __name__ == '__main__':
    main()
