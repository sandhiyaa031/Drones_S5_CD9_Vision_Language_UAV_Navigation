#!/usr/bin/env python3
"""Measure simulator and camera performance over one common time window.

Simultaneously samples, for --duration wall seconds:
  * Gazebo real-time factor (sim-time advance / wall-time advance, /clock)
  * Gazebo-native camera image rate (gz transport)
  * Gazebo-native upward depth image rate
  * ROS /camera/image_raw, /camera/camera_info, /depth_up/image_raw and
    /depth_up/camera_info rates
  * Gazebo server resident memory
  * PX4 vehicle_status and vehicle_local_position rates and largest gap
  * NVIDIA GPU utilisation and which processes hold a GPU context

Prints JSON. Read-only: publishes nothing.
"""

import argparse
import json
import os
import subprocess
import sys
import threading
import time

from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.image_pb2 import Image as GzImage
from gz.transport13 import Node as GzNode
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
    qos_profile_sensor_data)
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from sensor_msgs.msg import CameraInfo, Image


class Counter:
    """Arrival counter with wall-clock gap statistics."""

    def __init__(self):
        self.stamps = []
        self.lock = threading.Lock()

    def tick(self, *_):
        with self.lock:
            self.stamps.append(time.monotonic())

    def summary(self, t0, t1):
        with self.lock:
            s = [x for x in self.stamps if t0 <= x <= t1]
        if len(s) < 2:
            return {'count': len(s), 'rate_hz': 0.0, 'max_gap_s': None}
        gaps = [b - a for a, b in zip(s, s[1:])]
        return {'count': len(s),
                'rate_hz': round((len(s) - 1) / (s[-1] - s[0]), 2),
                'max_gap_s': round(max(gaps), 3),
                'mean_gap_s': round(sum(gaps) / len(gaps), 4)}


def gpu_snapshot():
    try:
        util = subprocess.run(
            ['nvidia-smi', '--query-gpu=utilization.gpu,memory.used',
             '--format=csv,noheader,nounits'],
            capture_output=True, text=True, timeout=5).stdout.strip()
        table = subprocess.run(
            ['nvidia-smi'], capture_output=True, text=True,
            timeout=5).stdout
        procs = [line.split()[-3] + ' ' + line.split()[-2]
                 for line in table.splitlines()
                 if (' G ' in line or ' C ' in line or ' C+G ' in line)
                 and 'MiB' in line]
        gpu, mem = (v.strip() for v in util.split(','))
        return {'util_percent': float(gpu), 'memory_mib': float(mem),
                'gpu_processes': procs}
    except Exception as exc:  # nvidia-smi absent or failed
        return {'error': str(exc)}


def gz_server_rss_mib():
    """Resident memory of the Gazebo server process, in MiB."""
    out = subprocess.run(['ps', '-eo', 'rss,comm,args'], capture_output=True,
                         text=True).stdout
    for line in out.splitlines():
        parts = line.split(None, 2)
        if (len(parts) == 3 and parts[1].startswith('ruby')
                and 'gz sim' in parts[2] and ' -s ' in parts[2]):
            return round(int(parts[0]) / 1024.0, 1)
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--world', default=os.environ.get(
        'WORLD', 'collapsed_building_rescue'))
    parser.add_argument('--model',
                        default=os.environ.get('UAV_NAME', 'x500_rescue_0'))
    parser.add_argument('--duration', type=float, default=20.0)
    parser.add_argument('--label', default='')
    parser.add_argument('--output')
    args = parser.parse_args()

    gz_image_topic = (f'/world/{args.world}/model/{args.model}/link/'
                      'camera_link/sensor/camera/image')
    gz_cam = Counter()
    gz_depth = Counter()
    clock = {'first': None, 'last': None}

    def on_clock(msg):
        sim = msg.sim.sec + msg.sim.nsec * 1e-9
        now = time.monotonic()
        if clock['first'] is None:
            clock['first'] = (now, sim)
        clock['last'] = (now, sim)

    gz_node = GzNode()
    gz_node.subscribe(GzImage, gz_image_topic, gz_cam.tick)
    gz_node.subscribe(GzImage, '/uav/depth_up/image', gz_depth.tick)
    gz_node.subscribe(Clock, f'/world/{args.world}/clock', on_clock)

    rclpy.init()
    node = Node('measure_sim')
    px4_qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST, depth=5)
    ros_img, ros_info, status, position, ros_depth, ros_depth_info = (
        Counter() for _ in range(6))
    size = {}

    def on_image(msg):
        ros_img.tick()
        size.update(width=msg.width, height=msg.height,
                    encoding=msg.encoding, frame_id=msg.header.frame_id)

    node.create_subscription(
        Image, '/camera/image_raw', on_image, qos_profile_sensor_data)
    node.create_subscription(
        CameraInfo, '/camera/camera_info', ros_info.tick,
        qos_profile_sensor_data)
    node.create_subscription(
        Image, '/depth_up/image_raw', ros_depth.tick,
        qos_profile_sensor_data, raw=True)
    node.create_subscription(
        CameraInfo, '/depth_up/camera_info', ros_depth_info.tick,
        qos_profile_sensor_data)
    node.create_subscription(
        VehicleStatus, '/fmu/out/vehicle_status_v4', status.tick, px4_qos)
    node.create_subscription(
        VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
        position.tick, px4_qos)

    time.sleep(2.0)  # discovery
    clock['first'] = None
    t0 = time.monotonic()
    gpu_samples = []
    next_gpu = t0
    while time.monotonic() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.monotonic() >= next_gpu:
            gpu_samples.append(gpu_snapshot())
            next_gpu += 2.0
    t1 = time.monotonic()

    rtf = None
    if clock['first'] and clock['last'] and clock['last'][0] > clock['first'][0]:
        rtf = round((clock['last'][1] - clock['first'][1]) /
                    (clock['last'][0] - clock['first'][0]), 3)
    utils = [g['util_percent'] for g in gpu_samples if 'util_percent' in g]
    report = {
        'label': args.label,
        'duration_s': round(t1 - t0, 2),
        'real_time_factor': rtf,
        'gazebo_camera': gz_cam.summary(t0, t1),
        'ros_camera_image_raw': ros_img.summary(t0, t1),
        'ros_camera_info': ros_info.summary(t0, t1),
        'gazebo_depth_up': gz_depth.summary(t0, t1),
        'ros_depth_up_image_raw': ros_depth.summary(t0, t1),
        'ros_depth_up_camera_info': ros_depth_info.summary(t0, t1),
        'gazebo_server_rss_mib': gz_server_rss_mib(),
        'image': size,
        'px4_vehicle_status': status.summary(t0, t1),
        'px4_local_position': position.summary(t0, t1),
        'gpu_util_percent_mean': (
            round(sum(utils) / len(utils), 1) if utils else None),
        'gpu_util_percent_max': max(utils) if utils else None,
        'gpu_last': gpu_samples[-1] if gpu_samples else None,
    }
    text = json.dumps(report, indent=2)
    print(text)
    if args.output:
        with open(args.output, 'w') as handle:
            handle.write(text + '\n')
    node.destroy_node()
    rclpy.try_shutdown()
    # gz-transport's Python node aborts in its destructor at interpreter
    # exit; the report is already written, so leave without running it.
    sys.stdout.flush()
    os._exit(0)


if __name__ == '__main__':
    main()
