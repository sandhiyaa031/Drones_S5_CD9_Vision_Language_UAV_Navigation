#!/usr/bin/env python3
"""Record the four time bases needed to judge PX4/Gazebo synchronisation.

  A  Gazebo simulation time            (/world/<w>/clock)
  B  depth image header stamp          (/uav/depth_up/image, Gazebo time)
  C  PX4 timestamps                    (vehicle_odometry, sensor_combined)
  D  arrival: wall clock and Gazebo time at the moment of reception

Also records the Gazebo IMU (simulation-stamped) so that PX4's gyro samples
can be matched to the very same physical samples, and the ground-truth UAV
height. Read-only; evaluation data only.
"""

import argparse
import os
import threading
import time

from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.image_pb2 import Image as GzImage
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
import numpy as np
from px4_msgs.msg import SensorCombined, VehicleOdometry
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--world', default=os.environ.get(
        'WORLD', 'collapsed_building_rescue'))
    parser.add_argument('--uav', default=os.environ.get(
        'UAV_NAME', 'x500_rescue_0'))
    parser.add_argument('--duration', type=float, default=300.0)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()

    sim = {'t': float('nan')}
    lock = threading.Lock()
    data = {k: [] for k in ('clock', 'gz_imu', 'depth', 'truth', 'px4_imu',
                            'odom')}

    def on_clock(msg):
        sim['t'] = msg.sim.sec + msg.sim.nsec * 1e-9
        with lock:
            data['clock'].append((time.time(), sim['t']))

    def on_imu(msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        w = msg.angular_velocity
        with lock:
            data['gz_imu'].append((stamp, w.x, w.y, w.z))

    def on_depth(msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        with lock:
            data['depth'].append((stamp, time.time(), sim['t']))

    def on_pose(msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        for p in msg.pose:
            if p.name == args.uav:
                with lock:
                    data['truth'].append((stamp, p.position.x, p.position.y,
                                          p.position.z))

    gz = GzNode()
    gz.subscribe(Clock, f'/world/{args.world}/clock', on_clock)
    gz.subscribe(IMU, f'/world/{args.world}/model/{args.uav}/link/base_link/'
                 'sensor/imu_sensor/imu', on_imu)
    gz.subscribe(GzImage, '/uav/depth_up/image', on_depth)
    gz.subscribe(Pose_V, f'/world/{args.world}/pose/info', on_pose)

    rclpy.init()
    node = Node('time_record')
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=50)

    def on_px4_imu(msg):
        g = msg.gyro_rad
        data['px4_imu'].append((msg.timestamp * 1e-6, float(g[0]),
                                float(g[1]), float(g[2]), time.time(),
                                sim['t'], msg.gyro_integral_dt * 1e-6))

    def on_odom(msg):
        data['odom'].append((msg.timestamp * 1e-6,
                             msg.timestamp_sample * 1e-6, time.time(),
                             sim['t'], float(msg.position[0]),
                             float(msg.position[1]), float(msg.position[2])))

    node.create_subscription(
        SensorCombined, '/fmu/out/sensor_combined', on_px4_imu, qos)
    node.create_subscription(
        VehicleOdometry, '/fmu/out/vehicle_odometry', on_odom, qos)

    t0 = time.monotonic()
    while time.monotonic() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.02)
    with lock:
        arrays = {k: np.array(v, dtype=np.float64) for k, v in data.items()}
    np.savez_compressed(args.output, **arrays)
    print({k: len(v) for k, v in arrays.items()}, flush=True)
    os._exit(0)


if __name__ == '__main__':
    main()
