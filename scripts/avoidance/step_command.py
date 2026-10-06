#!/usr/bin/env python3
"""Calibration tool: send scripted avoidance commands to flight_controller.

Publishes uav_interfaces/AvoidanceCommand on the same high-level interface
the planner uses (never to /fmu/in/*). Each step displaces the setpoint from
the mission setpoint for a fixed time, then releases it. Used to measure the
vehicle's closed-loop response (docs/debris_avoidance.md).

Steps: "north,east,up,seconds;..." in metres relative to the mission setpoint.
"""

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from uav_interfaces.msg import AvoidanceCommand, FlightStatus


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--steps', required=True)
    parser.add_argument('--gap', type=float, default=5.0,
                        help='seconds between steps (sim time)')
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    steps = [[float(v) for v in s.split(',')] for s in args.steps.split(';')]

    rclpy.init()
    node = Node('avoidance_step_command')
    pub = node.create_publisher(
        AvoidanceCommand, '/perception/debris/avoidance_command', 10)
    status = {}
    node.create_subscription(
        FlightStatus, '/uav/flight_status',
        lambda m: status.update(msg=m), 10)

    def sim_time():
        m = status.get('msg')
        return None if m is None else m.stamp.sec + m.stamp.nanosec * 1e-9

    def spin_until(condition, timeout=120.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)
            if condition():
                return True
        return False

    if not spin_until(lambda: 'msg' in status
                      and status['msg'].state == 'HOLD'):
        raise SystemExit('controller never reached HOLD')
    log = []
    for north, east, up, seconds in steps:
        t0 = sim_time()
        spin_until(lambda: sim_time() - t0 >= args.gap)
        m = status['msg']
        target = (m.mission_target.x + north, m.mission_target.y + east,
                  m.mission_target.z - up)
        start = sim_time()
        while sim_time() - start < seconds:
            cmd = AvoidanceCommand()
            cmd.header.frame_id = 'px4_local_ned'
            cmd.state = AvoidanceCommand.STATE_AVOIDING
            cmd.active = True
            cmd.target.x, cmd.target.y, cmd.target.z = target
            cmd.candidate = 'calibration'
            cmd.reason = 'step response calibration'
            pub.publish(cmd)
            rclpy.spin_once(node, timeout_sec=0.02)
        log.append({'t_start': start, 't_end': sim_time(),
                    'step': [north, east, up], 'target': list(target)})
        print(f'step {north},{east},{up} done', flush=True)
    json.dump(log, open(args.out, 'w'), indent=1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
