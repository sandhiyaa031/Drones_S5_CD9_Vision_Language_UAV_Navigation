#!/usr/bin/env python3
"""Record the mission manager's output of one flight (Phase 10).

Writes into --out-dir:
    mission_status.jsonl     MissionStatus when the phase or a count changes
    mission_commands.jsonl   every new MissionCommand (by sequence number)

Recording only: nothing here is fed back to any node.
"""

import argparse
import json
import os
import time

import rclpy
from rclpy.node import Node
from uav_interfaces.msg import MissionCommand, MissionStatus


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--out-dir', required=True)
    parser.add_argument('--duration', type=float, default=250.0)
    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    rclpy.init()
    node = Node('log_mission')
    status_out = open(os.path.join(args.out_dir, 'mission_status.jsonl'), 'w')
    command_out = open(os.path.join(args.out_dir, 'mission_commands.jsonl'),
                       'w')
    last = {'status': None, 'sequence': 0}

    def on_status(msg):
        key = (msg.phase, msg.flight_state, msg.candidates, msg.confirmed,
               msg.end_reason, tuple(msg.failed_checks),
               msg.avoidance_manoeuvres, round(msg.coverage_fraction, 1))
        if key == last['status']:
            return
        last['status'] = key
        status_out.write(json.dumps({
            'wall': round(time.time(), 2), 'phase': msg.phase_name,
            'flight_state': msg.flight_state,
            'elapsed': round(float(msg.elapsed), 1),
            'search_elapsed': round(float(msg.search_elapsed), 1),
            'coverage': round(float(msg.coverage_fraction), 3),
            'candidates': int(msg.candidates),
            'confirmed': int(msg.confirmed),
            'vlm_available': bool(msg.vlm_available),
            'battery': round(float(msg.battery_remaining), 3),
            'avoidance_manoeuvres': int(msg.avoidance_manoeuvres),
            'end_reason': msg.end_reason,
            'failed_checks': list(msg.failed_checks)},
            separators=(',', ':')) + '\n')
        status_out.flush()

    def on_command(msg):
        if msg.sequence == last['sequence']:
            return
        last['sequence'] = msg.sequence
        command_out.write(json.dumps({
            'wall': round(time.time(), 2), 'command': int(msg.command),
            'sequence': int(msg.sequence), 'reason': msg.reason},
            separators=(',', ':')) + '\n')
        command_out.flush()

    node.create_subscription(MissionStatus, '/mission/status', on_status, 20)
    node.create_subscription(MissionCommand, '/mission/command', on_command,
                             20)
    end = time.monotonic() + args.duration
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    status_out.close()
    command_out.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
