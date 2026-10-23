#!/usr/bin/env python3
"""Record the search planner's output of one flight (Phase 9).

Writes into --out-dir:
    search_status.jsonl   the planner's JSON status (2 Hz)
    search_goals.jsonl    every SearchGoal whose state or target changed
    flight_status.jsonl   flight_controller state changes

Recording only: nothing here is fed back to any node.
"""

import argparse
import json
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from uav_interfaces.msg import FlightStatus, SearchGoal


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--out-dir', required=True)
    parser.add_argument('--duration', type=float, default=200.0)
    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    rclpy.init()
    node = Node('log_search')
    status_out = open(os.path.join(args.out_dir, 'search_status.jsonl'), 'w')
    goals_out = open(os.path.join(args.out_dir, 'search_goals.jsonl'), 'w')
    flight_out = open(os.path.join(args.out_dir, 'flight_status.jsonl'), 'w')
    last = {'goal': None, 'flight': None}

    def on_goal(msg):
        key = (msg.state, round(msg.target.x, 2), round(msg.target.y, 2),
               msg.complete)
        if key == last['goal']:
            return
        last['goal'] = key
        goals_out.write(json.dumps({
            'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'state': int(msg.state),
            'target': [round(msg.target.x, 3), round(msg.target.y, 3)],
            'speed_limit': round(float(msg.speed_limit), 2),
            'complete': bool(msg.complete),
            'waypoint': [int(msg.waypoint_index), int(msg.waypoint_count)],
            'coverage': round(float(msg.coverage_fraction), 4),
            'inspecting': int(msg.inspecting_id), 'reason': msg.reason},
            separators=(',', ':')) + '\n')

    def on_flight(msg):
        key = (msg.state, msg.avoidance_active)
        if key == last['flight']:
            return
        last['flight'] = key
        flight_out.write(json.dumps({
            'stamp': msg.stamp.sec + msg.stamp.nanosec * 1e-9,
            'state': msg.state, 'avoidance_active': bool(
                msg.avoidance_active),
            'mission_target': [round(msg.mission_target.x, 3),
                               round(msg.mission_target.y, 3),
                               round(msg.mission_target.z, 3)],
            'speed_limit': round(float(msg.speed_limit), 2),
            'avoidance_accepted': int(msg.avoidance_accepted)},
            separators=(',', ':')) + '\n')

    node.create_subscription(SearchGoal, '/planning/search/goal', on_goal, 50)
    node.create_subscription(FlightStatus, '/uav/flight_status', on_flight,
                             50)
    node.create_subscription(
        String, '/planning/search/status',
        lambda m: status_out.write(m.data + '\n'), 50)
    end = time.monotonic() + args.duration
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    for handle in (status_out, goals_out, flight_out):
        handle.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
