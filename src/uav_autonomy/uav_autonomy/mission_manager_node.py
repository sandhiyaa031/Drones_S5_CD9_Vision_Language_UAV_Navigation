#!/usr/bin/env python3
"""ROS 2 node: mission manager (Phase 10).

Subscribes (status of the vehicle's own functions only)
    /uav/flight_status                    uav_interfaces/FlightStatus
    /perception/survivor/detections       uav_interfaces/SurvivorDetectionArray
    /perception/survivor/locations        uav_interfaces/SurvivorLocationArray
    /planning/search/goal                 uav_interfaces/SearchGoal
    /perception/debris/avoidance_command  uav_interfaces/AvoidanceCommand
    /fmu/out/battery_status[_v1]          px4_msgs/BatteryStatus
    /fmu/out/vehicle_local_position_v1    px4_msgs/VehicleLocalPosition
                                          (origin of the local frame, for
                                          latitude/longitude in the report)

Publishes
    /mission/command   uav_interfaces/MissionCommand  (START, RETURN_HOME);
                       flight_controller validates it and is the only node
                       that commands PX4
    /mission/status    uav_interfaces/MissionStatus, 2 Hz
    /mission/report    std_msgs/String, JSON, once at the end (latched)

The report is also written to the file given by `report_path`. This node
never publishes to /fmu/in/* and never reads Gazebo ground truth. Logic:
mission_logic.py.
"""

import json
import math
import time

from px4_msgs.msg import BatteryStatus, VehicleLocalPosition
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)
from std_msgs.msg import String
from uav_interfaces.msg import (
    AvoidanceCommand, FlightStatus, MissionCommand, MissionStatus,
    SearchGoal, SurvivorDetectionArray, SurvivorLocationArray)

from uav_autonomy.mission_interface import CMD_NONE, COMMAND_NAMES
from uav_autonomy.mission_logic import (
    Inputs,
    MissionManager,
    MissionSpec,
    PHASE_NAMES,
    PHASE_SEARCH,
    TERMINAL,
    VLM_AVAILABLE,
    classify,
)


class MissionManagerNode(Node):
    """Wrap MissionManager with ROS interfaces."""

    def __init__(self):
        """Load the mission and create publishers and subscribers."""
        super().__init__('mission_manager')
        self.declare_parameter('mission_file', '')
        self.declare_parameter('report_path', '')
        path = str(self.get_parameter('mission_file').value)
        self.report_path = str(self.get_parameter('report_path').value)
        spec = MissionSpec.load(path) if path else MissionSpec()
        self.manager = MissionManager(spec)
        self.rx = {}                 # stream name -> monotonic receive time
        self.flight = None
        self.vlm_status = None
        self.locations = []
        self.goal = None
        self.avoidance_seen = False
        self.battery = None
        self.origin = None
        self.home = None
        self.reported = False
        self.last_command_log = None

        self.pub_command = self.create_publisher(
            MissionCommand, '/mission/command', 10)
        self.pub_status = self.create_publisher(
            MissionStatus, '/mission/status', 10)
        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub_report = self.create_publisher(
            String, '/mission/report', latched)

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(
            FlightStatus, '/uav/flight_status', self.on_flight, 10)
        self.create_subscription(
            SurvivorDetectionArray, '/perception/survivor/detections',
            self.on_detections, 10)
        self.create_subscription(
            SurvivorLocationArray, '/perception/survivor/locations',
            self.on_locations, 10)
        self.create_subscription(
            SearchGoal, '/planning/search/goal', self.on_goal, 10)
        self.create_subscription(
            AvoidanceCommand, '/perception/debris/avoidance_command',
            self.on_avoidance, 10)
        for topic in ('/fmu/out/battery_status_v1',
                      '/fmu/out/battery_status'):
            self.create_subscription(BatteryStatus, topic, self.on_battery,
                                     px4_qos)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.on_position, px4_qos)
        self.create_timer(0.5, self.on_timer)
        problems = spec.problems()
        self.get_logger().info(
            f'Mission manager ready. Mission {spec.name!r}: area north '
            f'[{spec.north_min:.1f}, {spec.north_max:.1f}] east '
            f'[{spec.east_min:.1f}, {spec.east_max:.1f}] m, altitude '
            f'{spec.altitude_m:.1f} m, budget {spec.time_budget_s:.0f} s, '
            f'stop policy {spec.stop_policy}, require_vlm={spec.require_vlm}'
            + (f'. PROBLEMS: {problems}' if problems else ''))

    def _mark(self, name):
        self.rx[name] = time.monotonic()

    def _age(self, name):
        return time.monotonic() - self.rx[name] if name in self.rx \
            else float('inf')

    def on_flight(self, msg):
        """Flight controller state."""
        self.flight = msg
        self._mark('flight')
        if self.home is None and msg.state == 'SEARCH':
            self.home = (float(msg.mission_target.x),
                         float(msg.mission_target.y))

    def on_detections(self, msg):
        """Survivor perception is alive; note the VLM status."""
        self.vlm_status = int(msg.vlm_status)
        self._mark('detections')

    def on_locations(self, msg):
        """Located candidates (Phase 8)."""
        self.locations = [{
            'id': int(m.survivor_id),
            'ned': [m.position.x, m.position.y, m.position.z],
            'sigma_h': float(m.horizontal_sigma),
            'state': int(m.localization_state),
            'obs': int(m.observations),
            'confirmed': bool(m.vlm_confirmed),
            'rejected': bool(m.vlm_rejected)} for m in msg.locations]
        self._mark('locations')

    def on_goal(self, msg):
        """Search planner output."""
        self.goal = msg

    def on_avoidance(self, msg):
        """The debris chain is alive."""
        self.avoidance_seen = True
        self._mark('avoidance')
        if msg.perception_health != AvoidanceCommand.HEALTH_STALE:
            self._mark('debris_perception')

    def on_battery(self, msg):
        """Battery level from PX4."""
        if msg.connected and 0.0 <= msg.remaining <= 1.0:
            self.battery = float(msg.remaining)

    def on_position(self, msg):
        """Global origin of the local frame."""
        if msg.xy_global and math.isfinite(msg.ref_lat) \
                and math.isfinite(msg.ref_lon):
            self.origin = (float(msg.ref_lat), float(msg.ref_lon))

    def on_timer(self):
        """Run one mission cycle."""
        now = time.monotonic()
        x = Inputs(
            now=now,
            flight_state=None if self.flight is None else self.flight.state,
            flight_age=self._age('flight'),
            detections_age=self._age('detections'),
            vlm_status=self.vlm_status,
            locations_age=self._age('locations'),
            locations=self.locations,
            planner_present=self.count_publishers(
                '/planning/search/goal') > 0,
            search_complete=bool(self.goal is not None
                                 and self.goal.complete),
            coverage=0.0 if self.goal is None else float(
                self.goal.coverage_fraction),
            inspections_done=0 if self.goal is None else int(
                self.goal.inspections_done),
            avoidance_age=self._age('avoidance') if self.avoidance_seen
            else None,
            debris_perception_age=self._age('debris_perception')
            if 'debris_perception' in self.rx else None,
            avoidance_manoeuvres=0 if self.flight is None else int(
                self.flight.avoidance_accepted),
            battery_remaining=self.battery)
        before = self.manager.phase
        decision = self.manager.update(x)
        if self.manager.phase != before:
            note = self.manager.timeline[-1]['note']
            self.get_logger().info(
                f'MISSION {PHASE_NAMES[self.manager.phase]}'
                + (f': {note}' if note else ''))
        if decision.command != CMD_NONE:
            msg = MissionCommand()
            msg.stamp = self.get_clock().now().to_msg()
            msg.command = decision.command
            msg.sequence = decision.sequence
            msg.reason = decision.reason
            self.pub_command.publish(msg)
            if self.last_command_log != decision.sequence:
                self.last_command_log = decision.sequence
                self.get_logger().info(
                    f'MISSION command {COMMAND_NAMES[decision.command]}: '
                    f'{decision.reason}')
        self.publish_status(x)
        if self.manager.phase in TERMINAL and not self.reported:
            self.reported = True
            self.write_report()

    def publish_status(self, x):
        """MissionStatus for logging and for an operator."""
        m = self.manager
        msg = MissionStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.mission_name = m.spec.name
        msg.phase = m.phase
        msg.phase_name = PHASE_NAMES[m.phase]
        msg.flight_state = x.flight_state or ''
        msg.elapsed = 0.0 if m.t_start is None else float(
            (m.t_end or x.now) - m.t_start)
        msg.search_elapsed = 0.0 if m.t_search is None else float(
            (m.t_search_end or x.now) - m.t_search)
        msg.time_budget = float(m.spec.time_budget_s)
        msg.coverage_fraction = float(x.coverage)
        labels = [classify(loc, m.spec.report_min_observations)
                  for loc in x.locations]
        msg.candidates = sum(label in ('SURVIVOR_CONFIRMED',
                                       'UNVERIFIED_CANDIDATE')
                             for label in labels)
        msg.confirmed = labels.count('SURVIVOR_CONFIRMED')
        msg.vlm_available = x.vlm_status == VLM_AVAILABLE
        msg.battery_remaining = -1.0 if x.battery_remaining is None \
            else float(x.battery_remaining)
        msg.avoidance_manoeuvres = int(x.avoidance_manoeuvres)
        msg.end_reason = m.end_reason
        msg.failed_checks = list(m.failed_checks) \
            if m.phase != PHASE_SEARCH else []
        self.pub_status.publish(msg)

    def write_report(self):
        """Publish and store the mission report."""
        report = self.manager.report(self.home, self.origin)
        text = json.dumps(report, indent=1)
        self.pub_report.publish(String(data=text))
        if self.report_path:
            with open(self.report_path, 'w') as handle:
                handle.write(text + '\n')
        self.get_logger().info(
            f'MISSION REPORT: outcome {report["outcome"]}, search ended: '
            f'{report["search_end_reason"] or "-"}, coverage '
            f'{report["coverage_fraction"]:.2f}, entries '
            f'{report["summary"]}'
            + (f', written to {self.report_path}' if self.report_path
               else ''))


def main(args=None):
    """Run the node."""
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
