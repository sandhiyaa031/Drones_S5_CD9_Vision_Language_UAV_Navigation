#!/usr/bin/env python3

"""PX4 Offboard takeoff, altitude hold, and landing foundation."""

import math
import time

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


class FlightController(Node):
    """Drive one bounded PX4 SITL flight through the Phase 0 sequence."""

    TICK_SECONDS = 0.05
    SETPOINT_RATE_HZ = 20.0
    PRESTREAM_SECONDS = 1.5
    COMMAND_RETRY_SECONDS = 2.0
    MAX_COMMAND_ATTEMPTS = 4
    TELEMETRY_TIMEOUT_SECONDS = 1.0
    MODE_TIMEOUT_SECONDS = 15.0
    PREFLIGHT_TIMEOUT_SECONDS = 30.0
    ARM_TIMEOUT_SECONDS = 15.0
    TAKEOFF_TIMEOUT_SECONDS = 30.0
    HOLD_SECONDS = 30.0
    LAND_TIMEOUT_SECONDS = 45.0
    TARGET_ALTITUDE_METERS = 3.0
    ALTITUDE_TOLERANCE_METERS = 0.25
    ALTITUDE_HOLD_TOLERANCE_METERS = 0.35
    WAYPOINT_RADIUS_METERS = 0.50
    WAYPOINT_TIMEOUT_SECONDS = 45.0
    WAYPOINT_1_OFFSET_NORTH_METERS = -4.0
    WAYPOINT_1_OFFSET_EAST_METERS = -4.0
    WAYPOINT_2_OFFSET_NORTH_METERS = -4.0
    WAYPOINT_2_OFFSET_EAST_METERS = -5.5

    STATE_WAIT_FOR_DATA = 'WAIT_FOR_DATA'
    STATE_PRESTREAM = 'PRESTREAM'
    STATE_REQUEST_OFFBOARD = 'REQUEST_OFFBOARD'
    STATE_WAIT_OFFBOARD = 'WAIT_OFFBOARD'
    STATE_WAIT_PREFLIGHT = 'WAIT_PREFLIGHT'
    STATE_REQUEST_ARM = 'REQUEST_ARM'
    STATE_WAIT_ARMED = 'WAIT_ARMED'
    STATE_TAKEOFF = 'TAKEOFF'
    STATE_WAYPOINT_1 = 'WAYPOINT_1'
    STATE_WAYPOINT_2 = 'WAYPOINT_2'
    STATE_RETURN_HOME = 'RETURN_HOME'
    STATE_HOLD = 'HOLD'
    STATE_REQUEST_LAND = 'REQUEST_LAND'
    STATE_LANDING = 'LANDING'
    STATE_COMPLETE = 'COMPLETE'
    STATE_FAILSAFE = 'FAILSAFE'

    def __init__(self):
        """Create verified PX4 publishers, subscribers, and mission state."""
        super().__init__('flight_controller')

        self.declare_parameter('telemetry_only', False)
        self.telemetry_only = bool(
            self.get_parameter('telemetry_only').value)
        self.declare_parameter('waypoint_mission', False)
        self.waypoint_mission = bool(
            self.get_parameter('waypoint_mission').value)

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # Live interfaces verified against PX4 SITL and message versions.
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', px4_qos)

        self.position = None
        self.status = None
        self.land_detected = None
        self.last_position_rx = 0.0
        self.last_status_rx = 0.0
        self.last_land_rx = 0.0
        self.position_sample_count = 0
        self.status_sample_count = 0
        self.ack_sample_count = 0
        self.last_telemetry_log = 0.0
        self.latest_ack = None
        self.ack_serial = 0

        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            px4_qos,
        )
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v4',
            self.status_callback,
            px4_qos,
        )
        self.ack_sub = self.create_subscription(
            VehicleCommandAck,
            '/fmu/out/vehicle_command_ack_v1',
            self.ack_callback,
            px4_qos,
        )
        self.land_sub = self.create_subscription(
            VehicleLandDetected,
            '/fmu/out/vehicle_land_detected',
            self.land_callback,
            px4_qos,
        )

        self.state = self.STATE_WAIT_FOR_DATA
        self.state_started = time.monotonic()
        self.prestream_started = None
        self.prestream_count = 0
        self.last_command = None
        self.last_command_sent = 0.0
        self.command_attempts = 0
        self.command_ack_serial_at_send = 0
        self.command_result = None
        self.target_x = None
        self.target_y = None
        self.ground_z = None
        self.target_z = None
        self.home_x = None
        self.home_y = None
        self.waypoint_started = None
        self.hold_started = None
        self.last_altitude_log = 0.0
        self.last_wait_log = 0.0
        self.failure_reason = None

        self.timer = self.create_timer(self.TICK_SECONDS, self.control_loop)
        sequence = (
            'Offboard -> arm -> 3 m -> waypoint 1 -> waypoint 2 -> '
            'return -> land'
            if self.waypoint_mission else
            'Offboard -> arm -> 3 m -> 30 s hold -> land'
        )
        self.get_logger().info(
            'Flight controller ready; waiting for live PX4 telemetry. '
            f'telemetry_only={self.telemetry_only}, '
            f'waypoint_mission={self.waypoint_mission}. Sequence: '
            f'{sequence}.')

    def position_callback(self, msg):
        """Cache the latest PX4 local NED position."""
        self.position = msg
        self.last_position_rx = time.monotonic()
        self.position_sample_count += 1

    def status_callback(self, msg):
        """Cache current PX4 status and preflight state."""
        self.status = msg
        self.last_status_rx = time.monotonic()
        self.status_sample_count += 1

    def ack_callback(self, msg):
        """Record an ACK that matches the currently outstanding command."""
        self.latest_ack = msg
        self.ack_serial += 1
        self.ack_sample_count += 1
        if self.last_command is not None and msg.command == self.last_command:
            self.command_result = msg.result
            result_names = {
                VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED: 'ACCEPTED',
                VehicleCommandAck.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
                    'TEMPORARILY_REJECTED',
                VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED: 'DENIED',
                VehicleCommandAck.VEHICLE_CMD_RESULT_UNSUPPORTED:
                    'UNSUPPORTED',
                VehicleCommandAck.VEHICLE_CMD_RESULT_FAILED: 'FAILED',
                VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS:
                    'IN_PROGRESS',
            }
            self.get_logger().info(
                f'Command {msg.command} ACK: '
                f'{result_names.get(msg.result, str(msg.result))}')

    def land_callback(self, msg):
        """Cache PX4's landing detector output."""
        self.land_detected = msg
        self.last_land_rx = time.monotonic()

    def _set_state(self, state, message=None):
        if self.state != state:
            self.state = state
            self.state_started = time.monotonic()
            self.command_attempts = 0
            self.command_result = None
            self.last_command = None
            if message:
                self.get_logger().info(f'{state}: {message}')
            else:
                self.get_logger().info(f'State: {state}')

    def _publish_offboard_stream(self):
        mode = OffboardControlMode()
        mode.position = True
        mode.velocity = False
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        mode.timestamp = self._px4_timestamp()
        self.offboard_pub.publish(mode)

        if self.target_x is None or self.position is None:
            return

        setpoint = TrajectorySetpoint()
        setpoint.position = [
            float(self.target_x), float(self.target_y), float(self.target_z)]
        setpoint.yaw = float('nan')
        setpoint.timestamp = self._px4_timestamp()
        self.setpoint_pub.publish(setpoint)

    def _px4_timestamp(self):
        # PX4 expects microseconds. This steady-clock value is used by the
        # existing SITL bridge, matching the PX4 ROS 2 examples in this tree.
        return int(self.get_clock().now().nanoseconds / 1000)

    def _send_command(self, command, param1=0.0, param2=0.0, param3=0.0):
        now = time.monotonic()
        if self.last_command != command:
            self.last_command = command
            self.command_attempts = 0
            self.command_result = None
        if now - self.last_command_sent < self.COMMAND_RETRY_SECONDS:
            return
        if self.command_attempts >= self.MAX_COMMAND_ATTEMPTS:
            self._fail(f'Command {command} was not accepted after '
                       f'{self.command_attempts} attempts')
            return

        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._px4_timestamp()
        self.command_pub.publish(msg)
        self.command_attempts += 1
        self.last_command_sent = now
        self.command_ack_serial_at_send = self.ack_serial
        self.get_logger().info(
            f'Sent command {command} (attempt {self.command_attempts}/'
            f'{self.MAX_COMMAND_ATTEMPTS})')

    def _command_was_denied(self):
        if self.command_result in (
                VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED,
                VehicleCommandAck.VEHICLE_CMD_RESULT_UNSUPPORTED,
                VehicleCommandAck.VEHICLE_CMD_RESULT_FAILED,
                VehicleCommandAck.VEHICLE_CMD_RESULT_CANCELLED):
            self._fail(f'PX4 rejected command {self.last_command}; '
                       f'ACK result={self.command_result}')
            return True
        return False

    def _fail(self, reason):
        if self.state == self.STATE_FAILSAFE:
            return
        self.failure_reason = reason
        self.get_logger().error(reason)
        armed = (self.status is not None and
                 self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        if armed:
            self._set_state(self.STATE_FAILSAFE,
                            'requesting PX4 AUTO LAND and holding last target')
            self.last_command = None
        else:
            self._set_state(self.STATE_FAILSAFE, 'vehicle remains disarmed')

    def _altitude_above_start(self):
        if self.position is None or self.ground_z is None:
            return float('nan')
        # PX4 local coordinates are NED: a more negative z is higher.
        return self.ground_z - float(self.position.z)

    def _telemetry_fresh(self):
        now = time.monotonic()
        return (
            self.position is not None
            and self.status is not None
            and now - self.last_position_rx <= self.TELEMETRY_TIMEOUT_SECONDS
            and now - self.last_status_rx <= self.TELEMETRY_TIMEOUT_SECONDS
            and self.position.xy_valid
            and self.position.z_valid
        )

    def _check_global_timeouts(self):
        if self.state in (self.STATE_WAIT_FOR_DATA, self.STATE_COMPLETE,
                          self.STATE_FAILSAFE):
            return
        if not self._telemetry_fresh() and self.state != self.STATE_LANDING:
            self._fail('PX4 position/status telemetry became stale or invalid')

    def control_loop(self):
        """Advance guarded PX4 mode, arm, takeoff, hold, and landing states."""
        if self.telemetry_only:
            now = time.monotonic()
            if now - self.last_telemetry_log >= 2.0:
                position_fresh = (
                    self.position is not None
                    and now - self.last_position_rx
                    <= self.TELEMETRY_TIMEOUT_SECONDS
                )
                status_fresh = (
                    self.status is not None
                    and now - self.last_status_rx
                    <= self.TELEMETRY_TIMEOUT_SECONDS
                )
                telemetry_ready = self._telemetry_fresh()
                self.get_logger().info(
                    'TELEMETRY_ONLY '
                    f'ready={telemetry_ready} '
                    f'position_samples={self.position_sample_count} '
                    f'position_fresh={position_fresh} '
                    f'status_samples={self.status_sample_count} '
                    f'status_fresh={status_fresh} '
                    f'ack_samples={self.ack_sample_count}'
                )
                self.last_telemetry_log = now
            return

        self._check_global_timeouts()

        # Maintain Offboard proof and the current safe position target until
        # PX4 has acknowledged the explicit AUTO LAND request.
        if self.state not in (self.STATE_LANDING, self.STATE_COMPLETE):
            self._publish_offboard_stream()

        now = time.monotonic()

        if self.state == self.STATE_WAIT_FOR_DATA:
            if self._telemetry_fresh():
                self.target_x = float(self.position.x)
                self.target_y = float(self.position.y)
                self.home_x = self.target_x
                self.home_y = self.target_y
                self.ground_z = float(self.position.z)
                self.target_z = self.ground_z
                self.prestream_started = now
                self.prestream_count = 0
                self._set_state(
                    self.STATE_PRESTREAM,
                    'streaming current-position hold setpoints '
                    'before mode change')
            elif now - self.last_wait_log >= 5.0:
                pos_valid = (
                    self.position is not None
                    and self.position.xy_valid
                    and self.position.z_valid
                )
                self.get_logger().warning(
                    'Waiting for PX4 telemetry: '
                    f'position_received={self.position is not None}, '
                    f'status_received={self.status is not None}, '
                    f'position_valid={pos_valid}, '
                    f'position_age={now - self.last_position_rx:.1f}s, '
                    f'status_age={now - self.last_status_rx:.1f}s')
                self.last_wait_log = now
            return

        if self.state == self.STATE_PRESTREAM:
            self.prestream_count += 1
            if (self.prestream_started is not None
                    and now - self.prestream_started >= self.PRESTREAM_SECONDS
                    and self.prestream_count >= int(
                        self.SETPOINT_RATE_HZ * self.PRESTREAM_SECONDS)):
                self._set_state(self.STATE_REQUEST_OFFBOARD,
                                'prestream complete; requesting Offboard')
            return

        if self.state == self.STATE_REQUEST_OFFBOARD:
            if (self.status.nav_state
                    == VehicleStatus.NAVIGATION_STATE_OFFBOARD):
                self._set_state(self.STATE_WAIT_PREFLIGHT,
                                'PX4 reports Offboard; waiting for '
                                'preflight pass')
                return
            if now - self.state_started > self.MODE_TIMEOUT_SECONDS:
                self._fail('Timed out waiting for PX4 Offboard state')
                return
            self._send_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                param1=1.0,
                param2=6.0,
            )
            self._command_was_denied()
            return

        if self.state == self.STATE_WAIT_PREFLIGHT:
            if (self.status.nav_state
                    != VehicleStatus.NAVIGATION_STATE_OFFBOARD):
                self._set_state(self.STATE_REQUEST_OFFBOARD,
                                'Offboard state was lost; requesting again')
                return
            if self.status.pre_flight_checks_pass:
                self._set_state(self.STATE_REQUEST_ARM,
                                'preflight checks pass; requesting arm')
                return
            if now - self.state_started > self.PREFLIGHT_TIMEOUT_SECONDS:
                self._fail(
                    'Preflight did not pass; see PX4 health/arming output '
                    '(GCS link must be present when NAV_DLL_ACT > 0)')
            return

        if self.state == self.STATE_REQUEST_ARM:
            if not self.status.pre_flight_checks_pass:
                self._fail('Preflight check cleared while preparing to arm')
                return
            if (self.status.nav_state
                    != VehicleStatus.NAVIGATION_STATE_OFFBOARD):
                self._set_state(self.STATE_REQUEST_OFFBOARD,
                                'Offboard state was lost before arming')
                return
            if self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self._set_state(self.STATE_TAKEOFF,
                                'PX4 status confirms armed and Offboard')
                return
            self._send_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=1.0,
            )
            self._command_was_denied()
            if now - self.state_started > self.ARM_TIMEOUT_SECONDS:
                self._fail('Timed out waiting for PX4 armed state')
            return

        if self.state == self.STATE_TAKEOFF:
            if self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                self._fail('Takeoff guard: PX4 reports disarmed')
                return
            if (self.status.nav_state
                    != VehicleStatus.NAVIGATION_STATE_OFFBOARD):
                self._fail('Takeoff guard: PX4 is no longer in Offboard')
                return
            if self.target_z == self.ground_z:
                self.target_z = self.ground_z - self.TARGET_ALTITUDE_METERS
                self.get_logger().info(
                    f'Takeoff target: NED z={self.target_z:.2f} m '
                    f'(3.00 m above start)')
            altitude = self._altitude_above_start()
            if math.isfinite(altitude):
                self.get_logger().info(
                    f'Takeoff altitude: {altitude:.2f} m',
                    throttle_duration_sec=1.0)
            if (math.isfinite(altitude)
                    and abs(altitude - self.TARGET_ALTITUDE_METERS)
                    <= self.ALTITUDE_TOLERANCE_METERS):
                if self.waypoint_mission:
                    self.target_x = (
                        self.home_x + self.WAYPOINT_1_OFFSET_NORTH_METERS)
                    self.target_y = (
                        self.home_y + self.WAYPOINT_1_OFFSET_EAST_METERS)
                    self.waypoint_started = now
                    self._set_state(
                        self.STATE_WAYPOINT_1,
                        f'takeoff verified at {altitude:.2f} m; '
                        f'proceeding to waypoint 1 '
                        f'({self.target_x:.1f}, {self.target_y:.1f}) NED')
                    return
                self.hold_started = now
                self._set_state(self.STATE_HOLD,
                                f'altitude verified at {altitude:.2f} m; '
                                'starting 30 s hold')
                return
            if now - self.state_started > self.TAKEOFF_TIMEOUT_SECONDS:
                self._fail(f'Takeoff timeout at {altitude:.2f} m above start')
            return

        if self.state in (
                self.STATE_WAYPOINT_1,
                self.STATE_WAYPOINT_2,
                self.STATE_RETURN_HOME):
            if self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                self._fail('PX4 disarmed during waypoint flight')
                return
            if (self.status.nav_state
                    != VehicleStatus.NAVIGATION_STATE_OFFBOARD):
                self._fail('PX4 left Offboard during waypoint flight')
                return
            north_error = float(self.position.x) - self.target_x
            east_error = float(self.position.y) - self.target_y
            distance = math.hypot(north_error, east_error)
            altitude = self._altitude_above_start()
            self.get_logger().info(
                f'{self.state}: horizontal error {distance:.2f} m; '
                f'altitude {altitude:.2f} m',
                throttle_duration_sec=1.0)
            if now - self.waypoint_started > self.WAYPOINT_TIMEOUT_SECONDS:
                self._fail(
                    f'{self.state} timed out at {distance:.2f} m from target')
                return
            if (distance > self.WAYPOINT_RADIUS_METERS
                    or not math.isfinite(altitude)
                    or abs(altitude - self.TARGET_ALTITUDE_METERS)
                    > self.ALTITUDE_HOLD_TOLERANCE_METERS):
                return

            if self.state == self.STATE_WAYPOINT_1:
                self.target_x = (
                    self.home_x + self.WAYPOINT_2_OFFSET_NORTH_METERS)
                self.target_y = (
                    self.home_y + self.WAYPOINT_2_OFFSET_EAST_METERS)
                self.waypoint_started = now
                self._set_state(
                    self.STATE_WAYPOINT_2,
                    f'waypoint 1 reached; proceeding to waypoint 2 '
                    f'({self.target_x:.1f}, {self.target_y:.1f}) NED')
            elif self.state == self.STATE_WAYPOINT_2:
                self.target_x = self.home_x
                self.target_y = self.home_y
                self.waypoint_started = now
                self._set_state(
                    self.STATE_RETURN_HOME,
                    'waypoint 2 reached; returning to takeoff point '
                    f'({self.target_x:.1f}, {self.target_y:.1f}) NED')
            else:
                self._set_state(
                    self.STATE_REQUEST_LAND,
                    f'home reached at altitude {altitude:.2f} m; '
                    'requesting AUTO LAND')
            return

        if self.state == self.STATE_HOLD:
            if self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                self._fail('PX4 disarmed during altitude hold')
                return
            if (self.status.nav_state
                    != VehicleStatus.NAVIGATION_STATE_OFFBOARD):
                self._fail('PX4 left Offboard during altitude hold')
                return
            altitude = self._altitude_above_start()
            self.get_logger().info(
                f'Hold altitude: {altitude:.2f} m; '
                f'elapsed '
                f'{now - self.hold_started:.1f}/{self.HOLD_SECONDS:.0f} s',
                throttle_duration_sec=1.0)
            if (not math.isfinite(altitude)
                    or abs(altitude - self.TARGET_ALTITUDE_METERS)
                    > self.ALTITUDE_HOLD_TOLERANCE_METERS):
                self._fail(f'Altitude hold left tolerance: {altitude:.2f} m')
                return
            if now - self.hold_started >= self.HOLD_SECONDS:
                self._set_state(self.STATE_REQUEST_LAND,
                                '30 s altitude hold verified; '
                                'requesting AUTO LAND')
            return

        if self.state == self.STATE_REQUEST_LAND:
            if self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                self._fail('PX4 disarmed before the explicit landing request')
                return
            self._send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self._command_was_denied()
            if (self.command_result
                    == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED):
                self._set_state(self.STATE_LANDING,
                                'PX4 accepted NAV_LAND; '
                                'verifying landed state')
            return

        if self.state == self.STATE_LANDING:
            if (self.land_detected is not None
                    and now - self.last_land_rx
                    <= self.TELEMETRY_TIMEOUT_SECONDS
                    and self.land_detected.landed
                    and self.status.arming_state
                    == VehicleStatus.ARMING_STATE_DISARMED):
                self._set_state(self.STATE_COMPLETE,
                                'PX4 reports landed and disarmed')
                self.get_logger().info(
                    f'FLIGHT COMPLETE; final relative altitude '
                    f'{self._altitude_above_start():.2f} m')
                return
            if now - self.state_started > self.LAND_TIMEOUT_SECONDS:
                self._fail('Timed out waiting for landed and disarmed status')
            return

        if self.state == self.STATE_FAILSAFE:
            armed = (self.status is not None and
                     self.status.arming_state
                     == VehicleStatus.ARMING_STATE_ARMED)
            if armed:
                self._send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                if (self.land_detected is not None
                        and self.land_detected.landed):
                    self.get_logger().error(
                        'FAILSAFE: land detector reports landed; '
                        'controller will not issue a disarm command')
            return


def main(args=None):
    """Run the Phase 0 flight controller node."""
    rclpy.init(args=args)
    node = FlightController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warning(
            'Interrupted. PX4 remains responsible for its configured '
            'failsafe.')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
