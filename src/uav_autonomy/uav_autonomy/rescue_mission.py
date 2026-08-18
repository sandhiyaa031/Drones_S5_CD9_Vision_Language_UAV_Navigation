#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
)

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped


class RescueMission(Node):

    def __init__(self):
        super().__init__('rescue_mission')

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )

        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )

        self.command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            px4_qos
        )

        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        # Future real vision/VLM node publishes here.
        self.survivor_sub = self.create_subscription(
            PoseStamped,
            '/survivor/detection',
            self.survivor_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.control_loop)

        # Mission state
        self.state = 'PREPARE'
        self.state_time = 0.0

        # PX4 local NED position
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        # Sensors
        self.obstacle = False

        # Survivor detection
        self.survivor_found = False
        self.survivor_x = 0.0
        self.survivor_y = 0.0

        # Search pattern
        self.search_index = 0
        self.search_points = [
            (2.0, 0.0, -2.0),
            (2.0, 2.0, -2.0),
            (0.0, 2.0, -2.0),
            (-2.0, 2.0, -2.0),
            (-2.0, 0.0, -2.0),
            (-2.0, -2.0, -2.0),
            (0.0, -2.0, -2.0),
            (2.0, -2.0, -2.0),
        ]

        self.target = (0.0, 0.0, -2.0)

        # Avoidance target
        self.avoid_target = None

        self.get_logger().info(
            '========================================'
        )
        self.get_logger().info(
            ' AUTONOMOUS GPS-DENIED RESCUE MISSION'
        )
        self.get_logger().info(
            ' Search -> Detect -> Approach -> Land'
        )
        self.get_logger().info(
            '========================================'
        )

    # --------------------------------------------------
    # POSITION
    # --------------------------------------------------

    def position_callback(self, msg):
        self.x = float(msg.x)
        self.y = float(msg.y)
        self.z = float(msg.z)

    # --------------------------------------------------
    # LIDAR
    # --------------------------------------------------

    def lidar_callback(self, msg):

        n = len(msg.ranges)

        if n == 0:
            return

        sector = max(1, n // 12)

        front = (
            list(msg.ranges[:sector]) +
            list(msg.ranges[-sector:])
        )

        valid = [
            r for r in front
            if math.isfinite(r) and r > 0.05
        ]

        if valid:
            self.obstacle = min(valid) < 1.5
        else:
            self.obstacle = False

    # --------------------------------------------------
    # SURVIVOR DETECTION
    # --------------------------------------------------

    def survivor_callback(self, msg):

        self.survivor_found = True

        self.survivor_x = float(msg.pose.position.x)
        self.survivor_y = float(msg.pose.position.y)

        self.get_logger().warn(
            f'SURVIVOR DETECTED at '
            f'({self.survivor_x:.2f}, {self.survivor_y:.2f})'
        )

        if self.state in ('SEARCH', 'AVOID'):
            self.change_state('APPROACH')

    # --------------------------------------------------
    # OFFBOARD
    # --------------------------------------------------

    def publish_offboard(self):

        msg = OffboardControlMode()

        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        msg.timestamp = int(
            self.get_clock().now().nanoseconds / 1000
        )

        self.offboard_pub.publish(msg)

    # --------------------------------------------------
    # TRAJECTORY
    # --------------------------------------------------

    def publish_setpoint(self, x, y, z):

        msg = TrajectorySetpoint()

        msg.position = [
            float(x),
            float(y),
            float(z)
        ]

        msg.yaw = float('nan')

        msg.timestamp = int(
            self.get_clock().now().nanoseconds / 1000
        )

        self.trajectory_pub.publish(msg)

    # --------------------------------------------------
    # VEHICLE COMMAND
    # --------------------------------------------------

    def command(self, command, p1=0.0, p2=0.0):

        msg = VehicleCommand()

        msg.command = command
        msg.param1 = float(p1)
        msg.param2 = float(p2)

        msg.target_system = 1
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        msg.timestamp = int(
            self.get_clock().now().nanoseconds / 1000
        )

        self.command_pub.publish(msg)

    # --------------------------------------------------
    # STATE CHANGE
    # --------------------------------------------------

    def change_state(self, state):

        if self.state != state:
            self.get_logger().info(
                f'>>> MISSION STATE: {state}'
            )

        self.state = state
        self.state_time = 0.0

    # --------------------------------------------------
    # MAIN CONTROL LOOP
    # --------------------------------------------------

    def control_loop(self):

        self.state_time += 0.1

        # Keep OFFBOARD heartbeat alive until landing.
        if self.state not in ('LAND', 'COMPLETE'):
            self.publish_offboard()

        # --------------------------------------------------
        # PREPARE
        # --------------------------------------------------

        if self.state == 'PREPARE':

            self.publish_setpoint(
                0.0,
                0.0,
                -2.0
            )

            if self.state_time >= 2.0:

                self.get_logger().info(
                    'Requesting OFFBOARD + ARM'
                )

                self.command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    1.0,
                    6.0
                )

                self.command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    1.0
                )

                self.change_state('TAKEOFF')

        # --------------------------------------------------
        # TAKEOFF
        # --------------------------------------------------

        elif self.state == 'TAKEOFF':

            self.target = (
                0.0,
                0.0,
                -2.0
            )

            self.publish_setpoint(*self.target)

            if self.z < -1.5:

                self.get_logger().info(
                    'Takeoff complete.'
                )

                self.change_state('SEARCH')

        # --------------------------------------------------
        # SEARCH
        # --------------------------------------------------

        elif self.state == 'SEARCH':

            if self.survivor_found:
                self.change_state('APPROACH')
                return

            if self.obstacle:
                self.avoid_target = (
                    self.x,
                    self.y + 1.5,
                    -2.0
                )
                self.change_state('AVOID')
                return

            self.target = self.search_points[
                self.search_index
            ]

            self.publish_setpoint(*self.target)

            dx = self.x - self.target[0]
            dy = self.y - self.target[1]

            distance = math.hypot(dx, dy)

            if distance < 0.5:

                self.search_index += 1

                if self.search_index >= len(self.search_points):

                    self.get_logger().info(
                        'Search pattern completed. '
                        'No survivor detected.'
                    )

                    self.search_index = 0

                self.get_logger().info(
                    f'Moving to search waypoint '
                    f'{self.search_index + 1}/'
                    f'{len(self.search_points)}'
                )

        # --------------------------------------------------
        # AVOID
        # --------------------------------------------------

        elif self.state == 'AVOID':

            if self.avoid_target is None:

                self.avoid_target = (
                    self.x,
                    self.y + 1.5,
                    -2.0
                )

            self.publish_setpoint(
                *self.avoid_target
            )

            if not self.obstacle:

                self.get_logger().info(
                    'Obstacle cleared. Resuming search.'
                )

                self.avoid_target = None
                self.change_state('SEARCH')

        # --------------------------------------------------
        # APPROACH
        # --------------------------------------------------

        elif self.state == 'APPROACH':

            self.target = (
                self.survivor_x,
                self.survivor_y,
                -2.0
            )

            self.publish_setpoint(
                *self.target
            )

            distance = math.hypot(
                self.x - self.survivor_x,
                self.y - self.survivor_y
            )

            if distance < 0.7:

                self.get_logger().info(
                    'Reached survivor location.'
                )

                self.change_state('LAND')

        # --------------------------------------------------
        # LAND
        # --------------------------------------------------

        elif self.state == 'LAND':

            if self.state_time < 1.0:

                self.get_logger().info(
                    'Initiating landing.'
                )

                self.command(
                    VehicleCommand.VEHICLE_CMD_NAV_LAND
                )

            if self.state_time >= 3.0:

                self.get_logger().info(
                    'Landing command sent.'
                )

                self.change_state('COMPLETE')

        # --------------------------------------------------
        # COMPLETE
        # --------------------------------------------------

        elif self.state == 'COMPLETE':

            # Nothing else is commanded here.
            # PX4 handles the landing sequence.
            pass


def main(args=None):

    rclpy.init(args=args)

    node = RescueMission()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
