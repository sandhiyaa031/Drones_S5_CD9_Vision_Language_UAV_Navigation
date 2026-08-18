#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition
from sensor_msgs.msg import LaserScan
import math

class FlightController(Node):
    def __init__(self):
        super().__init__('flight_controller')
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            px4_qos
        )
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.state = "ARMING"
        self.state_timer = 0.0
        self.last_state = ""
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = -2.0
        self.obstacle_ahead = False

    def position_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z

    def lidar_callback(self, msg):
        ranges = msg.ranges
        if not ranges:
            return
        front_ranges = ranges[-20:] + ranges[:20]
        min_dist = min([r for r in front_ranges if not math.isinf(r) and not math.isnan(r)] + [float('inf')])
        self.obstacle_ahead = min_dist < 1.5

    def control_loop(self):
        offboard_msg = OffboardControlMode()
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(offboard_msg)

        self.state_timer += 0.1

        if self.state != self.last_state:
            self.get_logger().info(f"--- ACTIVE STATE: {self.state} ---")
            self.last_state = self.state

        if self.state == "ARMING":
            if self.state_timer > 1.0:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
                self.state = "TAKEOFF"
                self.state_timer = 0.0

        elif self.state == "TAKEOFF":
            self.target_x = 0.0
            self.target_y = 0.0
            self.target_z = -2.0
            if self.current_z < -1.5: 
                self.state = "EXPLORE"
                self.state_timer = 0.0

        elif self.state == "EXPLORE":
            if self.obstacle_ahead:
                self.state = "AVOID"
                self.state_timer = 0.0
            else:
                self.target_x += 0.05
                self.target_y = 0.0
                self.target_z = -2.0

        elif self.state == "AVOID":
            if not self.obstacle_ahead:
                self.state = "EXPLORE"
                self.state_timer = 0.0
            else:
                self.target_y += 0.05

        if self.state not in ["LAND", "COMPLETED"]:
            setpoint_msg = TrajectorySetpoint()
            setpoint_msg.position = [float(self.target_x), float(self.target_y), float(self.target_z)]
            setpoint_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.trajectory_pub.publish(setpoint_msg)

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FlightController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
