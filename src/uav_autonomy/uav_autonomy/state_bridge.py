#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class StateBridge(Node):

    def __init__(self):
        super().__init__('state_bridge')

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            px4_qos
        )

        self.get_logger().info(
            'State Bridge started: PX4 NED -> ROS ENU'
        )

    def odometry_callback(self, msg):

        # PX4 NED -> ROS ENU
        enu_x = float(msg.position[1])
        enu_y = float(msg.position[0])
        enu_z = -float(msg.position[2])

        enu_vx = float(msg.velocity[1])
        enu_vy = float(msg.velocity[0])
        enu_vz = -float(msg.velocity[2])

        odom = Odometry()

        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = enu_x
        odom.pose.pose.position.y = enu_y
        odom.pose.pose.position.z = enu_z

        # Explicit Python floats for ROS message conversion.
        odom.pose.pose.orientation.w = float(msg.q[0])
        odom.pose.pose.orientation.x = float(msg.q[1])
        odom.pose.pose.orientation.y = float(msg.q[2])
        odom.pose.pose.orientation.z = float(msg.q[3])

        odom.twist.twist.linear.x = enu_vx
        odom.twist.twist.linear.y = enu_vy
        odom.twist.twist.linear.z = enu_vz

        self.odom_pub.publish(odom)

        transform = TransformStamped()

        transform.header.stamp = odom.header.stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'

        transform.transform.translation.x = enu_x
        transform.transform.translation.y = enu_y
        transform.transform.translation.z = enu_z

        transform.transform.rotation.w = float(msg.q[0])
        transform.transform.rotation.x = float(msg.q[1])
        transform.transform.rotation.y = float(msg.q[2])
        transform.transform.rotation.z = float(msg.q[3])

        self.tf_broadcaster.sendTransform(transform)


def main(args=None):

    rclpy.init(args=args)

    node = StateBridge()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
