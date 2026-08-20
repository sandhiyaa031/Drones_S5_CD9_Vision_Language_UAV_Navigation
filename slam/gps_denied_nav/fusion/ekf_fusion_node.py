from __future__ import annotations

import sys
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Imu, Range
    from nav_msgs.msg import Odometry, Path
    from geometry_msgs.msg import PoseStamped
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    Node = object
    Imu = object
    Range = object
    Odometry = object
    Path = object
    PoseStamped = object

from .ekf_core import MultiSensorEKF


class EKFFusionNode(Node):
    def __init__(self):
        if not ROS2_AVAILABLE:
            raise RuntimeError("ROS 2 (rclpy) is required to run EKFFusionNode as a ROS node.")
        super().__init__('ekf_fusion_node')

        # Parameters
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('vo_topic', '/vo/odom')
        self.declare_parameter('rangefinder_topic', '/rangefinder/range')
        self.declare_parameter('output_frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('publish_rate_hz', 50.0)

        imu_topic = self.get_parameter('imu_topic').value
        vo_topic = self.get_parameter('vo_topic').value
        range_topic = self.get_parameter('rangefinder_topic').value
        self.output_frame_id = self.get_parameter('output_frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        rate_hz = self.get_parameter('publish_rate_hz').value

        self.ekf = MultiSensorEKF()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.create_subscription(Imu, imu_topic, self.imu_callback, sensor_qos)
        self.create_subscription(Odometry, vo_topic, self.vo_callback, 10)
        self.create_subscription(Range, range_topic, self.range_callback, sensor_qos)

        # Publishers
        self.filtered_odom_pub = self.create_publisher(Odometry, '/odom/filtered', 10)
        self.path_pub = self.create_publisher(Path, '/odom/filtered_path', 10)

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.output_frame_id

        self.last_stamp = None
        self.get_logger().info(f"EKF Fusion Node Initialized. Fusing {imu_topic} + {vo_topic} -> /odom/filtered")

    def imu_callback(self, msg: Imu):
        a_m = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ], dtype=np.float64)

        w_m = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ], dtype=np.float64)

        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos, vel, quat = self.ekf.predict_imu(a_m, w_m, stamp_sec)
        self.last_stamp = msg.header.stamp

        # Publish updated filtered odometry
        self._publish_odometry(msg.header.stamp, pos, vel, quat)

    def vo_callback(self, msg: Odometry):
        pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ], dtype=np.float64)

        quat = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ], dtype=np.float64)

        cov = np.array(msg.pose.covariance).reshape(6, 6) if len(msg.pose.covariance) == 36 else None
        self.ekf.update_visual_odometry(pos, quat, cov)

    def range_callback(self, msg: Range):
        if msg.range >= msg.min_range and msg.range <= msg.max_range:
            self.ekf.update_rangefinder(float(msg.range))

    def _publish_odometry(self, stamp, pos, vel, quat):
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.output_frame_id
        odom_msg.child_frame_id = self.child_frame_id

        odom_msg.pose.pose.position.x = float(pos[0])
        odom_msg.pose.pose.position.y = float(pos[1])
        odom_msg.pose.pose.position.z = float(pos[2])

        odom_msg.pose.pose.orientation.x = float(quat[0])
        odom_msg.pose.pose.orientation.y = float(quat[1])
        odom_msg.pose.pose.orientation.z = float(quat[2])
        odom_msg.pose.pose.orientation.w = float(quat[3])

        # Fill covariance from EKF P matrix
        # Error states 0:3 is position, 6:9 is orientation error
        cov_6x6 = np.zeros((6, 6), dtype=np.float64)
        cov_6x6[0:3, 0:3] = self.ekf.P[0:3, 0:3]
        cov_6x6[3:6, 3:6] = self.ekf.P[6:9, 6:9]
        odom_msg.pose.covariance = cov_6x6.flatten().tolist()

        odom_msg.twist.twist.linear.x = float(vel[0])
        odom_msg.twist.twist.linear.y = float(vel[1])
        odom_msg.twist.twist.linear.z = float(vel[2])

        self.filtered_odom_pub.publish(odom_msg)

        # Update Path
        pose_stamped = PoseStamped()
        pose_stamped.header = odom_msg.header
        pose_stamped.pose = odom_msg.pose.pose
        self.path_msg.poses.append(pose_stamped)
        if len(self.path_msg.poses) > 1500:
            self.path_msg.poses.pop(0)
        self.path_pub.publish(self.path_msg)


def main(args=None):
    if not ROS2_AVAILABLE:
        print("ROS 2 Python bindings not found. Run inside a ROS 2 environment.")
        return
    rclpy.init(args=args)
    node = EKFFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
