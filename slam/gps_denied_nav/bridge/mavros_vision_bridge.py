from __future__ import annotations

import sys
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from scipy.spatial.transform import Rotation as R_scipy
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    Node = object
    PoseStamped = object
    PoseWithCovarianceStamped = object
    Odometry = object
    from scipy.spatial.transform import Rotation as R_scipy


class MAVROSVisionBridgeNode(Node):
    def __init__(self):
        if not ROS2_AVAILABLE:
            raise RuntimeError("ROS 2 (rclpy) is required to run MAVROSVisionBridgeNode.")
        super().__init__('mavros_vision_bridge_node')

        # Parameters
        self.declare_parameter('input_odom_topic', '/odom/filtered')
        self.declare_parameter('target_frame', 'NED')  # 'NED' (ArduPilot/PX4 default) or 'ENU'
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('enable_covariance', True)

        input_topic = self.get_parameter('input_odom_topic').value
        self.target_frame = self.get_parameter('target_frame').value.upper()
        self.enable_cov = self.get_parameter('enable_covariance').value

        # Subscribers
        self.create_subscription(Odometry, input_topic, self.odom_callback, 10)

        # Publishers to MAVROS
        self.vision_pose_pub = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 10)
        self.vision_pose_cov_pub = self.create_publisher(PoseWithCovarianceStamped, '/mavros/vision_pose/pose_cov', 10)
        self.vision_odom_pub = self.create_publisher(Odometry, '/mavros/odometry/out', 10)

        # Rotation matrix to convert ENU -> NED:
        # ENU: X-East, Y-North, Z-Up
        # NED: X-North, Y-East, Z-Down
        # Transformation: X_ned = Y_enu, Y_ned = X_enu, Z_ned = -Z_enu
        self.R_enu_to_ned = np.array([
            [0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0]
        ], dtype=np.float64)

        self.get_logger().info(
            f"MAVROS Vision Bridge Initialized. Transforming {input_topic} (ENU) -> /mavros/vision_pose/pose ({self.target_frame})"
        )

    def odom_callback(self, msg: Odometry):
        # Extract ENU Position
        p_enu = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ], dtype=np.float64)

        # Extract ENU Quaternion [qx, qy, qz, qw]
        q_enu = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ], dtype=np.float64)

        if self.target_frame == 'NED':
            # Position transform
            p_out = self.R_enu_to_ned @ p_enu

            # Orientation transform: R_ned = R_enu_to_ned * R_enu * R_enu_to_ned^T
            R_curr = R_scipy.from_quat(q_enu).as_matrix()
            R_ned = self.R_enu_to_ned @ R_curr @ self.R_enu_to_ned.T
            q_out = R_scipy.from_matrix(R_ned).as_quat()
            frame_str = "map_ned"
        else:
            # Keep in ENU (MAVROS handles internal conversion if configured)
            p_out = p_enu
            q_out = q_enu
            frame_str = msg.header.frame_id

        # 1. Publish PoseStamped to /mavros/vision_pose/pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = frame_str

        pose_msg.pose.position.x = float(p_out[0])
        pose_msg.pose.position.y = float(p_out[1])
        pose_msg.pose.position.z = float(p_out[2])

        pose_msg.pose.orientation.x = float(q_out[0])
        pose_msg.pose.orientation.y = float(q_out[1])
        pose_msg.pose.orientation.z = float(q_out[2])
        pose_msg.pose.orientation.w = float(q_out[3])

        self.vision_pose_pub.publish(pose_msg)

        # 2. Publish PoseWithCovarianceStamped
        if self.enable_cov:
            cov_msg = PoseWithCovarianceStamped()
            cov_msg.header = pose_msg.header
            cov_msg.pose.pose = pose_msg.pose
            cov_msg.pose.covariance = msg.pose.covariance
            self.vision_pose_cov_pub.publish(cov_msg)

        # 3. Publish Odometry to /mavros/odometry/out
        out_odom = Odometry()
        out_odom.header = pose_msg.header
        out_odom.child_frame_id = "base_link_frd" if self.target_frame == 'NED' else "base_link"
        out_odom.pose.pose = pose_msg.pose
        out_odom.pose.covariance = msg.pose.covariance
        out_odom.twist = msg.twist
        self.vision_odom_pub.publish(out_odom)


def main(args=None):
    if not ROS2_AVAILABLE:
        print("ROS 2 Python bindings not found.")
        return
    rclpy.init(args=args)
    node = MAVROSVisionBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
