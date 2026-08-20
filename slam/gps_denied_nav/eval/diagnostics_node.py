from __future__ import annotations

import sys
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    Node = object
    Odometry = object
    DiagnosticArray = object
    DiagnosticStatus = object
    KeyValue = object


class DiagnosticsNode(Node):
    def __init__(self):
        if not ROS2_AVAILABLE:
            raise RuntimeError("ROS 2 (rclpy) is required to run DiagnosticsNode.")
        super().__init__('diagnostics_node')

        self.declare_parameter('min_healthy_features', 20)
        self.min_feats = self.get_parameter('min_healthy_features').value

        self.last_vo_stamp = None
        self.last_filtered_stamp = None
        self.vo_pos_var = 0.0
        self.filtered_pos_var = 0.0

        # Subscribers
        self.create_subscription(Odometry, '/vo/odom', self.vo_callback, 10)
        self.create_subscription(Odometry, '/odom/filtered', self.filtered_callback, 10)

        # Publisher
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        # Periodic timer (2 Hz)
        self.create_timer(0.5, self.publish_diagnostics)
        self.get_logger().info("GPS-Denied Navigation Diagnostics Node Initialized.")

    def vo_callback(self, msg: Odometry):
        self.last_vo_stamp = self.get_clock().now()
        if len(msg.pose.covariance) == 36:
            cov = np.array(msg.pose.covariance).reshape(6, 6)
            self.vo_pos_var = float(np.trace(cov[:3, :3]))

    def filtered_callback(self, msg: Odometry):
        self.last_filtered_stamp = self.get_clock().now()
        if len(msg.pose.covariance) == 36:
            cov = np.array(msg.pose.covariance).reshape(6, 6)
            self.filtered_pos_var = float(np.trace(cov[:3, :3]))

    def publish_diagnostics(self):
        now = self.get_clock().now()
        diag_arr = DiagnosticArray()
        diag_arr.header.stamp = now.to_msg()

        # Status 1: Visual Odometry Tracking Health
        vo_status = DiagnosticStatus()
        vo_status.name = "GPS_Denied_Nav: Visual Odometry"
        vo_status.hardware_id = "camera_vo"

        if self.last_vo_stamp is None:
            vo_status.level = DiagnosticStatus.WARN
            vo_status.message = "Waiting for Visual Odometry data"
        else:
            dt = (now - self.last_vo_stamp).nanoseconds * 1e-9
            if dt > 1.0:
                vo_status.level = DiagnosticStatus.ERROR
                vo_status.message = f"VO Timeout: No data for {dt:.1f}s"
            elif self.vo_pos_var > 0.5:
                vo_status.level = DiagnosticStatus.WARN
                vo_status.message = "VO Covariance Elevated (Low Feature Inliers)"
            else:
                vo_status.level = DiagnosticStatus.OK
                vo_status.message = "Visual Odometry Tracking Nominal"

        vo_status.values.append(KeyValue(key="position_covariance_trace", value=f"{self.vo_pos_var:.4f}"))
        diag_arr.status.append(vo_status)

        # Status 2: EKF Fusion Health
        ekf_status = DiagnosticStatus()
        ekf_status.name = "GPS_Denied_Nav: EKF Multi-Sensor Fusion"
        ekf_status.hardware_id = "ekf_fusion"

        if self.last_filtered_stamp is None:
            ekf_status.level = DiagnosticStatus.WARN
            ekf_status.message = "Waiting for EKF updates"
        else:
            dt = (now - self.last_filtered_stamp).nanoseconds * 1e-9
            if dt > 0.5:
                ekf_status.level = DiagnosticStatus.ERROR
                ekf_status.message = f"EKF Timeout: No output for {dt:.1f}s"
            elif self.filtered_pos_var > 0.2:
                ekf_status.level = DiagnosticStatus.WARN
                ekf_status.message = "EKF Variance High"
            else:
                ekf_status.level = DiagnosticStatus.OK
                ekf_status.message = "EKF State Estimation Optimal (50Hz)"

        ekf_status.values.append(KeyValue(key="ekf_position_variance_trace", value=f"{self.filtered_pos_var:.4f}"))
        diag_arr.status.append(ekf_status)

        self.diag_pub.publish(diag_arr)


def main(args=None):
    if not ROS2_AVAILABLE:
        print("ROS 2 Python bindings not found.")
        return
    rclpy.init(args=args)
    node = DiagnosticsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
