from __future__ import annotations

import sys
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    import tf2_ros
    from scipy.spatial.transform import Rotation as R_scipy
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    Node = object
    TransformStamped = object
    Odometry = object
    tf2_ros = object
    from scipy.spatial.transform import Rotation as R_scipy


class TFBroadcasterNode(Node):
    def __init__(self):
        if not ROS2_AVAILABLE:
            raise RuntimeError("ROS 2 (rclpy) is required to run TFBroadcasterNode.")
        super().__init__('tf_broadcaster_node')

        self.declare_parameter('odom_topic', '/odom/filtered')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('camera_optical_frame', 'camera_optical_frame')
        self.declare_parameter('imu_frame', 'imu_link')

        odom_topic = self.get_parameter('odom_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.camera_optical_frame = self.camera_optical_frame = self.get_parameter('camera_optical_frame').value
        self.imu_frame = self.get_parameter('imu_frame').value

        # TF Broadcasters
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Broadcast Static Sensor Transforms
        self._publish_static_transforms()

        # Subscribe to Odometry for dynamic odom -> base_link broadcast
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

        # Periodic timer for map -> odom transform (identity if no global loop closure, or updated by SLAM)
        self.create_timer(0.1, self.publish_map_to_odom)

        self.get_logger().info(f"TF2 Broadcaster Initialized. Listening to {odom_topic} for odom -> base_link")

    def _publish_static_transforms(self):
        """
        Publish fixed mechanical offsets between drone base_link and onboard sensors.
        """
        transforms = []
        now = self.get_clock().now().to_msg()

        # 1. base_link -> camera_link (Camera mounted 10cm forward, 5cm down, looking forward)
        t_cam = TransformStamped()
        t_cam.header.stamp = now
        t_cam.header.frame_id = self.base_frame
        t_cam.child_frame_id = self.camera_frame
        t_cam.transform.translation.x = 0.10
        t_cam.transform.translation.y = 0.0
        t_cam.transform.translation.z = -0.05
        t_cam.transform.rotation.x = 0.0
        t_cam.transform.rotation.y = 0.0
        t_cam.transform.rotation.z = 0.0
        t_cam.transform.rotation.w = 1.0
        transforms.append(t_cam)

        # 2. camera_link -> camera_optical_frame
        # Standard ROS camera optical frame rotation: X-right, Y-down, Z-forward
        # Optical transform: Roll -90 deg, Yaw -90 deg
        r_opt = R_scipy.from_euler('xyz', [-90, 0, -90], degrees=True).as_quat()
        t_opt = TransformStamped()
        t_opt.header.stamp = now
        t_opt.header.frame_id = self.camera_frame
        t_opt.child_frame_id = self.camera_optical_frame
        t_opt.transform.translation.x = 0.0
        t_opt.transform.translation.y = 0.0
        t_opt.transform.translation.z = 0.0
        t_opt.transform.rotation.x = float(r_opt[0])
        t_opt.transform.rotation.y = float(r_opt[1])
        t_opt.transform.rotation.z = float(r_opt[2])
        t_opt.transform.rotation.w = float(r_opt[3])
        transforms.append(t_opt)

        # 3. base_link -> imu_link (Center of Mass)
        t_imu = TransformStamped()
        t_imu.header.stamp = now
        t_imu.header.frame_id = self.base_frame
        t_imu.child_frame_id = self.imu_frame
        t_imu.transform.translation.x = 0.0
        t_imu.transform.translation.y = 0.0
        t_imu.transform.translation.z = 0.0
        t_imu.transform.rotation.w = 1.0
        transforms.append(t_imu)

        # 4. base_link -> rangefinder_link (Downward pointing sonar/LiDAR)
        t_rf = TransformStamped()
        t_rf.header.stamp = now
        t_rf.header.frame_id = self.base_frame
        t_rf.child_frame_id = "rangefinder_link"
        t_rf.transform.translation.x = 0.05
        t_rf.transform.translation.y = 0.0
        t_rf.transform.translation.z = -0.08
        r_down = R_scipy.from_euler('xyz', [0, 90, 0], degrees=True).as_quat()
        t_rf.transform.rotation.x = float(r_down[0])
        t_rf.transform.rotation.y = float(r_down[1])
        t_rf.transform.rotation.z = float(r_down[2])
        t_rf.transform.rotation.w = float(r_down[3])
        transforms.append(t_rf)

        self.static_tf_broadcaster.sendTransform(transforms)

    def odom_callback(self, msg: Odometry):
        """
        Broadcast dynamic odom -> base_link transform from filtered odometry.
        """
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    def publish_map_to_odom(self):
        """
        Broadcast map -> odom transform (static identity anchor when operating in local visual odom).
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    if not ROS2_AVAILABLE:
        print("ROS 2 Python bindings not found.")
        return
    rclpy.init(args=args)
    node = TFBroadcasterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
