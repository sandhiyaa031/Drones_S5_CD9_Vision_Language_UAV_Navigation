from __future__ import annotations

import sys
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField, Range
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from nav_msgs.msg import Odometry, Path
    from cv_bridge import CvBridge
    import tf2_ros
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    Node = object
    Image = object
    CameraInfo = object
    PointCloud2 = object
    PointField = object
    Range = object
    PoseStamped = object
    TransformStamped = object
    Odometry = object
    Path = object
    CvBridge = object

from .visual_odometer import VisualOdometer
from .feature_tracker import FeatureTracker


class VisualOdometryNode(Node):
    def __init__(self):
        if not ROS2_AVAILABLE:
            raise RuntimeError("ROS 2 (rclpy) is required to run VisualOdometryNode as a ROS node.")
        super().__init__('visual_odometry_node')

        # Declare ROS Parameters
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('rangefinder_topic', '/rangefinder/range')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('publish_pointcloud', True)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'camera_optical_frame')
        self.declare_parameter('max_features', 250)
        self.declare_parameter('detector_type', 'FAST')

        camera_topic = self.get_parameter('camera_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        rangefinder_topic = self.get_parameter('rangefinder_topic').value
        self.publish_debug_img = self.get_parameter('publish_debug_image').value
        self.publish_pcd = self.get_parameter('publish_pointcloud').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        max_feats = self.get_parameter('max_features').value
        det_type = self.get_parameter('detector_type').value

        self.bridge = CvBridge()
        self.tracker = FeatureTracker(max_features=max_feats, detector_type=det_type)

        # Default camera intrinsics (standard pinhole 640x480)
        default_K = np.array([
            [525.0, 0.0, 320.0],
            [0.0, 525.0, 240.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

        self.odometer = VisualOdometer(camera_matrix=default_K, tracker=self.tracker)
        self.latest_altitude = None
        self.has_camera_info = False

        # QoS Profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscribers
        self.create_subscription(Image, camera_topic, self.image_callback, sensor_qos)
        self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 10)
        self.create_subscription(Range, rangefinder_topic, self.rangefinder_callback, sensor_qos)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/vo/odom', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/vo/pose', 10)
        self.path_pub = self.create_publisher(Path, '/vo/path', 10)
        self.debug_img_pub = self.create_publisher(Image, '/vo/debug_image', 10)
        self.pcd_pub = self.create_publisher(PointCloud2, '/vo/sparse_map', 10)

        # Path message accumulator
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id

        self.get_logger().info(f"Visual Odometry Node Initialized on {camera_topic} (Frame: {self.frame_id})")

    def camera_info_callback(self, msg: CameraInfo):
        if not self.has_camera_info:
            K = np.array(msg.k).reshape(3, 3)
            D = np.array(msg.d)
            self.odometer.K = K
            self.odometer.dist_coeffs = D
            self.has_camera_info = True
            self.get_logger().info("Received and configured camera intrinsics.")

    def rangefinder_callback(self, msg: Range):
        if msg.range >= msg.min_range and msg.range <= msg.max_range:
            self.latest_altitude = float(msg.range)

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion error: {e}")
            return

        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        success, pos, quat, cov = self.odometer.process_frame(
            cv_image,
            timestamp=stamp_sec,
            altitude_measurement=self.latest_altitude
        )

        # Publish Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = msg.header.stamp
        odom_msg.header.frame_id = self.frame_id
        odom_msg.child_frame_id = self.child_frame_id

        odom_msg.pose.pose.position.x = float(pos[0, 0])
        odom_msg.pose.pose.position.y = float(pos[1, 0])
        odom_msg.pose.pose.position.z = float(pos[2, 0])

        odom_msg.pose.pose.orientation.x = float(quat[0])
        odom_msg.pose.pose.orientation.y = float(quat[1])
        odom_msg.pose.pose.orientation.z = float(quat[2])
        odom_msg.pose.pose.orientation.w = float(quat[3])

        # Fill 6x6 covariance flattened (36 elements)
        odom_msg.pose.covariance = cov.flatten().tolist()

        odom_msg.twist.twist.linear.x = float(self.odometer.linear_velocity[0, 0])
        odom_msg.twist.twist.linear.y = float(self.odometer.linear_velocity[1, 0])
        odom_msg.twist.twist.linear.z = float(self.odometer.linear_velocity[2, 0])

        self.odom_pub.publish(odom_msg)

        # Publish PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header = odom_msg.header
        pose_msg.pose = odom_msg.pose.pose
        self.pose_pub.publish(pose_msg)

        # Accumulate & Publish Path
        self.path_msg.header.stamp = msg.header.stamp
        self.path_msg.poses.append(pose_msg)
        if len(self.path_msg.poses) > 1000:
            self.path_msg.poses.pop(0)
        self.path_pub.publish(self.path_msg)

        # Publish Debug Image with Feature Tracks
        if self.publish_debug_img:
            debug_frame = self.tracker.draw_tracks(cv_image)
            dbg_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
            dbg_msg.header = msg.header
            self.debug_img_pub.publish(dbg_msg)


def main(args=None):
    if not ROS2_AVAILABLE:
        print("ROS 2 Python bindings not found. Use standalone demo or run inside a ROS 2 environment.")
        return
    rclpy.init(args=args)
    node = VisualOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
