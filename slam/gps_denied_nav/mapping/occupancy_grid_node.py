from __future__ import annotations

import sys
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
    from sensor_msgs.msg import PointCloud2, PointField
    import struct
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    Node = object
    OccupancyGrid = object
    MapMetaData = object
    Odometry = object
    PointCloud2 = object
    PointField = object

from .pointcloud_processor import PointCloudProcessor


class OccupancyGridNode(Node):
    def __init__(self):
        if not ROS2_AVAILABLE:
            raise RuntimeError("ROS 2 (rclpy) is required to run OccupancyGridNode.")
        super().__init__('occupancy_grid_node')

        # Parameters
        self.declare_parameter('resolution', 0.10)        # 10 cm per cell
        self.declare_parameter('width_m', 40.0)           # 40m x 40m area
        self.declare_parameter('height_m', 40.0)
        self.declare_parameter('origin_x', -20.0)
        self.declare_parameter('origin_y', -20.0)
        self.declare_parameter('min_obstacle_height', 0.2)
        self.declare_parameter('max_obstacle_height', 3.0)

        self.resolution = float(self.get_parameter('resolution').value)
        self.width_m = float(self.get_parameter('width_m').value)
        self.height_m = float(self.get_parameter('height_m').value)
        self.origin_x = float(self.get_parameter('origin_x').value)
        self.origin_y = float(self.get_parameter('origin_y').value)
        self.min_obs_h = float(self.get_parameter('min_obstacle_height').value)
        self.max_obs_h = float(self.get_parameter('max_obstacle_height').value)

        self.grid_w = int(self.width_m / self.resolution)
        self.grid_h = int(self.height_m / self.resolution)

        # Log-odds occupancy grid (-1: unknown, 0: free, 100: occupied)
        # Internal log-odds representation: l = log(p / (1-p))
        self.log_odds = np.zeros((self.grid_h, self.grid_w), dtype=np.float32)
        self.visited = np.zeros((self.grid_h, self.grid_w), dtype=bool)

        self.processor = PointCloudProcessor(voxel_size=self.resolution)
        self.latest_drone_pos = np.zeros(3)

        # Publishers
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 1)
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/costmap/costmap', 1)

        # Subscribers
        self.create_subscription(Odometry, '/odom/filtered', self.odom_callback, 10)
        self.create_subscription(PointCloud2, '/vo/sparse_map', self.pointcloud_callback, 10)

        # Periodic timer for map publishing
        self.create_timer(1.0, self.publish_grid_map)
        self.get_logger().info(f"Occupancy Grid Node Initialized ({self.grid_w}x{self.grid_h} @ {self.resolution}m/cell)")

    def odom_callback(self, msg: Odometry):
        self.latest_drone_pos[0] = msg.pose.pose.position.x
        self.latest_drone_pos[1] = msg.pose.pose.position.y
        self.latest_drone_pos[2] = msg.pose.pose.position.z

    def pointcloud_callback(self, msg: PointCloud2):
        # Extract points from PointCloud2 message
        points = self._parse_pointcloud2(msg)
        if len(points) == 0:
            return

        self.update_with_points(points, self.latest_drone_pos)

    def update_with_points(self, points: np.ndarray, drone_pos: np.ndarray):
        """
        Update grid using 3D points and drone origin.
        """
        # Downsample and filter outliers
        pts_ds = self.processor.voxel_downsample(points)

        # Filter by height window
        obs_mask = (pts_ds[:, 2] >= self.min_obs_h) & (pts_ds[:, 2] <= self.max_obs_h)
        obstacles = pts_ds[obs_mask]

        # Convert obstacle points to grid cells
        gx = ((obstacles[:, 0] - self.origin_x) / self.resolution).astype(int)
        gy = ((obstacles[:, 1] - self.origin_y) / self.resolution).astype(int)

        valid = (gx >= 0) & (gx < self.grid_w) & (gy >= 0) & (gy < self.grid_h)
        gx_valid = gx[valid]
        gy_valid = gy[valid]

        # Drone position in grid
        dx = int((drone_pos[0] - self.origin_x) / self.resolution)
        dy = int((drone_pos[1] - self.origin_y) / self.resolution)

        # Update log odds for occupied cells
        for x, y in zip(gx_valid, gy_valid):
            self.log_odds[y, x] = min(self.log_odds[y, x] + 0.85, 5.0)
            self.visited[y, x] = True

        # Clear drone neighborhood as free space
        if 0 <= dx < self.grid_w and 0 <= dy < self.grid_h:
            r = int(0.5 / self.resolution)
            y_min, y_max = max(0, dy - r), min(self.grid_h, dy + r + 1)
            x_min, x_max = max(0, dx - r), min(self.grid_w, dx + r + 1)
            self.log_odds[y_min:y_max, x_min:x_max] = np.maximum(self.log_odds[y_min:y_max, x_min:x_max] - 0.4, -4.0)
            self.visited[y_min:y_max, x_min:x_max] = True

    def publish_grid_map(self):
        # Convert log odds to probability [0, 100], and unvisited to -1
        prob_grid = np.ones((self.grid_h, self.grid_w), dtype=np.int8) * -1

        # p = 1 / (1 + exp(-l))
        p = 1.0 / (1.0 + np.exp(-self.log_odds))
        int_p = (p * 100).astype(np.int8)

        prob_grid[self.visited] = int_p[self.visited]

        # Construct ROS OccupancyGrid message
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'map'

        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.grid_w
        grid_msg.info.height = self.grid_h
        grid_msg.info.origin.position.x = self.origin_x
        grid_msg.info.origin.position.y = self.origin_y
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0

        grid_msg.data = prob_grid.flatten().tolist()

        self.map_pub.publish(grid_msg)
        self.costmap_pub.publish(grid_msg)

    def _parse_pointcloud2(self, msg: PointCloud2) -> np.ndarray:
        """
        Fast binary parsing of XYZ PointCloud2 message.
        """
        fmt = 'fff'
        point_step = msg.point_step
        n_points = len(msg.data) // point_step
        if n_points == 0:
            return np.empty((0, 3), dtype=np.float32)

        pts = []
        for i in range(n_points):
            offset = i * point_step
            x, y, z = struct.unpack_from(fmt, msg.data, offset)
            if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                pts.append([x, y, z])

        return np.array(pts, dtype=np.float32) if len(pts) > 0 else np.empty((0, 3), dtype=np.float32)


def main(args=None):
    if not ROS2_AVAILABLE:
        print("ROS 2 Python bindings not found.")
        return
    rclpy.init(args=args)
    node = OccupancyGridNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
