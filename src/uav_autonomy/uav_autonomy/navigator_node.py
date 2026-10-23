#!/usr/bin/env python3
"""ROS 2 node: map-aware navigator (village missions).

Sits between the search planner and the flight controller:

    search_planner --/planning/search/goal_raw--> NAVIGATOR
        --/planning/search/goal--> flight_controller (validates) --> PX4

Subscribes
    /planning/search/goal_raw   uav_interfaces/SearchGoal: where the search
                                planner wants to be (PX4 local frame)
    /map                        nav_msgs/OccupancyGrid from slam_toolbox
    /tf                         map->odom (slam_toolbox), odom->base_level
    /pose                       slam_toolbox's pose (counted: scan matches)
    /uav/scan                   the levelled scans handed to SLAM (is it
                                still being fed?)
    /fmu/out/vehicle_odometry   PX4's own pose
    /uav/flight_status          flight controller state

Publishes
    /planning/search/goal       the next safe set-point on a route planned
                                in the SLAM map (a proposal; the flight
                                controller validates it)
    /planning/navigator/path    nav_msgs/Path, frame map (for display)
    /planning/navigator/status  std_msgs/String, JSON (also tells the search
                                planner when a target cannot be reached)

The far goal is converted from the PX4 frame into the map frame through the
vehicle's two pose estimates; the route is planned in the map, where the
obstacles are; the next set-point is converted back. When the search planner
reports completion, the navigator routes the vehicle home on the map and
only then passes "complete" on.

If SLAM stops producing poses, or its pose and PX4's disagree by more than
the limit, the navigator stops publishing goals and says so. The flight
controller then returns home by its own rule (climb to the return altitude,
fly straight, land).

This node never publishes to /fmu/in/* and reads no simulator ground truth.
"""

import json
import math
import time

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
from px4_msgs.msg import VehicleOdometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from uav_interfaces.msg import FlightStatus, MissionCommand, SearchGoal

from uav_autonomy.navigation import (
    NAV_ARRIVED, NAV_BACKTRACK, NAV_LOCALIZATION_LOST, NAV_UNREACHABLE,
    NavParams,
    Navigator, PlanningGrid, Pose2D, map_to_px4, path_length, px4_to_map)
from uav_autonomy.slam_geometry import (
    enu_flu_rotation, ned_to_enu_xy, yaw_enu)

SEARCH_STATE = 'SEARCH'


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class NavigatorNode(Node):
    """Wrap Navigator with ROS interfaces."""

    def __init__(self):
        """Declare parameters and create publishers and subscribers."""
        super().__init__('navigator')
        defaults = NavParams()
        values = {}
        for name in defaults.__dataclass_fields__:
            self.declare_parameter('nav.' + name, getattr(defaults, name))
            values[name] = self.get_parameter('nav.' + name).value
        self.params = NavParams(**values)
        self.declare_parameter('home_radius_m', 2.0)
        self.declare_parameter('slam_wait_s', 12.0)
        self.declare_parameter('unreachable_report_s', 4.0)
        self.home_radius = float(self.get_parameter('home_radius_m').value)
        self.slam_wait = float(self.get_parameter('slam_wait_s').value)
        self.unreachable_s = float(
            self.get_parameter('unreachable_report_s').value)
        self.nav = Navigator(self.params)
        self.raw = None
        self.raw_rx = 0.0
        self.status = None
        self.px4 = None             # (stamp, Pose2D, down)
        self.map_odom = None
        self.odom_base = None
        self.tf_stamp = None
        self.scan_stamp = None
        self.slam_matches = 0
        self.map = None             # (data, resolution, origin)
        self.map_serial = 0
        self.grid = None
        self.grid_serial = -1
        self.grid_built = 0.0
        self.home = None
        self.search_started = None
        self.lost_reason = ''
        self.last_state = ''
        self.cycles = 0

        self.pub = self.create_publisher(
            SearchGoal, '/planning/search/goal', 10)
        self.pub_path = self.create_publisher(
            Path, '/planning/navigator/path', 2)
        self.pub_status = self.create_publisher(
            String, '/planning/navigator/status', 10)
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=20)
        self.create_subscription(
            SearchGoal, '/planning/search/goal_raw', self.on_raw, 10)
        self.create_subscription(
            OccupancyGrid, '/map', self.on_map, QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(TFMessage, '/tf', self.on_tf, 200)
        self.create_subscription(
            PoseWithCovarianceStamped, '/pose', self.on_slam_pose, 10)
        self.create_subscription(LaserScan, '/uav/scan', self.on_scan, 10)
        self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.on_odometry,
            px4_qos)
        self.create_subscription(
            FlightStatus, '/uav/flight_status', self.on_status, 10)
        self.return_requested = False
        self.create_subscription(
            MissionCommand, '/mission/command', self.on_mission_command, 10)
        self.create_timer(0.1, self.on_timer)
        self.get_logger().info(
            'Navigator ready: routes on the SLAM map; keep-out '
            f'{self.params.inflation_m:.2f} m, look-ahead '
            f'{self.params.lookahead_m:.1f} m')

    def on_raw(self, msg):
        """The search planner's own goal."""
        self.raw = msg
        self.raw_rx = time.monotonic()

    def on_map(self, msg):
        """slam_toolbox's occupancy grid."""
        self.map = (np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width), float(msg.info.resolution),
            (msg.info.origin.position.x, msg.info.origin.position.y))
        self.map_serial += 1

    def on_tf(self, msg):
        """Keep the two transforms that give the pose in the map."""
        for tf in msg.transforms:
            t = tf.transform
            pose = Pose2D(t.translation.x, t.translation.y,
                          yaw_of(t.rotation))
            if tf.header.frame_id == 'map' and tf.child_frame_id == 'odom':
                self.map_odom = pose
                self.tf_stamp = tf.header.stamp.sec \
                    + tf.header.stamp.nanosec * 1e-9
            elif tf.header.frame_id == 'odom' \
                    and tf.child_frame_id == 'base_level':
                self.odom_base = pose

    def on_slam_pose(self, msg):
        """slam_toolbox matched a scan (it does so only while moving)."""
        self.slam_matches += 1

    def on_scan(self, msg):
        """A levelled scan was handed to SLAM."""
        self.scan_stamp = msg.header.stamp.sec \
            + msg.header.stamp.nanosec * 1e-9

    def slam_liveness(self):
        """Stamp of the older of: slam_toolbox's map->odom transform (it
        republishes while alive) and the last scan handed to it. SLAM is
        usable only if both are recent."""
        if self.tf_stamp is None or self.scan_stamp is None:
            return None
        return min(self.tf_stamp, self.scan_stamp)

    def on_odometry(self, msg):
        """PX4's own pose, as a planar ENU pose."""
        if msg.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            return
        position = [float(v) for v in msg.position]
        q = [float(v) for v in msg.q]
        if not all(math.isfinite(v) for v in position + q):
            return
        x, y = ned_to_enu_xy(position)
        self.px4 = (msg.timestamp_sample * 1e-6,
                    Pose2D(x, y, yaw_enu(enu_flu_rotation(q))), position[2])

    def on_mission_command(self, msg):
        """RETURN_HOME from the mission manager (time budget, battery, ...):
        route home on the map exactly as after a finished search, instead of
        leaving it to the controller's straight-line return."""
        if int(msg.command) == MissionCommand.CMD_RETURN_HOME \
                and not self.return_requested:
            self.return_requested = True
            self.get_logger().info(
                f'Mission return requested ({msg.reason}): routing home on '
                'the SLAM map')

    def on_status(self, msg):
        """Flight controller state."""
        self.status = msg

    def pose_in_map(self):
        if self.map_odom is None or self.odom_base is None:
            return None
        x, y = self.map_odom.apply((self.odom_base.x, self.odom_base.y))
        return Pose2D(x, y, self.map_odom.yaw + self.odom_base.yaw)

    def planning_grid(self, points):
        """Rebuild the planning grid at most twice a second."""
        if self.map is None:
            return None
        stale = self.grid is None or (
            self.grid_serial != self.map_serial
            and time.monotonic() - self.grid_built > 0.5) or any(
                not self.grid.inside(self.grid.cell(q)) for q in points)
        if stale:
            data, resolution, origin = self.map
            self.grid = PlanningGrid(data, resolution, origin, points,
                                     self.params)
            self.grid_serial = self.map_serial
            self.grid_built = time.monotonic()
        return self.grid

    def on_timer(self):
        """One navigation cycle."""
        if self.px4 is None or self.status is None or self.raw is None:
            return
        if self.status.state != SEARCH_STATE:
            return
        now, pose_px4, _ = self.px4
        if self.search_started is None:
            self.search_started = now
            self.home = (float(self.status.mission_target.y),
                         float(self.status.mission_target.x))   # east, north
        if time.monotonic() - self.raw_rx > 1.0:
            return                  # the search planner is silent: so are we
        pose_map = self.pose_in_map()
        slam_stamp = self.slam_liveness()
        ok, reason = self.nav.localization_ok(now, slam_stamp, pose_px4,
                                              pose_map)
        waiting = not ok and slam_stamp is None \
            and now - self.search_started < self.slam_wait
        raw = self.raw
        out = SearchGoal()
        out.header = raw.header
        out.state = raw.state
        out.waypoint_index, out.waypoint_count = (raw.waypoint_index,
                                                  raw.waypoint_count)
        out.coverage_fraction = raw.coverage_fraction
        out.inspections_done = raw.inspections_done
        out.inspecting_id = raw.inspecting_id
        out.speed_limit = raw.speed_limit
        returning = raw.complete or self.return_requested
        goal_enu = self.home if returning else (float(raw.target.y),
                                                float(raw.target.x))
        state, unreachable = '', None
        if waiting:
            # just reached the mapping altitude: hold until SLAM has a pose
            target_enu = (pose_px4.x, pose_px4.y)
            state = 'WAITING_FOR_SLAM'
            out.reason = 'holding until SLAM reports a pose'
        elif not ok:
            if self.lost_reason != reason:
                self.lost_reason = reason
                self.get_logger().error(
                    f'LOCALIZATION LOST: {reason}; no goals are published. '
                    'The flight controller will return home by its own '
                    'rule.')
            self.publish_status(now, NAV_LOCALIZATION_LOST, pose_px4,
                                pose_map, goal_enu, None, reason)
            return
        else:
            position_map = (pose_map.x, pose_map.y)
            goal_map = px4_to_map(goal_enu, pose_px4, pose_map)
            grid = self.planning_grid([position_map, goal_map])
            carrot, limit, state = self.nav.update(
                now, grid, position_map, goal_map, clock=time.perf_counter)
            target_enu = map_to_px4(carrot, pose_px4, pose_map)
            if not returning:
                self.nav.record(position_map)       # breadcrumbs of the search
            if limit is not None:
                out.speed_limit = min(float(raw.speed_limit), float(limit))
            out.reason = f'{raw.reason} [{state}]'
            if state == NAV_UNREACHABLE \
                    and self.nav.unreachable_since is not None \
                    and now - self.nav.unreachable_since \
                    >= self.unreachable_s and not returning:
                unreachable = [float(raw.target.x), float(raw.target.y)]
            if returning:
                out.reason = f'returning to base on the map [{state}]'
                home_distance = math.hypot(pose_px4.x - self.home[0],
                                           pose_px4.y - self.home[1])
                stuck = (state == NAV_UNREACHABLE
                         and self.nav.unreachable_since is not None
                         and now - self.nav.unreachable_since
                         >= self.unreachable_s)
                if state != NAV_UNREACHABLE:
                    self.nav.stop_backtrack()
                elif stuck or self.nav.backtrack_idx is not None:
                    # no route home on the map (for example inside a pocket
                    # the inflated map has closed): retrace the flown path
                    crumb = self.nav.backtrack(position_map)
                    if crumb is not None:
                        target_enu = map_to_px4(crumb, pose_px4, pose_map)
                        state = NAV_BACKTRACK
                        stuck = False
                        slow = self.params.near_obstacle_speed_m_s
                        out.speed_limit = min(float(raw.speed_limit), slow)
                        out.reason = ('no route home on the map: retracing '
                                      'the flown path')
                # hand over to the controller's own final approach
                out.complete = home_distance <= self.home_radius \
                    or state == NAV_ARRIVED or stuck
        out.target.x, out.target.y = float(target_enu[1]), float(
            target_enu[0])                 # north, east
        out.target.z = raw.target.z
        self.pub.publish(out)
        if state != self.last_state:
            self.last_state = state
            self.get_logger().info(
                f'{state}: goal (north {goal_enu[1]:.1f}, east '
                f'{goal_enu[0]:.1f}) {self.nav.note}')
        self.cycles += 1
        if self.cycles % 5 == 0:
            self.publish_status(now, state, pose_px4, pose_map, goal_enu,
                                unreachable, out.reason)
            self.publish_path(raw.header)

    def publish_status(self, now, state, pose_px4, pose_map, goal_enu,
                       unreachable, reason):
        """JSON for recording and for the search planner."""
        plan_ms = self.nav.plan_ms[-50:]
        self.pub_status.publish(String(data=json.dumps({
            'stamp': round(now, 3), 'state': state, 'reason': reason,
            'px4_enu': [round(pose_px4.x, 3), round(pose_px4.y, 3),
                        round(pose_px4.yaw, 4)],
            'slam_map': None if pose_map is None else [
                round(pose_map.x, 3), round(pose_map.y, 3),
                round(pose_map.yaw, 4)],
            'goal_enu': [round(goal_enu[0], 2), round(goal_enu[1], 2)],
            'path_m': round(path_length(self.nav.path), 1)
            if self.nav.path else 0.0,
            'path_points': len(self.nav.path),
            'plans': self.nav.plans,
            'replans_blocked': self.nav.replans_blocked,
            'plan_ms_median': round(float(np.median(plan_ms)), 1)
            if plan_ms else None,
            'plan_ms_max': round(float(max(self.nav.plan_ms)), 1)
            if self.nav.plan_ms else None,
            'largest_slam_jump_m': round(self.nav.largest_jump_m, 2),
            'slam_scan_matches': self.slam_matches,
            'unreachable_target': unreachable,
            'map_cells': None if self.map is None else int(
                self.map[0].size)}, separators=(',', ':'))))

    def publish_path(self, header):
        """The planned route, in the map frame."""
        path = Path()
        path.header.stamp = header.stamp
        path.header.frame_id = 'map'
        for x, y in self.nav.path:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.pub_path.publish(path)


def main(args=None):
    """Run the node."""
    rclpy.init(args=args)
    node = NavigatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
