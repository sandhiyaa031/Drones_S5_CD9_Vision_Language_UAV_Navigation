#!/usr/bin/env python3
"""ROS 2 node: search planner (Phase 9).

Subscribes (the vehicle's own data only)
    /uav/flight_status               uav_interfaces/FlightStatus
    /fmu/out/vehicle_odometry        px4_msgs/VehicleOdometry
    /camera/camera_info              sensor_msgs/CameraInfo
    /perception/survivor/locations   uav_interfaces/SurvivorLocationArray

Publishes
    /planning/search/goal     uav_interfaces/SearchGoal, 10 Hz: a proposal;
                              flight_controller validates it and is the only
                              node that commands PX4
    /planning/search/status   std_msgs/String, JSON
    /planning/search/markers  visualization_msgs/MarkerArray (path)

The search starts when flight_controller reports the SEARCH state. The
search area is given relative to the takeoff point. This node never
publishes to /fmu/in/* and never reads Gazebo ground truth. Method:
search_planning.py.
"""

import json
import math

from geometry_msgs.msg import Point
from px4_msgs.msg import VehicleOdometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
    qos_profile_sensor_data)
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from uav_interfaces.msg import (
    FlightStatus, SearchGoal, SurvivorLocationArray)
from visualization_msgs.msg import Marker, MarkerArray

from uav_autonomy.search_planning import (
    Camera,
    PlannerParams,
    STATE_IDLE,
    STATE_NAMES,
    SearchArea,
    SearchPlanner,
    path_length,
    yaw_of,
)

LOCAL_FRAME = 'px4_local_ned'
SEARCH_STATE = 'SEARCH'


class SearchPlannerNode(Node):
    """Wrap SearchPlanner with ROS interfaces."""

    def __init__(self):
        """Declare parameters and create publishers and subscribers."""
        super().__init__('search_planner')
        defaults = PlannerParams()
        values = {}
        for name in defaults.__dataclass_fields__:
            self.declare_parameter('plan.' + name, getattr(defaults, name))
            values[name] = self.get_parameter('plan.' + name).value
        self.params = PlannerParams(**values)
        # search rectangle relative to the takeoff point [m]
        for name, value in (('area_north_min', -4.0), ('area_north_max', 4.0),
                            ('area_east_min', -4.0), ('area_east_max', 4.0)):
            self.declare_parameter(name, value)
        # optional: begin at the lane end nearest to this point (relative to
        # the takeoff point) instead of the one nearest to the vehicle
        self.declare_parameter('start_near_north', float('nan'))
        self.declare_parameter('start_near_east', float('nan'))
        self.start_near = (
            float(self.get_parameter('start_near_north').value),
            float(self.get_parameter('start_near_east').value))
        self.relative_area = [float(self.get_parameter(n).value) for n in (
            'area_north_min', 'area_north_max', 'area_east_min',
            'area_east_max')]

        self.cam = None
        self.planner = None
        self.status = None
        self.pose = None            # (stamp, position, q, speed)
        self.locations = []
        self.home = None
        self.last_goal = None
        self.cycles = 0

        self.pub = self.create_publisher(
            SearchGoal, '/planning/search/goal', 10)
        self.pub_status = self.create_publisher(
            String, '/planning/search/status', 10)
        self.pub_markers = self.create_publisher(
            MarkerArray, '/planning/search/markers', 2)

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=20)
        self.create_subscription(
            CameraInfo, '/camera/camera_info', self.on_info,
            qos_profile_sensor_data)
        self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.on_odometry,
            px4_qos)
        self.create_subscription(
            FlightStatus, '/uav/flight_status', self.on_status, 10)
        self.create_subscription(
            SurvivorLocationArray, '/perception/survivor/locations',
            self.on_locations, 10)
        # the map-aware navigator (village missions) reports targets it
        # cannot route to; they are skipped rather than waited for
        self.create_subscription(
            String, '/planning/navigator/status', self.on_navigator, 10)
        self.vlm_busy = False
        self.create_subscription(
            String, '/perception/survivor/status', self.on_survivor_status, 10)
        self.create_timer(0.1, self.on_timer)
        self.get_logger().info(
            'Search planner ready; area relative to takeoff: north '
            f'[{self.relative_area[0]:.1f}, {self.relative_area[1]:.1f}] '
            f'east [{self.relative_area[2]:.1f}, '
            f'{self.relative_area[3]:.1f}] m')

    def on_info(self, msg):
        """Store the camera intrinsics."""
        if self.cam is None:
            self.cam = Camera(fx=msg.k[0], fy=msg.k[4], width=msg.width,
                              height=msg.height)

    def on_odometry(self, msg):
        """Store the pose; add the camera footprint to the coverage map."""
        if msg.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            return
        position = [float(v) for v in msg.position]
        q = [float(v) for v in msg.q]
        if not all(math.isfinite(v) for v in position + q):
            return
        speed = math.hypot(float(msg.velocity[0]), float(msg.velocity[1]))
        stamp = msg.timestamp_sample * 1e-6
        previous = self.pose
        self.pose = (stamp, position, q, speed)
        if (self.planner is not None and self.status is not None
                and self.status.state == SEARCH_STATE
                and self.status.ground_valid
                and (previous is None or stamp - self.last_observed >= 0.1)):
            self.last_observed = stamp
            self.planner.observe(position, q, float(self.status.ground_z))

    last_observed = 0.0

    def on_status(self, msg):
        """Store the flight controller's state."""
        self.status = msg

    def on_locations(self, msg):
        """Store the located survivor candidates (Phase 8)."""
        self.locations = [{
            'id': int(m.survivor_id),
            'ned': [m.position.x, m.position.y, m.position.z],
            'height': float(m.height_above_ground),
            'state': int(m.localization_state),
            'obs': int(m.observations),
            'confirmed': bool(m.vlm_confirmed),
            'rejected': bool(m.vlm_rejected)} for m in msg.locations]

    def on_survivor_status(self, msg):
        """Is a VLM request pending? (the inspection waits for the answer)"""
        try:
            self.vlm_busy = bool(json.loads(msg.data).get('vlm_busy'))
        except ValueError:
            pass

    def on_navigator(self, msg):
        """Skip a target the navigator reports as unreachable."""
        if self.planner is None or self.pose is None:
            return
        try:
            target = json.loads(msg.data).get('unreachable_target')
        except ValueError:
            return
        if target and self.planner.skip_current(
                self.pose[0], target, 'no route in the map'):
            self.get_logger().warning(
                f'Target ({target[0]:.1f}, {target[1]:.1f}) skipped: the '
                'navigator found no route to it')

    def on_timer(self):
        """Plan one cycle and publish the goal."""
        if self.cam is None or self.pose is None or self.status is None:
            return
        stamp, position, q, speed = self.pose
        searching = self.status.state == SEARCH_STATE
        if self.planner is None:
            if not (searching and self.status.ground_valid):
                return
            # the controller holds the takeoff point when SEARCH begins
            self.home = (float(self.status.mission_target.x),
                         float(self.status.mission_target.y))
            area = SearchArea(self.home[0] + self.relative_area[0],
                              self.home[0] + self.relative_area[1],
                              self.home[1] + self.relative_area[2],
                              self.home[1] + self.relative_area[3])
            self.planner = SearchPlanner(area, self.cam, self.params)
            height = float(self.status.ground_z) - float(
                self.status.mission_target.z)
            reference = position[:2]
            if not math.isnan(self.start_near[0]):
                reference = (self.home[0] + self.start_near[0],
                             self.home[1] + self.start_near[1])
            waypoints = self.planner.start(stamp, reference, yaw_of(q),
                                           height)
            self.get_logger().info(
                f'Search started at {height:.1f} m: {len(waypoints)} '
                f'waypoints, path {path_length(waypoints):.1f} m, heading '
                f'{math.degrees(yaw_of(q)):.0f} deg; ' + ' -> '.join(
                    f'({n:.1f}, {e:.1f})' for n, e in waypoints))
        self.planner.vlm_busy = self.vlm_busy
        goal = self.planner.update(stamp, position[:2], speed,
                                   self.locations)
        if self.planner.state == STATE_IDLE:
            return
        if self.last_goal is None or (goal.state, goal.reason) != (
                self.last_goal.state, self.last_goal.reason):
            self.get_logger().info(
                f'{STATE_NAMES[goal.state]}: {goal.reason} -> '
                f'({goal.target_ne[0]:.1f}, {goal.target_ne[1]:.1f})')
        self.last_goal = goal
        msg = SearchGoal()
        sec = int(math.floor(stamp))
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = int((stamp - sec) * 1e9)
        msg.header.frame_id = LOCAL_FRAME
        msg.state = goal.state
        msg.target.x, msg.target.y = (float(goal.target_ne[0]),
                                      float(goal.target_ne[1]))
        msg.target.z = float(position[2])
        msg.speed_limit = float(goal.speed_limit)
        msg.complete = bool(goal.complete)
        msg.waypoint_index = int(goal.waypoint_index)
        msg.waypoint_count = int(goal.waypoint_count)
        msg.coverage_fraction = float(self.planner.grid.fraction())
        msg.inspections_done = len(self.planner.inspected)
        msg.inspecting_id = int(goal.inspecting_id)
        msg.reason = goal.reason
        self.pub.publish(msg)
        self.cycles += 1
        if self.cycles % 5 == 0:
            self.publish_status(stamp, position, speed, goal)
        if self.cycles % 20 == 1:
            self.publish_markers()

    def publish_status(self, stamp, position, speed, goal):
        """JSON summary for logging."""
        planner = self.planner
        self.pub_status.publish(String(data=json.dumps({
            'stamp': round(stamp, 3),
            'state': STATE_NAMES[goal.state],
            'flight_state': self.status.state,
            'position': [round(v, 3) for v in position],
            'speed': round(speed, 3),
            'target': [round(v, 3) for v in goal.target_ne],
            'speed_limit': goal.speed_limit,
            'waypoint': [goal.waypoint_index, goal.waypoint_count],
            'coverage': round(planner.grid.fraction(), 4),
            'coverage_oblique': round(planner.grid.oblique_fraction(), 4),
            'inspecting': goal.inspecting_id,
            'inspections': [{
                'id': i.entity_id,
                'target': [round(v, 2) for v in i.target_ne],
                'viewpoints': [[round(v, 2) for v in p]
                               for p in i.viewpoints],
                'started': round(i.started, 2),
                'ended': None if i.ended is None else round(i.ended, 2),
                'result': i.result} for i in planner.inspections],
            'skipped': planner.skipped_candidates,
            'skipped_targets': [[round(t[0], 2), round(t[1], 2), r]
                                for t, r in planner.skipped_targets],
            'waypoints': [[round(v, 2) for v in w]
                          for w in planner.waypoints],
            'area': [planner.area.north_min, planner.area.north_max,
                     planner.area.east_min, planner.area.east_max],
            'home': list(self.home), 'reason': goal.reason},
            separators=(',', ':'))))

    def publish_markers(self):
        """The planned coverage path as a line strip."""
        marker = Marker()
        marker.header.frame_id = LOCAL_FRAME
        marker.ns = 'search_path'
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.08
        marker.color.a, marker.color.g, marker.color.b = 0.9, 0.8, 1.0
        marker.pose.orientation.w = 1.0
        z = float(self.status.mission_target.z)
        for north, east in self.planner.waypoints:
            marker.points.append(Point(x=float(north), y=float(east), z=z))
        self.pub_markers.publish(MarkerArray(markers=[marker]))


def main(args=None):
    """Run the node."""
    rclpy.init(args=args)
    node = SearchPlannerNode()
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
