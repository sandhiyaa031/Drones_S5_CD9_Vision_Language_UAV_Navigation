#!/usr/bin/env python3
"""ROS 2 node: display helpers for RViz (village missions).

Everything shown is taken from a topic another node really publishes; this
node estimates nothing and commands nothing.

Publishes
    TF odom -> px4_local_ned (static)   so that markers published in PX4's
                                        NED frame (debris tracks, risk,
                                        survivor locations, search path)
                                        appear in the same view as the map
    /viz/flown_path     nav_msgs/Path     where PX4 says the vehicle has
                                          been (frame odom)
    /viz/slam_path      nav_msgs/Path     where SLAM says it has been
                                          (frame map)
    /viz/mission        MarkerArray       home pad, mission phase and
                                          navigator state as text, the
                                          search rectangle, report labels of
                                          located candidates

Note: odom -> px4_local_ned is exact only when the odometry handed to SLAM
is PX4's own (no injected drift), which is the case in every mission; it is
not valid in the drift experiments.
"""

import json
import math

from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster
from uav_interfaces.msg import MissionStatus, SurvivorLocationArray
from visualization_msgs.msg import Marker, MarkerArray


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class MissionViz(Node):
    """Republish real state in forms RViz can draw."""

    def __init__(self):
        """Create publishers and subscribers."""
        super().__init__('mission_viz')
        self.declare_parameter('altitude_m', 6.0)
        self.altitude = float(self.get_parameter('altitude_m').value)
        static = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'px4_local_ned'
        # NED axes expressed in ENU: x_ned = north = +y, y_ned = east = +x,
        # z_ned = down = -z  ->  rotation by 180 deg about (1, 1, 0)/sqrt(2)
        tf.transform.rotation.x = math.sqrt(0.5)
        tf.transform.rotation.y = math.sqrt(0.5)
        static.sendTransform(tf)
        self.static = static
        self.flown = Path()
        self.flown.header.frame_id = 'odom'
        self.slam = Path()
        self.slam.header.frame_id = 'map'
        self.map_odom = None
        self.last_xy = None
        self.mission = None
        self.navigator = {}
        self.search = {}
        self.locations = []
        self.pub_flown = self.create_publisher(Path, '/viz/flown_path', 2)
        self.pub_slam = self.create_publisher(Path, '/viz/slam_path', 2)
        self.pub_markers = self.create_publisher(
            MarkerArray, '/viz/mission', 2)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 200)
        self.create_subscription(
            MissionStatus, '/mission/status', self.on_mission, 10)
        self.create_subscription(
            String, '/planning/navigator/status', self.on_navigator, 10)
        self.create_subscription(
            String, '/planning/search/status', self.on_search, 10)
        self.create_subscription(
            SurvivorLocationArray, '/perception/survivor/locations',
            self.on_locations, 10)
        self.create_timer(0.5, self.on_timer)

    def on_tf(self, msg):
        """Extend the flown and SLAM paths from the real transforms."""
        for tf in msg.transforms:
            t = tf.transform
            if tf.header.frame_id == 'map' and tf.child_frame_id == 'odom':
                self.map_odom = (t.translation.x, t.translation.y,
                                 yaw_of(t.rotation))
            elif tf.header.frame_id == 'odom' \
                    and tf.child_frame_id == 'base_level':
                x, y = t.translation.x, t.translation.y
                if self.last_xy is not None and math.hypot(
                        x - self.last_xy[0], y - self.last_xy[1]) < 0.4:
                    continue
                self.last_xy = (x, y)
                pose = PoseStamped()
                pose.header.frame_id = 'odom'
                pose.header.stamp = tf.header.stamp
                pose.pose.position.x, pose.pose.position.y = x, y
                pose.pose.position.z = self.altitude
                pose.pose.orientation.w = 1.0
                self.flown.poses.append(pose)
                if self.map_odom is not None:
                    mx, my, myaw = self.map_odom
                    c, s = math.cos(myaw), math.sin(myaw)
                    p2 = PoseStamped()
                    p2.header.frame_id = 'map'
                    p2.header.stamp = tf.header.stamp
                    p2.pose.position.x = mx + c * x - s * y
                    p2.pose.position.y = my + s * x + c * y
                    p2.pose.position.z = self.altitude
                    p2.pose.orientation.w = 1.0
                    self.slam.poses.append(p2)
                self.flown.header.stamp = tf.header.stamp
                self.slam.header.stamp = tf.header.stamp

    def on_mission(self, msg):
        """Mission manager status."""
        self.mission = msg

    def on_navigator(self, msg):
        """Navigator status."""
        try:
            self.navigator = json.loads(msg.data)
        except ValueError:
            pass

    def on_search(self, msg):
        """Search planner status."""
        try:
            self.search = json.loads(msg.data)
        except ValueError:
            pass

    def on_locations(self, msg):
        """Located candidates (Phase 8 output)."""
        self.locations = msg.locations

    def on_timer(self):
        """Publish paths and markers."""
        self.pub_flown.publish(self.flown)
        self.pub_slam.publish(self.slam)
        markers = MarkerArray()

        def text(marker_id, frame, x, y, z, content, size=1.2,
                 colour=(1.0, 1.0, 1.0)):
            m = Marker()
            m.header.frame_id = frame
            m.ns, m.id = 'mission_text', marker_id
            m.type, m.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = (
                float(x), float(y), float(z))
            m.pose.orientation.w = 1.0
            m.scale.z = size
            m.color.r, m.color.g, m.color.b = colour
            m.color.a = 1.0
            m.text = content
            markers.markers.append(m)

        # home pad (the local origin is the takeoff point)
        home = Marker()
        home.header.frame_id = 'odom'
        home.ns, home.id = 'home', 0
        home.type, home.action = Marker.CYLINDER, Marker.ADD
        home.pose.orientation.w = 1.0
        home.scale.x = home.scale.y = 4.0
        home.scale.z = 0.1
        home.color.g, home.color.b, home.color.a = 0.8, 1.0, 0.7
        markers.markers.append(home)
        text(1, 'odom', 0.0, 0.0, 3.0, 'BASE', 1.5, (0.4, 0.9, 1.0))
        if self.mission is not None:
            m = self.mission
            lines = [f'MISSION: {m.phase_name}',
                     f'flight: {m.flight_state}',
                     f'navigator: {self.navigator.get("state", "-")}',
                     f'coverage: {100 * m.coverage_fraction:.0f} %',
                     f'candidates: {m.candidates}  confirmed: '
                     f'{m.confirmed}',
                     f'VLM: {"available" if m.vlm_available else "none"}',
                     f'avoidance manoeuvres: {m.avoidance_manoeuvres}']
            if m.end_reason:
                lines.append(f'ended: {m.end_reason}')
            text(2, 'odom', -18.0, -8.0, 12.0, '\n'.join(lines), 1.6)
        area = self.search.get('area')
        if area:
            # the planner's rectangle is in PX4 NED: north_min/max, east_
            box = Marker()
            box.header.frame_id = 'px4_local_ned'
            box.ns, box.id = 'search_area', 0
            box.type, box.action = Marker.LINE_STRIP, Marker.ADD
            box.pose.orientation.w = 1.0
            box.scale.x = 0.15
            box.color.r, box.color.g, box.color.a = 1.0, 0.9, 0.9
            for north, east in ((area[0], area[2]), (area[1], area[2]),
                                (area[1], area[3]), (area[0], area[3]),
                                (area[0], area[2])):
                box.points.append(Point(x=float(north), y=float(east),
                                        z=-self.altitude))
            markers.markers.append(box)
        for i, loc in enumerate(self.locations):
            if loc.vlm_confirmed:
                label, colour = 'SURVIVOR CONFIRMED', (0.2, 1.0, 0.2)
            elif loc.vlm_rejected:
                label, colour = 'rejected by VLM', (0.7, 0.7, 0.7)
            else:
                label, colour = 'candidate (unverified)', (1.0, 0.7, 0.1)
            text(100 + i, 'px4_local_ned', loc.position.x, loc.position.y,
                 loc.position.z - 2.0,
                 f'#{loc.survivor_id} {label}\n+/- '
                 f'{loc.horizontal_sigma:.2f} m', 0.9, colour)
        self.pub_markers.publish(markers)


def main(args=None):
    """Run the node."""
    rclpy.init(args=args)
    node = MissionViz()
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
