#!/usr/bin/env python3
"""ROS 2 node: debris avoidance planner.

Subscribes
    /perception/debris/risk          uav_interfaces/DebrisRiskArray
    /perception/debris/predictions   uav_interfaces/DebrisPredictionArray
    /perception/debris/tracks        uav_interfaces/DebrisTrackArray
    /fmu/out/vehicle_odometry        px4_msgs/VehicleOdometry
    /uav/flight_status               uav_interfaces/FlightStatus

Publishes
    /perception/debris/avoidance_command   uav_interfaces/AvoidanceCommand
    /perception/debris/avoidance_markers   visualization_msgs/MarkerArray
    /perception/debris/avoidance_debug     std_msgs/String (JSON per cycle)

The command is a proposal to flight_controller, which validates it and is
the only node that publishes to /fmu/in/*. This node never commands PX4 and
does not read Gazebo ground truth.

Time: debris states carry the depth-frame time, the UAV state carries PX4
timestamp_sample; both are the same simulation time base, and the planner
advances each debris path by the difference. Perception age is measured in
that time base. Only the odometry liveness check uses the arrival clock.
"""

import json
import time

from geometry_msgs.msg import Point
import numpy as np
from px4_msgs.msg import VehicleLocalPosition, VehicleOdometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)
from std_msgs.msg import String
from uav_interfaces.msg import (
    AvoidanceCommand, DebrisPredictionArray, DebrisRiskArray,
    DebrisTrackArray, FlightStatus)
from visualization_msgs.msg import Marker, MarkerArray

from uav_autonomy.debris_avoidance import (
    AvoidanceParams,
    AvoidancePlanner,
    HEALTH_NAMES,
    MODE_NAMES,
    Threat,
)
from uav_autonomy.debris_risk import (
    Knot, POTENTIAL_THREAT, STATE_NAMES, ballistic_knots, radii)

SCALAR_TYPES = (bool, int, float)


def stamp_s(stamp) -> float:
    """builtin_interfaces/Time -> float seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


def stamp_key(stamp) -> int:
    """builtin_interfaces/Time -> integer nanoseconds (dictionary key)."""
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class DebrisAvoidanceNode(Node):
    """Wrap the avoidance planner with ROS interfaces."""

    def __init__(self):
        """Declare parameters and create publishers and subscribers."""
        super().__init__('debris_avoidance')
        defaults = AvoidanceParams()
        values = {}
        for name in AvoidanceParams.__dataclass_fields__:
            default = getattr(defaults, name)
            if isinstance(default, SCALAR_TYPES):
                self.declare_parameter(name, default)
                values[name] = self.get_parameter(name).value
            elif (isinstance(default, tuple) and default
                  and isinstance(default[0], (float, str))):
                self.declare_parameter(name, list(default))
                values[name] = tuple(self.get_parameter(name).value)
        # "enabled: false" runs the planner and logs what it would do, but
        # never activates a command (baseline runs).
        self.declare_parameter('enabled', True)
        self.enabled = bool(self.get_parameter('enabled').value)
        self.declare_parameter('rate_hz', 30.0)
        self.params = AvoidanceParams(**values)
        self.planner = AvoidancePlanner(self.params)

        self.odom = None            # (t, p, v)
        self.odom_rx = None
        self.accel = []             # (t, a) recent PX4 acceleration samples
        self.status = None
        self.tracks = {}
        self.predictions = {}
        self.threats = []
        self.perception_stamp = None
        self.rejected_seen = 0
        self.last_log = ''

        self.pub = self.create_publisher(
            AvoidanceCommand, '/perception/debris/avoidance_command', 10)
        self.pub_markers = self.create_publisher(
            MarkerArray, '/perception/debris/avoidance_markers', 5)
        self.pub_debug = self.create_publisher(
            String, '/perception/debris/avoidance_debug', 50)
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=20)
        self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.on_odometry,
            px4_qos)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.on_local_position, px4_qos)
        self.create_subscription(
            FlightStatus, '/uav/flight_status', self.on_status, 10)
        self.create_subscription(
            DebrisTrackArray, '/perception/debris/tracks', self.on_tracks, 50)
        self.create_subscription(
            DebrisPredictionArray, '/perception/debris/predictions',
            self.on_predictions, 50)
        self.create_subscription(
            DebrisRiskArray, '/perception/debris/risk', self.on_risk, 50)
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value),
                          self.plan)
        self.get_logger().info(
            'Debris avoidance planner ready '
            f'({"ENABLED" if self.enabled else "DISABLED: baseline"}); '
            'commands are proposals to flight_controller')

    # -- inputs ----------------------------------------------------------
    def on_odometry(self, msg):
        """Store the UAV's own position and velocity (local NED)."""
        if (msg.pose_frame != VehicleOdometry.POSE_FRAME_NED
                or msg.velocity_frame != VehicleOdometry.VELOCITY_FRAME_NED):
            return
        self.odom = (msg.timestamp_sample * 1e-6,
                     np.array([float(v) for v in msg.position]),
                     np.array([float(v) for v in msg.velocity]))
        self.odom_rx = time.monotonic()

    def on_local_position(self, msg):
        """Keep PX4's acceleration estimate (local NED) for the model."""
        a = [float(msg.ax), float(msg.ay), float(msg.az)]
        if not np.all(np.isfinite(a)):
            return
        self.accel.append((msg.timestamp_sample * 1e-6, a))
        del self.accel[:-20]

    def measured_accel(self, t, window=0.06):
        """Mean acceleration over the last ``window`` s, or None."""
        recent = [a for ts, a in self.accel if t - window <= ts <= t + 0.02]
        return np.mean(recent, axis=0) if recent else None

    def on_status(self, msg):
        """Store the flight controller's state and mission setpoint."""
        self.status = msg

    def on_tracks(self, msg):
        """Keep recent track frames, keyed by sensor time."""
        self.tracks[stamp_key(msg.header.stamp)] = msg
        for old in sorted(self.tracks)[:-30]:
            del self.tracks[old]

    def on_predictions(self, msg):
        """Keep recent prediction frames, keyed by sensor time."""
        self.predictions[stamp_key(msg.header.stamp)] = msg
        for old in sorted(self.predictions)[:-30]:
            del self.predictions[old]

    def on_risk(self, msg):
        """Build the threat list for this perception frame."""
        key = stamp_key(msg.header.stamp)
        track_msg = self.tracks.get(key)
        pred_msg = self.predictions.get(key)
        by_id = ({tr.id: tr for tr in track_msg.tracks}
                 if track_msg is not None else {})
        preds = ({p.track_id: p for p in pred_msg.predictions}
                 if pred_msg is not None else {})
        threats = []
        for risk in msg.risks:
            tr = by_id.get(risk.track_id)
            if tr is None:
                continue
            p0 = np.array([tr.position.x, tr.position.y, tr.position.z])
            v0 = np.array([tr.velocity.x, tr.velocity.y, tr.velocity.z])
            cov = np.array(tr.covariance).reshape(6, 6)
            pred = preds.get(risk.track_id)
            if pred is not None and pred.states:
                knots = [Knot(0.0, p0, v0, cov[:3, :3])]
                for st in pred.states:
                    knots.append(Knot(
                        float(st.horizon),
                        np.array([st.position.x, st.position.y,
                                  st.position.z]),
                        np.array([st.velocity.x, st.velocity.y,
                                  st.velocity.z]),
                        np.array(st.covariance).reshape(6, 6)[:3, :3]))
            elif risk.risk_state == POTENTIAL_THREAT:
                knots = ballistic_knots(p0, v0, cov, self.params.risk)
            else:
                continue
            threats.append(Threat(
                int(risk.track_id), stamp_s(tr.stamp), knots, float(tr.size),
                int(risk.risk_state), float(tr.time_since_update)))
        self.threats = threats
        self.perception_stamp = stamp_s(msg.header.stamp)
        self.plan()

    # -- planning --------------------------------------------------------
    def plan(self):
        """Run one planning cycle and publish the command."""
        if self.odom is None or self.status is None:
            return
        started = time.perf_counter()
        t_uav, position, velocity = self.odom
        status = self.status
        refused = status.avoidance_rejected > self.rejected_seen
        self.rejected_seen = status.avoidance_rejected
        mission = np.array([status.mission_target.x, status.mission_target.y,
                            status.mission_target.z])
        perception_age = (t_uav - self.perception_stamp
                          if self.perception_stamp is not None else 1e9)
        plan = self.planner.update(
            t_uav, position, velocity, float(status.ground_z), mission,
            bool(status.airborne_mission and status.ground_valid),
            self.threats, perception_age, time.monotonic() - self.odom_rx,
            command_refused=refused,
            odometry_lag_s=max(0.0, -perception_age),
            accel=self.measured_accel(t_uav),
            speed_limit=(0.0 if status.avoidance_active
                         else float(status.speed_limit)))
        elapsed_ms = (time.perf_counter() - started) * 1e3

        cmd = AvoidanceCommand()
        stamp = (self.perception_stamp if self.perception_stamp is not None
                 else t_uav)
        cmd.header.stamp.sec = int(stamp)
        cmd.header.stamp.nanosec = int(round((stamp - int(stamp)) * 1e9))
        cmd.header.frame_id = 'px4_local_ned'
        cmd.state = plan.mode
        cmd.active = bool(plan.active and self.enabled)
        if plan.target is not None:
            cmd.target.x, cmd.target.y, cmd.target.z = (
                float(v) for v in plan.target)
        cmd.precautionary = plan.precautionary
        cmd.failsafe = plan.failsafe
        cmd.threat_ids = [int(i) for i in plan.threat_ids]
        cmd.predicted_min_separation = float(
            min(plan.predicted_min_separation, 1e6))
        cmd.cost = float(plan.cost)
        cmd.candidate = plan.candidate
        cmd.reason = plan.reason
        cmd.perception_health = plan.health
        cmd.speed_limit = float(plan.speed_limit) if self.enabled else 0.0
        self.pub.publish(cmd)

        if plan.transition is not None:
            old, new, why = plan.transition
            self.get_logger().info(
                f'[{t_uav:.3f}] {MODE_NAMES[old]} -> {MODE_NAMES[new]}: '
                f'{why}')
        if refused:
            self.get_logger().warning(
                f'flight_controller refused the command: '
                f'{status.last_rejection}')

        debug = {
            't_uav': round(t_uav, 4),
            'perception_stamp': (round(self.perception_stamp, 4)
                                 if self.perception_stamp else None),
            'perception_age': round(min(perception_age, 99.0), 4),
            'compute_ms': round(elapsed_ms, 3),
            'mode': MODE_NAMES[plan.mode], 'enabled': self.enabled,
            'health': HEALTH_NAMES[plan.health],
            'speed_limit': round(float(plan.speed_limit), 2),
            'limit_in_force': round(float(status.speed_limit), 2),
            'active': bool(plan.active), 'candidate': plan.candidate,
            'precautionary': plan.precautionary, 'failsafe': plan.failsafe,
            'reason': plan.reason,
            'target': ([round(float(v), 3) for v in plan.target]
                       if plan.target is not None else None),
            'uav_p': [round(float(v), 4) for v in position],
            'uav_v': [round(float(v), 4) for v in velocity],
            'mission': [round(float(v), 3) for v in mission],
            'flight_state': status.state,
            'controller_active': bool(status.avoidance_active),
            'threats': [{'id': th.track_id,
                         'state': STATE_NAMES[th.risk_state]}
                        for th in self.threats],
            'transition': ([MODE_NAMES[plan.transition[0]],
                            MODE_NAMES[plan.transition[1]],
                            plan.transition[2]]
                           if plan.transition else None),
            'predicted_min_separation': round(float(min(
                plan.predicted_min_separation, 1e6)), 3),
            'candidates': [{
                'name': c.name, 'ok': c.feasible, 'why': c.rejected,
                'clear': round(min(c.min_clearance, 99.0), 3),
                'sep': round(min(c.min_separation, 99.0), 3),
                'cost': round(c.cost, 3),
                'target': [round(float(v), 2) for v in c.target],
                'end': [round(float(v), 2) for v in c.path[-1]]}
                for c in plan.candidates] if self.threats else []}
        msg = String()
        msg.data = json.dumps(debug, separators=(',', ':'))
        self.pub_debug.publish(msg)
        self.pub_markers.publish(self._markers(cmd.header, position, mission,
                                               plan))

    def _markers(self, header, position, mission, plan):
        """Safety volume, nominal setpoint, candidate paths, selection."""
        array = MarkerArray()
        clear = Marker()
        clear.header = header
        clear.action = Marker.DELETEALL
        array.markers.append(clear)

        def make(kind, marker_id, ns, rgb, alpha=1.0):
            m = Marker()
            m.header = header
            m.ns, m.id, m.type = ns, marker_id, kind
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.color.r, m.color.g, m.color.b = rgb
            m.color.a = alpha
            return m

        _, safety = radii(0.0, self.params.risk)
        volume = make(Marker.SPHERE, 0, 'safety_volume', (0.2, 0.6, 0.9), 0.2)
        (volume.pose.position.x, volume.pose.position.y,
         volume.pose.position.z) = (float(v) for v in position)
        volume.scale.x = volume.scale.y = volume.scale.z = 2 * safety
        array.markers.append(volume)
        nominal = make(Marker.LINE_STRIP, 1, 'nominal', (1.0, 1.0, 1.0))
        nominal.scale.x = 0.03
        nominal.points = [Point(x=float(position[0]), y=float(position[1]),
                                z=float(position[2])),
                          Point(x=float(mission[0]), y=float(mission[1]),
                                z=float(mission[2]))]
        array.markers.append(nominal)
        for i, cand in enumerate(plan.candidates if self.threats else []):
            selected = plan.active and cand.name == plan.candidate
            colour = ((0.1, 0.4, 1.0) if selected
                      else (0.1, 0.7, 0.2) if cand.feasible
                      else (0.85, 0.15, 0.15))
            line = make(Marker.LINE_STRIP, 10 + i, 'candidates', colour,
                        1.0 if selected else 0.5)
            line.scale.x = 0.06 if selected else 0.02
            line.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                           for p in cand.path[::5]]
            array.markers.append(line)
        text = make(Marker.TEXT_VIEW_FACING, 2, 'mode', (1.0, 1.0, 1.0))
        (text.pose.position.x, text.pose.position.y) = (
            float(position[0]), float(position[1]))
        text.pose.position.z = float(position[2]) - 1.2
        text.scale.z = 0.3
        text.text = f'{MODE_NAMES[plan.mode]} {plan.candidate}'
        array.markers.append(text)
        return array


def main(args=None):
    """Run the debris avoidance planner node."""
    rclpy.init(args=args)
    node = DebrisAvoidanceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
