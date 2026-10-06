#!/usr/bin/env python3
"""ROS 2 node: collision risk of tracked debris for the UAV. Advisory only.

Subscribes
    /perception/debris/tracks        uav_interfaces/DebrisTrackArray
    /perception/debris/predictions   uav_interfaces/DebrisPredictionArray
    /fmu/out/vehicle_odometry        px4_msgs/VehicleOdometry

Publishes
    /perception/debris/risk          uav_interfaces/DebrisRiskArray
    /perception/debris/risk_markers  visualization_msgs/MarkerArray
        (UAV safety volume, predicted paths, closest-approach points and
        text with ID, state, separation and time to closest approach)
    /perception/debris/risk_debug    std_msgs/String, JSON per frame

One output per prediction frame, stamped with the track state time. The UAV
state is interpolated to that time from odometry (timestamp_sample). ROS
arrival time is not used.

This node issues no command of any kind, does not publish to /fmu/in/*, and
does not read Gazebo ground truth.
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
    DebrisPredictionArray, DebrisRisk, DebrisRiskArray, DebrisTrackArray,
    FlightStatus)
from visualization_msgs.msg import Marker, MarkerArray

from uav_autonomy.debris_avoidance import AvoidanceParams, simulate_response
from uav_autonomy.debris_risk import (
    Knot,
    POTENTIAL_THREAT,
    RiskParams,
    STATE_NAMES,
    UNKNOWN,
    UavStateBuffer,
    assess,
    ballistic_knots,
    highest_state,
    radii,
)

STATE_COLOURS = {0: (0.1, 0.7, 0.2), 1: (0.95, 0.75, 0.1),
                 2: (0.95, 0.5, 0.1), 3: (0.85, 0.15, 0.15),
                 4: (0.6, 0.3, 0.8), 5: (0.5, 0.5, 0.5)}


def stamp_s(stamp) -> float:
    """builtin_interfaces/Time -> float seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


class DebrisRiskNode(Node):
    """Wrap the risk estimator with ROS interfaces."""

    def __init__(self):
        """Declare parameters and create publishers and subscribers."""
        super().__init__('debris_risk')
        defaults = RiskParams()
        values = {}
        for name in RiskParams.__dataclass_fields__:
            default = getattr(defaults, name)
            if isinstance(default, tuple):
                default = list(default)
            self.declare_parameter(name, default)
            value = self.get_parameter(name).value
            values[name] = tuple(value) if isinstance(value, list) else value
        self.params = RiskParams(**values)
        self.uav = UavStateBuffer()
        self.tracks = {}            # stamp (ns) -> DebrisTrackArray
        self.accel = []             # (t, a) PX4 acceleration samples
        self.status = None          # latest FlightStatus
        self.response = AvoidanceParams()

        self.pub = self.create_publisher(
            DebrisRiskArray, '/perception/debris/risk', 20)
        self.pub_markers = self.create_publisher(
            MarkerArray, '/perception/debris/risk_markers', 5)
        self.pub_debug = self.create_publisher(
            String, '/perception/debris/risk_debug', 50)
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
        contact, safety = radii(0.0, self.params)
        self.get_logger().info(
            f'Debris risk ready (advisory): contact radius {contact:.2f} m, '
            f'safety radius {safety:.2f} m for a '
            f'{self.params.default_debris_size_m:.2f} m object')

    def on_odometry(self, msg):
        """Store the UAV's own position and velocity (local NED)."""
        if (msg.pose_frame != VehicleOdometry.POSE_FRAME_NED
                or msg.velocity_frame != VehicleOdometry.VELOCITY_FRAME_NED):
            return
        self.uav.add(msg.timestamp_sample * 1e-6,
                     [float(v) for v in msg.position],
                     [float(v) for v in msg.velocity])

    def on_local_position(self, msg):
        """Keep PX4's acceleration estimate (local NED)."""
        a = [float(msg.ax), float(msg.ay), float(msg.az)]
        if np.all(np.isfinite(a)):
            self.accel.append((msg.timestamp_sample * 1e-6, a))
            del self.accel[:-200]

    def on_status(self, msg):
        """Keep the flight controller's commanded setpoint."""
        self.status = msg

    def uav_motion(self, t, uav_p, uav_v):
        """(uav_a, uav_traj) for the configured UAV motion model."""
        model = self.params.uav_motion_model
        if model == 'constant_velocity':
            return None, None
        recent = [a for ts, a in self.accel if t - 0.06 <= ts <= t + 0.005]
        accel = np.mean(recent, axis=0) if recent else None
        if model == 'constant_acceleration':
            return accel, None
        status = self.status
        if (model == 'response' and status is not None
                and status.airborne_mission):
            target = [status.commanded.x, status.commanded.y,
                      status.commanded.z]
            if not status.avoidance_active:
                target = [status.mission_target.x, status.mission_target.y,
                          status.mission_target.z]
            limit = 0.0 if status.avoidance_active else float(
                status.speed_limit)
            times, pos, _ = simulate_response(
                uav_p, uav_v, [target], self.response, target, [limit],
                limit, accel)
            return None, (times, pos[0])
        return None, None

    def on_tracks(self, msg):
        """Keep recent track frames so tentative tracks can be assessed."""
        key = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.tracks[key] = msg
        for old in sorted(self.tracks)[:-30]:
            del self.tracks[old]

    def on_predictions(self, msg):
        """Assess every predicted track, and fast tentative tracks."""
        started = time.perf_counter()
        t = stamp_s(msg.header.stamp)
        key = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        track_msg = self.tracks.get(key)
        by_id = ({tr.id: tr for tr in track_msg.tracks}
                 if track_msg is not None else {})
        uav = self.uav.at(t)

        out = DebrisRiskArray()
        out.header = msg.header
        out.uav_state_valid = uav is not None
        results = []
        if uav is None:
            for pred in msg.predictions:
                risk = DebrisRisk()
                risk.track_id = pred.track_id
                risk.stamp = pred.stamp
                risk.track_status = pred.track_status
                risk.risk_state = UNKNOWN
                out.risks.append(risk)
        else:
            uav_p, uav_v = uav
            uav_a, uav_traj = self.uav_motion(t, uav_p, uav_v)
            (out.uav_position.x, out.uav_position.y,
             out.uav_position.z) = (float(v) for v in uav_p)
            (out.uav_velocity.x, out.uav_velocity.y,
             out.uav_velocity.z) = (float(v) for v in uav_v)
            predicted = set()
            for pred in msg.predictions:
                predicted.add(pred.track_id)
                tr = by_id.get(pred.track_id)
                if tr is None or not pred.states:
                    continue
                knots = [Knot(0.0,
                              np.array([tr.position.x, tr.position.y,
                                        tr.position.z]),
                              np.array([tr.velocity.x, tr.velocity.y,
                                        tr.velocity.z]),
                              np.array(tr.covariance).reshape(6, 6)[:3, :3])]
                for st in pred.states:
                    knots.append(Knot(
                        float(st.horizon),
                        np.array([st.position.x, st.position.y,
                                  st.position.z]),
                        np.array([st.velocity.x, st.velocity.y,
                                  st.velocity.z]),
                        np.array(st.covariance).reshape(6, 6)[:3, :3]))
                results.append(assess(
                    pred.track_id, stamp_s(pred.stamp), pred.track_status,
                    knots, uav_p, uav_v, float(tr.size), self.params,
                    uav_a=uav_a, uav_traj=uav_traj))
            for tr in by_id.values():
                if (tr.id in predicted or tr.status != 0
                        or tr.hits < self.params.potential_min_hits
                        or tr.time_since_update > 0.0):
                    continue
                knots = ballistic_knots(
                    [tr.position.x, tr.position.y, tr.position.z],
                    [tr.velocity.x, tr.velocity.y, tr.velocity.z],
                    np.array(tr.covariance).reshape(6, 6), self.params)
                results.append(assess(
                    tr.id, stamp_s(tr.stamp), tr.status, knots, uav_p, uav_v,
                    float(tr.size), self.params, tentative=True,
                    uav_a=uav_a, uav_traj=uav_traj))
            out.highest_state = highest_state(results)
            out.potential_threat_present = any(
                r.state == POTENTIAL_THREAT for r in results)
            for r in results:
                out.risks.append(self._to_msg(r))
        self.pub.publish(out)
        elapsed_ms = (time.perf_counter() - started) * 1e3

        debug = String()
        debug.data = json.dumps({
            'stamp': round(t, 6), 'processing_ms': round(elapsed_ms, 4),
            'uav_valid': uav is not None,
            'uav_p': ([round(float(v), 4) for v in uav[0]]
                      if uav is not None else None),
            'uav_v': ([round(float(v), 4) for v in uav[1]]
                      if uav is not None else None),
            'risks': [{
                'id': r.track_id, 'track_status': r.track_status,
                'state': STATE_NAMES[r.state], 'score': round(r.score, 3),
                'tca': round(r.tca, 4), 'd_min': round(r.d_min, 4),
                'rel_speed': round(r.relative_speed, 3),
                'd_now': round(r.current_separation, 4),
                'contact_r': round(r.contact_radius, 3),
                'safety_r': round(r.safety_radius, 3),
                'unc': round(r.uncertainty_margin, 4),
                'd_cons': round(r.conservative_separation, 4),
                'intersects': r.intersects,
                'collision': r.predicted_collision,
                'horizon': r.horizon,
                'lin_tca': round(r.linear_tca, 4),
                'lin_d': round(r.linear_d_min, 4),
                'ca_p': [round(float(v), 3) for v in r.closest_point],
                'p': [round(float(v), 3) for v in r.path[0]],
                'sep_h': [round(v, 3) for v in r.separation_at_horizons]}
                for r in results]}, separators=(',', ':'))
        self.pub_debug.publish(debug)
        if uav is not None:
            self.pub_markers.publish(self._markers(msg.header, uav[0],
                                                   results))

    @staticmethod
    def _to_msg(r):
        risk = DebrisRisk()
        risk.track_id = r.track_id
        sec = int(r.stamp)
        risk.stamp.sec = sec
        risk.stamp.nanosec = int(round((r.stamp - sec) * 1e9))
        risk.track_status = r.track_status
        risk.risk_state = r.state
        risk.risk_score = float(r.score)
        risk.time_to_closest_approach = float(r.tca)
        risk.minimum_predicted_separation = float(r.d_min)
        risk.relative_speed = float(r.relative_speed)
        (risk.closest_point.x, risk.closest_point.y,
         risk.closest_point.z) = (float(v) for v in r.closest_point)
        (risk.uav_closest_point.x, risk.uav_closest_point.y,
         risk.uav_closest_point.z) = (float(v) for v in r.uav_closest_point)
        risk.current_separation = float(r.current_separation)
        risk.horizons = [float(v) for v in r.horizons]
        risk.separation_at_horizons = [
            float(v) for v in r.separation_at_horizons]
        risk.contact_radius = float(r.contact_radius)
        risk.safety_radius = float(r.safety_radius)
        risk.uncertainty_margin = float(r.uncertainty_margin)
        risk.conservative_separation = float(r.conservative_separation)
        risk.safety_volume_intersected = r.intersects
        risk.predicted_collision = r.predicted_collision
        risk.prediction_horizon = float(r.horizon)
        risk.linear_time_to_closest_approach = float(r.linear_tca)
        risk.linear_minimum_separation = float(r.linear_d_min)
        return risk

    def _markers(self, header, uav_p, results):
        """Safety volume, predicted paths, closest-approach points, text."""
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

        _, safety = radii(0.0, self.params)
        worst = highest_state(results)
        volume = make(Marker.SPHERE, 0, 'safety_volume',
                      STATE_COLOURS[worst], 0.25)
        (volume.pose.position.x, volume.pose.position.y,
         volume.pose.position.z) = (float(v) for v in uav_p)
        scale = self.params.safety_axes_scale
        volume.scale.x = 2 * safety * scale[0]
        volume.scale.y = 2 * safety * scale[1]
        volume.scale.z = 2 * safety * scale[2]
        array.markers.append(volume)
        for r in results:
            colour = STATE_COLOURS[r.state]
            base = 10 + r.track_id * 10
            path = make(Marker.LINE_STRIP, base, 'predicted_path', colour)
            path.scale.x = 0.04
            path.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                           for p in r.path]
            array.markers.append(path)
            now = make(Marker.SPHERE, base + 1, 'debris', colour)
            (now.pose.position.x, now.pose.position.y,
             now.pose.position.z) = (float(v) for v in r.path[0])
            now.scale.x = now.scale.y = now.scale.z = 0.3
            array.markers.append(now)
            ca = make(Marker.SPHERE, base + 2, 'closest_approach', colour,
                      0.6)
            (ca.pose.position.x, ca.pose.position.y,
             ca.pose.position.z) = (float(v) for v in r.closest_point)
            ca.scale.x = ca.scale.y = ca.scale.z = 0.2
            array.markers.append(ca)
            link = make(Marker.LINE_LIST, base + 3, 'separation', colour)
            link.scale.x = 0.02
            link.points = [
                Point(x=float(r.closest_point[0]),
                      y=float(r.closest_point[1]),
                      z=float(r.closest_point[2])),
                Point(x=float(r.uav_closest_point[0]),
                      y=float(r.uav_closest_point[1]),
                      z=float(r.uav_closest_point[2]))]
            array.markers.append(link)
            text = make(Marker.TEXT_VIEW_FACING, base + 4, 'label',
                        (1.0, 1.0, 1.0))
            text.pose.position.x = float(r.path[0][0])
            text.pose.position.y = float(r.path[0][1])
            text.pose.position.z = float(r.path[0][2]) - 0.5
            text.scale.z = 0.3
            text.text = (f'#{r.track_id} {STATE_NAMES[r.state]}\n'
                         f'd_min {r.d_min:.2f} m in {r.tca:.2f} s')
            array.markers.append(text)
        return array


def main(args=None):
    """Run the debris risk node."""
    rclpy.init(args=args)
    node = DebrisRiskNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
