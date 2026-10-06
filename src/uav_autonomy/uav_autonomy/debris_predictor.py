#!/usr/bin/env python3
"""ROS 2 node: future trajectories of confirmed debris tracks.

Subscribes
    /perception/debris/tracks   uav_interfaces/DebrisTrackArray

Publishes
    /perception/debris/predictions         uav_interfaces/DebrisPredictionArray
    /perception/debris/prediction_markers  visualization_msgs/MarkerArray
        (current position, track ID, velocity arrow, predicted path with one
        marker per horizon; markers shrink and fade with the horizon)
    /perception/debris/predictions_debug   std_msgs/String, JSON per frame

Times are the track's own (sensor) timestamps; a predicted state's stamp is
track stamp + horizon. Tentative, stale and dead tracks get no prediction.

Prediction only: no separation, risk or commands are computed here. This
node does not publish to /fmu/in/* and does not read Gazebo ground truth.
"""

import json
import time

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from uav_interfaces.msg import (
    DebrisPrediction, DebrisPredictionArray, DebrisTrackArray,
    PredictedState)
from visualization_msgs.msg import Marker, MarkerArray

from uav_autonomy.debris_prediction import (
    DebrisPredictor,
    MODEL_NAMES,
    PredictorParams,
    TrackState,
)


def to_time(seconds: float) -> Time:
    """Float seconds -> builtin_interfaces/Time."""
    sec = int(seconds)
    return Time(sec=sec, nanosec=int(round((seconds - sec) * 1e9)))


class DebrisPredictorNode(Node):
    """Wrap DebrisPredictor with ROS interfaces."""

    def __init__(self):
        """Declare parameters and create publishers and the subscriber."""
        super().__init__('debris_predictor')
        defaults = PredictorParams()
        values = {}
        for name in PredictorParams.__dataclass_fields__:
            default = getattr(defaults, name)
            if isinstance(default, tuple):
                default = list(default)
            self.declare_parameter(name, default)
            value = self.get_parameter(name).value
            values[name] = tuple(value) if isinstance(value, list) else value
        self.predictor = DebrisPredictor(PredictorParams(**values))
        self.pub = self.create_publisher(
            DebrisPredictionArray, '/perception/debris/predictions', 20)
        self.pub_markers = self.create_publisher(
            MarkerArray, '/perception/debris/prediction_markers', 5)
        self.pub_debug = self.create_publisher(
            String, '/perception/debris/predictions_debug', 50)
        self.create_subscription(
            DebrisTrackArray, '/perception/debris/tracks', self.on_tracks, 50)
        self.get_logger().info(
            f'Debris predictor ready: model={self.predictor.params.model}, '
            f'horizons={list(self.predictor.params.horizons_s)} s')

    def on_tracks(self, msg):
        """Predict every valid track of one frame."""
        started = time.perf_counter()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        states = []
        for tr in msg.tracks:
            x = np.array([tr.position.x, tr.position.y, tr.position.z,
                          tr.velocity.x, tr.velocity.y, tr.velocity.z])
            states.append(TrackState(
                id=tr.id,
                stamp=tr.stamp.sec + tr.stamp.nanosec * 1e-9,
                x=x, P=np.array(tr.covariance).reshape(6, 6),
                status=tr.status,
                time_since_update=float(tr.time_since_update)))
        predictions = self.predictor.update(states)

        array = DebrisPredictionArray()
        array.header = msg.header
        for pred in predictions:
            out = DebrisPrediction()
            out.track_id = pred.track_id
            out.stamp = to_time(pred.stamp)
            out.track_status = pred.status
            out.time_since_update = float(pred.time_since_update)
            out.model = pred.model
            (out.acceleration.x, out.acceleration.y,
             out.acceleration.z) = (float(v) for v in pred.acceleration)
            for pt in pred.points:
                state = PredictedState()
                state.horizon = float(pt.horizon)
                state.stamp = to_time(pt.stamp)
                (state.position.x, state.position.y,
                 state.position.z) = (float(v) for v in pt.x[:3])
                (state.velocity.x, state.velocity.y,
                 state.velocity.z) = (float(v) for v in pt.x[3:])
                state.covariance = [float(v) for v in pt.P.reshape(-1)]
                state.below_takeoff_plane = pt.below_takeoff_plane
                out.states.append(state)
            array.predictions.append(out)
        self.pub.publish(array)
        elapsed_ms = (time.perf_counter() - started) * 1e3

        by_id = {s.id: s for s in states}
        debug = String()
        debug.data = json.dumps({
            'stamp': round(stamp, 6),
            'processing_ms': round(elapsed_ms, 4),
            'tracks_in': len(states),
            'predictions': [{
                'id': p.track_id, 'status': p.status,
                'tsu': round(p.time_since_update, 4),
                'model': MODEL_NAMES[p.model],
                'accel': [round(float(v), 3) for v in p.acceleration],
                'state_p': [round(float(v), 4)
                            for v in by_id[p.track_id].x[:3]],
                'state_v': [round(float(v), 4)
                            for v in by_id[p.track_id].x[3:]],
                'points': [{
                    'h': pt.horizon,
                    'p': [round(float(v), 4) for v in pt.x[:3]],
                    'v': [round(float(v), 4) for v in pt.x[3:]],
                    'sigma': round(float(np.sqrt(np.trace(pt.P[:3, :3]))),
                                   4),
                    'below': pt.below_takeoff_plane}
                    for pt in p.points]}
                for p in predictions]}, separators=(',', ':'))
        self.pub_debug.publish(debug)
        self.pub_markers.publish(self._markers(msg.header, predictions,
                                               by_id))

    @staticmethod
    def _markers(header, predictions, by_id):
        """Current state plus the predicted path, fading with the horizon."""
        array = MarkerArray()
        clear = Marker()
        clear.header = header
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        for pred in predictions:
            state = by_id[pred.track_id]
            base = pred.track_id * 20

            def make(kind, offset, ns, rgba):
                m = Marker()
                m.header = header
                m.ns, m.id, m.type = ns, base + offset, kind
                m.action = Marker.ADD
                m.pose.orientation.w = 1.0
                m.color.r, m.color.g, m.color.b, m.color.a = rgba
                return m

            now = make(Marker.SPHERE, 0, 'current', (1.0, 0.55, 0.0, 1.0))
            (now.pose.position.x, now.pose.position.y,
             now.pose.position.z) = (float(v) for v in state.x[:3])
            now.scale.x = now.scale.y = now.scale.z = 0.3
            array.markers.append(now)
            text = make(Marker.TEXT_VIEW_FACING, 1, 'id',
                        (1.0, 1.0, 1.0, 1.0))
            text.pose.position.x = float(state.x[0])
            text.pose.position.y = float(state.x[1])
            text.pose.position.z = float(state.x[2]) - 0.5
            text.scale.z = 0.3
            text.text = (f'#{pred.track_id} '
                         f'{np.linalg.norm(state.x[3:]):.1f} m/s')
            array.markers.append(text)
            arrow = make(Marker.ARROW, 2, 'velocity', (1.0, 0.55, 0.0, 1.0))
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.05, 0.1, 0.1
            arrow.points = [
                Point(x=float(state.x[0]), y=float(state.x[1]),
                      z=float(state.x[2])),
                Point(x=float(state.x[0] + 0.2 * state.x[3]),
                      y=float(state.x[1] + 0.2 * state.x[4]),
                      z=float(state.x[2] + 0.2 * state.x[5]))]
            array.markers.append(arrow)
            path = make(Marker.LINE_STRIP, 3, 'predicted_path',
                        (0.2, 0.6, 1.0, 0.9))
            path.scale.x = 0.04
            path.points = [Point(x=float(state.x[0]), y=float(state.x[1]),
                                 z=float(state.x[2]))]
            longest = pred.points[-1].horizon
            for index, pt in enumerate(pred.points):
                path.points.append(Point(x=float(pt.x[0]), y=float(pt.x[1]),
                                         z=float(pt.x[2])))
                fade = 1.0 - 0.7 * pt.horizon / longest
                dot = make(Marker.SPHERE, 4 + index, 'horizon',
                           (0.2, 0.6, 1.0, fade))
                (dot.pose.position.x, dot.pose.position.y,
                 dot.pose.position.z) = (float(v) for v in pt.x[:3])
                dot.scale.x = dot.scale.y = dot.scale.z = 0.12 + 0.18 * fade
                array.markers.append(dot)
            array.markers.append(path)
        return array


def main(args=None):
    """Run the debris predictor node."""
    rclpy.init(args=args)
    node = DebrisPredictorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
