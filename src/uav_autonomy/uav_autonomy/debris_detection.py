"""Debris detection from one metric depth image (no ROS imports).

Pipeline for each frame of the upward depth camera:

    depth image -> valid mask -> connected regions, split at depth jumps
                -> 3D points per region -> size / pixel-count gates
                -> detections in the sensor frame
                -> (optional) UAV pose -> detections in the local NED frame
                -> static-object filter in the local frame

Inputs are sensor data only: the depth image, the camera intrinsics, and the
vehicle's own odometry. Nothing here knows where debris was released.

Frames
    sensor (depth_up_link): +X optical axis, +Y image left, +Z image up.
        pixel (u, v), axial depth d  ->  p_s = (d, -(u-cx) d/fx, -(v-cy) d/fy)
    body FLU (base_link): the sensor is mounted with +X_s = +Z_b (up),
        +Y_s = +Y_b, +Z_s = -X_b, at SENSOR_OFFSET_FLU from base_link.
    body FRD (PX4): (x, -y, -z) of FLU.
    local NED (PX4 local frame): p_ned = R(q) p_frd + position.
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2
import numpy as np

SENSOR_OFFSET_FLU = np.array([0.0, 0.0, 0.15])


@dataclass(frozen=True)
class Intrinsics:
    """Pinhole intrinsics of the depth camera."""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


@dataclass(frozen=True)
class DetectorParams:
    """Detector thresholds (see docs/debris_perception.md)."""

    min_range_m: float = 0.5      # nearer returns: UAV body / rotors
    max_range_m: float = 35.0     # far clip of the sensor
    depth_jump_m: float = 0.30    # split a region where depth jumps by more
    min_pixels: int = 12          # smaller regions are treated as noise
    min_size_m: float = 0.08      # smallest physical extent kept
    max_size_m: float = 2.5       # larger surfaces: building / ground
    # static-object filter (local frame)
    static_radius_m: float = 0.20
    static_time_s: float = 0.75
    static_min_fraction: float = 0.6
    history_s: float = 2.0


@dataclass
class Detection:
    """One debris candidate. Positions are of the *visible surface*."""

    position_sensor: np.ndarray           # centroid, sensor frame [m]
    size_sensor: np.ndarray               # extent along sensor X, Y, Z [m]
    pixels: int
    bbox_uv: Tuple[int, int, int, int]    # u_min, v_min, u_max, v_max
    range_m: float
    touches_border: bool
    confidence: float
    position_local: Optional[np.ndarray] = None   # NED [m], if pose known
    is_static: bool = False

    @property
    def size_m(self) -> float:
        """Largest lateral extent, the 'approximate size'."""
        return float(max(self.size_sensor[1], self.size_sensor[2]))


@dataclass
class FrameResult:
    """Everything the detector decided for one frame."""

    detections: List[Detection] = field(default_factory=list)
    rejected: dict = field(default_factory=dict)   # reason -> count
    valid_pixels: int = 0
    near_pixels: int = 0


def backproject(us, vs, depth, k: Intrinsics) -> np.ndarray:
    """Pixels and axial depth -> points in the sensor frame, shape (N, 3)."""
    d = np.asarray(depth, dtype=np.float64)
    return np.column_stack([
        d, -(np.asarray(us) - k.cx) * d / k.fx,
        -(np.asarray(vs) - k.cy) * d / k.fy])


def _split_by_depth(depths: np.ndarray, jump: float) -> np.ndarray:
    """Label 1D depth values into groups separated by gaps larger than jump."""
    order = np.argsort(depths)
    gaps = np.diff(depths[order]) > jump
    labels_sorted = np.concatenate([[0], np.cumsum(gaps)])
    labels = np.empty(len(depths), dtype=np.int32)
    labels[order] = labels_sorted
    return labels


def _confidence(pixels: int, depth_std: float, touches_border: bool,
                params: DetectorParams) -> float:
    """Heuristic score in [0, 1]; not a calibrated probability.

    More pixels, a compact depth distribution and a fully visible region each
    raise it. See docs/debris_perception.md.
    """
    support = min(1.0, pixels / 150.0)
    compact = 1.0 - min(1.0, depth_std / max(params.depth_jump_m, 1e-6))
    score = 0.30 + 0.35 * support + 0.20 * compact
    score += 0.0 if touches_border else 0.15
    return float(min(max(score, 0.0), 1.0))


def detect_frame(depth: np.ndarray, k: Intrinsics,
                 params: DetectorParams = DetectorParams()) -> FrameResult:
    """Detect debris candidates in one depth image (sensor frame only)."""
    result = FrameResult()
    depth = np.asarray(depth)
    finite = np.isfinite(depth)
    near = np.isneginf(depth) | (finite & (depth < params.min_range_m))
    valid = finite & (depth >= params.min_range_m) & (
        depth <= params.max_range_m)
    result.valid_pixels = int(valid.sum())
    result.near_pixels = int(near.sum())
    if result.valid_pixels == 0:
        return result

    def reject(reason):
        result.rejected[reason] = result.rejected.get(reason, 0) + 1

    count, labels = cv2.connectedComponents(valid.astype(np.uint8), 8)
    for label in range(1, count):
        vs, us = np.nonzero(labels == label)
        d = depth[vs, us].astype(np.float64)
        groups = (_split_by_depth(d, params.depth_jump_m)
                  if d.max() - d.min() > params.depth_jump_m
                  else np.zeros(len(d), dtype=np.int32))
        for group in range(int(groups.max()) + 1):
            sel = groups == group
            gu, gv, gd = us[sel], vs[sel], d[sel]
            if len(gd) < params.min_pixels:
                reject('too_few_pixels')
                continue
            # Cheap pre-check from the pixel bounding box: its metric width
            # at the region's nearest depth is a lower bound on the extent.
            span_px = max(gu.max() - gu.min(), gv.max() - gv.min())
            if span_px * float(gd.min()) / k.fx > 2.0 * params.max_size_m:
                reject('too_large')
                continue
            points = backproject(gu, gv, gd, k)
            centroid = points.mean(axis=0)
            pixel_m = float(np.median(gd)) / k.fx   # footprint of one pixel
            size = points.max(axis=0) - points.min(axis=0)
            size[1] += pixel_m
            size[2] += pixel_m
            lateral = float(max(size[1], size[2]))
            if lateral > params.max_size_m:
                reject('too_large')
                continue
            if lateral < params.min_size_m:
                reject('too_small')
                continue
            border = bool(gu.min() == 0 or gv.min() == 0
                          or gu.max() == k.width - 1
                          or gv.max() == k.height - 1)
            result.detections.append(Detection(
                position_sensor=centroid, size_sensor=size,
                pixels=int(len(gd)),
                bbox_uv=(int(gu.min()), int(gv.min()), int(gu.max()),
                         int(gv.max())),
                range_m=float(np.linalg.norm(centroid)),
                touches_border=border,
                confidence=_confidence(len(gd), float(gd.std()), border,
                                       params)))
    return result


# --- frames -----------------------------------------------------------------

def quat_to_matrix(w: float, x: float, y: float, z: float) -> np.ndarray:
    """Rotation matrix of a unit quaternion (Hamilton, w first)."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def sensor_to_body_frd(p_s: np.ndarray) -> np.ndarray:
    """Sensor-frame point(s) -> PX4 body FRD frame (origin base_link)."""
    p = np.atleast_2d(p_s)
    flu = np.column_stack([-p[:, 2], p[:, 1], p[:, 0]]) + SENSOR_OFFSET_FLU
    frd = np.column_stack([flu[:, 0], -flu[:, 1], -flu[:, 2]])
    return frd if np.ndim(p_s) == 2 else frd[0]


def sensor_to_local(p_s: np.ndarray, position_ned: np.ndarray,
                    q_wxyz: np.ndarray) -> np.ndarray:
    """Sensor-frame point(s) -> PX4 local NED using the vehicle pose."""
    frd = np.atleast_2d(sensor_to_body_frd(p_s))
    ned = frd @ quat_to_matrix(*q_wxyz).T + np.asarray(position_ned)
    return ned if np.ndim(p_s) == 2 else ned[0]


class PoseBuffer:
    """Vehicle poses keyed by PX4 timestamp_sample; interpolates to a time."""

    def __init__(self, horizon_s: float = 3.0, max_gap_s: float = 0.05,
                 max_extrapolation_s: float = 0.03):
        self.horizon_s = horizon_s
        self.max_gap_s = max_gap_s
        self.max_extrapolation_s = max_extrapolation_s
        self._t: List[float] = []
        self._p: List[np.ndarray] = []
        self._q: List[np.ndarray] = []

    def add(self, t: float, position_ned, q_wxyz):
        """Append one pose sample (ignored if not newer or not finite)."""
        p = np.asarray(position_ned, dtype=np.float64)
        q = np.asarray(q_wxyz, dtype=np.float64)
        if not (np.all(np.isfinite(p)) and np.all(np.isfinite(q))):
            return
        if self._t and t <= self._t[-1]:
            return
        self._t.append(t)
        self._p.append(p)
        self._q.append(q / np.linalg.norm(q))
        while self._t and self._t[0] < t - self.horizon_s:
            self._t.pop(0)
            self._p.pop(0)
            self._q.pop(0)

    def latest_time(self) -> Optional[float]:
        """Time of the newest sample, or None."""
        return self._t[-1] if self._t else None

    def at(self, t: float):
        """Pose (position, quaternion) at time t, or None if not covered."""
        if not self._t:
            return None
        ts = self._t
        if t <= ts[0]:
            ok = ts[0] - t <= self.max_extrapolation_s
            return (self._p[0], self._q[0]) if ok else None
        if t >= ts[-1]:
            ok = t - ts[-1] <= self.max_extrapolation_s
            return (self._p[-1], self._q[-1]) if ok else None
        i = int(np.searchsorted(ts, t))
        t0, t1 = ts[i - 1], ts[i]
        if t1 - t0 > self.max_gap_s:
            return None
        a = (t - t0) / (t1 - t0)
        p = (1 - a) * self._p[i - 1] + a * self._p[i]
        q0, q1 = self._q[i - 1], self._q[i]
        if np.dot(q0, q1) < 0:
            q1 = -q1
        q = (1 - a) * q0 + a * q1     # samples are <= 50 ms apart
        return p, q / np.linalg.norm(q)


class StaticFilter:
    """Flag detections that stay in one place in the local frame.

    A detection is static when an earlier detection at least
    ``static_time_s`` old lies within ``static_radius_m`` of it, and at least
    ``static_min_fraction`` of the frames since then also contain one there.
    Falling debris moves away from where it was; structure that the UAV flies
    past does not, once expressed in the local frame.
    """

    def __init__(self, params: DetectorParams = DetectorParams()):
        self.params = params
        self._frames: List[Tuple[float, np.ndarray]] = []

    def update(self, t: float, positions_local: np.ndarray) -> np.ndarray:
        """Return a boolean static flag per position; then store the frame."""
        p = self.params
        positions = np.asarray(positions_local, dtype=np.float64).reshape(
            -1, 3)
        self._frames = [f for f in self._frames
                        if 0 <= t - f[0] <= p.history_s]
        flags = np.zeros(len(positions), dtype=bool)
        for i, pos in enumerate(positions):
            hit_times = [ft for ft, fp in self._frames if len(fp) and np.min(
                np.linalg.norm(fp - pos, axis=1)) <= p.static_radius_m]
            if not hit_times or t - min(hit_times) < p.static_time_s:
                continue
            since = [ft for ft, _ in self._frames if ft >= min(hit_times)]
            flags[i] = len(hit_times) / len(since) >= p.static_min_fraction
        self._frames.append((t, positions.copy()))
        return flags


class DebrisDetector:
    """Frame detector + local-frame transform + static filter."""

    def __init__(self, intrinsics: Intrinsics,
                 params: DetectorParams = DetectorParams()):
        self.k = intrinsics
        self.params = params
        self.static_filter = StaticFilter(params)

    def process(self, depth: np.ndarray, stamp: float,
                pose=None) -> FrameResult:
        """Run one frame. ``pose`` is (position_ned, q_wxyz) or None."""
        result = detect_frame(depth, self.k, self.params)
        if pose is None or not result.detections:
            if pose is not None:
                self.static_filter.update(stamp, np.zeros((0, 3)))
            return result
        local = sensor_to_local(
            np.array([d.position_sensor for d in result.detections]),
            pose[0], pose[1])
        flags = self.static_filter.update(stamp, local)
        kept = []
        for det, pos, static in zip(result.detections, local, flags):
            det.position_local = pos
            det.is_static = bool(static)
            if static:
                result.rejected['static'] = result.rejected.get(
                    'static', 0) + 1
            else:
                kept.append(det)
        result.detections = kept
        return result


def depth_to_debug_image(depth: np.ndarray, result: FrameResult,
                         max_range_m: float = 35.0) -> np.ndarray:
    """BGR visualisation: depth as colour, detections as labelled boxes."""
    finite = np.isfinite(depth)
    scaled = np.zeros(depth.shape, dtype=np.uint8)
    scaled[finite] = np.clip(
        255.0 * (1.0 - depth[finite] / max_range_m), 0, 255).astype(np.uint8)
    image = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
    image[~finite] = (25, 25, 25)
    for det in result.detections:
        u0, v0, u1, v1 = det.bbox_uv
        cv2.rectangle(image, (u0 - 2, v0 - 2), (u1 + 2, v1 + 2),
                      (255, 255, 255), 1)
        label = (f'{det.range_m:.1f}m {det.size_m:.2f}m '
                 f'c={det.confidence:.2f}')
        cv2.putText(image, label, (max(0, u0 - 2), max(12, v0 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    text = (f'detections={len(result.detections)} '
            f'rejected={dict(result.rejected)}')
    cv2.putText(image, text, (6, depth.shape[0] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    return image


def is_finite_pose(position, q) -> bool:
    """Guard used by the node before storing an odometry sample."""
    return all(math.isfinite(float(v)) for v in list(position) + list(q))
