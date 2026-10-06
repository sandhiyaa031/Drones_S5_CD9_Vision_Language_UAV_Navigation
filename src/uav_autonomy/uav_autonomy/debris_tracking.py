"""Multi-object debris tracker (no ROS imports).

Each track is a Kalman filter on the state

    x = [p_x, p_y, p_z, v_x, v_y, v_z]^T        (PX4 local NED, metres, m/s)

Measurement
    z = position of the centroid of the object's VISIBLE SURFACE, as published
    by the Phase 2 detector in the local frame:  z = H x + w,  H = [I3 0].
    The tracked point is therefore that surface point, not the box centre.

Motion model (variable dt, taken from sensor timestamps)
    p' = p + v dt (+ 0.5 g dt^2)        v' = v (+ g dt)
    with discrete white-noise acceleration as process noise,
    Q_axis = sigma_a^2 [[dt^4/4, dt^3/2], [dt^3/2, dt^2]].
    ``gravity_m_s2`` is the known gravitational acceleration applied along +z
    (down in NED). 0 gives the plain constant-velocity model.

Association
    Tracks are predicted to the frame's timestamp; detections are assigned by
    the Hungarian algorithm on squared Mahalanobis distance, gated by a
    chi-square threshold and by a physical reach limit.

Time
    Only sensor timestamps are used. A frame whose stamp is not newer than
    the last processed frame is dropped (out-of-order, duplicate or stale).
"""

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

import numpy as np
from scipy.optimize import linear_sum_assignment

TENTATIVE, CONFIRMED, COASTING = 0, 1, 2
STATUS_NAMES = {TENTATIVE: 'TENTATIVE', CONFIRMED: 'CONFIRMED',
                COASTING: 'COASTING'}


@dataclass(frozen=True)
class TrackerParams:
    """Tracker settings; rationale in docs/debris_tracking.md."""

    gravity_m_s2: float = 9.8            # 0 = plain constant velocity
    sigma_accel_xy: float = 3.0          # process noise [m/s^2]
    sigma_accel_z: float = 3.0           # use about 12 if gravity is 0
    sigma_meas_m: float = 0.10           # measurement noise, each axis [m]
    init_sigma_vel: float = 8.0          # prior velocity uncertainty [m/s]
    init_velocity_z: float = 0.0         # prior vertical velocity [m/s]
    gate_chi2: float = 16.27             # chi-square 3 dof, 99.9 %
    max_speed_m_s: float = 25.0          # reach limit: |innovation| <=
    reach_margin_m: float = 0.6          #   max_speed * dt + margin
    confirm_hits: int = 3
    # A track is confirmed only once its measurements have moved this far
    # from where it was first seen. Structure that enters the view (roof
    # edges while climbing through the opening) is detected for a moment
    # before the detector's static filter removes it; it never moves in the
    # local frame, so it never becomes a confirmed debris track.
    confirm_min_displacement_m: float = 0.10
    # Only measurements of a fully visible object count towards confirmation
    # (hits and displacement). A region cut by the image border is used to
    # update an existing track, with this larger position uncertainty.
    partial_sigma_m: float = 0.30
    tentative_max_misses: int = 2        # frames
    confirmed_max_misses: int = 6        # frames without a measurement
    max_coast_s: float = 1.0             # time without a measurement
    max_dt_s: float = 2.0                # longer gap: drop all tracks
    history_len: int = 60
    # Merged detections: when two objects touch in the depth image the
    # detector reports one larger region. A confirmed track left without a
    # measurement, whose prediction lies inside such a larger region that
    # another track claimed, is held (no miss counted) instead of dying.
    merge_size_ratio: float = 1.4
    merge_margin_m: float = 0.25
    merge_max_hold_s: float = 0.6


@dataclass
class Measurement:
    """One detection in the local frame."""

    position: np.ndarray
    size_m: float = 0.0
    confidence: float = 1.0
    # True when the region touched the image border: only part of the object
    # (or of a structure) was visible, so the centroid is biased.
    partial: bool = False


@dataclass
class Track:
    """State of one tracked object."""

    id: int
    x: np.ndarray
    P: np.ndarray
    stamp: float
    born: float
    last_update: float
    hits: int = 1
    misses: int = 0
    status: int = TENTATIVE
    size_m: float = 0.0
    score: float = 1.0
    last_measurement: Optional[np.ndarray] = None
    last_innovation: Optional[np.ndarray] = None
    last_nis: Optional[float] = None
    last_dt: float = 0.0
    merged_frames: int = 0
    first_position: Optional[np.ndarray] = None
    displacement: float = 0.0
    full_hits: int = 0      # measurements of the fully visible object
    history: List[np.ndarray] = field(default_factory=list)

    @property
    def age(self) -> float:
        """Seconds since the first measurement."""
        return self.stamp - self.born

    @property
    def time_since_update(self) -> float:
        """Seconds since the last associated measurement."""
        return self.stamp - self.last_update

    @property
    def confidence(self) -> float:
        """Heuristic 0..1: support, recency and detector score."""
        support = min(1.0, self.hits / 5.0)
        recency = math.exp(-self.time_since_update / 0.3)
        return float(support * recency * min(1.0, max(0.0, self.score)))


H = np.hstack([np.eye(3), np.zeros((3, 3))])


def transition(dt: float) -> np.ndarray:
    """State transition matrix for a time step dt."""
    f = np.eye(6)
    f[0, 3] = f[1, 4] = f[2, 5] = dt
    return f


def process_noise(dt: float, params: TrackerParams) -> np.ndarray:
    """Discrete white-noise-acceleration covariance for a time step dt."""
    q = np.zeros((6, 6))
    for axis, sigma in enumerate((params.sigma_accel_xy,
                                  params.sigma_accel_xy,
                                  params.sigma_accel_z)):
        s2 = sigma * sigma
        q[axis, axis] = s2 * dt ** 4 / 4.0
        q[axis, axis + 3] = q[axis + 3, axis] = s2 * dt ** 3 / 2.0
        q[axis + 3, axis + 3] = s2 * dt ** 2
    return q


def predict_state(x: np.ndarray, p: np.ndarray, dt: float,
                  params: TrackerParams):
    """Propagate mean and covariance by dt (dt >= 0)."""
    f = transition(dt)
    x_new = f @ x
    if params.gravity_m_s2:
        x_new[2] += 0.5 * params.gravity_m_s2 * dt * dt
        x_new[5] += params.gravity_m_s2 * dt
    return x_new, f @ p @ f.T + process_noise(dt, params)


class DebrisTracker:
    """Create, update and delete tracks from timestamped detection frames."""

    def __init__(self, params: TrackerParams = TrackerParams()):
        """Start with no tracks."""
        self.params = params
        self.tracks: Dict[int, Track] = {}
        self.next_id = 1
        self.last_stamp: Optional[float] = None
        self.dropped_frames = {'out_of_order': 0, 'duplicate': 0}
        self.deleted: List[Track] = []
        self.resets = 0

    # -- frame handling ----------------------------------------------------

    def step(self, stamp: float,
             measurements: Sequence[Measurement]) -> Optional[List[Track]]:
        """Process one detection frame; return tracks, or None if dropped."""
        p = self.params
        if self.last_stamp is not None:
            if stamp == self.last_stamp:
                self.dropped_frames['duplicate'] += 1
                return None
            if stamp < self.last_stamp:
                self.dropped_frames['out_of_order'] += 1
                return None
            if stamp - self.last_stamp > p.max_dt_s:
                # Nothing sensible can be associated across such a gap.
                self.deleted.extend(self.tracks.values())
                self.tracks = {}
                self.resets += 1
        self.last_stamp = stamp
        self.deleted = []

        for track in self.tracks.values():
            dt = stamp - track.stamp
            track.x, track.P = predict_state(track.x, track.P, dt, p)
            track.last_dt = dt
            track.stamp = stamp

        assigned = self._associate(measurements)
        used = set()
        for track_id, index in assigned.items():
            self._update(self.tracks[track_id], measurements[index])
            used.add(index)
        for track_id, track in self.tracks.items():
            if track_id not in assigned:
                if self._hidden_in_merge(track, measurements, used):
                    track.merged_frames += 1
                else:
                    track.misses += 1
                track.last_measurement = None
                track.last_innovation = None
                track.last_nis = None
                if track.status == CONFIRMED:
                    track.status = COASTING
        for index, meas in enumerate(measurements):
            if index not in used:
                self._birth(stamp, meas)
        self._prune()
        for track in self.tracks.values():
            track.history.append(track.x[:3].copy())
            del track.history[:-p.history_len]
        return list(self.tracks.values())

    # -- internals ---------------------------------------------------------

    def _associate(self, measurements) -> Dict[int, int]:
        p = self.params
        ids = list(self.tracks)
        if not ids or not measurements:
            return {}
        big = 1e6
        cost = np.full((len(ids), len(measurements)), big)
        r = np.eye(3) * p.sigma_meas_m ** 2
        for i, track_id in enumerate(ids):
            track = self.tracks[track_id]
            s_inv = np.linalg.inv(H @ track.P @ H.T + r)
            reach = (p.max_speed_m_s * max(track.time_since_update, 0.0)
                     + p.reach_margin_m)
            for j, meas in enumerate(measurements):
                nu = meas.position - track.x[:3]
                if np.linalg.norm(nu) > reach:
                    continue
                d2 = float(nu @ s_inv @ nu)
                if d2 <= p.gate_chi2:
                    # confirmed tracks win ties against tentative ones
                    cost[i, j] = d2 + (0.0 if track.status != TENTATIVE
                                       else 1e-3)
        rows, cols = linear_sum_assignment(cost)
        return {ids[i]: int(j) for i, j in zip(rows, cols)
                if cost[i, j] < big}

    def _hidden_in_merge(self, track, measurements, used) -> bool:
        """True if another track's (larger) detection covers this track."""
        p = self.params
        if (track.status == TENTATIVE
                or track.time_since_update > p.merge_max_hold_s):
            return False
        for index in used:
            meas = measurements[index]
            if meas.size_m < p.merge_size_ratio * max(track.size_m, 1e-3):
                continue
            reach = 0.5 * meas.size_m + p.merge_margin_m
            if np.linalg.norm(meas.position - track.x[:3]) <= reach:
                return True
        return False

    def _update(self, track: Track, meas: Measurement):
        p = self.params
        sigma = p.sigma_meas_m
        if meas.partial:
            # centroid of the visible part only: uncertain by the part that
            # is out of view, of the order of the object size
            sigma = max(sigma, p.partial_sigma_m)
        if track.size_m and meas.size_m >= p.merge_size_ratio * track.size_m:
            # A region much larger than this object is probably two objects
            # seen as one; its centroid is only known to within its extent.
            sigma = max(sigma, 0.5 * meas.size_m)
        r = np.eye(3) * sigma ** 2
        nu = meas.position - H @ track.x
        s = H @ track.P @ H.T + r
        k = track.P @ H.T @ np.linalg.inv(s)
        track.x = track.x + k @ nu
        i_kh = np.eye(6) - k @ H
        track.P = i_kh @ track.P @ i_kh.T + k @ r @ k.T      # Joseph form
        track.last_update = track.stamp
        track.hits += 1
        track.misses = 0
        track.last_measurement = meas.position.copy()
        track.last_innovation = nu
        track.last_nis = float(nu @ np.linalg.inv(s) @ nu)
        if not track.size_m:
            track.size_m = meas.size_m
        elif meas.partial:
            pass                    # a cut-off region says little about size
        elif meas.size_m < self.params.merge_size_ratio * track.size_m:
            track.size_m = 0.7 * track.size_m + 0.3 * meas.size_m
        track.score = 0.7 * track.score + 0.3 * meas.confidence
        if not meas.partial:
            track.full_hits += 1
            if track.first_position is None:
                track.first_position = meas.position.copy()
            track.displacement = max(track.displacement, float(
                np.linalg.norm(meas.position - track.first_position)))
        if (track.status != TENTATIVE or (
                track.full_hits >= p.confirm_hits
                and track.displacement >= p.confirm_min_displacement_m)):
            track.status = CONFIRMED

    def _birth(self, stamp: float, meas: Measurement):
        p = self.params
        x = np.zeros(6)
        x[:3] = meas.position
        x[5] = p.init_velocity_z
        cov = np.diag([p.sigma_meas_m ** 2] * 3 + [p.init_sigma_vel ** 2] * 3)
        self.tracks[self.next_id] = Track(
            id=self.next_id, x=x, P=cov, stamp=stamp, born=stamp,
            last_update=stamp, size_m=meas.size_m, score=meas.confidence,
            last_measurement=meas.position.copy(),
            first_position=None if meas.partial else meas.position.copy(),
            full_hits=0 if meas.partial else 1)
        self.next_id += 1

    def _prune(self):
        p = self.params
        for track_id in list(self.tracks):
            track = self.tracks[track_id]
            if track.status == TENTATIVE:
                dead = track.misses > p.tentative_max_misses
            else:
                dead = (track.misses > p.confirmed_max_misses
                        or track.time_since_update > p.max_coast_s)
            if dead or not np.all(np.isfinite(track.x)):
                self.deleted.append(self.tracks.pop(track_id))
