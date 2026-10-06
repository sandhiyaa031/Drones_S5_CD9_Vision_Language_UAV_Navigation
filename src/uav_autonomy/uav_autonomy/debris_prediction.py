"""Trajectory prediction for tracked debris (no ROS imports).

Starts from a Phase 3 track state x = [p, v] (PX4 local NED; p is the centroid
of the object's visible surface) with covariance P at the track's own
timestamp, and extrapolates it to a set of horizons:

    p(t + h) = p + v h + 0.5 a h^2          v(t + h) = v + a h

Three models differ only in the acceleration a:

    cv         a = 0
    ca         a = acceleration estimated from the track's recent velocities
    ballistic  a = (0, 0, g): known gravity along local down

Uncertainty: P(h) = F(h) P F(h)^T + Q(h), with the white-noise-acceleration Q
of the tracker and a model-specific sigma_a. For ``ca`` the variance of the
estimated acceleration is added.

Prediction only: nothing here evaluates separation, risk or avoidance.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

MODEL_CV, MODEL_CA, MODEL_BALLISTIC = 0, 1, 2
MODEL_NAMES = {MODEL_CV: 'cv', MODEL_CA: 'ca', MODEL_BALLISTIC: 'ballistic'}
MODEL_IDS = {v: k for k, v in MODEL_NAMES.items()}

HORIZONS_S = (0.1, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0)


@dataclass(frozen=True)
class PredictorParams:
    """Predictor settings; rationale in docs/debris_prediction.md."""

    horizons_s: Tuple[float, ...] = HORIZONS_S
    gravity_m_s2: float = 9.8
    # 'ballistic', 'cv', 'ca', or 'auto' (see select_model)
    model: str = 'ballistic'
    # process noise used to grow the covariance over the horizon [m/s^2]
    sigma_accel_cv: float = 10.0       # unmodelled gravity
    sigma_accel_ca: float = 3.0
    sigma_accel_ballistic: float = 3.0
    # acceleration estimate (for 'ca' and for model selection)
    accel_window_s: float = 0.40
    accel_min_samples: int = 5
    accel_min_span_s: float = 0.12
    # 'auto': the estimate must differ from gravity by more than this, with
    # at least accel_min_samples, before gravity is abandoned
    gravity_tolerance_m_s2: float = 4.0
    # validity of a prediction
    max_time_since_update_s: float = 0.35
    takeoff_plane_z: float = 0.0       # local down coordinate of takeoff


@dataclass
class TrackState:
    """What the predictor needs from one DebrisTrack."""

    id: int
    stamp: float
    x: np.ndarray            # [p, v]
    P: np.ndarray            # 6x6
    status: int
    time_since_update: float


@dataclass
class PredictedPoint:
    """State at one horizon."""

    horizon: float
    stamp: float
    x: np.ndarray
    P: np.ndarray
    below_takeoff_plane: bool


@dataclass
class Prediction:
    """Predicted trajectory of one track."""

    track_id: int
    stamp: float
    status: int
    time_since_update: float
    model: int
    acceleration: np.ndarray
    points: List[PredictedPoint]


def propagate(x: np.ndarray, p: np.ndarray, h: float, accel: np.ndarray,
              sigma_accel: float, accel_cov: Optional[np.ndarray] = None):
    """State and covariance h seconds ahead under constant acceleration."""
    f = np.eye(6)
    f[0, 3] = f[1, 4] = f[2, 5] = h
    x_new = f @ x
    x_new[:3] += 0.5 * accel * h * h
    x_new[3:] += accel * h
    q = np.zeros((6, 6))
    s2 = sigma_accel * sigma_accel
    for axis in range(3):
        q[axis, axis] = s2 * h ** 4 / 4.0
        q[axis, axis + 3] = q[axis + 3, axis] = s2 * h ** 3 / 2.0
        q[axis + 3, axis + 3] = s2 * h ** 2
    p_new = f @ p @ f.T + q
    if accel_cov is not None:
        g = np.vstack([0.5 * h * h * np.eye(3), h * np.eye(3)])   # dx/da
        p_new = p_new + g @ accel_cov @ g.T
    return x_new, p_new


def estimate_acceleration(history: Sequence[Tuple[float, np.ndarray]],
                          params: PredictorParams):
    """Least-squares slope of velocity over the recent history.

    ``history`` is a sequence of (stamp, velocity) of measurement-updated
    track states, oldest first. Returns (acceleration, covariance) or None if
    there is too little data.
    """
    if not history:
        return None
    t_end = history[-1][0]
    recent = [(t, v) for t, v in history
              if t_end - t <= params.accel_window_s]
    if len(recent) < params.accel_min_samples:
        return None
    t = np.array([r[0] for r in recent]) - t_end
    if t[-1] - t[0] < params.accel_min_span_s:
        return None
    v = np.array([r[1] for r in recent])
    a_mat = np.column_stack([t, np.ones_like(t)])
    coef, *_ = np.linalg.lstsq(a_mat, v, rcond=None)
    accel = coef[0]
    residual = v - a_mat @ coef
    dof = max(1, len(t) - 2)
    var = (residual ** 2).sum(axis=0) / dof / max(
        1e-9, ((t - t.mean()) ** 2).sum())
    return accel, np.diag(var)


def select_model(accel_estimate, params: PredictorParams):
    """Model choice for 'auto'.

    Falling debris is the expected case, so gravity is the default. It is
    abandoned for the measured acceleration only when a sufficiently long
    velocity history contradicts it.
    """
    gravity = np.array([0.0, 0.0, params.gravity_m_s2])
    if accel_estimate is None:
        return MODEL_BALLISTIC
    accel, _ = accel_estimate
    if np.linalg.norm(accel - gravity) <= params.gravity_tolerance_m_s2:
        return MODEL_BALLISTIC
    return MODEL_CA


class DebrisPredictor:
    """Keep a short velocity history per track and predict trajectories."""

    def __init__(self, params: PredictorParams = PredictorParams()):
        """Start with no history."""
        self.params = params
        self.history: Dict[int, List[Tuple[float, np.ndarray]]] = {}

    def is_valid(self, track: TrackState) -> bool:
        """Confirmed, and updated recently enough to be worth predicting."""
        return (track.status in (1, 2)          # CONFIRMED, COASTING
                and track.time_since_update
                <= self.params.max_time_since_update_s
                and bool(np.all(np.isfinite(track.x))))

    def update(self, tracks: Sequence[TrackState],
               model: Optional[str] = None) -> List[Prediction]:
        """Predict every valid track in one frame of tracks."""
        p = self.params
        alive = set()
        out = []
        for track in tracks:
            alive.add(track.id)
            hist = self.history.setdefault(track.id, [])
            if track.time_since_update == 0.0 and (
                    not hist or track.stamp > hist[-1][0]):
                hist.append((track.stamp, track.x[3:].copy()))
                while hist and track.stamp - hist[0][0] > 2.0:
                    hist.pop(0)
            if self.is_valid(track):
                out.append(self.predict(track, model or p.model))
        for track_id in list(self.history):
            if track_id not in alive:
                del self.history[track_id]      # track died: forget it
        return out

    def predict(self, track: TrackState, model: str) -> Prediction:
        """Extrapolate one track with the named model."""
        p = self.params
        estimate = estimate_acceleration(self.history.get(track.id, []), p)
        model_id = (select_model(estimate, p) if model == 'auto'
                    else MODEL_IDS[model])
        accel_cov = None
        if model_id == MODEL_BALLISTIC:
            accel = np.array([0.0, 0.0, p.gravity_m_s2])
            sigma = p.sigma_accel_ballistic
        elif model_id == MODEL_CA:
            if estimate is None:
                # not enough history yet: no acceleration can be claimed
                model_id = MODEL_CV
                accel, sigma = np.zeros(3), p.sigma_accel_cv
            else:
                accel, accel_cov = estimate
                sigma = p.sigma_accel_ca
        else:
            accel, sigma = np.zeros(3), p.sigma_accel_cv
        points = []
        for h in p.horizons_s:
            x_h, p_h = propagate(track.x, track.P, h, accel, sigma, accel_cov)
            points.append(PredictedPoint(
                horizon=h, stamp=track.stamp + h, x=x_h, P=p_h,
                below_takeoff_plane=bool(x_h[2] > p.takeoff_plane_z)))
        return Prediction(
            track_id=track.id, stamp=track.stamp, status=track.status,
            time_since_update=track.time_since_update, model=model_id,
            acceleration=accel, points=points)
