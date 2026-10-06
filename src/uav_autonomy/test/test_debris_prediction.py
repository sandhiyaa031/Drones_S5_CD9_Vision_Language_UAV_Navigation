"""Unit tests for debris trajectory prediction (no ROS or Gazebo)."""

import numpy as np
import pytest

from uav_autonomy.debris_prediction import (
    DebrisPredictor,
    HORIZONS_S,
    MODEL_BALLISTIC,
    MODEL_CA,
    MODEL_CV,
    PredictorParams,
    TrackState,
    estimate_acceleration,
    propagate,
    select_model,
)
from uav_autonomy.debris_tracking import (
    COASTING,
    CONFIRMED,
    DebrisTracker,
    Measurement,
    TENTATIVE,
    TrackerParams,
)

G = 9.8


def state(track_id=1, stamp=10.0, p=(0, 0, -10), v=(0, 0, 0),
          status=CONFIRMED, tsu=0.0, sigma=0.05):
    return TrackState(id=track_id, stamp=stamp,
                      x=np.array(list(p) + list(v), dtype=float),
                      P=np.eye(6) * sigma ** 2, status=status,
                      time_since_update=tsu)


def by_h(prediction):
    return {pt.horizon: pt for pt in prediction.points}


def test_all_requested_horizons_are_produced():
    pred = DebrisPredictor().update([state()])[0]
    assert tuple(pt.horizon for pt in pred.points) == HORIZONS_S
    assert HORIZONS_S == (0.1, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0)


def test_stationary_object_cv_stays_put():
    pred = DebrisPredictor(PredictorParams(model='cv')).update(
        [state(p=(1, 2, -8))])[0]
    assert pred.model == MODEL_CV
    for pt in pred.points:
        assert pt.x[:3] == pytest.approx([1, 2, -8])
        assert pt.x[3:] == pytest.approx([0, 0, 0])


def test_constant_velocity():
    pred = DebrisPredictor(PredictorParams(model='cv')).update(
        [state(p=(0, 0, -10), v=(1.0, -2.0, 3.0))])[0]
    pt = by_h(pred)[2.0]
    assert pt.x[:3] == pytest.approx([2.0, -4.0, -4.0])
    assert pt.x[3:] == pytest.approx([1.0, -2.0, 3.0])


def test_ballistic_fall_matches_closed_form():
    pred = DebrisPredictor().update(
        [state(p=(0, 0, -20), v=(0, 0, 2.0))])[0]
    assert pred.model == MODEL_BALLISTIC
    assert pred.acceleration == pytest.approx([0, 0, G])
    for pt in pred.points:
        h = pt.horizon
        assert pt.x[2] == pytest.approx(-20 + 2.0 * h + 0.5 * G * h * h)
        assert pt.x[5] == pytest.approx(2.0 + G * h)
        assert pt.x[:2] == pytest.approx([0, 0])


def test_lateral_throw():
    pred = DebrisPredictor().update(
        [state(p=(3, 0, -8), v=(-3.4, 0.5, 1.0))])[0]
    pt = by_h(pred)[1.0]
    assert pt.x[:2] == pytest.approx([3 - 3.4, 0.5])
    assert pt.x[3:5] == pytest.approx([-3.4, 0.5])     # unchanged laterally
    assert pt.x[2] == pytest.approx(-8 + 1.0 + 0.5 * G)


def test_high_speed_object():
    pred = DebrisPredictor().update([state(p=(0, 0, -30), v=(5, 0, 20))])[0]
    pt = by_h(pred)[0.5]
    assert pt.x[:3] == pytest.approx([2.5, 0, -30 + 10 + 0.5 * G * 0.25])
    assert np.all(np.isfinite(pt.P))


def test_coordinate_frame_down_is_positive_z():
    """PX4 local NED: a falling object moves towards +z."""
    pred = DebrisPredictor().update([state(p=(0, 0, -10))])[0]
    zs = [pt.x[2] for pt in pred.points]
    assert all(b > a for a, b in zip(zs, zs[1:]))
    assert not by_h(pred)[1.0].below_takeoff_plane      # z = -5.1
    assert by_h(pred)[2.0].below_takeoff_plane          # z = +9.6 > 0


def test_timestamps_are_state_time_plus_horizon():
    pred = DebrisPredictor().update([state(stamp=123.456)])[0]
    assert pred.stamp == 123.456
    for pt in pred.points:
        assert pt.stamp == pytest.approx(123.456 + pt.horizon)


def test_uncertainty_grows_with_horizon():
    pred = DebrisPredictor().update([state()])[0]
    traces = [np.trace(pt.P[:3, :3]) for pt in pred.points]
    assert all(b > a for a, b in zip(traces, traces[1:]))
    assert all(np.allclose(pt.P, pt.P.T) for pt in pred.points)


def test_multiple_objects_keep_their_ids():
    preds = DebrisPredictor().update([
        state(1, p=(0, 0, -10)), state(2, p=(1, 1, -12), v=(1, 0, 0)),
        state(3, p=(-1, 2, -9), v=(0, -1, 4))])
    assert [p.track_id for p in preds] == [1, 2, 3]
    assert by_h(preds[1])[1.0].x[0] == pytest.approx(2.0)


def test_tentative_tracks_get_no_prediction():
    assert DebrisPredictor().update([state(status=TENTATIVE)]) == []


def test_stale_track_expires():
    predictor = DebrisPredictor()
    fresh = state(status=COASTING, tsu=0.2)
    stale = state(status=COASTING, tsu=0.5)
    assert len(predictor.update([fresh])) == 1
    assert predictor.update([stale]) == []


def test_dead_track_is_forgotten():
    predictor = DebrisPredictor()
    predictor.update([state(1), state(2)])
    assert set(predictor.history) == {1, 2}
    assert [p.track_id for p in predictor.update([state(2, stamp=10.1)])] \
        == [2]
    assert set(predictor.history) == {2}
    assert predictor.update([]) == []
    assert predictor.history == {}


def test_non_finite_state_is_not_predicted():
    bad = state()
    bad.x[0] = np.nan
    assert DebrisPredictor().update([bad]) == []


def test_acceleration_estimate_from_velocity_history():
    params = PredictorParams()
    hist = [(10.0 + 0.033 * i, np.array([0.0, 0.5 * 0.033 * i, G * 0.033 * i]))
            for i in range(10)]
    accel, cov = estimate_acceleration(hist, params)
    assert accel == pytest.approx([0.0, 0.5, G], abs=1e-6)
    assert estimate_acceleration(hist[:3], params) is None      # too few
    close = [(10.0 + 0.001 * i, np.zeros(3)) for i in range(10)]
    assert estimate_acceleration(close, params) is None         # too short


def test_constant_acceleration_model_uses_estimated_acceleration():
    predictor = DebrisPredictor(PredictorParams(model='ca'))
    a_true = np.array([2.0, 0.0, 4.0])
    for i in range(10):
        t = 10.0 + 0.033 * i
        dt = t - 10.0
        pred = predictor.update([state(
            stamp=t, p=0.5 * a_true * dt * dt + [0, 0, -10], v=a_true * dt)])
    assert pred[0].model == MODEL_CA
    assert pred[0].acceleration == pytest.approx(a_true, abs=1e-6)
    pt = by_h(pred[0])[1.0]
    assert pt.x[3:] == pytest.approx(a_true * (dt + 1.0), abs=1e-5)


def test_constant_acceleration_without_history_falls_back_to_cv():
    pred = DebrisPredictor(PredictorParams(model='ca')).update(
        [state(v=(1, 0, 0))])[0]
    assert pred.model == MODEL_CV
    assert pred.acceleration == pytest.approx([0, 0, 0])


def test_auto_selects_gravity_for_falling_and_ca_for_supported_object():
    params = PredictorParams(model='auto')
    falling = [(10 + 0.033 * i, np.array([0, 0, G * 0.033 * i]))
               for i in range(10)]
    sliding = [(10 + 0.033 * i, np.array([1.0, 0, 0.0])) for i in range(10)]
    assert select_model(estimate_acceleration(falling, params),
                        params) == MODEL_BALLISTIC
    assert select_model(estimate_acceleration(sliding, params),
                        params) == MODEL_CA
    assert select_model(None, params) == MODEL_BALLISTIC


def test_propagate_adds_acceleration_uncertainty():
    x, p = np.zeros(6), np.eye(6) * 0.01
    _, p_plain = propagate(x, p, 1.0, np.zeros(3), 3.0)
    _, p_extra = propagate(x, p, 1.0, np.zeros(3), 3.0, np.eye(3) * 4.0)
    assert np.trace(p_extra) > np.trace(p_plain)


# --- end to end with the tracker, irregular timing and gaps -----------------

def tracked_fall(stamps, drop=(), p0=(1.0, 0.5, -25.0), v0=(0.5, 0.0, 0.0)):
    """Feed a falling object through the real tracker, then predict."""
    tracker = DebrisTracker(TrackerParams())
    predictor = DebrisPredictor()
    p0, v0 = np.array(p0), np.array(v0)

    def truth(t):
        dt = t - stamps[0]
        return p0 + v0 * dt + np.array([0, 0, 0.5 * G * dt * dt])
    last = None
    for i, t in enumerate(stamps):
        if i in drop:
            continue
        tracks = tracker.step(t, [Measurement(truth(t), 0.4, 0.9)])
        states = [TrackState(tr.id, tr.stamp, tr.x.copy(), tr.P.copy(),
                             tr.status, tr.time_since_update)
                  for tr in tracks]
        preds = predictor.update(states)
        if preds:
            last = (t, preds[0])
    return last, truth


@pytest.mark.parametrize('drop', [(), {12, 13, 14}, set(range(12, 27))],
                         ids=['no gap', '100 ms gap', '500 ms gap'])
def test_prediction_after_gaps(drop):
    stamps = 10.0 + np.arange(36) / 30.0
    (t, pred), truth = tracked_fall(stamps, drop)
    assert pred.track_id == 1                       # same track throughout
    for pt in pred.points:
        assert np.linalg.norm(pt.x[:3] - truth(t + pt.horizon)) < 0.15
        assert np.all(np.isfinite(pt.P))


def test_prediction_with_irregular_timestamps():
    rng = np.random.default_rng(2)
    stamps = 10.0 + np.cumsum(rng.choice([0.033, 0.066, 0.1], size=20))
    (t, pred), truth = tracked_fall(stamps)
    assert pred.stamp == pytest.approx(stamps[-1])
    for pt in pred.points:
        assert np.linalg.norm(pt.x[:3] - truth(t + pt.horizon)) < 0.15


def test_cv_prediction_of_falling_object_is_biased_ballistic_is_not():
    stamps = 10.0 + np.arange(30) / 30.0
    (t, pred), truth = tracked_fall(stamps)
    ballistic_error = abs(by_h(pred)[1.0].x[2] - truth(t + 1.0)[2])
    cv = DebrisPredictor(PredictorParams(model='cv')).predict(
        TrackState(1, t, np.concatenate([truth(t), [0.5, 0, G * (t - 10)]]),
                   np.eye(6) * 0.01, CONFIRMED, 0.0), 'cv')
    cv_error = abs(by_h(cv)[1.0].x[2] - truth(t + 1.0)[2])
    assert cv_error == pytest.approx(0.5 * G, abs=0.01)   # 4.9 m short
    assert ballistic_error < 0.1
