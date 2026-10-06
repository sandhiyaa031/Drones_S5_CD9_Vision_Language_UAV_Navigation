"""Unit tests for the debris tracker (pure Python, no ROS or Gazebo)."""

import copy

import numpy as np
import pytest

from uav_autonomy.debris_tracking import (
    COASTING,
    CONFIRMED,
    DebrisTracker,
    Measurement,
    TENTATIVE,
    TrackerParams,
    predict_state,
    process_noise,
    transition,
)

G = 9.8
CV = TrackerParams(gravity_m_s2=0.0, sigma_accel_z=12.0)  # constant velocity
BALLISTIC = TrackerParams()                               # default: gravity


def times(rate_hz, duration, start=10.0):
    return start + np.arange(int(duration * rate_hz)) / rate_hz


def run(tracker, stamps, trajectories, noise=0.0, seed=0, drop=()):
    """Feed positions of several objects; return per-frame track lists."""
    rng = np.random.default_rng(seed)
    out = []
    for i, t in enumerate(stamps):
        if i in drop:
            continue
        meas = []
        for traj in trajectories:
            p = traj(t)
            if p is not None:
                meas.append(Measurement(
                    np.asarray(p, float) + rng.normal(0, noise, 3), 0.4, 0.9))
        out.append((t, copy.deepcopy(tracker.step(t, meas))))
    return out


def falling(p0, v0=(0, 0, 0), t0=10.0):
    p0, v0 = np.asarray(p0, float), np.asarray(v0, float)
    return lambda t: p0 + v0 * (t - t0) + np.array(
        [0, 0, 0.5 * G * (t - t0) ** 2])


def linear(p0, v, t0=10.0, t_end=None, t_start=None):
    p0, v = np.asarray(p0, float), np.asarray(v, float)

    def f(t):
        if t_end is not None and t > t_end:
            return None
        if t_start is not None and t < t_start:
            return None
        return p0 + v * (t - t0)
    return f


def moving(t, t0=10.0):
    """One measurement moving at 3 m/s (so the track can be confirmed)."""
    return [Measurement(np.array([0.0, 0.0, -5.0 + 3.0 * (t - t0)]))]


def confirmed(tracks):
    return [tr for tr in tracks if tr.status in (CONFIRMED, COASTING)]


# --- model -----------------------------------------------------------------

def test_transition_and_process_noise_scale_with_dt():
    assert transition(0.2)[0, 3] == pytest.approx(0.2)
    q1, q2 = process_noise(0.1, CV), process_noise(0.2, CV)
    assert q2[5, 5] == pytest.approx(4 * q1[5, 5])       # dt^2
    assert q2[2, 2] == pytest.approx(16 * q1[2, 2])      # dt^4
    assert np.allclose(q1, q1.T)


def test_prediction_with_and_without_gravity():
    x = np.array([0, 0, -10.0, 1.0, 0, 2.0])
    p = np.eye(6)
    x_cv, _ = predict_state(x, p, 0.5, CV)
    assert x_cv == pytest.approx([0.5, 0, -9.0, 1.0, 0, 2.0])
    x_g, _ = predict_state(x, p, 0.5, BALLISTIC)
    assert x_g[2] == pytest.approx(-9.0 + 0.5 * G * 0.25)
    assert x_g[5] == pytest.approx(2.0 + G * 0.5)


# --- single object ---------------------------------------------------------

def test_stationary_object():
    out = run(DebrisTracker(CV), times(30, 2), [linear((1, 2, -8), (0, 0, 0))])
    tr = out[-1][1][0]
    # followed with a stable ID and zero velocity, but never promoted to a
    # confirmed debris track because it does not move
    assert tr.status == TENTATIVE and tr.id == 1
    assert {t.id for _, trs in out for t in trs} == {1}
    assert tr.x[:3] == pytest.approx([1, 2, -8], abs=1e-3)
    assert np.linalg.norm(tr.x[3:]) < 0.05


def test_constant_velocity_is_estimated():
    v = (1.5, -0.5, 6.0)
    out = run(DebrisTracker(CV), times(30, 1.5), [linear((0, 0, -12), v)])
    tr = out[-1][1][0]
    assert tr.x[3:] == pytest.approx(v, abs=0.05)
    assert tr.hits == len(out)


# The constant-velocity model lags a 9.8 m/s^2 acceleration by about 1 m/s
# with the default process noise; the gravity model does not.
@pytest.mark.parametrize('params,tolerance', [(CV, 1.3), (BALLISTIC, 0.1)])
def test_falling_object_velocity(params, tolerance):
    stamps = times(30, 1.2)
    out = run(DebrisTracker(params), stamps, [falling((1, 0, -12))])
    t, tracks = out[-1]
    true_vz = G * (t - 10.0)
    assert len(tracks) == 1
    assert tracks[0].x[5] == pytest.approx(true_vz, abs=tolerance)
    assert tracks[0].x[2] == pytest.approx(falling((1, 0, -12))(t)[2],
                                           abs=0.1)


def test_constant_velocity_model_lags_under_gravity():
    """Documents the bias that motivates the gravity term."""
    out = run(DebrisTracker(CV), times(30, 1.2), [falling((1, 0, -12))])
    errors = [tr[0].x[5] - G * (t - 10.0) for t, tr in out[10:]]
    assert max(errors) < 0          # always under-estimates the fall speed
    out_g = run(DebrisTracker(BALLISTIC), times(30, 1.2),
                [falling((1, 0, -12))])
    errors_g = [abs(tr[0].x[5] - G * (t - 10.0)) for t, tr in out_g[10:]]
    assert max(errors_g) < 0.5 * abs(np.mean(errors))


def test_lateral_motion():
    out = run(DebrisTracker(BALLISTIC), times(30, 1.0),
              [falling((3, 0, -8), v0=(-3.4, 0.5, 0))])
    tr = out[-1][1][0]
    assert tr.x[3:5] == pytest.approx([-3.4, 0.5], abs=0.1)


def test_accelerating_object_stays_tracked():
    def accel(t):
        return np.array([0.5 * 4.0 * (t - 10.0) ** 2, 0.0, -8.0])
    out = run(DebrisTracker(CV), times(30, 1.5), [accel])
    ids = {tr.id for _, tracks in out for tr in tracks}
    assert ids == {1}
    # unmodelled 4 m/s^2: the velocity estimate follows but lags
    assert 4.5 < out[-1][1][0].x[3] < 6.0


def test_noisy_measurements():
    out = run(DebrisTracker(BALLISTIC), times(30, 1.2),
              [falling((1, 1, -12))], noise=0.05, seed=3)
    assert {tr.id for _, tracks in out for tr in tracks} == {1}
    t, tracks = out[-1]
    assert tracks[0].x[5] == pytest.approx(G * (t - 10.0), abs=1.0)
    assert np.linalg.norm(
        tracks[0].x[:3] - falling((1, 1, -12))(t)) < 0.15


# --- timing ----------------------------------------------------------------

@pytest.mark.parametrize('rate', [30.0, 24.0, 10.0])
def test_velocity_is_unbiased_at_any_frame_rate(rate):
    v = (0.0, 2.0, 5.0)
    out = run(DebrisTracker(CV), times(rate, 2.0), [linear((0, 0, -15), v)])
    assert out[-1][1][0].x[3:] == pytest.approx(v, abs=0.05)


def test_irregular_timestamps_give_same_velocity():
    rng = np.random.default_rng(1)
    stamps = 10.0 + np.cumsum(rng.choice([0.033, 0.066, 0.1], size=40))
    v = (1.0, -1.0, 7.0)
    out = run(DebrisTracker(CV), stamps, [linear((0, 0, -30), v)])
    assert out[-1][1][0].x[3:] == pytest.approx(v, abs=0.05)
    assert out[-1][1][0].last_dt == pytest.approx(stamps[-1] - stamps[-2])


@pytest.mark.parametrize('params', [CV, BALLISTIC])
def test_100ms_gap_keeps_the_same_track(params):
    stamps = times(30, 1.2)
    out = run(DebrisTracker(params), stamps, [falling((1, 0, -12))],
              drop={15, 16})
    assert {tr.id for _, tracks in out for tr in tracks} == {1}
    assert out[-1][1][0].status == CONFIRMED


@pytest.mark.parametrize('params', [CV, BALLISTIC])
def test_500ms_gap_does_not_break_the_tracker(params):
    stamps = times(30, 1.6)
    drop = set(range(15, 30))                       # 0.5 s without frames
    out = run(DebrisTracker(params), stamps, [falling((1, 0, -20))],
              drop=drop)
    t, tracks = out[-1]
    assert all(np.all(np.isfinite(tr.x)) and np.all(np.isfinite(tr.P))
               for _, trs in out for tr in trs)
    live = confirmed(tracks)
    assert len(live) == 1
    assert live[0].x[5] == pytest.approx(G * (t - 10.0), abs=1.5)
    assert np.linalg.norm(live[0].x[:3] - falling((1, 0, -20))(t)) < 0.2


def test_500ms_gap_identity_with_gravity_model():
    stamps = times(30, 1.6)
    out = run(DebrisTracker(BALLISTIC), stamps, [falling((1, 0, -20))],
              drop=set(range(15, 30)))
    assert {tr.id for _, tracks in out for tr in tracks} == {1}


def test_out_of_order_and_duplicate_frames_are_dropped():
    tracker = DebrisTracker(CV)
    m = [Measurement(np.array([0.0, 0.0, -5.0]))]
    assert tracker.step(10.0, m) is not None
    assert tracker.step(10.1, m) is not None
    assert tracker.step(10.1, m) is None            # duplicate stamp
    assert tracker.step(10.05, m) is None           # older than last
    assert tracker.dropped_frames == {'out_of_order': 1, 'duplicate': 1}
    assert tracker.tracks[1].hits == 2              # unaffected


def test_stale_gap_resets_tracks():
    tracker = DebrisTracker(CV)
    for t in (10.0, 10.03, 10.06, 10.1):
        tracker.step(t, moving(t))
    assert tracker.tracks[1].status == CONFIRMED
    tracks = tracker.step(15.0, moving(15.0))       # 4.9 s later
    assert tracker.resets == 1
    assert [tr.id for tr in tracks] == [2]


# --- birth, death, multiple objects ----------------------------------------

def test_birth_needs_three_hits():
    tracker = DebrisTracker(CV)
    assert tracker.step(10.00, moving(10.00))[0].status == TENTATIVE
    assert tracker.step(10.03, moving(10.03))[0].status == TENTATIVE
    assert tracker.step(10.06, moving(10.06))[0].status == CONFIRMED


def test_static_structure_is_never_confirmed():
    tracker = DebrisTracker(BALLISTIC)
    m = [Measurement(np.array([1.0, 0.1, -2.6]))]
    statuses = set()
    for i in range(30):                              # 1 s of detections
        jitter = np.array([0.01 * (i % 2), 0.0, 0.01 * (i % 3)])
        tracks = tracker.step(10.0 + i / 30.0,
                              [Measurement(m[0].position + jitter)])
        statuses |= {tr.status for tr in tracks}
    assert statuses == {TENTATIVE}
    for i in range(30, 40):                          # detector drops it
        tracks = tracker.step(10.0 + i / 30.0, [])
    assert tracks == []


def test_object_released_from_rest_is_confirmed_quickly():
    out = run(DebrisTracker(BALLISTIC), times(30, 0.5),
              [falling((1, 0, -12))])
    first = next(t for t, trs in out if confirmed(trs))
    assert first - 10.0 <= 0.2


def test_single_false_detection_never_confirms():
    tracker = DebrisTracker(CV)
    tracker.step(10.0, [Measurement(np.array([0.0, 0.0, -5.0]))])
    for i in range(1, 6):
        tracks = tracker.step(10.0 + 0.033 * i, [])
    assert tracks == []


def test_death_after_object_disappears():
    stamps = times(30, 2.0)
    out = run(DebrisTracker(CV), stamps,
              [linear((0, 0, -9), (0, 0, 3), t_end=10.5)])
    statuses = [[tr.status for tr in tracks] for _, tracks in out]
    assert [CONFIRMED] in statuses and [COASTING] in statuses
    assert statuses[-1] == []
    gone_at = next(t for t, tracks in out if t > 10.5 and not tracks)
    assert gone_at - 10.5 < 0.5


def test_new_object_entering_gets_new_id():
    out = run(DebrisTracker(CV), times(30, 1.5), [
        linear((0, 0, -9), (0, 0, 3)),
        linear((3, 3, -12), (0, 0, 5), t_start=10.7)])
    ids = sorted({tr.id for tr in out[-1][1]})
    assert ids == [1, 2]
    first_second = next(t for t, trs in out if len(trs) == 2)
    assert first_second == pytest.approx(10.7, abs=0.04)


@pytest.mark.parametrize('n', [2, 3])
def test_multiple_objects_keep_their_ids(n):
    trajs = [falling((1.0 * i, 0.8 * i, -14 - i)) for i in range(n)]
    out = run(DebrisTracker(BALLISTIC), times(30, 1.2), trajs, noise=0.02)
    t, tracks = out[-1]
    assert len(confirmed(tracks)) == n
    for i, traj in enumerate(trajs):
        nearest = min(tracks, key=lambda tr: np.linalg.norm(
            tr.x[:3] - traj(t)))
        assert nearest.id == i + 1
    assert {tr.id for _, trs in out for tr in trs} == set(range(1, n + 1))


def test_crossing_trajectories_do_not_swap_ids():
    a = linear((-2, 0, -8), (4, 0, 0))
    b = linear((2, 0.25, -8), (-4, 0, 0))          # pass 0.25 m apart
    out = run(DebrisTracker(CV), times(30, 1.0), [a, b], noise=0.01)
    t, tracks = out[-1]
    by_id = {tr.id: tr for tr in tracks}
    assert np.linalg.norm(by_id[1].x[:3] - a(t)) < 0.15
    assert np.linalg.norm(by_id[2].x[:3] - b(t)) < 0.15
    assert by_id[1].x[3] > 3 and by_id[2].x[3] < -3


def test_crossing_falling_objects_do_not_swap_ids():
    a = falling((-1.5, 0, -14), v0=(3, 0, 0))
    b = falling((1.5, 0.25, -14), v0=(-3, 0, 0))
    out = run(DebrisTracker(BALLISTIC), times(30, 1.0), [a, b], noise=0.01)
    t, tracks = out[-1]
    by_id = {tr.id: tr for tr in tracks}
    assert set(by_id) == {1, 2}
    assert np.linalg.norm(by_id[1].x[:3] - a(t)) < 0.15
    assert np.linalg.norm(by_id[2].x[:3] - b(t)) < 0.15


def test_temporary_overlap_merged_detection():
    """Two objects merge into one detection for 4 frames, then separate."""
    a = linear((-1, 0, -8), (2, 0, 4))
    b = linear((1, 0, -8), (-2, 0, 4))
    tracker = DebrisTracker(CV)
    out = []
    for i, t in enumerate(times(30, 1.2)):
        pa, pb = a(t), b(t)
        if abs(pa[0] - pb[0]) < 0.3:               # overlapping in the image
            meas = [Measurement((pa + pb) / 2)]
        else:
            meas = [Measurement(pa), Measurement(pb)]
        out.append((t, copy.deepcopy(tracker.step(t, meas))))
    t, tracks = out[-1]
    live = confirmed(tracks)
    assert len(live) == 2
    assert {tr.id for tr in live} == {1, 2}
    by_id = {tr.id: tr for tr in live}
    assert by_id[1].x[3] > 1 and by_id[2].x[3] < -1   # identities kept


def test_missed_frames_for_one_of_two_objects():
    a = falling((0, 0, -14))
    seen = {'n': 0}

    def b(t):
        seen['n'] += 1
        return None if 12 <= seen['n'] <= 14 else falling((2, 1, -15))(t)
    out = run(DebrisTracker(BALLISTIC), times(30, 1.2), [a, b])
    assert {tr.id for _, trs in out for tr in trs} == {1, 2}
    assert len(confirmed(out[-1][1])) == 2


def test_covariance_grows_while_coasting_and_shrinks_on_update():
    tracker = DebrisTracker(CV)
    for t in (10.0, 10.03, 10.06, 10.1):
        tracker.step(t, moving(t))
    before = np.trace(tracker.tracks[1].P)
    tracker.step(10.2, [])
    coasting = np.trace(tracker.tracks[1].P)
    tracker.step(10.23, moving(10.23))
    after = np.trace(tracker.tracks[1].P)
    assert coasting > before and after < coasting
    assert tracker.tracks[1].time_since_update == 0.0


def test_innovation_and_dt_are_logged():
    tracker = DebrisTracker(CV)
    tracker.step(10.0, [Measurement(np.array([0.0, 0.0, -5.0]))])
    tracks = tracker.step(10.05, [Measurement(np.array([0.1, 0.0, -5.0]))])
    assert tracks[0].last_dt == pytest.approx(0.05)
    assert tracks[0].last_innovation == pytest.approx([0.1, 0, 0])
    assert tracks[0].last_nis is not None


def test_confidence_falls_while_coasting():
    tracker = DebrisTracker(CV)
    m = [Measurement(np.array([0.0, 0.0, -5.0]), 0.4, 0.9)]
    for i in range(8):
        tracker.step(10.0 + 0.033 * i, m)
    fresh = tracker.tracks[1].confidence
    tracker.step(10.5, [])
    assert tracker.tracks[1].confidence < 0.5 * fresh


def test_default_model_includes_gravity():
    assert TrackerParams().gravity_m_s2 == pytest.approx(9.8)


def test_merged_detection_keeps_both_tracks_alive():
    """Two adjacent objects reported as one large region for 0.4 s."""
    a = falling((0.0, 0.0, -14))
    b = falling((0.45, 0.0, -14))
    tracker = DebrisTracker(BALLISTIC)
    stamps = times(30, 1.2)
    for i, t in enumerate(stamps):
        pa, pb = a(t), b(t)
        if 10 <= i < 22:                       # merged: one 0.85 m region
            meas = [Measurement((pa + pb) / 2, 0.85, 0.9)]
        else:
            meas = [Measurement(pa, 0.4, 0.9), Measurement(pb, 0.4, 0.9)]
        tracks = copy.deepcopy(tracker.step(t, meas))
    live = confirmed(tracks)
    assert sorted(tr.id for tr in live) == [1, 2]
    assert tracker.next_id == 3                # no extra track was created
    by_id = {tr.id: tr for tr in live}
    assert np.linalg.norm(by_id[1].x[:3] - a(t)) < 0.3
    assert np.linalg.norm(by_id[2].x[:3] - b(t)) < 0.3


def test_partial_views_do_not_confirm_a_track():
    """A region on the image border (e.g. a roof edge) with a moving centroid."""
    tracker = DebrisTracker(BALLISTIC)
    statuses = set()
    for i in range(20):
        pos = np.array([1.0 - 0.1 * i, 0.1, -2.6])     # centroid slides
        tracks = tracker.step(10.0 + i / 30.0,
                              [Measurement(pos, 0.8, 0.8, partial=True)])
        statuses |= {tr.status for tr in tracks}
    assert statuses == {TENTATIVE}


def test_object_entering_from_the_side_confirms_once_fully_visible():
    traj = falling((3.0, 0, -8), v0=(-3.0, 0, 0))
    tracker = DebrisTracker(BALLISTIC)
    first_confirmed = None
    for i, t in enumerate(times(30, 0.8)):
        partial = i < 6                                 # cut by the border
        tracks = tracker.step(t, [Measurement(traj(t), 0.4, 0.8, partial)])
        if first_confirmed is None and confirmed(tracks):
            first_confirmed = i
    assert first_confirmed is not None and 7 <= first_confirmed <= 9
    assert {tr.id for tr in tracks} == {1}             # same track all along


def test_partial_measurement_is_trusted_less():
    def final_x(partial):
        tracker = DebrisTracker(BALLISTIC)
        traj = falling((0.0, 0, -12))
        for i, t in enumerate(times(30, 0.5)):
            tracker.step(t, [Measurement(traj(t), 0.4, 0.9)])
        t += 1 / 30.0
        biased = traj(t) + np.array([0.3, 0, 0])
        tracker.step(t, [Measurement(biased, 0.4, 0.9, partial)])
        return tracker.tracks[1].x[0]
    assert final_x(True) < 0.5 * final_x(False)


def test_region_touches_border_from_sensor_box():
    from types import SimpleNamespace as NS
    from uav_autonomy.debris_tracker import region_touches_border
    k = (268.5, 268.5, 320.0, 240.0, 640, 480)
    centred = region_touches_border(NS(x=5.0, y=0.0, z=0.0),
                                    NS(x=0.0, y=0.4, z=0.4), k)
    # 5 m up, 5.9 m to the left: the box reaches the left image edge
    at_edge = region_touches_border(NS(x=5.0, y=5.9, z=0.0),
                                    NS(x=0.0, y=0.4, z=0.4), k)
    assert not centred and at_edge
