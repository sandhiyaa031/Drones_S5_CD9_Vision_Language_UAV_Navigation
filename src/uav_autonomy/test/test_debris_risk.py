"""Unit tests for debris collision-risk estimation (no ROS or Gazebo)."""

import numpy as np
import pytest

from uav_autonomy.debris_prediction import (
    DebrisPredictor, HORIZONS_S, TrackState)
from uav_autonomy.debris_risk import (
    CRITICAL,
    Knot,
    POTENTIAL_THREAT,
    RiskParams,
    SAFE,
    UavStateBuffer,
    WARNING,
    WATCH,
    assess,
    ballistic_knots,
    classify,
    highest_state,
    linear_closest_approach,
    radii,
    risk_score,
    sampled_closest_approach,
)

G = 9.8
P = RiskParams()
UAV = np.array([0.0, 0.0, -3.0])          # hovering 3 m above takeoff (NED)
STILL = np.zeros(3)


def knots_for(p, v, sigma=0.05, status=1):
    """Predicted path from the real Phase 4 predictor."""
    track = TrackState(1, 100.0, np.array(list(p) + list(v), float),
                       np.eye(6) * sigma ** 2, status, 0.0)
    pred = DebrisPredictor().update([track])[0]
    out = [Knot(0.0, track.x[:3], track.x[3:], track.P[:3, :3])]
    out += [Knot(pt.horizon, pt.x[:3], pt.x[3:], pt.P[:3, :3])
            for pt in pred.points]
    return out


def falling_past(offset_x, height_above=8.0, v=(0, 0, 0)):
    """Object released `height_above` the UAV, `offset_x` to the north."""
    return knots_for((offset_x, 0.0, UAV[2] - height_above), v)


def run(knots, uav_p=UAV, uav_v=STILL, size=0.4, **kw):
    return assess(1, 100.0, 1, knots, uav_p, uav_v, size, P, **kw)


# --- geometry ----------------------------------------------------------------

def test_radii_are_sum_of_documented_terms():
    contact, safety = radii(0.4, P)
    assert contact == pytest.approx(0.40 + 0.71 * 0.4)
    assert safety == pytest.approx(contact + 0.30)
    assert radii(0.0, P) == radii(0.4, P)          # default size when unknown
    assert radii(0.8, P)[0] > contact              # bigger debris, bigger radius


def test_linear_closed_form():
    t, d = linear_closest_approach(np.array([10.0, 2.0, 0.0]),
                                   np.array([-5.0, 0.0, 0.0]), 5.0)
    assert (t, d) == pytest.approx((2.0, 2.0))
    t, d = linear_closest_approach(np.array([10.0, 2.0, 0.0]),
                                   np.array([-5.0, 0.0, 0.0]), 1.0)
    assert t == 1.0 and d == pytest.approx(np.hypot(5.0, 2.0))   # clamped
    t, d = linear_closest_approach(np.array([1.0, 0, 0]),
                                   np.array([3.0, 0, 0]), 2.0)
    assert (t, d) == pytest.approx((0.0, 1.0))                   # receding
    assert linear_closest_approach(np.array([1.0, 2, 2]), np.zeros(3),
                                   2.0) == pytest.approx((0.0, 3.0))


# --- stationary UAV ------------------------------------------------------------

def test_falling_debris_safe_pass():
    r = run(falling_past(2.5))
    assert r.state == SAFE
    assert r.d_min == pytest.approx(2.5, abs=0.02)
    assert not r.intersects and not r.predicted_collision
    # 8 m above, from rest: reaches the UAV's height after sqrt(2*8/g)
    assert r.tca == pytest.approx(np.sqrt(16 / G), abs=0.02)
    assert r.relative_speed == pytest.approx(G * r.tca, abs=0.2)


def test_direct_hit_is_critical():
    r = run(falling_past(0.1))
    assert r.state == CRITICAL and r.predicted_collision and r.intersects
    assert r.d_min == pytest.approx(0.1, abs=0.02)


def test_near_miss_inside_safety_volume_is_warning_then_critical():
    contact, safety = radii(0.4, P)
    offset = 0.5 * (contact + safety)              # between the two radii
    far = run(falling_past(offset, height_above=12.0))
    assert far.state == WARNING and far.tca > P.reaction_time_s
    assert far.intersects and not far.predicted_collision
    near = run(falling_past(offset, height_above=1.0))
    assert near.state == CRITICAL and near.tca <= P.reaction_time_s


def test_exact_threshold():
    contact, safety = radii(0.4, P)
    assert classify(safety, safety, 1.5, contact, safety, P) == SAFE
    assert classify(safety - 1e-6, safety - 1e-6, 1.5, contact, safety,
                    P) == WARNING
    assert classify(contact, contact, 1.5, contact, safety, P) == WARNING
    assert classify(contact - 1e-6, 0, 1.5, contact, safety, P) == CRITICAL
    assert classify(safety - 0.01, 0, P.reaction_time_s, contact, safety,
                    P) == CRITICAL
    assert classify(safety - 0.01, 0, P.reaction_time_s + 0.01, contact,
                    safety, P) == WARNING


def test_inside_safety_volume_now():
    r = run(knots_for((0.3, 0.0, -3.2), (0, 0, 6.0)))
    assert r.state == CRITICAL and r.current_separation < r.safety_radius
    assert r.tca <= 0.05


def test_uncertainty_overlap_gives_watch():
    contact, safety = radii(0.4, P)
    offset = safety + 0.25
    confident = run(falling_past(offset))
    assert confident.conservative_separation < confident.d_min
    # the margin at ~1.3 s is larger than 0.25 m, and capped by the
    # measured error envelope rather than the predictor's wide covariance
    assert confident.state == WATCH
    cap = P.uncertainty_cap_base_m + P.uncertainty_cap_rate_m_s * confident.tca
    assert 0.25 < confident.uncertainty_margin <= cap + 1e-9
    tight = assess(1, 100.0, 1,
                   [Knot(k.h, k.p, k.v, np.eye(3) * 1e-6)
                    for k in falling_past(offset)], UAV, STILL, 0.4, P)
    assert tight.state == SAFE
    assert tight.d_min == pytest.approx(confident.d_min)


def test_sampled_result_differs_from_linear_for_falling_debris():
    """Constant relative velocity is wrong for an accelerating object."""
    r = run(falling_past(0.1, height_above=10.0))
    assert r.d_min == pytest.approx(0.1, abs=0.02)        # sampled: a hit
    assert r.linear_d_min > 5.0                           # linear: far away
    assert r.state == CRITICAL


def test_sampling_finds_minimum_between_predictor_horizons():
    # closest approach at ~0.61 s, between the 0.5 s and 0.75 s states
    r = run(falling_past(0.1, height_above=1.8))
    assert 0.5 < r.tca < 0.75
    assert r.d_min < min(r.separation_at_horizons) - 0.3


# --- closest approach relative to the horizon -----------------------------------

def test_closest_approach_before_horizon():
    r = run(falling_past(1.0, height_above=5.0))
    assert 0 < r.tca < r.horizon
    assert r.horizon == pytest.approx(2.0)


def test_closest_approach_at_and_beyond_horizon():
    # 30 m above from rest: 2 s of fall covers 19.6 m, still 10.4 m above
    r = run(falling_past(0.0, height_above=30.0))
    assert r.tca == pytest.approx(2.0)                    # end of the horizon
    assert r.d_min == pytest.approx(30.0 - 0.5 * G * 4.0, abs=0.05)
    assert r.state == SAFE                                # nothing extrapolated
    assert not r.predicted_collision


def test_horizons_and_separations_are_reported():
    r = run(falling_past(2.0))
    assert tuple(r.horizons) == HORIZONS_S
    assert len(r.separation_at_horizons) == len(HORIZONS_S)
    assert r.separation_at_horizons[0] == pytest.approx(
        np.hypot(2.0, 8.0 - 0.5 * G * 0.01), abs=0.01)


# --- relative motion ---------------------------------------------------------

def test_moving_uav_changes_the_outcome():
    """Debris falling 3 m north: safe for a hover, a hit if the UAV flies in."""
    knots = falling_past(3.0, height_above=8.0)
    hover = run(knots)
    assert hover.state == SAFE
    t_fall = np.sqrt(16 / G)
    flying = run(knots, uav_v=np.array([3.0 / t_fall, 0.0, 0.0]))
    assert flying.state == CRITICAL and flying.d_min < 0.2
    assert flying.uav_closest_point[0] == pytest.approx(3.0, abs=0.1)


def test_uav_moving_away_makes_a_hit_safe():
    knots = falling_past(0.0, height_above=8.0)
    assert run(knots).state == CRITICAL
    assert run(knots, uav_v=np.array([4.0, 0.0, 0.0])).state == SAFE


def test_climbing_uav_meets_debris_sooner():
    knots = falling_past(0.1, height_above=8.0)
    hover = run(knots)
    climb = run(knots, uav_v=np.array([0.0, 0.0, -2.0]))     # up in NED
    assert climb.tca < hover.tca
    assert climb.relative_speed > hover.relative_speed
    assert climb.state == CRITICAL


def test_high_and_low_relative_velocity():
    fast = run(knots_for((0.1, 0, -9.0), (0, 0, 15.0)))
    assert fast.state == CRITICAL and fast.relative_speed > 15.0
    assert fast.tca < 0.5
    slow = run(knots_for((3.0, 0.0, -3.0), (-0.3, 0.0, -G * 1.0)))
    assert np.isfinite(slow.d_min) and slow.relative_speed >= 0.0
    static = assess(1, 100.0, 1, [Knot(0.0, np.array([5.0, 0, -3.0]),
                                       np.zeros(3))], UAV, STILL, 0.4, P)
    assert static.d_min == pytest.approx(5.0) and static.tca == 0.0


def test_coordinate_frame_ned():
    """North/east offsets and 'above' (negative down) are handled as NED."""
    above = run(knots_for((0.0, 0.0, -11.0), (0, 0, 0)))
    assert above.closest_point[2] == pytest.approx(UAV[2], abs=0.1)
    east = run(knots_for((0.0, 2.0, -11.0), (0, 0, 0)))
    assert east.d_min == pytest.approx(2.0, abs=0.02)
    assert east.closest_point[1] == pytest.approx(2.0)
    below = run(knots_for((0.0, 0.0, -1.0), (0, 0, 2.0)))    # under the UAV
    assert below.tca == 0.0 and below.d_min == pytest.approx(2.0)


def test_ellipsoid_scale_extends_the_volume_vertically():
    tall = RiskParams(safety_axes_scale=(1.0, 1.0, 0.5))
    knot = [Knot(0.0, np.array([0.0, 0.0, -4.5]), np.zeros(3))]
    sphere = assess(1, 0.0, 1, knot, UAV, STILL, 0.4, P)
    ellipsoid = assess(1, 0.0, 1, knot, UAV, STILL, 0.4, tall)
    assert sphere.d_min == pytest.approx(1.5)
    assert ellipsoid.d_min == pytest.approx(3.0)             # scaled distance
    lateral = [Knot(0.0, np.array([1.5, 0.0, -3.0]), np.zeros(3))]
    assert assess(1, 0.0, 1, lateral, UAV, STILL, 0.4,
                  tall).d_min == pytest.approx(1.5)


# --- several objects -----------------------------------------------------------

def test_multiple_debris_keep_ids_and_highest_state():
    results = [
        assess(7, 100.0, 1, falling_past(3.0), UAV, STILL, 0.4, P),
        assess(8, 100.0, 1, falling_past(0.1), UAV, STILL, 0.4, P),
        assess(9, 100.0, 1, falling_past(0.9, 12.0), UAV, STILL, 0.4, P)]
    assert [r.track_id for r in results] == [7, 8, 9]
    assert [r.state for r in results] == [SAFE, CRITICAL, WARNING]
    assert highest_state(results) == CRITICAL
    assert highest_state(results[:1]) == SAFE


# --- tentative tracks ------------------------------------------------------------

def test_tentative_fast_object_towards_uav_is_potential_threat():
    knots = ballistic_knots((0.2, 0.0, -9.0), (0.0, 0.0, 6.0),
                            np.eye(6) * 0.3 ** 2, P)
    r = assess(3, 100.0, 0, knots, UAV, STILL, 0.4, P, tentative=True)
    assert r.state == POTENTIAL_THREAT
    assert r.intersects
    assert highest_state([r]) == SAFE          # never raises the confirmed level


def test_tentative_object_missing_the_uav_is_ignored():
    knots = ballistic_knots((4.0, 0.0, -9.0), (0.0, 0.0, 6.0), None, P)
    assert assess(3, 100.0, 0, knots, UAV, STILL, 0.4, P,
                  tentative=True).state == SAFE


def test_tentative_slow_object_is_not_a_threat():
    """Structure seen for a moment near the UAV has no real velocity."""
    knots = ballistic_knots((0.9, 0.1, -3.4), (0.1, 0.0, 0.2), None, P)
    r = assess(3, 100.0, 0, knots, UAV, STILL, 0.4, P, tentative=True)
    assert r.intersects and r.state == SAFE


def test_tentative_never_becomes_warning_or_critical():
    knots = ballistic_knots((0.0, 0.0, -4.0), (0.0, 0.0, 9.0), None, P)
    r = assess(3, 100.0, 0, knots, UAV, STILL, 0.4, P, tentative=True)
    assert r.predicted_collision and r.state == POTENTIAL_THREAT


def test_ballistic_knots_follow_gravity():
    knots = ballistic_knots((0, 0, -10.0), (1.0, 0, 0), None, P)
    last = knots[-1]
    assert last.h == 2.0
    assert last.p == pytest.approx([2.0, 0.0, -10.0 + 0.5 * G * 4.0])
    assert last.v == pytest.approx([1.0, 0.0, G * 2.0])


# --- score, time, UAV state buffer ------------------------------------------------

def test_risk_score_orders_situations():
    hit = run(falling_past(0.1, 3.0)).score
    near = run(falling_past(1.1, 12.0)).score
    far = run(falling_past(4.0)).score
    assert 0.0 <= far < near < hit <= 1.0
    contact, safety = radii(0.4, P)
    assert risk_score(contact, 0.0, contact, safety, 2.0, P) == 1.0
    assert risk_score(safety + 5, 0.0, contact, safety, 2.0, P) == 0.0


def test_timestamp_is_passed_through_and_tca_is_relative():
    r = assess(1, 123.456, 1, falling_past(0.1), UAV, STILL, 0.4, P)
    assert r.stamp == 123.456
    assert 0.0 <= r.tca <= r.horizon


def test_uav_state_buffer_interpolates_and_expires():
    buf = UavStateBuffer()
    assert buf.at(1.0) is None
    buf.add(1.00, [0, 0, -3], [1, 0, 0])
    buf.add(1.01, [0.01, 0, -3], [1, 0, 0])
    p, v = buf.at(1.005)
    assert p == pytest.approx([0.005, 0, -3]) and v == pytest.approx(
        [1, 0, 0])
    assert buf.at(1.02) is not None                # 10 ms: held
    assert buf.at(1.5) is None                     # stale: no UAV state
    buf.add(1.005, [9, 9, 9], [9, 9, 9])           # out of order: ignored
    assert buf.at(1.005)[0] == pytest.approx([0.005, 0, -3])
    buf.add(2.0, [np.nan, 0, 0], [0, 0, 0])        # invalid: ignored
    assert buf.at(1.01) is not None
    buf.add(1.6, [1, 0, -3], [1, 0, 0])
    assert buf.at(1.3) is None                     # 590 ms gap


def test_expired_prediction_yields_no_risk_entry():
    """A stale or dead track has no Phase 4 prediction, so nothing to assess."""
    predictor = DebrisPredictor()
    stale = TrackState(1, 100.0, np.array([0, 0, -9, 0, 0, 5.0]),
                       np.eye(6) * 0.01, 2, 0.6)        # coasting for 0.6 s
    assert predictor.update([stale]) == []


def test_sampled_closest_approach_single_state():
    ca = sampled_closest_approach(
        [Knot(0.0, np.array([3.0, 4.0, -3.0]), np.zeros(3))], UAV, STILL, P)
    assert ca['d'] == pytest.approx(5.0) and ca['t'] == 0.0


def test_uncertainty_margin_uses_covariance_when_it_is_small():
    knots = [Knot(k.h, k.p, k.v, np.eye(3) * 0.05 ** 2)
             for k in falling_past(2.0)]
    r = assess(1, 100.0, 1, knots, UAV, STILL, 0.4, P)
    assert r.uncertainty_margin == pytest.approx(0.05, abs=1e-6)
    assert r.conservative_separation == pytest.approx(r.d_min - 0.05)


# -- Phase 6.1: UAV motion models -------------------------------------------

def _falling_knots(north, height, t_hit):
    """Box that reaches the UAV altitude at ``north`` after ``t_hit`` s."""
    from uav_autonomy.debris_risk import ballistic_knots
    p = [north, 0.0, -5.0 - height]
    vz = (height - 0.5 * 9.8 * t_hit ** 2) / t_hit
    return ballistic_knots(p, [0.0, 0.0, vz], np.diag([0.01] * 6),
                           RiskParams())


def test_default_uav_model_is_constant_velocity():
    assert RiskParams().uav_motion_model == 'constant_velocity'
    knots = _falling_knots(0.0, 10.0, 1.2)
    base = assess(1, 0.0, 1, knots, [0, 0, -5], [0, 0, 0], 0.4)
    same = assess(1, 0.0, 1, knots, [0, 0, -5], [0, 0, 0], 0.4,
                  uav_a=[5.0, 0, 0])          # ignored by the default model
    assert same.d_min == base.d_min and same.state == base.state


def test_constant_acceleration_model_sees_where_the_uav_will_be():
    import dataclasses
    params = dataclasses.replace(RiskParams(),
                                 uav_motion_model='constant_acceleration',
                                 uav_accel_hold_s=1.0)
    # UAV at rest but accelerating north at 4 m/s^2: after 1.0 s it is 2 m
    # north, where the box comes down.
    knots = _falling_knots(2.0, 10.0, 1.0)
    cv = assess(1, 0.0, 1, knots, [0, 0, -5], [0, 0, 0], 0.4)
    ca = assess(1, 0.0, 1, knots, [0, 0, -5], [0, 0, 0], 0.4, params,
                uav_a=[4.0, 0, 0])
    assert cv.d_min > 1.8 and cv.state in (0, 1)
    assert ca.d_min < 0.3 and ca.state == 3


def test_acceleration_deadband_and_clip():
    from uav_autonomy.debris_risk import uav_trajectory
    params = RiskParams()
    t, quiet = uav_trajectory([0, 0, 0], [1, 0, 0], [0.3, 0, 0], 1.0, params)
    assert quiet[-1, 0] == pytest.approx(1.0)            # inside deadband
    t, hard = uav_trajectory([0, 0, 0], [0, 0, 0], [40.0, 0, 0], 1.0, params)
    hold = params.uav_accel_hold_s
    expected = (0.5 * params.uav_accel_max_m_s2 * hold ** 2
                + params.uav_accel_max_m_s2 * hold * (1.0 - hold))
    assert hard[-1, 0] == pytest.approx(expected)


def test_explicit_uav_trajectory_is_used():
    knots = _falling_knots(3.0, 10.0, 1.0)
    times = np.linspace(0, 2, 101)
    path = np.column_stack([3.0 * times, 0 * times, -5 + 0 * times])
    r = assess(1, 0.0, 1, knots, [0, 0, -5], [0, 0, 0], 0.4,
               uav_traj=(times, path))
    assert r.d_min < 0.3 and r.tca == pytest.approx(1.0, abs=0.05)
