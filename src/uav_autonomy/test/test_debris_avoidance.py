"""Unit tests for the debris avoidance planner and the controller rules."""

import dataclasses
import math

import numpy as np
import pytest

from uav_autonomy.avoidance_interface import (
    AvoidanceLimits, limit_setpoint, recovered, validate_command)
from uav_autonomy.debris_avoidance import (
    AVOIDANCE_FAILSAFE, AVOIDING, AvoidanceParams, AvoidancePlanner, Box,
    CLEARING, DEFENSIVE, DEGRADED, FRESH, NORMAL_FLIGHT, STALE, PREPARE_AVOIDANCE, RESUME_MISSION, Threat,
    best_effort, debris_path, evaluate, in_structure, make_candidates,
    select, simulate_response)
from uav_autonomy.debris_risk import (
    CRITICAL, POTENTIAL_THREAT, RiskParams, SAFE, WARNING, WATCH,
    ballistic_knots, radii)

P = AvoidanceParams()
GROUND = 0.0
HOVER = np.array([0.0, 0.0, -5.0])      # 5 m above takeoff, local NED
COV = np.diag([0.01] * 3 + [0.04] * 3)


def falling(north, east, height_above, t=10.0, vz=3.0, vn=0.0, ve=0.0,
            state=CRITICAL, track_id=1, uav=HOVER, size=0.4):
    """Ballistic threat ``height_above`` metres above the UAV."""
    p = [uav[0] + north, uav[1] + east, uav[2] - height_above]
    return Threat(track_id, t, ballistic_knots(p, [vn, ve, vz], COV,
                                               RiskParams()), size, state)


def step(planner, t, threats, position=HOVER, velocity=(0, 0, 0),
         mission=HOVER, airborne=True, perception_age=0.04,
         odometry_age=0.01, refused=False):
    return planner.update(t, position, velocity, GROUND, mission, airborne,
                          threats, perception_age, odometry_age, refused)


def true_min_separation(target, threat, p0=HOVER, v0=(0, 0, 0), t_uav=10.0,
                        in_force=HOVER):
    times, pos, _ = simulate_response(p0, v0, [target], P, in_force)
    d_pos, _, valid = debris_path(threat.knots, times + t_uav - threat.stamp)
    valid &= d_pos[:, 2] <= GROUND
    return np.linalg.norm(d_pos[valid] - pos[0, valid], axis=1).min()


# -- response model --------------------------------------------------------

def test_response_matches_measured_step():
    # results/phase6/calibration_step: 4 m horizontal step from hover.
    times, pos, _ = simulate_response(HOVER, [0, 0, 0], [[4, 0, -5]], P,
                                      previous_target=HOVER)
    for t, measured in ((0.4, 0.13), (0.6, 0.44), (0.8, 0.94), (1.0, 1.56)):
        assert pos[0, int(round(t / P.model_dt_s)), 0] == pytest.approx(
            measured, abs=P.model_error_m)


def test_response_matches_measured_reversal():
    # Same flight: the setpoint returned to hover while the vehicle was
    # 3.9 m out, still moving away at 1.34 m/s and already decelerating
    # (measured acceleration -2.16 m/s^2).
    p0 = np.array([3.875, -0.145, -4.941])
    _, pos, _ = simulate_response(
        p0, [1.343, 0.143, 0.02], [[0.012, -0.014, -4.946]], P,
        a0=[-2.16, 0.13, -0.11])
    for t, measured in ((0.4, 0.075), (0.8, -0.681), (1.0, -1.292)):
        assert pos[0, int(round(t / P.model_dt_s)), 0] - p0[0] == \
            pytest.approx(measured, abs=P.model_error_m)


def test_response_starts_smoothly():
    # Attitude lag: almost no displacement in the first 0.2 s.
    times, pos, _ = simulate_response(HOVER, [0, 0, 0], [[4, 0, -5]], P,
                                      previous_target=HOVER)
    assert pos[0, int(round(0.2 / P.model_dt_s)), 0] < 0.05


def test_setpoint_in_force_sets_initial_acceleration():
    # Cruising towards a far setpoint: with it in force the vehicle keeps
    # accelerating; without one the model starts from zero acceleration.
    far = HOVER + np.array([6.0, 0.0, 0.0])
    _, with_prev, _ = simulate_response(HOVER, [1, 0, 0], [far], P,
                                        previous_target=far)
    _, without, _ = simulate_response(HOVER, [1, 0, 0], [far], P)
    assert with_prev[0, 10, 0] > without[0, 10, 0]


def test_response_converges_without_overshoot_blowup():
    q = dataclasses.replace(P, horizon_s=8.0)
    _, pos, vel = simulate_response(HOVER, [0, 0, 0], [[2.5, 0, -5]], q)
    assert pos[0, -1, 0] == pytest.approx(2.5, abs=0.1)
    assert np.abs(pos[0, :, 0]).max() < 3.0


def test_climb_is_negative_down():
    _, pos, _ = simulate_response(HOVER, [0, 0, 0], [[0, 0, -7]], P)
    assert pos[0, -1, 2] < HOVER[2] - 1.0


# -- geometry, frames, timestamps -----------------------------------------

def test_directions_are_local_ned():
    names = {c.name: c.target for c in make_candidates(HOVER, HOVER, P)}
    assert names['north_4'][0] == pytest.approx(4.0)
    assert names['east_4'][1] == pytest.approx(4.0)
    assert names['climb_2'][2] == pytest.approx(-7.0)     # up = smaller z
    assert names['descend_1'][2] == pytest.approx(-4.0)
    assert np.allclose(names['hold'], HOVER)


def test_debris_path_matches_free_fall():
    th = falling(0, 0, 10)
    pos, _, valid = debris_path(th.knots, [0.0, 0.3, 1.2])
    assert valid.all()
    assert pos[1, 2] == pytest.approx(-15 + 3 * 0.3 + 4.9 * 0.09, abs=1e-3)
    assert pos[2, 2] == pytest.approx(-15 + 3 * 1.2 + 4.9 * 1.44, abs=1e-3)


def test_debris_path_does_not_extrapolate():
    th = falling(0, 0, 10)
    _, _, valid = debris_path(th.knots, [-0.1, 2.5])
    assert not valid.any()


def test_track_time_offset_is_applied():
    # The same track seen 0.2 s ago is 0.2 s further along its path now.
    th = falling(0, 0, 12, t=10.0)
    fresh = make_candidates(HOVER, HOVER, P)
    late = make_candidates(HOVER, HOVER, P)
    evaluate(fresh, HOVER, [0, 0, 0], HOVER, GROUND, [th], 10.0, P)
    evaluate(late, HOVER, [0, 0, 0], HOVER, GROUND, [th], 10.2, P)
    east = [c for c in fresh if c.name == 'east_4'][0]
    east_late = [c for c in late if c.name == 'east_4'][0]
    assert east_late.min_clearance < east.min_clearance


def test_debris_below_ground_is_ignored():
    th = falling(0, 0, -4.9)          # 0.1 m above the ground, falling
    cands = make_candidates(HOVER, HOVER, P)
    evaluate(cands, HOVER, [0, 0, 0], HOVER, GROUND, [th], 10.5, P)
    assert cands[0].feasible


# -- structure exclusion ---------------------------------------------------

def test_structure_zone_and_corridor():
    pts = np.array([[2.0, 2.0, -3.5],      # over the roof, 3.5 m: keep out
                    [2.0, 2.0, -4.5],      # over the roof, 4.5 m: free
                    [0.0, 0.5, -3.0],      # in the roof opening: corridor
                    [8.0, 0.0, -2.0]])     # outside the building
    assert in_structure(pts, GROUND, P).tolist() == [True, False, False,
                                                     False]


def test_candidates_into_structure_are_rejected():
    inside = np.array([0.0, 0.0, -3.0])    # hovering in the roof opening
    th = falling(0, 0, 9, uav=inside)
    cands = make_candidates(inside, inside, P)
    evaluate(cands, inside, [0, 0, 0], inside, GROUND, [th], 10.0, P)
    by = {c.name: c for c in cands}
    for name in ('north_4', 'south_4', 'west_4', 'north_east_4'):
        assert by[name].rejected == 'structure exclusion zone'
    assert by['climb_2'].rejected != 'structure exclusion zone'


def test_descent_into_roof_clearance_is_rejected():
    over_roof = np.array([2.0, 2.0, -4.5])
    cands = make_candidates(over_roof, over_roof, P)
    evaluate(cands, over_roof, [0, 0, 0], over_roof, GROUND, [], 10.0, P)
    by = {c.name: c for c in cands}
    assert by['descend_1'].rejected == 'structure exclusion zone'


def test_custom_zone():
    q = dataclasses.replace(
        P, keep_out=(Box((1.0, 9.0), (-9.0, 9.0), (0.0, 20.0)),),
        corridors=())
    cands = make_candidates(HOVER, HOVER, q)
    evaluate(cands, HOVER, [0, 0, 0], HOVER, GROUND, [], 10.0, q)
    by = {c.name: c for c in cands}
    assert not by['north_4'].feasible and by['south_4'].feasible


# -- candidate rejection and selection ------------------------------------

def test_altitude_limits_reject():
    low = np.array([20.0, 0.0, -2.0])
    cands = make_candidates(low, low, P)
    evaluate(cands, low, [0, 0, 0], low, GROUND, [], 10.0, P)
    assert {c.name: c for c in cands}['descend_1'].rejected == \
        'below minimum altitude'
    high = np.array([20.0, 0.0, -11.0])
    cands = make_candidates(high, high, P)
    evaluate(cands, high, [0, 0, 0], high, GROUND, [], 10.0, P)
    assert {c.name: c for c in cands}['climb_2'].rejected == \
        'above maximum altitude'


def test_speed_limit_rejects():
    q = dataclasses.replace(P, max_speed_xy=2.3)
    cands = make_candidates(HOVER, HOVER, q)
    evaluate(cands, HOVER, [0, 0, 0], HOVER, GROUND, [], 10.0, q)
    by = {c.name: c for c in cands}
    assert by['east_4'].rejected == 'horizontal speed limit'
    assert by['east_2.5'].feasible


def test_no_threat_selects_hold():
    cands = make_candidates(HOVER, HOVER, P)
    evaluate(cands, HOVER, [0, 0, 0], HOVER, GROUND, [], 10.0, P)
    assert select(cands).name == 'hold'


def test_direct_hit_rejects_hold_and_selected_candidate_is_safe():
    th = falling(0.05, 0.0, 12)
    cands = make_candidates(HOVER, HOVER, P)
    evaluate(cands, HOVER, [0, 0, 0], HOVER, GROUND, [th], 10.0, P)
    assert cands[0].rejected == 'safety radius + uncertainty margin'
    chosen = select(cands)
    assert chosen is not None and chosen.name != 'hold'
    _, safety = radii(0.4, P.risk)
    assert true_min_separation(chosen.target, th) >= safety


def test_selection_moves_away_from_offset_threat():
    # Debris coming down 0.6 m east of the UAV: go west, not east.
    th = falling(0.0, 0.6, 12)
    cands = make_candidates(HOVER, HOVER, P)
    evaluate(cands, HOVER, [0, 0, 0], HOVER, GROUND, [th], 10.0, P)
    chosen = select(cands)
    assert chosen.target[1] < HOVER[1]


def test_cost_prefers_smaller_safe_displacement():
    th = falling(0.0, 0.9, 14)           # skims the safety volume
    cands = make_candidates(HOVER, HOVER, P)
    evaluate(cands, HOVER, [0, 0, 0], HOVER, GROUND, [th], 10.0, P)
    chosen = select(cands)
    assert chosen.name.endswith('2.5')


def test_cost_terms_and_weights():
    cands = make_candidates(HOVER, HOVER, P)
    evaluate(cands, HOVER, [0, 0, 0], HOVER, GROUND, [], 10.0, P,
             previous_target=np.array([0.0, 4.0, -5.0]))
    east = {c.name: c for c in cands}['east_4']
    assert east.terms == pytest.approx({
        'risk': 0.0, 'deviation': 4.0, 'effort': 4.0, 'change': 0.0,
        'altitude': 0.0})
    assert east.cost == pytest.approx(P.w_distance * 4 + P.w_energy * 4)
    climb = {c.name: c for c in cands}['climb_2']
    assert climb.terms['altitude'] == pytest.approx(2.0)
    assert climb.terms['change'] == pytest.approx(math.hypot(4, 2))


def test_near_miss_outside_safety_volume_keeps_hold():
    th = falling(0.0, 2.5, 12, state=SAFE)
    cands = make_candidates(HOVER, HOVER, P)
    evaluate(cands, HOVER, [0, 0, 0], HOVER, GROUND, [th], 10.0, P)
    assert select(cands).name == 'hold'


def test_all_tracks_constrain_the_choice():
    # One threat overhead, a second coming down where "west" would lead.
    overhead = falling(0.0, 0.0, 12, track_id=1)
    west = falling(0.0, -1.6, 12, track_id=2, state=SAFE)
    cands = make_candidates(HOVER, HOVER, P)
    evaluate(cands, HOVER, [0, 0, 0], HOVER, GROUND, [overhead, west], 10.0,
             P)
    chosen = select(cands)
    assert chosen.target[1] > HOVER[1] - 0.1       # not towards the second
    _, safety = radii(0.4, P.risk)
    for th in (overhead, west):
        assert true_min_separation(chosen.target, th) >= safety


def test_crossing_threat():
    # Thrown from the east: 3 m/s westward, arrives at the UAV.
    t_fall = 1.3
    th = falling(0.0, 3.0 * t_fall, 0.5 * 9.8 * t_fall ** 2, vz=0.0, ve=-3.0)
    cands = make_candidates(HOVER, HOVER, P)
    evaluate(cands, HOVER, [0, 0, 0], HOVER, GROUND, [th], 10.0, P)
    assert not cands[0].feasible
    chosen = select(cands)
    _, safety = radii(0.4, P.risk)
    assert chosen is not None
    assert true_min_separation(chosen.target, th) >= safety


def test_moving_uav():
    # Flying north at 2 m/s towards a waypoint; debris aimed where the
    # UAV will be in 1.4 s.
    v = np.array([2.0, 0.0, 0.0])
    mission = HOVER + np.array([10.0, 0.0, 0.0])
    _, nominal, _ = simulate_response(HOVER, v, [mission], P, mission)
    th = falling(nominal[0, 70, 0], 0.0, 0.5 * 9.8 * 1.4 ** 2, vz=0.0)
    cands = make_candidates(HOVER, mission, P)
    evaluate(cands, HOVER, v, mission, GROUND, [th], 10.0, P)
    assert not cands[0].feasible                   # carrying on is unsafe
    chosen = select(cands)
    assert chosen is not None
    _, safety = radii(0.4, P.risk)
    assert true_min_separation(chosen.target, th, v0=v,
                               in_force=mission) >= safety


def test_no_feasible_candidate_gives_best_effort():
    th = falling(0.0, 0.0, 1.5, vz=8.0)            # 0.17 s away: too late
    cands = make_candidates(HOVER, HOVER, P)
    evaluate(cands, HOVER, [0, 0, 0], HOVER, GROUND, [th], 10.0, P)
    assert select(cands) is None
    # Nothing gains 0.2 m over staying: the best effort is to stay.
    assert best_effort(cands).name == 'hold'
    # With a little more time something does help, and it is chosen.
    th = falling(0.0, 0.0, 4.0, vz=6.0)            # 0.48 s away
    cands = make_candidates(HOVER, HOVER, P)
    evaluate(cands, HOVER, [0, 0, 0], HOVER, GROUND, [th], 10.0, P)
    assert select(cands) is None
    fallback = best_effort(cands)
    assert fallback.name != 'hold'
    assert fallback.min_clearance == max(
        c.min_clearance for c in cands
        if c.rejected in ('', 'safety radius + uncertainty margin'))


# -- state machine ---------------------------------------------------------

def test_no_debris_stays_normal():
    pl = AvoidancePlanner()
    for k in range(20):
        plan = step(pl, 10 + 0.03 * k, [])
        assert plan.mode == NORMAL_FLIGHT and not plan.active
    assert pl.transitions == [] and pl.manoeuvres == 0


@pytest.mark.parametrize('state', [SAFE, WATCH])
def test_safe_and_watch_do_not_trigger(state):
    pl = AvoidancePlanner()
    plan = step(pl, 10.0, [falling(0.0, 2.5, 12, state=state)])
    assert plan.mode == NORMAL_FLIGHT and not plan.active


def test_potential_threat_prepares_without_manoeuvre():
    pl = AvoidancePlanner()
    plan = step(pl, 10.0, [falling(0, 0, 12, state=POTENTIAL_THREAT)])
    assert plan.mode == PREPARE_AVOIDANCE
    assert not plan.active and not plan.precautionary
    assert plan.candidate and plan.candidate != 'hold'   # a plan is ready
    assert pl.manoeuvres == 0


def test_potential_threat_precautionary_option_is_flagged_and_small():
    pl = AvoidancePlanner(dataclasses.replace(
        P, precautionary_on_potential=True))
    plan = step(pl, 10.0, [falling(0, 0, 12, state=POTENTIAL_THREAT)])
    assert plan.mode == PREPARE_AVOIDANCE and plan.active
    assert plan.precautionary
    assert np.linalg.norm(plan.target - HOVER) <= \
        P.precautionary_distance_m + 1e-6


def test_prepare_times_out_to_normal():
    pl = AvoidancePlanner()
    step(pl, 10.0, [falling(0, 0, 12, state=POTENTIAL_THREAT)])
    assert step(pl, 10.3, []).mode == PREPARE_AVOIDANCE
    assert step(pl, 10.7, []).mode == NORMAL_FLIGHT


@pytest.mark.parametrize('state', [WARNING, CRITICAL])
def test_warning_and_critical_trigger_avoidance(state):
    pl = AvoidancePlanner()
    plan = step(pl, 10.0, [falling(0, 0, 12, state=state)])
    assert plan.mode == AVOIDING and plan.active
    assert plan.threat_ids == (1,)
    assert plan.transition[0] == NORMAL_FLIGHT
    assert ('CRITICAL' if state == CRITICAL else 'WARNING') in \
        plan.transition[2]


def test_prepare_then_warning_goes_to_avoiding():
    pl = AvoidancePlanner()
    step(pl, 10.0, [falling(0, 0, 12, state=POTENTIAL_THREAT)])
    plan = step(pl, 10.07, [falling(0, 0, 12, t=10.07, state=WARNING)])
    assert plan.mode == AVOIDING
    assert [tr[1:3] for tr in pl.transitions] == [
        (NORMAL_FLIGHT, PREPARE_AVOIDANCE), (PREPARE_AVOIDANCE, AVOIDING)]


def test_target_is_kept_while_it_stays_safe():
    pl = AvoidancePlanner()
    th = falling(0, 0, 12)
    first = step(pl, 10.0, [th])
    times, pos, vel = simulate_response(HOVER, [0, 0, 0], [first.target], P,
                                        HOVER)
    for k in (5, 10, 15, 20):
        plan = step(pl, 10.0 + times[k], [th], position=pos[0, k],
                    velocity=vel[0, k])
        assert plan.mode == AVOIDING
        assert np.allclose(plan.target, first.target)
    assert pl.manoeuvres == 1


def test_commitment_ignores_small_cost_differences():
    pl = AvoidancePlanner()
    th = falling(0, 0, 12)
    first = step(pl, 10.0, [th])
    # A little later and displaced: other candidates may now be slightly
    # cheaper, but the flown setpoint is still feasible and is kept.
    times, pos, vel = simulate_response(HOVER, [0, 0, 0], [first.target], P,
                                        HOVER)
    names = set()
    for k in range(2, 40, 2):
        plan = step(pl, 10.0 + times[k], [th], position=pos[0, k],
                    velocity=vel[0, k])
        if plan.mode == AVOIDING:
            names.add(plan.candidate)
    assert names == {first.candidate}


def test_nominal_path_trigger_off_follows_risk_states_only():
    # Accelerating along a leg; the track is rated SAFE (constant-velocity
    # assumption) although the flight to the waypoint will meet it.
    mission = HOVER + np.array([6.0, 0.0, 0.0])
    th = falling(5.0, 0.0, 0.5 * 9.8 * 1.3 ** 2, vz=0.0, state=SAFE)
    pl = AvoidancePlanner()
    plan = step(pl, 10.0, [th], velocity=[1.0, 0, 0], mission=mission)
    assert plan.mode == NORMAL_FLIGHT and not plan.active


def test_nominal_path_trigger_on_uses_the_planned_flight():
    mission = HOVER + np.array([6.0, 0.0, 0.0])
    _, nominal, _ = simulate_response(HOVER, [1.0, 0, 0], [mission], P,
                                      mission)
    th = falling(nominal[0, 65, 0], 0.0, 0.5 * 9.8 * 1.3 ** 2, vz=0.0,
                 state=SAFE)
    pl = AvoidancePlanner(dataclasses.replace(P, nominal_path_trigger=True))
    plan = step(pl, 10.0, [th], velocity=[1.0, 0, 0], mission=mission)
    assert plan.mode == AVOIDING and plan.active
    assert 'planner model' in plan.transition[2]
    # ... but never for an unconfirmed object, and not when hovering clear.
    pl = AvoidancePlanner(dataclasses.replace(P, nominal_path_trigger=True))
    th.risk_state = POTENTIAL_THREAT
    assert step(pl, 10.0, [th], velocity=[1.0, 0, 0],
                mission=mission).mode == PREPARE_AVOIDANCE
    pl = AvoidancePlanner(dataclasses.replace(P, nominal_path_trigger=True))
    assert step(pl, 10.0, [falling(0.0, 2.5, 12, state=SAFE)]).mode == \
        NORMAL_FLIGHT


def test_full_cycle_returns_to_mission():
    pl = AvoidancePlanner()
    th = falling(0, 0, 12)
    first = step(pl, 10.0, [th])
    away = HOVER + (first.target - HOVER) * 0.5
    # The debris has passed (track gone): clear, then resume.
    plan = step(pl, 11.8, [], position=away)
    assert plan.mode == CLEARING and plan.active
    plan = step(pl, 11.8 + P.clear_hold_s + 0.01, [], position=away)
    assert plan.mode == RESUME_MISSION and not plan.active
    plan = step(pl, 12.5, [], position=away)
    assert plan.mode == RESUME_MISSION           # not back yet
    plan = step(pl, 14.0, [], position=HOVER + [0.1, 0.0, 0.0])
    assert plan.mode == NORMAL_FLIGHT
    assert [tr[2] for tr in pl.transitions] == [
        AVOIDING, CLEARING, RESUME_MISSION, NORMAL_FLIGHT]


def test_does_not_resume_into_a_falling_object():
    pl = AvoidancePlanner()
    first = step(pl, 10.0, [falling(0, 0, 12)])
    away = HOVER + (first.target - HOVER) * 0.25
    # Still above the mission setpoint and, from where the UAV now is, only
    # SAFE: but it would arrive just as the UAV got back, so keep avoiding.
    still = falling(0, 0, 12, t=10.8, state=SAFE)
    plan = step(pl, 10.8, [still], position=away)
    assert plan.mode in (AVOIDING, AVOIDANCE_FAILSAFE) and plan.active


def test_repeated_threats():
    pl = AvoidancePlanner()
    step(pl, 10.0, [falling(0, 0, 12)])
    step(pl, 12.0, [])
    step(pl, 12.4, [])
    assert step(pl, 14.0, []).mode == NORMAL_FLIGHT
    plan = step(pl, 20.0, [falling(0, 0, 12, t=20.0, track_id=2)])
    assert plan.mode == AVOIDING and plan.threat_ids == (2,)
    assert pl.manoeuvres == 2


def test_new_threat_while_resuming():
    pl = AvoidancePlanner()
    first = step(pl, 10.0, [falling(0, 0, 12)])
    away = HOVER + (first.target - HOVER) * 0.5
    step(pl, 12.0, [], position=away)
    assert step(pl, 12.4, [], position=away).mode == RESUME_MISSION
    plan = step(pl, 12.5, [falling(0, 0, 12, t=12.5, track_id=2, uav=away)],
                position=away)
    assert plan.mode == AVOIDING and plan.active


def test_mission_target_is_never_modified():
    pl = AvoidancePlanner()
    mission = HOVER.copy()
    step(pl, 10.0, [falling(0, 0, 12)], mission=mission)
    assert np.array_equal(mission, HOVER)


def test_no_feasible_goes_to_failsafe_best_effort():
    pl = AvoidancePlanner()
    plan = step(pl, 10.0, [falling(0, 0, 4.0, vz=6.0)])     # 0.48 s away
    assert plan.mode == AVOIDANCE_FAILSAFE
    assert plan.failsafe and plan.active and plan.target is not None
    assert 'best effort' in plan.reason
    assert plan.candidates[0].min_clearance < max(
        c.min_clearance for c in plan.candidates)


def test_best_effort_does_not_leave_hold_for_a_negligible_gain():
    # In the roof opening: sideways is forbidden, and neither climbing nor
    # descending gets out of the way. Climbing 14 mm "better" on paper must
    # not send the UAV up into the debris.
    inside = np.array([0.0, 0.0, -3.0])
    pl = AvoidancePlanner()
    plan = step(pl, 10.0, [falling(0.05, 0.0, 9, uav=inside)],
                position=inside, mission=inside)
    assert plan.mode == AVOIDANCE_FAILSAFE and plan.failsafe
    assert not plan.active and plan.candidate == 'hold'


def test_too_late_for_any_motion_is_reported_as_failsafe():
    # Impact inside the vehicle's dead time: no candidate changes anything.
    pl = AvoidancePlanner()
    plan = step(pl, 10.0, [falling(0, 0, 1.5, vz=8.0)])
    assert plan.mode == AVOIDANCE_FAILSAFE and plan.failsafe


def test_boxed_in_holds_mission_setpoint():
    # Every displacement is forbidden by geometry: nothing to command.
    q = dataclasses.replace(
        P, keep_out=(Box((-50, 50), (-50, 50), (0.0, 50.0)),),
        corridors=(Box((-0.3, 0.3), (-0.3, 0.3), (0.0, 5.5)),),
        climb_distances_m=(2.0,), descend_distances_m=())
    pl = AvoidancePlanner(q)
    plan = step(pl, 10.0, [falling(0, 0, 12)])
    assert plan.mode == AVOIDANCE_FAILSAFE and plan.failsafe
    # Only "hold" breaks no vehicle limit; it is the best effort.
    assert not plan.active and plan.candidate == 'hold'


def test_stale_odometry_releases_command():
    pl = AvoidancePlanner()
    step(pl, 10.0, [falling(0, 0, 12)])
    plan = step(pl, 10.1, [falling(0, 0, 12)], odometry_age=0.8)
    assert plan.mode == AVOIDANCE_FAILSAFE
    assert not plan.active and plan.failsafe and 'odometry' in plan.reason


def test_odometry_behind_perception_releases_command():
    pl = AvoidancePlanner()
    step(pl, 10.0, [falling(0, 0, 12)])
    plan = pl.update(10.1, HOVER, [0, 0, 0], GROUND, HOVER, True,
                     [falling(0, 0, 12)], 0.0, 0.01, odometry_lag_s=0.3)
    assert plan.mode == AVOIDANCE_FAILSAFE and not plan.active


def test_short_arrival_gap_is_not_stale_odometry():
    # The simulator can pause for a few hundred ms when an entity is
    # created; that is not a loss of odometry.
    pl = AvoidancePlanner()
    plan = step(pl, 10.0, [falling(0, 0, 12)], odometry_age=0.3)
    assert plan.mode == AVOIDING


def test_perception_health_levels_and_speed_caps():
    pl = AvoidancePlanner()
    fresh = step(pl, 10.0, [], perception_age=0.05)
    assert (fresh.health, fresh.speed_limit) == (FRESH, 0.0)
    degraded = step(pl, 10.1, [], perception_age=0.3)
    assert degraded.health == DEGRADED
    assert degraded.speed_limit == P.degraded_speed_limit
    assert degraded.mode == NORMAL_FLIGHT and not degraded.active
    stale = step(pl, 10.2, [], perception_age=0.7)
    assert stale.health == STALE
    assert stale.speed_limit == P.stale_speed_limit
    assert stale.mode == NORMAL_FLIGHT          # still inside the hold time


def test_stale_perception_is_not_no_debris():
    # A CRITICAL threat seen 0.7 s ago is still acted on: its path is
    # advanced in time and the manoeuvre starts.
    pl = AvoidancePlanner()
    th = falling(0, 0, 25, t=9.3)
    plan = step(pl, 10.0, [th], perception_age=0.7)
    assert plan.health == STALE
    assert plan.mode == AVOIDING and plan.active


def test_defensive_after_hold_time_and_recovery():
    pl = AvoidancePlanner()
    plan = step(pl, 10.0, [falling(0, 0, 12, t=8.5)], perception_age=1.5)
    assert plan.mode == DEFENSIVE and not plan.active and plan.failsafe
    assert plan.speed_limit == P.stale_speed_limit
    assert 'defensive' in plan.reason
    again = step(pl, 10.1, [falling(0, 0, 12, t=8.5)], perception_age=1.6)
    assert again.mode == DEFENSIVE and again.transition is None
    back = step(pl, 10.5, [], perception_age=0.04)
    assert back.mode == NORMAL_FLIGHT and back.speed_limit == 0.0
    assert [tr[2] for tr in pl.transitions] == [DEFENSIVE, NORMAL_FLIGHT]


def test_stale_perception_while_avoiding_keeps_target_then_defensive():
    pl = AvoidancePlanner()
    first = step(pl, 10.0, [falling(0, 0, 12)])
    plan = step(pl, 11.5, [], perception_age=1.5)
    assert plan.active and np.allclose(plan.target, first.target)
    assert plan.failsafe
    plan = step(pl, 10.0 + P.max_manoeuvre_s + 0.1, [], perception_age=4.1)
    assert plan.mode == DEFENSIVE and not plan.active


def test_prepare_requests_speed_cap_without_manoeuvre():
    pl = AvoidancePlanner()
    plan = step(pl, 10.0, [falling(0, 0, 12, state=POTENTIAL_THREAT)])
    assert plan.mode == PREPARE_AVOIDANCE and not plan.active
    assert plan.speed_limit == P.prepare_speed_limit


def test_no_speed_cap_requested_on_the_ground():
    pl = AvoidancePlanner()
    plan = step(pl, 10.0, [], airborne=False, perception_age=5.0)
    assert plan.speed_limit == 0.0 and plan.mode == NORMAL_FLIGHT


def test_speed_limited_hold_is_modelled():
    # Cruising under a 2 m/s cap towards a far waypoint: the model must not
    # predict an acceleration to the uncapped speed.
    far = HOVER + np.array([12.0, 0.0, 0.0])
    _, capped, vel = simulate_response(HOVER, [2.0, 0, 0], [far], P, far,
                                       [2.0], 2.0)
    assert np.linalg.norm(vel[0, :, :2], axis=1).max() < 2.2
    _, free, vel = simulate_response(HOVER, [2.0, 0, 0], [far], P, far)
    assert np.linalg.norm(vel[0, :, :2], axis=1).max() > 4.0


def test_measured_acceleration_initialises_the_model():
    far = HOVER + np.array([6.0, 0.0, 0.0])
    _, braking, _ = simulate_response(HOVER, [3.0, 0, 0], [far], P, far,
                                      a0=[-4.0, 0, 0])
    _, pushing, _ = simulate_response(HOVER, [3.0, 0, 0], [far], P, far,
                                      a0=[4.0, 0, 0])
    assert pushing[0, 20, 0] > braking[0, 20, 0] + 0.2


def test_old_tracks_are_not_used():
    pl = AvoidancePlanner()
    th = falling(0, 0, 12)
    th.time_since_update = 0.5
    assert step(pl, 10.0, [th]).mode == NORMAL_FLIGHT


def test_not_airborne_never_commands():
    pl = AvoidancePlanner()
    plan = step(pl, 10.0, [falling(0, 0, 12)], airborne=False)
    assert plan.mode == NORMAL_FLIGHT and not plan.active


def test_refused_command_tries_another_candidate():
    pl = AvoidancePlanner()
    first = step(pl, 10.0, [falling(0, 0, 12)])
    second = step(pl, 10.03, [falling(0, 0, 12)], refused=True)
    assert second.active and second.candidate != first.candidate


# -- controller-side rules -------------------------------------------------

LIM = AvoidanceLimits()
POS = (0.0, 0.0, -5.0)


def test_controller_accepts_valid_command():
    assert validate_command((4.0, 0.0, -5.0), POS, 0.0, 0.05, True, LIM) == \
        (True, None)


@pytest.mark.parametrize('target, age, allowed, text', [
    ((4.0, 0.0, -5.0), 0.05, False, 'flight state'),
    ((4.0, 0.0, -5.0), 0.8, True, 'stale'),
    ((math.nan, 0.0, -5.0), 0.05, True, 'finite'),
    ((9.0, 0.0, -5.0), 0.05, True, 'from the vehicle'),
    ((0.0, 0.0, -1.0), 0.05, True, 'below'),
    ((0.0, 0.0, -9.0), 0.05, True, None),
])
def test_controller_refuses_invalid_commands(target, age, allowed, text):
    ok, reason = validate_command(target, POS, 0.0, age, allowed, LIM)
    assert ok == (text is None)
    if text:
        assert text in reason


def test_controller_altitude_ceiling():
    ok, reason = validate_command((0.0, 0.0, -16.0), (0.0, 0.0, -12.0), 0.0,
                                  0.05, True, LIM)
    assert not ok and 'above' in reason


def test_controller_altitude_uses_takeoff_reference():
    # Takeoff point at z = +0.3 (down): 1.4 m above it is too low.
    ok, _ = validate_command((0.0, 0.0, -1.1), (0.0, 0.0, -2.0), 0.3, 0.05,
                             True, LIM)
    assert not ok


def test_recovered():
    assert recovered((0.2, 0.1, -5.1), POS, LIM)
    assert not recovered((1.0, 0.0, -5.0), POS, LIM)
    assert not recovered((0.0, 0.0, -5.6), POS, LIM)


def test_speed_cap_shortens_the_setpoint():
    # 12 m away with a 2.9 m/s cap: the setpoint is 2.9 / 0.95 m ahead.
    x, y = limit_setpoint((0.0, 0.0), (12.0, 0.0), 2.9)
    assert (x, y) == pytest.approx((2.9 / 0.95, 0.0))
    # direction is kept
    x, y = limit_setpoint((1.0, 1.0), (7.0, 9.0), 1.9)
    assert math.hypot(x - 1.0, y - 1.0) == pytest.approx(2.0)
    assert (y - 1.0) / (x - 1.0) == pytest.approx(8.0 / 6.0)


def test_speed_cap_leaves_near_setpoints_and_no_cap_alone():
    assert limit_setpoint((0.0, 0.0), (1.0, 0.5), 2.9) == (1.0, 0.5)
    assert limit_setpoint((0.0, 0.0), (12.0, 0.0), 0.0) == (12.0, 0.0)
