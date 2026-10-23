"""Unit tests for the mission state machine (Phase 10). No ROS."""

import os

import pytest

from uav_autonomy.mission_interface import (
    CMD_LAND_NOW,
    CMD_NONE,
    CMD_RETURN_HOME,
    CMD_START,
    validate_mission_command,
)
from uav_autonomy.mission_logic import (
    Inputs,
    MissionManager,
    MissionSpec,
    PHASE_COMPLETE,
    PHASE_FAILSAFE,
    PHASE_LANDING,
    PHASE_PREFLIGHT,
    PHASE_REFUSED,
    PHASE_RETURN,
    PHASE_SEARCH,
    PHASE_STARTING,
    PHASE_TAKEOFF,
    STOP_FIRST_CONFIRMED,
    classify,
    local_to_geodetic,
)

SRC = os.path.join(os.path.dirname(__file__), '..', 'uav_autonomy')
VLM_UNAVAILABLE = 1


def ready(now, **kw):
    """Inputs with every function alive and the controller waiting."""
    base = dict(now=now, flight_state='WAIT_FOR_DATA', flight_age=0.1,
                detections_age=0.1, vlm_status=VLM_UNAVAILABLE,
                locations_age=0.1, planner_present=True)
    base.update(kw)
    return Inputs(**base)


def started(spec=None, **kw):
    """A manager that has passed preflight and reached SEARCH at t=10."""
    m = MissionManager(spec or MissionSpec())
    m.update(ready(0.0, **kw))
    assert m.update(ready(2.5, **kw)).command == CMD_START
    m.update(ready(5.0, flight_state='TAKEOFF', **kw))
    m.update(ready(10.0, flight_state='SEARCH', **kw))
    assert m.phase == PHASE_SEARCH
    return m


def loc(entity_id=1, confirmed=False, rejected=False, obs=100, state=2,
        ned=(5.0, 1.0, -1.0)):
    return {'id': entity_id, 'ned': list(ned), 'sigma_h': 0.21,
            'state': state, 'obs': obs, 'confirmed': confirmed,
            'rejected': rejected}


# -- controller-side command rules -------------------------------------------

def test_start_only_on_the_ground():
    assert validate_mission_command(CMD_START, 1, 0, 0.1,
                                    'WAIT_FOR_DATA') == (True, None)
    ok, reason = validate_mission_command(CMD_START, 1, 0, 0.1, 'SEARCH')
    assert not ok and 'not allowed' in reason


def test_return_home_only_in_mission_flight_states():
    for state in ('SEARCH', 'HOLD', 'WAYPOINT_1'):
        assert validate_mission_command(CMD_RETURN_HOME, 2, 1, 0.1, state)[0]
    for state in ('TAKEOFF', 'LANDING', 'FAILSAFE', 'WAIT_FOR_DATA',
                  'RETURN_HOME', 'COMPLETE'):
        assert not validate_mission_command(CMD_RETURN_HOME, 2, 1, 0.1,
                                            state)[0]


def test_land_now_allowed_while_returning_but_not_in_failsafe():
    assert validate_mission_command(CMD_LAND_NOW, 3, 2, 0.1,
                                    'RETURN_HOME')[0]
    assert not validate_mission_command(CMD_LAND_NOW, 3, 2, 0.1,
                                        'FAILSAFE')[0]


def test_stale_command_is_refused():
    ok, reason = validate_mission_command(CMD_START, 1, 0, 3.0,
                                          'WAIT_FOR_DATA')
    assert not ok and 'stale' in reason


def test_repeated_sequence_is_ignored_silently():
    assert validate_mission_command(CMD_RETURN_HOME, 2, 2, 0.1,
                                    'SEARCH') == (False, None)
    assert validate_mission_command(CMD_NONE, 9, 2, 0.1,
                                    'SEARCH') == (False, None)


def test_unknown_command_is_refused():
    assert not validate_mission_command(77, 5, 2, 0.1, 'SEARCH')[0]


# -- mission specification ---------------------------------------------------

def test_default_mission_is_valid():
    assert MissionSpec().problems() == []


def test_mission_from_file_content():
    spec = MissionSpec.from_dict({
        'name': 'yard', 'search_area': {'north': [-4, 12], 'east': [-4, 4]},
        'altitude_m': 5, 'time_budget_s': 120, 'require_vlm': True})
    assert (spec.north_min, spec.north_max) == (-4.0, 12.0)
    assert spec.require_vlm is True and spec.time_budget_s == 120.0
    assert spec.problems() == []


def test_unknown_field_is_an_error():
    with pytest.raises(ValueError):
        MissionSpec.from_dict({'altitude': 5})


@pytest.mark.parametrize('change, word', [
    ({'north_min': 3.5, 'north_max': 4.0}, 'below 1 m'),
    ({'north_max': 60.0}, 'above 50 m'),
    ({'north_max': 30.0}, '25 m'),
    ({'altitude_m': 1.0}, 'altitude'),
    ({'altitude_m': 40.0}, 'altitude'),
    ({'time_budget_s': 5.0}, 'time budget'),
    ({'time_budget_s': 900.0}, 'time budget'),
    ({'stop_policy': 'whenever'}, 'stop policy'),
    ({'north_min': float('nan')}, 'not finite'),
    ({'battery_return_fraction': 1.5}, 'battery')])
def test_invalid_missions_are_reported(change, word):
    spec = MissionSpec(**change)
    assert any(word in p for p in spec.problems())


# -- preflight ---------------------------------------------------------------

def test_start_is_sent_after_checks_hold_for_two_seconds():
    m = MissionManager(MissionSpec())
    assert m.update(ready(0.0)).command == CMD_NONE
    assert m.phase == PHASE_PREFLIGHT
    assert m.update(ready(1.0)).command == CMD_NONE
    decision = m.update(ready(2.1))
    assert decision.command == CMD_START and decision.sequence == 1
    assert m.phase == PHASE_STARTING


def test_start_is_repeated_until_the_controller_moves():
    m = MissionManager(MissionSpec())
    m.update(ready(0.0))
    m.update(ready(2.1))
    again = m.update(ready(2.6))
    assert again.command == CMD_START and again.sequence == 1
    assert m.update(ready(3.0, flight_state='PRESTREAM')).command == CMD_NONE
    assert m.phase == PHASE_TAKEOFF


@pytest.mark.parametrize('kw, name', [
    ({'detections_age': 9.0}, 'survivor_perception'),
    ({'locations_age': 9.0}, 'survivor_localization'),
    ({'planner_present': False}, 'search_planner'),
    ({'flight_age': 9.0}, 'flight_controller'),
    ({'flight_state': 'SEARCH'}, 'flight_controller')])
def test_missing_function_blocks_the_start_then_refuses(kw, name):
    m = MissionManager(MissionSpec())
    for t in (0.0, 5.0, 29.0):
        assert m.update(ready(t, **kw)).command == CMD_NONE
        assert m.phase == PHASE_PREFLIGHT
    assert any(c.startswith(name) for c in m.failed_checks)
    m.update(ready(31.0, **kw))
    assert m.phase == PHASE_REFUSED
    assert any(c.startswith(name) for c in m.refusal)


def test_check_that_recovers_allows_the_start():
    m = MissionManager(MissionSpec())
    m.update(ready(0.0, planner_present=False))
    m.update(ready(5.0, planner_present=False))
    m.update(ready(6.0))
    assert m.update(ready(8.1)).command == CMD_START


def test_required_vlm_missing_refuses():
    m = MissionManager(MissionSpec(require_vlm=True))
    m.update(ready(0.0))
    m.update(ready(31.0))
    assert m.phase == PHASE_REFUSED and any('vlm' in c for c in m.refusal)
    report = m.report()
    assert report['outcome'] == 'REFUSED'
    assert 'The flight was not started.' in report['notes']


def test_required_vlm_present_starts():
    m = MissionManager(MissionSpec(require_vlm=True))
    m.update(ready(0.0, vlm_status=0))
    assert m.update(ready(2.1, vlm_status=0)).command == CMD_START


def test_required_debris_protection_missing_refuses():
    m = MissionManager(MissionSpec(require_debris_protection=True))
    m.update(ready(0.0))
    m.update(ready(31.0))
    assert m.phase == PHASE_REFUSED
    assert any('debris_avoidance' in c for c in m.refusal)


def test_low_battery_blocks_the_start():
    m = MissionManager(MissionSpec(battery_return_fraction=0.3))
    m.update(ready(0.0, battery_remaining=0.35))
    m.update(ready(31.0, battery_remaining=0.35))
    assert m.phase == PHASE_REFUSED and any('battery' in c for c in m.refusal)


def test_invalid_mission_is_refused_immediately():
    m = MissionManager(MissionSpec(altitude_m=40.0))
    m.update(ready(0.0))
    assert m.phase == PHASE_REFUSED and any('altitude' in c for c in m.refusal)


def test_controller_that_never_starts_is_reported():
    m = MissionManager(MissionSpec())
    m.update(ready(0.0))
    m.update(ready(2.1))
    m.update(ready(13.0))
    assert m.phase == PHASE_REFUSED
    assert m.refusal == ['flight controller did not start']


# -- search supervision ------------------------------------------------------

def test_nominal_mission_follows_the_flight_controller():
    m = started()
    assert m.update(ready(20.0, flight_state='SEARCH')).command == CMD_NONE
    m.update(ready(60.0, flight_state='RETURN_HOME', search_complete=True,
                   coverage=1.0))
    assert m.phase == PHASE_RETURN and m.end_reason == 'search area covered'
    m.update(ready(70.0, flight_state='LANDING', search_complete=True))
    assert m.phase == PHASE_LANDING
    m.update(ready(80.0, flight_state='COMPLETE', search_complete=True,
                   coverage=1.0))
    assert m.phase == PHASE_COMPLETE
    phases = [t['phase'] for t in m.timeline]
    assert phases == ['PREFLIGHT', 'STARTING', 'TAKEOFF', 'SEARCH', 'RETURN',
                      'LANDING', 'COMPLETE']
    report = m.report()
    assert report['outcome'] == 'COMPLETE' and report['search_time_s'] == 50.0
    assert report['objective_met'] is True
    assert report['duration_s'] == 77.5


def test_time_budget_ends_the_search():
    m = started(MissionSpec(time_budget_s=30.0))
    assert m.update(ready(39.0, flight_state='SEARCH')).command == CMD_NONE
    decision = m.update(ready(41.0, flight_state='SEARCH'))
    assert decision.command == CMD_RETURN_HOME
    assert 'time budget' in decision.reason
    again = m.update(ready(41.5, flight_state='SEARCH'))
    assert again.command == CMD_RETURN_HOME
    assert again.sequence == decision.sequence
    assert m.update(ready(42.0, flight_state='RETURN_HOME')).command == \
        CMD_NONE
    assert m.phase == PHASE_RETURN and 'time budget' in m.end_reason
    m.update(ready(60.0, flight_state='COMPLETE'))
    report = m.report()
    assert report['outcome'] == 'COMPLETE' and not report['objective_met']


def test_low_battery_ends_the_search():
    m = started(battery_remaining=0.9)
    decision = m.update(ready(20.0, flight_state='SEARCH',
                              battery_remaining=0.24))
    assert decision.command == CMD_RETURN_HOME and 'battery' in decision.reason


def test_lost_perception_ends_the_search():
    m = started()
    assert m.update(ready(20.0, flight_state='SEARCH',
                          detections_age=2.0)).command == CMD_NONE
    decision = m.update(ready(21.0, flight_state='SEARCH',
                              detections_age=3.5))
    assert decision.command == CMD_RETURN_HOME
    assert 'perception lost' in decision.reason


def test_lost_debris_protection_ends_the_search_when_required():
    spec = MissionSpec(require_debris_protection=True)
    m = started(spec, avoidance_age=0.1)
    decision = m.update(ready(20.0, flight_state='SEARCH',
                              avoidance_age=5.0))
    assert decision.command == CMD_RETURN_HOME
    assert 'debris avoidance lost' in decision.reason


def test_lost_localisation_ends_the_search():
    m = started()
    assert m.update(ready(20.0, flight_state='SEARCH',
                          locations_age=2.0)).command == CMD_NONE
    decision = m.update(ready(21.0, flight_state='SEARCH',
                              locations_age=3.5))
    assert decision.command == CMD_RETURN_HOME
    assert 'localisation lost' in decision.reason


def test_stale_debris_perception_ends_a_protected_search():
    """The avoidance planner is alive but has no depth data."""
    spec = MissionSpec(require_debris_protection=True)
    m = started(spec, avoidance_age=0.1, debris_perception_age=0.1)
    assert m.update(ready(20.0, flight_state='SEARCH', avoidance_age=0.1,
                          debris_perception_age=2.0)).command == CMD_NONE
    decision = m.update(ready(21.0, flight_state='SEARCH', avoidance_age=0.1,
                              debris_perception_age=3.5))
    assert decision.command == CMD_RETURN_HOME
    assert 'debris perception stale' in decision.reason


def test_stale_debris_perception_is_ignored_without_the_requirement():
    m = started(avoidance_age=0.1, debris_perception_age=0.1)
    assert m.update(ready(20.0, flight_state='SEARCH', avoidance_age=0.1,
                          debris_perception_age=30.0)).command == CMD_NONE


def test_debris_chain_is_not_needed_unless_required():
    m = started()
    assert m.update(ready(20.0, flight_state='SEARCH',
                          avoidance_age=None)).command == CMD_NONE


def test_first_confirmed_policy_ends_the_search():
    m = started(MissionSpec(stop_policy=STOP_FIRST_CONFIRMED))
    assert m.update(ready(20.0, flight_state='SEARCH',
                          locations=[loc()])).command == CMD_NONE
    decision = m.update(ready(21.0, flight_state='SEARCH',
                              locations=[loc(confirmed=True)]))
    assert decision.command == CMD_RETURN_HOME and 'confirmed' in \
        decision.reason


def test_first_confirmed_needs_a_localised_well_observed_entry():
    m = started(MissionSpec(stop_policy=STOP_FIRST_CONFIRMED))
    for entry in (loc(confirmed=True, state=1), loc(confirmed=True, obs=5)):
        assert m.update(ready(20.0, flight_state='SEARCH',
                              locations=[entry])).command == CMD_NONE


def test_area_covered_policy_ignores_a_confirmation():
    m = started()
    assert m.update(ready(20.0, flight_state='SEARCH',
                          locations=[loc(confirmed=True)])).command == \
        CMD_NONE


def test_search_ended_by_the_controller_is_explained():
    m = started()
    m.update(ready(20.0, flight_state='RETURN_HOME', search_complete=False))
    assert 'flight controller ended the search' in m.end_reason


def test_failsafe_is_terminal_and_reported():
    m = started()
    m.update(ready(20.0, flight_state='FAILSAFE'))
    assert m.phase == PHASE_FAILSAFE
    assert m.update(ready(30.0, flight_state='COMPLETE')).command == CMD_NONE
    assert m.phase == PHASE_FAILSAFE
    assert m.report()['search_end_reason'] == 'flight controller failsafe'


def test_stale_flight_status_does_not_change_the_phase():
    m = started()
    m.update(ready(20.0, flight_state='COMPLETE', flight_age=10.0))
    assert m.phase == PHASE_SEARCH


def test_lost_flight_controller_concludes_the_mission_with_a_report():
    m = started()
    m.update(ready(20.0, flight_state='SEARCH', flight_age=5.0,
                   locations=[loc()]))
    assert m.phase == PHASE_SEARCH
    decision = m.update(ready(26.0, flight_state='SEARCH', flight_age=11.0,
                              locations=[loc()]))
    assert decision.command == CMD_NONE and m.phase == PHASE_FAILSAFE
    report = m.report()
    assert 'flight controller status lost' in report['search_end_reason']
    assert report['objective_met'] is False and len(report['entries']) == 1


def test_no_command_after_completion():
    m = started(MissionSpec(time_budget_s=30.0))
    m.update(ready(20.0, flight_state='COMPLETE'))
    assert m.update(ready(100.0, flight_state='COMPLETE')).command == CMD_NONE


# -- report ------------------------------------------------------------------

def test_classification_is_honest():
    assert classify(loc(confirmed=True), 15) == 'SURVIVOR_CONFIRMED'
    assert classify(loc(), 15) == 'UNVERIFIED_CANDIDATE'
    assert classify(loc(rejected=True), 15) == 'REJECTED_BY_VLM'
    assert classify(loc(confirmed=True, obs=5), 15) == 'LOW_CONFIDENCE'
    # a confirmation outweighs an earlier rejection of the same entry
    assert classify(loc(confirmed=True, rejected=True), 15) == \
        'SURVIVOR_CONFIRMED'


def test_report_without_vlm_confirms_nothing():
    m = started()
    m.update(ready(60.0, flight_state='COMPLETE', coverage=1.0,
                   locations=[loc(1), loc(2, obs=6), loc(3)]))
    report = m.report(home_ne=(0.5, 0.0), origin=(47.397742, 8.545594))
    assert report['vlm_available'] is False
    assert report['summary'] == {'UNVERIFIED_CANDIDATE': 2,
                                 'LOW_CONFIDENCE': 1}
    assert 'SURVIVOR_CONFIRMED' not in report['summary']
    assert any('No vision-language model' in n for n in report['notes'])
    first = report['entries'][0]
    assert first['from_takeoff_m'] == {'north': 4.5, 'east': 1.0}
    assert abs(first['latitude_deg'] - 47.397742) < 1e-4
    assert first['horizontal_sigma_m'] == 0.21


def test_report_lists_confirmed_first():
    m = started(vlm_status=0)
    m.update(ready(60.0, flight_state='COMPLETE', vlm_status=0, locations=[
        loc(1), loc(2, rejected=True), loc(3, confirmed=True)]))
    report = m.report()
    assert [e['classification'] for e in report['entries']] == [
        'SURVIVOR_CONFIRMED', 'UNVERIFIED_CANDIDATE', 'REJECTED_BY_VLM']
    assert report['vlm_available'] is True


def test_local_to_geodetic_distances():
    lat, lon = local_to_geodetic(111.19, 0.0, 0.0, 0.0)
    assert lat == pytest.approx(0.001, rel=1e-3) and lon == 0.0
    lat, lon = local_to_geodetic(0.0, 100.0, 60.0, 10.0)
    assert lon - 10.0 == pytest.approx(0.0017986, rel=1e-3)


# -- audits ------------------------------------------------------------------

@pytest.mark.parametrize('name', ['mission_logic.py',
                                  'mission_manager_node.py',
                                  'mission_interface.py'])
def test_no_ground_truth_and_no_px4_commands(name):
    text = open(os.path.join(SRC, name)).read()
    text = text.replace('never publishes to /fmu/in/*', '')
    for forbidden in ('/fmu/in/', 'gz.msgs', 'gz.transport', '/world/',
                      'poses.csv', 'TrajectorySetpoint', 'VehicleCommand',
                      'OffboardControlMode'):
        assert forbidden not in text, forbidden
