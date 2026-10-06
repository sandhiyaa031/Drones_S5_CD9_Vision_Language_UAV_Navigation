"""Unit tests for the in-flight safety monitor (pure Python, no ROS)."""

import glob
import math
import os

import pytest

from uav_autonomy.flight_safety import (
    FlightSafetyMonitor,
    PositionSample,
    SafetyLimits,
    altitude_excursion,
)

DT = 0.02  # 50 Hz, the rate PX4 publishes local position to ROS


def sample(t, x=0.0, y=0.0, z=-3.0, vx=0.0, vy=0.0, vz=0.0,
           ax=0.0, ay=0.0, az=0.0, **counters):
    return PositionSample(t=t, x=x, y=y, z=z, vx=vx, vy=vy, vz=vz,
                          ax=ax, ay=ay, az=az, **counters)


def run(samples, altitude=3.0, monitor=None):
    monitor = monitor or FlightSafetyMonitor()
    result = None
    for s in samples:
        result = monitor.update(s, altitude)
    return result


def test_steady_hover_is_clean():
    assert run(sample(i * DT) for i in range(500)) is None


def test_nominal_waypoint_leg_is_clean():
    # Accelerate at 6 m/s^2 (largest measured in a clean flight) to 5 m/s.
    samples, x, v = [], 0.0, 0.0
    for i in range(300):
        a = 6.0 if v < 5.0 else 0.0
        v_next = min(5.0, v + a * DT)
        x += 0.5 * (v + v_next) * DT
        samples.append(sample(i * DT, x=x, vx=v_next, ax=a))
        v = v_next
    assert run(samples) is None


def test_impact_acceleration_is_detected():
    samples = [sample(i * DT) for i in range(10)]
    samples.append(sample(10 * DT, ax=80.0, ay=60.0))
    assert 'impact' in run(samples)


def test_vertical_impact_is_detected():
    assert 'impact' in run([sample(0.0), sample(DT, az=-45.0)])


def test_acceleration_just_below_limit_passes():
    assert run([sample(0.0), sample(DT, ax=29.0)]) is None


def test_position_jump_is_detected():
    samples = [sample(i * DT) for i in range(5)]
    samples.append(sample(5 * DT, x=1.0))  # 1 m in 20 ms at zero velocity
    assert 'position moved' in run(samples)


def test_small_position_noise_passes():
    samples = [sample(i * DT, x=0.02 * (i % 2)) for i in range(50)]
    assert run(samples) is None


def test_velocity_jump_is_detected():
    samples = [sample(0.0), sample(DT, vx=5.0, x=0.05)]
    assert 'velocity changed' in run(samples)


def test_reset_counter_change_is_detected():
    samples = [sample(0.0), sample(DT, xy_reset_counter=1)]
    assert 'xy_reset_counter' in run(samples)


@pytest.mark.parametrize('name', [
    'z_reset_counter', 'vxy_reset_counter', 'vz_reset_counter'])
def test_every_reset_counter_is_watched(name):
    assert name in run([sample(0.0), sample(DT, **{name: 3})])


def test_speed_envelope():
    assert 'horizontal speed' in run([sample(0.0, vx=5.0, vy=5.0)])
    assert 'vertical speed' in run([sample(0.0, vz=-3.5)])
    assert run([sample(0.0, vx=4.9, vz=1.7)]) is None


def test_non_finite_sample_fails_even_on_ground():
    assert 'non-finite' in run([sample(0.0, x=math.nan)], altitude=0.0)


def test_ground_contact_is_ignored():
    # Touchdown bumps and pre-takeoff resets must not raise a violation.
    samples = [sample(0.0), sample(DT, ax=200.0, xy_reset_counter=1)]
    assert run(samples, altitude=0.1) is None


def test_no_continuity_check_across_takeoff_boundary():
    monitor = FlightSafetyMonitor()
    assert monitor.update(sample(0.0), 0.1) is None
    # First airborne sample differs a lot from the last ground sample.
    assert monitor.update(sample(DT, x=2.0), 3.0) is None


def test_violation_is_latched():
    monitor = FlightSafetyMonitor()
    monitor.update(sample(0.0, ax=100.0), 3.0)
    assert monitor.update(sample(DT), 3.0) is not None
    monitor.reset()
    assert monitor.update(sample(2 * DT), 3.0) is None


def test_long_sample_gap_is_left_to_stale_check():
    samples = [sample(0.0), sample(5.0, x=3.0)]
    assert run(samples) is None


def test_dropped_sample_does_not_false_trigger():
    # 100 ms gap while decelerating hard but legitimately.
    samples = [sample(0.0, vx=5.0, ax=-6.0),
               sample(0.1, x=0.47, vx=4.4, ax=-6.0)]
    assert run(samples) is None


def test_custom_limits():
    limits = SafetyLimits(max_horizontal_speed_m_s=2.0)
    monitor = FlightSafetyMonitor(limits)
    assert 'horizontal speed' in monitor.update(sample(0.0, vx=2.5), 3.0)


def test_altitude_excursion():
    assert altitude_excursion(3.2, 3.0, 1.0) is None
    assert 'excursion' in altitude_excursion(1.6, 3.0, 1.0)
    assert 'excursion' in altitude_excursion(4.3, 3.0, 1.0)
    assert 'not finite' in altitude_excursion(math.nan, 3.0, 1.0)


# --- Replay of recorded PX4 logs (skipped when the logs are not present) ---

RESULTS = os.path.join(
    os.path.dirname(__file__), '..', '..', '..', 'results', 'phase0')


def replay(path, stride):
    """Feed a ULog through the monitor; return (violation, s after arm)."""
    np = pytest.importorskip('numpy')
    pyulog = pytest.importorskip('pyulog')
    log = pyulog.ULog(path, message_name_filter_list=[
        'vehicle_local_position', 'vehicle_status'])
    data = {d.name: d.data for d in log.data_list}
    p = data['vehicle_local_position']
    status = data['vehicle_status']
    armed = status['timestamp'][status['arming_state'] == 2]
    t_arm = armed[0] * 1e-6
    monitor = FlightSafetyMonitor()
    ground = float(p['z'][0])
    for i in range(0, len(p['timestamp']), stride):
        t = p['timestamp'][i] * 1e-6
        if t < t_arm:
            continue
        s = PositionSample(
            t=t, x=float(p['x'][i]), y=float(p['y'][i]), z=float(p['z'][i]),
            vx=float(p['vx'][i]), vy=float(p['vy'][i]),
            vz=float(p['vz'][i]), ax=float(p['ax'][i]),
            ay=float(p['ay'][i]), az=float(p['az'][i]),
            xy_reset_counter=int(p['xy_reset_counter'][i]),
            z_reset_counter=int(p['z_reset_counter'][i]),
            vxy_reset_counter=int(p['vxy_reset_counter'][i]),
            vz_reset_counter=int(p['vz_reset_counter'][i]))
        reason = monitor.update(s, ground - s.z)
        if reason:
            return reason, t - t_arm
    assert np is not None
    return None, None


# The log is ~125 Hz; strides emulate 62, 42, 25 and 12 Hz delivery.
@pytest.mark.parametrize('stride', [2, 3, 5, 10])
def test_recorded_collision_is_caught(stride):
    path = os.path.join(RESULTS, 'final_config', 'waypoint_01', 'flight.ulg')
    if not os.path.exists(path):
        pytest.skip('recorded collision log not present')
    reason, seconds = replay(path, stride)
    assert reason is not None
    # Ground truth from the log: first impact 8.66 s after arming.
    assert 8.5 <= seconds <= 9.5


def test_recorded_clean_hovers_do_not_trigger():
    paths = sorted(glob.glob(
        os.path.join(RESULTS, 'final_config', 'hover_0*', 'flight.ulg')))
    if not paths:
        pytest.skip('recorded hover logs not present')
    for path in paths:
        assert replay(path, 3) == (None, None), path
