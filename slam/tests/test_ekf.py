"""
Unit tests for 15-State Extended Kalman Filter (EKF).
"""

import numpy as np
import pytest
from gps_denied_nav.fusion.ekf_core import MultiSensorEKF


def test_ekf_initialization():
    ekf = MultiSensorEKF()
    state = ekf.get_state()
    assert np.allclose(state["position"], [0, 0, 0])
    assert np.allclose(state["velocity"], [0, 0, 0])
    assert np.allclose(state["quaternion"], [0, 0, 0, 1.0])
    assert state["covariance"].shape == (15, 15)


def test_ekf_imu_prediction_stationary():
    ekf = MultiSensorEKF()
    # Stationary hover IMU: accel measures [0, 0, +9.80665] (counteracting gravity), gyro measures [0, 0, 0]
    a_hover = np.array([0.0, 0.0, 9.80665])
    w_hover = np.array([0.0, 0.0, 0.0])

    t0 = 0.0
    ekf.predict_imu(a_hover, w_hover, t0)

    # Propagate 1 second (100 steps of dt=0.01)
    for step in range(1, 101):
        t = step * 0.01
        pos, vel, quat = ekf.predict_imu(a_hover, w_hover, t)

    # In stationary hover with gravity compensation, velocity and position should remain near 0
    assert abs(pos[0]) < 0.1
    assert abs(pos[1]) < 0.1
    assert abs(pos[2]) < 0.1


def test_ekf_vo_measurement_update():
    ekf = MultiSensorEKF()
    ekf.predict_imu(np.array([0.0, 0.0, 9.80665]), np.array([0.0, 0.0, 0.0]), 0.0)

    # Injected realistic VO observation step at (0.15, 0.10, 0.05)
    vo_pos = np.array([0.15, 0.10, 0.05])
    vo_quat = np.array([0.0, 0.0, 0.0, 1.0])
    success = ekf.update_visual_odometry(vo_pos, vo_quat)

    assert success is True
    state = ekf.get_state()
    # EKF state should move towards observed VO position
    assert state["position"][0] > 0.02
    assert state["position"][1] > 0.01


def test_ekf_rangefinder_update():
    ekf = MultiSensorEKF()
    ekf.predict_imu(np.array([0.0, 0.0, 9.80665]), np.array([0.0, 0.0, 0.0]), 0.0)

    # Initial altitude step
    success = ekf.update_rangefinder(0.25)
    assert success is True
    state = ekf.get_state()
    assert state["position"][2] > 0.10
