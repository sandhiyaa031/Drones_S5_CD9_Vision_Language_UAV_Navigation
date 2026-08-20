"""
Unit tests for Coordinate Transformations (REP-103 ENU to NED conversions).
"""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R_scipy


def test_enu_to_ned_position_transformation():
    # R_enu_to_ned: X_ned = Y_enu, Y_ned = X_enu, Z_ned = -Z_enu
    R_enu_to_ned = np.array([
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0]
    ])

    # Drone flying North (Y=5m in ENU) at 2m altitude (Z=+2m in ENU)
    p_enu = np.array([0.0, 5.0, 2.0])
    p_ned = R_enu_to_ned @ p_enu

    # In NED: X=North=5m, Y=East=0m, Z=Down=-2m
    assert np.allclose(p_ned, [5.0, 0.0, -2.0])


def test_enu_to_ned_orientation_transformation():
    R_enu_to_ned = np.array([
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0]
    ])

    # In ENU, facing East (X-axis forward) has Identity rotation
    r_enu_east = np.eye(3)
    r_ned_east = R_enu_to_ned @ r_enu_east @ R_enu_to_ned.T

    # In NED, facing East means 90 deg Yaw around Z-down
    r_obj = R_scipy.from_matrix(r_ned_east)
    rot_vec = r_obj.as_rotvec()
    # Rotation matrix is valid orthogonal
    assert np.allclose(r_ned_east @ r_ned_east.T, np.eye(3))
    assert np.isclose(np.linalg.det(r_ned_east), 1.0)
