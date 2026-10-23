"""Geometry shared by the SLAM bridge, the navigator and offline scoring.

Pure numpy; no ROS. Frames:

    PX4 local   NED (north, east, down), body FRD   - what PX4 estimates
    ROS / SLAM  ENU (east, north, up),   body FLU   - what slam_toolbox uses
    level frame the body frame with roll and pitch removed (yaw only)

A planar lidar measures in the body's own plane, which tilts when the
vehicle accelerates. `level_scan` re-expresses every return in the level
frame using the attitude estimate and drops returns that are no longer near
the flight plane (they hit the ground, or pass above or below the slice
being mapped).
"""

import math

import numpy as np

NED_TO_ENU = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
FRD_TO_FLU = np.diag([1.0, -1.0, -1.0])


def quat_to_matrix(w, x, y, z):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def enu_flu_rotation(q_ned_frd):
    """Rotation ENU<-FLU from PX4's attitude quaternion (NED<-FRD)."""
    return NED_TO_ENU @ quat_to_matrix(*q_ned_frd) @ FRD_TO_FLU


def yaw_enu(rot_enu_flu) -> float:
    """Heading of the body x axis in ENU (0 = east, counter-clockwise)."""
    return math.atan2(rot_enu_flu[1, 0], rot_enu_flu[0, 0])


def ned_to_enu_xy(position_ned):
    return float(position_ned[1]), float(position_ned[0])


def yaw_matrix(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def level_scan(ranges, angle_min, angle_increment, rot_enu_flu,
               height_above_ground, range_min, range_max,
               max_plane_offset_m=0.8, min_height_m=1.5, bins=None):
    """Re-express a tilted planar scan in the level (yaw-only) frame.

    Returns (ranges in the level frame, same angular grid, inf where empty;
    number of returns dropped). A return is kept only if its point is
    within max_plane_offset_m of the vehicle's horizontal plane and at
    least min_height_m above the ground.
    """
    ranges = np.asarray(ranges, dtype=np.float64)
    n = len(ranges)
    bins = bins or n
    angles = angle_min + angle_increment * np.arange(n)
    valid = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    ranges = np.where(valid, ranges, 0.0)
    tilt = yaw_matrix(-yaw_enu(rot_enu_flu)) @ rot_enu_flu
    points = np.column_stack([ranges * np.cos(angles), ranges * np.sin(angles),
                              np.zeros(n)]) @ tilt.T
    keep = valid & (np.abs(points[:, 2]) <= max_plane_offset_m) & (
        height_above_ground + points[:, 2] >= min_height_m)
    out = np.full(bins, np.inf)
    planar = np.hypot(points[keep, 0], points[keep, 1])
    bearing = np.arctan2(points[keep, 1], points[keep, 0])
    index = np.round((bearing - angle_min) / angle_increment).astype(int) \
        % bins
    np.minimum.at(out, index, planar)
    out[out < range_min] = np.inf
    return out, int(valid.sum() - keep.sum())


class DriftModel:
    """Synthetic odometry degradation for SLAM experiments.

    The pose handed to SLAM is rebuilt from PX4's position increments, each
    scaled by (1 + scale_error) and rotated by a heading error that grows
    at yaw_rate rad/s. With both at zero the output equals the input. This
    is test equipment: it makes the odometry worse on purpose, to measure
    how much the scan matcher corrects.
    """

    def __init__(self, scale_error=0.0, yaw_rate_rad_s=0.0):
        self.scale = 1.0 + scale_error
        self.rate = yaw_rate_rad_s
        self.xy = None
        self.last = None
        self.t0 = None

    def error_yaw(self, t):
        return 0.0 if self.t0 is None else self.rate * (t - self.t0)

    def update(self, t, x, y, yaw):
        """PX4 (x, y, yaw) in ENU -> degraded (x, y, yaw)."""
        if self.xy is None:
            self.xy, self.last, self.t0 = np.array([x, y]), (x, y), t
            return x, y, yaw
        err = self.error_yaw(t)
        dx, dy = x - self.last[0], y - self.last[1]
        c, s = math.cos(err), math.sin(err)
        self.xy = self.xy + self.scale * np.array(
            [c * dx - s * dy, s * dx + c * dy])
        self.last = (x, y)
        return float(self.xy[0]), float(self.xy[1]), yaw + err


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi
