"""
Visual Odometry Engine using 5-Point Essential Matrix, Triangulation, and PnP with Scale Recovery.
Maintains 6-DoF accumulated trajectory in standard coordinate frames.
"""

import numpy as np
import cv2
from typing import Optional, Tuple, Dict, List
from scipy.spatial.transform import Rotation as R_scipy
from .feature_tracker import FeatureTracker


class VisualOdometer:
    def __init__(
        self,
        camera_matrix: np.ndarray,
        dist_coeffs: Optional[np.ndarray] = None,
        tracker: Optional[FeatureTracker] = None,
        min_inliers_for_pose: int = 15,
        ransac_prob: float = 0.999,
        ransac_threshold_px: float = 1.0,
        default_scale: float = 1.0
    ):
        """
        Initialize the Visual Odometry pipeline.

        Args:
            camera_matrix: 3x3 Intrinsic matrix K [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
            dist_coeffs: Distortion coefficients (k1, k2, p1, p2, k3)
            tracker: FeatureTracker instance
            min_inliers_for_pose: Minimum RANSAC inliers required for valid pose update
            ransac_prob: RANSAC confidence
            ransac_threshold_px: RANSAC pixel threshold for Essential Matrix
            default_scale: Baseline scale factor when metric scale is not directly measured
        """
        self.K = camera_matrix.astype(np.float64)
        self.dist_coeffs = np.zeros(5, dtype=np.float64) if dist_coeffs is None else dist_coeffs.astype(np.float64)
        self.tracker = tracker if tracker is not None else FeatureTracker()

        self.min_inliers_for_pose = min_inliers_for_pose
        self.ransac_prob = ransac_prob
        self.ransac_threshold_px = ransac_threshold_px
        self.default_scale = default_scale

        # Current 6-DoF World Pose (Camera frame relative to World Start Frame)
        # World coordinate standard: X-Right/East, Y-Forward/North, Z-Up
        self.R_world_cam: np.ndarray = np.eye(3, dtype=np.float64)
        self.t_world_cam: np.ndarray = np.zeros((3, 1), dtype=np.float64)

        # Velocity in world frame (m/s)
        self.linear_velocity: np.ndarray = np.zeros((3, 1), dtype=np.float64)
        self.angular_velocity: np.ndarray = np.zeros((3, 1), dtype=np.float64)

        # Tracking state metrics
        self.is_initialized: bool = False
        self.last_inlier_count: int = 0
        self.last_reproj_error: float = 0.0
        self.last_timestamp: Optional[float] = None
        self.trajectory: List[np.ndarray] = []  # List of [x, y, z] positions
        self.triangulated_map_points: Dict[int, np.ndarray] = {}  # 3D points {fid: [X, Y, Z]}

    def process_frame(
        self,
        image: np.ndarray,
        timestamp: float,
        altitude_measurement: Optional[float] = None,
        external_scale: Optional[float] = None
    ) -> Tuple[bool, np.ndarray, np.ndarray, np.ndarray]:
        """
        Process a new camera frame, track features, estimate motion, and update pose.

        Args:
            image: Camera frame (Grayscale or BGR)
            timestamp: Timestamp in seconds
            altitude_measurement: Rangefinder/barometer altitude in meters (optional)
            external_scale: Explicit scale multiplier (e.g. from IMU or rangefinder)

        Returns:
            success: Whether pose was successfully estimated
            position: 3x1 vector [x, y, z] in world frame
            quaternion: 4-element array [qx, qy, qz, qw] (ROS convention)
            covariance: 6x6 pose covariance matrix (x, y, z, roll, pitch, yaw)
        """
        curr_pts, prev_pts, feature_ids = self.tracker.track(image)

        dt = (timestamp - self.last_timestamp) if self.last_timestamp is not None else 0.033
        if dt <= 0:
            dt = 0.033
        self.last_timestamp = timestamp

        if len(curr_pts) < self.min_inliers_for_pose:
            # Insufficient features: propagate with constant velocity or hold pose
            self.last_inlier_count = len(curr_pts)
            self.t_world_cam += self.linear_velocity * dt
            pos = self.t_world_cam.copy()
            quat = self._rotation_to_quat(self.R_world_cam)
            cov = self._build_covariance(is_degraded=True)
            return False, pos, quat, cov

        # Undistort 2D points for geometric estimation
        curr_undist = cv2.undistortPoints(curr_pts.reshape(-1, 1, 2), self.K, self.dist_coeffs, P=self.K).reshape(-1, 2)
        prev_undist = cv2.undistortPoints(prev_pts.reshape(-1, 1, 2), self.K, self.dist_coeffs, P=self.K).reshape(-1, 2)

        # Estimate Essential Matrix with 5-point RANSAC
        E, inlier_mask = cv2.findEssentialMat(
            curr_undist,
            prev_undist,
            self.K,
            method=cv2.RANSAC,
            prob=self.ransac_prob,
            threshold=self.ransac_threshold_px
        )

        if E is None or E.shape != (3, 3) or inlier_mask is None:
            self.last_inlier_count = 0
            cov = self._build_covariance(is_degraded=True)
            return False, self.t_world_cam.copy(), self._rotation_to_quat(self.R_world_cam), cov

        inliers = inlier_mask.ravel() == 1
        num_inliers = int(np.sum(inliers))
        self.last_inlier_count = num_inliers

        if num_inliers < self.min_inliers_for_pose:
            cov = self._build_covariance(is_degraded=True)
            return False, self.t_world_cam.copy(), self._rotation_to_quat(self.R_world_cam), cov

        # Recover relative Rotation and Translation unit vector (prev_cam -> curr_cam)
        _, R_rel, t_rel, pose_mask = cv2.recoverPose(
            E,
            curr_undist,
            prev_undist,
            self.K,
            mask=inlier_mask
        )

        # Transform from camera optical frame to drone body frame (FLU):
        # Camera optical: X-right, Y-down, Z-forward
        # Drone body (FLU): X-forward, Y-left, Z-up
        R_cam_to_body = np.array([
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0]
        ], dtype=np.float64)

        # Delta translation in camera optical frame
        t_delta_cam = -R_rel.T @ t_rel
        # Convert to drone body frame
        t_delta_body = R_cam_to_body @ t_delta_cam

        # Determine metric scale factor (typical quadrotor step per frame ~0.03-0.08m)
        scale = self._compute_scale(t_delta_body, altitude_measurement, external_scale, dt)
        t_delta_body_scaled = t_delta_body * scale

        # Relative rotation in body frame: R_body_delta = R_cam_to_body * R_rel^T * R_cam_to_body^T
        R_delta_cam = R_rel.T
        R_delta_body = R_cam_to_body @ R_delta_cam @ R_cam_to_body.T

        # Update World Pose in ENU:
        delta_pos_world = self.R_world_cam @ t_delta_body_scaled

        self.t_world_cam += delta_pos_world
        # If altitude sensor is available, gently constrain Z drift
        if altitude_measurement is not None and altitude_measurement > 0.05:
            self.t_world_cam[2, 0] = self.t_world_cam[2, 0] * 0.9 + float(altitude_measurement) * 0.1

        self.R_world_cam = self.R_world_cam @ R_delta_body

        # Orthogonalize rotation matrix
        u, _, vt = np.linalg.svd(self.R_world_cam)
        self.R_world_cam = u @ vt

        # Calculate velocities
        self.linear_velocity = delta_pos_world / dt
        rot_vec, _ = cv2.Rodrigues(R_delta_body)
        self.angular_velocity = rot_vec / dt

        # Triangulate points for mapping
        self._triangulate_inliers(curr_undist[inliers], prev_undist[inliers], [feature_ids[i] for i, m in enumerate(inliers) if m], R_rel, t_rel * scale)

        self.trajectory.append(self.t_world_cam.flatten().copy())
        self.is_initialized = True

        quat = self._rotation_to_quat(self.R_world_cam)
        cov = self._build_covariance(is_degraded=False, inlier_ratio=num_inliers / len(curr_pts))

        return True, self.t_world_cam.copy(), quat, cov

    def _compute_scale(
        self,
        t_rel: np.ndarray,
        altitude_measurement: Optional[float],
        external_scale: Optional[float],
        dt: float
    ) -> float:
        """
        Compute metric scale factor per frame.
        """
        if external_scale is not None and external_scale > 0:
            return float(external_scale)

        speed = np.linalg.norm(self.linear_velocity)
        if speed > 0.1:
            step = speed * dt
            return float(np.clip(step, 0.01, 0.20))

        # Default per-frame baseline step (at 30 FPS ~ 1.5m/s = 0.05m/step)
        return float(np.clip(self.default_scale * dt, 0.02, 0.10))

    def _triangulate_inliers(
        self,
        curr_pts: np.ndarray,
        prev_pts: np.ndarray,
        fids: List[int],
        R_rel: np.ndarray,
        t_rel: np.ndarray
    ):
        """
        Triangulate 3D coordinates for active visual landmarks.
        """
        if len(curr_pts) == 0:
            return

        P0 = self.K @ np.hstack((np.eye(3), np.zeros((3, 1))))
        P1 = self.K @ np.hstack((R_rel, t_rel))

        pts4d = cv2.triangulatePoints(P0, P1, prev_pts.T, curr_pts.T)
        pts3d = (pts4d[:3] / (pts4d[3] + 1e-8)).T

        for fid, p3 in zip(fids, pts3d):
            # Check depth validity (positive Z and reasonable distance < 30m)
            if p3[2] > 0.2 and p3[2] < 30.0:
                # Transform to world frame
                p3_world = (self.R_world_cam @ p3.reshape(3, 1) + self.t_world_cam).flatten()
                self.triangulated_map_points[fid] = p3_world

        # Keep map size bounded
        if len(self.triangulated_map_points) > 2000:
            # Prune oldest keys
            remove_keys = list(self.triangulated_map_points.keys())[:500]
            for k in remove_keys:
                del self.triangulated_map_points[k]

    def _rotation_to_quat(self, R_mat: np.ndarray) -> np.ndarray:
        """
        Convert 3x3 rotation matrix to quaternion [x, y, z, w].
        """
        r = R_scipy.from_matrix(R_mat)
        return r.as_quat()  # Returns [x, y, z, w]

    def _build_covariance(self, is_degraded: bool = False, inlier_ratio: float = 1.0) -> np.ndarray:
        """
        Build 6x6 pose covariance matrix for [x, y, z, roll, pitch, yaw].
        """
        cov = np.eye(6, dtype=np.float64)
        if is_degraded:
            cov[:3, :3] *= 1.0       # High position uncertainty (1.0 m^2)
            cov[3:, 3:] *= 0.5       # High orientation uncertainty
        else:
            base_pos_var = max(0.001, 0.05 * (1.0 - inlier_ratio))
            base_rot_var = max(0.0005, 0.02 * (1.0 - inlier_ratio))
            cov[:3, :3] *= base_pos_var
            cov[3:, 3:] *= base_rot_var
        return cov

    def get_3d_point_cloud(self) -> np.ndarray:
        """
        Get all triangulated 3D map points as (N, 3) numpy array.
        """
        if len(self.triangulated_map_points) == 0:
            return np.empty((0, 3), dtype=np.float32)
        return np.array(list(self.triangulated_map_points.values()), dtype=np.float32)

    def reset(self):
        """
        Reset Visual Odometer state.
        """
        self.R_world_cam = np.eye(3, dtype=np.float64)
        self.t_world_cam = np.zeros((3, 1), dtype=np.float64)
        self.linear_velocity = np.zeros((3, 1), dtype=np.float64)
        self.angular_velocity = np.zeros((3, 1), dtype=np.float64)
        self.is_initialized = False
        self.last_timestamp = None
        self.trajectory.clear()
        self.triangulated_map_points.clear()
