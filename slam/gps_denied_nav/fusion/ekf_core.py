"""
15-State Error-State Extended Kalman Filter (ES-EKF) for UAV Multi-Sensor Fusion.
Fuses IMU (high-rate 100Hz), Visual Odometry (30Hz), and Rangefinder/Barometer (20Hz).
State Vector (15): [pos(3), vel(3), orient_error(3), accel_bias(3), gyro_bias(3)]
Nominal State: [pos(3), vel(3), quaternion(4), accel_bias(3), gyro_bias(3)]
"""

import numpy as np
from scipy.spatial.transform import Rotation as R_scipy
from typing import Optional, Tuple, Dict


class MultiSensorEKF:
    def __init__(
        self,
        accel_noise: float = 0.05,        # m/s^2 / sqrt(Hz)
        gyro_noise: float = 0.005,        # rad/s / sqrt(Hz)
        accel_bias_noise: float = 0.0001, # m/s^3 / sqrt(Hz)
        gyro_bias_noise: float = 0.00001, # rad/s^2 / sqrt(Hz)
        vo_pos_noise: float = 0.05,       # m
        vo_rot_noise: float = 0.02,       # rad
        range_noise: float = 0.03,        # m
        gravity_norm: float = 9.80665
    ):
        self.g = np.array([0.0, 0.0, -gravity_norm], dtype=np.float64)  # ENU World gravity vector

        # Nominal States
        self.p = np.zeros(3, dtype=np.float64)                          # Position (x, y, z) [m]
        self.v = np.zeros(3, dtype=np.float64)                          # Velocity (vx, vy, vz) [m/s]
        self.q = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)      # Orientation Quaternion [qx, qy, qz, qw]
        self.b_a = np.zeros(3, dtype=np.float64)                        # Accel bias [m/s^2]
        self.b_g = np.zeros(3, dtype=np.float64)                        # Gyro bias [rad/s]

        # 15x15 Error-State Covariance Matrix P
        # Error states: [delta_p(3), delta_v(3), delta_theta(3), delta_ba(3), delta_bg(3)]
        self.P = np.eye(15, dtype=np.float64) * 0.01
        self.P[0:3, 0:3] = np.eye(3) * 0.25       # 0.5m initial position uncertainty
        self.P[3:6, 3:6] = np.eye(3) * 0.10       # 0.3m/s initial velocity uncertainty
        self.P[6:9, 6:9] = np.eye(3) * 0.01       # 0.1 rad initial orientation uncertainty
        self.P[9:12, 9:12] = np.eye(3) * 0.001    # Accel bias uncertainty
        self.P[12:15, 12:15] = np.eye(3) * 0.0001 # Gyro bias uncertainty

        # Noise parameters
        self.accel_noise = accel_noise
        self.gyro_noise = gyro_noise
        self.accel_bias_noise = accel_bias_noise
        self.gyro_bias_noise = gyro_bias_noise
        self.vo_pos_noise = vo_pos_noise
        self.vo_rot_noise = vo_rot_noise
        self.range_noise = range_noise

        self.last_imu_time: Optional[float] = None
        self.is_initialized: bool = False

    def predict_imu(self, a_m: np.ndarray, w_m: np.ndarray, timestamp: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        IMU Mechanization & Covariance Propagation step (predict step).

        Args:
            a_m: Measured body linear acceleration [ax, ay, az] (m/s^2)
            w_m: Measured body angular velocity [wx, wy, wz] (rad/s)
            timestamp: Timestamp in seconds

        Returns:
            pos: (3,) Estimated position [x, y, z]
            vel: (3,) Estimated velocity [vx, vy, vz]
            quat: (4,) Estimated quaternion [qx, qy, qz, qw]
        """
        if self.last_imu_time is None:
            self.last_imu_time = timestamp
            self.is_initialized = True
            return self.p.copy(), self.v.copy(), self.q.copy()

        dt = timestamp - self.last_imu_time
        self.last_imu_time = timestamp

        if dt <= 0.0 or dt > 0.5:
            dt = 0.01  # Fallback for timestamp jitter

        # Unbias IMU measurements
        a_unbiased = a_m - self.b_a
        w_unbiased = w_m - self.b_g

        # Current rotation matrix (Body to World)
        R_b_w = R_scipy.from_quat(self.q).as_matrix()

        # 1. Propagate Nominal State
        # Linear acceleration in world frame (subtract gravity)
        a_w = R_b_w @ a_unbiased + self.g

        # Position and Velocity update (Trapezoidal / Euler integration)
        self.p += self.v * dt + 0.5 * a_w * (dt ** 2)
        self.v += a_w * dt

        # Orientation update using quaternion integration
        rot_vec = w_unbiased * dt
        angle = np.linalg.norm(rot_vec)
        if angle > 1e-8:
            dq = R_scipy.from_rotvec(rot_vec).as_quat()  # [qx, qy, qz, qw]
            # Quaternion multiplication: q_new = q * dq
            r_curr = R_scipy.from_quat(self.q)
            r_dq = R_scipy.from_quat(dq)
            self.q = (r_curr * r_dq).as_quat()
            # Normalize quaternion
            self.q /= np.linalg.norm(self.q)

        # 2. Error-State Transition Matrix Fx (15x15)
        Fx = np.eye(15, dtype=np.float64)
        Fx[0:3, 3:6] = np.eye(3) * dt

        # Skew-symmetric matrix of a_unbiased
        a_skew = self._skew_symmetric(a_unbiased)
        Fx[3:6, 6:9] = -R_b_w @ a_skew * dt
        Fx[3:6, 9:12] = -R_b_w * dt

        w_skew = self._skew_symmetric(w_unbiased)
        Fx[6:9, 6:9] = np.eye(3) - w_skew * dt
        Fx[6:9, 12:15] = -np.eye(3) * dt

        # Process Noise Covariance Q (15x15)
        Q = np.zeros((15, 15), dtype=np.float64)
        var_a = (self.accel_noise ** 2) * dt
        var_w = (self.gyro_noise ** 2) * dt
        var_ba = (self.accel_bias_noise ** 2) * dt
        var_bg = (self.gyro_bias_noise ** 2) * dt

        Q[0:3, 0:3] = np.eye(3) * (var_a * (dt ** 2) / 4.0)
        Q[3:6, 3:6] = np.eye(3) * var_a
        Q[6:9, 6:9] = np.eye(3) * var_w
        Q[9:12, 9:12] = np.eye(3) * var_ba
        Q[12:15, 12:15] = np.eye(3) * var_bg

        # Propagate Error Covariance: P = Fx * P * Fx^T + Q
        self.P = Fx @ self.P @ Fx.T + Q

        # Enforce symmetry
        self.P = 0.5 * (self.P + self.P.T)

        return self.p.copy(), self.v.copy(), self.q.copy()

    def update_visual_odometry(self, vo_pos: np.ndarray, vo_quat: np.ndarray, vo_cov: Optional[np.ndarray] = None) -> bool:
        """
        Measurement update using 6-DoF Visual Odometry (Position and Orientation).

        Args:
            vo_pos: (3,) Observed position [x, y, z]
            vo_quat: (4,) Observed quaternion [qx, qy, qz, qw]
            vo_cov: (6, 6) Optional covariance matrix
        """
        # Innovation in position
        delta_p = vo_pos.flatten() - self.p

        # Innovation in orientation (convert difference to rotation vector)
        r_ekf = R_scipy.from_quat(self.q)
        r_vo = R_scipy.from_quat(vo_quat)
        delta_r = (r_ekf.inv() * r_vo).as_rotvec()

        # Measurement residual y (6x1)
        y = np.hstack((delta_p, delta_r))

        # Measurement matrix H (6x15)
        H = np.zeros((6, 15), dtype=np.float64)
        H[0:3, 0:3] = np.eye(3)   # Position observed directly
        H[3:6, 6:9] = np.eye(3)   # Orientation error observed directly

        # Measurement Noise R (6x6)
        if vo_cov is not None and vo_cov.shape == (6, 6):
            R_mat = vo_cov.copy()
        else:
            R_mat = np.eye(6, dtype=np.float64)
            R_mat[0:3, 0:3] *= (self.vo_pos_noise ** 2)
            R_mat[3:6, 3:6] *= (self.vo_rot_noise ** 2)

        # Innovation Covariance S = H * P * H^T + R
        S = H @ self.P @ H.T + R_mat

        # Mahalanobis Distance adaptive gating
        try:
            S_inv = np.linalg.inv(S)
            m_dist_sq = y.T @ S_inv @ y
            if m_dist_sq > 250.0:
                # Soft gating: inflate measurement covariance for large innovations
                R_mat *= 4.0
                S = H @ self.P @ H.T + R_mat
                S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return False

        # Kalman Gain K = P * H^T * S^-1
        K = self.P @ H.T @ S_inv

        # Error State Correction dx (15x1)
        dx = K @ y

        # Inject Error State into Nominal State
        self._inject_error_state(dx)

        # Update Covariance (Joseph form for numerical stability)
        I_KH = np.eye(15) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R_mat @ K.T
        self.P = 0.5 * (self.P + self.P.T)

        return True

    def update_rangefinder(self, altitude_z: float) -> bool:
        """
        Measurement update using 1D Rangefinder / Sonar / Barometer (Z altitude).
        """
        # Innovation in Z
        y = np.array([altitude_z - self.p[2]], dtype=np.float64)

        H = np.zeros((1, 15), dtype=np.float64)
        H[0, 2] = 1.0  # Z position

        R_scalar = np.array([[self.range_noise ** 2]])
        S = H @ self.P @ H.T + R_scalar
        S_inv = 1.0 / S[0, 0]

        # Mahalanobis gate
        if (y[0] ** 2) * S_inv > 16.0:  # 4-sigma threshold
            return False

        K = self.P @ H.T * S_inv
        dx = K.flatten() * y[0]

        self._inject_error_state(dx)

        I_KH = np.eye(15) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R_scalar @ K.T
        self.P = 0.5 * (self.P + self.P.T)

        return True

    def _inject_error_state(self, dx: np.ndarray):
        """
        Inject computed error state vector dx into nominal state and reset error state.
        """
        # Position
        self.p += dx[0:3]
        # Velocity
        self.v += dx[3:6]
        # Orientation
        delta_theta = dx[6:9]
        if np.linalg.norm(delta_theta) > 1e-8:
            dq = R_scipy.from_rotvec(delta_theta)
            r_curr = R_scipy.from_quat(self.q)
            self.q = (r_curr * dq).as_quat()
            self.q /= np.linalg.norm(self.q)
        # Biases
        self.b_a += dx[9:12]
        self.b_g += dx[12:15]

        # Clamp biases to reasonable physical bounds
        self.b_a = np.clip(self.b_a, -1.0, 1.0)
        self.b_g = np.clip(self.b_g, -0.2, 0.2)

    def _skew_symmetric(self, v: np.ndarray) -> np.ndarray:
        """
        Compute skew-symmetric cross-product matrix of 3D vector.
        """
        return np.array([
            [0.0, -v[2], v[1]],
            [v[2], 0.0, -v[0]],
            [-v[1], v[0], 0.0]
        ], dtype=np.float64)

    def get_state(self) -> Dict[str, np.ndarray]:
        """
        Retrieve current estimated state dictionary.
        """
        return {
            "position": self.p.copy(),
            "velocity": self.v.copy(),
            "quaternion": self.q.copy(),
            "accel_bias": self.b_a.copy(),
            "gyro_bias": self.b_g.copy(),
            "covariance": self.P.copy()
        }
