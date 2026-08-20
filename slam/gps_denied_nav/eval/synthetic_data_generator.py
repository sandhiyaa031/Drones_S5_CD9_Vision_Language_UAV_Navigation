"""
Synthetic GPS-Denied Data Generator & Environment Simulator.
Generates realistic 3D camera feeds, noisy IMU measurements, rangefinder data,
and Ground Truth trajectories for algorithm testing and evaluation without physical hardware.
"""

import numpy as np
import cv2
from typing import Tuple, List, Dict
from scipy.spatial.transform import Rotation as R_scipy


class SyntheticDataGenerator:
    def __init__(
        self,
        img_width: int = 640,
        img_height: int = 480,
        fx: float = 525.0,
        fy: float = 525.0,
        fps: float = 30.0,
        total_time_sec: float = 20.0,
        flight_pattern: str = "figure8"  # "figure8", "rectangle", "spiral"
    ):
        self.w = img_width
        self.h = img_height
        self.fx = fx
        self.fy = fy
        self.cx = img_width / 2.0
        self.cy = img_height / 2.0
        self.fps = fps
        self.dt = 1.0 / fps
        self.total_time_sec = total_time_sec
        self.total_frames = int(total_time_sec * fps)
        self.flight_pattern = flight_pattern

        self.K = np.array([
            [fx, 0.0, self.cx],
            [0.0, fy, self.cy],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

        # Generate 3D virtual landmark points in the environment
        self.world_landmarks = self._generate_3d_environment()

        # Noise parameters
        self.accel_noise_std = 0.03
        self.gyro_noise_std = 0.005
        self.range_noise_std = 0.02

        # Precompute Ground Truth Trajectory
        self.gt_timestamps = []
        self.gt_positions = []
        self.gt_orientations = []  # Quaternions [qx, qy, qz, qw]
        self.gt_velocities = []
        self.gt_accelerations = []
        self.gt_angular_velocities = []

        self._generate_ground_truth_trajectory()

    def _generate_3d_environment(self) -> np.ndarray:
        """
        Create a rich 3D point cloud of an indoor GPS-denied room (15m x 15m x 4m)
        with floor, ceiling, 4 walls, and internal obstacle pillars.
        """
        np.random.seed(42)
        points = []

        # Floor (Z = 0)
        x_floor = np.random.uniform(-7.5, 7.5, 800)
        y_floor = np.random.uniform(-7.5, 7.5, 800)
        z_floor = np.zeros(800)
        points.append(np.column_stack([x_floor, y_floor, z_floor]))

        # Ceiling (Z = 4.0)
        x_ceil = np.random.uniform(-7.5, 7.5, 400)
        y_ceil = np.random.uniform(-7.5, 7.5, 400)
        z_ceil = np.ones(400) * 4.0
        points.append(np.column_stack([x_ceil, y_ceil, z_ceil]))

        # North & South Walls (Y = 7.5 and Y = -7.5)
        for y_wall in [-7.5, 7.5]:
            xw = np.random.uniform(-7.5, 7.5, 400)
            zw = np.random.uniform(0.0, 4.0, 400)
            yw = np.ones(400) * y_wall
            points.append(np.column_stack([xw, yw, zw]))

        # East & West Walls (X = 7.5 and X = -7.5)
        for x_wall in [-7.5, 7.5]:
            yw = np.random.uniform(-7.5, 7.5, 400)
            zw = np.random.uniform(0.0, 4.0, 400)
            xw = np.ones(400) * x_wall
            points.append(np.column_stack([xw, yw, zw]))

        # 4 Internal Obstacle Pillars
        pillar_centers = [(-3.0, -3.0), (3.0, -3.0), (-3.0, 3.0), (3.0, 3.0)]
        for cx, cy in pillar_centers:
            theta = np.random.uniform(0, 2 * np.pi, 200)
            r = 0.5
            px = cx + r * np.cos(theta)
            py = cy + r * np.sin(theta)
            pz = np.random.uniform(0.0, 3.5, 200)
            points.append(np.column_stack([px, py, pz]))

        return np.vstack(points).astype(np.float64)

    def _generate_ground_truth_trajectory(self):
        """
        Generate smooth continuous 6-DoF quadrotor motion profiles.
        """
        t_arr = np.linspace(0, self.total_time_sec, self.total_frames)

        for t in t_arr:
            if self.flight_pattern == "figure8":
                # Lemniscate of Gerono / Figure 8 trajectory
                scale = 3.5
                omega = 2 * np.pi / (self.total_time_sec * 0.75)
                x = scale * np.sin(omega * t)
                y = (scale * 0.8) * np.sin(2 * omega * t)
                z = 1.8 + 0.3 * np.sin(omega * t)  # Altitude oscillation

                # Analytical derivatives for velocity
                vx = scale * omega * np.cos(omega * t)
                vy = (scale * 0.8) * 2 * omega * np.cos(2 * omega * t)
                vz = 0.3 * omega * np.cos(omega * t)

                # Analytical derivatives for acceleration
                ax = -scale * (omega ** 2) * np.sin(omega * t)
                ay = -(scale * 0.8) * (4 * omega ** 2) * np.sin(2 * omega * t)
                az = -0.3 * (omega ** 2) * np.sin(omega * t)

                # Yaw heading aligns with velocity vector
                yaw = np.arctan2(vy, vx)
                pitch = np.clip(-ax * 0.05, -0.3, 0.3)
                roll = np.clip(ay * 0.05, -0.3, 0.3)
                w_yaw = (vx * ay - vy * ax) / (vx ** 2 + vy ** 2 + 1e-6)
                w_vec = np.array([0.0, 0.0, w_yaw])

            elif self.flight_pattern == "rectangle":
                # Smooth rectangular circuit
                period = self.total_time_sec / 2.0
                phase = (t % period) / period
                if phase < 0.25:
                    s = phase / 0.25
                    x = -3.0 + 6.0 * s; y = -3.0; vx = 6.0 / (period * 0.25); vy = 0.0; yaw = 0.0
                elif phase < 0.5:
                    s = (phase - 0.25) / 0.25
                    x = 3.0; y = -3.0 + 6.0 * s; vx = 0.0; vy = 6.0 / (period * 0.25); yaw = np.pi / 2
                elif phase < 0.75:
                    s = (phase - 0.5) / 0.25
                    x = 3.0 - 6.0 * s; y = 3.0; vx = -6.0 / (period * 0.25); vy = 0.0; yaw = np.pi
                else:
                    s = (phase - 0.75) / 0.25
                    x = -3.0; y = 3.0 - 6.0 * s; vx = 0.0; vy = -6.0 / (period * 0.25); yaw = -np.pi / 2

                z = 1.5 + 0.2 * np.sin(2 * np.pi * t / period)
                vz = 0.2 * (2 * np.pi / period) * np.cos(2 * np.pi * t / period)
                ax, ay, az = 0.0, 0.0, 0.0
                roll, pitch = 0.0, 0.0
                w_vec = np.array([0.0, 0.0, 0.0])

            else:  # Spiral
                r = 0.5 + 2.5 * (t / self.total_time_sec)
                theta = 3 * 2 * np.pi * (t / self.total_time_sec)
                x = r * np.cos(theta)
                y = r * np.sin(theta)
                z = 1.0 + 1.5 * (t / self.total_time_sec)
                vx = -r * 6 * np.pi / self.total_time_sec * np.sin(theta)
                vy = r * 6 * np.pi / self.total_time_sec * np.cos(theta)
                vz = 1.5 / self.total_time_sec
                ax, ay, az = 0.0, 0.0, 0.0
                yaw = theta + np.pi / 2
                roll, pitch = 0.0, 0.0
                w_vec = np.array([0.0, 0.0, 6 * np.pi / self.total_time_sec])

            rot = R_scipy.from_euler('xyz', [roll, pitch, yaw])
            quat = rot.as_quat()  # [qx, qy, qz, qw]

            self.gt_timestamps.append(t)
            self.gt_positions.append(np.array([x, y, z]))
            self.gt_orientations.append(quat)
            self.gt_velocities.append(np.array([vx, vy, vz]))
            self.gt_accelerations.append(np.array([ax, ay, az]))
            self.gt_angular_velocities.append(w_vec)

        self.gt_positions = np.array(self.gt_positions)
        self.gt_orientations = np.array(self.gt_orientations)
        self.gt_velocities = np.array(self.gt_velocities)
        self.gt_accelerations = np.array(self.gt_accelerations)
        self.gt_angular_velocities = np.array(self.gt_angular_velocities)

    def get_frame(self, frame_idx: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float, float]:
        """
        Synthesize camera image, noisy IMU acceleration, noisy IMU angular velocity, rangefinder altitude, and timestamp.

        Returns:
            image: (H, W, 3) BGR rendered camera frame
            accel_m: (3,) Noisy body-frame linear acceleration (m/s^2)
            gyro_m: (3,) Noisy body-frame angular velocity (rad/s)
            range_z: Measured altitude above floor (m)
            timestamp: Time in seconds
        """
        idx = max(0, min(frame_idx, self.total_frames - 1))
        t = self.gt_timestamps[idx]

        pos_w = self.gt_positions[idx]
        quat_w = self.gt_orientations[idx]
        accel_w = self.gt_accelerations[idx]
        omega_w = self.gt_angular_velocities[idx]

        R_body_to_world = R_scipy.from_quat(quat_w).as_matrix()
        R_world_to_body = R_body_to_world.T

        # Camera is mounted forward: transform body to camera frame
        # Camera optical frame: X-right, Y-down, Z-forward
        R_body_to_cam = np.array([
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
            [1.0, 0.0, 0.0]
        ], dtype=np.float64)

        R_world_to_cam = R_body_to_cam @ R_world_to_body
        t_cam_w = -R_world_to_cam @ pos_w.reshape(3, 1)

        # Render 2D image from 3D landmarks
        img = np.zeros((self.h, self.w, 3), dtype=np.uint8)
        # Background gradient
        img[:, :] = (35, 30, 30)

        # Draw grid room perspective lines
        # Transform 3D points to camera frame
        pts_cam = (R_world_to_cam @ self.world_landmarks.T + t_cam_w).T

        # Keep points in front of camera
        valid = pts_cam[:, 2] > 0.3
        pts_valid = pts_cam[valid]

        if len(pts_valid) > 0:
            # Perspective projection
            u = (self.fx * (pts_valid[:, 0] / pts_valid[:, 2]) + self.cx).astype(int)
            v = (self.fy * (pts_valid[:, 1] / pts_valid[:, 2]) + self.cy).astype(int)

            in_frame = (u >= 0) & (u < self.w) & (v >= 0) & (v < self.h)
            u_in = u[in_frame]
            v_in = v[in_frame]
            z_in = pts_valid[in_frame, 2]

            # Render keypoint markers (simulate textured wall features)
            for x_px, y_px, depth in zip(u_in, v_in, z_in):
                # Brightness attenuation with depth
                intensity = int(np.clip(255 - depth * 18, 50, 255))
                size = max(2, int(6.0 / (depth + 0.1)))
                cv2.circle(img, (x_px, y_px), size, (intensity, intensity, intensity), -1)
                # Add cross pattern for sharp corners
                cv2.drawMarker(img, (x_px, y_px), (intensity, intensity // 2, 0), cv2.MARKER_CROSS, size * 2, 1)

        # Add image sensor noise
        noise = np.random.normal(0, 3.0, (self.h, self.w, 3)).astype(np.int16)
        img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)

        # Generate realistic IMU data
        # Accel measures specific force: a_b = R_w_b * (a_w - g_w)
        g_w = np.array([0.0, 0.0, -9.80665])
        a_specific_force_b = R_world_to_body @ (accel_w - g_w)
        accel_m = a_specific_force_b + np.random.normal(0, self.accel_noise_std, 3)

        # Gyro measures body angular rate
        gyro_m = (R_world_to_body @ omega_w) + np.random.normal(0, self.gyro_noise_std, 3)

        # Rangefinder measures distance to floor (pos_w[2])
        range_z = pos_w[2] + np.random.normal(0, self.range_noise_std)

        return img, accel_m, gyro_m, range_z, t
