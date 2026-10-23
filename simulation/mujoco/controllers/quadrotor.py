"""Quadrotor plant helper and cascaded flight controller for the MuJoCo sim.

This controller is NOT PX4. It is a standalone position -> velocity ->
attitude -> body-torque cascade (thrust-vector / geometric attitude error)
with a control allocation to four motor thrusts and a first-order motor lag.
It uses the simulator's own state (no sensor noise, no estimator) unless noise
is added by the caller. The vehicle moves only through the four motor forces
and reaction torques applied by MuJoCo; nothing is teleported.

Frames: world ENU (z up); body FLU (x forward, y left, z up).
"""

from dataclasses import dataclass, field
import math

import mujoco
import numpy as np

G = 9.81
MASS = 2.064
INERTIA = np.array([0.02167, 0.02167, 0.04])
T_MAX = 8.55                      # N per motor (8.54858e-6 * 1000^2)
TAU_UP, TAU_DOWN = 0.0125, 0.025  # motor lag [s], as in the Gazebo model
# rotor positions (x, y) in FLU and reaction-torque signs (same as the MJCF)
ROTORS = np.array([[0.13, 0.22], [0.13, -0.22], [-0.13, 0.20], [-0.13, -0.20]])
YAW_SIGN = np.array([-1.0, 1.0, 1.0, -1.0])
KM = 0.016


def quat_to_rot(q):
    r = np.zeros(9)
    mujoco.mju_quat2Mat(r, q)
    return r.reshape(3, 3)


def vee(m):
    return np.array([m[2, 1], m[0, 2], m[1, 0]])


@dataclass
class Gains:
    kp_pos: float = 1.1           # position error -> velocity [1/s]
    v_max_xy: float = 4.0
    v_max_up: float = 2.5
    v_max_down: float = 1.5
    kp_vel: float = 3.2           # velocity error -> acceleration [1/s]
    ki_vel: float = 0.5
    i_limit: float = 2.0          # [m/s2]
    a_max_xy: float = 4.5
    tilt_max: float = math.radians(32.0)
    k_att: float = 130.0          # attitude error -> angular acceleration
    k_rate: float = 20.0
    yaw_rate_max: float = 1.0


class Quadrotor:
    """MuJoCo model + data of the vehicle, with the motor lag."""

    def __init__(self, model, data, body='uav'):
        self.m, self.d = model, data
        self.body = model.body(body).id
        self.root = model.joint('root').qposadr[0]
        self.vroot = model.joint('root').dofadr[0]
        self.thrust = np.zeros(4)           # actual motor thrust [N]
        self.cmd = np.zeros(4)
        self.site_lidar = model.site('lidar').id

    def state(self):
        d = self.d
        p = d.qpos[self.root:self.root + 3].copy()
        q = d.qpos[self.root + 3:self.root + 7].copy()
        v = d.qvel[self.vroot:self.vroot + 3].copy()
        w_body = d.qvel[self.vroot + 3:self.vroot + 6].copy()   # body frame
        return p, quat_to_rot(q), v, w_body

    def apply(self, thrust_cmd, dt):
        """Motor lag, then the motor forces for the next physics step."""
        self.cmd = np.clip(thrust_cmd, 0.0, T_MAX)
        tau = np.where(self.cmd > self.thrust, TAU_UP, TAU_DOWN)
        self.thrust += (self.cmd - self.thrust) * (1.0 - np.exp(-dt / tau))
        self.d.ctrl[:4] = self.thrust
        # velocity-proportional rotor drag (about 8.1e-5 * w per rotor)
        _, rot, v, _ = self.state()
        w_rot = np.sqrt(np.maximum(self.thrust, 0.0) / 8.54858e-6)
        c = 8.06428e-5 * float(w_rot.sum())
        v_body = rot.T @ v
        f_body = -c * np.array([v_body[0], v_body[1], 0.0])
        self.d.xfrc_applied[self.body, :3] = rot @ f_body


class CascadedController:
    def __init__(self, gains=None):
        self.g = gains or Gains()
        self.int_v = np.zeros(3)
        self.yaw_ref = 0.0
        # allocation: [T, tau_x, tau_y, tau_z] = A f
        self.A = np.vstack([np.ones(4), ROTORS[:, 1], -ROTORS[:, 0],
                            YAW_SIGN * KM])
        self.A_inv = np.linalg.inv(self.A)
        self.last = {}

    def reset(self):
        self.int_v[:] = 0.0

    def update(self, p, rot, v, w_body, p_ref, v_ref, yaw_ref, dt):
        """Returns the four motor thrust commands [N]."""
        g = self.g
        # position -> velocity
        v_des = g.kp_pos * (p_ref - p) + v_ref
        vxy = np.linalg.norm(v_des[:2])
        if vxy > g.v_max_xy:
            v_des[:2] *= g.v_max_xy / vxy
        v_des[2] = float(np.clip(v_des[2], -g.v_max_down, g.v_max_up))
        # velocity -> acceleration (with a limited integrator)
        e_v = v_des - v
        self.int_v += g.ki_vel * e_v * dt
        self.int_v = np.clip(self.int_v, -g.i_limit, g.i_limit)
        a_des = g.kp_vel * e_v + self.int_v
        axy = np.linalg.norm(a_des[:2])
        if axy > g.a_max_xy:
            a_des[:2] *= g.a_max_xy / axy
        f_des = MASS * (a_des + np.array([0.0, 0.0, G]))
        # tilt limit
        horiz = np.linalg.norm(f_des[:2])
        max_h = max(f_des[2], 1.0) * math.tan(g.tilt_max)
        if horiz > max_h:
            f_des[:2] *= max_h / horiz
        z_b_des = f_des / np.linalg.norm(f_des)
        thrust = float(f_des @ rot[:, 2])
        thrust = max(thrust, 0.0)
        # desired attitude: z axis from the force, heading from yaw_ref
        x_c = np.array([math.cos(yaw_ref), math.sin(yaw_ref), 0.0])
        y_b = np.cross(z_b_des, x_c)
        y_b /= max(np.linalg.norm(y_b), 1e-9)
        x_b = np.cross(y_b, z_b_des)
        r_des = np.column_stack([x_b, y_b, z_b_des])
        e_r = 0.5 * vee(r_des.T @ rot - rot.T @ r_des)
        tau = INERTIA * (-g.k_att * e_r - g.k_rate * w_body) \
            + np.cross(w_body, INERTIA * w_body)
        f = self.A_inv @ np.array([thrust, tau[0], tau[1], tau[2]])
        f = np.clip(f, 0.0, T_MAX)
        self.last = {'a_des': a_des, 'thrust': thrust, 'e_r': e_r}
        return f
