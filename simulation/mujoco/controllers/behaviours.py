"""Flight behaviours built on the cascaded controller: takeoff, goto, land.

A behaviour is a function of the Sim that returns the next Setpoint; the
reference position is a "carrot" that moves toward the target at a capped
speed and is not allowed to run more than `lead` metres ahead of the vehicle.
"""

import math

import numpy as np

from controllers.flight_sim import Setpoint


class Carrot:
    def __init__(self, p0, lead=2.5):
        self.p = np.array(p0, float)
        self.lead = lead

    def toward(self, sim, target, speed, yaw=0.0):
        p_vehicle = sim.state()[0]
        target = np.asarray(target, float)
        to = target - self.p
        dist = float(np.linalg.norm(to))
        v_ref = np.zeros(3)
        if dist > 1e-6 and np.linalg.norm(self.p - p_vehicle) < self.lead:
            step = min(speed * sim.dt_ctrl, dist)
            self.p = self.p + to / dist * step
            v_ref = to / dist * speed if dist > speed * 0.3 else \
                to / dist * speed * dist / max(speed * 0.3, 1e-6)
        return Setpoint(self.p.copy(), v_ref, yaw)


def takeoff(sim, altitude, rate=1.5, hold=2.0, timeout=40.0):
    """Climb at `rate` to `altitude`, then hold for `hold` seconds."""
    carrot = Carrot(sim.state()[0], lead=1.5)
    state = {'t_hold': None}
    target = np.array([carrot.p[0], carrot.p[1], altitude])

    def provider(s):
        return carrot.toward(s, target, rate)

    def done(s):
        if abs(s.state()[0][2] - altitude) < 0.15:
            if state['t_hold'] is None:
                state['t_hold'] = s.t
            return s.t - state['t_hold'] >= hold
        state['t_hold'] = None
        return False
    ok = sim.run(provider, done, timeout)
    return ok, carrot


def goto(sim, carrot, waypoints, speed, tol=0.5, timeout=120.0,
         on_tick=None, yaw=0.0):
    """Fly through waypoints (each reached within tol and slow enough)."""
    idx = {'i': 0}
    reached = []

    def provider(s):
        if on_tick:
            on_tick(s)
        return carrot.toward(s, waypoints[idx['i']], speed, yaw)

    def done(s):
        p, _, v, _ = s.state()
        if np.linalg.norm(p - waypoints[idx['i']]) < tol:
            reached.append((s.t, p.copy()))
            idx['i'] += 1
        return idx['i'] >= len(waypoints)
    ok = sim.run(provider, done, timeout)
    return ok, reached


def land(sim, carrot, xy, rate=0.7, timeout=40.0):
    """Descend over xy, cut the motors on ground contact."""
    state = {'t_down': None}

    def provider(s):
        if s.ground_contact:
            if state['t_down'] is None:
                state['t_down'] = s.t
            return Setpoint(s.state()[0], motors_off=True)
        z = s.state()[0][2]
        slow = rate if z > 1.0 else 0.35
        target = np.array([xy[0], xy[1], -0.3])
        return carrot.toward(s, target, slow)

    def done(s):
        return state['t_down'] is not None and s.t - state['t_down'] > 1.0
    ok = sim.run(provider, done, timeout)
    return ok
