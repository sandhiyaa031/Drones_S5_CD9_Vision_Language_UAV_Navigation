#!/usr/bin/env python3
"""Fit the planner's response model to measured setpoint steps (offline).

Reads a flight log containing scripted steps (scripts/avoidance/
step_command.py), fits the lag, kp, kv and the acceleration limit of
uav_autonomy.debris_avoidance.simulate_response, and reports the model
error for every step, including those not used in the fit.
"""

import argparse
import dataclasses
import json

import numpy as np
from pyulog import ULog
from scipy.optimize import least_squares

from uav_autonomy.debris_avoidance import AvoidanceParams, simulate_response


def load(path):
    u = ULog(path, ['vehicle_local_position', 'trajectory_setpoint'])
    lp = u.get_dataset('vehicle_local_position').data
    sp = u.get_dataset('trajectory_setpoint').data
    t = lp['timestamp_sample'] * 1e-6
    p = np.c_[lp['x'], lp['y'], lp['z']]
    v = np.c_[lp['vx'], lp['vy'], lp['vz']]
    ts = sp['timestamp'] * 1e-6
    s = np.c_[sp['position[0]'], sp['position[1]'], sp['position[2]']]
    steps = []
    for j in np.where(np.linalg.norm(np.diff(s, axis=0), axis=1) > 0.5)[0]:
        t0 = ts[j + 1]
        if (t0 < t[0] + 12 or t0 > t[-1] - 8
                or np.linalg.norm(s[j + 1] - s[j]) > 4.5):
            continue            # takeoff and landing setpoint changes
        i0 = np.searchsorted(t, t0)
        steps.append({'t0': float(t0), 'target': s[j + 1], 'p0': p[i0],
                      'previous': s[j],
                      'v0': v[i0], 'delta': s[j + 1] - s[j],
                      'speed0': float(np.linalg.norm(v[i0])),
                      'from_rest': bool(np.linalg.norm(v[i0]) < 0.4)})
    return t, p, steps


def model_error(params, t, p, step, span):
    times, pos, _ = simulate_response(step['p0'], step['v0'],
                                      [step['target']], params,
                                      previous_target=step['previous'])
    keep = times <= span
    measured = np.array([np.interp(step['t0'] + times[keep], t, p[:, k])
                         for k in range(3)]).T
    return pos[0, keep] - measured


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('ulog')
    parser.add_argument('--span', type=float, default=1.5)
    parser.add_argument('--output')
    args = parser.parse_args()
    t, p, steps = load(args.ulog)
    base = AvoidanceParams()
    horizontal = [s for s in steps if abs(s['delta'][2]) < 0.1]
    vertical = [s for s in steps if abs(s['delta'][2]) >= 0.1]

    def with_xy(x):
        return dataclasses.replace(base, tau_xy_s=x[0], kp_xy=x[1],
                                   kv_xy=x[2], a_xy_max=x[3])

    def with_z(x, b):
        return dataclasses.replace(b, tau_z_s=x[0], kp_z=x[1], kv_z=x[2],
                                   a_z_max=x[3])

    # Every step is used, including those started while moving.
    fit_xy = least_squares(
        lambda x: np.concatenate([
            model_error(with_xy(x), t, p, s, args.span)[:, :2].ravel()
            for s in horizontal]),
        [0.25, 0.95, 2.0, 6.0], bounds=([0.02, 0.3, 0.5, 2.0],
                                        [0.6, 3.0, 8.0, 15.0]))
    fitted = with_xy(fit_xy.x)
    fit_z = least_squares(
        lambda x: np.concatenate([
            model_error(with_z(x, fitted), t, p, s, args.span)[:, 2]
            for s in vertical]),
        [0.15, 1.0, 4.0, 6.0], bounds=([0.02, 0.3, 0.5, 1.0],
                                       [0.6, 3.0, 12.0, 15.0]))
    fitted = with_z(fit_z.x, fitted)
    names = ('tau_xy_s', 'kp_xy', 'kv_xy', 'a_xy_max', 'tau_z_s', 'kp_z',
             'kv_z', 'a_z_max')
    report = {'fit': {n: round(float(v), 3) for n, v in zip(
        names, list(fit_xy.x) + list(fit_z.x))}, 'steps': []}
    for label, params in (('fitted', fitted), ('defaults', base)):
        for s in steps:
            unit = s['delta'] / np.linalg.norm(s['delta'])
            err = model_error(params, t, p, s, args.span) @ unit
            times = np.arange(err.size) * params.model_dt_s
            report['steps'].append({
                'model': label, 't0': round(s['t0'], 2),
                'delta': [round(float(x), 2) for x in s['delta']],
                'from_rest': s['from_rest'],
                'along_error_at': {
                    str(h): round(float(np.interp(h, times, err)), 3)
                    for h in (0.4, 0.6, 0.8, 1.0, 1.5) if h <= args.span},
                'max_abs_m': round(float(np.abs(err).max()), 3),
                'rms_m': round(float(np.sqrt(np.mean(err ** 2))), 3)})
    text = json.dumps(report, indent=1)
    print(json.dumps(report['fit']))
    for row in report['steps']:
        print(row['model'], row['t0'], row['delta'], row['from_rest'],
              row['along_error_at'], 'max', row['max_abs_m'])
    if args.output:
        open(args.output, 'w').write(text + '\n')


if __name__ == '__main__':
    main()
