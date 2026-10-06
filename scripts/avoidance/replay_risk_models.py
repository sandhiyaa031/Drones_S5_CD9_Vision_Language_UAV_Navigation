#!/usr/bin/env python3
"""Compare UAV motion models in the risk estimator on recorded flights.

Replays the logged detections through the tracker and predictor (as
scripts/perception/evaluate_risk.py does) and assesses every frame with
    cv        constant UAV velocity (Phase 5)
    ca        constant acceleration (PX4 vehicle_local_position ax/ay/az)
    response  the calibrated response model flying to the setpoint that the
              flight controller had commanded at that time
UAV state, acceleration and setpoint come from the flight's own ULog. Ground
truth is used only to score: when did each model first raise WARNING or
CRITICAL for a box, relative to the box reaching the UAV's altitude.
"""

import argparse
import dataclasses
import json
import os
import sys

import numpy as np
from pyulog import ULog

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'perception'))
import evaluate_avoidance as ev  # noqa: E402
import replay_tracker as rt  # noqa: E402

from uav_autonomy.debris_avoidance import (  # noqa: E402
    AvoidanceParams, simulate_response)
from uav_autonomy.debris_prediction import (  # noqa: E402
    DebrisPredictor, PredictorParams, TrackState)
from uav_autonomy.debris_risk import (  # noqa: E402
    Knot, RiskParams, STATE_NAMES, assess)
from uav_autonomy.debris_tracking import (  # noqa: E402
    DebrisTracker, Measurement, TrackerParams)

ALERT = ('WARNING', 'CRITICAL')


def load_uav(run_dir):
    u = ULog(os.path.join(run_dir, 'flight.ulg'),
             ['vehicle_local_position', 'trajectory_setpoint'])
    lp = u.get_dataset('vehicle_local_position').data
    sp = u.get_dataset('trajectory_setpoint').data
    return {'t': lp['timestamp_sample'] * 1e-6,
            'p': np.c_[lp['x'], lp['y'], lp['z']],
            'v': np.c_[lp['vx'], lp['vy'], lp['vz']],
            'a': np.c_[lp['ax'], lp['ay'], lp['az']],
            'ts': sp['timestamp'] * 1e-6,
            'sp': np.c_[sp['position[0]'], sp['position[1]'],
                        sp['position[2]']]}


def state_at(uav, t, window=0.10):
    i = int(np.searchsorted(uav['t'], t))
    if i <= 0 or i >= len(uav['t']):
        return None
    w = (t - uav['t'][i - 1]) / (uav['t'][i] - uav['t'][i - 1])
    p = (1 - w) * uav['p'][i - 1] + w * uav['p'][i]
    v = (1 - w) * uav['v'][i - 1] + w * uav['v'][i]
    sel = (uav['t'] > t - window) & (uav['t'] <= t)
    a = uav['a'][sel].mean(axis=0) if sel.any() else np.zeros(3)
    j = max(int(np.searchsorted(uav['ts'], t)) - 1, 0)
    return p, v, a, uav['sp'][j]


def replay(run_dir, models, risk_params, speed_limit=0.0):
    """{model: {track_id: [(stamp, state, d_min, position)]}}."""
    tracker = DebrisTracker(TrackerParams())
    predictor = DebrisPredictor(PredictorParams())
    uav = load_uav(run_dir)
    plan = AvoidanceParams()
    out = {m: {} for m in models}
    for line in open(os.path.join(run_dir, 'detections.jsonl')):
        try:
            d = json.loads(line)['d']
        except (json.JSONDecodeError, KeyError):
            continue
        meas = [Measurement(np.array(i['p_local'], dtype=float),
                            i['size_m'], i['confidence'],
                            bool(i.get('touches_border', False)))
                for i in d['items'] if i['p_local'] is not None]
        tracks = tracker.step(d['stamp'], meas)
        if tracks is None:
            continue
        state = state_at(uav, d['stamp'])
        if state is None:
            continue
        p, v, a, setpoint = state
        by_id = {tr.id: tr for tr in tracks}
        preds = predictor.update([TrackState(
            tr.id, tr.stamp, tr.x.copy(), tr.P.copy(), tr.status,
            tr.time_since_update) for tr in tracks])
        response = None
        for pred in preds:
            tr = by_id[pred.track_id]
            knots = [Knot(0.0, tr.x[:3], tr.x[3:], tr.P[:3, :3])]
            knots += [Knot(pt.horizon, pt.x[:3], pt.x[3:], pt.P[:3, :3])
                      for pt in pred.points]
            for model in models:
                kw = {}
                params = risk_params
                if model == 'ca':
                    params = dataclasses.replace(
                        risk_params,
                        uav_motion_model='constant_acceleration')
                    kw['uav_a'] = a
                elif model == 'response':
                    if response is None:
                        times, pos, _ = simulate_response(
                            p, v, [setpoint], plan, setpoint,
                            speed_limits=[speed_limit], a0=a)
                        response = (times, pos[0])
                    kw['uav_traj'] = response
                r = assess(tr.id, tr.stamp, tr.status, knots, p, v,
                           tr.size_m, params, **kw)
                out[model].setdefault(tr.id, []).append(
                    (d['stamp'], STATE_NAMES[r.state], r.d_min,
                     tr.x[:3].copy()))
    return out


def score(run_dir, models, risk_params, speed_limit=0.0):
    """Per released box: first alert lead time under each model."""
    boxes, _, _ = ev.truth(run_dir)
    debris = rt.load_truth(run_dir, 'x500_rescue_0', 1.0)[0]
    rep = replay(run_dir, models, risk_params, speed_limit)
    rows = []
    for box in boxes:
        obj = debris[box['name']]
        t_pass = box['t_pass_uav_altitude']
        if t_pass is None:
            continue
        row = {'box': box['index'], 'label': box['label'],
               'class': box['whole_life']['class'],
               'min_surface_m': box['whole_life']['min_surface_m']}
        for model in models:
            first = None
            for track_id, frames in rep[model].items():
                for stamp, state, d_min, pos in frames:
                    if not (obj['t'][0] <= stamp <= t_pass):
                        continue
                    truth = np.array([np.interp(stamp, obj['t'],
                                                obj['p'][:, k])
                                      for k in range(3)])
                    if np.linalg.norm(pos - truth) > 1.5:
                        continue        # another object's track
                    if state in ALERT and (first is None or stamp < first):
                        first = stamp
            row[model] = (None if first is None
                          else round(t_pass - first, 3))
        rows.append(row)
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('runs', nargs='+')
    parser.add_argument('--models', default='cv,ca,response')
    parser.add_argument('--accel-hold', type=float)
    parser.add_argument('--speed-limit', type=float, default=0.0)
    parser.add_argument('--output')
    args = parser.parse_args()
    params = RiskParams()
    if args.accel_hold is not None:
        params = dataclasses.replace(params,
                                     uav_accel_hold_s=args.accel_hold)
    models = args.models.split(',')
    report = {}
    for run in args.runs:
        rows = score(run.rstrip('/'), models, params, args.speed_limit)
        report[run] = rows
        print(run)
        for row in rows:
            print('  box', row['box'], row['class'],
                  f"{row['min_surface_m']:.2f} m",
                  ' '.join(f"{m}={row[m]}" for m in models),
                  f"[{row['label']}]")
    if args.output:
        json.dump(report, open(args.output, 'w'), indent=1)


if __name__ == '__main__':
    main()
