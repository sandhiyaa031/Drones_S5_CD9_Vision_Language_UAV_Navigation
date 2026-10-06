#!/usr/bin/env python3
"""Score debris trajectory predictions against Gazebo ground truth (offline).

Default: replay recorded detections (detections.jsonl) through the final
tracker and through the predictor with each model, then score. With
--logged the predictions logged from the live node (predictions.jsonl) are
scored instead.

A prediction made at time t for horizon h is compared with the true object
at t + h, only while that object is still in free fall at t + h (before its
first impact). Ground truth is used for scoring only.
"""

import argparse
import json
import os
import sys
import time
from collections import defaultdict

import numpy as np
from scipy.optimize import linear_sum_assignment

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import replay_tracker as rt  # noqa: E402

from uav_autonomy.debris_prediction import (  # noqa: E402
    DebrisPredictor, MODEL_NAMES, PredictorParams, TrackState)
from uav_autonomy.debris_tracking import (  # noqa: E402
    DebrisTracker, Measurement, TrackerParams)

MODELS = ('cv', 'ca', 'ballistic', 'auto')
SPEED_BINS = ((0, 4), (4, 8), (8, 99))
GAP_BINS = ((0, 0.05), (0.05, 0.15), (0.15, 9))


def replay(run_dir):
    """Yield (stamp, frame_dt, {model: [prediction dicts]}, proc_ms)."""
    tracker = DebrisTracker(TrackerParams())
    predictors = {m: DebrisPredictor(PredictorParams(model=m))
                  for m in MODELS}
    last = None
    with open(os.path.join(run_dir, 'detections.jsonl')) as handle:
        for line in handle:
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
            states = [TrackState(tr.id, tr.stamp, tr.x.copy(), tr.P.copy(),
                                 tr.status, tr.time_since_update)
                      for tr in tracks]
            out, proc = {}, {}
            for m, predictor in predictors.items():
                started = time.perf_counter()
                preds = predictor.update(states)
                proc[m] = (time.perf_counter() - started) * 1e3
                out[m] = [to_dict(p) for p in preds]
            dt = d['stamp'] - last if last is not None else 0.0
            last = d['stamp']
            yield d['stamp'], dt, out, proc


def to_dict(pred):
    return {'id': pred.track_id, 'status': pred.status,
            'tsu': pred.time_since_update,
            'model': MODEL_NAMES[pred.model],
            'p': pred.points[0].x[:3] - 0,   # placeholder, replaced below
            'points': [{'h': pt.horizon, 'p': pt.x[:3].tolist(),
                        'v': pt.x[3:].tolist(),
                        'sigma': float(np.sqrt(np.trace(pt.P[:3, :3])))}
                       for pt in pred.points],
            'state_p': None}


def logged(run_dir):
    last = None
    with open(os.path.join(run_dir, 'predictions.jsonl')) as handle:
        for line in handle:
            try:
                d = json.loads(line)
            except json.JSONDecodeError:
                continue
            dt = d['stamp'] - last if last is not None else 0.0
            last = d['stamp']
            yield (d['stamp'], dt, {'live': d['predictions']},
                   {'live': d.get('processing_ms', 0.0)})


def rmse(v):
    v = np.asarray(v, dtype=float)
    return round(float(np.sqrt(np.mean(v ** 2))), 4) if len(v) else None


def score(run_dirs, source, min_altitude):
    acc = defaultdict(lambda: defaultdict(lambda: defaultdict(list)))
    counts = defaultdict(lambda: defaultdict(lambda: {'made': 0,
                                                      'scored': 0,
                                                      'after_impact': 0}))
    proc = defaultdict(list)
    chosen = defaultdict(lambda: defaultdict(int))
    frames_with_tracks = defaultdict(int)
    for run_dir in run_dirs:
        debris, hover, _, _ = rt.load_truth(run_dir, 'x500_rescue_0',
                                            min_altitude)
        for t, dt, by_model, ms in source(run_dir):
            for m, value in ms.items():
                proc[m].append(value)
            if not hover[0] <= t <= hover[1]:
                continue
            truth = {n: rt.truth_at(o, t) for n, o in debris.items()}
            truth = {n: v for n, v in truth.items()
                     if v is not None and t <= debris[n]['fall_end']}
            for m, preds in by_model.items():
                if not preds:
                    continue
                frames_with_tracks[m] += 1
                names = list(truth)
                # the prediction's first point minus motion gives no state;
                # match on the 0.1 s point against truth at t + 0.1
                pair = {}
                if names:
                    cost = np.full((len(names), len(preds)), 9.0)
                    for i, n in enumerate(names):
                        tr = rt.truth_at(debris[n], t + preds[0]['points'][
                            0]['h'])
                        if tr is None:
                            continue
                        for j, pr in enumerate(preds):
                            cost[i, j] = np.linalg.norm(
                                np.array(pr['points'][0]['p']) - tr[0])
                    for i, j in zip(*linear_sum_assignment(cost)):
                        if cost[i, j] <= 0.8:
                            pair[j] = names[i]
                for j, pr in enumerate(preds):
                    chosen[m][pr['model']] += 1
                    name = pair.get(j)
                    for pt in pr['points']:
                        h = pt['h']
                        c = counts[m][h]
                        c['made'] += 1
                        if name is None:
                            continue
                        obj = debris[name]
                        tr = rt.truth_at(obj, t + h)
                        if tr is None or t + h > obj['fall_end']:
                            c['after_impact'] += 1
                            continue
                        c['scored'] += 1
                        p_true, v_true = tr
                        p_hat, v_hat = np.array(pt['p']), np.array(pt['v'])
                        e_surf = rt.surface_distance(p_hat, p_true,
                                                     obj['size'])
                        speed = float(np.linalg.norm(truth[name][1]))
                        a = acc[m][h]
                        a['surface'].append(e_surf)
                        a['centre'].append(float(np.linalg.norm(
                            p_hat - p_true)))
                        a['vel'].append(float(np.linalg.norm(
                            v_hat - v_true)))
                        a['vz'].append(float(v_hat[2] - v_true[2]))
                        a['pz'].append(float(p_hat[2] - p_true[2]))
                        a['speed'].append(speed)
                        a['gap'].append(dt)
                        a['coast'].append(pr['tsu'] > 0)
                        a['sigma'].append(pt.get('sigma', np.nan))
    report = {}
    for m, by_h in acc.items():
        rows = []
        for h in sorted(by_h):
            a = {k: np.array(v) for k, v in by_h[h].items()}
            c = counts[m][h]
            row = {
                'horizon_s': h, 'predictions_made': c['made'],
                'scored': c['scored'],
                'truth_already_impacted': c['after_impact'],
                'position_rmse_surface_m': rmse(a['surface']),
                'position_rmse_centre_m': rmse(a['centre']),
                'position_error_surface_median_m': round(float(np.median(
                    a['surface'])), 4),
                'position_error_surface_p95_m': round(float(np.percentile(
                    a['surface'], 95)), 4),
                'vertical_position_bias_m': round(float(a['pz'].mean()), 4),
                'velocity_rmse_m_s': rmse(a['vel']),
                'vertical_velocity_bias_m_s': round(float(a['vz'].mean()),
                                                    4),
                'by_speed': {
                    f'{lo}-{hi if hi < 99 else "+"} m/s': {
                        'n': int(((a['speed'] >= lo)
                                  & (a['speed'] < hi)).sum()),
                        'rmse_surface_m': rmse(a['surface'][
                            (a['speed'] >= lo) & (a['speed'] < hi)])}
                    for lo, hi in SPEED_BINS},
                'by_preceding_frame_gap': {
                    f'{int(lo * 1e3)}-{int(hi * 1e3) if hi < 9 else "+"} ms':
                    {'n': int(((a['gap'] >= lo) & (a['gap'] < hi)).sum()),
                     'rmse_surface_m': rmse(a['surface'][
                         (a['gap'] >= lo) & (a['gap'] < hi)])}
                    for lo, hi in GAP_BINS},
                'updated_state_rmse_surface_m': rmse(
                    a['surface'][~a['coast']]),
                'coasting_state_rmse_surface_m': rmse(
                    a['surface'][a['coast']]),
                'coasting_n': int(a['coast'].sum()),
                'centre_error_within_2_sigma_fraction': round(float(np.mean(
                    a['centre'] <= 2 * a['sigma'])), 3)
                if np.all(np.isfinite(a['sigma'])) else None,
            }
            rows.append(row)
        ms = np.array(proc[m]) if proc[m] else np.array([0.0])
        report[m] = {
            'frames_with_predictions': frames_with_tracks[m],
            'models_used': dict(chosen[m]),
            'processing_ms': {'median': round(float(np.median(ms)), 4),
                              'p95': round(float(np.percentile(ms, 95)), 4),
                              'max': round(float(ms.max()), 4)},
            'horizons': rows}
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('run_dirs', nargs='+')
    parser.add_argument('--logged', action='store_true')
    parser.add_argument('--min-altitude', type=float, default=2.8)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    report = score(args.run_dirs, logged if args.logged else replay,
                   args.min_altitude)
    report['runs'] = [os.path.basename(os.path.abspath(r))
                      for r in args.run_dirs]
    with open(args.output, 'w') as handle:
        json.dump(report, handle, indent=1)
    for m, r in report.items():
        if m == 'runs':
            continue
        print(f"== {m}  models used {r['models_used']}  "
              f"proc ms {r['processing_ms']}")
        for row in r['horizons']:
            print(f"  h={row['horizon_s']:<4} n={row['scored']:<5} "
                  f"impacted={row['truth_already_impacted']:<5} "
                  f"rmse surf={row['position_rmse_surface_m']} "
                  f"centre={row['position_rmse_centre_m']} "
                  f"z bias={row['vertical_position_bias_m']} "
                  f"vel rmse={row['velocity_rmse_m_s']} "
                  f"vz bias={row['vertical_velocity_bias_m_s']} "
                  f"2sig={row['centre_error_within_2_sigma_fraction']}")


if __name__ == '__main__':
    main()
