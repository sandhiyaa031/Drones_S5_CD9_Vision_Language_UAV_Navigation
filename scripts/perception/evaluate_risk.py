#!/usr/bin/env python3
"""Score collision-risk estimates against Gazebo ground truth (offline).

Default: replay recorded detections through tracker, predictor and risk
estimator, taking the UAV state from PX4's own log (flight.ulg, the same data
the node receives as vehicle_odometry). With --logged, the risk.jsonl written
by the live node is scored instead.

Ground truth (poses.csv) is used only here, to decide what really happened:
for each released box, the smallest distance between the UAV centre and the
box surface during its fall.

    COLLISION   surface distance <= uav_radius
    NEAR_MISS   surface distance <= uav_radius + margin
    SAFE        otherwise
"""

import argparse
import json
import os
import sys
import time
from collections import Counter, defaultdict

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import replay_tracker as rt  # noqa: E402

from uav_autonomy.debris_prediction import (  # noqa: E402
    DebrisPredictor, PredictorParams, TrackState)
from uav_autonomy.debris_risk import (  # noqa: E402
    Knot, RiskParams, STATE_NAMES, UavStateBuffer, assess, ballistic_knots)
from uav_autonomy.debris_tracking import (  # noqa: E402
    DebrisTracker, Measurement, TrackerParams)

P = RiskParams()
ORDER = ['none', 'SAFE', 'WATCH', 'POTENTIAL_THREAT', 'WARNING', 'CRITICAL']
RANK = {s: i for i, s in enumerate(ORDER)}
ALERT = ('WARNING', 'CRITICAL')


def uav_from_ulog(run_dir):
    from pyulog import ULog
    log = ULog(os.path.join(run_dir, 'flight.ulg'),
               message_name_filter_list=['vehicle_local_position'])
    d = log.data_list[0].data
    buf = UavStateBuffer(horizon_s=1e9)
    for i in range(len(d['timestamp'])):
        buf.add(d['timestamp_sample'][i] * 1e-6,
                [d['x'][i], d['y'][i], d['z'][i]],
                [d['vx'][i], d['vy'][i], d['vz'][i]])
    return buf


def replay(run_dir):
    """Yield per-frame dicts in the format of the node's risk_debug."""
    tracker = DebrisTracker(TrackerParams())
    predictor = DebrisPredictor(PredictorParams())
    uav = uav_from_ulog(run_dir)
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
            started = time.perf_counter()
            state = uav.at(d['stamp'])
            by_id = {tr.id: tr for tr in tracks}
            preds = predictor.update([TrackState(
                tr.id, tr.stamp, tr.x.copy(), tr.P.copy(), tr.status,
                tr.time_since_update) for tr in tracks])
            risks = []
            if state is not None:
                done = set()
                for pred in preds:
                    tr = by_id[pred.track_id]
                    done.add(tr.id)
                    knots = [Knot(0.0, tr.x[:3], tr.x[3:], tr.P[:3, :3])]
                    knots += [Knot(pt.horizon, pt.x[:3], pt.x[3:],
                                   pt.P[:3, :3]) for pt in pred.points]
                    risks.append(assess(tr.id, tr.stamp, tr.status, knots,
                                        state[0], state[1], tr.size_m, P))
                for tr in tracks:
                    if (tr.id in done or tr.status != 0
                            or tr.hits < P.potential_min_hits
                            or tr.time_since_update > 0):
                        continue
                    risks.append(assess(
                        tr.id, tr.stamp, tr.status,
                        ballistic_knots(tr.x[:3], tr.x[3:], tr.P, P),
                        state[0], state[1], tr.size_m, P, tentative=True))
            ms = (time.perf_counter() - started) * 1e3
            yield {'stamp': d['stamp'], 'processing_ms': ms,
                   'uav_valid': state is not None,
                   'uav_p': state[0].tolist() if state else None,
                   'risks': [{
                       'id': r.track_id, 'track_status': r.track_status,
                       'state': STATE_NAMES[r.state], 'tca': r.tca,
                       'd_min': r.d_min, 'collision': r.predicted_collision,
                       'intersects': r.intersects, 'lin_d': r.linear_d_min,
                       'lin_tca': r.linear_tca, 'unc': r.uncertainty_margin,
                       'p': r.path[0].tolist(), 'score': r.score}
                       for r in risks]}


def logged(run_dir):
    with open(os.path.join(run_dir, 'risk.jsonl')) as handle:
        for line in handle:
            try:
                yield json.loads(line)
            except json.JSONDecodeError:
                continue


def truth_outcomes(run_dir, min_altitude, scope='whole_life'):
    """True closest approach of every released box to the UAV."""
    debris, window, uav, origin = rt.load_truth(run_dir, 'x500_rescue_0',
                                                min_altitude)
    # UAV base_link truth in local NED
    base_z = 0.24
    t_u = uav[:, 0]
    uav_ned = np.column_stack([uav[:, 2] - origin[1], uav[:, 1] - origin[0],
                               -(uav[:, 3] + base_z - origin[2])])
    out = {}
    for name, obj in debris.items():
        # 'free_fall': up to the object's first impact with anything.
        # 'whole_life': everything, since an object deflected by structure
        # or by another box may still come near the UAV afterwards.
        if scope == 'free_fall':
            sel = obj['t'] <= obj['fall_end'] + 0.10
        else:
            sel = np.ones(len(obj['t']), dtype=bool)
        t = obj['t'][sel]
        if len(t) < 5 or not (window[0] <= t[0] <= window[1]):
            continue
        u = np.column_stack([np.interp(t, t_u, uav_ned[:, i])
                             for i in range(3)])
        centre = np.linalg.norm(obj['p'][sel] - u, axis=1)
        surface = np.array([rt.surface_distance(u[i], obj['p'][sel][i],
                                                obj['size'])
                            for i in range(len(t))])
        i = int(np.argmin(surface))
        klass = ('COLLISION' if surface[i] <= P.uav_radius_m else
                 'NEAR_MISS' if surface[i] <= P.uav_radius_m + P.margin_m
                 else 'SAFE')
        out[name] = {'t_release': float(t[0]), 't_ca': float(t[i]),
                     'after_first_impact': bool(
                         t[i] > obj['fall_end'] + 0.05),
                     'min_surface_m': float(surface[i]),
                     'min_centre_m': float(centre.min()), 'class': klass,
                     'obj': obj}
    return out, window


def score(run_dirs, source, min_altitude, scope='whole_life'):
    objects = []
    proc, frames, valid = [], 0, 0
    late_alerts, orphan_alerts, orphan_examples = 0, 0, []
    stamps = []
    for run_dir in run_dirs:
        truth, window = truth_outcomes(run_dir, min_altitude, scope)
        series = defaultdict(list)
        for frame in source(run_dir):
            t = frame['stamp']
            proc.append(frame.get('processing_ms', 0.0))
            if not window[0] <= t <= window[1]:
                continue
            frames += 1
            stamps.append(t)
            valid += bool(frame.get('uav_valid'))
            for r in frame['risks']:
                best = None
                for name, tr in truth.items():
                    st = rt.truth_at(tr['obj'], t)
                    if st is None:
                        continue
                    d = float(np.linalg.norm(np.array(r['p']) - st[0]))
                    if d <= 0.6 and (best is None or d < best[0]):
                        best = (d, name)
                if best is None:
                    if r['state'] in ALERT:
                        orphan_alerts += 1
                        if len(orphan_examples) < 8:
                            orphan_examples.append(
                                {'run': os.path.basename(run_dir),
                                 't': round(t, 3), 'state': r['state'],
                                 'd_min': round(r['d_min'], 2),
                                 'p': [round(v, 2) for v in r['p']]})
                    continue
                name = best[1]
                if t > truth[name]['t_ca'] + 0.05:
                    late_alerts += r['state'] in ALERT
                    continue
                series[name].append((t, r))
        for name, tr in truth.items():
            rows = series.get(name, [])
            states = [r['state'] for _, r in rows]
            top = max(states, key=lambda s: RANK[s]) if states else 'none'
            first_alert = next((t for t, r in rows if r['state'] in ALERT),
                               None)
            first_potential = next(
                (t for t, r in rows if r['state'] == 'POTENTIAL_THREAT'),
                None)
            first_collision_flag = next(
                (t for t, r in rows if r['collision']), None)
            ahead = [(t, r) for t, r in rows if tr['t_ca'] - t >= 0.10
                     and r['track_status'] != 0]
            collapsed = []
            for s in states:
                if not collapsed or collapsed[-1] != s:
                    collapsed.append(s)
            downgrades = sum(1 for a, b in zip(collapsed, collapsed[1:])
                             if RANK[b] < RANK[a]
                             and 'POTENTIAL_THREAT' not in (a, b))
            at_lead = {}
            for lead in (1.5, 1.0, 0.75, 0.5, 0.25):
                cands = [r for t, r in rows
                         if abs((tr['t_ca'] - t) - lead) <= 0.05]
                if cands:
                    at_lead[lead] = max((r['state'] for r in cands),
                                        key=lambda s: RANK[s])
            objects.append({
                'run': os.path.basename(os.path.abspath(run_dir)),
                'object': name, 'label': tr['obj'].get('label', ''),
                'truth_class': tr['class'],
                'closest_approach_after_first_impact':
                    tr['after_first_impact'],
                'true_min_surface_m': round(tr['min_surface_m'], 3),
                'true_min_centre_m': round(tr['min_centre_m'], 3),
                'risk_frames': len(rows), 'highest_state': top,
                'transitions': ' > '.join(collapsed),
                'downgrades_before_closest_approach': downgrades,
                'lead_time_s': round(tr['t_ca'] - first_alert, 3)
                if first_alert is not None else None,
                'potential_threat_lead_s':
                    round(tr['t_ca'] - first_potential, 3)
                    if first_potential is not None else None,
                'predicted_collision_flag': first_collision_flag is not None,
                'd_min_error_vs_centre_m': [
                    round(r['d_min'] - tr['min_centre_m'], 3)
                    for _, r in ahead],
                'd_min_error_vs_surface_m': [
                    round(r['d_min'] - tr['min_surface_m'], 3)
                    for _, r in ahead],
                'linear_d_min_error_vs_centre_m': [
                    round(r['lin_d'] - tr['min_centre_m'], 3)
                    for _, r in ahead],
                'tca_error_s': [round(t + r['tca'] - tr['t_ca'], 3)
                                for t, r in ahead],
                'state_at_lead': at_lead})

    matrix = {c: Counter() for c in ('SAFE', 'NEAR_MISS', 'COLLISION')}
    for o in objects:
        matrix[o['truth_class']][o['highest_state']] += 1
    pos = [o for o in objects if o['truth_class'] != 'SAFE']
    neg = [o for o in objects if o['truth_class'] == 'SAFE']
    tp = sum(o['highest_state'] in ALERT for o in pos)
    fn = len(pos) - tp
    fp = sum(o['highest_state'] in ALERT for o in neg)
    tn = len(neg) - fp
    col = [o for o in objects if o['truth_class'] == 'COLLISION']

    def stats(v):
        v = np.asarray(v, dtype=float)
        if not len(v):
            return None
        return {'n': int(len(v)), 'mean': round(float(v.mean()), 3),
                'median': round(float(np.median(v)), 3),
                'p05': round(float(np.percentile(v, 5)), 3),
                'p95': round(float(np.percentile(v, 95)), 3),
                'min': round(float(v.min()), 3),
                'max': round(float(v.max()), 3)}

    def flat(key, subset=objects):
        return [e for o in subset for e in o[key]]

    lead_by_time = {}
    for lead in (1.5, 1.0, 0.75, 0.5, 0.25):
        have = [o for o in pos if lead in o['state_at_lead']]
        lead_by_time[str(lead)] = {
            'objects_tracked_at_that_time': len(have),
            'of_positives': len(pos),
            'alerting': sum(o['state_at_lead'][lead] in ALERT for o in have)}
    dt = np.diff(stamps) if len(stamps) > 1 else np.array([0.0])
    ms = np.array(proc) if proc else np.array([0.0])
    return {
        'definition': {
            'collision': f'true surface distance <= {P.uav_radius_m} m',
            'near_miss': 'true surface distance <= '
                         f'{P.uav_radius_m + P.margin_m:.2f} m',
            'predicted_positive': 'highest state before the true closest '
                                  'approach is WARNING or CRITICAL'},
        'objects': len(objects),
        'confusion_matrix_truth_by_highest_state': {
            c: {s: matrix[c].get(s, 0) for s in ORDER} for c in matrix},
        'intrusion_detection': {
            'true_positive': tp, 'false_negative': fn, 'false_positive': fp,
            'true_negative': tn,
            'precision': round(tp / (tp + fp), 3) if tp + fp else None,
            'recall': round(tp / (tp + fn), 3) if tp + fn else None,
            'false_positive_rate': round(fp / (fp + tn), 3)
            if fp + tn else None,
            'false_negative_rate': round(fn / (fn + tp), 3)
            if fn + tp else None},
        'collisions': {
            'count': len(col),
            'reached_critical': sum(o['highest_state'] == 'CRITICAL'
                                    for o in col),
            'predicted_collision_flag': sum(o['predicted_collision_flag']
                                            for o in col),
            'lead_time_s': stats([o['lead_time_s'] for o in col
                                  if o['lead_time_s'] is not None])},
        'warning_lead_time_s_all_true_intrusions': stats(
            [o['lead_time_s'] for o in pos if o['lead_time_s'] is not None]),
        'state_before_closest_approach_true_intrusions': lead_by_time,
        'd_min_error_vs_true_centre_m': stats(
            flat('d_min_error_vs_centre_m')),
        'd_min_error_vs_true_surface_m': stats(
            flat('d_min_error_vs_surface_m')),
        'linear_closed_form_d_min_error_vs_centre_m': stats(
            flat('linear_d_min_error_vs_centre_m')),
        'time_to_closest_approach_error_s': stats(flat('tca_error_s')),
        'downgrades_before_closest_approach': sum(
            o['downgrades_before_closest_approach'] for o in objects),
        'transition_patterns': Counter(
            o['transitions'] for o in objects).most_common(12),
        'alerts_after_true_closest_approach': late_alerts,
        'alerts_not_on_any_falling_object': orphan_alerts,
        'orphan_alert_examples': orphan_examples,
        'frames': frames,
        'uav_state_valid_fraction': round(valid / max(1, frames), 4),
        'message_rate_hz_sim_time': round(
            (len(stamps) - 1) / max(1e-9, stamps[-1] - stamps[0]), 2)
        if len(stamps) > 1 else None,
        'frame_gap_ms_p95': round(float(np.percentile(dt, 95)) * 1e3, 1),
        'processing_ms': {'median': round(float(np.median(ms)), 3),
                          'p95': round(float(np.percentile(ms, 95)), 3),
                          'max': round(float(ms.max()), 3)},
        'false_negatives': [o for o in pos
                            if o['highest_state'] not in ALERT],
        'false_positives': [o for o in neg if o['highest_state'] in ALERT],
        'per_object': [{k: v for k, v in o.items()
                        if not isinstance(v, list)} for o in objects],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('run_dirs', nargs='+')
    parser.add_argument('--logged', action='store_true')
    parser.add_argument('--min-altitude', type=float, default=2.8)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    source = logged if args.logged else replay
    report = {scope: score(args.run_dirs, source, args.min_altitude, scope)
              for scope in ('free_fall', 'whole_life')}
    with open(args.output, 'w') as handle:
        json.dump(report, handle, indent=1)
    for scope, r in report.items():
        print('==', scope, 'objects', r['objects'])
        print(json.dumps(r['confusion_matrix_truth_by_highest_state']))
        print(r['intrusion_detection'])
        print('collisions', r['collisions'])
        print('lead', r['warning_lead_time_s_all_true_intrusions'])
        for o in r['false_negatives']:
            print('  FN', o['run'], o['object'], o['truth_class'],
                  o['true_min_surface_m'], o['highest_state'],
                  'after impact' if o['closest_approach_after_first_impact']
                  else 'free fall', 'frames', o['risk_frames'])


if __name__ == '__main__':
    main()
