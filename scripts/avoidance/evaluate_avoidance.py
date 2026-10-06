#!/usr/bin/env python3
"""Score avoidance flights against Gazebo ground truth (offline).

For every run directory given: true closest approach of every released box
to the UAV, collisions, mission outcome, path length, deviation, and every
manoeuvre the planner made (timing, latency, kinematics, recovery).
Ground truth (poses.csv) is used here only; nothing in flight reads it.

A paired baseline run (same scenario, planner disabled) tells which
releases were real threats, hence which manoeuvres were unnecessary.

Definitions
    collision      true distance from the UAV centre to the box surface
                   <= UAV radius (0.40 m), as in Phase 5
    near miss      <= UAV radius + margin (0.70 m)
    nominal        for a release made while the UAV held position (HOLD):
                   the closest approach the box would have had to a UAV
                   that stayed on the mission setpoint. This says whether
                   the release was a real threat, in the same run, without
                   relying on the baseline flight (which ends at its first
                   collision). A manoeuvre answering a release whose
                   nominal class is SAFE is counted as unnecessary.
    lead time      manoeuvre start -> the box reaching the UAV's altitude
    start latency  first WARNING/CRITICAL risk frame (sensor time) -> the
                   flight controller reporting the command active
"""

import argparse
import json
import os
import sys

import numpy as np
from pyulog import ULog

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'perception'))
import replay_tracker as rt  # noqa: E402

UAV_RADIUS = 0.40
MARGIN = 0.30
BASE_Z = 0.24
ACTIVE_MODES = ('AVOIDING', 'AVOIDANCE_FAILSAFE')


def jsonl(path):
    rows = []
    if os.path.exists(path):
        for line in open(path):
            try:
                rows.append(json.loads(line))
            except ValueError:
                continue
    return rows


def info(run_dir):
    out = {}
    path = os.path.join(run_dir, 'trial_info.txt')
    if os.path.exists(path):
        for token in open(path).read().split():
            if '=' in token:
                key, value = token.split('=', 1)
                out[key] = value
    log = os.path.join(run_dir, 'flight_controller.log')
    out['failsafe_reason'] = None
    if os.path.exists(log):
        for line in open(log):
            if 'FAILSAFE' in line and out['failsafe_reason'] is None:
                out['failsafe_reason'] = line.split(']: ', 1)[-1].strip()
    return out


def klass(d):
    return ('COLLISION' if d <= UAV_RADIUS else
            'NEAR_MISS' if d <= UAV_RADIUS + MARGIN else 'SAFE')


def truth(run_dir, records=()):
    """Per-box closest approach, and the UAV truth path in local NED."""
    rec_t = np.array([r['t_uav'] for r in records])
    if not os.path.exists(os.path.join(run_dir, 'releases.json')):
        json.dump([], open(os.path.join(run_dir, 'releases.json'), 'w'))
    debris, _, uav, origin = rt.load_truth(run_dir, 'x500_rescue_0', 1.0)
    t_u = uav[:, 0]
    uav_ned = np.column_stack([uav[:, 2] - origin[1], uav[:, 1] - origin[0],
                               -(uav[:, 3] + BASE_Z - origin[2])])
    releases = json.load(open(os.path.join(run_dir, 'releases.json')))
    boxes = []
    for index, rel in enumerate(releases):
        obj = debris.get(rel['name'])
        if obj is None or len(obj['t']) < 5:
            continue
        t = obj['t']
        u = np.column_stack([np.interp(t, t_u, uav_ned[:, i])
                             for i in range(3)])
        surface = np.array([rt.surface_distance(u[i], obj['p'][i],
                                                obj['size'])
                            for i in range(len(t))])
        centre = np.linalg.norm(obj['p'] - u, axis=1)
        fall = t <= obj['fall_end'] + 0.10
        i_all = int(np.argmin(surface))
        i_fall = int(np.argmin(np.where(fall, surface, np.inf)))
        passed = np.nonzero(obj['p'][:, 2] >= u[:, 2] - 0.3)[0]

        nominal = None
        if len(rec_t):
            rec = records[min(int(np.searchsorted(rec_t, t[0])),
                              len(records) - 1)]
            # Not meaningful if the box touched the UAV: its path after
            # the contact is no longer the one it would have had.
            if (rec['flight_state'] == 'HOLD'
                    and klass(surface[i_all]) != 'COLLISION'):
                point = np.array(rec['mission'])
                d_nom = min(rt.surface_distance(point, obj['p'][i],
                                                obj['size'])
                            for i in range(len(t)))
                nominal = {'min_surface_m': round(float(d_nom), 3),
                           'class': klass(d_nom)}
        boxes.append({
            'nominal': nominal,
            'index': index, 'name': rel['name'], 'label': rel.get('label'),
            't_release': float(t[0]),
            't_pass_uav_altitude': (float(t[passed[0]]) if len(passed)
                                    else None),
            'free_fall': {'min_surface_m': round(float(surface[i_fall]), 3),
                          'min_centre_m': round(float(
                              centre[fall].min()), 3),
                          't': float(t[i_fall]),
                          'class': klass(surface[i_fall])},
            'whole_life': {'min_surface_m': round(float(surface[i_all]), 3),
                           'min_centre_m': round(float(centre.min()), 3),
                           't': float(t[i_all]),
                           'after_first_impact': bool(
                               t[i_all] > obj['fall_end'] + 0.05),
                           'class': klass(surface[i_all])}})
    airborne = uav[:, 3] > 1.0
    path = float(np.linalg.norm(np.diff(uav_ned[airborne], axis=0),
                                axis=1).sum())
    return boxes, path, (t_u, uav_ned)


def kinematics(run_dir):
    path = os.path.join(run_dir, 'flight.ulg')
    if not os.path.exists(path):
        return None
    lp = ULog(path, ['vehicle_local_position']).get_dataset(
        'vehicle_local_position').data
    return {'t': lp['timestamp_sample'] * 1e-6,
            'v': np.c_[lp['vx'], lp['vy'], lp['vz']],
            'a': np.c_[lp['ax'], lp['ay'], lp['az']]}


def manoeuvres(records, risk_rows, boxes, kin):
    """Each manoeuvre: NORMAL/PREPARE/RESUME -> avoiding ... -> NORMAL."""
    alerts = []     # (stamp, ids) of risk frames with WARNING/CRITICAL
    for row in risk_rows:
        ids = [r['id'] for r in row.get('risks', [])
               if r['state'] in ('WARNING', 'CRITICAL')]
        if ids:
            alerts.append((row['stamp'], ids))
    out, current = [], None
    for rec in records:
        tr = rec.get('transition')
        t = rec['t_uav']
        if tr and tr[1] in ACTIVE_MODES and tr[0] not in ACTIVE_MODES \
                and tr[0] != 'CLEARING':
            if current is not None and current.get('t_resume') is None:
                current['t_resume'] = t      # interrupted before resuming
            if current is not None and current.get('t_normal') is None:
                # A new threat arrived during the return: same excursion,
                # counted as a separate manoeuvre.
                current['t_normal'] = t
            current = {'t_start': t, 'first_mode': tr[1],
                       'trigger': tr[2], 'candidates': [],
                       'failsafe': False, 'threat_ids': set(),
                       'commanded': False,
                       't_active': None, 't_resume': None, 't_normal': None,
                       'max_deviation_m': 0.0, 'max_speed_m_s': 0.0,
                       'start_p': rec['uav_p'], 'mission': rec['mission'],
                       'feasible_at_start': sum(
                           c['ok'] for c in rec['candidates']),
                       'candidates_at_start': len(rec['candidates']),
                       'predicted_min_separation_m':
                           rec['predicted_min_separation']}
            out.append(current)
        if current is None or current['t_normal'] is not None:
            continue
        if rec['mode'] in ACTIVE_MODES or rec['mode'] == 'CLEARING':
            if rec['candidate'] and (not current['candidates'] or
                                     current['candidates'][-1]
                                     != rec['candidate']):
                current['candidates'].append(rec['candidate'])
            current['failsafe'] |= rec['mode'] == 'AVOIDANCE_FAILSAFE'
            current['threat_ids'].update(
                th['id'] for th in rec['threats']
                if th['state'] in ('WARNING', 'CRITICAL'))
        current['commanded'] |= bool(
            rec['active'] and rec.get('enabled', True)
            and rec['mode'] in ACTIVE_MODES)
        if rec['controller_active'] and current['t_active'] is None:
            current['t_active'] = t
        dev = float(np.linalg.norm(np.subtract(rec['uav_p'],
                                               current['mission'])))
        if rec['flight_state'] == 'HOLD':
            current['max_deviation_m'] = max(current['max_deviation_m'], dev)
        current['max_speed_m_s'] = max(current['max_speed_m_s'], float(
            np.linalg.norm(rec['uav_v'])))
        if tr and tr[1] == 'RESUME_MISSION':
            current['t_resume'] = t
        if tr and tr[1] == 'NORMAL_FLIGHT':
            current['t_normal'] = t
    out = [m for m in out if m['commanded']]
    for m in out:
        end = m['t_normal'] or m['t_resume'] or m['t_start'] + 4.0
        first_alert = next((s for s, ids in alerts
                            if m['t_start'] - 1.0 <= s <= m['t_start']
                            and (not m['threat_ids']
                                 or m['threat_ids'] & set(ids))), None)
        m['t_first_alert'] = first_alert
        m['start_latency_s'] = (
            round(m['t_active'] - first_alert, 3)
            if first_alert is not None and m['t_active'] is not None
            else None)
        m['duration_s'] = (round(m['t_resume'] - m['t_start'], 3)
                           if m['t_resume'] else None)
        m['recovery_s'] = (round(m['t_normal'] - m['t_resume'], 3)
                           if m['t_normal'] and m['t_resume'] else None)
        if kin is not None:
            sel = (kin['t'] >= m['t_start']) & (kin['t'] <= end)
            if sel.any():
                m['max_accel_m_s2'] = round(float(np.linalg.norm(
                    kin['a'][sel], axis=1).max()), 2)
                m['max_speed_m_s'] = round(float(np.linalg.norm(
                    kin['v'][sel], axis=1).max()), 2)
        # The release this manoeuvre answers: the latest one before it.
        prior = [b for b in boxes
                 if b['t_release'] <= m['t_start'] <= b['t_release'] + 3.0]
        # Several boxes in the air: the one that was the real threat
        # (smallest nominal separation), else the latest.
        box = None
        if prior:
            box = min(prior, key=lambda b: (
                b['nominal']['min_surface_m'] if b['nominal'] else 99.0,
                -b['t_release']))
        m['release_index'] = box['index'] if box else None
        m['lead_time_s'] = (
            round(box['t_pass_uav_altitude'] - m['t_start'], 3)
            if box and box['t_pass_uav_altitude'] else None)
        m['release_to_start_s'] = (round(m['t_start'] - box['t_release'], 3)
                                   if box else None)
        m['threat_ids'] = sorted(m['threat_ids'])
        m['max_deviation_m'] = round(m['max_deviation_m'], 3)
    return out


def evaluate(run_dir, baseline=None):
    meta = info(run_dir)
    records = [r for r in jsonl(os.path.join(run_dir, 'avoidance.jsonl'))
               if 't_uav' in r]
    boxes, path, _ = truth(run_dir, records)
    kin = kinematics(run_dir)
    mans = manoeuvres(records, jsonl(os.path.join(run_dir, 'risk.jsonl')),
                      boxes, kin)
    base_class = {}
    if baseline is not None:
        # The baseline flight ends at its first collision (failsafe
        # landing): later releases there are not comparable.
        for b in baseline['boxes']:
            base_class[b['index']] = b['whole_life']['class']
            if b['whole_life']['class'] == 'COLLISION':
                break
    by_index = {b['index']: b for b in boxes}
    for m in mans:
        box = by_index.get(m['release_index'])
        if box is None:
            m['unnecessary'] = True          # no release explains it
        elif box['nominal'] is not None:
            m['unnecessary'] = box['nominal']['class'] == 'SAFE'
        elif m['release_index'] in base_class:
            m['unnecessary'] = base_class[m['release_index']] == 'SAFE'
        else:
            m['unnecessary'] = None
    airborne = [r for r in records if r['flight_state'] in (
        'TAKEOFF', 'HOLD', 'WAYPOINT_1', 'WAYPOINT_2', 'RETURN_HOME')]
    planning = [r['compute_ms'] for r in airborne if r['candidates']]
    span = (airborne[-1]['t_uav'] - airborne[0]['t_uav']) if airborne else 0
    hold_dev = [float(np.linalg.norm(np.subtract(r['uav_p'], r['mission'])))
                for r in airborne if r['flight_state'] == 'HOLD']

    def pct(values, q):
        return round(float(np.percentile(values, q)), 3) if values else None

    def count(scope, klass):
        return sum(b[scope]['class'] == klass for b in boxes)

    return {
        'run': os.path.relpath(run_dir),
        'scenario': meta.get('scenario'), 'variant': meta.get('variant'),
        'controller_outcome': meta.get('controller_outcome'),
        'mission_completed': meta.get('controller_outcome') == 'COMPLETE',
        'failsafe_reason': meta.get('failsafe_reason'),
        'releases': len(boxes),
        'collisions_whole_life': count('whole_life', 'COLLISION'),
        'collisions_free_fall': count('free_fall', 'COLLISION'),
        'near_misses_whole_life': count('whole_life', 'NEAR_MISS'),
        'min_surface_separation_m': (min(
            b['whole_life']['min_surface_m'] for b in boxes)
            if boxes else None),
        'path_length_m': round(path, 2),
        'max_hold_deviation_m': (round(max(hold_dev), 3) if hold_dev
                                 else None),
        'manoeuvres': len(mans),
        'failsafe_manoeuvres': sum(m['failsafe'] for m in mans),
        'unnecessary_manoeuvres': sum(bool(m['unnecessary']) for m in mans),
        'nominal_collisions': sum(
            b['nominal'] is not None and b['nominal']['class'] == 'COLLISION'
            for b in boxes),
        'nominal_threats': sum(
            b['nominal'] is not None and b['nominal']['class'] != 'SAFE'
            for b in boxes),
        'planner': {
            'cycles_airborne': len(airborne),
            'command_rate_hz': (round(len(airborne) / span, 1) if span
                                else None),
            'compute_ms_with_threats': {
                'n': len(planning), 'p50': pct(planning, 50),
                'p95': pct(planning, 95), 'max': pct(planning, 100)},
            'compute_ms_all': {
                'p50': pct([r['compute_ms'] for r in airborne], 50),
                'max': pct([r['compute_ms'] for r in airborne], 100)}},
        'boxes': boxes, 'manoeuvre_list': mans}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('runs', nargs='+')
    parser.add_argument('--baseline-root',
                        help='directory holding the paired baseline runs '
                             '(same sub-directory names)')
    parser.add_argument('--output')
    args = parser.parse_args()
    results = []
    for run in args.runs:
        run = run.rstrip('/')
        baseline = None
        if args.baseline_root:
            pair = os.path.join(args.baseline_root, os.path.basename(run))
            if os.path.exists(os.path.join(pair, 'poses.csv')):
                baseline = evaluate(pair)
        res = evaluate(run, baseline)
        results.append(res)
        print(f"\n== {res['run']}  outcome={res['controller_outcome']} "
              f"releases={res['releases']} "
              f"collisions={res['collisions_whole_life']} "
              f"min_surface={res['min_surface_separation_m']} "
              f"path={res['path_length_m']} m "
              f"manoeuvres={res['manoeuvres']} "
              f"(failsafe {res['failsafe_manoeuvres']}, unnecessary "
              f"{res['unnecessary_manoeuvres']})")
        if res['failsafe_reason']:
            print('   ', res['failsafe_reason'][:150])
        for b in res['boxes']:
            nom = (f"nominal {b['nominal']['min_surface_m']:.2f} m "
                   f"{b['nominal']['class']}; " if b['nominal'] else '')
            print(f"   box {b['index']} [{b['label']}] {nom}fall "
                  f"{b['free_fall']['min_surface_m']:.2f} m "
                  f"{b['free_fall']['class']}; whole "
                  f"{b['whole_life']['min_surface_m']:.2f} m "
                  f"{b['whole_life']['class']}")
        for m in res['manoeuvre_list']:
            print(f"   man t={m['t_start']:.2f} {m['first_mode']} "
                  f"{'/'.join(m['candidates'])} box={m['release_index']} "
                  f"feasible={m['feasible_at_start']}/"
                  f"{m['candidates_at_start']} lead={m['lead_time_s']} "
                  f"latency={m['start_latency_s']} dur={m['duration_s']} "
                  f"rec={m['recovery_s']} dev={m['max_deviation_m']} "
                  f"v={m['max_speed_m_s']} a={m.get('max_accel_m_s2')} "
                  f"unnecessary={m['unnecessary']}")
        print('   planner', json.dumps(res['planner']))
    if args.output:
        json.dump(results, open(args.output, 'w'), indent=1)


if __name__ == '__main__':
    main()
