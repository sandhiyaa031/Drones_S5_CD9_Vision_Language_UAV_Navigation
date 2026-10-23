#!/usr/bin/env python3
"""OFFLINE scoring of a mission (Phase 10).

Reads <run>/mission_report.json (written by the mission manager in flight)
and checks it against simulator ground truth and the PX4 flight log. Ground
truth is used here only.

Reported per run
    outcome, why the search ended, phase timeline, commands sent
    whether the vehicle flew at all and whether it landed (flight log)
    every report entry: what it really was and how far off its position is
    survivors in the area that are missing from the report

Usage: evaluate_mission.py [--out FILE] RUN...
"""

import argparse
import json
import math
import os
import sys

WS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(WS, 'src', 'uav_autonomy'))
sys.path.insert(0, os.path.join(WS, 'scripts', 'survivor'))

import evaluate_localization as loc_eval  # noqa: E402

MATCH_RADIUS_M = 1.5


def load_jsonl(path):
    rows = []
    if os.path.exists(path):
        for line in open(path):
            try:
                rows.append(json.loads(line))
            except ValueError:
                continue
    return rows


def evaluate(run_dir):
    run_dir = run_dir.rstrip('/')
    info = open(os.path.join(run_dir, 'trial_info.txt')).read()
    outcome = next((tok.split('=', 1)[1] for tok in info.split()
                    if tok.startswith('controller_outcome=')), 'UNKNOWN')
    result = {'controller_outcome': outcome}
    report_path = os.path.join(run_dir, 'mission_report.json')
    if not os.path.exists(report_path):
        result['error'] = 'no mission report was written'
        return result
    report = json.load(open(report_path))
    commands = load_jsonl(os.path.join(run_dir, 'mission_commands.jsonl'))
    result.update({
        'mission': report['mission']['name'],
        'mission_outcome': report['outcome'],
        'objective_met': report.get('objective_met'),
        'search_end_reason': report['search_end_reason'],
        'refused_because': report['refused_because'],
        'phases': [t['phase'] for t in report['timeline']],
        'commands': [[c['command'], c['reason']] for c in commands],
        'duration_s': report['duration_s'],
        'search_time_s': report['search_time_s'],
        'coverage_fraction': report['coverage_fraction'],
        'inspections': report['inspections'],
        'avoidance_manoeuvres': report['avoidance_manoeuvres'],
        'vlm_available': report['vlm_available'],
        'battery_min': report['battery_min'],
        'summary': report['summary'], 'notes': report['notes']})

    # "flew" = PX4's own log shows the vehicle armed
    flew = False
    verification = os.path.join(run_dir, 'verification.json')
    if os.path.exists(verification):
        v = json.load(open(verification))
        flew = bool(v['checks'].get('armed'))
        result['flight_log_checks_passed'] = all(v['checks'].values())
        result['failed_flight_log_checks'] = [
            k for k, ok in v['checks'].items() if not ok]
        result['max_altitude_m'] = v['metrics'].get('max_altitude_m')
        result['landing_offset_m'] = v['metrics'].get(
            'landing_horizontal_offset_m')
    result['flew'] = flew
    if not flew:
        return result

    world, spec = loc_eval.load_world(run_dir)
    t, p, _, _ = loc_eval.load_ulog_pose(run_dir)
    offset = loc_eval.world_to_local(run_dir, t, p)
    home = report['mission']
    truths = [{'name': s['name'], 'kind': 'survivor',
               'ned': (s['position'][1] - offset[0],
                       s['position'][0] - offset[1])}
              for s in spec['survivors']]
    truths += [{'name': o['name'], 'kind': o.get('kind', 'object'),
                'ned': (o['position'][1] - offset[0],
                        o['position'][0] - offset[1])}
               for o in spec['objects']]
    entries = []
    for e in report['entries']:
        best, best_d = None, None
        for truth in truths:
            d = math.hypot(e['local_ned_m'][0] - truth['ned'][0],
                           e['local_ned_m'][1] - truth['ned'][1])
            rank = d if truth['kind'] == 'survivor' else d + 0.3
            if best_d is None or rank < best_rank:
                best, best_d, best_rank = truth, d, rank
        matched = best is not None and best_d <= MATCH_RADIUS_M
        entries.append({
            'id': e['id'], 'classification': e['classification'],
            'sigma_m': e['horizontal_sigma_m'],
            'observations': e['observations'],
            'truth': best['name'] if matched else None,
            'truth_kind': best['kind'] if matched else None,
            'error_m': round(best_d, 2) if matched else None})
    result['world'] = world
    result['entries'] = entries
    listed = {e['truth'] for e in entries
              if e['truth_kind'] == 'survivor' and e['classification'] in (
                  'SURVIVOR_CONFIRMED', 'UNVERIFIED_CANDIDATE')}
    # the local frame starts at the takeoff point (within centimetres)
    in_area = [tr['name'] for tr in truths if tr['kind'] == 'survivor'
               and home['north_min'] <= tr['ned'][0] <= home['north_max']
               and home['east_min'] <= tr['ned'][1] <= home['east_max']]
    result.update({
        'survivors_in_area': in_area,
        'survivors_reported': sorted(listed & set(in_area)),
        'survivors_missing': sorted(set(in_area) - listed),
        'confirmed_non_survivors': [
            e['truth'] for e in entries
            if e['classification'] == 'SURVIVOR_CONFIRMED'
            and e['truth_kind'] != 'survivor'],
        'candidate_non_survivors': [
            e['truth'] for e in entries
            if e['classification'] == 'UNVERIFIED_CANDIDATE'
            and e['truth_kind'] != 'survivor']})
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('runs', nargs='+')
    parser.add_argument('--out')
    args = parser.parse_args()
    results = {}
    for run_dir in args.runs:
        r = evaluate(run_dir)
        name = os.path.basename(run_dir.rstrip('/'))
        results[name] = r
        print(f'== {name}: controller {r["controller_outcome"]}')
        if 'error' in r:
            print('   ' + r['error'])
            continue
        print(f'   mission {r["mission_outcome"]} (objective met: '
              f'{r["objective_met"]}); phases '
              f'{" > ".join(r["phases"])}')
        print(f'   search ended: {r["search_end_reason"] or "-"}; refused: '
              f'{r["refused_because"] or "-"}')
        print(f'   commands: {r["commands"]}')
        print(f'   flew {r["flew"]}; log checks '
              f'{r.get("flight_log_checks_passed")} '
              f'{r.get("failed_flight_log_checks") or ""}; search '
              f'{r["search_time_s"]} s; coverage {r["coverage_fraction"]}; '
              f'inspections {r["inspections"]}; avoidance '
              f'{r["avoidance_manoeuvres"]}; battery min {r["battery_min"]}')
        print(f'   report summary {r["summary"]}; VLM available '
              f'{r["vlm_available"]}')
        for e in r.get('entries', []):
            print(f'     #{e["id"]} {e["classification"]:21s} -> '
                  f'{e["truth"]} ({e["truth_kind"]}) error {e["error_m"]} m '
                  f'sigma {e["sigma_m"]} obs {e["observations"]}')
        if r['flew']:
            print(f'   survivors reported {r["survivors_reported"]} missing '
                  f'{r["survivors_missing"]}; confirmed non-survivors '
                  f'{r["confirmed_non_survivors"]}')
    if args.out:
        with open(args.out, 'w') as handle:
            json.dump(results, handle, indent=1)


if __name__ == '__main__':
    main()
