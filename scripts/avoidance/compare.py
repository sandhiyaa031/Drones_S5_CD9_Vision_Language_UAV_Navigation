#!/usr/bin/env python3
"""Baseline (no avoidance) against avoidance on the same scenarios.

Reads results/phase6/baseline/<scenario> and results/phase6/avoid/<scenario>,
scores each with evaluate_avoidance, and writes a JSON summary and a
Markdown table.
"""

import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import evaluate_avoidance as ev  # noqa: E402


def stats(values):
    v = [x for x in values if x is not None]
    if not v:
        return None
    return {'n': len(v), 'min': round(float(np.min(v)), 3),
            'p50': round(float(np.median(v)), 3),
            'p95': round(float(np.percentile(v, 95)), 3),
            'max': round(float(np.max(v)), 3)}


def totals(runs):
    releases = sum(r['releases'] for r in runs)
    collisions = sum(r['collisions_whole_life'] for r in runs)
    seps = [b['whole_life']['min_surface_m'] for r in runs
            for b in r['boxes']]
    return {
        'runs': len(runs), 'releases': releases,
        'collisions': collisions,
        'collision_rate_per_release': (round(collisions / releases, 3)
                                       if releases else None),
        'runs_with_collision': sum(r['collisions_whole_life'] > 0
                                   for r in runs),
        'near_misses': sum(r['near_misses_whole_life'] for r in runs),
        'missions_completed': sum(r['mission_completed'] for r in runs),
        'min_surface_separation_m': min(seps) if seps else None,
        'separation_m': stats(seps),
        'path_length_m': round(sum(r['path_length_m'] for r in runs), 1),
        'manoeuvres': sum(r['manoeuvres'] for r in runs),
        'failsafe_manoeuvres': sum(r['failsafe_manoeuvres'] for r in runs),
        'unnecessary_manoeuvres': sum(r['unnecessary_manoeuvres'] or 0
                                      for r in runs)}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--root', default='results/phase6')
    parser.add_argument('--output', default='results/phase6/comparison.json')
    parser.add_argument('--markdown',
                        default='results/phase6/comparison.md')
    args = parser.parse_args()
    base_root = os.path.join(args.root, 'baseline')
    avoid_root = os.path.join(args.root, 'avoid')
    names = sorted(n for n in os.listdir(avoid_root)
                   if os.path.exists(os.path.join(avoid_root, n,
                                                  'poses.csv')))
    rows, base_runs, avoid_runs = [], [], []
    for name in names:
        base = None
        if os.path.exists(os.path.join(base_root, name, 'poses.csv')):
            base = ev.evaluate(os.path.join(base_root, name))
            base_runs.append(base)
        avoid = ev.evaluate(os.path.join(avoid_root, name), base)
        avoid_runs.append(avoid)
        rows.append({'scenario': name, 'baseline': base, 'avoid': avoid})

    mans = [m for r in avoid_runs for m in r['manoeuvre_list']]
    planning = [r['planner']['compute_ms_with_threats'] for r in avoid_runs
                if r['planner']['compute_ms_with_threats']['n']]
    summary = {
        'baseline': totals(base_runs), 'avoidance': totals(avoid_runs),
        'manoeuvres': {
            'count': len(mans),
            'lead_time_s': stats([m['lead_time_s'] for m in mans]),
            'start_latency_s': stats([m['start_latency_s'] for m in mans]),
            'release_to_start_s': stats([m['release_to_start_s']
                                         for m in mans]),
            'duration_s': stats([m['duration_s'] for m in mans]),
            'recovery_s': stats([m['recovery_s'] for m in mans]),
            'max_deviation_m': stats([m['max_deviation_m'] for m in mans
                                      if m['max_deviation_m'] > 0]),
            'max_speed_m_s': stats([m['max_speed_m_s'] for m in mans]),
            'max_accel_m_s2': stats([m.get('max_accel_m_s2')
                                     for m in mans])},
        'planner': {
            'compute_ms_p50': stats([p['p50'] for p in planning]),
            'compute_ms_p95': stats([p['p95'] for p in planning]),
            'compute_ms_max': stats([p['max'] for p in planning]),
            'command_rate_hz': stats([r['planner']['command_rate_hz']
                                      for r in avoid_runs])}}
    json.dump({'summary': summary, 'scenarios': rows},
              open(args.output, 'w'), indent=1)

    def cell(r):
        if r is None:
            return ['-'] * 7
        sep = r['min_surface_separation_m']
        return [f"{r['collisions_whole_life']}/{r['releases']}",
                '-' if sep is None else f'{sep:.2f}',
                'yes' if r['mission_completed'] else 'NO',
                f"{r['path_length_m']:.1f}",
                '-' if r['max_hold_deviation_m'] is None
                else f"{r['max_hold_deviation_m']:.2f}",
                f"{r['manoeuvres']} ({r['failsafe_manoeuvres']})",
                '-' if r['unnecessary_manoeuvres'] is None
                else str(r['unnecessary_manoeuvres'])]

    head = ['collisions', 'min sep [m]', 'mission', 'path [m]',
            'max dev [m]', 'manoeuvres (failsafe)', 'unnecessary']
    lines = ['| scenario | variant | ' + ' | '.join(head) + ' |',
             '|---|---|' + '---|' * len(head)]
    for row in rows:
        lines.append(f"| {row['scenario']} | baseline | "
                     + ' | '.join(cell(row['baseline'])) + ' |')
        lines.append('|  | avoidance | ' + ' | '.join(cell(row['avoid']))
                     + ' |')
    text = '\n'.join(lines)
    open(args.markdown, 'w').write(text + '\n')
    print(text)
    print(json.dumps(summary, indent=1))


if __name__ == '__main__':
    main()
