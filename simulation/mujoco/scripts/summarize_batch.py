#!/usr/bin/env python3
"""Table and JSON summary of the MuJoCo mission batch (results/batch/*)."""
import json, os, sys
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
base = os.path.join(ROOT, 'results', sys.argv[1] if len(sys.argv) > 1 else 'batch')
rows = []
for name in sorted(os.listdir(base)):
    f = os.path.join(base, name, 'mission_report.json')
    if not os.path.exists(f):
        continue
    r = json.load(open(f))
    errs = [e['error_to_nearest_survivor_m'] for e in r['entries']
            if e['scored_as'] == 'survivor']
    truth = json.load(open(os.path.join(ROOT, 'worlds', 'shapes',
                                        f"{r['scene']}.json")))
    a = r['mission']['search_area']
    in_area = [sv['name'] for sv in truth['survivors']
               if a['north'][0] <= sv['position'][1] <= a['north'][1]
               and a['east'][0] <= sv['position'][0] <= a['east'][1]]
    rows.append({
        'run': name, 'scene': r['scene'], 'outcome': r['outcome'],
        'search_end': r['search_end'], 'search_time_s': r['search_time_s'],
        'coverage': r['coverage'], 'survivors_in_area': len(in_area), 'survivors_in_scene': len(r['survivors_in_scene']),
        'survivors_found': len([n for n in r['survivors_found'] if n in in_area]),
        'survivor_entries': len(errs), 'best_error_per_survivor_m': [
            min(e['error_to_nearest_survivor_m'] for e in r['entries']
                if e['scored_as'] == 'survivor' and e['nearest_survivor'] == s)
            for s in r['survivors_found']],
        'lookalike_entries': sum(e['scored_as'] == 'look-alike' for e in r['entries']),
        'collisions': len(r['collisions']), 'landed': r['landed'],
        'landing_offset_m': r['landing_offset_m'],
        'touchdown_speed_m_s': r['touchdown_speed_m_s'],
        'min_lidar_range_m': r['min_lidar_range_m']})
json.dump(rows, open(os.path.join(ROOT, 'results', (sys.argv[1] if len(sys.argv) > 1 else 'batch') + '_summary.json' if len(sys.argv) > 1 else 'summary.json'), 'w'), indent=1)
print(f"{'run':22s} {'outcome':9s} {'t[s]':>6s} {'cov':>5s} {'found':>6s} {'err[m]':>14s} {'look-alike':>10s} {'coll':>4s} landed")
for r in rows:
    print(f"{r['run']:22s} {r['outcome']:9s} {r['search_time_s']:6.1f} {r['coverage']:5.2f} "
          f"{r['survivors_found']}/{r['survivors_in_area']:<4d} {str(r['best_error_per_survivor_m']):>14s} "
          f"{r['lookalike_entries']:10d} {r['collisions']:4d} {r['landed']}")
