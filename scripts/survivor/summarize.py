#!/usr/bin/env python3
"""Phase 7 summary: every live run scored, plus pooled totals (offline)."""

import argparse
import glob
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import evaluate_survivor as ev  # noqa: E402


def pooled(results, key):
    tp = sum(r[key]['tp'] for r in results)
    fn = sum(r[key]['fn'] for r in results)
    fp = sum(r[key]['fp'] for r in results)
    frames = sum(r[key]['frames'] for r in results)
    none = sum(r[key]['frames_without_survivor'] for r in results)
    fp_none = sum(round((r[key]['false_positive_rate_without_survivor'] or 0)
                        * r[key]['frames_without_survivor'])
                  for r in results)
    fp_frames = sum(round((r[key]['false_positive_rate_per_frame'] or 0)
                          * r[key]['frames']) for r in results)
    p = tp / (tp + fp) if tp + fp else None
    r = tp / (tp + fn) if tp + fn else None
    partial = {k: sum(x[key]['partial_frames'][k] for x in results)
               for k in ('frames', 'strong', 'weak', 'none')}

    def rnd(v):
        return None if v is None else round(v, 4)
    return {'frames': frames, 'tp': tp, 'fn': fn, 'fp': fp,
            'precision': rnd(p), 'recall': rnd(r),
            'f1': rnd(2 * p * r / (p + r) if p and r else None),
            'false_positive_rate_per_frame': rnd(fp_frames / frames
                                                 if frames else None),
            'false_positive_rate_without_survivor': rnd(
                fp_none / none if none else None),
            'frames_without_survivor': none,
            'false_negative_rate': rnd(1 - r if r is not None else None),
            'partial_frames': partial,
            'weak_candidates_on_nothing': sum(
                x[key]['weak_candidates_on_nothing'] for x in results)}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--root', default='results/phase7/live')
    parser.add_argument('--suffix', default='',
                        help='only runs whose name ends with this')
    parser.add_argument('--exclude-suffix', default='')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    runs = sorted(d for d in glob.glob(os.path.join(args.root, '*'))
                  if os.path.isdir(d) and d.endswith(args.suffix)
                  and not (args.exclude_suffix
                           and d.endswith(args.exclude_suffix)))
    results = [ev.evaluate(run) for run in runs]
    head = ('| run | frames | state counts | detector TP/FN/FP | '
            'published TP/FN/FP | partial (strong/weak/none) | '
            'confirmed | false confirm. | VLM req/ans/err | '
            'VLM latency p50 [s] | confirm delay [s] |')
    lines = [head, '|' + '---|' * 11]
    for r in results:
        d, p, v = (r['candidate_detector_live'],
                   r['published_after_temporal_confirmation'], r['vlm'])
        part = d['partial_frames']
        delays = [x['confirmation_delay_s']
                  for x in r['survivor_delays'].values()
                  if x['confirmation_delay_s'] is not None]
        states = ', '.join(f'{k} {n}' for k, n in sorted(
            r['frame_states'].items()))
        lines.append(
            f"| {os.path.basename(r['run'])} | {r['frames_airborne']} | "
            f"{states} | {d['tp']}/{d['fn']}/{d['fp']} | "
            f"{p['tp']}/{p['fn']}/{p['fp']} | "
            f"{part['strong']}/{part['weak']}/{part['none']} | "
            f"{len(r['survivors_confirmed'])}/"
            f"{len(r['survivors_in_world'])} | "
            f"{len(r['false_confirmations'])} | "
            f"{v['requests']}/{v['answers']}/{v['errors']} | "
            f"{v['latency_s']['p50']} | "
            f"{', '.join(str(x) for x in delays) or '-'} |")
    answers = [a for r in results for a in r['vlm']['log']
               if a['kind'] == 'answer']
    latency = [a['latency_s'] for a in answers]
    callback = [r['timing']['callback_ms'] for r in results]
    e2e = [r['timing']['end_to_end_latency_s'] for r in results]
    cpu = [r['timing']['node_cpu_percent_of_one_core']['median']
           for r in results
           if r['timing']['node_cpu_percent_of_one_core']['median']]
    by_truth = {}
    for a in answers:
        kind = a['truth'].split(':')[0]
        cell = by_truth.setdefault(kind, {'confirm': 0, 'reject': 0,
                                          'uncertain': 0})
        cell[a['decision']] += 1
    summary = {
        'runs': len(results),
        'candidate_detector_live': pooled(results,
                                          'candidate_detector_live'),
        'candidate_detector_replay_recorded_frames': pooled(
            results, 'candidate_detector_replay_recorded_frames'),
        'published_after_temporal_confirmation': pooled(
            results, 'published_after_temporal_confirmation'),
        'survivors': {
            'present': sum(len(r['survivors_in_world']) for r in results),
            'confirmed': sum(len(r['survivors_confirmed'])
                             for r in results),
            'missed_by_detector': sum(len(r['survivors_missed'])
                                      for r in results),
            'false_confirmations': sum(len(r['false_confirmations'])
                                       for r in results),
            'confirmed_frames_on_survivor': sum(
                r['confirmed_frames']['on_survivor'] for r in results),
            'confirmed_frames_on_non_survivor': sum(
                r['confirmed_frames']['on_non_survivor'] for r in results)},
        'vlm_live': {
            'requests': sum(r['vlm']['requests'] for r in results),
            'answers': len(answers),
            'errors': sum(r['vlm']['errors'] for r in results),
            'unavailable': sum(r['vlm']['unavailable'] for r in results),
            'decisions_by_truth': by_truth,
            'latency_s': {
                'p50': ev.pct(latency, 50), 'p95': ev.pct(latency, 95),
                'max': ev.pct(latency, 100)}},
        'timing': {
            'callback_ms_p50_range': [min(c['p50'] for c in callback),
                                      max(c['p50'] for c in callback)],
            'callback_ms_max': max(c['max'] for c in callback),
            'end_to_end_latency_s_p50_range': [
                min(e['p50'] for e in e2e), max(e['p50'] for e in e2e)],
            'end_to_end_latency_s_p95_max': max(e['p95'] for e in e2e),
            'node_cpu_percent_median': (round(float(np.median(cpu)), 1)
                                        if cpu else None),
            'camera_rate_hz_range': [
                min(r['camera_rate_hz'] for r in results),
                max(r['camera_rate_hz'] for r in results)]}}
    json.dump({'summary': summary, 'runs': results},
              open(args.output, 'w'), indent=1)
    open(args.output.replace('.json', '.md'), 'w').write(
        '\n'.join(lines) + '\n')
    print('\n'.join(lines))
    print(json.dumps(summary, indent=1))


if __name__ == '__main__':
    main()
