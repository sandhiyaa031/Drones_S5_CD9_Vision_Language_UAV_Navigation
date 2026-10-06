#!/usr/bin/env python3
"""Evaluate the VLM alone on the controlled crop dataset (offline).

Every crop in the dataset is sent once through the same client code the
node uses (uav_autonomy.vlm_client.request_once). Every outcome is kept,
including errors and "uncertain"; nothing is retried or filtered.

    correct confirmation   survivor_clear  -> confirm
    false rejection        survivor_clear  -> reject
    correct rejection      non-survivor    -> reject
    false confirmation     non-survivor    -> confirm
    uncertain              any             -> uncertain
    survivor_partial crops are ambiguous by construction and are reported
    as a distribution, not as right or wrong.
"""

import argparse
import json
import os

import numpy as np

from uav_autonomy.vlm_client import VlmConfig, probe, request_once

CONFIRM_CONFIDENCE = 0.6        # ConfirmationParams.vlm_confirm_confidence


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--dataset', default='results/phase7/vlm_dataset')
    parser.add_argument('--endpoint',
                        default=os.environ.get('VLM_ENDPOINT', ''))
    parser.add_argument('--model', default=os.environ.get('VLM_MODEL', ''))
    parser.add_argument('--timeout', type=float, default=120.0)
    parser.add_argument('--output')
    args = parser.parse_args()
    config = VlmConfig(endpoint=args.endpoint, model=args.model,
                       api_key=os.environ.get('VLM_API_KEY', ''),
                       timeout_s=args.timeout)
    if not probe(config):
        raise SystemExit(f'VLM endpoint not available: {args.endpoint!r} '
                         f'model {args.model!r}. Nothing was evaluated.')
    items = json.load(open(os.path.join(args.dataset, 'dataset.json')))
    rows = []
    for item in items:
        jpeg = open(os.path.join(args.dataset, 'crops',
                                 item['id'] + '.jpg'), 'rb').read()
        out = request_once(config, jpeg)
        rows.append({**item, 'kind': out.kind, 'decision': out.decision,
                     'confidence': out.confidence,
                     'visibility': out.visibility, 'target': out.target,
                     'description': out.description,
                     'latency_s': round(out.latency_s, 3),
                     'error': out.error, 'raw': out.raw[:300]})
        print(f"{item['id']:32s} expected {item['expected']:9s} -> "
              f"{out.kind}:{out.decision or out.error} "
              f"{out.confidence:.2f} {out.visibility} \"{out.target}\" "
              f"{out.latency_s:.2f} s", flush=True)

    def effective(row):
        """What the node would do with this answer."""
        if row['kind'] != 'answer':
            return 'error'
        if row['decision'] == 'confirm' and \
                row['confidence'] < CONFIRM_CONFIDENCE:
            return 'uncertain'
        return row['decision']

    table = {}
    for row in rows:
        cell = table.setdefault(row['category'], {
            'n': 0, 'confirm': 0, 'reject': 0, 'uncertain': 0, 'error': 0})
        cell['n'] += 1
        cell[effective(row)] += 1
    positives = [r for r in rows if r['expected'] == 'confirm']
    negatives = [r for r in rows if r['expected'] == 'reject']
    latency = [r['latency_s'] for r in rows if r['kind'] == 'answer']

    def count(rows_, what):
        return sum(effective(r) == what for r in rows_)

    summary = {
        'model': args.model, 'endpoint': args.endpoint, 'crops': len(rows),
        'by_category': table,
        'survivor_clear': {
            'n': len(positives),
            'correct_confirmations': count(positives, 'confirm'),
            'false_rejections': count(positives, 'reject'),
            'uncertain': count(positives, 'uncertain'),
            'errors': count(positives, 'error')},
        'non_survivor': {
            'n': len(negatives),
            'correct_rejections': count(negatives, 'reject'),
            'false_confirmations': count(negatives, 'confirm'),
            'uncertain': count(negatives, 'uncertain'),
            'errors': count(negatives, 'error')},
        'false_confirmations': [
            {'id': r['id'], 'object': r['object'], 'target': r['target'],
             'confidence': r['confidence']}
            for r in negatives if effective(r) == 'confirm'],
        'false_rejections': [
            {'id': r['id'], 'target': r['target'],
             'range_m': r.get('range_m')}
            for r in positives if effective(r) == 'reject'],
        'latency_s': {
            'n': len(latency),
            'p50': round(float(np.percentile(latency, 50)), 3),
            'p95': round(float(np.percentile(latency, 95)), 3),
            'max': round(float(np.max(latency)), 3)} if latency else None}
    print(json.dumps(summary, indent=1))
    if args.output:
        json.dump({'summary': summary, 'rows': rows},
                  open(args.output, 'w'), indent=1)


if __name__ == '__main__':
    main()
