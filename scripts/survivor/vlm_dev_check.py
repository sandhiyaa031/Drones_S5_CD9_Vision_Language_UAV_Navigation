#!/usr/bin/env python3
"""Quick look at VLM answers on a fixed slice of a crop dataset (dev tool).

Used while designing the prompt, on the DEVELOPMENT dataset only. The
reported evaluation uses evaluate_vlm.py on a separate held-out dataset.
"""
import argparse
import json
import os

from uav_autonomy.vlm_client import VlmConfig, request_once

parser = argparse.ArgumentParser()
parser.add_argument('--dataset', required=True)
parser.add_argument('--per-category', type=int, default=6)
args = parser.parse_args()
config = VlmConfig(endpoint=os.environ['VLM_ENDPOINT'],
                   model=os.environ['VLM_MODEL'], timeout_s=180)
items = json.load(open(os.path.join(args.dataset, 'dataset.json')))
tally = {}
for category in ('survivor_clear', 'survivor_partial', 'distractor',
                 'rubble', 'structure_shadow', 'debris'):
    group = [i for i in items if i['category'] == category]
    step = max(1, len(group) // args.per_category)
    for item in group[::step][:args.per_category]:
        out = request_once(config, open(os.path.join(
            args.dataset, 'crops', item['id'] + '.jpg'), 'rb').read())
        key = out.decision if out.kind == 'answer' else 'error'
        cell = tally.setdefault(category, {})
        cell[key] = cell.get(key, 0) + 1
        print(f"{item['id']:30s} {item['object']:22s} -> {key:9s} "
              f"{out.confidence:5.2f} {out.visibility:8s} "
              f"\"{out.target}\" {out.latency_s:.1f}s {out.error} "
              f"{out.raw[:110]!r}", flush=True)
print(tally)
