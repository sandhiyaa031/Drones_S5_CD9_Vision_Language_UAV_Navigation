#!/usr/bin/env python3
"""Minimum useful survivor image size (offline, recorded frames).

Live flights reach 12 m altitude, where the manikin is still ~60 px long.
To find where perception stops working, recorded frames with the survivor
fully visible are reduced in size before being given to the detector: a
frame reduced by a factor s shows the survivor at s times its size, as it
would appear from about 1/s times the range (the pixel footprint grows the
same way; atmospheric and motion effects are not modelled).

For each reduction the detector's hit rate on the true survivor is measured
(strong candidate / weak candidate / nothing). With --vlm, a fixed number of
crops per size is also sent to the VLM (same client and cropping as the
node), to find the size below which the VLM stops confirming.
"""

import argparse
import glob
import json
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import truth as tr  # noqa: E402

from uav_autonomy.survivor_detection import (  # noqa: E402
    DetectorParams, detect)
from uav_autonomy.vlm_client import (  # noqa: E402
    VlmConfig, encode_crop, probe, request_once)

SCALES = (1.0, 0.8, 0.65, 0.5, 0.4, 0.3, 0.25, 0.2, 0.15, 0.1)
BINS = ((0, 10), (10, 15), (15, 20), (20, 30), (30, 45), (45, 70),
        (70, 120), (120, 2000))


def bin_of(size):
    return next(f'{lo}-{hi}' for lo, hi in BINS if lo <= size < hi)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('runs', nargs='+')
    parser.add_argument('--stride', type=int, default=4)
    parser.add_argument('--vlm', action='store_true')
    parser.add_argument('--vlm-per-bin', type=int, default=6)
    parser.add_argument('--output')
    args = parser.parse_args()
    strong_at = DetectorParams().strong_confidence
    table = {}
    vlm_jobs = {}
    for run in args.runs:
        scene = tr.Scene(run.rstrip('/'))
        files = sorted(glob.glob(os.path.join(run, 'frames_raw',
                                              '*.jpg')))[::args.stride]
        for path in files:
            stamp = float(os.path.basename(path)[:-4])
            survivors = [s for s in scene.survivors_at(stamp)
                         if s['class'] == 'VISIBLE'
                         and s['fraction'] >= 0.95]
            if not survivors:
                continue
            image = cv2.imread(path)
            for scale in SCALES:
                small = image if scale == 1.0 else cv2.resize(
                    image, None, fx=scale, fy=scale,
                    interpolation=cv2.INTER_AREA)
                found = detect(small)
                for s in survivors:
                    size = s['full_long_side_px'] * scale
                    box = [v * scale for v in s['bbox']]
                    full = [v * scale for v in s['full_bbox']]
                    hits = [c for c in found
                            if tr.match(c.center, box)
                            or tr.match(c.center, full, 0.1)]
                    cell = table.setdefault(bin_of(size), {
                        'frames': 0, 'strong': 0, 'weak': 0, 'none': 0,
                        'head_px': []})
                    cell['frames'] += 1
                    cell['head_px'].append(s['head_px'] * scale)
                    if any(c.confidence >= strong_at for c in hits):
                        cell['strong'] += 1
                    elif hits:
                        cell['weak'] += 1
                    else:
                        cell['none'] += 1
                    jobs = vlm_jobs.setdefault(bin_of(size), [])
                    x, y, w, h = (int(round(v)) for v in box)
                    if args.vlm and w >= 4 and h >= 4:
                        jobs.append((small, (x, y, w, h), size))
    report = {'detector': {}, 'vlm': {}}
    for label in [f'{lo}-{hi}' for lo, hi in BINS]:
        if label not in table:
            continue
        cell = table[label]
        report['detector'][label] = {
            'frames': cell['frames'],
            'strong_rate': round(cell['strong'] / cell['frames'], 3),
            'weak_rate': round(cell['weak'] / cell['frames'], 3),
            'missed_rate': round(cell['none'] / cell['frames'], 3),
            'head_px_median': round(float(np.median(cell['head_px'])), 1)}
    print('survivor long side [px] -> detector')
    for label, row in report['detector'].items():
        print(f'  {label:9s}', row)
    if args.vlm:
        config = VlmConfig(endpoint=os.environ.get('VLM_ENDPOINT', ''),
                           model=os.environ.get('VLM_MODEL', ''),
                           timeout_s=120.0)
        if not probe(config):
            raise SystemExit('VLM endpoint not available; VLM part skipped')
        print('survivor long side [px] -> VLM (model %s)' % config.model)
        for label in [f'{lo}-{hi}' for lo, hi in BINS]:
            jobs = vlm_jobs.get(label, [])
            if not jobs:
                continue
            step = max(1, len(jobs) // args.vlm_per_bin)
            counts = {'confirm': 0, 'reject': 0, 'uncertain': 0, 'error': 0}
            for image, box, size in jobs[::step][:args.vlm_per_bin]:
                out = request_once(config, encode_crop(image, box, config))
                key = out.decision if out.kind == 'answer' else 'error'
                if key == 'confirm' and out.confidence < 0.6:
                    key = 'uncertain'
                counts[key] += 1
            report['vlm'][label] = counts
            print(f'  {label:9s}', counts, flush=True)
    if args.output:
        json.dump(report, open(args.output, 'w'), indent=1)


if __name__ == '__main__':
    main()
