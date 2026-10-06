#!/usr/bin/env python3
"""Build the controlled crop dataset for the VLM evaluation (offline).

Crops are cut from recorded camera frames of the Phase 7 flights with the
same cropping code the node uses (padding, resizing, JPEG). The label of a
crop comes from offline ground truth (truth.py), never from the detector:

    survivor_clear     manikin >= 60 % visible            expect: confirm
    survivor_partial   manikin 15-60 % visible            ambiguous
    distractor         placed non-survivor objects        expect: reject
    rubble             rubble blocks                      expect: reject
    debris             falling / fallen debris boxes      expect: reject
    structure_shadow   roof, slab edges, shadows, floor   expect: reject

Selection is mechanical, so that no example is chosen for being easy or
hard: per run, each object is sampled with a fixed time stride up to a fixed
number of crops; if a category then exceeds its cap, every k-th crop is
kept.
"""

import argparse
import glob
import json
import os
import sys

import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import truth as tr  # noqa: E402

from uav_autonomy.vlm_client import VlmConfig, encode_crop  # noqa: E402

EXPECT = {'survivor_clear': 'confirm', 'survivor_partial': 'ambiguous',
          'distractor': 'reject', 'rubble': 'reject', 'debris': 'reject',
          'structure_shadow': 'reject'}
# fixed background boxes (x, y, w, h) used on frames without a survivor
BACKGROUND = [(80, 60, 260, 200), (520, 120, 240, 200), (900, 80, 280, 220),
              (120, 420, 240, 220), (500, 400, 280, 240),
              (880, 600, 300, 260)]


def clip(bbox, min_side=16):
    x, y, w, h = bbox
    x0, y0 = max(0, int(x)), max(0, int(y))
    x1, y1 = min(tr.IMAGE_W, int(x + w)), min(tr.IMAGE_H, int(y + h))
    if x1 - x0 < min_side or y1 - y0 < min_side:
        return None
    return x0, y0, x1 - x0, y1 - y0


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('runs', nargs='+')
    parser.add_argument('--out', default='results/phase7/vlm_dataset')
    parser.add_argument('--stride-s', type=float, default=1.5,
                        help='seconds between crops of one object in a run')
    parser.add_argument('--per-object', type=int, default=4,
                        help='most crops of one object from one run')
    args = parser.parse_args()
    os.makedirs(os.path.join(args.out, 'crops'), exist_ok=True)
    config = VlmConfig()
    items = []
    for run in args.runs:
        run = run.rstrip('/')
        scene = tr.Scene(run)
        last, count = {}, {}

        def take(key, category, stamp):
            """Stride and per-object limits, applied identically everywhere."""
            stride = 0.4 if category == 'debris' else args.stride_s
            limit = args.per_object * (3 if category.startswith('survivor')
                                       else 1)
            if stamp - last.get(key, -1e9) < stride:
                return False
            if count.get(key, 0) >= limit:
                return False
            last[key] = stamp
            count[key] = count.get(key, 0) + 1
            return True

        for path in sorted(glob.glob(os.path.join(run, 'frames_raw',
                                                  '*.jpg'))):
            stamp = float(os.path.basename(path)[:-4])
            if scene.camera(stamp)[2] <= 1.5:
                continue
            image = None
            found = []
            survivors = scene.survivors_at(stamp)
            for s in survivors:
                if s['class'] == 'NOT_VISIBLE' or s['bbox'] is None:
                    continue
                category = ('survivor_clear' if s['class'] == 'VISIBLE'
                            else 'survivor_partial')
                box = clip(s['bbox'])
                if box and take(s['name'] + category, category, stamp):
                    found.append((category, s['name'], box, {
                        'fraction': s['fraction'],
                        'range_m': s['range_m'],
                        'altitude_m': s['altitude_m']}))
            for o in scene.objects_at(stamp):
                if o['kind'] == 'occluder' or o['fraction'] < (
                        0.3 if o['kind'] == 'debris' else 0.5):
                    continue
                box = clip(o['bbox'])
                if box and take(o['name'], o['kind'], stamp):
                    found.append((o['kind'], o['name'], box, {}))
            if not scene.survivors and not scene.objects:
                for i, bg in enumerate(BACKGROUND):
                    if take(f'bg{i}', 'structure_shadow', stamp):
                        found.append(('structure_shadow', f'region_{i}', bg,
                                      {}))
            for category, name, box, extra in found:
                items.append({
                    'category': category,
                    'expected': EXPECT[category], 'object': name,
                    'run': os.path.basename(run), 'stamp': stamp,
                    'bbox': list(box), 'frame': path, **extra})
    # category caps by uniform thinning
    caps = {'survivor_clear': 45, 'survivor_partial': 30, 'rubble': 30,
            'structure_shadow': 24, 'distractor': 45, 'debris': 20}
    kept = []
    for category, cap in caps.items():
        group = [i for i in items if i['category'] == category]
        if len(group) > cap:
            step = len(group) / cap
            group = [group[int(k * step)] for k in range(cap)]
        kept += group
    items = kept
    for old in glob.glob(os.path.join(args.out, 'crops', '*.jpg')):
        os.remove(old)
    for index, item in enumerate(items):
        image = cv2.imread(item.pop('frame'))
        item['id'] = f"{index:04d}_{item['category']}"
        open(os.path.join(args.out, 'crops', item['id'] + '.jpg'),
             'wb').write(encode_crop(image, item['bbox'], config))
    json.dump(items, open(os.path.join(args.out, 'dataset.json'), 'w'),
              indent=1)
    summary = {}
    for item in items:
        summary[item['category']] = summary.get(item['category'], 0) + 1
    print(len(items), 'crops:', summary)


if __name__ == '__main__':
    main()
