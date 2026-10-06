#!/usr/bin/env python3
"""Score survivor perception against OFFLINE ground truth.

Two levels are scored separately, never mixed:

  candidate detector   the deterministic image detector, frame by frame
      'live'     the raw candidates the node logged for every camera frame
      'replay'   the same detector re-run on the recorded camera frames
  published output     what the node published after temporal confirmation
                       (stable tracks and their semantic state), and the VLM
                       requests and answers it logged

Frame truth (truth.py): each survivor is VISIBLE (>= 60 % of its upper
surface in view and unoccluded), PARTIAL (15-60 %) or NOT_VISIBLE.

  TP  a strong candidate whose centre lies in a VISIBLE survivor's box
  FN  a VISIBLE survivor with no strong candidate on it
  FP  a strong candidate on nothing that is a survivor
      (a strong candidate on a PARTIAL survivor is neither TP nor FP; PARTIAL
      frames are reported on their own)
  precision = TP / (TP + FP)   recall = TP / (TP + FN)
  false-positive rate = frames with an FP / all airborne frames
  false-negative rate = FN / (TP + FN)
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

from uav_autonomy.survivor_confirmation import (  # noqa: E402
    ConfirmationParams, STATE_NAMES, SurvivorMonitor, semantic_state)
from uav_autonomy.survivor_detection import (  # noqa: E402
    DetectorParams, detect)

STRONG = DetectorParams().strong_confidence
POSITIVE_STATES = ('CANDIDATE', 'SURVIVOR_CONFIRMED')


def jsonl(path):
    out = []
    if os.path.exists(path):
        for line in open(path):
            try:
                out.append(json.loads(line))
            except ValueError:
                continue
    return out


def center(bbox):
    return bbox[0] + bbox[2] / 2.0, bbox[1] + bbox[3] / 2.0


def on_survivor(bbox, survivor):
    """Is this box on the survivor (its visible part, or where it stands)?"""
    return (tr.match(center(bbox), survivor['bbox'])
            or tr.match(center(bbox), survivor['full_bbox'], 0.1))


def pct(values, q):
    return round(float(np.percentile(values, q)), 3) if len(values) else None


class Counter:
    """Frame-level confusion counts for one level."""

    def __init__(self):
        self.tp = self.fn = self.fp = 0
        self.frames = self.fp_frames = self.no_survivor_frames = 0
        self.fp_no_survivor_frames = 0
        self.partial = {'frames': 0, 'strong': 0, 'weak': 0, 'none': 0}
        self.weak_on_nothing = 0
        self.on_hidden_survivor = 0
        self.fp_objects = {}
        self.by_size = {}

    def add(self, survivors, objects, boxes):
        """``boxes``: list of (bbox, strong: bool) reported in this frame."""
        self.frames += 1
        used = set()
        any_visible = False
        for s in survivors:
            # on the survivor: inside the box of its visible part, or
            # inside the box it would have if nothing hid it (an occluder
            # can leave two separate visible pieces)
            hits = [i for i, (b, _) in enumerate(boxes)
                    if tr.match(center(b), s['bbox'])
                    or tr.match(center(b), s['full_bbox'], 0.1)]
            strong = [i for i in hits if boxes[i][1]]
            used.update(hits)
            if s['class'] == 'VISIBLE':
                any_visible = True
                found = bool(strong)
                self.tp += found
                self.fn += not found
                size = s['full_long_side_px']
                label = next(f'{lo}-{hi}' for lo, hi in (
                    (0, 20), (20, 40), (40, 60), (60, 90), (90, 140),
                    (140, 220), (220, 5000)) if lo <= size < hi)
                row = self.by_size.setdefault(label, [0, 0])
                row[0] += 1
                row[1] += found
            elif s['class'] == 'PARTIAL':
                any_visible = True
                self.partial['frames'] += 1
                self.partial['strong' if strong else
                             'weak' if hits else 'none'] += 1
        # a candidate on a survivor that truth calls NOT_VISIBLE (under
        # 15 % in view) is the survivor, not a false positive
        for s in survivors:
            if s['class'] != 'NOT_VISIBLE':
                continue
            hidden = [i for i, (b, _) in enumerate(boxes)
                      if i not in used and tr.match(center(b),
                                                    s['full_bbox'])]
            self.on_hidden_survivor += len(hidden)
            used.update(hidden)
        false = [i for i, (b, strong) in enumerate(boxes)
                 if i not in used and strong]
        self.weak_on_nothing += sum(
            1 for i, (b, strong) in enumerate(boxes)
            if i not in used and not strong)
        self.fp += len(false)
        self.fp_frames += bool(false)
        if not any_visible and not any(
                s['class'] != 'NOT_VISIBLE' for s in survivors):
            self.no_survivor_frames += 1
            self.fp_no_survivor_frames += bool(false)
        for i in false:
            name = next((o['name'] for o in objects
                         if tr.match(center(boxes[i][0]), o['bbox'], 0.15)),
                        'background')
            self.fp_objects[name] = self.fp_objects.get(name, 0) + 1

    def summary(self):
        p = self.tp / (self.tp + self.fp) if self.tp + self.fp else None
        r = self.tp / (self.tp + self.fn) if self.tp + self.fn else None
        f1 = (2 * p * r / (p + r)) if p and r else (
            0.0 if p is not None and r is not None else None)

        def rnd(v):
            return None if v is None else round(v, 4)
        return {
            'frames': self.frames, 'tp': self.tp, 'fn': self.fn,
            'fp': self.fp, 'precision': rnd(p), 'recall': rnd(r),
            'f1': rnd(f1),
            'false_positive_rate_per_frame': rnd(
                self.fp_frames / self.frames if self.frames else None),
            'frames_without_survivor': self.no_survivor_frames,
            'false_positive_rate_without_survivor': rnd(
                self.fp_no_survivor_frames / self.no_survivor_frames
                if self.no_survivor_frames else None),
            'false_negative_rate': rnd(1 - r if r is not None else None),
            'partial_frames': self.partial,
            'weak_candidates_on_nothing': self.weak_on_nothing,
            'candidates_on_barely_visible_survivor': self.on_hidden_survivor,
            'false_positives_by_object': self.fp_objects,
            'recall_by_true_size_px': {
                k: {'frames': v[0], 'detected': v[1],
                    'recall': round(v[1] / v[0], 3)}
                for k, v in sorted(self.by_size.items(),
                                   key=lambda kv: int(kv[0].split('-')[0]))}}


def evaluate(run_dir, replay=True):
    scene = tr.Scene(run_dir)
    status = jsonl(os.path.join(run_dir, 'survivor.jsonl'))
    msgs = jsonl(os.path.join(run_dir, 'survivor_msgs.jsonl'))
    airborne = [r for r in status
                if scene.camera(r['stamp'])[2] > 1.0]
    live, published = Counter(), Counter()
    first_visible, first_stable, first_confirmed = {}, {}, {}
    first_partial = {}
    false_confirmed = {}
    confidences, sizes = [], []
    states = {}
    held_frames, temporal_delay = 0, {}
    confirmed_frames = {'on_survivor': 0, 'on_non_survivor': 0}
    for row in airborne:
        stamp = row['stamp']
        survivors = scene.survivors_at(stamp)
        objects = scene.objects_at(stamp)
        states[row['state']] = states.get(row['state'], 0) + 1
        live.add(survivors, objects, [
            (c[:4], c[4] >= STRONG) for c in row['raw_candidates']])
        # published level: tracks detected in this frame (a track held
        # after its object left the view is flagged "not measured" in the
        # message and is counted separately)
        published.add(survivors, objects, [
            (t['bbox'], t['state'] in POSITIVE_STATES)
            for t in row['tracks']
            if t['state'] != 'REJECTED' and t['measured']])
        held_frames += sum(1 for t in row['tracks'] if not t['measured'])
        for t in row['tracks']:
            temporal_delay.setdefault(t['id'], t['age'])
        for s in survivors:
            if s['class'] == 'VISIBLE':
                first_visible.setdefault(s['name'], stamp)
            if s['class'] in ('VISIBLE', 'PARTIAL'):
                first_partial.setdefault(s['name'], stamp)
            for t in row['tracks']:
                if not on_survivor(t['bbox'], s):
                    continue
                if s['class'] != 'NOT_VISIBLE':
                    first_stable.setdefault(s['name'], stamp)
                    if t['state'] == 'SURVIVOR_CONFIRMED':
                        first_confirmed.setdefault(s['name'], stamp)
                if s['class'] == 'VISIBLE' and t['measured']:
                    confidences.append(t['conf'])
                    sizes.append([s['altitude_m'], s['range_m'],
                                  s['full_long_side_px'], s['head_px'],
                                  t['bbox'][2], t['bbox'][3], t['conf']])
        for t in row['tracks']:
            if t['state'] != 'SURVIVOR_CONFIRMED' or not t['measured']:
                continue
            if any(on_survivor(t['bbox'], s) for s in survivors):
                confirmed_frames['on_survivor'] += 1
            else:
                confirmed_frames['on_non_survivor'] += 1
        for t in row['tracks']:
            if t['state'] == 'SURVIVOR_CONFIRMED' and t['measured'] \
                    and not any(on_survivor(t['bbox'], s)
                                for s in survivors):
                name = next((o['name'] for o in objects
                             if tr.match(center(t['bbox']), o['bbox'],
                                         0.15)), 'background')
                false_confirmed[t['id']] = name

    # VLM requests / answers logged by the node, labelled offline
    events = [e for r in status for e in r.get('vlm_events', [])]
    answers = []
    by_stamp = {r['stamp']: r for r in status}
    for e in events:
        if e['kind'] == 'request':
            continue
        label = 'unknown'
        row = by_stamp.get(e.get('frame_stamp'))
        if row is not None:
            track = next((t for t in row['tracks']
                          if t['id'] == e['track']), None)
            if track is not None:
                survivors = scene.survivors_at(row['stamp'])
                hit = next((s for s in survivors
                            if on_survivor(track['bbox'], s)), None)
                if hit is not None:
                    label = ('survivor_' + hit['class'].lower())
                else:
                    label = next(
                        (o['kind'] + ':' + o['name']
                         for o in scene.objects_at(row['stamp'])
                         if tr.match(center(track['bbox']), o['bbox'],
                                     0.15)), 'background')
        answers.append({**e, 'truth': label})
    ok = [a for a in answers if a['kind'] == 'answer']
    latency = [a['latency_s'] for a in ok]

    delays = {}
    for name, t0 in first_partial.items():
        delays[name] = {
            'first_in_view_s': round(t0, 3),
            'first_fully_visible_s': first_visible.get(name),
            'candidate_delay_s': (
                round(first_stable[name] - t0, 3)
                if name in first_stable else None),
            'confirmation_delay_s': (
                round(first_confirmed[name] - t0, 3)
                if name in first_confirmed else None)}

    e2e = [m['sim_time_received'] - m['stamp'] for m in msgs
           if m.get('sim_time_received')]
    stamps = np.array([r['stamp'] for r in airborne])
    cpu = []
    cpu_path = os.path.join(run_dir, 'survivor_cpu.json')
    if os.path.exists(cpu_path):
        cpu = json.load(open(cpu_path))['cpu_percent_of_one_core']
    sizes = np.array(sizes)
    result = {
        'run': os.path.relpath(run_dir), 'world': scene.world,
        'survivors_in_world': [s['name'] for s in scene.survivors],
        'frames_airborne': len(airborne),
        'camera_rate_hz': (round((len(stamps) - 1)
                                 / (stamps[-1] - stamps[0]), 1)
                           if len(stamps) > 1 else None),
        'image': status[0]['size'] + [status[0]['encoding']] if status
        else None,
        'frame_states': states,
        'vlm_status': sorted({r['vlm_status'] for r in status}),
        'candidate_detector_live': live.summary(),
        'published_after_temporal_confirmation': published.summary(),
        'survivor_delays': delays,
        'temporal_confirmation_delay_s': {
            'tracks': len(temporal_delay),
            'median': pct(list(temporal_delay.values()), 50),
            'max': pct(list(temporal_delay.values()), 100)},
        'track_frames_held_without_detection': held_frames,
        'survivors_confirmed': sorted(first_confirmed),
        'survivors_missed': sorted(set(first_visible) - set(first_stable)),
        'false_confirmations': false_confirmed,
        'confirmed_frames': confirmed_frames,
        'vlm': {
            'requests': sum(e['kind'] == 'request' for e in events),
            'answers': len(ok),
            'errors': sum(a['kind'] == 'error' for a in answers),
            'unavailable': sum(a['kind'] == 'unavailable' for a in answers),
            'latency_s': {'p50': pct(latency, 50), 'p95': pct(latency, 95),
                          'max': pct(latency, 100)},
            'log': [{k: a.get(k) for k in (
                'track', 'kind', 'truth', 'decision', 'confidence',
                'visibility', 'target', 'description', 'latency_s',
                'crop_px', 'error')}
                for a in answers]},
        'timing': {
            'callback_ms': {
                'p50': pct([r['processing_ms'] for r in status], 50),
                'p95': pct([r['processing_ms'] for r in status], 95),
                'max': pct([r['processing_ms'] for r in status], 100)},
            'detector_ms_p50': pct([r['detect_ms'] for r in status], 50),
            'end_to_end_latency_s': {'p50': pct(e2e, 50), 'p95': pct(e2e, 95),
                                     'max': pct(e2e, 100)},
            'node_cpu_percent_of_one_core': {
                'median': pct(cpu, 50), 'max': pct(cpu, 100)}},
        'image_quality': None if not len(sizes) else {
            'altitude_m': [round(float(sizes[:, 0].min()), 2),
                           round(float(sizes[:, 0].max()), 2)],
            'range_m': [round(float(sizes[:, 1].min()), 2),
                        round(float(sizes[:, 1].max()), 2)],
            'true_long_side_px': [round(float(sizes[:, 2].min()), 1),
                                  round(float(sizes[:, 2].max()), 1)],
            'head_px': [round(float(sizes[:, 3].min()), 1),
                        round(float(sizes[:, 3].max()), 1)],
            'detected_bbox_px_median': [
                round(float(np.median(sizes[:, 4])), 1),
                round(float(np.median(sizes[:, 5])), 1)],
            'candidate_confidence': {
                'min': round(float(sizes[:, 6].min()), 2),
                'median': round(float(np.median(sizes[:, 6])), 2)}}}

    if replay:
        counter, monitor = Counter(), SurvivorMonitor(ConfirmationParams())
        stable_counter = Counter()
        blur = []
        files = sorted(glob.glob(os.path.join(run_dir, 'frames_raw',
                                              '*.jpg')))
        n = 0
        for path in files:
            stamp = float(os.path.basename(path)[:-4])
            if scene.camera(stamp)[2] <= 1.0:
                continue
            image = cv2.imread(path)
            found = detect(image)
            n += 1
            survivors = scene.survivors_at(stamp)
            objects = scene.objects_at(stamp)
            counter.add(survivors, objects, [
                ((c.x, c.y, c.width, c.height), c.confidence >= STRONG)
                for c in found])
            stable = monitor.update(stamp, found)
            stable_counter.add(survivors, objects, [
                ((t.candidate.x, t.candidate.y, t.candidate.width,
                  t.candidate.height),
                 STATE_NAMES[semantic_state(t, monitor.params)]
                 in POSITIVE_STATES) for t in stable if t.measured])
            for s in survivors:
                if s['class'] == 'VISIBLE' and s['bbox']:
                    x, y, w, h = (int(max(0, v)) for v in s['bbox'])
                    crop = cv2.cvtColor(image[y:y + h, x:x + w],
                                        cv2.COLOR_BGR2GRAY)
                    if crop.size > 100:
                        blur.append(float(cv2.Laplacian(
                            crop, cv2.CV_64F).var()))
        result['candidate_detector_replay_recorded_frames'] = {
            'frames': n, **counter.summary()}
        result['temporal_confirmation_replay_recorded_frames'] = \
            stable_counter.summary()
        result['image_quality_sharpness_laplacian_var'] = {
            'p05': pct(blur, 5), 'median': pct(blur, 50)}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('runs', nargs='+')
    parser.add_argument('--no-replay', action='store_true')
    parser.add_argument('--output')
    args = parser.parse_args()
    results = []
    for run in args.runs:
        res = evaluate(run.rstrip('/'), replay=not args.no_replay)
        results.append(res)
        live = res['candidate_detector_live']
        pub = res['published_after_temporal_confirmation']
        print(f"\n== {res['run']} (world {res['world']}) frames "
              f"{res['frames_airborne']} @ {res['camera_rate_hz']} Hz, "
              f"states {res['frame_states']}, {res['vlm_status']}")
        for label, c in (('detector/live', live), ('published', pub)):
            print(f"   {label:14s} TP {c['tp']} FN {c['fn']} FP {c['fp']} "
                  f"P {c['precision']} R {c['recall']} F1 {c['f1']} "
                  f"FP-rate {c['false_positive_rate_per_frame']} "
                  f"partial {c['partial_frames']} FP on "
                  f"{c['false_positives_by_object']}")
        if 'candidate_detector_replay_recorded_frames' in res:
            c = res['candidate_detector_replay_recorded_frames']
            print(f"   detector/replay frames {c['frames']} TP {c['tp']} "
                  f"FN {c['fn']} FP {c['fp']} P {c['precision']} "
                  f"R {c['recall']} F1 {c['f1']}")
        print('   delays', res['survivor_delays'], '| first detection -> '
              'stable track:', res['temporal_confirmation_delay_s'],
              '| held frames', res['track_frames_held_without_detection'])
        print('   confirmed', res['survivors_confirmed'], 'missed',
              res['survivors_missed'], 'false confirmations',
              res['false_confirmations'])
        v = res['vlm']
        print(f"   VLM requests {v['requests']} answers {v['answers']} "
              f"errors {v['errors']} unavailable {v['unavailable']} "
              f"latency {v['latency_s']}")
        for a in v['log']:
            print('      ', {k: x for k, x in a.items() if x is not None})
        print('   timing', res['timing'])
        print('   image quality', res['image_quality'],
              res.get('image_quality_sharpness_laplacian_var'))
        print('   recall by size', live['recall_by_true_size_px'])
    if args.output:
        json.dump(results, open(args.output, 'w'), indent=1)


if __name__ == '__main__':
    main()
