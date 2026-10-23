#!/usr/bin/env python3
"""SENSOR TEST (not a mission): render the downward camera over a survivor and over look-alikes,
send each detector crop to the project's VLM worker, record latency and decision."""
import os, sys, time, json
import cv2, mujoco, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, '..'))
sys.path.insert(0, ROOT); sys.path.insert(0, os.path.join(ROOT, '..', '..', 'src', 'uav_autonomy'))
from controllers.flight_sim import Sim
from controllers.perception import DownCamera
from uav_autonomy import survivor_detection as sd
from uav_autonomy.vlm_client import VlmConfig, VlmWorker
model = sys.argv[1] if len(sys.argv) > 1 else 'gemma3:4b'
out = os.path.join(ROOT, 'results', 'faculty', 'vlm_test'); os.makedirs(out, exist_ok=True)
w = VlmWorker(VlmConfig(endpoint='http://127.0.0.1:11434/v1', model=model, timeout_s=120.0))
targets = [('urban_C', 'survivor_block', 17.0, 55.0), ('urban_C', 'survivor_courtyard', 57.5, 16.0),
           ('urban_C', 'tarp_alley', 23.9, 62.0), ('urban_C', 'barrel_avenue', 23.0, 29.0),
           ('urban_C', 'cone_street', 33.0, 40.0), ('urban_C', 'crate_ball_gap', 41.0, 36.0)]
sim = Sim('urban_C'); cam = DownCamera(sim.m, sim.d); rows = []
for scene, name, x, y in targets:
    r = sim.quad.root
    sim.d.qpos[r:r + 3] = [x, y, 4.5]; sim.d.qpos[r + 3:r + 7] = [1, 0, 0, 0]
    sim.d.qvel[:] = 0; mujoco.mj_forward(sim.m, sim.d)
    frame = cam.frame_bgr(); cands = sd.detect(frame)
    if not cands:
        rows.append({'object': name, 'detector': 'no candidate'}); print(name, 'no candidate'); continue
    c = max(cands, key=lambda k: k.confidence)
    t0 = time.time(); w.submit(frame, (c.x, c.y, c.x + c.width, c.y + c.height), {'name': name})
    res = []
    while not res and time.time() - t0 < 150: time.sleep(0.2); res = w.poll()
    o = res[0] if res else None
    cv2.imwrite(os.path.join(out, f'crop_{name}.png'), frame[max(c.y - 20, 0):c.y + c.height + 20, max(c.x - 20, 0):c.x + c.width + 20])
    row = {'object': name, 'detector_confidence': round(c.confidence, 2), 'kind': getattr(o, 'kind', 'timeout'),
           'decision': getattr(o, 'decision', ''), 'vlm_confidence': getattr(o, 'confidence', None),
           'description': getattr(o, 'description', ''), 'latency_s': round(getattr(o, 'latency_s', -1), 1), 'error': getattr(o, 'error', '')}
    rows.append(row); print(json.dumps(row)[:300], flush=True)
json.dump({'model': model, 'rows': rows}, open(os.path.join(out, f'vlm_test_{model.replace(":","_")}.json'), 'w'), indent=1)
w.stop()
