#!/usr/bin/env python3
"""OFFLINE ground truth for survivor-perception scoring.

For a recorded flight this computes, for any camera frame time, where each
survivor manikin (and each distractor object) appears in the downward camera
image and how much of it is visible. It uses the simulator's recorded UAV
pose (poses.csv), the world description (SDF) and the positions written by
make_worlds.py.

This module is evaluation tooling. Nothing that runs in flight imports it,
and the perception node never sees any of this information.
"""

import csv
import json
import math
import os
import xml.etree.ElementTree as ET

import numpy as np

WS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
IMAGE_W, IMAGE_H = 1280, 960
HFOV = 1.74                                   # mono_cam model
FX = FY = (IMAGE_W / 2.0) / math.tan(HFOV / 2.0)
CX, CY = IMAGE_W / 2.0, IMAGE_H / 2.0
CAMERA_OFFSET = np.array([0.0, 0.0, 0.10])    # x500_rescue include pose


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def quat_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


# sensor frame in the model: optical axis (+x) straight down
R_MODEL_SENSOR = rpy_matrix(0.0, 1.5707, 0.0)


def _pose(text):
    v = [float(x) for x in (text or '0 0 0 0 0 0').split()]
    return np.array(v[:3]), rpy_matrix(*v[3:6])


def world_boxes(sdf_path, skip=('ground_plane',)):
    """Oriented boxes of every static shape in the world (occluders).

    Spheres and cylinders are represented by their bounding boxes.
    Returns a list of (name, centre, rotation, half_size).
    """
    world = ET.parse(sdf_path).getroot().find('world')
    out = []
    for model in world.findall('model'):
        name = model.get('name')
        if name in skip or name.startswith('survivor'):
            continue
        pm, rm = _pose(model.findtext('pose'))
        for link in model.findall('link'):
            pl, rl = _pose(link.findtext('pose'))
            for visual in link.findall('visual'):
                pv, rv = _pose(visual.findtext('pose'))
                geometry = visual.find('geometry')
                if geometry is None or len(geometry) == 0:
                    continue
                shape = geometry[0]
                if shape.tag == 'box':
                    size = [float(v) for v in shape.findtext('size').split()]
                elif shape.tag == 'sphere':
                    r = float(shape.findtext('radius'))
                    size = [2 * r] * 3
                elif shape.tag == 'cylinder':
                    r = float(shape.findtext('radius'))
                    size = [2 * r, 2 * r, float(shape.findtext('length'))]
                else:
                    continue
                rot = rm @ rl @ rv
                centre = pm + rm @ (pl + rl @ pv)
                out.append((name, centre, rot, np.array(size) / 2.0))
    return out


def _box_top_points(centre, size, rot=np.eye(3), n=3):
    """Grid on the top face of a box (manikin frame)."""
    sx, sy, sz = size
    pts = []
    for a in np.linspace(-0.45, 0.45, n):
        for b in np.linspace(-0.45, 0.45, n):
            pts.append(np.asarray(centre) + rot @ np.array(
                [a * sx, b * sy, sz / 2.0]))
    return pts


def manikin_points():
    """Sample points of the manikin, in its own frame.

    'upper': points on upward-facing surfaces of head, torso and arms (what
    a camera above can see); 'all' adds the legs, for the bounding box.
    """
    upper = []
    head = np.array([0.0, 0.0, 1.53])
    for elevation in (20, 50, 80):
        for azimuth in range(0, 360, 60):
            e, a = math.radians(elevation), math.radians(azimuth)
            upper.append(head + 0.22 * np.array(
                [math.cos(e) * math.cos(a), math.cos(e) * math.sin(a),
                 math.sin(e)]))
    upper += _box_top_points([0, 0, 1.03], [0.42, 0.28, 0.68])
    upper += _box_top_points([0, 0.34, 1.05], [0.18, 0.62, 0.18],
                             rpy_matrix(0.12, 0, 0.18))
    upper += _box_top_points([0, -0.34, 1.05], [0.18, 0.62, 0.18],
                             rpy_matrix(-0.12, 0, -0.18))
    legs = (_box_top_points([0, 0.12, 0.34], [0.22, 0.18, 0.68], n=2)
            + _box_top_points([0, -0.12, 0.34], [0.22, 0.18, 0.68], n=2))
    return np.array(upper), np.array(upper + legs)


def _ray_hits_box(origin, target, centre, rot, half, shrink=0.02):
    """True if the segment origin->target passes through the box."""
    o = rot.T @ (origin - centre)
    d = rot.T @ (target - origin)
    lo, hi = 0.0, 1.0 - 1e-3
    h = half - shrink
    for k in range(3):
        if abs(d[k]) < 1e-12:
            if abs(o[k]) > h[k]:
                return False
            continue
        t0, t1 = (-h[k] - o[k]) / d[k], (h[k] - o[k]) / d[k]
        if t0 > t1:
            t0, t1 = t1, t0
        lo, hi = max(lo, t0), min(hi, t1)
        if lo > hi:
            return False
    return True


class Scene:
    """World truth for one run."""

    def __init__(self, run_dir):
        info = open(os.path.join(run_dir, 'trial_info.txt')).read()
        world = 'base'
        for token in info.split():
            if token.startswith('world=') and token != \
                    'world=collapsed_building_rescue':
                world = token.split('=', 1)[1]
        self.world = world
        spec = json.load(open(os.path.join(
            WS, 'results/phase7/worlds', f'{world}.json')))
        self.survivors = spec['survivors']
        self.objects = spec['objects']
        self.boxes = world_boxes(os.path.join(WS, spec['sdf']))
        self.upper, self.all_points = manikin_points()
        t, p, q = [], [], []
        with open(os.path.join(run_dir, 'poses.csv')) as handle:
            for row in csv.DictReader(handle):
                if row['name'] != 'x500_rescue_0':
                    continue
                t.append(float(row['t_sim']))
                p.append([float(row[k]) for k in ('x', 'y', 'z')])
                q.append([float(row[k]) for k in ('qx', 'qy', 'qz', 'qw')])
        self.t, self.p, self.q = np.array(t), np.array(p), np.array(q)
        self.debris = {}
        if os.path.exists(os.path.join(run_dir, 'releases.json')):
            sizes = {r['name']: r['size'] for r in json.load(
                open(os.path.join(run_dir, 'releases.json')))}
            rows = {}
            with open(os.path.join(run_dir, 'poses.csv')) as handle:
                for row in csv.DictReader(handle):
                    if row['name'] in sizes:
                        rows.setdefault(row['name'], []).append(
                            [float(row[k]) for k in
                             ('t_sim', 'x', 'y', 'z')])
            self.debris = {k: (np.array(v), sizes[k])
                           for k, v in rows.items()}

    def camera(self, stamp):
        """Camera position and world_from_sensor rotation at a frame time."""
        i = int(np.clip(np.searchsorted(self.t, stamp), 1, len(self.t) - 1))
        if abs(self.t[i - 1] - stamp) < abs(self.t[i] - stamp):
            i -= 1
        w = 0.0
        j = min(i + 1, len(self.t) - 1)
        if self.t[j] > self.t[i]:
            w = float(np.clip((stamp - self.t[i]) / (self.t[j] - self.t[i]),
                              0.0, 1.0))
        position = (1 - w) * self.p[i] + w * self.p[j]
        rot = quat_matrix(*self.q[i])
        return position + rot @ CAMERA_OFFSET, rot @ R_MODEL_SENSOR, \
            float(position[2])

    @staticmethod
    def project(points, cam_p, cam_r):
        """World points -> (u, v), depth along the optical axis."""
        s = (np.atleast_2d(points) - cam_p) @ cam_r      # sensor frame
        depth = s[:, 0]
        safe = np.maximum(depth, 1e-6)
        uv = np.column_stack([CX - FX * s[:, 1] / safe,
                              CY - FY * s[:, 2] / safe])
        return uv, depth

    def _visible(self, points, cam_p, cam_r, ignore=()):
        uv, depth = self.project(points, cam_p, cam_r)
        inside = ((depth > 0.1) & (uv[:, 0] >= 0) & (uv[:, 0] < IMAGE_W)
                  & (uv[:, 1] >= 0) & (uv[:, 1] < IMAGE_H))
        clear = inside.copy()
        for i, point in enumerate(np.atleast_2d(points)):
            if not inside[i]:
                continue
            for name, centre, rot, half in self.boxes:
                if name in ignore:
                    continue
                if _ray_hits_box(cam_p, point, centre, rot, half):
                    clear[i] = False
                    break
        return uv, depth, inside, clear

    def survivors_at(self, stamp):
        """Truth for every survivor in this frame."""
        cam_p, cam_r, altitude = self.camera(stamp)
        out = []
        for survivor in self.survivors:
            x, y = survivor['position']
            rot = rpy_matrix(0, 0, survivor.get('yaw', 0.0))
            base = np.array([x, y, 0.0])
            upper = base + self.upper @ rot.T
            everything = base + self.all_points @ rot.T
            uv, depth, inside, clear = self._visible(upper, cam_p, cam_r)
            fraction = float(clear.mean())
            uv_all, _, in_all, clear_all = self._visible(everything, cam_p,
                                                         cam_r)
            shown = uv_all[clear_all] if clear_all.any() else uv_all[in_all]
            bbox = None
            if len(shown):
                x0, y0 = shown.min(axis=0)
                x1, y1 = shown.max(axis=0)
                bbox = [float(x0), float(y0), float(x1 - x0), float(y1 - y0)]
            # where and how large the whole manikin would be if nothing hid
            # it (used so that a detection of a barely visible survivor is
            # not counted as a false positive)
            full_uv, full_depth = self.project(upper, cam_p, cam_r)
            span = full_uv.max(axis=0) - full_uv.min(axis=0)
            full_bbox = None
            every_uv, every_depth = self.project(everything, cam_p, cam_r)
            front = every_depth > 0.1          # (low passes: head above)
            if front.any():
                fx0, fy0 = every_uv[front].min(axis=0)
                fx1, fy1 = every_uv[front].max(axis=0)
                full_bbox = [float(fx0), float(fy0), float(fx1 - fx0),
                             float(fy1 - fy0)]
            head = base + rot @ np.array([0.0, 0.0, 1.53])
            rng = float(np.linalg.norm(head - cam_p))
            out.append({
                'name': survivor['name'], 'fraction': round(fraction, 3),
                'class': ('VISIBLE' if fraction >= 0.6 else
                          'PARTIAL' if fraction >= 0.15 else 'NOT_VISIBLE'),
                'bbox': bbox, 'full_bbox': full_bbox,
                'range_m': round(rng, 2),
                'altitude_m': round(altitude, 2),
                'full_long_side_px': round(float(span.max()), 1),
                'head_px': round(0.44 * FX / max(rng, 0.1), 1)})
        return out

    def objects_at(self, stamp):
        """Projected boxes of distractors / rubble / debris in this frame."""
        cam_p, cam_r, _ = self.camera(stamp)
        out = []
        items = [(o['name'], o['kind'], np.array(o['position']),
                  np.array(o['size']), rpy_matrix(0, 0, o.get('yaw', 0.0)))
                 for o in self.objects]
        for name, (track, size) in self.debris.items():
            if not (track[0, 0] <= stamp <= track[-1, 0]):
                continue
            pos = np.array([np.interp(stamp, track[:, 0], track[:, k])
                            for k in (1, 2, 3)])
            items.append((name, 'debris', pos, np.array(size), np.eye(3)))
        for name, centre, rot, half in self.boxes:
            if name.startswith(('rubble_', 'fallen_concrete')) and not any(
                    name == o['name'] for o in self.objects):
                items.append((name, 'rubble', centre, half * 2, rot))
        for name, kind, centre, size, rot in items:
            corners = np.array([centre + rot @ (np.array([a, b, c]) * size
                                                / 2.0)
                                for a in (-1, 1) for b in (-1, 1)
                                for c in (-1, 1)])
            uv, depth, inside, clear = self._visible(
                np.vstack([corners, centre]), cam_p, cam_r, ignore=(name,))
            if not clear.any():
                continue
            x0, y0 = np.clip(uv[inside].min(axis=0), 0, [IMAGE_W, IMAGE_H])
            x1, y1 = np.clip(uv[inside].max(axis=0), 0, [IMAGE_W, IMAGE_H])
            out.append({'name': name, 'kind': kind,
                        'fraction': round(float(clear.mean()), 2),
                        'bbox': [float(x0), float(y0), float(x1 - x0),
                                 float(y1 - y0)]})
        return out


def match(center, bbox, grow=0.3):
    """Is a detection centre inside a truth box grown by ``grow``?"""
    if bbox is None:
        return False
    x, y, w, h = bbox
    pad = grow * max(w, h, 20.0)
    return (x - pad <= center[0] <= x + w + pad
            and y - pad <= center[1] <= y + h + pad)
