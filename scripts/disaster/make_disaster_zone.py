#!/usr/bin/env python3
"""Generate the urban earthquake-collapse disaster zone (static geometry).

Everything is procedural: textures (numpy/OpenCV noise, stains, cracks),
meshes (damaged facades built from a cell grid, jittered concrete slabs,
rubble piles from random convex chunks) and collision boxes that lie inside
the visible geometry. No external asset is used (see
docs/DISASTER_ASSET_LICENSES.md).

Frames: Gazebo world ENU (x east, y north, z up), metres. The UAV base pad is
at the world origin, PX4 local NED = (world y, world x, -world z).

Layout (about 120 m x 110 m):

   y 100 +-----------------------------------------------------------+
         |            street (blocked at y~76 by fallen panels)       |
      68 |  B1 collapsed      alley   |  |   B2 commercial block,     |
      48 |  apartment block   (open)  |  |   facade fallen            |
      32 |=========== avenue (east-west, y 22..32) ====================|
      22 | B4 leaning   |  |          B3 U-shaped complex, courtyard  |
         | tower        |  |          opening north to the avenue     |
       0 |  BASE pad    |  |                                          |
     -14 +-----------------------------------------------------------+
        -40            28..38 (north-south street)                  90  x

Outputs (all new; nothing existing is overwritten)
    src/uav_autonomy/models_disaster/dz_assets/        meshes + textures
    src/uav_autonomy/worlds/disaster/<scenario>.sdf    with shadows (demo)
    src/uav_autonomy/worlds/disaster_fast/<scenario>.sdf  no shadows
    results/phase7/worlds/<scenario>.json              truth, OFFLINE scoring
                                                        only (same format as
                                                        the village worlds)

Usage: make_disaster_zone.py [--scenario NAME ...]   (default: all)
"""

import argparse
import importlib.util
import json
import math
import os

import cv2
import numpy as np
from scipy.spatial import ConvexHull

WS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
ASSETS = os.path.join(WS, 'src/uav_autonomy/models_disaster/dz_assets')
MESH_DIR = os.path.join(ASSETS, 'meshes')
OUT_SDF = os.path.join(WS, 'src/uav_autonomy/worlds/disaster')
OUT_SPEC = os.path.join(WS, 'results/phase7/worlds')
URI = 'model://dz_assets/meshes/'

_spec = importlib.util.spec_from_file_location(
    'make_village', os.path.join(WS, 'scripts/village/make_village.py'))
mv = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(mv)     # reuse manikin, distractor, sdf header

STOREY = 3.0           # floors at 3, 6, 9, 12 m; 6.0 is the lidar slice
CELL_W, CELL_H = 1.0, 0.5
BAND_ROWS = (11, 12)   # rows covering z 5.5..6.5, the mapped slice
TILE_M = 3.0           # texture repeat length

# --------------------------------------------------------------------------
# textures
# --------------------------------------------------------------------------


def _noise(rng, size, scale):
    small = rng.random((size // scale + 2, size // scale + 2)).astype(
        np.float32)
    return cv2.resize(small, (size, size), interpolation=cv2.INTER_CUBIC)


def _cracks(img, rng, count, dark=0.35):
    h = img.shape[0]
    for _ in range(count):
        x, y = rng.integers(0, h, 2)
        ang = rng.random() * 6.28
        pts = [(int(x), int(y))]
        for _ in range(rng.integers(6, 18)):
            ang += rng.normal(0, 0.5)
            step = rng.integers(8, 28)
            x = int(x + step * math.cos(ang))
            y = int(y + step * math.sin(ang))
            pts.append((x, y))
        cv2.polylines(img, [np.array(pts, np.int32)], False,
                      (dark, dark, dark), 1 + int(rng.random() * 1.5),
                      cv2.LINE_AA)


def make_texture(name, base_rgb, seed, kind):
    """512 x 512 muted material, saved as PNG. base_rgb in 0..1 (R,G,B)."""
    rng = np.random.default_rng(seed)
    n = 512
    lum = (0.55 * _noise(rng, n, 64) + 0.30 * _noise(rng, n, 16)
           + 0.15 * _noise(rng, n, 4))
    lum = 0.80 + 0.35 * (lum - 0.5)
    img = np.dstack([lum * c for c in base_rgb]).astype(np.float32)
    if kind in ('concrete', 'dark_concrete'):
        streak = _noise(rng, n, 128)
        streak = cv2.resize(streak[:8, :], (n, n))      # vertical rain streaks
        img *= (0.80 + 0.35 * streak)[:, :, None]
        for _ in range(7):                               # stains
            x, y, r = rng.integers(0, n, 2).tolist() + [
                int(rng.integers(20, 70))]
            blot = np.zeros((n, n), np.float32)
            cv2.circle(blot, (x, y), r, 1.0, -1)
            blot = cv2.GaussianBlur(blot, (0, 0), r / 1.5)
            img *= (1.0 - 0.12 * blot)[:, :, None]
        _cracks(img, rng, 26)
    elif kind == 'masonry':
        bh, bw = 24, 64
        for row in range(n // bh):
            off = (row % 2) * bw // 2
            for col in range(-1, n // bw + 1):
                x0 = col * bw + off
                tone = 0.8 + 0.35 * rng.random()
                cv2.rectangle(img, (x0 + 2, row * bh + 2),
                              (x0 + bw - 2, row * bh + bh - 2),
                              tuple(float(c * tone) for c in base_rgb), -1)
                cv2.rectangle(img, (x0, row * bh), (x0 + bw, row * bh + bh),
                              (0.25, 0.25, 0.24), 1)
        img *= (0.7 + 0.5 * _noise(rng, n, 8))[:, :, None]
        _cracks(img, rng, 10)
    elif kind == 'asphalt':
        img *= (0.8 + 0.4 * _noise(rng, n, 2))[:, :, None]
        for _ in range(7):                               # repair patches
            x, y = rng.integers(0, n - 80, 2)
            w, h = rng.integers(40, 120, 2)
            tone = 0.8 + 0.5 * rng.random()
            img[y:y + h, x:x + w] *= tone
        _cracks(img, rng, 28, dark=0.12)
    elif kind == 'rubble':
        img *= (0.75 + 0.5 * _noise(rng, n, 3))[:, :, None]
        for _ in range(180):                             # chips
            x, y = rng.integers(0, n, 2)
            r = int(rng.integers(2, 9))
            tone = 0.55 + 0.9 * rng.random()
            col = tuple(float(c * tone) for c in base_rgb)
            cv2.circle(img, (int(x), int(y)), r, col, -1)
    elif kind == 'rust':
        img *= (0.7 + 0.6 * _noise(rng, n, 6))[:, :, None]
    elif kind == 'dirt':
        img *= (0.75 + 0.5 * _noise(rng, n, 4))[:, :, None]
        for _ in range(60):
            x, y = rng.integers(0, n, 2)
            cv2.circle(img, (int(x), int(y)), int(rng.integers(3, 14)),
                       tuple(float(c * 0.7) for c in base_rgb), -1)
    img = np.clip(img, 0, 1)
    out = (img[:, :, ::-1] * 255).astype(np.uint8)
    os.makedirs(MESH_DIR, exist_ok=True)
    cv2.imwrite(os.path.join(MESH_DIR, f'tex_{name}.png'), out)


# muted, low-saturation palette (R, G, B); browns have saturation < 0.5 so
# they stay clear of the detector's clothing mask (H 5-22, S >= 170/255)
MATERIALS = {
    'concrete': ((0.62, 0.61, 0.58), 'concrete'),
    'dark_concrete': ((0.40, 0.40, 0.40), 'dark_concrete'),
    'beige': ((0.66, 0.60, 0.50), 'concrete'),
    'masonry': ((0.52, 0.45, 0.40), 'masonry'),
    'rubble': ((0.57, 0.54, 0.49), 'rubble'),
    'rust': ((0.42, 0.33, 0.27), 'rust'),
    'asphalt': ((0.36, 0.36, 0.37), 'asphalt'),
    'dirt': ((0.52, 0.50, 0.47), 'dirt'),
}

# --------------------------------------------------------------------------
# meshes
# --------------------------------------------------------------------------


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


class Mesh:
    def __init__(self):
        self.tris = []

    def quad(self, a, b, c, d, inside):
        """Quad a-b-c-d, wound to face away from the point 'inside'."""
        a, b, c, d = (np.asarray(p, float) for p in (a, b, c, d))
        n = np.cross(b - a, c - a)
        if np.dot(n, (a + b + c + d) / 4 - np.asarray(inside)) < 0:
            a, b, c, d = d, c, b, a
        self.tris.append((a, b, c))
        self.tris.append((a, c, d))

    def box(self, centre, size, rpy=(0, 0, 0), jitter=0.0, rng=None):
        centre = np.asarray(centre, float)
        half = np.asarray(size, float) / 2
        corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1)
                            for sz in (-1, 1)], float) * half
        if jitter and rng is not None:
            corners = corners + rng.normal(0, jitter, corners.shape)
        corners = corners @ rpy_matrix(*rpy).T + centre

        def c(ix, iy, iz):
            return corners[ix * 4 + iy * 2 + iz]
        faces = [((0, 0, 0), (0, 0, 1), (0, 1, 1), (0, 1, 0)),
                 ((1, 0, 0), (1, 0, 1), (1, 1, 1), (1, 1, 0)),
                 ((0, 0, 0), (0, 0, 1), (1, 0, 1), (1, 0, 0)),
                 ((0, 1, 0), (0, 1, 1), (1, 1, 1), (1, 1, 0)),
                 ((0, 0, 0), (0, 1, 0), (1, 1, 0), (1, 0, 0)),
                 ((0, 0, 1), (0, 1, 1), (1, 1, 1), (1, 0, 1))]
        for f in faces:
            self.quad(*(c(*i) for i in f), inside=corners.mean(axis=0))

    def hull(self, points):
        points = np.asarray(points, float)
        hull = ConvexHull(points)
        centre = points.mean(axis=0)
        for s in hull.simplices:
            a, b, c = points[s]
            if np.dot(np.cross(b - a, c - a), a - centre) < 0:
                b, c = c, b
            self.tris.append((a, b, c))

    def add_transformed(self, other, pose):
        """Append other's triangles moved by pose (x, y, z, roll, pitch, yaw)."""
        rot = rpy_matrix(*pose[3:6])
        shift = np.asarray(pose[:3], float)
        for a, b, c in other.tris:
            self.tris.append((rot @ a + shift, rot @ b + shift,
                              rot @ c + shift))

    def write(self, name, material):
        base, _kind = MATERIALS[material]
        path = os.path.join(MESH_DIR, f'{name}.obj')
        lines = [f'mtllib {name}.mtl', f'o {name}', 'usemtl mat']
        vs, vts, vns, fs = [], [], [], []
        for a, b, c in self.tris:
            n = np.cross(b - a, c - a)
            ln = np.linalg.norm(n)
            if ln < 1e-12:
                continue
            n = n / ln
            axis = int(np.argmax(np.abs(n)))
            u_i, v_i = [(1, 2), (0, 2), (0, 1)][axis]
            idx = []
            for p in (a, b, c):
                vs.append(p)
                vts.append((p[u_i] / TILE_M, p[v_i] / TILE_M))
                vns.append(n)
                idx.append(len(vs))
            fs.append(idx)
        lines += [f'v {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}' for p in vs]
        lines += [f'vt {u:.4f} {v:.4f}' for u, v in vts]
        lines += [f'vn {n[0]:.4f} {n[1]:.4f} {n[2]:.4f}' for n in vns]
        lines += [f'f {i}/{i}/{i} {j}/{j}/{j} {k}/{k}/{k}'
                  for i, j, k in fs]
        with open(path, 'w') as handle:
            handle.write('\n'.join(lines) + '\n')
        with open(os.path.join(MESH_DIR, f'{name}.mtl'), 'w') as handle:
            handle.write(f'newmtl mat\nKa 0.6 0.6 0.6\nKd 1 1 1\n'
                         f'Ks 0.02 0.02 0.02\nmap_Kd tex_{material}.png\n')
        return len(self.tris)


# --------------------------------------------------------------------------
# world assembly
# --------------------------------------------------------------------------


class Zone:
    def __init__(self):
        self.models = []
        self.structures = []     # truth footprints (world frame)
        self.tri_count = 0
        self.names = set()
        self.batches = {}        # material -> Mesh in the world frame
        self.shapes = []         # collision shapes of batched elements
        self.export = []         # ALL collision shapes (for the MuJoCo scene)

    def footprint(self, name, cx, cy, w, d, yaw, z0, z1, kind='structure'):
        self.structures.append({
            'name': name, 'kind': kind, 'centre': [round(cx, 3), round(cy, 3)],
            'size': [round(w, 3), round(d, 3)], 'yaw': round(yaw, 4),
            'z': [round(z0, 2), round(z1, 2)]})

    def element(self, name, pose, mesh, material, collisions, shadow=True,
                batch=True):
        """One static element: visual mesh + collision shapes.

        batch=True merges the visual into one world-frame mesh per material
        (a few draw calls instead of hundreds: measured, the separate meshes
        cost about 40 % of the real-time factor on the Intel GPU). The
        element keeps its own collision-only model. batch=False keeps a
        separate visual (used for scenario-specific elements).

        collisions: list of ('box', (x,y,z), (sx,sy,sz)) or
                    ('cyl', (x,y,z), (radius, length)) in the model frame.
        """
        assert name not in self.names, name
        self.names.add(name)
        rot_e = rpy_matrix(*pose[3:6])
        for kind, pos, dims in collisions:
            wp_e = rot_e @ np.asarray(pos, float) + np.asarray(pose[:3])
            self.export.append({
                'kind': kind, 'pos': [round(float(v), 3) for v in wp_e],
                'rpy': [round(float(v), 4) for v in pose[3:6]],
                'dims': [round(float(v), 3) for v in dims],
                'material': material})
        parts = []
        if batch:
            self.batches.setdefault(material, Mesh()).add_transformed(
                mesh, pose)
            self.tri_count += len(mesh.tris)
        else:
            self.tri_count += mesh.write(name, material)
            parts.append(
                f'<visual name="v"><cast_shadows>{str(shadow).lower()}'
                f'</cast_shadows><geometry><mesh><uri>{URI}{name}.obj'
                '</uri></mesh></geometry></visual>')
        if batch:
            # collisions of batched elements go into ONE static body: each
            # separate model is a separate physics body (measured: 250
            # bodies cost about 30 % of the real-time factor)
            rot = rpy_matrix(*pose[3:6])
            for kind, pos, dims in collisions:
                wp = rot @ np.asarray(pos, float) + np.asarray(pose[:3])
                self.shapes.append((kind, wp, pose[3:6], dims))
            collisions = []
        for i, (kind, pos, dims) in enumerate(collisions):
            if kind == 'box':
                geom = f'<box><size>{dims[0]:.3f} {dims[1]:.3f} {dims[2]:.3f}'\
                       '</size></box>'
            else:
                geom = (f'<cylinder><radius>{dims[0]:.3f}</radius>'
                        f'<length>{dims[1]:.3f}</length></cylinder>')
            parts.append(
                f'<collision name="c{i}"><pose>{pos[0]:.3f} {pos[1]:.3f} '
                f'{pos[2]:.3f} 0 0 0</pose><geometry>{geom}</geometry>'
                '</collision>')
        if parts:
            pose_s = ' '.join(f'{v:.4f}' for v in pose)
            self.models.append(
                f'<model name="{name}"><static>true</static><pose>{pose_s}'
                f'</pose><link name="link">{"".join(parts)}</link></model>')

    def write_batches(self):
        """One visual-only model per material; returns their SDF strings."""
        out = []
        for material, mesh in sorted(self.batches.items()):
            name = f'dz_common_{material}'
            mesh.write(name, material)
            out.append(
                f'<model name="{name}"><static>true</static><link name="link">'
                '<visual name="v"><cast_shadows>true</cast_shadows><geometry>'
                f'<mesh><uri>{URI}{name}.obj</uri></mesh></geometry></visual>'
                '</link></model>')
        cols = []
        for i, (kind, wp, rpy, dims) in enumerate(self.shapes):
            if kind == 'box':
                geom = (f'<box><size>{dims[0]:.3f} {dims[1]:.3f} '
                        f'{dims[2]:.3f}</size></box>')
            else:
                geom = (f'<cylinder><radius>{dims[0]:.3f}</radius>'
                        f'<length>{dims[1]:.3f}</length></cylinder>')
            cols.append(
                f'<collision name="c{i}"><pose>{wp[0]:.3f} {wp[1]:.3f} '
                f'{wp[2]:.3f} {rpy[0]:.4f} {rpy[1]:.4f} {rpy[2]:.4f}</pose>'
                f'<geometry>{geom}</geometry></collision>')
        out.append('<model name="dz_collision"><static>true</static>'
                   f'<link name="link">{"".join(cols)}</link></model>')
        return out


def world_xy(pose, local):
    """Local (x, y) -> world (x, y) for a model pose (x, y, z, r, p, yaw)."""
    c, s = math.cos(pose[5]), math.sin(pose[5])
    return (pose[0] + c * local[0] - s * local[1],
            pose[1] + s * local[0] + c * local[1])


# -- facades ---------------------------------------------------------------

def facade(zone, name, cx, cy, yaw, width, height, thick=0.35, seed=1,
           windows=True, top_cuts=(), holes=(), material='concrete',
           slice_truth=True):
    """A wall built from a 1 m x 0.5 m cell grid: window openings (kept out of
    the 5.5-7.0 m spandrel band so the lidar slice meets solid wall),
    a jagged broken top edge and explicit holes.

    top_cuts: (i0, i1, keep_height_m) caps the wall over columns i0..i1.
    holes: (i0, i1, z0_m, z1_m) removed rectangles (may open the slice band).
    """
    rng = np.random.default_rng(seed)
    nx, nz = int(round(width / CELL_W)), int(round(height / CELL_H))
    occ = np.ones((nx, nz), bool)
    if windows:
        for k in range(int(height // STOREY)):
            z0 = k * STOREY
            for i in range(nx):
                if i % 3 != 0 and 1 <= i < nx - 1:
                    for j in range(int((z0 + 1.0) / CELL_H),
                                   int((z0 + 2.5) / CELL_H)):
                        if j < nz:
                            occ[i, j] = False
    top = np.full(nx, nz)
    for i0, i1, keep in top_cuts:
        top[max(i0, 0):min(i1 + 1, nx)] = int(keep / CELL_H)
    walk = 0
    for i in range(nx):                       # jagged edge where cut
        walk = int(np.clip(walk + rng.integers(-1, 2), -3, 3))
        if top[i] < nz:
            top[i] = int(np.clip(top[i] + walk, 2, nz))
    for i in range(nx):
        occ[i, top[i]:] = False
    for i0, i1, z0, z1 in holes:
        occ[max(i0, 0):min(i1 + 1, nx), int(z0 / CELL_H):int(z1 / CELL_H)] = \
            False
    mesh = Mesh()
    hx, ht = width / 2, thick / 2
    # front and back faces: greedy merge of occupied cells into rectangles
    # (same planes, same texture coordinates; far fewer triangles)
    rem = occ.copy()
    for j in range(nz):
        for i in range(nx):
            if not rem[i, j]:
                continue
            w = 1
            while i + w < nx and rem[i + w, j]:
                w += 1
            h = 1
            while j + h < nz and rem[i:i + w, j + h].all():
                h += 1
            rem[i:i + w, j:j + h] = False
            x0, x1 = i * CELL_W - hx, (i + w) * CELL_W - hx
            z0, z1 = j * CELL_H, (j + h) * CELL_H
            inside = ((x0 + x1) / 2, 0, (z0 + z1) / 2)
            mesh.quad((x0, -ht, z0), (x1, -ht, z0), (x1, -ht, z1),
                      (x0, -ht, z1), inside)
            mesh.quad((x0, ht, z0), (x1, ht, z0), (x1, ht, z1),
                      (x0, ht, z1), inside)
    for i in range(nx):
        for j in range(nz):
            if not occ[i, j]:
                continue
            x0, x1 = i * CELL_W - hx, (i + 1) * CELL_W - hx
            z0, z1 = j * CELL_H, (j + 1) * CELL_H
            inside = ((x0 + x1) / 2, 0, (z0 + z1) / 2)
            if i == 0 or not occ[i - 1, j]:
                mesh.quad((x0, -ht, z0), (x0, ht, z0), (x0, ht, z1),
                          (x0, -ht, z1), inside)
            if i == nx - 1 or not occ[i + 1, j]:
                mesh.quad((x1, -ht, z0), (x1, ht, z0), (x1, ht, z1),
                          (x1, -ht, z1), inside)
            if j == nz - 1 or not occ[i, j + 1]:
                mesh.quad((x0, -ht, z1), (x1, -ht, z1), (x1, ht, z1),
                          (x0, ht, z1), inside)
            if j > 0 and not occ[i, j - 1]:
                mesh.quad((x0, -ht, z0), (x1, -ht, z0), (x1, ht, z0),
                          (x0, ht, z0), inside)
    # collision: vertical runs per column, equal neighbouring columns merged
    def runs(i):
        out, j = [], 0
        while j < nz:
            if occ[i, j]:
                k = j
                while k < nz and occ[i, k]:
                    k += 1
                out.append((j, k))
                j = k
            else:
                j += 1
        return tuple(out)
    cols, i = [], 0
    while i < nx:
        r = runs(i)
        k = i
        while k + 1 < nx and runs(k + 1) == r:
            k += 1
        cols.append((i, k, r))
        i = k + 1
    pose = (cx, cy, 0, 0, 0, yaw)
    collisions = []
    for i0, i1, r in cols:
        for j0, j1 in r:
            x0, x1 = i0 * CELL_W - hx, (i1 + 1) * CELL_W - hx
            z0, z1 = j0 * CELL_H, j1 * CELL_H
            collisions.append(('box', ((x0 + x1) / 2, 0, (z0 + z1) / 2),
                               ((x1 - x0) - 0.06, thick - 0.06, z1 - z0)))
            if slice_truth and j0 <= BAND_ROWS[0] and j1 > BAND_ROWS[1]:
                wx, wy = world_xy(pose, ((x0 + x1) / 2, 0))
                zone.footprint(f'{name}_{i0}', wx, wy, x1 - x0, thick, yaw,
                               z0, z1, kind='wall')
    zone.element(name, pose, mesh, material, collisions)


def slab(zone, name, x, y, z, sx, sy, st, rpy=(0, 0, 0), seed=1, jitter=0.04,
         material='concrete', truth=True, batch=True):
    """Jittered concrete slab (floor, roof or fallen panel)."""
    rng = np.random.default_rng(seed)
    mesh = Mesh()
    mesh.box((0, 0, 0), (sx, sy, st), jitter=jitter, rng=rng)
    pose = (x, y, z, rpy[0], rpy[1], rpy[2])
    # collision: same pose and orientation, inset
    zone.element(name, pose, mesh, material,
                 [('box', (0, 0, 0), (max(sx - 0.15, 0.05), max(sy - 0.15, 0.05),
                          max(st - 0.1, 0.05)))],
                 batch=batch)
    if truth:
        rot = rpy_matrix(*rpy)
        corners = np.array([[a, b, c] for a in (-sx / 2, sx / 2)
                            for b in (-sy / 2, sy / 2)
                            for c in (-st / 2, st / 2)]) @ rot.T + [x, y, z]
        lo, hi = corners.min(axis=0), corners.max(axis=0)
        zone.footprint(name, (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2,
                       hi[0] - lo[0], hi[1] - lo[1], 0.0, lo[2], hi[2])


def column(zone, name, x, y, height, size=0.55, seed=1, tilt=0.0):
    rng = np.random.default_rng(seed)
    mesh = Mesh()
    mesh.box((0, 0, height / 2), (size, size, height), rpy=(tilt, 0, 0),
             jitter=0.03, rng=rng)
    zone.element(name, (x, y, 0, 0, 0, rng.random() * 3), mesh,
                 'dark_concrete',
                 [('box', (0, 0, height / 2), (size - 0.1, size - 0.1,
                                               height - 0.1))])
    zone.footprint(name, x, y, size, size, 0.0, 0.0, height, kind='pole')


def rubble_pile(zone, name, cx, cy, radius, height, count, seed,
                exclude=(), material='rubble', beams=4):
    """Mound of random convex chunks, plus beams sticking out of it.

    Surface height h(r) = height * (1 - (r/radius)^1.6); collision is four
    stacked cylinders that stay below that surface."""
    rng = np.random.default_rng(seed)
    mesh = Mesh()

    def surface(r):
        return height * max(1.0 - (r / radius) ** 1.6, 0.0)
    placed = 0
    tries = 0
    while placed < count and tries < count * 6:
        tries += 1
        r = radius * math.sqrt(rng.random())
        ang = rng.random() * 6.283
        x, y = r * math.cos(ang), r * math.sin(ang)
        if any(math.hypot(cx + x - ex, cy + y - ey) < er
               for ex, ey, er in exclude):
            continue
        size = float(np.clip(rng.lognormal(-0.35, 0.5), 0.25,
                             0.30 * height + 0.5))
        z = surface(r) * (0.55 + 0.45 * rng.random()) + size * 0.1
        pts = rng.normal(0, 1, (10, 3))
        pts = pts / np.linalg.norm(pts, axis=1)[:, None]
        pts = pts * (size / 2) * np.array([1.0, 0.7 + 0.5 * rng.random(),
                                           0.4 + 0.5 * rng.random()])
        pts = pts * (0.75 + 0.5 * rng.random((10, 1)))
        pts = pts @ rpy_matrix(*(rng.random(3) * 6.283)).T + [x, y, z]
        pts[:, 2] = np.maximum(pts[:, 2], 0.0)
        mesh.hull(pts)
        placed += 1
    beam_mesh = Mesh()
    for _ in range(beams):
        r = radius * (0.2 + 0.6 * rng.random())
        ang = rng.random() * 6.283
        x, y = r * math.cos(ang), r * math.sin(ang)
        if any(math.hypot(cx + x - ex, cy + y - ey) < er
               for ex, ey, er in exclude):
            continue
        length = 2.5 + 3.0 * rng.random()
        beam_mesh.box((x, y, surface(r) + 0.4),
                      (length, 0.28, 0.30),
                      rpy=(0, (rng.random() - 0.5) * 0.9,
                           rng.random() * 3.14), jitter=0.02, rng=rng)
    zone.element(name, (cx, cy, 0, 0, 0, 0), mesh, material,
                 [('cyl', (0, 0, surface(radius * f) * 0.8 / 2),
                   (radius * f, max(surface(radius * f) * 0.8, 0.2)))
                  for f in (0.85, 0.6, 0.4, 0.2)])
    if beams and beam_mesh.tris:
        zone.element(name + '_beams', (cx, cy, 0, 0, 0, 0), beam_mesh,
                     'rust', [])
    zone.footprint(name, cx, cy, radius * 1.4, radius * 1.4, 0.0, 0.0,
                   height, kind='rubble')


def road_patch(zone, name, x0, y0, x1, y1):
    """Visual-only asphalt rectangle (2 cm above the ground)."""
    mesh = Mesh()
    mesh.quad((x0, y0, 0.02), (x1, y0, 0.02), (x1, y1, 0.02), (x0, y1, 0.02),
              inside=((x0 + x1) / 2, (y0 + y1) / 2, -1))
    zone.element(name, (0, 0, 0, 0, 0, 0), mesh, 'asphalt', [], shadow=False)


# -- buildings -------------------------------------------------------------


def floors_partial(zone, name, cx, cy, w, d, levels, seed, nx=3, ny=3):
    """Floor slabs as a grid of tiles: some missing, the others drooping.

    levels: list of (z, keep_fraction). Tiles are jittered slabs; no truth
    footprint (inside the building, below the facade tops)."""
    rng = np.random.default_rng(seed)
    tw, td = (w - 0.7) / nx, (d - 0.7) / ny
    for li, (z, keep) in enumerate(levels):
        for ix in range(nx):
            for iy in range(ny):
                if rng.random() > keep:
                    continue
                x = cx - (w - 0.7) / 2 + (ix + 0.5) * tw
                y = cy - (d - 0.7) / 2 + (iy + 0.5) * td
                droop = 0.04 + 0.14 * (1.0 - keep)
                slab(zone, f'{name}_L{li}_{ix}{iy}', x, y,
                     z + rng.normal(0, 0.12) - 0.15 * (1.0 - keep) * 3,
                     tw - 0.1, td - 0.1, 0.30,
                     (rng.normal(0, droop), rng.normal(0, droop), 0.0),
                     seed=seed + 10 * li + 3 * ix + iy, truth=False)

def building_shell(zone, name, cx, cy, w, d, storeys, seed, damage,
                   floors=True, roof=True):
    """Four facades around a rectangle plus floor slabs.

    damage: dict side -> dict(top_cuts=[], holes=[], absent=False) with sides
    'S', 'N', 'W', 'E'. The facade width runs along x for S/N, along y for
    W/E (yaw 90 deg)."""
    h = storeys * STOREY
    sides = {'S': (cx, cy - d / 2, 0.0, w), 'N': (cx, cy + d / 2, 0.0, w),
             'W': (cx - w / 2, cy, math.pi / 2, d),
             'E': (cx + w / 2, cy, math.pi / 2, d)}
    for k, (sx, sy, yaw, width) in sides.items():
        dm = damage.get(k, {})
        if dm.get('absent'):
            continue
        facade(zone, f'{name}_{k}', sx, sy, yaw, width, h,
               seed=seed + ord(k), top_cuts=dm.get('top_cuts', ()),
               holes=dm.get('holes', ()),
               material=dm.get('material', 'concrete'))
    if floors:
        floors_partial(zone, f'{name}_fl', cx, cy, w, d,
                       [(s * STOREY, max(0.85 - 0.15 * s, 0.3))
                        for s in range(1, storeys)], seed + 100)
    if roof:
        floors_partial(zone, f'{name}_rf', cx, cy, w, d,
                       [(h + 0.15, 0.55)], seed + 200)


# --------------------------------------------------------------------------
# the zone
# --------------------------------------------------------------------------

# survivor spots: (x, y, ledge top height, yaw). The ledges (broken floor
# slabs on rubble) exist in every scenario; manikins only where listed.
SPOTS = {
    's_block': (17.0, 55.0, 2.0, 0.4),       # B1 collapse, east side
    's_courtyard': (57.5, 16.0, 0.8, 1.2),   # U-shaped courtyard
    's_front': (41.5, 43.0, 1.4, 2.0),       # gap between street and B2
}
DISTRACTORS = [('barrel_avenue', 'barrel', 23.0, 29.0),
               ('cone_street', 'cone', 33.0, 40.0),
               ('tarp_alley', 'tarp', 24.0, 62.0),
               ('crate_ball_gap', 'crate_and_ball', 41.0, 36.0),
               ('barrel_courtyard', 'barrel', 66.0, 17.0)]

SCENARIOS = {
    'urban_A': ('reconnaissance, no survivors, no look-alikes', [], [], []),
    'urban_B': ('one survivor on the B1 collapse', ['s_block'], [], []),
    'urban_B2': ('one survivor in the courtyard (not the place of B)',
                 ['s_courtyard'], [], []),
    'urban_C': ('three survivors in three places, five look-alikes',
                ['s_block', 's_courtyard', 's_front'], DISTRACTORS, []),
    'urban_D': ('two survivors partly under leaning slabs, one look-alike',
                ['s_block', 's_courtyard'], DISTRACTORS[:1],
                ['s_block', 's_courtyard']),
    'urban_E': ('one survivor inside the collapsed block, under a leaning slab',
                ['s_block'], [], ['s_block']),
    'urban_F': ('no survivors, five look-alikes (false-positive control)',
                [], DISTRACTORS, []),
}


def build_zone(zone):
    # ground and streets (visual only; the plane in the header collides)
    ground = Mesh()
    ground.quad((-150, -130, 0), (150, -130, 0), (150, 170, 0),
                (-150, 170, 0), inside=(0, 0, -1))
    zone.element('ground_dirt', (0, 0, 0, 0, 0, 0), ground, 'dirt', [],
                 shadow=False)
    road_patch(zone, 'road_avenue', -40, 22, 95, 32)
    road_patch(zone, 'road_cross', 28, -14, 38, 100)
    pad = Mesh()
    pad.box((0, 0, 0.02), (14, 14, 0.04))
    zone.element('base_lot', (0, 0, 0, 0, 0, 0), pad, 'concrete', [])
    mark = Mesh()
    for c, s in (((0, 0, 0.05), (3.2, 0.4, 0.02)),
                 ((-1.4, 0, 0.05), (0.4, 3.2, 0.02)),
                 ((1.4, 0, 0.05), (0.4, 3.2, 0.02))):
        mark.box(c, s)
    zone.element('base_marking', (0, 0, 0, 0, 0, 0), mark, 'beige', [],
                 shadow=False)

    # -- B1: collapsed apartment block (west part standing, east pancaked)
    facade(zone, 'b1_W', -10, 58, math.pi / 2, 20, 15.0, seed=11,
           top_cuts=[(0, 4, 11.0), (5, 9, 15.0), (10, 13, 12.0),
                     (14, 19, 8.5)],
           holes=[(6, 12, 3.0, 9.5)])
    facade(zone, 'b1_S', 5, 48, 0.0, 30, 15.0, seed=12,
           top_cuts=[(10, 16, 9.0), (17, 29, 4.0)],
           holes=[(20, 23, 4.5, 8.0)])
    facade(zone, 'b1_N', 5, 68, 0.0, 30, 15.0, seed=13,
           top_cuts=[(0, 3, 12.0), (9, 18, 11.0), (19, 29, 3.0)],
           holes=[(2, 8, 4.5, 11.0)])
    floors_partial(zone, 'b1_fl', -3.0, 58, 12.6, 19.2,
                   [(3.0, 0.8), (6.0, 0.65), (9.0, 0.5), (12.0, 0.35)], 20,
                   nx=3, ny=4)
    for i, (x, y, z, sx, sy, rpy) in enumerate([
            (8.5, 55.0, 1.3, 7.5, 9.0, (0.10, -0.18, 0.1)),
            (8.0, 62.5, 3.0, 8.0, 7.0, (-0.12, 0.15, 0.3)),
            (7.0, 54.0, 4.6, 6.0, 7.5, (0.20, 0.10, -0.2)),
            (9.5, 63.5, 5.9, 7.0, 6.0, (0.30, -0.10, 0.2))]):
        slab(zone, f'b1_pancake{i}', x, y, z, sx, sy, 0.35, rpy, seed=30 + i)
    for i, (x, y, h) in enumerate([(4.5, 50.0, 9.0), (4.5, 66.5, 7.0),
                                   (13.0, 50.2, 5.0), (12.5, 66.5, 8.0)]):
        column(zone, f'b1_col{i}', x, y, h, seed=40 + i)
    rubble_pile(zone, 'b1_pile', 16.5, 58.0, 9.0, 4.2, 260, 51,
                exclude=[(17.0, 55.0, 2.4)])
    rubble_pile(zone, 'b1_alley_pile', 24.0, 51.0, 4.6, 2.6, 80, 52)
    rubble_pile(zone, 'b1_foot_w', -12.5, 52.0, 4.5, 2.4, 70, 53)
    rubble_pile(zone, 'b1_foot_n', 6.0, 70.5, 5.0, 3.0, 80, 54)

    # -- B2: commercial block, south facade fallen onto the street side
    facade(zone, 'b2_W', 44, 55, math.pi / 2, 18, 12.0, seed=61,
           top_cuts=[(0, 7, 6.5)])
    facade(zone, 'b2_N', 55, 64, 0.0, 22, 12.0, seed=62,
           top_cuts=[(0, 5, 8.0), (14, 21, 5.5)])
    facade(zone, 'b2_E', 66, 55, math.pi / 2, 18, 12.0, seed=63,
           top_cuts=[(0, 4, 9.0), (13, 17, 7.0)], holes=[(6, 12, 4.5, 8.5)])
    floors_partial(zone, 'b2_fl', 55, 55, 21.2, 17.2,
                   [(3.0, 0.7), (6.0, 0.55), (9.0, 0.35)], 70, nx=4, ny=3)
    slab(zone, 'b2_fallen_facade', 55.0, 41.6, 2.7, 20.0, 10.0, 0.4,
         (0.5, 0.0, 0.0), seed=75)
    rubble_pile(zone, 'b2_pile', 48.0, 40.0, 5.5, 3.2, 100, 76,
                exclude=[(41.5, 43.0, 2.4)])

    # -- B3: U-shaped complex around a courtyard that opens to the avenue
    facade(zone, 'b3_back_S', 60, -4, 0.0, 32, 9.0, seed=81)
    facade(zone, 'b3_back_N', 60, 6, 0.0, 16, 9.0, seed=82,
           top_cuts=[(4, 11, 6.0)])
    facade(zone, 'b3_back_W', 44, 1, math.pi / 2, 10, 9.0, seed=83)
    facade(zone, 'b3_back_E', 76, 1, math.pi / 2, 10, 9.0, seed=84)
    facade(zone, 'b3_wA_W', 44, 13, math.pi / 2, 14, 9.0, seed=85)
    facade(zone, 'b3_wA_E', 52, 13, math.pi / 2, 14, 9.0, seed=86)
    facade(zone, 'b3_wA_N', 48, 20, 0.0, 8, 9.0, seed=87,
           top_cuts=[(0, 7, 5.0)])
    facade(zone, 'b3_eA_W', 68, 13, math.pi / 2, 14, 12.0, seed=88)
    facade(zone, 'b3_eA_E', 76, 13, math.pi / 2, 14, 12.0, seed=89,
           holes=[(4, 9, 4.5, 8.5)])
    facade(zone, 'b3_eA_N', 72, 20, 0.0, 8, 12.0, seed=90,
           top_cuts=[(0, 7, 4.5)])
    floors_partial(zone, 'b3_fb', 60, 1, 31.2, 9.2, [(3.0, 0.8), (6.0, 0.6)],
                   95, nx=6, ny=2)
    floors_partial(zone, 'b3_fw', 48, 13, 7.2, 13.2,
                   [(3.0, 0.75), (6.0, 0.5)], 97, nx=2, ny=3)
    floors_partial(zone, 'b3_fe', 72, 13, 7.2, 13.2,
                   [(3.0, 0.7), (6.0, 0.5), (9.0, 0.35)], 99, nx=2, ny=3)
    rubble_pile(zone, 'b3_court_pile', 60.5, 13.0, 4.2, 2.4, 70, 101,
                exclude=[(57.5, 16.0, 2.3)])
    rubble_pile(zone, 'b3_wA_pile', 48.0, 23.5, 4.0, 2.6, 50, 102)

    # -- B4: leaning tower; B5, B6: damaged low-rise shells; B7, B8 north
    facade(zone, 'b4_S', -20, 36, 0.0, 12, 15.0, seed=111,
           top_cuts=[(6, 11, 9.0)])
    facade(zone, 'b4_N', -20, 48, 0.0, 12, 15.0, seed=112,
           holes=[(3, 7, 5.0, 8.0)])
    facade(zone, 'b4_W', -26, 42, math.pi / 2, 12, 15.0, seed=113)
    facade(zone, 'b4_E', -14, 42, math.pi / 2, 12, 15.0, seed=114,
           top_cuts=[(4, 11, 7.0)])
    floors_partial(zone, 'b4_fl', -20, 42, 11.2, 11.2,
                   [(3.0, 0.8), (6.0, 0.6), (9.0, 0.45), (12.0, 0.3)], 115)
    slab(zone, 'b4_leaning_slab', -20.0, 33.6, 3.2, 8.0, 6.0, 0.35,
         (0.6, 0.0, 0.0), seed=118)
    rubble_pile(zone, 'b4_pile', -12.0, 35.0, 4.0, 2.5, 55, 119)
    building_shell(zone, 'b5', -26, 11, 16, 14, 3, 120,
                   {'W': {'top_cuts': [(0, 13, 5.0)]},
                    'E': {'top_cuts': [(5, 13, 7.5)]}}, roof=False)
    rubble_pile(zone, 'b5_pile', -15.0, 11.0, 4.0, 2.4, 50, 121)
    building_shell(zone, 'b6', 88, 50, 14, 16, 3, 130,
                   {'E': {'absent': True}}, roof=False)
    rubble_pile(zone, 'b6_pile', 95.0, 50.0, 5.0, 3.0, 70, 131)
    building_shell(zone, 'b7', 14, 90, 20, 14, 4, 140,
                   {'E': {'top_cuts': [(0, 13, 6.5)]},
                    'S': {'top_cuts': [(12, 19, 8.0)]}})
    building_shell(zone, 'b8', 56, 90, 22, 14, 4, 150,
                   {'W': {'top_cuts': [(3, 10, 7.0)]},
                    'N': {'top_cuts': [(14, 21, 5.5)]}})

    # -- the street north of the avenue is blocked at y ~ 76
    facade(zone, 'block_panel_a', 31.6, 75.0, 0.18, 6.4, 9.0, seed=161,
           top_cuts=[(0, 2, 6.5), (3, 5, 8.0)])
    facade(zone, 'block_panel_b', 36.0, 77.4, -0.12, 5.6, 8.5, seed=162,
           top_cuts=[(1, 3, 6.5)])
    rubble_pile(zone, 'block_pile', 33.0, 76.0, 6.0, 4.4, 150, 163)
    # broken walls along both sides of the street north of y = 66 turn it
    # into a corridor, so the block forces a detour through the alley west
    # of it (about 7 m wide between the apartment block and the wall)
    facade(zone, 'street_wall_W', 27.0, 74.0, math.pi / 2, 16, 8.0, seed=171,
           windows=False, top_cuts=[(5, 10, 6.8), (11, 15, 8.0)])
    facade(zone, 'street_wall_E', 39.0, 74.0, math.pi / 2, 16, 8.0, seed=172,
           windows=False, top_cuts=[(0, 4, 7.0), (8, 15, 8.0)])

    # wrecked cars on the avenue, small debris piles on the streets
    for i, (x, y, yaw, mat) in enumerate([
            (8.0, 27.0, 0.1, 'rust'), (18.0, 24.5, -0.2, 'dark_concrete'),
            (48.0, 29.0, 0.4, 'beige'), (63.0, 25.0, 0.0, 'rust'),
            (33.0, 6.0, 1.5, 'dark_concrete'), (33.0, 56.0, 1.6, 'beige')]):
        slab(zone, f'wreck{i}', x, y, 0.7, 4.2, 1.8, 1.4, (0, 0, yaw),
             seed=170 + i, jitter=0.08, material=mat, truth=False)
    rng = np.random.default_rng(7)
    for i in range(10):
        if i < 5:
            x, y = float(rng.uniform(-30, 90)), float(rng.uniform(23, 31))
        else:
            x, y = float(rng.uniform(29, 37)), float(rng.uniform(-10, 70))
        if (x - 8) ** 2 + (y - 27) ** 2 < 25 or x < 0 and y < 20:
            continue
        rubble_pile(zone, f'street_debris{i}', x, y, 1.6, 0.6, 14, 180 + i,
                    beams=0)

    # ledges for the survivors (the same in every scenario)
    for key, (x, y, top, yaw) in SPOTS.items():
        slab(zone, f'ledge_{key}', x, y, top - 0.2, 2.6, 2.6, 0.4, seed=190,
             material='rubble', truth=False)


def place_survivor(vw, name, key):
    x, y, top, yaw = SPOTS[key]
    mv.manikin(vw, name, x, y, yaw)
    old = f'<pose>{x:.4g} {y:.4g} 0 0 0 {yaw:.4g}</pose>'
    new = f'<pose>{x:.4g} {y:.4g} {top:.4g} 0 0 {yaw:.4g}</pose>'
    assert old in vw.models[-1]
    vw.models[-1] = vw.models[-1].replace(old, new)


def place_cover(zone, key):
    x, y, top, yaw = SPOTS[key]
    dx, dy = math.sin(-yaw) * 0.0, 0.25
    cxy = world_xy((x, y, 0, 0, 0, yaw), (0, dy))
    slab(zone, f'cover_{key}', cxy[0], cxy[1], top + 1.9, 1.7, 1.2, 0.12,
         (0, 0.35, yaw), seed=200, material='dark_concrete', truth=False,
         batch=False)
    sx, sy = world_xy((x, y, 0, 0, 0, yaw), (0.75, 0.25))
    slab(zone, f'cover_{key}_post', sx, sy, top + 0.8, 0.15, 1.0, 1.6,
         (0, 0, yaw), seed=201, material='dark_concrete', truth=False,
         batch=False)


def header(shadows):
    text = mv.sdf_header()
    for a, b in [
            ('GENERATED by scripts/village/make_village.py',
             'GENERATED by scripts/disaster/make_disaster_zone.py'),
            ('<ambient>0.45 0.45 0.45 1</ambient>',
             '<ambient>0.44 0.44 0.45 1</ambient>'),
            ('<ambient_light>0.45 0.45 0.45</ambient_light>',
             '<ambient_light>0.44 0.44 0.45</ambient_light>'),
            ('0.62 0.74 0.88 1', '0.52 0.53 0.55 1'),
            ('0.62 0.74 0.88', '0.52 0.53 0.55'),
            ('<diffuse>0.95 0.93 0.88 1</diffuse>',
             '<diffuse>0.58 0.58 0.57 1</diffuse>'),
            ('<camera_pose>-38 -52 62 0 0.62 0.86</camera_pose>',
             '<camera_pose>-52 -46 48 0 0.55 0.75</camera_pose>')]:
        assert a in text, a
        text = text.replace(a, b)
    if not shadows:
        text = text.replace('<shadows>true</shadows>',
                            '<shadows>false</shadows>').replace(
            '<cast_shadows>true</cast_shadows>',
            '<cast_shadows>false</cast_shadows>')
    return text


def generate(name, textures_done, shapes_only=False):
    description, survivors, distractors, covers = SCENARIOS[name]
    zone = Zone()
    build_zone(zone)
    vw = mv.World()
    spec = {'world': name, 'description': description,
            'sdf': f'src/uav_autonomy/worlds/disaster/{name}.sdf',
            'survivors': [], 'objects': []}
    for key in survivors:
        place_survivor(vw, f'survivor_{key[2:]}', key)
        x, y, top, yaw = SPOTS[key]
        spec['survivors'].append({'name': f'survivor_{key[2:]}',
                                  'position': [x, y], 'z': top, 'yaw': yaw})
    for dname, kind, x, y in DISTRACTORS:
        if any(d[0] == dname for d in distractors) or \
                dname in [d[0] for d in distractors]:
            mv.distractor(vw, spec, dname, kind, x, y)
    for key in covers:
        place_cover(zone, key)
    spec['structures'] = zone.structures
    spec['places'] = 'missions/places_disaster.json'
    os.makedirs(OUT_SPEC, exist_ok=True)
    shapes_dir = os.path.join(WS, 'results/disaster_gazebo/worlds_shapes')
    os.makedirs(shapes_dir, exist_ok=True)
    with open(os.path.join(shapes_dir, f'{name}.json'), 'w') as handle:
        json.dump({'scenario': name, 'description': description,
                   'frame': 'world ENU, metres; base pad at the origin',
                   'shapes': zone.export, 'survivors': spec['survivors'],
                   'objects': spec['objects']}, handle)
    if shapes_only:
        return zone                 # no SDF, mesh or truth file is touched
    batch_models = zone.write_batches()
    body = '\n'.join('    ' + m for m in batch_models + zone.models
                     + vw.models)
    tail = '\n  </world>\n</sdf>\n'
    for directory, shadows in ((OUT_SDF, True), (OUT_SDF + '_fast', False)):
        os.makedirs(directory, exist_ok=True)
        with open(os.path.join(directory, f'{name}.sdf'), 'w') as handle:
            handle.write(header(shadows) + body + tail)
    with open(os.path.join(OUT_SPEC, f'{name}.json'), 'w') as handle:
        json.dump(spec, handle, indent=1)
    return zone


def write_assets():
    os.makedirs(MESH_DIR, exist_ok=True)
    for f in os.listdir(MESH_DIR):          # stale generated meshes only
        if f.endswith(('.obj', '.mtl')):
            os.remove(os.path.join(MESH_DIR, f))
    for i, (mat, (rgb, kind)) in enumerate(MATERIALS.items()):
        make_texture(mat, rgb, 1000 + i, kind)
    with open(os.path.join(ASSETS, 'model.config'), 'w') as handle:
        handle.write('<?xml version="1.0"?>\n<model><name>dz_assets</name>'
                     '<version>1.0</version><sdf version="1.9">model.sdf'
                     '</sdf><description>Procedural meshes and textures of '
                     'the urban disaster zone (generated).</description>'
                     '</model>\n')
    with open(os.path.join(ASSETS, 'model.sdf'), 'w') as handle:
        handle.write('<?xml version="1.0"?>\n<sdf version="1.9"><model '
                     'name="dz_assets"><static>true</static><link '
                     'name="l"/></model></sdf>\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--scenario', action='append')
    parser.add_argument('--shapes-only', action='store_true',
                        help='only write results/disaster_gazebo/'
                             'worlds_shapes/<scenario>.json (collision shapes '
                             'for the MuJoCo scene); no world, mesh or truth '
                             'file is written')
    args = parser.parse_args()
    if not args.shapes_only:
        write_assets()
    for name in args.scenario or sorted(SCENARIOS):
        zone = generate(name, True, shapes_only=args.shapes_only)
        if args.shapes_only:
            print(f'{name}: {len(zone.export)} collision shapes exported')
            continue
        spec = json.load(open(os.path.join(OUT_SPEC, f'{name}.json')))
        print(f'{name}: {len(zone.models)} static models, '
              f'{zone.tri_count} triangles, {len(spec["survivors"])} '
              f'survivor(s), {len(spec["objects"])} look-alike(s) -> '
              f'worlds/disaster/{name}.sdf')


if __name__ == '__main__':
    main()
