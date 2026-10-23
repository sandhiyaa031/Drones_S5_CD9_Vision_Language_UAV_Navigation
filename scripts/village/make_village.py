#!/usr/bin/env python3
"""Generate the village-scale disaster-response worlds (static geometry).

One layout, several scenarios. Everything is built from a few reusable shape
helpers; nothing here is copied SDF. All structures are static bodies (they
cost almost nothing to simulate); debris is spawned at run time by the
existing scenario tool.

Frames: Gazebo world, ENU (x east, y north, z up), metres. The UAV base
(launch and landing pad) is at the world origin, so PX4 local NED
(north, east, down) = (world y, world x, -z).

Layout (about 125 m x 130 m):

    y (north)
    105 ┌──────────────────────────────────────────────────────────────┐
        │  houses        market square        houses        water tower│
     62 ├═══════════════════ road R3 (east-west) ══════════════════════┤
        │ houses ║   DISASTER ZONE: collapsed   ║ damaged ║   houses   │
        │        ║   multi-storey building      ║ house   ║            │
     20 ├════════╬══════════ road R2 ═══════════╬═════════╬════════════┤
        │ houses ║ houses        trees          ║  houses              │
      0 │  BASE ═╝ (access road)                                       │
    -25 └──────────────────────────────────────────────────────────────┘
       -30      12 (road R1)                    54 (road R4)          95  x (east)

Outputs
    src/uav_autonomy/worlds/village/<scenario>.sdf
    results/phase7/worlds/<scenario>.json   truth for OFFLINE scoring only:
                                            survivors, look-alike objects,
                                            and the footprint and height of
                                            every structure

Usage: make_village.py [--scenario NAME ...]   (default: all)
"""

import argparse
import json
import math
import os

WS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
OUT_SDF = os.path.join(WS, 'src/uav_autonomy/worlds/village')
OUT_SPEC = os.path.join(WS, 'results/phase7/worlds')
WORLD_NAME = 'collapsed_building_rescue'   # the bridge and tools use this name

# colours (r, g, b)
GRASS = (0.33, 0.45, 0.24)
ROAD = (0.16, 0.16, 0.17)
MARK = (0.85, 0.85, 0.80)
CONCRETE = (0.62, 0.62, 0.60)
DARK_CONCRETE = (0.42, 0.42, 0.41)
RUBBLE = (0.47, 0.44, 0.40)
BRICK = (0.55, 0.30, 0.22)
PLASTER = [(0.86, 0.82, 0.72), (0.80, 0.74, 0.62), (0.88, 0.86, 0.80),
           (0.74, 0.70, 0.64), (0.82, 0.76, 0.70)]
ROOF = [(0.48, 0.20, 0.16), (0.36, 0.22, 0.18), (0.30, 0.30, 0.32)]
WINDOW = (0.10, 0.14, 0.20)
TRUNK = (0.32, 0.22, 0.14)
LEAF = [(0.16, 0.38, 0.16), (0.20, 0.44, 0.18), (0.14, 0.33, 0.14)]
ORANGE = (1.0, 0.28, 0.04)
SKIN = (1.0, 0.78, 0.54)
BLUE = (0.025, 0.16, 0.72)
STEEL = (0.50, 0.52, 0.56)


class World:
    """Collects models; knows what it built (for the truth file)."""

    def __init__(self):
        self.models = []
        self.structures = []      # footprints for offline map scoring
        self.count = 0

    def _shape(self, kind, dims):
        if kind == 'box':
            return f'<box><size>{dims[0]} {dims[1]} {dims[2]}</size></box>'
        if kind == 'cylinder':
            return (f'<cylinder><radius>{dims[0]}</radius>'
                    f'<length>{dims[1]}</length></cylinder>')
        return f'<sphere><radius>{dims[0]}</radius></sphere>'

    def part(self, kind, pose, dims, colour, collide=True):
        """One visual (and collision) of the model being built."""
        self.count += 1
        r, g, b = colour
        pose_s = ' '.join(f'{v:.4g}' for v in pose)
        geom = self._shape(kind, dims)
        out = (f'<visual name="v{self.count}"><pose>{pose_s}</pose>'
               f'<geometry>{geom}</geometry><material>'
               f'<ambient>{r * 0.8:.3f} {g * 0.8:.3f} {b * 0.8:.3f} 1'
               f'</ambient><diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>'
               '<specular>0.05 0.05 0.05 1</specular></material></visual>')
        if collide:
            out += (f'<collision name="c{self.count}"><pose>{pose_s}</pose>'
                    f'<geometry>{geom}</geometry></collision>')
        return out

    def model(self, name, pose, parts):
        pose_s = ' '.join(f'{v:.4g}' for v in pose)
        self.models.append(
            f'<model name="{name}"><static>true</static><pose>{pose_s}'
            f'</pose><link name="link">{"".join(parts)}</link></model>')

    def footprint(self, name, x, y, w, d, yaw, z0, z1, kind='structure'):
        """Record an upright box for the truth file (world frame)."""
        self.structures.append({
            'name': name, 'kind': kind, 'centre': [round(x, 3), round(y, 3)],
            'size': [round(w, 3), round(d, 3)], 'yaw': round(yaw, 4),
            'z': [round(z0, 2), round(z1, 2)]})


def rot(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return c * x - s * y, s * x + c * y


# -- reusable builders --------------------------------------------------------

def road(w, name, x0, y0, x1, y1, width=7.0):
    """Flat strip with a dashed centre line (visual only)."""
    length = math.hypot(x1 - x0, y1 - y0)
    yaw = math.atan2(y1 - y0, x1 - x0)
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    parts = [w.part('box', (0, 0, 0.01, 0, 0, 0), (length, width, 0.02),
                    ROAD, collide=False)]
    n = int(length // 6)
    for i in range(n):
        parts.append(w.part(
            'box', (-length / 2 + 3 + 6 * i, 0, 0.022, 0, 0, 0),
            (2.2, 0.18, 0.004), MARK, collide=False))
    w.model(name, (cx, cy, 0, 0, 0, yaw), parts)


def house(w, name, x, y, width, depth, height, yaw=0.0, wall=0, roof=0,
          gable=True):
    """Box house with a gable roof and window bands."""
    colour = PLASTER[wall % len(PLASTER)]
    parts = [w.part('box', (0, 0, height / 2, 0, 0, 0),
                    (width, depth, height), colour)]
    storeys = max(1, int(height // 3))
    for s in range(storeys):
        z = 1.6 + 3.0 * s
        for side in (-1, 1):
            parts.append(w.part(
                'box', (0, side * (depth / 2 + 0.01), z, 0, 0, 0),
                (width * 0.7, 0.03, 1.0), WINDOW, collide=False))
            parts.append(w.part(
                'box', (side * (width / 2 + 0.01), 0, z, 0, 0, 0),
                (0.03, depth * 0.6, 1.0), WINDOW, collide=False))
    if gable:
        rise = 1.8
        half = depth / 2
        slope = math.hypot(half, rise)
        tilt = math.atan2(rise, half)
        for side in (-1, 1):
            parts.append(w.part(
                'box', (0, side * half / 2, height + rise / 2,
                        side * tilt, 0, 0),
                (width + 0.6, slope + 0.3, 0.15), ROOF[roof % len(ROOF)]))
    else:
        parts.append(w.part('box', (0, 0, height + 0.15, 0, 0, 0),
                            (width + 0.4, depth + 0.4, 0.3), DARK_CONCRETE))
    w.model(name, (x, y, 0, 0, 0, yaw), parts)
    w.footprint(name, x, y, width, depth, yaw, 0.0, height)


def tree(w, name, x, y, height=8.5, crown=2.6, leaf=0):
    trunk = height - crown
    parts = [w.part('cylinder', (0, 0, trunk / 2, 0, 0, 0), (0.28, trunk),
                    TRUNK),
             w.part('sphere', (0, 0, trunk + crown * 0.55, 0, 0, 0),
                    (crown,), LEAF[leaf % len(LEAF)]),
             w.part('sphere', (crown * 0.45, 0.3, trunk + crown * 0.1, 0, 0,
                               0), (crown * 0.7,), LEAF[(leaf + 1) % 3])]
    w.model(name, (x, y, 0, 0, 0, 0), parts)
    w.footprint(name, x, y, 2 * crown, 2 * crown, 0.0, trunk - crown * 0.45,
                height + crown * 0.55, kind='tree')


def wall(w, name, x0, y0, x1, y1, height=2.0, thick=0.25, colour=BRICK):
    length = math.hypot(x1 - x0, y1 - y0)
    yaw = math.atan2(y1 - y0, x1 - x0)
    w.model(name, ((x0 + x1) / 2, (y0 + y1) / 2, 0, 0, 0, yaw),
            [w.part('box', (0, 0, height / 2, 0, 0, 0),
                    (length, thick, height), colour)])
    w.footprint(name, (x0 + x1) / 2, (y0 + y1) / 2, length, thick, yaw, 0.0,
                height, kind='wall')


def manikin(w, name, x, y, yaw=0.0):
    """The Phase 7 rescue manikin (same shapes and colours)."""
    parts = [
        w.part('box', (0, 0, 1.03, 0, 0, 0), (0.42, 0.28, 0.68), ORANGE),
        w.part('sphere', (0, 0, 1.53, 0, 0, 0), (0.22,), SKIN),
        w.part('box', (0, 0.34, 1.05, 0.12, 0, 0.18), (0.18, 0.62, 0.18),
               ORANGE),
        w.part('box', (0, -0.34, 1.05, -0.12, 0, -0.18), (0.18, 0.62, 0.18),
               ORANGE),
        w.part('box', (0, 0.12, 0.34, 0, 0, 0), (0.22, 0.18, 0.68), BLUE),
        w.part('box', (0, -0.12, 0.34, 0, 0, 0), (0.22, 0.18, 0.68), BLUE)]
    w.model(name, (x, y, 0, 0, 0, yaw), parts)


def base(w):
    """Launch and landing pad, hut, mast, and an emergency landing field."""
    parts = [w.part('box', (0, 0, 0.005, 0, 0, 0), (8, 8, 0.01), CONCRETE,
                    collide=False),
             w.part('box', (0, 0, 0.012, 0, 0, 0), (2.4, 0.35, 0.004), MARK,
                    collide=False),
             w.part('box', (-1.0, 0, 0.012, 0, 0, 0), (0.35, 2.4, 0.004),
                    MARK, collide=False),
             w.part('box', (1.0, 0, 0.012, 0, 0, 0), (0.35, 2.4, 0.004),
                    MARK, collide=False)]
    w.model('base_pad', (0, 0, 0, 0, 0, 0), parts)
    house(w, 'base_hut', -11, -4, 5, 4, 3.2, 0.0, wall=2, gable=False)
    w.model('base_mast', (-11, -9, 0, 0, 0, 0), [
        w.part('cylinder', (0, 0, 5, 0, 0, 0), (0.12, 10), STEEL),
        w.part('box', (0, 0, 9.6, 0, 0, 0), (1.2, 0.1, 0.5), ORANGE,
               collide=False)])
    w.footprint('base_mast', -11, -9, 0.3, 0.3, 0.0, 0.0, 10.0, kind='pole')
    # emergency landing field: an open, marked rectangle
    w.model('emergency_field', (-18, 12, 0, 0, 0, 0), [
        w.part('box', (0, 0, 0.006, 0, 0, 0), (10, 10, 0.012),
               (0.55, 0.62, 0.45), collide=False),
        w.part('box', (0, 0, 0.014, 0, 0, 0.785), (6, 0.4, 0.004), MARK,
               collide=False),
        w.part('box', (0, 0, 0.014, 0, 0, -0.785), (6, 0.4, 0.004), MARK,
               collide=False)])


def collapsed_building(w, cx=35.0, cy=41.0):
    """Multi-storey block, mostly collapsed: a standing stair core, partial
    walls, exposed rooms, tilted floor slabs, beams and rubble piles.

    Footprint 26 m x 20 m. Only the stair core (12 m) and one wall panel
    (8.5 m) stand above the 6 m flight level."""
    p = []

    def box(x, y, z, sx, sy, sz, colour=CONCRETE, rpy=(0, 0, 0)):
        p.append(w.part('box', (x, y, z, rpy[0], rpy[1], rpy[2]),
                        (sx, sy, sz), colour))

    # floor plate
    p.append(w.part('box', (0, 0, 0.06, 0, 0, 0), (26, 20, 0.12),
                    DARK_CONCRETE))
    # standing stair core, north-west corner (12 m)
    box(-10, 7, 6.0, 6, 6, 12.0, PLASTER[3])
    w.footprint('collapsed_core', cx - 10, cy + 7, 6, 6, 0.0, 0.0, 12.0)
    for s in range(4):
        p.append(w.part('box', (-10, 3.99, 1.6 + 3 * s, 0, 0, 0),
                        (4, 0.03, 1.0), WINDOW, collide=False))
    # a tall wall panel still standing on the east side (8.5 m)
    box(12.8, -3, 4.25, 0.4, 7, 8.5, PLASTER[1])
    w.footprint('collapsed_east_panel', cx + 12.8, cy - 3, 0.4, 7, 0.0, 0.0,
                8.5)
    # ground-floor walls, broken to different heights (all below 5 m)
    for x, y, sx, sy, h in (
            (-3, 9.8, 8, 0.35, 3.2), (6, 9.8, 6, 0.35, 1.6),
            (12.8, 6, 0.35, 7, 2.4), (-12.8, -4, 0.35, 12, 2.8),
            (-6, -9.8, 10, 0.35, 3.0), (7, -9.8, 9, 0.35, 1.2),
            # interior partitions: rooms open to the sky
            (-4, 2, 0.3, 9, 2.6), (3, 4, 0.3, 6, 3.4), (3, -5, 0.3, 5, 2.0),
            (-8, -2, 7, 0.3, 2.4), (8, 0, 7, 0.3, 2.9), (0, -4, 6, 0.3, 1.8)):
        box(x, y, h / 2 + 0.12, sx, sy, h, PLASTER[(int(x) + 20) % 5])
    # collapsed upper floors: large tilted slabs
    box(4, 5.5, 2.3, 9, 7, 0.35, CONCRETE, (0.42, 0.10, 0.2))
    box(-2, -5.5, 1.5, 8, 6, 0.35, CONCRETE, (-0.30, 0.22, -0.3))
    box(9, -5, 1.1, 6, 5, 0.3, DARK_CONCRETE, (0.18, -0.35, 0.5))
    box(-7, 1, 3.4, 5, 4, 0.3, CONCRETE, (0.0, 0.5, 0.1))
    # beams and columns
    for x, y, yaw, tilt, length in ((1, 0, 0.3, 0.25, 9), (-5, 5, 1.2, -0.3,
                                                           7),
                                    (8, 3, -0.6, 0.15, 8), (10, -8, 0.1, 0.4,
                                                            6),
                                    (-9, -7, 0.9, 0.2, 7)):
        box(x, y, 1.0 + abs(tilt) * 2, length, 0.35, 0.45, DARK_CONCRETE,
            (0, tilt, yaw))
    for x, y, h in ((-1, 9, 4.6), (7, -9, 3.8), (-12, -9, 5.2), (12, 9.5,
                                                                 4.0)):
        box(x, y, h / 2, 0.45, 0.45, h, DARK_CONCRETE)
    # rubble piles
    k = 0
    for x, y, n in ((6, -2, 7), (-6, -6, 6), (0, 6, 6), (10, 7, 5),
                    (-10, -1, 5), (2, -8, 5)):
        for i in range(n):
            k += 1
            a = 2.399 * k
            r = 0.5 + 0.35 * (i % 4)
            s = 0.5 + 0.25 * ((k * 7) % 5)
            box(x + r * math.cos(a), y + r * math.sin(a), 0.12 + s * 0.35,
                s, s * 0.8, s * 0.7, RUBBLE if k % 3 else DARK_CONCRETE,
                (0.3 * math.sin(k), 0.3 * math.cos(k * 1.3), a))
    w.model('collapsed_block', (cx, cy, 0, 0, 0, 0), p)
    # rubble thrown onto the street around it
    k = 0
    for x, y in ((cx - 15, cy - 6), (cx + 15, cy + 4), (cx + 3, cy - 12.5),
                 (cx - 5, cy + 12.5)):
        parts = []
        for i in range(5):
            k += 1
            s = 0.5 + 0.2 * (k % 4)
            parts.append(w.part(
                'box', (0.8 * math.cos(2.4 * k), 0.8 * math.sin(2.4 * k),
                        s * 0.3, 0.2 * k, 0.1 * k, 0.5 * k),
                (s, s * 0.8, s * 0.6), RUBBLE))
        w.model(f'street_rubble_{k}', (x, y, 0, 0, 0, 0), parts)


def damaged_house(w, x=66.0, y=40.0):
    """A house with one half standing (9 m) and the other collapsed."""
    p = [w.part('box', (-3.5, 0, 4.5, 0, 0, 0), (7, 10, 9.0), PLASTER[0]),
         w.part('box', (-3.5, 0, 9.15, 0, 0, 0), (7.4, 10.4, 0.3),
                DARK_CONCRETE)]
    for s in range(3):
        p.append(w.part('box', (-7.01, 0, 1.6 + 3 * s, 0, 0, 0),
                        (0.03, 6, 1.0), WINDOW, collide=False))
    # collapsed half: low walls, a fallen slab, rubble, an open courtyard
    for bx, by, sx, sy, h in ((3.5, 4.9, 7, 0.3, 2.2), (6.9, 0, 0.3, 10, 1.4),
                              (3.5, -4.9, 7, 0.3, 2.8)):
        p.append(w.part('box', (bx, by, h / 2, 0, 0, 0), (sx, sy, h),
                        PLASTER[4]))
    p.append(w.part('box', (3.2, 1.5, 1.6, 0.35, 0.15, 0.3), (6, 5, 0.3),
                    CONCRETE))
    for i in range(8):
        s = 0.5 + 0.2 * (i % 3)
        p.append(w.part(
            'box', (3.5 + 2.2 * math.cos(2.4 * i), -1 + 2.4 * math.sin(
                2.4 * i), s * 0.3, 0.3 * i, 0.2 * i, 0.7 * i),
            (s, s * 0.8, s * 0.6), RUBBLE))
    w.model('damaged_house', (x, y, 0, 0, 0, 0), p)
    w.footprint('damaged_house_standing', x - 3.5, y, 7, 10, 0.0, 0.0, 9.3)


def distractor(w, spec, name, kind, x, y):
    """Objects that share the survivor's colours."""
    if kind == 'barrel':
        parts = [w.part('cylinder', (0, 0, 0.45, 0, 0, 0), (0.3, 0.9),
                        ORANGE)]
        size = [0.6, 0.6, 0.9]
    elif kind == 'cone':
        parts = [w.part('cylinder', (0, 0, 0.35, 0, 0, 0), (0.18, 0.7),
                        ORANGE),
                 w.part('box', (0, 0, 0.02, 0, 0, 0), (0.5, 0.5, 0.04),
                        ORANGE)]
        size = [0.5, 0.5, 0.7]
    elif kind == 'tarp':
        parts = [w.part('box', (0, 0, 0.05, 0, 0, 0.3), (1.6, 1.1, 0.06),
                        ORANGE)]
        size = [1.6, 1.1, 0.1]
    elif kind == 'crate_and_ball':      # orange box touching a skin-tone ball
        parts = [w.part('box', (0, 0, 0.3, 0, 0, 0), (0.6, 0.45, 0.6),
                        ORANGE),
                 w.part('sphere', (0.45, 0, 0.2, 0, 0, 0), (0.2,), SKIN)]
        size = [1.0, 0.45, 0.6]
    else:
        raise ValueError(kind)
    w.model(name, (x, y, 0, 0, 0, 0), parts)
    spec['objects'].append({
        'name': name, 'kind': 'distractor', 'shape': kind,
        'position': [x, y, size[2] / 2], 'size': size, 'yaw': 0.0})


def cover(w, name, x, y, yaw, height=1.9, colour=CONCRETE):
    """A leaning slab that hides part of a survivor from above."""
    w.model(name, (x, y, 0, 0, 0, yaw), [
        w.part('box', (0, 0.25, height, 0.0, 0.35, 0), (1.7, 1.2, 0.12),
               colour),
        w.part('box', (0.75, 0.25, height / 2 - 0.15, 0, 0, 0),
               (0.15, 1.0, height - 0.3), DARK_CONCRETE)])


# -- the village --------------------------------------------------------------

def build_village(w):
    w.model('ground_visual', (32, 40, 0, 0, 0, 0), [
        w.part('box', (0, 0, -0.01, 0, 0, 0), (170, 175, 0.02), GRASS,
               collide=False)])
    # roads
    road(w, 'road_r1', 12, -22, 12, 102)
    road(w, 'road_r2', -28, 20, 92, 20)
    road(w, 'road_r3', -28, 62, 92, 62)
    road(w, 'road_r4', 54, 20, 54, 102)
    road(w, 'road_base', 2, 0, 8.5, 0, width=5)
    base(w)

    # (name, x, y, width, depth, height, yaw, wall colour, roof colour)
    houses = [
        # between the base and road R2: blocks the straight line to the site
        ('house_s1', 3, 11, 9, 7, 7.5, 0.0, 0, 0),
        ('house_s2', 26, 6, 10, 8, 8.0, 0.0, 1, 1),
        ('house_s3', 40, 9, 9, 8, 9.0, 0.1, 2, 2),
        ('house_s4', 26, -12, 11, 8, 7.5, 0.0, 3, 0),
        ('house_s5', 64, 8, 10, 9, 8.5, 0.0, 4, 1),
        ('house_s6', 80, 6, 9, 8, 7.5, -0.1, 0, 2),
        # west of road R1
        ('house_w1', -4, 33, 10, 8, 8.0, 0.0, 1, 1),
        ('house_w2', -4, 47, 10, 8, 9.5, 0.0, 2, 0),
        ('house_w3', -20, 34, 9, 9, 7.5, 0.2, 3, 2),
        ('house_w4', -20, 50, 9, 8, 8.5, 0.0, 4, 1),
        # an alley 6 m wide between two long houses, east of road R4
        ('house_e1', 84, 33, 10, 8, 8.0, 0.0, 0, 0),
        ('house_e2', 84, 47, 10, 8, 9.0, 0.0, 1, 2),
        # north of road R3
        ('house_n1', -10, 74, 10, 8, 8.0, 0.0, 2, 1),
        ('house_n2', -10, 90, 10, 9, 9.0, 0.0, 3, 0),
        ('house_n3', 66, 74, 10, 8, 8.5, 0.0, 4, 2),
        ('house_n4', 82, 78, 9, 9, 7.5, 0.3, 0, 1),
        ('house_n5', 68, 92, 11, 8, 9.5, 0.0, 1, 0),
    ]
    for item in houses:
        house(w, *item)
    # landmarks
    house(w, 'chapel', 26, 96, 8, 12, 8.0, 0.0, wall=2, roof=2)
    w.model('chapel_tower', (26, 88, 0, 0, 0, 0), [
        w.part('box', (0, 0, 7.5, 0, 0, 0), (3.5, 3.5, 15.0), PLASTER[2]),
        w.part('box', (0, 0, 15.6, 0, 0, 0.785), (3.0, 3.0, 1.2), ROOF[2])])
    w.footprint('chapel_tower', 26, 88, 3.5, 3.5, 0.0, 0.0, 16.2)
    w.model('water_tower', (88, 96, 0, 0, 0, 0), [
        w.part('cylinder', (0, 0, 5.0, 0, 0, 0), (0.8, 10.0), STEEL),
        w.part('cylinder', (0, 0, 12.0, 0, 0, 0), (3.0, 4.0), STEEL)])
    w.footprint('water_tower', 88, 96, 1.6, 1.6, 0.0, 0.0, 14.0, kind='pole')
    # market square north of R3 with stalls (low) and a fountain
    w.model('market_square', (34, 78, 0, 0, 0, 0), [
        w.part('box', (0, 0, 0.008, 0, 0, 0), (24, 18, 0.016),
               (0.70, 0.66, 0.58), collide=False),
        w.part('cylinder', (0, 0, 0.3, 0, 0, 0), (2.0, 0.6), CONCRETE),
        w.part('cylinder', (0, 0, 1.0, 0, 0, 0), (0.4, 1.4), CONCRETE),
        w.part('box', (-8, 5, 1.2, 0, 0, 0), (3, 2, 2.4), (0.6, 0.2, 0.2)),
        w.part('box', (8, -5, 1.2, 0, 0, 0), (3, 2, 2.4), (0.2, 0.3, 0.6)),
        w.part('box', (7, 6, 1.2, 0, 0, 0), (3, 2, 2.4), (0.25, 0.5, 0.3))])
    # trees (crowns reach the flight level)
    trees = [(18, -8), (18, 30), (18, 52), (6, 26), (-12, 22), (48, 26),
             (48, 58), (20, 58), (60, 26), (60, 56), (74, 24), (-26, 62),
             (0, 68), (46, 68), (60, 66), (90, 64), (40, -8), (58, -6),
             (-24, -14), (76, 56), (-6, 60), (8, 84), (50, 92), (36, 16)]
    for i, (x, y) in enumerate(trees):
        tree(w, f'tree_{i:02d}', x, y, height=8.0 + (i % 3) * 0.6,
             crown=2.3 + (i % 4) * 0.2, leaf=i)
    # boundary and garden walls (below the flight level)
    wall(w, 'wall_south', -30, -24, 94, -24)
    wall(w, 'wall_west', -30, -24, -30, 104)
    wall(w, 'wall_north', -30, 104, 94, 104)
    wall(w, 'wall_east', 94, -24, 94, 104)
    wall(w, 'garden_w1', -10, 28, -10, 40, 1.6)
    wall(w, 'garden_n3', 60, 80, 72, 80, 1.6)
    wall(w, 'yard_e', 78, 28, 78, 52, 1.8)
    # gentle mounds (visual terrain variation; the ground stays flat where
    # the UAV searches)
    for i, (x, y, r) in enumerate(((-22, 84, 7), (84, -14, 8), (-22, 4, 5))):
        w.model(f'mound_{i}', (x, y, -r * 0.82, 0, 0, 0), [
            w.part('sphere', (0, 0, 0, 0, 0, 0), (r,), GRASS)])
    collapsed_building(w)
    damaged_house(w)


SCENARIOS = {
    # name: (description, survivors [(name, x, y, yaw)], distractors
    #        [(name, kind, x, y)], covers [(x, y, yaw)])
    'village_A': ('reconnaissance, no survivors, no look-alikes', [], [], []),
    'village_B': ('one survivor in an exposed room of the collapsed building',
                  [('survivor_room', 45.5, 44.0, 0.4)], [], []),
    'village_B2': ('one survivor, a different room (not the place of B)',
                   [('survivor_west_room', 26.5, 36.5, 1.2)], [], []),
    'village_C': ('three survivors in two buildings, with look-alikes',
                  [('survivor_room', 45.5, 44.0, 0.4),
                   ('survivor_west_room', 26.5, 36.5, 1.2),
                   ('survivor_courtyard', 71.0, 36.4, 2.0)],
                  [('barrel_street', 'barrel', 22.5, 30.0),
                   ('cone_road', 'cone', 53.0, 33.0),
                   ('tarp_rubble', 'tarp', 32.0, 49.0),
                   ('crate_ball', 'crate_and_ball', 40.0, 33.0),
                   ('barrel_yard', 'barrel', 71.5, 43.0)], []),
    'village_D': ('difficult rubble: two survivors partly under slabs',
                  [('survivor_under_slab', 33.5, 44.0, 0.2),
                   ('survivor_by_beam', 26.5, 36.5, 2.4)],
                  [('tarp_rubble', 'tarp', 32.0, 49.0)],
                  [(33.5, 44.0, 0.2), (26.5, 36.5, 2.4)]),
    'village_F': ('no survivors, look-alikes only (false-positive control)',
                  [],
                  [('barrel_street', 'barrel', 22.5, 30.0),
                   ('cone_road', 'cone', 53.0, 33.0),
                   ('tarp_rubble', 'tarp', 32.0, 49.0),
                   ('crate_ball', 'crate_and_ball', 40.0, 33.0),
                   ('barrel_room', 'barrel', 45.5, 44.0)], []),
}


def sdf_header():
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<!-- GENERATED by scripts/village/make_village.py - do not edit by hand. -->
<sdf version="1.9">
  <world name="{WORLD_NAME}">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <!-- same geographic reference as the original world: PX4 compares the
         simulated magnetometer with the field expected at this location -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg>8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>
    <scene>
      <grid>false</grid>
      <ambient>0.45 0.45 0.45 1</ambient>
      <background>0.62 0.74 0.88 1</background>
      <shadows>true</shadows>
    </scene>
    <gui fullscreen="0">
      <plugin filename="MinimalScene" name="3D View">
        <gz-gui><title>3D View</title><property type="bool" key="showTitleBar">false</property><property type="string" key="state">docked</property></gz-gui>
        <engine>ogre2</engine><scene>scene</scene>
        <ambient_light>0.45 0.45 0.45</ambient_light>
        <background_color>0.62 0.74 0.88</background_color>
        <camera_pose>-38 -52 62 0 0.62 0.86</camera_pose>
      </plugin>
      <plugin filename="GzSceneManager" name="Scene Manager"><gz-gui><property key="resizable" type="bool">false</property><property key="width" type="double">5</property><property key="height" type="double">5</property><property key="state" type="string">floating</property><property key="showTitleBar" type="bool">false</property></gz-gui></plugin>
      <plugin filename="InteractiveViewControl" name="Interactive view control"><gz-gui><property key="resizable" type="bool">false</property><property key="width" type="double">5</property><property key="height" type="double">5</property><property key="state" type="string">floating</property><property key="showTitleBar" type="bool">false</property></gz-gui></plugin>
      <plugin filename="CameraTracking" name="Camera Tracking"><gz-gui><property key="resizable" type="bool">false</property><property key="width" type="double">5</property><property key="height" type="double">5</property><property key="state" type="string">floating</property><property key="showTitleBar" type="bool">false</property></gz-gui></plugin>
      <plugin filename="WorldControl" name="World control"><gz-gui><title>World control</title><property type="bool" key="showTitleBar">false</property><property type="bool" key="resizable">false</property><property type="double" key="height">72</property><property type="double" key="z">1</property><property type="string" key="state">floating</property><anchors target="3D View"><line own="left" target="left"/><line own="bottom" target="bottom"/></anchors></gz-gui><play_pause>true</play_pause><step>true</step><start_paused>false</start_paused></plugin>
      <plugin filename="WorldStats" name="World stats"><gz-gui><title>World stats</title><property type="bool" key="showTitleBar">false</property><property type="bool" key="resizable">false</property><property type="double" key="height">110</property><property type="double" key="width">290</property><property type="double" key="z">1</property><property type="string" key="state">floating</property><anchors target="3D View"><line own="right" target="right"/><line own="bottom" target="bottom"/></anchors></gz-gui><sim_time>true</sim_time><real_time>true</real_time><real_time_factor>true</real_time_factor><iterations>false</iterations></plugin>
    </gui>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 80 0 0 0</pose>
      <diffuse>0.95 0.93 0.88 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.45 0.30 -0.84</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>400 400</size></plane></geometry></collision>
      </link>
    </model>
"""


def generate(name):
    description, survivors, distractors, covers = SCENARIOS[name]
    w = World()
    build_village(w)
    spec = {'world': name, 'description': description,
            'sdf': f'src/uav_autonomy/worlds/village/{name}.sdf',
            'survivors': [], 'objects': []}
    for sname, x, y, yaw in survivors:
        manikin(w, sname, x, y, yaw)
        spec['survivors'].append({'name': sname, 'position': [x, y],
                                  'yaw': yaw})
    for dname, kind, x, y in distractors:
        distractor(w, spec, dname, kind, x, y)
    for i, (x, y, yaw) in enumerate(covers):
        cover(w, f'cover_{i}', x, y, yaw)
    spec['structures'] = w.structures
    spec['places'] = 'missions/places_village.json'
    os.makedirs(OUT_SDF, exist_ok=True)
    os.makedirs(OUT_SPEC, exist_ok=True)
    text = sdf_header() + '\n'.join(
        '    ' + m for m in w.models) + '\n  </world>\n</sdf>\n'
    with open(os.path.join(OUT_SDF, f'{name}.sdf'), 'w') as handle:
        handle.write(text)
    # the same world without shadows, for headless evaluation (measured:
    # real-time factor 0.69 with shadows, 0.83 without, on this machine)
    os.makedirs(OUT_SDF + '_fast', exist_ok=True)
    with open(os.path.join(OUT_SDF + '_fast', f'{name}.sdf'), 'w') as handle:
        handle.write(text.replace('<shadows>true</shadows>',
                                  '<shadows>false</shadows>').replace(
            '<cast_shadows>true</cast_shadows>',
            '<cast_shadows>false</cast_shadows>'))
    with open(os.path.join(OUT_SPEC, f'{name}.json'), 'w') as handle:
        json.dump(spec, handle, indent=1)
    return len(w.models), len(survivors), len(distractors)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--scenario', action='append')
    args = parser.parse_args()
    for name in args.scenario or sorted(SCENARIOS):
        models, survivors, objects = generate(name)
        print(f'{name}: {models} models, {survivors} survivor(s), '
              f'{objects} look-alike(s) -> worlds/village/{name}.sdf')


if __name__ == '__main__':
    main()
