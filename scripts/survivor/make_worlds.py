#!/usr/bin/env python3
"""Generate the Phase 7 test worlds from the project world (deterministic).

Each variant keeps the world name "collapsed_building_rescue" (so the camera
bridge and PX4 attach unchanged) and differs only in what is placed in it:
the survivor manikin (moved, removed, duplicated), an occluding panel,
rubble around it, and distractor objects.

Outputs
    src/uav_autonomy/worlds/survivor_variants/<name>.sdf
    results/phase7/worlds/<name>.json   positions of survivors and
        distractors, for OFFLINE scoring only. Nothing that runs in flight
        reads these files or the SDF.
"""

import json
import os
import re

WS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
BASE = os.path.join(WS, 'src/uav_autonomy/worlds/collapsed_building_rescue.sdf')
OUT_SDF = os.path.join(WS, 'src/uav_autonomy/worlds/survivor_variants')
OUT_SPEC = os.path.join(WS, 'results/phase7/worlds')

GREY = (0.45, 0.45, 0.45)
ORANGE = (1.0, 0.28, 0.04)         # same material as the manikin's clothing
TAN = (0.58, 0.50, 0.38)           # debris colours of the spawner
BROWN = (0.48, 0.34, 0.20)
SKIN = (1.0, 0.78, 0.54)           # same material as the manikin's head
BLUE = (0.025, 0.16, 0.72)
RED = (0.8, 0.05, 0.05)
YELLOW = (0.95, 0.85, 0.1)


def material(colour):
    r, g, b = colour
    return (f'<material><ambient>{r * 0.95:.3f} {g * 0.75:.3f} '
            f'{b * 0.75:.3f} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>'
            '<specular>0.1 0.1 0.1 1</specular></material>')


def static_model(name, pose, geometry, colour):
    return f'''
    <model name="{name}">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="link">
        <collision name="collision"><geometry>{geometry}</geometry></collision>
        <visual name="visual"><geometry>{geometry}</geometry>{material(colour)}</visual>
      </link>
    </model>
'''


def box(name, x, y, z, sx, sy, sz, colour, yaw=0.0, kind='distractor'):
    return {'name': name, 'kind': kind, 'shape': 'box',
            'position': [x, y, z], 'size': [sx, sy, sz], 'yaw': yaw,
            'colour': colour,
            'sdf': static_model(name, f'{x} {y} {z} 0 0 {yaw}',
                                f'<box><size>{sx} {sy} {sz}</size></box>',
                                colour)}


def sphere(name, x, y, z, r, colour, kind='distractor'):
    return {'name': name, 'kind': kind, 'shape': 'sphere',
            'position': [x, y, z], 'size': [2 * r, 2 * r, 2 * r], 'yaw': 0.0,
            'colour': colour,
            'sdf': static_model(name, f'{x} {y} {z} 0 0 0',
                                f'<sphere><radius>{r}</radius></sphere>',
                                colour)}


def cylinder(name, x, y, z, r, length, colour, kind='distractor'):
    return {'name': name, 'kind': kind, 'shape': 'cylinder',
            'position': [x, y, z], 'size': [2 * r, 2 * r, length],
            'yaw': 0.0, 'colour': colour,
            'sdf': static_model(
                name, f'{x} {y} {z} 0 0 0',
                f'<cylinder><radius>{r}</radius><length>{length}</length>'
                '</cylinder>', colour)}


def main():
    base = open(BASE).read()
    match = re.search(r'\n\s*<!--[^\n]*manikin[^\n]*-->\s*\n\s*<model name="survivor_target">.*?</model>\n',
                      base, re.S)
    assert match, 'survivor model not found in the base world'
    survivor_block = match.group(0)
    without = base.replace(survivor_block, '\n')
    assert 'survivor_target' not in without

    def survivor_at(name, x, y, yaw=0.0):
        block = survivor_block.replace('name="survivor_target"',
                                       f'name="{name}"')
        return block.replace('<pose>1.2 0 0 0 0 0</pose>',
                             f'<pose>{x} {y} 0 0 0 {yaw}</pose>', 1)

    inside = {'name': 'survivor_target', 'position': [1.2, 0.0], 'yaw': 0.0}
    outside = {'name': 'survivor_outdoor', 'position': [0.0, 9.0],
               'yaw': 0.6}

    distractors = [
        # where the manikin stands in the base world: an orange box
        box('orange_box', 1.2, 0.0, 0.25, 0.5, 0.5, 0.5, ORANGE),
        cylinder('orange_cylinder', 0.35, 0.65, 0.3, 0.15, 0.6, ORANGE),
        sphere('skin_sphere', -0.45, -0.55, 0.22, 0.22, SKIN),
        box('blue_box', 0.25, -0.65, 0.2, 0.4, 0.3, 0.4, BLUE),
        box('red_box', -0.6, 0.45, 0.2, 0.4, 0.4, 0.4, RED),
        box('yellow_box', 1.75, -0.6, 0.2, 0.35, 0.35, 0.4, YELLOW),
        # hard case: orange object with a skin-coloured sphere beside it
        box('orange_slab', 1.7, 0.55, 0.15, 0.7, 0.3, 0.3, ORANGE, yaw=0.3),
        sphere('skin_sphere_by_slab', 1.25, 0.6, 0.2, 0.2, SKIN),
        # on the roof and outside, along the transit route
        box('orange_box_roof', -1.8, 0.2, 3.25, 0.4, 0.4, 0.4, ORANGE),
        box('brown_box_roof', 0.0, 1.6, 3.2, 0.4, 0.4, 0.4, BROWN),
        box('orange_plank_outside', 0.6, 6.5, 0.1, 1.5, 0.3, 0.2, ORANGE,
            yaw=0.4),
        box('tan_box_outside', -0.5, 8.0, 0.2, 0.4, 0.4, 0.4, TAN),
    ]
    occluder = [box('fallen_panel', 1.08, 0.32, 1.97, 0.7, 1.0, 0.05, GREY,
                    kind='occluder')]
    rubble = [
        box('rubble_tan', 0.72, 0.52, 0.3, 0.5, 0.5, 0.6, TAN, 0.4, 'rubble'),
        box('rubble_brown', 1.72, -0.45, 0.25, 0.5, 0.4, 0.5, BROWN, -0.3,
            'rubble'),
        box('rubble_grey', 1.3, 0.85, 0.2, 0.6, 0.3, 0.4, GREY, 0.2, 'rubble'),
        box('rubble_pillar', 1.25, -0.88, 0.9, 0.35, 0.3, 1.8, BROWN, 0.1,
            'rubble'),
        box('rubble_tan_small', 0.75, -0.35, 0.15, 0.3, 0.3, 0.3, TAN, 0.7,
            'rubble'),
    ]
    variants = {
        'occluded': (base, [inside], occluder),
        'rubble': (base, [inside], rubble),
        'none': (without, [], []),
        'distractors': (without, [], distractors),
        'outdoor': (without.replace('</world>', survivor_at(
            'survivor_outdoor', 0.0, 9.0, 0.6) + '\n</world>'),
            [outside], []),
        'multi': (base.replace('</world>', survivor_at(
            'survivor_outdoor', 0.0, 9.0, 0.6) + '\n</world>'),
            [inside, outside],
            [d for d in distractors if d['name'] in (
                'orange_cylinder', 'skin_sphere', 'orange_box_roof',
                'orange_plank_outside', 'tan_box_outside')]),
    }
    os.makedirs(OUT_SDF, exist_ok=True)
    os.makedirs(OUT_SPEC, exist_ok=True)
    json.dump({'world': 'base', 'sdf': os.path.relpath(BASE, WS),
               'survivors': [inside], 'objects': []},
              open(os.path.join(OUT_SPEC, 'base.json'), 'w'), indent=1)
    for name, (sdf, survivors, objects) in variants.items():
        text = sdf.replace('</world>', ''.join(o['sdf'] for o in objects)
                           + '\n</world>')
        assert text.count('<world name="collapsed_building_rescue">') == 1
        path = os.path.join(OUT_SDF, f'{name}.sdf')
        open(path, 'w').write(text)
        json.dump({'world': name, 'sdf': os.path.relpath(path, WS),
                   'survivors': survivors,
                   'objects': [{k: v for k, v in o.items() if k != 'sdf'}
                               for o in objects]},
                  open(os.path.join(OUT_SPEC, f'{name}.json'), 'w'),
                  indent=1)
        print(f'{name}: {len(survivors)} survivor(s), {len(objects)} '
              f'object(s) -> {os.path.relpath(path, WS)}')


if __name__ == '__main__':
    main()
