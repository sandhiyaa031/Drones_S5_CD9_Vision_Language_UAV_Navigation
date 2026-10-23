#!/usr/bin/env python3
"""Build the MuJoCo urban-disaster scene (MJCF) from the exported shapes.

Input : worlds/shapes/<scenario>.json - the collision shapes of the Gazebo
        urban disaster zone (same layout, written by ros2_ws/scripts/disaster/
        make_disaster_zone.py --shapes-only). The shapes are boxes and
        cylinders in the world frame (ENU, base pad at the origin).
Output: worlds/mj_<scenario>.xml

The MuJoCo scene is a simplification of the Gazebo world: the same walls,
floor slabs, columns and rubble mounds as solid primitives with the same
procedural textures, but no window meshes (facade openings in the 5.5-7 m
band are kept because the shapes are split around them), no beams and no
particle effects. Survivors are the same manikin (orange torso, skin head,
blue legs); look-alikes are the same barrels, cone, tarp and crate.

Usage: make_mujoco_scene.py [--scenario urban_C ...]   (default: all)
"""

import argparse
import json
import math
import os

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..'))

ORANGE = (1.0, 0.28, 0.04)
SKIN = (1.0, 0.78, 0.54)
BLUE = (0.025, 0.16, 0.72)
N_DEBRIS = 6


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def quat(rpy):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, rpy_matrix(*rpy).flatten())
    return q


def f(values):
    return ' '.join(f'{v:.4f}' for v in values)


def rgba(c, a=1.0):
    return f'{c[0]:.3f} {c[1]:.3f} {c[2]:.3f} {a}'


def manikin(name, x, y, z, yaw):
    def part(kind, pos, size, c, rpy=(0, 0, 0)):
        q = quat(rpy)
        return (f'<geom type="{kind}" pos="{f(pos)}" size="{f(size)}" '
                f'quat="{f(q)}" rgba="{rgba(c)}" contype="0" conaffinity="0"/>')
    q = quat((0, 0, yaw))
    return (f'<body name="{name}" pos="{x:.3f} {y:.3f} {z:.3f}" quat="{f(q)}">'
            + part('box', (0, 0, 1.03), (0.21, 0.14, 0.34), ORANGE)
            + part('sphere', (0, 0, 1.53), (0.22,), SKIN)
            + part('box', (0, 0.34, 1.05), (0.09, 0.31, 0.09), ORANGE,
                   (0.12, 0, 0.18))
            + part('box', (0, -0.34, 1.05), (0.09, 0.31, 0.09), ORANGE,
                   (-0.12, 0, -0.18))
            + part('box', (0, 0.12, 0.34), (0.11, 0.09, 0.34), BLUE)
            + part('box', (0, -0.12, 0.34), (0.11, 0.09, 0.34), BLUE)
            + '</body>')


def distractor(name, shape, x, y):
    def geom(kind, pos, size, c):
        return (f'<geom type="{kind}" pos="{f(pos)}" size="{f(size)}" '
                f'rgba="{rgba(c)}" contype="0" conaffinity="0"/>')
    if shape == 'barrel':
        body = geom('cylinder', (0, 0, 0.45), (0.3, 0.45), ORANGE)
    elif shape == 'cone':
        body = (geom('cylinder', (0, 0, 0.35), (0.18, 0.35), ORANGE)
                + geom('box', (0, 0, 0.02), (0.25, 0.25, 0.02), ORANGE))
    elif shape == 'tarp':
        body = geom('box', (0, 0, 0.05), (0.8, 0.55, 0.03), ORANGE)
    else:                                   # crate_and_ball
        body = (geom('box', (0, 0, 0.3), (0.3, 0.225, 0.3), ORANGE)
                + geom('sphere', (0.45, 0, 0.2), (0.2,), SKIN))
    return f'<body name="{name}" pos="{x:.3f} {y:.3f} 0">{body}</body>'


def build(scenario):
    src = os.path.join(ROOT, 'worlds', 'shapes', f'{scenario}.json')
    data = json.load(open(src))
    mats = ['concrete', 'dark_concrete', 'beige', 'masonry', 'rubble', 'rust',
            'asphalt', 'dirt']
    tex = '\n'.join(
        f'    <texture name="t_{m}" type="2d" file="tex_{m}.png"/>'
        f'\n    <material name="m_{m}" texture="t_{m}" texuniform="true" '
        f'texrepeat="0.3333 0.3333" specular="0.05" shininess="0.05"/>'
        for m in mats)
    geoms = []
    for s in data['shapes']:
        mat = f'm_{s["material"]}'
        q = quat(s['rpy'])
        if s['kind'] == 'box':
            half = [v / 2 for v in s['dims']]
            geoms.append(
                f'<geom type="box" pos="{f(s["pos"])}" size="{f(half)}" '
                f'quat="{f(q)}" material="{mat}" contype="1" conaffinity="1"/>')
        else:
            r, length = s['dims']
            geoms.append(
                f'<geom type="cylinder" pos="{f(s["pos"])}" '
                f'size="{r:.3f} {length / 2:.3f}" quat="{f(q)}" '
                f'material="{mat}" contype="1" conaffinity="1"/>')
    extras = []
    for sv in data['survivors']:
        extras.append(manikin(sv['name'], sv['position'][0], sv['position'][1],
                              sv['z'], sv['yaw']))
    for ob in data['objects']:
        extras.append(distractor(ob['name'], ob['shape'], ob['position'][0],
                                 ob['position'][1]))
    # falling-debris pool: free boxes parked far away; the scenario "releases"
    # one by writing its position and velocity (physics does the rest)
    debris = []
    for i in range(N_DEBRIS):
        size = (0.20, 0.20, 0.20)
        debris.append(
            f'<body name="debris{i}" pos="{-120 + 3 * i} -110 0.2">'
            f'<freejoint name="debris{i}_joint"/>'
            f'<geom name="debris{i}_geom" type="box" size="{f(size)}" '
            f'mass="1.5" rgba="0.55 0.20 0.65 1" contype="4" conaffinity="1"/>'
            '</body>')
    roads = [
        ('road_avenue', 27.5, 27.0, 67.5, 5.0),
        ('road_cross', 33.0, 43.0, 5.0, 57.0),
    ]
    road_xml = '\n    '.join(
        f'<geom name="{n}" type="box" pos="{cx} {cy} 0.01" size="{hx} {hy} 0.01" '
        'material="m_asphalt" contype="0" conaffinity="0"/>'
        for n, cx, cy, hx, hy in roads)
    xml = f'''<mujoco model="mj_{scenario}">
  <!-- GENERATED by scripts/make_mujoco_scene.py from worlds/shapes/{scenario}.json -->
  <compiler texturedir="../models/textures" angle="radian"/>
  <option timestep="0.002" gravity="0 0 -9.81" cone="elliptic"/>
  <size memory="64M"/>
  <visual>
    <global offwidth="1280" offheight="960"/>
    <quality shadowsize="2048"/>
    <headlight ambient="0.42 0.42 0.43" diffuse="0.35 0.35 0.35" specular="0 0 0"/>
    <rgba haze="0.52 0.53 0.55 1"/>
  </visual>
  <asset>
    <texture name="sky" type="skybox" builtin="flat" rgb1="0.52 0.53 0.55" rgb2="0.52 0.53 0.55" width="32" height="32"/>
{tex}
  </asset>
  <worldbody>
    <light name="overcast" directional="true" pos="0 0 80" dir="-0.4 0.3 -0.85" diffuse="0.5 0.5 0.49" castshadow="true"/>
    <geom name="ground" type="plane" size="150 150 0.1" material="m_dirt" contype="1" conaffinity="1" friction="0.9 0.01 0.001"/>
    {road_xml}
    <geom name="base_pad" type="box" pos="0 0 0.02" size="7 7 0.02" material="m_concrete" contype="0" conaffinity="0"/>
    {chr(10).join('    ' + g for g in geoms)}
    {chr(10).join('    ' + e for e in extras)}
    {chr(10).join('    ' + d for d in debris)}
  </worldbody>
</mujoco>
'''
    # the vehicle is added at the mujoco level (actuators included)
    xml = xml.replace('</mujoco>', '  <include file="../models/x500_body.xml"/>\n</mujoco>')
    out = os.path.join(ROOT, 'worlds', f'mj_{scenario}.xml')
    open(out, 'w').write(xml)
    m = mujoco.MjModel.from_xml_path(out)
    return out, m.ngeom, m.nbody


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--scenario', action='append')
    args = parser.parse_args()
    names = args.scenario or sorted(
        f[:-5] for f in os.listdir(os.path.join(ROOT, 'worlds', 'shapes'))
        if f.endswith('.json'))
    for name in names:
        out, ngeom, nbody = build(name)
        print(f'{name}: {ngeom} geoms, {nbody} bodies -> {os.path.relpath(out, ROOT)}')


if __name__ == '__main__':
    main()
