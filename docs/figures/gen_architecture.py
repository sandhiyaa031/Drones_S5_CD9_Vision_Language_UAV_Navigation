#!/usr/bin/env python3
"""Generate the simplified UAV disaster-rescue architecture diagram (16:9) for slides.

Usage:  python3 gen_architecture.py            # writes architecture.drawio next to this file
        python3 gen_architecture.py --png      # also exports architecture.png (needs google-chrome + network)
"""
import html
import json
import os
import subprocess
import sys
import tempfile
import urllib.request
from xml.sax.saxutils import escape

HERE = os.path.dirname(os.path.abspath(__file__))
W, H = 2000, 1125                      # 16:9 canvas

FONT = 'fontFamily=Helvetica;'
LINE = '#4A5A6A'
PALETTE = {                            # fill, stroke
    'mission': ('#E8F0FB', '#7A9CC6'),
    'percep':  ('#E9F5E9', '#7FB383'),
    'nav':     ('#F0EAF6', '#9C82B3'),
    'flight':  ('#FDECEA', '#C9807C'),
    'px4':     ('#FFF6DD', '#D9B95A'),
}

cells, geo = [], {}


def q(s):
    return escape(s, {'"': '&quot;'})


def block(cid, x, y, w, h, label, kind, size=20, dashed=False, white=False, bold=True):
    fill, stroke = PALETTE[kind]
    if white:
        fill = '#FFFFFF'
    st = (f'rounded=1;arcSize=14;whiteSpace=wrap;html=1;{FONT}fontSize={size};'
          f'fontColor=#1F2933;fillColor={fill};strokeColor={stroke};strokeWidth=2;'
          f'align=center;verticalAlign=middle;spacing=8;' + ('dashed=1;dashPattern=8 5;' if dashed else '')
          + ('fontStyle=1;' if bold else ''))
    geo[cid] = (x, y, w, h)
    cells.append(f'<mxCell id="{cid}" value="{q(label)}" style="{st}" vertex="1" parent="1">'
                 f'<mxGeometry x="{x}" y="{y}" width="{w}" height="{h}" as="geometry"/></mxCell>')


def group(cid, x, y, w, h, title, kind):
    fill, stroke = PALETTE[kind]
    st = (f'rounded=1;arcSize=6;whiteSpace=wrap;html=1;{FONT}fontSize=20;fontStyle=1;fontColor=#1F2933;'
          f'fillColor={fill};strokeColor={stroke};strokeWidth=2;verticalAlign=top;align=center;spacingTop=10;')
    geo[cid] = (x, y, w, h)
    cells.append(f'<mxCell id="{cid}" value="{q(title)}" style="{st}" vertex="1" parent="1">'
                 f'<mxGeometry x="{x}" y="{y}" width="{w}" height="{h}" as="geometry"/></mxCell>')


def frac(cid, px, py):
    x, y, w, h = geo[cid]
    return round((px - x) / w, 4), round((py - y) / h, 4)


def arrow(src, tgt, p0, p1, pts=(), both=False):
    ex, ey = frac(src, *p0)
    nx, ny = frac(tgt, *p1)
    st = (f'html=1;rounded=0;endArrow=block;endFill=1;endSize=8;strokeColor={LINE};strokeWidth=2.5;'
          f'exitX={ex};exitY={ey};exitDx=0;exitDy=0;entryX={nx};entryY={ny};entryDx=0;entryDy=0;'
          + ('startArrow=block;startFill=1;startSize=8;' if both else ''))
    wp = ''
    if pts:
        wp = '<Array as="points">' + ''.join(f'<mxPoint x="{x}" y="{y}"/>' for x, y in pts) + '</Array>'
    eid = f'e{len(cells)}'
    cells.append(f'<mxCell id="{eid}" style="{st}" edge="1" parent="1" source="{src}" target="{tgt}">'
                 f'<mxGeometry relative="1" as="geometry">{wp}</mxGeometry></mxCell>')


# invisible page frame so the exported image is exactly 16:9
cells.append(f'<mxCell id="frame" value="" style="rounded=0;fillColor=#FFFFFF;strokeColor=none;" vertex="1" parent="1">'
             f'<mxGeometry x="0" y="0" width="{W}" height="{H}" as="geometry"/></mxCell>')
cells.append(f'<mxCell id="title" value="UAV Disaster-Rescue System" style="text;html=1;{FONT}fontSize=30;'
             f'fontStyle=1;fontColor=#1F2933;align=left;verticalAlign=middle;" vertex="1" parent="1">'
             f'<mxGeometry x="50" y="40" width="800" height="44" as="geometry"/></mxCell>')

MID = 580                                  # centre line of the primary flow
BH = 120                                   # main block height
top = MID - BH // 2

# ---- primary flow (left to right)
block('inst', 40, top, 190, BH, 'Mission Instruction', 'mission')
block('plan', 300, top, 190, BH, 'Mission<br>Planning', 'mission')

# ---- parallel branches
group('g_surv', 560, 270, 300, 270, 'Survivor Detection', 'percep')
block('cam', 590, 335, 240, 70, 'Camera', 'percep', size=19, white=True, bold=False)
block('vlm', 590, 450, 240, 70, 'VLM Confirmation<br><font style="font-size:15px" color="#6B7785">optional</font>',
      'percep', size=19, white=True, dashed=True, bold=False)
arrow('cam', 'vlm', (710, 405), (710, 450))

group('g_map', 560, 620, 300, 270, 'Mapping and<br>Debris Avoidance', 'percep')
block('slam', 590, 705, 240, 70, 'LiDAR / SLAM', 'percep', size=19, white=True, bold=False)
block('debr', 590, 795, 240, 70, 'Depth / Debris Detection', 'percep', size=19, white=True, bold=False)

block('nav', 930, top, 250, BH, 'Survivor Localization and Navigation', 'nav')
block('fc', 1250, top, 190, BH, 'Flight Control', 'flight')
block('px4', 1510, top, 190, BH, 'PX4 + Gazebo', 'px4')
block('rep', 1770, top, 190, BH, 'Survivor Report and Safe Landing', 'mission')

# ---- arrows
arrow('inst', 'plan', (230, MID), (300, MID))
arrow('plan', 'g_surv', (490, MID), (560, 405), pts=[(525, MID), (525, 405)])
arrow('plan', 'g_map', (490, MID), (560, 755), pts=[(525, MID), (525, 755)])
arrow('g_surv', 'nav', (860, 405), (930, MID), pts=[(895, 405), (895, MID)])
arrow('g_map', 'nav', (860, 755), (930, MID), pts=[(895, 755), (895, MID)])
arrow('nav', 'fc', (1180, MID), (1250, MID))
arrow('fc', 'px4', (1440, MID), (1510, MID), both=True)
arrow('px4', 'rep', (1700, MID), (1770, MID))
# debris-avoidance proposals go to the flight controller (the only PX4 commander), routed below the row
arrow('g_map', 'fc', (710, 890), (1345, top + BH), pts=[(710, 960), (1345, 960)])

xml = (f'<mxfile host="app.diagrams.net"><diagram id="uav-arch" name="UAV Architecture">'
       f'<mxGraphModel dx="{W}" dy="{H}" grid="0" gridSize="10" guides="1" tooltips="1" connect="1" arrows="1" '
       f'fold="1" page="1" pageScale="1" pageWidth="{W}" pageHeight="{H}" math="0" shadow="0" background="#FFFFFF">'
       f'<root><mxCell id="0"/><mxCell id="1" parent="0"/>' + ''.join(cells) + '</root></mxGraphModel></diagram></mxfile>')

drawio_path = os.path.join(HERE, 'architecture.drawio')
open(drawio_path, 'w', encoding='utf-8').write(xml)
print('wrote', drawio_path)

if '--png' in sys.argv:
    png_path = os.path.join(HERE, 'architecture.png')
    with tempfile.TemporaryDirectory() as tmp:
        viewer = os.path.join(tmp, 'viewer.js')
        req = urllib.request.Request('https://viewer.diagrams.net/js/viewer-static.min.js',
                                     headers={'User-Agent': 'Mozilla/5.0'})
        open(viewer, 'wb').write(urllib.request.urlopen(req, timeout=60).read())
        cfg = {'highlight': '#0000ff', 'nav': False, 'resize': False, 'toolbar': '', 'edit': None, 'xml': xml}
        page = os.path.join(tmp, 'view.html')
        open(page, 'w').write(
            f'<html><body style="margin:0;background:#fff;overflow:hidden"><style>html{{overflow:hidden}}</style><div class="mxgraph" style="width:{W}px;overflow:hidden" '
            f'data-mxgraph="{html.escape(json.dumps(cfg))}"></div><script src="viewer.js"></script></body></html>')
        subprocess.run(['google-chrome', '--headless=new', '--no-sandbox', '--disable-gpu',
                        '--force-device-scale-factor=2', f'--window-size={W},{H}',
                        '--virtual-time-budget=20000', f'--screenshot={png_path}', f'file://{page}'],
                       check=True, capture_output=True, timeout=120)
    print('wrote', png_path)
