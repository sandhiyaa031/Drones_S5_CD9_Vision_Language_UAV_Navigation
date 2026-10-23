#!/usr/bin/env bash
# Render still views of a disaster-zone world with injected static cameras and
# save PNGs. Uses a temporary copy of the world (the world file is untouched).
# Usage: scripts/disaster/render_views.sh [scenario] [out_dir]
#   defaults: urban_C  results/disaster_gazebo/screenshots/<scenario>
# Rendering uses whatever GPU Gazebo gets (Intel if NVIDIA is unavailable).
set -u
WS_DIR=$(cd "$(dirname "$0")/../.." && pwd)
SC=${1:-urban_C}
OUT=${2:-$WS_DIR/results/disaster_gazebo/screenshots/$SC}
SRC=$WS_DIR/src/uav_autonomy/worlds/disaster/$SC.sdf
mkdir -p "$OUT"
TMP=$(mktemp -d)
set +u; source /opt/ros/jazzy/setup.bash; set -u
# name x y z roll pitch yaw  (pitch > 0 looks down; yaw 0 = east)
VIEWS=(
 "overview_sw -55 -45 52 0 0.55 0.72"
 "overview_se 120 -40 52 0 0.55 2.40"
 "block_b1 -12 30 24 0 0.50 0.95"
 "collapse_close 40 40 14 0 0.42 1.10"
 "courtyard 58 -14 20 0 0.62 1.57"
 "street_blocked 33 44 16 0 0.30 1.57"
 "top_down_site 18 58 60 0 1.45 0.0"
 "base 14 -14 9 0 0.35 0.78"
)
python3 - "$SRC" "$TMP/world.sdf" "${VIEWS[@]}" <<'PY'
import sys
src, dst, *views = sys.argv[1:]
text = open(src).read()
cams = ''
for v in views:
    n, x, y, z, r, p, yw = v.split()
    cams += (f'<model name="cam_{n}"><static>true</static><pose>{x} {y} {z} '
             f'{r} {p} {yw}</pose><link name="l"><sensor name="c" '
             f'type="camera"><topic>/dz/{n}</topic><update_rate>1</update_rate>'
             '<camera><horizontal_fov>1.2</horizontal_fov><image><width>960'
             '</width><height>600</height></image><clip><near>0.2</near>'
             '<far>400</far></clip></camera></sensor></link></model>\n')
plugin = ('<plugin filename="gz-sim-sensors-system" '
          'name="gz::sim::systems::Sensors"><render_engine>ogre2'
          '</render_engine></plugin>')
text = text.replace('<physics ', plugin + '<physics ', 1)
text = text.replace('  </world>', cams + '  </world>')
open(dst, 'w').write(text)
PY
export GZ_SIM_RESOURCE_PATH=$WS_DIR/src/uav_autonomy/models_disaster
gz sim -r -s "$TMP/world.sdf" > "$OUT/gazebo_render.log" 2>&1 &
GZ=$!
sleep "${RENDER_WAIT:-25}"
for v in "${VIEWS[@]}"; do
    n=${v%% *}
    python3 "$WS_DIR/scripts/disaster/gz_snapshot.py" /dz/$n "$OUT/$n.png" --timeout 40
done
kill $GZ 2>/dev/null; sleep 2; kill -9 $GZ 2>/dev/null
rm -rf "$TMP"
