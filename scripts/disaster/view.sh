#!/usr/bin/env bash
# Move the Gazebo window's camera in the urban disaster world.
# Usage: scripts/disaster/view.sh overview|collapse|courtyard|street|base|follow|free
set -u
pose() {  # x y z roll pitch yaw  -> gz move_to/pose
    q=$(python3 - "$4" "$5" "$6" <<'PY'
import math, sys
r, p, y = (float(v) for v in sys.argv[1:4])
cr, sr, cp, sp, cy, sy = (math.cos(r/2), math.sin(r/2), math.cos(p/2),
                          math.sin(p/2), math.cos(y/2), math.sin(y/2))
print(sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy,
      cr*cp*cy + sr*sp*sy)
PY
)
    set -- "$1" "$2" "$3" $q
    gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera \
        --reptype gz.msgs.Boolean --timeout 2000 \
        --req "pose: {position: {x: $1, y: $2, z: $3}, orientation: {x: $4, y: $5, z: $6, w: $7}}" > /dev/null
}
case "${1:-overview}" in
    overview)  pose -50 -45 48 0 0.55 0.78 ;;
    collapse)  pose -30 28 22 0 0.50 0.35 ;;
    courtyard) pose 58 -22 22 0 0.60 1.57 ;;
    street)    pose 33 8 14 0 0.28 1.57 ;;
    base)      pose 14 -14 9 0 0.35 0.78 ;;
    follow)    gz service -s /gui/follow --reqtype gz.msgs.StringMsg \
                   --reptype gz.msgs.Boolean --timeout 2000 \
                   --req 'data: "x500_rescue_0"' > /dev/null
               gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d \
                   --reptype gz.msgs.Boolean --timeout 2000 \
                   --req 'x: -6, y: -6, z: 4' > /dev/null ;;
    free)      gz service -s /gui/follow --reqtype gz.msgs.StringMsg \
                   --reptype gz.msgs.Boolean --timeout 2000 \
                   --req 'data: ""' > /dev/null ;;
    *) echo "usage: $0 overview|collapse|courtyard|street|base|follow|free"; exit 2 ;;
esac
