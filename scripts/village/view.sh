#!/usr/bin/env bash
# Move the Gazebo window's camera (village world).
# Usage: scripts/village/view.sh overview|site|base|street|follow|free
#   overview  the whole village from the south-west
#   site      close view of the collapsed building
#   base      the launch and landing pad
#   street    down the main road towards the disaster zone
#   follow    follow the UAV          free   stop following
set -u
pose() {  # x y z  qx qy qz qw
    gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera \
        --reptype gz.msgs.Boolean --timeout 2000 \
        --req "pose: {position: {x: $1, y: $2, z: $3}, orientation: {x: $4, y: $5, z: $6, w: $7}}" > /dev/null
}
case "${1:-overview}" in
    overview) pose -38 -52 62  -0.118 0.272 0.376 0.878 ;;
    site)     pose 12 18 30    -0.150 0.330 0.390 0.846 ;;
    base)     pose -14 -14 12  -0.110 0.250 0.380 0.884 ;;
    street)   pose 12 -18 9    -0.060 0.090 0.700 0.706 ;;
    follow)   gz service -s /gui/follow --reqtype gz.msgs.StringMsg \
                  --reptype gz.msgs.Boolean --timeout 2000 \
                  --req 'data: "x500_rescue_0"' > /dev/null
              gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d \
                  --reptype gz.msgs.Boolean --timeout 2000 \
                  --req 'x: -9, y: -6, z: 7' > /dev/null ;;
    free)     gz service -s /gui/follow --reqtype gz.msgs.StringMsg \
                  --reptype gz.msgs.Boolean --timeout 2000 \
                  --req 'data: ""' > /dev/null ;;
    *) echo "usage: $0 overview|site|base|street|follow|free"; exit 2 ;;
esac
