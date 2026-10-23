#!/usr/bin/env bash
# Isolation check: running the MuJoCo simulation must not change any Gazebo
# result file or PX4 stored parameter, and uses no Gazebo/ROS process.
# Usage (nothing else running): tests/check_isolation.sh
set -u
HERE=$(cd "$(dirname "$0")/.." && pwd)
WS=$(cd "$HERE/../.." && pwd)
sums() { (cd "$WS/results/disaster_gazebo" && find live demo -type f \( -name 'mission_report.json' -o -name 'verification.json' -o -name 'flight.ulg' \) 2>/dev/null | sort | xargs sha256sum; sha256sum "$WS"/../PX4-Autopilot/build/px4_sitl_default/rootfs/parameters.bson 2>/dev/null) | sha256sum | cut -c1-16; }
procs() { ps -eo comm,args | grep -E "^(px4|MicroXRCEAgent|ruby|gz|rviz2|flight_controll)" | grep -v grep | wc -l; }
before=$(sums); p0=$(procs)
echo "gazebo result checksum before: $before   sim/ROS processes before: $p0"
MUJOCO_GL=${MUJOCO_GL:-egl} "$HERE/.venv/bin/python" -m pytest "$HERE/tests" -q -x 2>&1 | tail -2
p1=$(procs); after=$(sums)
echo "gazebo result checksum after:  $after   sim/ROS processes after: $p1"
[ "$before" = "$after" ] && [ "$p0" = "0" ] && [ "$p1" = "0" ] && echo "ISOLATION OK" || echo "ISOLATION CHECK FAILED"
