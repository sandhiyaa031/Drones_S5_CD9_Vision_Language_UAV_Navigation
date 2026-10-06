#!/usr/bin/env bash
# Visible demonstration of the system as it stands: Gazebo GUI, autonomous
# takeoff / 3 m hover / landing, seeded falling debris, live debris detector.
# Opens two image windows: the downward camera and the detector's debug view.
#
# Usage: scripts/demo_live.sh [plan.json] [run_dir]
set -u
WS_DIR=$(cd "$(dirname "$0")/.." && pwd)
PLAN=${1:-$WS_DIR/results/sensor_study/plans/seed_11.json}
RUN=${2:-$WS_DIR/results/demo/$(date +%Y%m%d_%H%M%S)}
export DEMO_PLAN=$PLAN
# Image windows are opened first and given time to appear, so that their
# start-up load is over before the flight begins.
PRE='
setsid ros2 run rqt_image_view rqt_image_view /perception/debris/debug_image > /dev/null 2>&1 &
setsid ros2 run rqt_image_view rqt_image_view /camera/image_raw > /dev/null 2>&1 &
sleep 12
'
HOOK='
python3 $WS_DIR/scripts/perception/log_detections.py --out-dir $RUN_DIR --duration 58 &
python3 $WS_DIR/scripts/sensor_study/spawn_study.py --plan $DEMO_PLAN --out $RUN_DIR/releases.json --dwell 3.0
wait
'
HEADLESS=0 SETTLE_SECONDS=20 PERCEPTION=1 NVIDIA_OFFLOAD=1 CAMERA_BRIDGE=1 \
    PRE_HOOK="$PRE" DURING_HOOK="$HOOK" \
    "$WS_DIR/scripts/run_flight_trial.sh" "$RUN" hover
for pid in $(pgrep -x rqt_image_view); do kill "$pid" 2>/dev/null; done
