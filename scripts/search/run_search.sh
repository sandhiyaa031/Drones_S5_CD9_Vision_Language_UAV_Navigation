#!/usr/bin/env bash
# One Phase 9 search flight: take off, cover the search area, inspect located
# candidates, return and land.
#
# Usage: scripts/search/run_search.sh <world> [tag]
#   world: base | outdoor | multi | none | distractors | occluded | rubble
# Output: ${RESULTS_ROOT:-results/phase9/live}/<world>[_tag]/
# Environment:
#   AREA="nmin,nmax,emin,emax"   search rectangle relative to takeoff [m]
#                                (default: building only for base/occluded/
#                                rubble, building + yard to the north else)
#   ALT=5.0                      flight altitude [m]
#   DEBRIS=1                     also run the debris chain with avoidance and
#                                drop the boxes of results/phase6/plans/
#                                C_direct.json on the UAV during the search
#   LIGHT=1                      record simulator poses only, no camera frames
#   PLANNER_ARGS                 extra ROS arguments for the planner
#   NVIDIA_OFFLOAD=0|1, HEADLESS=0|1, VLM_ENDPOINT / VLM_MODEL as in Phase 7
set -u
WORLD_NAME=${1:?world}
TAG=${2:-}
WS_DIR=$(cd "$(dirname "$0")/../.." && pwd)
OUT=${RESULTS_ROOT:-$WS_DIR/results/phase9/live}/$WORLD_NAME${TAG:+_$TAG}
VARIANTS=$WS_DIR/src/uav_autonomy/worlds/survivor_variants
ALT=${ALT:-5.0}
case "$WORLD_NAME" in
    base|occluded|rubble) DEFAULT_AREA="-4,4,-4,4" ;;
    *)                    DEFAULT_AREA="-4,12,-4,4" ;;
esac
AREA=${AREA:-$DEFAULT_AREA}
IFS=, read -r N0 N1 E0 E1 <<< "$AREA"
# ROS parameters are typed: make every bound a float
for v in N0 N1 E0 E1; do printf -v "$v" '%.2f' "${!v}"; done
[ "$WORLD_NAME" != "base" ] && export WORLD_SDF=$VARIANTS/$WORLD_NAME.sdf
LOG_SECONDS=${LOG_SECONDS:-230}

# LIGHT=1: record simulator poses only (no camera frames), for a loaded machine
FRAMES="--frame-every 5"
[ "${LIGHT:-0}" = "1" ] && FRAMES="--poses-only"
HOOK="python3 \$WS_DIR/scripts/sensor_study/record.py --out-dir \$RUN_DIR --duration $LOG_SECONDS $FRAMES & python3 \$WS_DIR/scripts/perception/log_survivor.py --out-dir \$RUN_DIR --duration $LOG_SECONDS & python3 \$WS_DIR/scripts/perception/log_locations.py --out-dir \$RUN_DIR --duration $LOG_SECONDS & python3 \$WS_DIR/scripts/perception/log_search.py --out-dir \$RUN_DIR --duration $LOG_SECONDS &"
PERCEPTION=0
AVOIDANCE=off
if [ "${DEBRIS:-0}" = "1" ]; then
    PERCEPTION=1
    AVOIDANCE=on
    HOOK="$HOOK python3 \$WS_DIR/scripts/perception/log_detections.py --out-dir \$RUN_DIR/debris --duration $LOG_SECONDS --save-debug-every 1000 & python3 \$WS_DIR/scripts/sensor_study/spawn_study.py --plan ${PLAN:-$WS_DIR/results/phase6/plans/C_direct.json} --out \$RUN_DIR/releases.json --start-altitude $(awk "BEGIN{print $ALT-0.2}") --settle ${DEBRIS_SETTLE:-6} --max-duration 60;"
fi
HOOK="$HOOK wait"

rm -rf "$OUT"
mkdir -p "$(dirname "$OUT")"
NVIDIA_OFFLOAD=${NVIDIA_OFFLOAD:-0} CAMERA_BRIDGE=1 SURVIVOR=1 LOCALIZATION=1 \
    SEARCH_PLANNER=1 PERCEPTION=$PERCEPTION AVOIDANCE=$AVOIDANCE \
    SEARCH_ARGS="--ros-args -p area_north_min:=$N0 -p area_north_max:=$N1 -p area_east_min:=$E0 -p area_east_max:=$E1 ${PLANNER_ARGS:-}" \
    DURING_HOOK="$HOOK" \
    CONTROLLER_ARGS="-p search_mission:=true -p target_altitude:=$ALT -p arrival_max_speed:=0.3 ${EXTRA_CONTROLLER_ARGS:-}" \
    VERIFY_ARGS="--target-alt $ALT --hold-seconds 15 --max-range 25" \
    FLIGHT_TIMEOUT=${FLIGHT_TIMEOUT:-330} \
    timeout $((${FLIGHT_TIMEOUT:-330} + 90)) "$WS_DIR/scripts/run_flight_trial.sh" "$OUT" hover \
    > "$OUT.out" 2>&1
echo "exit=$?" >> "$OUT.out"
echo "scenario=search world=$WORLD_NAME area=$AREA altitude=$ALT debris=${DEBRIS:-0} vlm_endpoint=${VLM_ENDPOINT:-none} vlm_model=${VLM_MODEL:-none}" >> "$OUT/trial_info.txt"
grep -h "controller outcome" "$OUT.out"
grep -h "Search started\|INSPECTING\|COMPLETE" "$OUT/search_planner.log" | cut -c1-220 | head -14
