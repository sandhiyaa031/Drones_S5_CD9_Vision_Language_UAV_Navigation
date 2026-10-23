#!/usr/bin/env bash
# One Phase 10 mission: the mission manager checks the system, starts the
# flight, supervises the search and writes the survivor report.
#
# Usage: scripts/mission/run_mission.sh <mission.json> <world> [tag]
#   world: base | outdoor | multi | none | distractors | occluded | rubble
# Output: ${RESULTS_ROOT:-results/phase10/live}/<mission name>_<world>[_tag]/
#         with mission_report.json
# Environment:
#   DEBRIS=1               run the debris chain with avoidance and drop the
#                          boxes of results/phase6/plans/C_direct.json
#   LIGHT=1                record simulator poses only (loaded machine)
#   KILL_PERCEPTION_AT=s   fault injection: stop the survivor perception
#                          node s seconds after the controller starts
#   NO_PLANNER=1           fault injection: do not start the search planner
#   KILL_AT=s KILL_COMM=n  fault injection: stop process n (15-character
#                          `comm` name, e.g. search_planner, mission_manager,
#                          survivor_locali, debris_avoidanc, parameter_bridg,
#                          MicroXRCEAgent, flight_controll) after s seconds
#   NVIDIA_OFFLOAD=0|1, HEADLESS=0|1, VLM_ENDPOINT / VLM_MODEL as before
set -u
MISSION=$(realpath "${1:?mission file}")
WORLD_NAME=${2:?world}
TAG=${3:-}
WS_DIR=$(cd "$(dirname "$0")/../.." && pwd)
VARIANTS=$WS_DIR/src/uav_autonomy/worlds/survivor_variants
read -r NAME N0 N1 E0 E1 ALT BUDGET PRIO_N PRIO_E < <(python3 - "$MISSION" <<'PY'
import json, sys
m = json.load(open(sys.argv[1]))
a = m.get('search_area', {'north': [-4, 4], 'east': [-4, 4]})
print(m.get('name', 'mission'), *('%.2f' % float(v) for v in (
    a['north'][0], a['north'][1], a['east'][0], a['east'][1],
    m.get('altitude_m', 5.0), m.get('time_budget_s', 240),
    m.get('priority_north', float('nan')),
    m.get('priority_east', float('nan')))))
PY
)
OUT=${RESULTS_ROOT:-$WS_DIR/results/phase10/live}/${NAME}_$WORLD_NAME${TAG:+_$TAG}
[ "$WORLD_NAME" != "base" ] && export WORLD_SDF=$VARIANTS/$WORLD_NAME.sdf
VILLAGE=0
SEARCH_REMAP=""
EXTRA_LOGGERS=""
VERIFY_RANGE=25
case "$WORLD_NAME" in village_*|urban_*)
    # Village missions: lidar model, SLAM, map-aware navigator, safe return.
    VILLAGE=1
    VDIR=village_fast
    [ "${HEADLESS:-1}" = "0" ] && VDIR=village      # with shadows, for viewing
    case "$WORLD_NAME" in urban_*) VDIR=disaster_fast
        export SLAM_PARAMS=${SLAM_PARAMS:-$WS_DIR/src/uav_autonomy/config/slam_toolbox_urban.yaml}
        # PX4's status stream is ~2 Hz of sim time; at the urban world's real-time
        # factor (about 0.5 on Intel graphics) the 1.0 s default would trip the
        # stale-telemetry failsafe without any link loss. Position stays at 1.0 s.
        EXTRA_CONTROLLER_ARGS="${EXTRA_CONTROLLER_ARGS:-} -p status_timeout_s:=${STATUS_TIMEOUT_S:-3.0} -p position_timeout_s:=${POSITION_TIMEOUT_S:-1.0}"
        # every return is routed on the SLAM map at the search altitude: a vertical
        # climb is not safe under slabs or lintels (two impacts, urban_B_onebody and
        # urban_C_demo1); the climb to 18 m only happens near the base
        EXTRA_CONTROLLER_ARGS="$EXTRA_CONTROLLER_ARGS -p map_return:=true"
        [ "${HEADLESS:-1}" = "0" ] && VDIR=disaster ;; esac
    export WORLD_SDF=$WS_DIR/src/uav_autonomy/worlds/$VDIR/$WORLD_NAME.sdf
    export MODELS_DIR=$WS_DIR/src/uav_autonomy/models_slam
    export EXTRA_RESOURCE_PATH=$WS_DIR/src/uav_autonomy/models_slam:$WS_DIR/src/uav_autonomy/models:$WS_DIR/src/uav_autonomy/models_disaster
    export BRIDGE_CONFIG=$WS_DIR/src/uav_autonomy/config/bridge_village.yaml
    export SLAM=1 NAVIGATOR=1
    SEARCH_REMAP="-r /planning/search/goal:=/planning/search/goal_raw"
    EXTRA_CONTROLLER_ARGS="${EXTRA_CONTROLLER_ARGS:-} -p search_max_range:=150.0 -p return_altitude:=${RETURN_ALTITUDE:-18.0} -p search_max_seconds:=$(awk "BEGIN{printf \"%.1f\", $BUDGET+180}")"
    EXTRA_LOGGERS="python3 \$WS_DIR/scripts/perception/log_slam.py --out-dir \$RUN_DIR --duration \$LOG_SECONDS &"
    VERIFY_RANGE=150
    VERIFY_CEILING="--max-alt ${RETURN_ALTITUDE:-18.0}"
    LOG_SECONDS=${LOG_SECONDS:-$(awk "BEGIN{printf \"%d\", $BUDGET+260}")}
    FLIGHT_TIMEOUT=${FLIGHT_TIMEOUT:-$(awk "BEGIN{printf \"%d\", $BUDGET+330}")}
    ;;
esac
LOG_SECONDS=${LOG_SECONDS:-250}
FRAMES="--frame-every 5"
[ "${LIGHT:-0}" = "1" ] && FRAMES="--poses-only"

export LOG_SECONDS
HOOK="$EXTRA_LOGGERS python3 \$WS_DIR/scripts/sensor_study/record.py --out-dir \$RUN_DIR --duration $LOG_SECONDS $FRAMES & python3 \$WS_DIR/scripts/perception/log_survivor.py --out-dir \$RUN_DIR --duration $LOG_SECONDS & python3 \$WS_DIR/scripts/perception/log_locations.py --out-dir \$RUN_DIR --duration $LOG_SECONDS & python3 \$WS_DIR/scripts/perception/log_search.py --out-dir \$RUN_DIR --duration $LOG_SECONDS & python3 \$WS_DIR/scripts/perception/log_mission.py --out-dir \$RUN_DIR --duration $LOG_SECONDS &"
PERCEPTION=0
AVOIDANCE=off
if [ "${DEBRIS:-0}" = "1" ]; then
    PERCEPTION=1
    AVOIDANCE=on
    HOOK="$HOOK python3 \$WS_DIR/scripts/perception/log_detections.py --out-dir \$RUN_DIR/debris --duration $LOG_SECONDS --save-debug-every 1000 & python3 \$WS_DIR/scripts/sensor_study/spawn_study.py --plan ${PLAN:-$WS_DIR/results/phase6/plans/C_direct.json} --out \$RUN_DIR/releases.json --start-altitude $(awk "BEGIN{print $ALT-0.2}") --settle ${DEBRIS_SETTLE:-6} --max-duration 60;"
fi
if [ -n "${KILL_AT:-}" ] && [ -n "${KILL_COMM:-}" ]; then
    # generic fault injection: stop one process (name as shown by `ps -o comm`)
    HOOK="$HOOK (sleep $KILL_AT; pkill -x $KILL_COMM; echo fault-injected: $KILL_COMM stopped) &"
fi
if [ -n "${KILL_PERCEPTION_AT:-}" ]; then
    HOOK="$HOOK (sleep $KILL_PERCEPTION_AT; pkill -x survivor_percep; echo fault-injected: survivor perception stopped) &"
fi
# optional extra background command(s) for diagnostics, e.g. a gap monitor
[ -n "${EXTRA_HOOK:-}" ] && HOOK="$HOOK $EXTRA_HOOK"
HOOK="$HOOK wait"
PLANNER=1
[ "${NO_PLANNER:-0}" = "1" ] && PLANNER=0

rm -rf "$OUT"
mkdir -p "$(dirname "$OUT")"
NVIDIA_OFFLOAD=${NVIDIA_OFFLOAD:-0} CAMERA_BRIDGE=1 SURVIVOR=1 LOCALIZATION=1 \
    SEARCH_PLANNER=$PLANNER PERCEPTION=$PERCEPTION AVOIDANCE=$AVOIDANCE \
    MISSION_FILE="$MISSION" \
    SEARCH_ARGS="--ros-args $SEARCH_REMAP -p start_near_north:=$PRIO_N -p start_near_east:=$PRIO_E -p area_north_min:=$N0 -p area_north_max:=$N1 -p area_east_min:=$E0 -p area_east_max:=$E1 ${PLANNER_ARGS:-}" \
    DURING_HOOK="$HOOK" \
    CONTROLLER_ARGS="-p search_mission:=true -p mission_managed:=true -p target_altitude:=$ALT -p arrival_max_speed:=0.3 ${EXTRA_CONTROLLER_ARGS:-}" \
    VERIFY_ARGS="--target-alt $ALT --hold-seconds ${VERIFY_HOLD:-15} --max-range $VERIFY_RANGE ${VERIFY_CEILING:-}" \
    FLIGHT_TIMEOUT=${FLIGHT_TIMEOUT:-360} \
    timeout $((${FLIGHT_TIMEOUT:-360} + 90)) "$WS_DIR/scripts/run_flight_trial.sh" "$OUT" hover \
    > "$OUT.out" 2>&1
echo "exit=$?" >> "$OUT.out"
echo "scenario=mission mission=$NAME world=$WORLD_NAME debris=${DEBRIS:-0} kill_perception_at=${KILL_PERCEPTION_AT:-none} kill=${KILL_COMM:-none}@${KILL_AT:-none} no_planner=${NO_PLANNER:-0} vlm_endpoint=${VLM_ENDPOINT:-none} vlm_model=${VLM_MODEL:-none}" >> "$OUT/trial_info.txt"
cp "$MISSION" "$OUT/mission.json"
grep -h "controller outcome" "$OUT.out"
grep -h "MISSION" "$OUT/mission_manager.log" | cut -c1-230
