#!/usr/bin/env bash
# One Phase 7 survivor-perception flight.
#
# Usage: scripts/survivor/run_scenario.sh <scenario> [tag]
# Output: results/phase7/live/<scenario>[_tag]/
# The VLM is used if VLM_ENDPOINT and VLM_MODEL are set in the environment;
# NO_VLM=1 clears them for this run (honest "unavailable" behaviour).
# HOLD_OVERRIDE=<seconds> lengthens the hover (slow, CPU-only VLM).
# LOCALIZATION=1 also starts and records the Phase 8 localization node.
# RESULTS_ROOT=<dir> writes there instead of results/phase7/live;
# NVIDIA_OFFLOAD=0 renders on the integrated GPU.
set -u
SCENARIO=${1:?scenario}
TAG=${2:-}
WS_DIR=$(cd "$(dirname "$0")/../.." && pwd)
OUT=${RESULTS_ROOT:-$WS_DIR/results/phase7/live}/$SCENARIO${TAG:+_$TAG}
VARIANTS=$WS_DIR/src/uav_autonomy/worlds/survivor_variants

WORLD_NAME=base
MODE=hover
ALT=5.0
HOLD=20
CTRL=""
WP=""
PLAN=""
ROUTE="-p arrival_max_speed:=0.3 -p waypoint_1_north:=13.0 -p waypoint_1_east:=0.0 -p waypoint_2_north:=9.0 -p waypoint_2_east:=3.0"
case "$SCENARIO" in
    A_clear)        ;;
    B_occluded)     WORLD_NAME=occluded ;;
    C_rubble)       WORLD_NAME=rubble ;;
    D_altitude)     ALT=12.0; HOLD=8 ;;
    E_moving_20)    WORLD_NAME=outdoor; MODE=waypoint; WP="13,0;9,3;0,0"
                    CTRL="$ROUTE -p transit_speed_limit:=2.0" ;;
    E_moving_29)    WORLD_NAME=outdoor; MODE=waypoint; WP="13,0;9,3;0,0"
                    CTRL="$ROUTE -p transit_speed_limit:=2.9" ;;
    F_none)         WORLD_NAME=none; MODE=waypoint; WP="13,0;9,3;0,0"
                    CTRL="$ROUTE -p transit_speed_limit:=2.9" ;;
    F_none_debris)  WORLD_NAME=none; HOLD=30
                    PLAN=$WS_DIR/results/phase6/plans/B_safe.json ;;
    G_distractors)  WORLD_NAME=distractors ;;
    G_distractors_route) WORLD_NAME=distractors; MODE=waypoint; WP="13,0;9,3;0,0"
                    CTRL="$ROUTE -p transit_speed_limit:=2.0" ;;
    H_multi)        WORLD_NAME=multi; MODE=waypoint; WP="13,0;9,3;0,0"
                    CTRL="$ROUTE -p transit_speed_limit:=2.0" ;;
    *) echo "unknown scenario $SCENARIO" >&2; exit 2 ;;
esac
HOLD=${HOLD_OVERRIDE:-$HOLD}
CTRL="$CTRL -p target_altitude:=$ALT -p hold_seconds:=$HOLD.0"
[ "$WORLD_NAME" != "base" ] && export WORLD_SDF=$VARIANTS/$WORLD_NAME.sdf
if [ "${NO_VLM:-0}" = "1" ]; then
    unset VLM_ENDPOINT VLM_MODEL
fi
LOG_SECONDS=$((HOLD + 75))

HOOK="python3 \$WS_DIR/scripts/sensor_study/record.py --out-dir \$RUN_DIR --duration $LOG_SECONDS --frame-every 3 & python3 \$WS_DIR/scripts/perception/log_survivor.py --out-dir \$RUN_DIR --duration $LOG_SECONDS &"
if [ -n "$PLAN" ]; then
    HOOK="$HOOK python3 \$WS_DIR/scripts/sensor_study/spawn_study.py --plan $PLAN --out \$RUN_DIR/releases.json --start-altitude $(awk "BEGIN{print $ALT-0.2}") --max-duration $((HOLD - 7));"
fi
if [ "${LOCALIZATION:-0}" = "1" ]; then
    HOOK="$HOOK python3 \$WS_DIR/scripts/perception/log_locations.py --out-dir \$RUN_DIR --duration $LOG_SECONDS &"
fi
HOOK="$HOOK wait"

rm -rf "$OUT"
mkdir -p "$(dirname "$OUT")"
NVIDIA_OFFLOAD=${NVIDIA_OFFLOAD:-1} CAMERA_BRIDGE=1 SURVIVOR=1 \
    DURING_HOOK="$HOOK" CONTROLLER_ARGS="$CTRL" WAYPOINTS="$WP" \
    VERIFY_ARGS="--target-alt $ALT --hold-seconds $HOLD --max-range 20" \
    FLIGHT_TIMEOUT=$((HOLD + 150)) \
    timeout $((HOLD + 230)) "$WS_DIR/scripts/run_flight_trial.sh" "$OUT" $MODE \
    > "$OUT.out" 2>&1
echo "exit=$?" >> "$OUT.out"
echo "scenario=$SCENARIO world=$WORLD_NAME mode=$MODE altitude=$ALT vlm_endpoint=${VLM_ENDPOINT:-none} vlm_model=${VLM_MODEL:-none}" >> "$OUT/trial_info.txt"
grep -h "controller outcome" "$OUT.out"
grep -h "VLM\|ready\|rejected" "$OUT/survivor_perception.log" | cut -c1-200 | head -12
tail -1 "$OUT/during_hook.log"
