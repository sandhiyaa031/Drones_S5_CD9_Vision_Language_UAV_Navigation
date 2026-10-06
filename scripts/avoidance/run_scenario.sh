#!/usr/bin/env bash
# One Phase 6 scenario flight, with the avoidance planner either acting
# ("avoid") or running disabled ("baseline": it logs, the UAV never reacts).
# The scenario (releases, mission, altitude) is identical in both variants.
#
# Usage: scripts/avoidance/run_scenario.sh <scenario> <baseline|avoid> [tag]
# Output: results/phase6/<variant>/<scenario>[_tag]/
# Extra planner arguments: AVOIDANCE_ARGS="-p name:=value ..."
set -u
SCENARIO=${1:?scenario}
VARIANT=${2:?baseline|avoid}
TAG=${3:-}
WS_DIR=$(cd "$(dirname "$0")/../.." && pwd)
PLANS=${PLANS:-$WS_DIR/results/phase6/plans}
RESULTS_ROOT=${RESULTS_ROOT:-$WS_DIR/results/phase6}
OUT=$RESULTS_ROOT/$VARIANT/$SCENARIO${TAG:+_$TAG}

MODE=hover
ALT=5.0
PLAN=$SCENARIO
START_ALT=4.8
SETTLE=1.5
HOLD=30
CTRL="-p target_altitude:=5.0"
WP=""
case "$SCENARIO" in
    A_none)      PLAN="" ;;
    G_climbing)  START_ALT=3.4; SETTLE=0 ;;
    H_transit)   MODE=waypoint; SETTLE=0; WP="6,0;3,5.2;0,0"
                 CTRL="$CTRL -p arrival_max_speed:=0.3 -p waypoint_1_north:=6.0 -p waypoint_1_east:=0.0 -p waypoint_2_north:=3.0 -p waypoint_2_east:=5.2" ;;
    I_search)    MODE=waypoint; SETTLE=0; WP="3,0;3,3;0,0"
                 CTRL="$CTRL -p arrival_max_speed:=0.3 -p waypoint_1_north:=3.0 -p waypoint_1_east:=0.0 -p waypoint_2_north:=3.0 -p waypoint_2_east:=3.0" ;;
    J_shaft)     ALT=3.0; START_ALT=2.85; CTRL="" ;;
    seed_*)      HOLD=60; CTRL="$CTRL -p hold_seconds:=60.0" ;;
    F2_lateral)  HOLD=45; CTRL="$CTRL -p hold_seconds:=45.0" ;;
    T_*)         # cruise at a capped speed on 12 m legs: T_<speed*10>
                 SPEED=$(echo "${SCENARIO#T_}" | awk '{printf "%.1f", $1/10}')
                 MODE=waypoint; SETTLE=0; WP="18,0;9,15.6;0,0"; MAX_RANGE=25
                 CTRL="$CTRL -p transit_speed_limit:=$SPEED -p arrival_max_speed:=0.3 -p waypoint_1_north:=18.0 -p waypoint_1_east:=0.0 -p waypoint_2_north:=9.0 -p waypoint_2_east:=15.6" ;;
    GAP_*)       # GAP_<gap ms>_<delay ms>: direct hits with a depth gap
                 GAP_MS=$(echo "$SCENARIO" | cut -d_ -f2); DELAY_MS=$(echo "$SCENARIO" | cut -d_ -f3)
                 PLAN=C_direct
                 export DETECTOR_ARGS="--ros-args -r /depth_up/image_raw:=/depth_up/image_gated"
                 GATE="python3 \$WS_DIR/scripts/avoidance/depth_gate.py --run-dir \$RUN_DIR --gap $(awk "BEGIN{print $GAP_MS/1000}") --delay $(awk "BEGIN{print $DELAY_MS/1000}") --duration 75 &" ;;
esac
LOG_SECONDS=$((HOLD + 50))
RELEASE_WINDOW=$((HOLD - 7))

HOOK="${GATE:-} python3 \$WS_DIR/scripts/sensor_study/record.py --out-dir \$RUN_DIR --duration $LOG_SECONDS --poses-only & python3 \$WS_DIR/scripts/perception/log_detections.py --out-dir \$RUN_DIR --duration $LOG_SECONDS --save-debug-every 0 &"
if [ -n "$PLAN" ]; then
    HOOK="$HOOK python3 \$WS_DIR/scripts/sensor_study/spawn_study.py --plan $PLANS/$PLAN.json --out \$RUN_DIR/releases.json --start-altitude $START_ALT --settle $SETTLE --max-duration $RELEASE_WINDOW;"
fi
HOOK="$HOOK wait"

AVOID=on
[ "$VARIANT" = "baseline" ] && AVOID=baseline
rm -rf "$OUT"
mkdir -p "$(dirname "$OUT")"
PERCEPTION=1 NVIDIA_OFFLOAD=1 CAMERA_BRIDGE=1 AVOIDANCE=$AVOID \
    DURING_HOOK="$HOOK" CONTROLLER_ARGS="$CTRL" WAYPOINTS="$WP" \
    VERIFY_ARGS="--target-alt $ALT --hold-seconds $HOLD --max-range ${MAX_RANGE:-15}" \
    FLIGHT_TIMEOUT=$((HOLD + 150)) \
    timeout $((HOLD + 230)) "$WS_DIR/scripts/run_flight_trial.sh" "$OUT" $MODE \
    > "$OUT.out" 2>&1
echo "exit=$?" >> "$OUT.out"
{
    echo "scenario=$SCENARIO variant=$VARIANT mode=$MODE altitude=$ALT plan=$PLAN"
} >> "$OUT/trial_info.txt"
grep -h "controller outcome\|\"passed\"" "$OUT.out"
grep -h "FAILSAFE\|AVOIDANCE\|refused\|timed out" "$OUT/flight_controller.log" | cut -c1-230 | head -30
grep -h " -> " "$OUT/debris_avoidance.log" | cut -c1-200 | head -40
