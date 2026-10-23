#!/usr/bin/env bash
# Phase 13 (repeated integrated missions) and Phase 14 (fault injection).
# Usage: scripts/mission/run_phase13_14.sh [13|14|all]
# Results: results/phase13/live, results/phase14/live. About 3.5 min per
# flight; 12 + 8 flights.
set -u
WS_DIR=$(cd "$(dirname "$0")/../.." && pwd)
cd "$WS_DIR" || exit 1
WHAT=${1:-all}
M=$WS_DIR/results/phase13/missions
mkdir -p "$M" results/phase13/live results/phase14/live
python3 scripts/survivor/make_worlds.py > /dev/null
# the missions come from instructions, through the deterministic interpreter
python3 scripts/mission/interpret_mission.py "Search the whole site for survivors and avoid falling debris." --interpreter rules --yes --out "$M/site_protected.json" --record "$M/site_protected.interpretation.json" > /dev/null
python3 scripts/mission/interpret_mission.py "Search the collapsed building for survivors and avoid falling debris." --interpreter rules --yes --out "$M/building_protected.json" --record "$M/building_protected.interpretation.json" > /dev/null
python3 scripts/mission/interpret_mission.py "Search the whole site for survivors." --interpreter rules --yes --out "$M/site.json" --record "$M/site.interpretation.json" > /dev/null

if [ "$WHAT" = "13" ] || [ "$WHAT" = "all" ]; then
    for i in 01 02 03 04 05 06 07 08; do
        RESULTS_ROOT=$WS_DIR/results/phase13/live DEBRIS=1 LIGHT=1 \
            scripts/mission/run_mission.sh "$M/site_protected.json" multi r$i | grep "controller outcome" | sed "s/^/site r$i: /"
    done
    for i in 01 02 03 04; do
        RESULTS_ROOT=$WS_DIR/results/phase13/live DEBRIS=1 LIGHT=1 \
            scripts/mission/run_mission.sh "$M/building_protected.json" base r$i | grep "controller outcome" | sed "s/^/building r$i: /"
    done
fi

if [ "$WHAT" = "14" ] || [ "$WHAT" = "all" ]; then
    fault() {   # name, process, mission file, extra environment
        local name=$1 comm=$2 mission=$3; shift 3
        env RESULTS_ROOT=$WS_DIR/results/phase14/live LIGHT=1 KILL_AT=40 \
            KILL_COMM=$comm "$@" scripts/mission/run_mission.sh "$mission" multi "$name" | grep "controller outcome" | sed "s/^/$name: /"
    }
    fault kill_search_planner search_planner "$M/site.json"
    fault kill_mission_manager mission_manager "$M/site.json"
    fault kill_localization survivor_locali "$M/site.json"
    fault kill_camera_bridge parameter_bridg "$M/site.json"
    fault kill_avoidance_planner debris_avoidanc "$M/site_protected.json" DEBRIS=1
    fault kill_debris_detector debris_depth_de "$M/site_protected.json" DEBRIS=1
    fault kill_xrce_agent MicroXRCEAgent "$M/site.json" FLIGHT_TIMEOUT=150
    fault kill_flight_controller flight_controll "$M/site.json" FLIGHT_TIMEOUT=150
fi
echo BATCH_DONE
