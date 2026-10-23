#!/usr/bin/env bash
# Village evaluation batch (headless). About 6 minutes per flight.
# Usage: scripts/village/run_village_batch.sh [core|vlm|faults|slam|all|v2]
# Results: results/village/live/, one folder per flight; nothing is deleted.
# The VLM group needs the NVIDIA GPU and a running Ollama server with
# gemma3:4b; it is skipped (and says so) otherwise.
set -u
WS_DIR=$(cd "$(dirname "$0")/../.." && pwd)
cd "$WS_DIR" || exit 1
WHAT=${1:-all}
export RESULTS_ROOT=$WS_DIR/results/village/live NVIDIA_OFFLOAD=${NVIDIA_OFFLOAD:-1} LIGHT=1
M=$WS_DIR/missions
python3 scripts/village/make_village.py > /dev/null
interp() { python3 scripts/mission/interpret_mission.py "$1" --interpreter rules --places $M/places_village.json --yes --out "$2" > /dev/null; }
interp "Search the collapsed building for survivors." $M/village_collapsed_building.json
interp "Search the collapsed building for survivors and avoid falling debris." $M/village_collapsed_building_protected.json
interp "Search the village for survivors, prioritize the collapsed building, avoid falling debris, report survivor locations, and return to base." $M/village_headline.json
interp "Search the collapsed building for survivors, only report confirmed ones." $M/village_collapsed_building_confirmed.json
interp "Search the village for survivors, prioritize the collapsed building, only report confirmed ones, and avoid falling debris." $M/village_headline_confirmed.json
fly() { echo "=== $*"; "$@" 2>&1 | grep "controller outcome\|MISSION R" | cut -c1-200; }

if [ "$WHAT" = "core" ] || [ "$WHAT" = "all" ]; then
    fly scripts/mission/run_mission.sh $M/village_collapsed_building.json village_A r1
    fly scripts/mission/run_mission.sh $M/village_collapsed_building.json village_B r1
    fly scripts/mission/run_mission.sh $M/village_collapsed_building.json village_B r2
    fly scripts/mission/run_mission.sh $M/village_collapsed_building.json village_B2 r1
    fly env DEBRIS=1 DEBRIS_SETTLE=75 scripts/mission/run_mission.sh $M/village_collapsed_building_protected.json village_B debris
fi
if [ "$WHAT" = "vlm" ] || [ "$WHAT" = "all" ]; then
    if nvidia-smi > /dev/null 2>&1 && pgrep -x ollama > /dev/null; then
        "$HOME/ollama/bin/ollama" run gemma3:4b "Reply with the single word ready." > /dev/null 2>&1
        export VLM_ENDPOINT=http://127.0.0.1:11434/v1 VLM_MODEL=gemma3:4b
        fly env DEBRIS=1 DEBRIS_SETTLE=75 scripts/mission/run_mission.sh $M/village_headline_confirmed.json village_C vlm
        fly scripts/mission/run_mission.sh $M/village_collapsed_building_confirmed.json village_D vlm
        fly scripts/mission/run_mission.sh $M/village_collapsed_building_confirmed.json village_F vlm
        fly scripts/mission/run_mission.sh $M/village_collapsed_building_confirmed.json village_B vlm
        unset VLM_ENDPOINT VLM_MODEL
    else
        echo "=== VLM group skipped: no NVIDIA GPU or no Ollama server"
    fi
fi
if [ "$WHAT" = "faults" ] || [ "$WHAT" = "all" ]; then
    fly env KILL_AT=75 KILL_COMM=async_slam_tool scripts/mission/run_mission.sh $M/village_collapsed_building.json village_B kill_slam
    fly env KILL_AT=75 KILL_COMM=navigator scripts/mission/run_mission.sh $M/village_collapsed_building.json village_B kill_navigator
fi
if [ "$WHAT" = "slam" ] || [ "$WHAT" = "all" ]; then
    # degraded odometry handed to SLAM: 3 % scale error and 6 deg/min drift
    fly env SLAM_BRIDGE_ARGS="-p odom_scale_error:=0.03 -p odom_yaw_drift_deg_per_min:=6.0" scripts/mission/run_mission.sh $M/village_collapsed_building.json village_B odom_drift
    fly env SLAM_BRIDGE_ARGS="-p odom_scale_error:=0.08 -p odom_yaw_drift_deg_per_min:=15.0" scripts/mission/run_mission.sh $M/village_collapsed_building.json village_B odom_drift_strong
fi
if [ "$WHAT" = "v2" ]; then
    # Repeats flown after the navigator fix (a goal that can only be
    # approached is now reported as unreachable). Not part of "all": the
    # first batch is kept as it was flown.
    interp "Search the market square for survivors." $M/village_market_square.json
    fly scripts/mission/run_mission.sh $M/village_collapsed_building.json village_A v2_r1
    fly scripts/mission/run_mission.sh $M/village_collapsed_building.json village_B v2_r1
    fly scripts/mission/run_mission.sh $M/village_collapsed_building.json village_B v2_r2
    fly scripts/mission/run_mission.sh $M/village_collapsed_building.json village_B2 v2_r1
    fly env DEBRIS=1 DEBRIS_SETTLE=75 scripts/mission/run_mission.sh $M/village_collapsed_building_protected.json village_B v2_debris
    fly scripts/mission/run_mission.sh $M/village_market_square.json village_C v2_unfamiliar
    if nvidia-smi > /dev/null 2>&1 && pgrep -x ollama > /dev/null; then
        "$HOME/ollama/bin/ollama" run gemma3:4b "Reply with the single word ready." > /dev/null 2>&1
        export VLM_ENDPOINT=http://127.0.0.1:11434/v1 VLM_MODEL=gemma3:4b
        fly env DEBRIS=1 DEBRIS_SETTLE=75 scripts/mission/run_mission.sh $M/village_headline_confirmed.json village_C v2_vlm
        fly scripts/mission/run_mission.sh $M/village_collapsed_building_confirmed.json village_D v2_vlm
        unset VLM_ENDPOINT VLM_MODEL
    else
        echo "=== VLM flights skipped: no NVIDIA GPU or no Ollama server"
    fi
fi
echo VILLAGE_BATCH_DONE
