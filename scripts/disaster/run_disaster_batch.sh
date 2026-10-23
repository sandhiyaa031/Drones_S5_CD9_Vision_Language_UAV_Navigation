#!/usr/bin/env bash
# Urban disaster-zone evaluation batch (headless, Intel or NVIDIA).
# Usage: scripts/disaster/run_disaster_batch.sh [core|faults|all|ext]
#   all / core / faults : first batch (areas that include building interiors;
#                         kept as flown, see MULTI_SIM_PROGRESS.md)
#   ext                 : final batch on the EXTERIOR areas of places_disaster.json
# Results: results/disaster_gazebo/live/<run>/, nothing is deleted. About 6-10
# minutes per flight. No VLM is used here (candidates are reported unverified).
set -u
WS_DIR=$(cd "$(dirname "$0")/../.." && pwd)
cd "$WS_DIR" || exit 1
set +u; source /opt/ros/jazzy/setup.bash; source install/setup.bash; set -u
WHAT=${1:-all}
export RESULTS_ROOT=$WS_DIR/results/disaster_gazebo/live LIGHT=1
export NVIDIA_OFFLOAD=${NVIDIA_OFFLOAD:-0}
M=$WS_DIR/missions
python3 scripts/disaster/make_disaster_zone.py > /dev/null
fly() { echo "=== $*"; "$@" 2>&1 | grep "controller outcome\|MISSION R" | cut -c1-220; }
GAPS='python3 $WS_DIR/scripts/disaster/gap_monitor.py --out $RUN_DIR/gaps.jsonl --duration 700 --gap 0.7 &'
if [ "$WHAT" = "core" ] || [ "$WHAT" = "all" ]; then
    fly env EXTRA_HOOK="$GAPS" scripts/mission/run_mission.sh $M/urban_collapsed_block.json urban_A b1
    fly env EXTRA_HOOK="$GAPS" scripts/mission/run_mission.sh $M/urban_courtyard.json urban_B2 b1
    fly env EXTRA_HOOK="$GAPS" DEBRIS=1 DEBRIS_SETTLE=70 PLAN=$WS_DIR/results/disaster_gazebo/plans/urban_lead.json scripts/mission/run_mission.sh $M/urban_headline.json urban_C headline
    fly env EXTRA_HOOK="$GAPS" scripts/mission/run_mission.sh $M/urban_collapsed_block.json urban_D b1
    fly env EXTRA_HOOK="$GAPS" scripts/mission/run_mission.sh $M/urban_collapsed_block.json urban_F b1
    fly env EXTRA_HOOK="$GAPS" scripts/mission/run_mission.sh $M/urban_collapsed_block.json urban_B b2
fi
if [ "$WHAT" = "faults" ] || [ "$WHAT" = "all" ]; then
    fly env KILL_AT=75 KILL_COMM=async_slam_tool scripts/mission/run_mission.sh $M/urban_collapsed_block.json urban_B kill_slam
    fly env KILL_AT=75 KILL_COMM=navigator scripts/mission/run_mission.sh $M/urban_collapsed_block.json urban_B kill_navigator
fi
if [ "$WHAT" = "ext" ]; then
    fly env EXTRA_HOOK="$GAPS" scripts/mission/run_mission.sh $M/urban_collapsed_block.json urban_A x1
    fly env EXTRA_HOOK="$GAPS" scripts/mission/run_mission.sh $M/urban_collapsed_block.json urban_B x1
    fly env EXTRA_HOOK="$GAPS" scripts/mission/run_mission.sh $M/urban_collapsed_block.json urban_B x2
    fly env EXTRA_HOOK="$GAPS" scripts/mission/run_mission.sh $M/urban_courtyard.json urban_B2 x1
    fly env EXTRA_HOOK="$GAPS" scripts/mission/run_mission.sh $M/urban_collapsed_block.json urban_D x1
    fly env EXTRA_HOOK="$GAPS" scripts/mission/run_mission.sh $M/urban_collapsed_block.json urban_F x1
    fly env EXTRA_HOOK="$GAPS" DEBRIS=1 DEBRIS_SETTLE=70 PLAN=$WS_DIR/results/disaster_gazebo/plans/urban_lead.json scripts/mission/run_mission.sh $M/urban_headline.json urban_C xheadline
    fly env EXTRA_HOOK="$GAPS" DEBRIS=1 DEBRIS_SETTLE=70 PLAN=$WS_DIR/results/disaster_gazebo/plans/urban_lead.json scripts/mission/run_mission.sh $M/urban_collapsed_block_protected.json urban_B xdebris
    fly env KILL_AT=60 KILL_COMM=async_slam_tool scripts/mission/run_mission.sh $M/urban_collapsed_block.json urban_B xkill_slam
    fly env KILL_AT=60 KILL_COMM=navigator scripts/mission/run_mission.sh $M/urban_collapsed_block.json urban_B xkill_navigator
fi
echo URBAN_BATCH_DONE
