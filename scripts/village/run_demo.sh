#!/usr/bin/env bash
# The complete visual demonstration: one instruction, the village world,
# Gazebo and RViz windows.
#
# Usage: scripts/village/run_demo.sh ["instruction"] [scenario] [tag]
#   defaults: the headline instruction, scenario village_C, tag demo_<time>
# What starts: Gazebo (with window), PX4, the lidar drone, SLAM
# (slam_toolbox), the navigator, debris chain with avoidance (when the
# instruction asks for it; boxes are dropped over the site), survivor
# perception, localisation, search planner, mission manager, RViz.
# The VLM is used if the NVIDIA GPU is up and an Ollama server is running;
# otherwise candidates are reported unverified (and a mission that says
# "confirmed only" is refused, as it should be).
# While it runs, switch the Gazebo view with:  scripts/village/view.sh site
set -u
WS_DIR=$(cd "$(dirname "$0")/../.." && pwd)
TEXT=${1:-"Search the village for survivors, prioritize the collapsed building, avoid falling debris, report survivor locations, and return to base."}
SCENARIO=${2:-village_C}
TAG=${3:-demo_$(date +%H%M%S)}
python3 "$WS_DIR/scripts/village/make_village.py" > /dev/null
if nvidia-smi > /dev/null 2>&1; then
    export NVIDIA_OFFLOAD=1
    if pgrep -x ollama > /dev/null; then
        export VLM_ENDPOINT=http://127.0.0.1:11434/v1 VLM_MODEL=gemma3:4b
        "$HOME/ollama/bin/ollama" run gemma3:4b "Reply with the single word ready." > /dev/null 2>&1
        echo "[demo] VLM live (gemma3:4b on the GPU)"
    else
        echo "[demo] no Ollama server: running without VLM"
    fi
else
    export NVIDIA_OFFLOAD=0
    echo "[demo] no NVIDIA GPU: Intel rendering, no VLM (expect a slow simulation)"
fi
PLACES=$WS_DIR/missions/places_village.json INTERPRETER=${INTERPRETER:-rules} \
    HEADLESS=${HEADLESS:-0} VIZ=${VIZ:-1} LIGHT=1 DEBRIS_SETTLE=${DEBRIS_SETTLE:-75} \
    RESULTS_ROOT=${RESULTS_ROOT:-$WS_DIR/results/village/demo} \
    "$WS_DIR/scripts/mission/run_instruction.sh" "$TEXT" "$SCENARIO" "$TAG"
