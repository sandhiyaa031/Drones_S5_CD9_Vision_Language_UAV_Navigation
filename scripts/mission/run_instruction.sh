#!/usr/bin/env bash
# Phase 11 end to end: a natural-language instruction is interpreted on the
# ground, read back, and (if accepted) flown as a mission.
#
# Usage: scripts/mission/run_instruction.sh "<instruction>" <world> [tag]
# Output: ${RESULTS_ROOT:-results/phase11/live}/<tag or mission name>/ with
#         interpretation.json, mission.json and, if flown, mission_report.json
# Environment:
#   INTERPRETER=llm|rules   (default llm). With llm, a local Ollama server is
#                           started for the interpretation if none is running
#                           and STOPPED AGAIN before the simulation starts:
#                           on this machine a model on the CPU starves the
#                           simulator (Phase 7).
#   LLM_MODEL               default gemma3-4b-cpu
#   PLACES=file             place catalogue (default missions/places.json;
#                           village: missions/places_village.json)
#   CONFIRM=ask|yes         ask the operator (default) or accept unattended
#   DEBRIS=auto|0|1         auto: drop boxes when the mission requires debris
#                           protection (so that the protection is exercised)
#   everything run_mission.sh understands (LIGHT, HEADLESS, …)
set -u
TEXT=${1:?instruction}
WORLD_NAME=${2:?world}
TAG=${3:-instruction_$(date +%H%M%S)}
WS_DIR=$(cd "$(dirname "$0")/../.." && pwd)
ROOT=${RESULTS_ROOT:-$WS_DIR/results/phase11/live}
PREP=$ROOT/${TAG}_interpretation
mkdir -p "$PREP"
INTERPRETER=${INTERPRETER:-llm}
STARTED_OLLAMA=""
if [ "$INTERPRETER" = "llm" ] && ! pgrep -x ollama >/dev/null; then
    OLLAMA_HOST=127.0.0.1:11434 nohup "$HOME/ollama/bin/ollama" serve \
        > "$PREP/ollama.log" 2>&1 &
    STARTED_OLLAMA=$!
    sleep 4
fi
YES=""
[ "${CONFIRM:-ask}" = "yes" ] && YES="--yes"
python3 "$WS_DIR/scripts/mission/interpret_mission.py" "$TEXT" \
    --interpreter "$INTERPRETER" --model "${LLM_MODEL:-gemma3-4b-cpu}" \
    --places "${PLACES:-$WS_DIR/missions/places.json}" \
    --out "$PREP/mission.json" --record "$PREP/interpretation.json" $YES
status=$?
if [ -n "$STARTED_OLLAMA" ]; then
    kill "$STARTED_OLLAMA" 2>/dev/null
    sleep 2
fi
if [ $status -ne 0 ]; then
    echo "[instruction] no flight (interpreter exit $status)"
    exit $status
fi
if pgrep -x ollama >/dev/null; then
    echo "[instruction] WARNING: a language-model server is still running; "\
"it may slow the simulation"
fi
DEBRIS_FLAG=${DEBRIS:-auto}
if [ "$DEBRIS_FLAG" = "auto" ]; then
    DEBRIS_FLAG=$(python3 -c "import json,sys; print(1 if json.load(open(sys.argv[1])).get('require_debris_protection') else 0)" "$PREP/mission.json")
fi
RESULTS_ROOT=$ROOT DEBRIS=$DEBRIS_FLAG \
    "$WS_DIR/scripts/mission/run_mission.sh" "$PREP/mission.json" "$WORLD_NAME" "$TAG"
NAME=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['name'])" "$PREP/mission.json")
RUN=$ROOT/${NAME}_${WORLD_NAME}_$TAG
cp "$PREP/interpretation.json" "$RUN/interpretation.json" 2>/dev/null
echo "[instruction] run folder: $RUN"
