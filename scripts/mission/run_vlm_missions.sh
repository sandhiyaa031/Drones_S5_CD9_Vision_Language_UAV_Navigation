#!/usr/bin/env bash
# Integrated missions with the VLM live (needs the NVIDIA GPU).
#
# Usage: scripts/mission/run_vlm_missions.sh
# Results: results/phase16/live/ ; about 4 minutes per flight, 7 flights.
#
# What it does: checks the GPU, starts the local model server, warms the
# model, then flies instructions that depend on VLM confirmation. The model
# server keeps running during the flights (the perception node needs it) and
# is stopped at the end if this script started it.
set -u
WS_DIR=$(cd "$(dirname "$0")/../.." && pwd)
cd "$WS_DIR" || exit 1
if ! nvidia-smi > /dev/null 2>&1; then
    echo "NVIDIA GPU not available (nvidia-smi fails). Boot kernel 7.0.0-30" \
         "and run this again. Nothing was started."
    exit 1
fi
OUT=$WS_DIR/results/phase16
mkdir -p "$OUT/live"
STARTED=""
if ! pgrep -x ollama > /dev/null; then
    OLLAMA_HOST=127.0.0.1:11434 OLLAMA_KEEP_ALIVE=60m nohup \
        "$HOME/ollama/bin/ollama" serve > "$OUT/ollama.log" 2>&1 &
    STARTED=$!
    sleep 5
fi
echo "[vlm] warming gemma3:4b (first load can take minutes)"
"$HOME/ollama/bin/ollama" run gemma3:4b "Reply with the single word ready." \
    > "$OUT/warmup.txt" 2>&1
nvidia-smi --query-gpu=name,memory.used,memory.total --format=csv,noheader \
    | tee "$OUT/gpu_before.txt"
python3 scripts/survivor/make_worlds.py > /dev/null

export VLM_ENDPOINT=http://127.0.0.1:11434/v1 VLM_MODEL=gemma3:4b
export LLM_MODEL=gemma3:4b NVIDIA_OFFLOAD=1 LIGHT=1 CONFIRM=yes
export RESULTS_ROOT=$OUT/live

fly() {   # tag, world, instruction
    echo "=== $1 ($2): $3"
    scripts/mission/run_instruction.sh "$3" "$2" "$1" 2>&1 \
        | grep -v "WARNING: a language-model server" | cut -c1-230
}
fly site_confirmed_1 multi "Search the whole site for survivors, only report confirmed ones, and avoid falling debris."
fly site_confirmed_2 multi "Search the whole site for survivors, only report confirmed ones, and avoid falling debris."
fly site_confirmed_3 multi "Search the whole site for survivors, only report confirmed ones, and avoid falling debris."
fly site_first_found multi "Search the whole site for survivors and come back as soon as you find one."
fly building_confirmed base "Search the collapsed building for survivors and only report confirmed ones."
fly yard_confirmed outdoor "Search the yard for survivors, only report confirmed ones."
fly distractors_only distractors "Search the building for survivors, only report confirmed ones."

nvidia-smi --query-gpu=name,memory.used,memory.total --format=csv,noheader \
    > "$OUT/gpu_after.txt"
[ -n "$STARTED" ] && kill "$STARTED" 2>/dev/null
find "$OUT/live" -maxdepth 1 -type d -name 'search_*' | sort \
    | xargs python3 scripts/mission/summarize_runs.py --stats \
        --out "$OUT/summary.json" 2>&1 | cut -c1-260
echo VLM_BATCH_DONE
