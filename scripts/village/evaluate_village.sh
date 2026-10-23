#!/usr/bin/env bash
# Score village flights offline (ground truth is used here only).
# Usage: scripts/village/evaluate_village.sh [run folder ...]
#   default: every run under results/village/live
# Writes results/village/summary_missions.json and summary_slam.json.
set -u
WS_DIR=$(cd "$(dirname "$0")/../.." && pwd)
cd "$WS_DIR" || exit 1
if [ $# -gt 0 ]; then RUNS=("$@"); else
    mapfile -t RUNS < <(find results/village/live -maxdepth 1 -type d -name 'search_*' | sort)
fi
python3 scripts/mission/summarize_runs.py --stats --out results/village/summary_missions.json "${RUNS[@]}"
for r in "${RUNS[@]}"; do
    world=$(grep -o 'world=village_[A-Za-z0-9]*' "$r/trial_info.txt" | tail -1 | cut -d= -f2)
    [ -f "$r/slam_poses.jsonl" ] && python3 scripts/slam/evaluate_slam.py --world "${world:-village_C}" --out "$r/slam_score.json" "$r"
done
python3 - "${RUNS[@]}" <<'PY'
import json, os, sys
out = {}
for r in sys.argv[1:]:
    p = os.path.join(r, 'slam_score.json')
    if os.path.exists(p):
        out.update(json.load(open(p)))
json.dump(out, open('results/village/summary_slam.json', 'w'), indent=1)
print(f'SLAM scores for {len(out)} runs -> results/village/summary_slam.json')
PY
