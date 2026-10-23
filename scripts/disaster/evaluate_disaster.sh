#!/usr/bin/env bash
# Score urban disaster-zone flights offline (ground truth is used here only).
# Usage: scripts/disaster/evaluate_disaster.sh [run folder ...]
#   default: every run under results/disaster_gazebo/live and the demo folders
# Writes results/disaster_gazebo/summary_missions.json and summary_slam.json.
set -u
WS_DIR=$(cd "$(dirname "$0")/../.." && pwd)
cd "$WS_DIR" || exit 1
if [ $# -gt 0 ]; then RUNS=("$@"); else
    mapfile -t RUNS < <(find results/disaster_gazebo/live results/disaster_gazebo/demo -maxdepth 1 -type d -name 'search_*' | sort)
fi
python3 scripts/mission/summarize_runs.py --stats --out results/disaster_gazebo/summary_missions.json "${RUNS[@]}"
for r in "${RUNS[@]}"; do
    world=$(grep -o 'world=urban_[A-Za-z0-9]*' "$r/trial_info.txt" | tail -1 | cut -d= -f2)
    [ -f "$r/slam_poses.jsonl" ] && python3 scripts/slam/evaluate_slam.py --world "${world:-urban_C}" --out "$r/slam_score.json" "$r"
done
python3 - "${RUNS[@]}" <<'PY'
import json, os, sys
out = {}
for r in sys.argv[1:]:
    p = os.path.join(r, 'slam_score.json')
    if os.path.exists(p):
        out.update(json.load(open(p)))
json.dump(out, open('results/disaster_gazebo/summary_slam.json', 'w'), indent=1)
print(f'SLAM scores for {len(out)} runs -> results/disaster_gazebo/summary_slam.json')
PY
