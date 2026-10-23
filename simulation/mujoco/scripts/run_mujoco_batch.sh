#!/usr/bin/env bash
# MuJoCo mission batch (headless). Results: results/batch/<name>/mission_report.json
# Usage: scripts/run_mujoco_batch.sh        (about 15 minutes)
set -u
HERE=$(cd "$(dirname "$0")/.." && pwd)
cd "$HERE" || exit 1
export MUJOCO_GL=${MUJOCO_GL:-egl}
run() { name=$1; scene=$2; shift 2; echo "=== $name ($scene)"; .venv/bin/python -u scripts/run_mission.py "$@" --scene "$scene" --out results/${BATCH_DIR:-batch}/$name 2>&1 | grep -E "outcome|search_end|coverage|survivors_found|false_candidates|collisions|landed|entry|REFUSED" | cut -c1-230; }
run block_B urban_B "Search the collapsed apartment block for survivors, report survivor locations, and return to base."
run courtyard_B2 urban_B2 "Search the courtyard for survivors, report survivor locations, and return to base."
run block_D_occluded urban_D "Search the collapsed apartment block for survivors, report survivor locations, and return to base."
run block_F_lookalikes urban_F "Search the collapsed apartment block for survivors, report survivor locations, and return to base."
run zone_C_headline urban_C "Search the disaster zone for survivors, prioritize the collapsed apartment block, report survivor locations, and return to base."
run courtyard_C urban_C "Search the courtyard for survivors, report survivor locations, and return to base."
echo MUJOCO_BATCH_DONE
