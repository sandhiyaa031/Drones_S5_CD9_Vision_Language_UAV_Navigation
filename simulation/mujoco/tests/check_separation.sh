#!/usr/bin/env bash
# Simulator separation check (about 12 minutes, nothing else may be running).
#  A. a Gazebo mission is flying; a MuJoCo PX4 trial is started: it must refuse
#     (exit 3) and the Gazebo flight must be unaffected and finish COMPLETE.
#  B. a MuJoCo PX4 trial is flying; a Gazebo mission is started: it must refuse
#     (exit 3) and the MuJoCo trial must be unaffected and finish COMPLETE.
# Output: results/audit/separation/
set -u
HERE=$(cd "$(dirname "$0")/.." && pwd)
WS=$(cd "$HERE/../.." && pwd)
OUT=$HERE/results/audit/separation; mkdir -p "$OUT"
set +u; source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"; set -u
export NVIDIA_OFFLOAD=0 LIGHT=1
procs() { ps -eo comm | awk '$1 ~ /^(px4|MicroXRCEAgent|ruby|flight_controll)$/' | sort | uniq -c | tr '\n' ' '; }
echo "processes at start: [$(procs)]" | tee "$OUT/summary.txt"

# ---- A
RESULTS_ROOT=$OUT/gazebo scripts_dir=$WS setsid nohup "$WS/scripts/mission/run_mission.sh" "$WS/missions/urban_collapsed_block.json" urban_B sepA > "$OUT/A_gazebo.log" 2>&1 < /dev/null &
d=$OUT/gazebo/search_collapsed_block_urban_B_sepA
until [ -f "$d/mission_manager.log" ] && grep -q "MISSION SEARCH" "$d/mission_manager.log"; do sleep 3; done
echo "A: Gazebo flight in SEARCH, processes [$(procs)]" | tee -a "$OUT/summary.txt"
"$HERE/scripts/run_px4_mujoco_trial.sh" sepA_refused 3.0 > "$OUT/A_mujoco_attempt.log" 2>&1; rc=$?
echo "A: MuJoCo PX4 trial exit code $rc (expected 3): $(head -1 "$OUT/A_mujoco_attempt.log" | cut -c1-120)" | tee -a "$OUT/summary.txt"
echo "A: Gazebo processes after the refused attempt [$(procs)]" | tee -a "$OUT/summary.txt"
until grep -q "MISSION REPORT" "$d/mission_manager.log" 2>/dev/null; do sleep 3; done; sleep 25
echo "A: Gazebo outcome: $(grep -o 'outcome [A-Z]*' "$d/mission_manager.log" | tail -1) | PX4 log checks: $(python3 -c "import json;print(json.load(open('$d/verification.json'))['passed'])" 2>&1)" | tee -a "$OUT/summary.txt"
sleep 5; echo "A: processes after the flight [$(procs)]" | tee -a "$OUT/summary.txt"

# ---- B
setsid nohup "$HERE/scripts/run_px4_mujoco_trial.sh" sepB 3.0 > "$OUT/B_mujoco.log" 2>&1 < /dev/null &
until grep -q "flight_controller (hover" "$OUT/B_mujoco.log" 2>/dev/null; do sleep 2; done; sleep 10
echo "B: MuJoCo PX4 trial running, processes [$(procs)]" | tee -a "$OUT/summary.txt"
RESULTS_ROOT=$OUT/gazebo2 "$WS/scripts/mission/run_mission.sh" "$WS/missions/urban_collapsed_block.json" urban_B sepB_refused > "$OUT/B_gazebo_attempt.log" 2>&1; rc=$?
echo "B: Gazebo mission refused: $(grep -m1 REFUSING "$OUT"/gazebo2/*sepB_refused.out | cut -c1-80) (the wrapper run_mission.sh hides the exit code; the refusal text and exit=3 are in its .out)" | tee -a "$OUT/summary.txt"
until grep -q "controller outcome" "$OUT/B_mujoco.log"; do sleep 3; done; sleep 15
echo "B: MuJoCo trial: $(grep 'controller outcome' "$OUT/B_mujoco.log") | PX4 log checks: $(python3 -c "import json;print(json.load(open('$HERE/results/px4_hil/sepB/verification.json'))['passed'])" 2>&1)" | tee -a "$OUT/summary.txt"
sleep 5; echo "B: processes at the end [$(procs)]" | tee -a "$OUT/summary.txt"
echo SEPARATION_CHECK_DONE | tee -a "$OUT/summary.txt"
