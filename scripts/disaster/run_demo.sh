#!/usr/bin/env bash
# One command: the urban disaster zone, one sentence, Gazebo window and RViz.
#
# Usage: scripts/disaster/run_demo.sh ["instruction"] [scenario] [tag]
#   defaults: the headline instruction, scenario urban_C, tag demo_<time>
# What starts: Gazebo (with window, shadows), PX4, the lidar UAV, SLAM
# (slam_toolbox), the navigator, the debris chain with avoidance (when the
# instruction asks for it), survivor perception, localisation, the search
# planner, the mission manager, RViz, and a few extra cameras whose frames
# are saved as screenshots of the running simulation.
# The VLM is used only if the NVIDIA GPU works and an Ollama server runs;
# otherwise candidates are reported unverified (and a "confirmed only"
# mission is refused, as it should be).
# While it runs the window camera chases the drone (CHASE=0 to turn off);
# other views: CHASE=0 and scripts/disaster/view.sh overview|collapse|courtyard|street
set -u
WS_DIR=$(cd "$(dirname "$0")/../.." && pwd)
TEXT=${1:-"Search the collapsed apartment block for survivors, only report confirmed ones, avoid falling debris, report survivor locations, and return to base."}
SCENARIO=${2:-urban_C}
TAG=${3:-demo_$(date +%H%M%S)}
python3 "$WS_DIR/scripts/disaster/make_disaster_zone.py" --scenario "$SCENARIO" > /dev/null
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
    if [ "${VLM_CPU:-1}" = "1" ] && [ -x "$HOME/ollama/bin/ollama" ]; then
        # CPU inference of the same Gemma 3 4B model (no NVIDIA driver on this kernel):
        # about 50-100 s per crop. It runs in a worker thread; flight control never waits.
        if ! curl -s -m 3 http://127.0.0.1:11434/api/tags > /dev/null; then
            # pinned to the last 12 CPUs (E-cores + 2 P-cores) at the lowest priority: otherwise its 8 busy threads
            # starve PX4/Gazebo and the controller's telemetry watchdog fires (measured, 2 failsafes)
            (cd "$HOME" && OLLAMA_HOST=127.0.0.1:11434 setsid nohup taskset -c ${VLM_CPUS:-12-23} nice -n 19 "$HOME/ollama/bin/ollama" serve > "$HOME/ollama/serve_demo.log" 2>&1 < /dev/null &)
            for _ in $(seq 20); do curl -s -m 2 http://127.0.0.1:11434/api/tags > /dev/null && break; sleep 1; done
        fi
        # lowest priority: the simulation and the flight controller keep their CPU time
        for pid in $(pgrep -x ollama); do renice -n 19 -p "$pid" > /dev/null 2>&1; done
        for pid in $(pgrep -f "ollama runner"); do renice -n 19 -p "$pid" > /dev/null 2>&1; done
        # PX4's status stream (about 2 Hz of sim time) gaps beyond 3 s of wall time when the CPU VLM,
        # the Gazebo window and the debris chain share the machine (failsafes at "goal is stale" in 3 runs)
        export STATUS_TIMEOUT_S=${STATUS_TIMEOUT_S:-8.0}
        export POSITION_TIMEOUT_S=${POSITION_TIMEOUT_S:-3.0}
        export VLM_ENDPOINT=http://127.0.0.1:11434/v1 VLM_MODEL=gemma3-4b-cpu
        export SURVIVOR_ARGS="--ros-args -p vlm_timeout_s:=170.0 -p request_only_when_inspecting:=true -p confirmation.stop_after_confirm:=true -p confirmation.max_answers:=3 -p crop_dir:=${RESULTS_ROOT:-$WS_DIR/results/disaster_gazebo/demo}/vlm_crops_${3:-demo}"
        export PLANNER_ARGS="-p plan.inspect_timeout_s:=320.0 -p plan.inspect_dwell_s:=25.0"
        echo "[demo] VLM on the CPU (gemma3-4b-cpu, slow: ~1 min per crop, asynchronous)"
    else
        echo "[demo] no NVIDIA GPU: Intel rendering, no VLM"
    fi
fi
OUTROOT=${RESULTS_ROOT:-$WS_DIR/results/disaster_gazebo/demo}
# CHASE=1 (default): the window camera stays behind the drone (scripts/disaster/chase_cam.py);
# CHASE=0 leaves the camera to you (scripts/disaster/view.sh ...).
CHASE_CMD=""
[ "${CHASE:-1}" = "1" ] && CHASE_CMD="python3 \$WS_DIR/scripts/disaster/chase_cam.py --duration \${LOG_SECONDS:-900} > \$RUN_DIR/chase_cam.log 2>&1 & "
export EXTRA_HOOK="${CHASE_CMD}python3 \$WS_DIR/scripts/disaster/capture_views.py --out-dir \$RUN_DIR/screenshots --duration \${LOG_SECONDS:-500} --interval ${SHOT_INTERVAL:-20} & \$WS_DIR/scripts/disaster/capture_windows.sh \$RUN_DIR/screenshots_windows \${LOG_SECONDS:-500} ${SHOT_INTERVAL:-25} &"
PLACES=$WS_DIR/missions/places_disaster.json INTERPRETER=${INTERPRETER:-rules} \
    HEADLESS=${HEADLESS:-0} VIZ=${VIZ:-1} LIGHT=1 DEBRIS_SETTLE=${DEBRIS_SETTLE:-70} \
    PLAN=${PLAN:-$WS_DIR/results/disaster_gazebo/plans/urban_lead.json} \
    RESULTS_ROOT=$OUTROOT CONFIRM=${CONFIRM:-ask} \
    "$WS_DIR/scripts/mission/run_instruction.sh" "$TEXT" "$SCENARIO" "$TAG"
