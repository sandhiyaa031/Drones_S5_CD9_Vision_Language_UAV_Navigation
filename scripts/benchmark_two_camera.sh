#!/usr/bin/env bash
# Acceptance benchmark for the final two-camera configuration:
#   downward mono 1280x960 @ 30 Hz + upward depth 640x480 @ 30 Hz (35 m clip),
#   NVIDIA rendering, UXRCE_DDS_SYNCT=0, simulation only (no flight).
#
# Usage: scripts/benchmark_two_camera.sh <run_dir> [duration_s (default 320)]
#
# Refuses to count as a pass unless Gazebo actually rendered on the NVIDIA
# GPU. BENCH_ALLOW_ANY_GPU=1 runs it anyway (result is then marked as such).
set -u
RUN_DIR=${1:?usage: benchmark_two_camera.sh <run_dir> [duration_s]}
DURATION=${2:-320}
WS_DIR=$(cd "$(dirname "$0")/.." && pwd)

if ! nvidia-smi > /dev/null 2>&1; then
    echo "NVIDIA driver is not loaded (nvidia-smi fails) on kernel $(uname -r)." >&2
    if [ "${BENCH_ALLOW_ANY_GPU:-0}" != "1" ]; then
        echo "See docs/runbook.md, 'NVIDIA driver state'. Not running." >&2
        exit 2
    fi
    OFFLOAD=0
else
    OFFLOAD=1
fi

export BENCH_DURATION=$DURATION
HOOK='
python3 $WS_DIR/scripts/sensor_integration/time_record.py --duration $BENCH_DURATION --output $RUN_DIR/time_capture.npz > $RUN_DIR/time_record.out 2>&1 &
n=$(( BENCH_DURATION / 60 )); [ $n -lt 1 ] && n=1
for i in $(seq 0 $n); do
    python3 $WS_DIR/scripts/measure_sim.py --duration 20 --label t$((i*60))s --output $RUN_DIR/measure_$i.json > /dev/null
    [ $i -lt $n ] && sleep 40
done
wait
python3 $WS_DIR/scripts/sensor_integration/time_analyze.py $RUN_DIR/time_capture.npz --output $RUN_DIR/time_analysis.json > /dev/null
'
UXRCE_SYNCT=0 NVIDIA_OFFLOAD=$OFFLOAD CAMERA_BRIDGE=1 PROJECT_MODEL=x500_rescue \
    TRIAL_HOOK="$HOOK" "$WS_DIR/scripts/run_flight_trial.sh" "$RUN_DIR" none
python3 "$WS_DIR/scripts/benchmark_report.py" "$RUN_DIR"
