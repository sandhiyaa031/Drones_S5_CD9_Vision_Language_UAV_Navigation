#!/usr/bin/env bash
set -e

echo "==========================================================================="
echo "  GPS-DENIED AUTONOMOUS UAV NAVIGATION - COMPLETE RUNNER (MEMBER 4)"
echo "==========================================================================="

echo ""
echo "[1/4] Running Automated Unit Tests (pytest)..."
python3 -m pytest -v tests/

echo ""
echo "[2/4] Running 3D GPS-Denied Flight Simulation, SLAM and Sensor Fusion..."
python3 scripts/run_synthetic_demo.py

echo ""
echo "[3/4] Running Trajectory Benchmark and Accuracy Evaluation..."
python3 scripts/evaluate_trajectory.py --gt ground_truth.txt --est estimated_trajectory.txt --align --output_plot benchmark_results.png

echo ""
echo "[4/4] Generating PowerPoint Presentation (Member4_GPS_Denied_Navigation.pptx)..."
python3 scripts/generate_presentation.py

echo ""
echo "==========================================================================="
echo "  [SUCCESS] ALL CODE, SIMULATIONS, BENCHMARKS AND PPT GENERATED!"
echo "==========================================================================="
