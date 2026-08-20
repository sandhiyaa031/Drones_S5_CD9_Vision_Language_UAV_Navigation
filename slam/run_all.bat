@echo off
echo ===========================================================================
echo   GPS-DENIED AUTONOMOUS UAV NAVIGATION - COMPLETE ONE-CLICK RUNNER (MEMBER 4)
echo ===========================================================================

echo.
echo [1/4] Running Automated Unit Tests (pytest)...
python -m pytest -v tests/
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Unit tests failed!
    pause
    exit /b %ERRORLEVEL%
)

echo.
echo [2/4] Running 3D GPS-Denied Flight Simulation, SLAM and Sensor Fusion...
python scripts/run_synthetic_demo.py
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Simulation failed!
    pause
    exit /b %ERRORLEVEL%
)

echo.
echo [3/4] Running Trajectory Benchmark and Accuracy Evaluation...
python scripts/evaluate_trajectory.py --gt ground_truth.txt --est estimated_trajectory.txt --align --output_plot benchmark_results.png
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Trajectory evaluation failed!
    pause
    exit /b %ERRORLEVEL%
)

echo.
echo [4/4] Generating PowerPoint Presentation (Member4_GPS_Denied_Navigation.pptx)...
python scripts/generate_presentation.py
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Presentation generation failed!
    pause
    exit /b %ERRORLEVEL%
)

echo.
echo ===========================================================================
echo   [SUCCESS] ALL CODE, SIMULATIONS, BENCHMARKS AND PPT GENERATED!
echo   Outputs created in this folder:
echo     - Member4_GPS_Denied_Navigation.pptx (8-slide PowerPoint deck)
echo     - evaluation_benchmark_plot.png (4-panel trajectory plots)
echo     - benchmark_results.png (ATE / RPE accuracy report)
echo     - dashboard_snapshot_mid.png and dashboard_snapshot_final.png
echo     - ground_truth.txt and estimated_trajectory.txt
echo ===========================================================================
pause
