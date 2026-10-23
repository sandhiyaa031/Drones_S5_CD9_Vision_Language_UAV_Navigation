# Phase 1 — Environment and sensors

**Status: DONE**

This page is a summary with links. The original evidence stays where it was produced; nothing was moved or copied except the figures in [`../figures/`](../figures/).

## What was built

A clean static collapsed-building world that runs in real time, a measured choice of sensors (downward RGB camera for survivors, upward 100° depth camera for debris), validated camera geometry, and a time base in which PX4 and Gazebo stamps agree.

## Key results

* real-time factor 0.41 (old world) → 0.9986 over a 320 s benchmark (NVIDIA GPU), both cameras 30.2–30.3 Hz, memory growth 3.6 MiB
* sensor study: 3 flights, 20 of 24 releases analysed; a downward camera cannot see debris in time
* depth error on flat targets ≤ 1e-5 m (no noise is simulated); falling-box centre error 13–22 mm median, 41 mm max; 21 of 24 boxes reconstructed
* no rotor or body pixels in 501 hover frames
* time base: with `UXRCE_DDS_SYNCT=0` the PX4-to-simulation timing error is 0 ms (default setting: 6.0 s drift over a 48 s hover)

## Failures and limits

* the performance target needs the NVIDIA driver; on the Intel GPU the real-time factor is 0.90–0.94
* the depth sensor has no noise model
* the zenith camera cannot see low, lateral threats
* the time base is a simulation solution, not a hardware clock-synchronisation method

## Source

* [`worlds/collapsed_building_rescue.sdf`](../../../src/uav_autonomy/worlds/collapsed_building_rescue.sdf)
* [`scripts/run_flight_trial.sh`](../../../scripts/run_flight_trial.sh)

## Evidence

* [`docs/sensor_integration.md`](../../../docs/sensor_integration.md)
* [`results/sensor_study/README.md`](../../sensor_study/README.md)
* [`results/sensor_integration/README.md`](../../sensor_integration/README.md)
* [`results/sensor_integration/v09_benchmark_nvidia/`](../../sensor_integration/v09_benchmark_nvidia/)
* [`results/time_sync/README.md`](../../time_sync/README.md)
* [`results/phase1/`](../../phase1/)

## Figures

* [01_downward_camera_collapsed_building.jpg](../figures/01_downward_camera_collapsed_building.jpg)
* [02_depth_camera_orientation_check.png](../figures/02_depth_camera_orientation_check.png)
* [03_sensor_study_warning_time_by_camera.png](../figures/03_sensor_study_warning_time_by_camera.png)
* [04_sensor_study_downward_camera_visibility.png](../figures/04_sensor_study_downward_camera_visibility.png)

Full detail: [`../MASTER_RESULTS.md`](../MASTER_RESULTS.md) · numbers with sources: [`../METRICS_SUMMARY.md`](../METRICS_SUMMARY.md) · equations: [`../MATHEMATICAL_METHODS.md`](../MATHEMATICAL_METHODS.md)
