# Reproduction

Everything was built and run on one laptop. Values below are the ones in
use, not recommendations.

## Environment

| Item | Value |
|---|---|
| Machine | Intel Core i7-14650HX, 15 GB RAM, NVIDIA RTX 4050 Laptop (6 GB) + Intel iGPU, PRIME on-demand |
| OS | Ubuntu 24.04.4 LTS |
| Kernel with a working NVIDIA driver | `7.0.0-30-generic`, driver `nvidia-driver-595-open 595.84` |
| ROS 2 | Jazzy (`/opt/ros/jazzy`) |
| Gazebo | Gazebo Sim (Harmonic) 8.11.0, with `ros_gz_bridge` |
| PX4-Autopilot | commit `d983b6424f3341b1e402f15d4127731c7de8e974` (2026-08-05), unmodified source, SITL target `px4_sitl_default` |
| px4_msgs | commit `f719702a6ae14ec73d22e4ec99193687c5ed788c` (versioned topics `vehicle_local_position_v1`, `vehicle_status_v4`, `vehicle_command_ack_v1`) |
| Micro XRCE-DDS Agent | `/usr/local/bin/MicroXRCEAgent`, `udp4 -p 8888` |
| Python | 3.12.3; numpy 1.26.4, scipy 1.11.4, OpenCV 4.6.0, matplotlib 3.6.3, pyulog |
| ROS packages used | `rclpy`, `px4_msgs`, `sensor_msgs`, `vision_msgs`, `visualization_msgs`, `geometry_msgs`, `std_msgs`, `ros_gz_bridge`, `rqt_image_view` |
| VLM runtime | Ollama 0.35.1, installed in the user folder `~/ollama` (no system install); models `gemma3:4b` and the CPU variant `gemma3-4b-cpu` in `~/.ollama` |

If the machine boots a different kernel (found twice: `7.0.0-34-generic`),
`nvidia-smi` fails and Gazebo renders on the Intel GPU at real-time factor
0.90–0.94. See `docs/runbook.md`, "NVIDIA driver state".

## Layout

```
~/Desktop/drones/
├── PX4-Autopilot/              PX4 source and SITL build (not modified)
├── ros2_ws/                    this repository
│   ├── src/uav_autonomy/       nodes, pure-Python cores, models, worlds, config, tests
│   ├── src/uav_interfaces/     message definitions
│   ├── src/px4_msgs/
│   ├── scripts/                trial runner, scenario runners, evaluation tools
│   ├── docs/                   one design document per phase
│   └── results/                evidence per phase; results/project_progress/ is this package
├── base paper/
└── PROJECT_CONTEXT.md
```

| Name | Value |
|---|---|
| Gazebo world | `collapsed_building_rescue` (`src/uav_autonomy/worlds/collapsed_building_rescue.sdf`); Phase 7 variants in `worlds/survivor_variants/` |
| UAV model | `x500_rescue` (`src/uav_autonomy/models/x500_rescue`), spawned as `x500_rescue_0`; PX4 airframe `gz_x500` |
| Gazebo server plugin list | `src/uav_autonomy/config/gz_server.config` |
| Camera bridge | `src/uav_autonomy/config/camera_bridge_rescue.yaml` |

## Build

```bash
cd ~/Desktop/drones/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install
```

PX4 must already be built in `../PX4-Autopilot`
(`build/px4_sitl_default/bin/px4`).

## Environment variables

Read by `scripts/run_flight_trial.sh` (all optional):

| Variable | Meaning |
|---|---|
| `NVIDIA_OFFLOAD=1` | render on the NVIDIA GPU (sets `__NV_PRIME_RENDER_OFFLOAD`, `__GLX_VENDOR_LIBRARY_NAME`, `__EGL_VENDOR_LIBRARY_FILENAMES`) |
| `CAMERA_BRIDGE=1` | bridge both cameras to ROS |
| `PERCEPTION=1` | start detector, tracker, predictor, risk |
| `AVOIDANCE=on\|baseline\|off` | start the avoidance planner (acting / disabled / not started) |
| `SURVIVOR=1` | start the survivor perception node |
| `VLM_ENDPOINT`, `VLM_MODEL` | VLM for the survivor node, e.g. `http://127.0.0.1:11434/v1`, `gemma3:4b`. `VLM_API_KEY` only if an endpoint needs one |
| `HEADLESS=0` | open the Gazebo GUI |
| `SETTLE_SECONDS` | wait after PX4 start (use 20 with the GUI) |
| `CONTROLLER_ARGS`, `VERIFY_ARGS` | e.g. `-p target_altitude:=5.0`, `--target-alt 5.0` |
| `UXRCE_SYNCT` | PX4 `UXRCE_DDS_SYNCT` for the run (default 0; stored parameters are restored afterwards) |
| `WORLD_SDF` | another world file with the same world name |

## Launch sequence

One command cold-starts Gazebo, spawns the UAV, starts PX4, the XRCE agent,
the GCS heartbeat, the bridge and the requested nodes, flies, tears down and
verifies the PX4 log:

```bash
cd ~/Desktop/drones/ros2_ws && NVIDIA_OFFLOAD=1 CAMERA_BRIDGE=1 scripts/run_flight_trial.sh results/my_run hover
```

Phase scenario runners (use the same script underneath):

```bash
scripts/avoidance/run_scenario.sh C_direct avoid
```
```bash
VLM_ENDPOINT=http://127.0.0.1:11434/v1 VLM_MODEL=gemma3:4b scripts/survivor/run_scenario.sh A_clear
```

The per-terminal manual sequence is in `GAZEBO_DEMO.md`.

## VLM setup

```bash
OLLAMA_HOST=127.0.0.1:11434 OLLAMA_KEEP_ALIVE=60m ~/ollama/bin/ollama serve
```
```bash
~/ollama/bin/ollama pull gemma3:4b
```

The server listens on localhost only; camera crops never leave the machine.
Without `VLM_ENDPOINT` the survivor node runs and reports `VLM_UNAVAILABLE`.

## Tests

```bash
cd ~/Desktop/drones/ros2_ws/src/uav_autonomy && source /opt/ros/jazzy/setup.bash && source ../../install/setup.bash && python3 -m pytest test/ -q --ignore=test/test_copyright.py --ignore=test/test_flake8.py --ignore=test/test_pep257.py
```

Expected: `315 passed`.

## Scoring a run (offline, uses ground truth)

```bash
cd ~/Desktop/drones/ros2_ws && PYTHONPATH=src/uav_autonomy python3 scripts/avoidance/evaluate_avoidance.py results/phase6/avoid/C_direct
```
```bash
PYTHONPATH=src/uav_autonomy python3 scripts/survivor/evaluate_survivor.py results/phase7/live/A_clear_vlm
```

## Useful topics and expected output

| Command | Expected |
|---|---|
| `ros2 topic hz /depth_up/image_raw` | ≈ 18–25 Hz at the ROS side (30 Hz in Gazebo) |
| `ros2 topic hz /camera/image_raw` | ≈ 15–20 Hz with NVIDIA, ≈ 14 Hz on the Intel GPU |
| `ros2 topic echo /uav/flight_status --field state` | `TAKEOFF`, `HOLD`, …, `COMPLETE` |
| `ros2 topic echo /perception/debris/risk --field highest_state` | 0 SAFE … 3 CRITICAL |
| `ros2 topic echo /perception/survivor/detections --field state` | 0 NO_TARGET, 1 CANDIDATE, 2 SURVIVOR_CONFIRMED, 3 UNCERTAIN |
| controller log | `FLIGHT COMPLETE`, or `FAILSAFE: requesting PX4 AUTO LAND` |
| `verification.json` in the run folder | `"passed": true` for a plain hover |

## What is and is not in Git

Tracked: source, scripts, docs, result summaries (JSON, Markdown, plots).
Ignored by `.gitignore`: PX4 logs (`results/**/*.ulg`), raw camera frames
(`frames_raw/`), ground-truth pose logs (`poses.csv`), `.npy/.npz` arrays,
`build/ install/ log/`. Those stay on the local disk.

The branch `claude-development` is at commit `b39741a`; all later work is in
the working tree and has not been committed.
