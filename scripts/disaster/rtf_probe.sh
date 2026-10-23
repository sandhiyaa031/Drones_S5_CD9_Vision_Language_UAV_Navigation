#!/usr/bin/env bash
# Real-time factor of a world WITH the project's UAV and its sensors, without
# PX4 (the simulator free-runs). Compares worlds / render settings quickly.
# Usage: scripts/disaster/rtf_probe.sh WORLD.sdf [seconds]   (prints RTF samples)
set -u
WS_DIR=$(cd "$(dirname "$0")/../.." && pwd)
W=$(realpath "$1"); SECS=${2:-40}
ROOTFS=$WS_DIR/../PX4-Autopilot/build/px4_sitl_default/rootfs
set +u; source /opt/ros/jazzy/setup.bash; source "$ROOTFS/gz_env.sh"; set -u
export GZ_SIM_RESOURCE_PATH=$WS_DIR/src/uav_autonomy/models_slam:$WS_DIR/src/uav_autonomy/models:$WS_DIR/src/uav_autonomy/models_disaster:${GZ_SIM_RESOURCE_PATH:-}
export GZ_SIM_SERVER_CONFIG_PATH=$WS_DIR/src/uav_autonomy/config/gz_server.config
gz sim -r -s "$W" > /dev/null 2>&1 &
GZ=$!
for _ in $(seq 60); do gz service -l 2>/dev/null | grep -q "^/world/collapsed_building_rescue/create$" && break; sleep 0.5; done
gz service -s /world/collapsed_building_rescue/create --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean --timeout 5000 \
  --req "name: \"x500_rescue_0\", allow_renaming: false, sdf_filename: \"$WS_DIR/src/uav_autonomy/models_slam/x500_rescue/model.sdf\"" > /dev/null 2>&1
# Gazebo renders a sensor only while something subscribes to it:
# SENSORS="cam depth lidar" (any subset; default all three; "" = none)
SENSORS=${SENSORS-cam depth lidar}
[ -n "$SENSORS" ] && python3 "$WS_DIR/scripts/disaster/gz_sink.py" "$((SECS+5))" $SENSORS &
sleep "$SECS"
gz topic -e -t /world/collapsed_building_rescue/stats -n 4 2>/dev/null | grep real_time_factor | tr '\n' ' '; echo
kill $GZ 2>/dev/null; sleep 2; kill -9 $GZ 2>/dev/null
