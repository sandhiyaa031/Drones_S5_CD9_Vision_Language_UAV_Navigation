#!/usr/bin/env bash
# PX4 SITL in the loop with the MuJoCo simulator (HIL bridge), flown by the
# project's UNCHANGED flight_controller.py through the XRCE-DDS agent.
#
# Starts: MuJoCo HIL bridge (TCP server 4560) -> PX4 (airframe 10016_none_iris,
# the MAVLink-simulator airframe) -> Micro XRCE-DDS agent -> GCS heartbeat ->
# flight_controller (hover: Offboard, arm, takeoff, hold, land).
# Needs ROS 2 (sourced) and the built workspace. Does NOT start Gazebo and must
# not run while a Gazebo flight is running (CPU contention spoils both).
#
# Usage: [MODE=hover|waypoint] run_px4_mujoco_trial.sh [TAG] [TARGET_ALTITUDE_M]
# Results: simulation/mujoco/results/px4_hil/<TAG>/
# PX4's stored parameters are saved and restored (UXRCE_DDS_SYNCT=0 for this
# run only, as in ros2_ws/scripts/run_flight_trial.sh).
set -u
HERE=$(cd "$(dirname "$0")/.." && pwd)
WS_DIR=$(cd "$HERE/../.." && pwd)
PX4_DIR=$(cd "$WS_DIR/../PX4-Autopilot" && pwd)
ROOTFS=$PX4_DIR/build/px4_sitl_default/rootfs
TAG=${1:-hil_$(date +%H%M%S)}; ALT=${2:-3.0}; MODE=${MODE:-hover}   # hover | waypoint
RUN_DIR=$HERE/results/px4_hil/$TAG
# never overwrite an earlier result folder (evidence): add a time suffix instead
if [ -d "$RUN_DIR" ] && [ -n "$(ls -A "$RUN_DIR" 2>/dev/null)" ]; then RUN_DIR=${RUN_DIR}_$(date +%H%M%S); echo "[trial] tag exists, using $RUN_DIR"; fi
mkdir -p "$RUN_DIR"
set +u; source /opt/ros/jazzy/setup.bash; source "$WS_DIR/install/setup.bash"; set -u
PARAM_BACKUP_DIR=$ROOTFS/.px4_param_backup_mujoco

mine() { ps -eo pid,comm,args | awk '$2 ~ /^(px4|MicroXRCEAgent|flight_controll)$/ || ($2 ~ /^python/ && /px4_hil_bridge|gcs_heartbeat|ros2 run uav_autonomy/) {print $1}'; }
others() { ps -eo pid,comm,args | awk '$2 ~ /^(px4|MicroXRCEAgent|flight_controll|ruby|gz)/ || ($2 ~ /^python/ && /px4_hil_bridge|gcs_heartbeat|ros2 run uav_autonomy/) {print $1}'; }
restore() {
    [ -d "$PARAM_BACKUP_DIR" ] || return
    for f in parameters.bson parameters_backup.bson; do
        [ -f "$PARAM_BACKUP_DIR/$f" ] && cp -p "$PARAM_BACKUP_DIR/$f" "$ROOTFS/$f"
    done
    rm -rf "$PARAM_BACKUP_DIR"
}
cleanup() {
    (cd "$ROOTFS" && timeout 5 ../bin/px4-shutdown >/dev/null 2>&1)
    local p; p=$(mine); [ -n "$p" ] && kill -INT $p 2>/dev/null; sleep 3
    p=$(mine); [ -n "$p" ] && kill -9 $p 2>/dev/null; sleep 1
    restore
}
# Refuse BEFORE the cleanup trap exists: a refused launch must not kill the
# simulation that is already running (a Gazebo flight, or another trial).
if [ -n "$(others)" ]; then echo "[trial] REFUSING TO START: simulation processes are running: $(others | tr '\n' ' ')" >&2; exit 3; fi
trap cleanup EXIT
trap 'exit 130' INT TERM
restore
mkdir -p "$PARAM_BACKUP_DIR"
cp -p "$ROOTFS/parameters.bson" "$ROOTFS/parameters_backup.bson" "$PARAM_BACKUP_DIR/" 2>/dev/null
export PX4_PARAM_UXRCE_DDS_SYNCT=0 PX4_PARAM_SIM_GZ_EN=0
before=$(ls -1t "$ROOTFS"/log/*/*.ulg 2>/dev/null | head -1)

echo "[trial] MuJoCo HIL bridge"
"$HERE/.venv/bin/python" -u "$HERE/scripts/px4_hil_bridge.py" --scene "${SCENE:-urban_A}" \
    --duration "${BRIDGE_SECONDS:-200}" --out "$RUN_DIR" > "$RUN_DIR/bridge.log" 2>&1 &
sleep 2
echo "[trial] PX4 (10016_none_iris)"
( cd "$ROOTFS" && PX4_SYS_AUTOSTART=10016 exec ../bin/px4 -d ) > "$RUN_DIR/px4.log" 2>&1 &
for _ in $(seq 60); do grep -q 'Startup script returned successfully' "$RUN_DIR/px4.log" 2>/dev/null && break; sleep 1; done
grep -q 'Startup script returned successfully' "$RUN_DIR/px4.log" || { echo "[trial] PX4 did not start"; exit 4; }
MicroXRCEAgent udp4 -p 8888 > "$RUN_DIR/xrce_agent.log" 2>&1 &
python3 "$WS_DIR/scripts/gcs_heartbeat.py" > "$RUN_DIR/gcs_heartbeat.log" 2>&1 &
sleep "${SETTLE_SECONDS:-15}"
echo "[trial] flight_controller (hover at $ALT m)"
WPFLAG=false; GATE=""
# waypoint mode: only call a waypoint (and home) reached when the vehicle has slowed
# down (the Gazebo missions use 0.3 m/s too). Without it the controller starts
# AUTO LAND while flying past home and the landing drifted 2.1-2.2 m (hilwp1,
# audit_wp_default); with it 0.37 m (audit_wp_gate).
[ "$MODE" = "waypoint" ] && { WPFLAG=true; GATE="-p arrival_max_speed:=${ARRIVAL_MAX_SPEED:-0.3}"; }
ros2 run uav_autonomy flight_controller --ros-args -p waypoint_mission:=$WPFLAG \
    -p target_altitude:=$ALT $GATE ${CONTROLLER_ARGS:-} > "$RUN_DIR/flight_controller.log" 2>&1 &
for _ in $(seq "${FLIGHT_TIMEOUT:-300}"); do
    grep -qE 'FLIGHT COMPLETE|FAILSAFE' "$RUN_DIR/flight_controller.log" 2>/dev/null && break
    grep -q 'STALL' "$RUN_DIR/bridge.log" 2>/dev/null && break
    sleep 1
done
outcome=NO_OUTCOME
grep -q 'FLIGHT COMPLETE' "$RUN_DIR/flight_controller.log" && outcome=COMPLETE
grep -q 'FAILSAFE' "$RUN_DIR/flight_controller.log" && outcome=FAILSAFE
grep -q 'STALL' "$RUN_DIR/bridge.log" 2>/dev/null && { outcome=BRIDGE_STALL; echo "[trial] the bridge reported a lockstep STALL (see bridge.log)"; }
echo "[trial] controller outcome: $outcome"
sleep 3
after=$(ls -1t "$ROOTFS"/log/*/*.ulg 2>/dev/null | head -1)
if [ -n "$after" ] && [ "$after" != "$before" ]; then
    cp "$after" "$RUN_DIR/flight.ulg"
    WPARG=""; [ "$MODE" = "waypoint" ] && WPARG="--waypoints=${WAYPOINTS:--4,-4;-4,-5.5;0,0}"
    python3 "$WS_DIR/scripts/verify_ulog.py" --mode "$MODE" --target-alt "$ALT" \
        --hold-seconds 15 $WPARG --output "$RUN_DIR/verification.json" "$RUN_DIR/flight.ulg" > "$RUN_DIR/verify.out" 2>&1
    tail -3 "$RUN_DIR/verify.out"
fi
echo "controller_outcome=$outcome" > "$RUN_DIR/trial_info.txt"
