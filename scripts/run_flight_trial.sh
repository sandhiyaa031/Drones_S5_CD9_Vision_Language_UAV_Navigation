#!/usr/bin/env bash
# One cold-start flight trial: Gazebo + PX4 SITL + XRCE agent + GCS heartbeat
# + flight_controller, then teardown and ULog verification.
#
# Usage: scripts/run_flight_trial.sh <run_dir> [hover|waypoint|none]
#   none = start the simulation only, run TRIAL_HOOK, tear down (no flight).
#
# Environment (all optional):
#   PX4_DIR        PX4-Autopilot checkout   (default: ../PX4-Autopilot)
#   WORLD          Gazebo world name        (default: collapsed_building_rescue)
#   WORLD_SDF      world file; if it does not exist, WORLD is looked up in
#                  PX4's own worlds directory instead
#   PROJECT_MODEL  project UAV model        (default: x500_rescue; "none" = MODEL)
#   MODEL          PX4 sim model, used when PROJECT_MODEL=none
#                                           (default: gz_x500_mono_cam_down)
#   GZ_SERVER_CONFIG  Gazebo server plugin list (default: project config;
#                  set to "px4" to use PX4's unmodified server.config)
#   EXTRA_RESOURCE_PATH  model directory searched before PX4's models
#   UXRCE_SYNCT    PX4 UXRCE_DDS_SYNCT for this run only (default: 0; "px4" =
#                  leave PX4's own setting). Restored at teardown.
#   PX4_EXTRA_PARAMS  extra PX4 parameters for this run only, as
#                  "NAME=VALUE NAME=VALUE" (diagnostics). Applied through
#                  PX4's own PX4_PARAM_<name> override; the stored parameter
#                  files are saved first and restored at teardown, and the
#                  effective values and the restoration are checked and
#                  written to <run_dir>/px4_param_state.txt.
#   HEADLESS       1 = no Gazebo GUI        (default: 1)
#   NVIDIA_OFFLOAD 1 = render on the NVIDIA GPU via PRIME offload (default: 0)
#   CAMERA_BRIDGE  1 = also bridge the camera to ROS (default: 0)
#   PERCEPTION     1 = start the debris detector, tracker, predictor and
#                  risk estimator (default: 0)
#   SURVIVOR       1 = start the survivor perception node (downward
#                  camera; needs CAMERA_BRIDGE=1). SURVIVOR_ARGS = extra ROS
#                  arguments. The VLM endpoint and model are taken from the
#                  environment (VLM_ENDPOINT, VLM_MODEL) if set.
#   LOCALIZATION   1 = start the survivor localization node (needs
#                  SURVIVOR=1). LOCALIZATION_ARGS = extra ROS arguments.
#   AVOIDANCE      on = start the avoidance planner; baseline = start it
#                  disabled (it logs but never acts); off (default)
#   AVOIDANCE_ARGS extra ROS arguments for the planner (e.g. "-p name:=v")
#   DETECTOR_ARGS, RISK_ARGS  extra ROS arguments for those nodes
#   CONTROLLER_ARGS extra ROS arguments for flight_controller
#                  (e.g. "-p target_altitude:=5.0")
#   VERIFY_ARGS    extra arguments for verify_ulog.py (e.g. "--target-alt 5")
#   SETTLE_SECONDS wait after PX4 start before anything else (default: 8)
#   PRE_HOOK       shell command run (to completion) before the controller
#   DURING_HOOK    shell command started in the background with the controller
#   TRIAL_HOOK     shell command run before teardown (RUN_DIR is exported)
#   POST_FAILSAFE_WAIT seconds to keep logging after a FAILSAFE (default: 20)
#   FLIGHT_TIMEOUT seconds to wait for the controller to finish (default: 150)
#
# flight_controller is the only process here that publishes to /fmu/in/*.
set -u

RUN_DIR=${1:?usage: run_flight_trial.sh <run_dir> [hover|waypoint]}
MODE=${2:-hover}
WS_DIR=$(cd "$(dirname "$0")/.." && pwd)
PX4_DIR=${PX4_DIR:-$(cd "$WS_DIR/../PX4-Autopilot" && pwd)}
WORLD=${WORLD:-collapsed_building_rescue}
WORLD_SDF=${WORLD_SDF:-$WS_DIR/src/uav_autonomy/worlds/$WORLD.sdf}
MODEL=${MODEL:-gz_x500_mono_cam_down}
# Project-owned UAV model (src/uav_autonomy/models). It is spawned by this
# script and PX4 attaches to it with the stock X500 airframe. "none" makes PX4
# spawn its own MODEL instead (the Phase 0/1 configuration).
PROJECT_MODEL=${PROJECT_MODEL:-x500_rescue}
MODELS_DIR=$WS_DIR/src/uav_autonomy/models
if [ "$PROJECT_MODEL" != "none" ] && [ -f "$MODELS_DIR/$PROJECT_MODEL/model.sdf" ]; then
    UAV_NAME=${PROJECT_MODEL}_0
    EXTRA_RESOURCE_PATH=${EXTRA_RESOURCE_PATH:-$MODELS_DIR}
    BRIDGE_CONFIG=${BRIDGE_CONFIG:-$WS_DIR/src/uav_autonomy/config/camera_bridge_rescue.yaml}
else
    PROJECT_MODEL=none
    UAV_NAME=${MODEL#gz_}_0
    BRIDGE_CONFIG=${BRIDGE_CONFIG:-$WS_DIR/src/uav_autonomy/config/camera_bridge.yaml}
fi
GZ_SERVER_CONFIG=${GZ_SERVER_CONFIG:-$WS_DIR/src/uav_autonomy/config/gz_server.config}
HEADLESS=${HEADLESS:-1}
NVIDIA_OFFLOAD=${NVIDIA_OFFLOAD:-0}
CAMERA_BRIDGE=${CAMERA_BRIDGE:-0}
FLIGHT_TIMEOUT=${FLIGHT_TIMEOUT:-150}
ROOTFS="$PX4_DIR/build/px4_sitl_default/rootfs"
# PX4 XRCE-DDS time synchronisation for this run (default 0 = off).
# With it off, PX4 timestamps on ROS are PX4's own clock, which in lockstep
# simulation equals Gazebo simulation time exactly (docs/sensor_integration.md).
# This is a simulation time base, not a real-hardware clock synchronisation.
# PX4 persists "param set", so its stored parameter files are saved before the
# run and restored at teardown; nothing is changed permanently.
# UXRCE_SYNCT=px4 leaves PX4's own setting and files untouched.
UXRCE_SYNCT=${UXRCE_SYNCT:-0}
[ "$UXRCE_SYNCT" = "px4" ] && UXRCE_SYNCT=
PARAM_BACKUP_DIR=$WS_DIR/.px4_param_backup
PX4_EXTRA_PARAMS=${PX4_EXTRA_PARAMS:-}
param_files_sha() {
    # One fingerprint of PX4's two stored parameter files.
    cat "$ROOTFS/parameters.bson" "$ROOTFS/parameters_backup.bson" 2>/dev/null | sha256sum | cut -d' ' -f1
}

mkdir -p "$RUN_DIR"
RUN_DIR=$(cd "$RUN_DIR" && pwd)
export RUN_DIR WS_DIR WORLD UAV_NAME

stale=$(ps -eo pid,comm,args | awk '$2 ~ /^(px4|MicroXRCEAgent|flight_controll|parameter_bridg)$/ || ($2 ~ /^(ruby|gz)/ && /gz sim/) || ($2 ~ /^python/ && /gcs_heartbeat|ros2 run uav_autonomy/) || $2 ~ /^(debris_(depth_de|tracker|predicto|risk|avoidanc)|survivor_percep|survivor_locali)/ || ($2 ~ /^python/ && /scripts\/(sensor_study|sensor_integration|perception)\/|scripts\/avoidance\/(step_command|depth_gate)/)' || true)
if [ -n "$stale" ]; then
    echo "REFUSING TO START: stale simulation processes are running:" >&2
    echo "$stale" >&2
    exit 3
fi

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"
set -u

if [ "$NVIDIA_OFFLOAD" = "1" ]; then
    export __NV_PRIME_RENDER_OFFLOAD=1
    export __GLX_VENDOR_LIBRARY_NAME=nvidia
    export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
fi

PIDS=()
sim_pids() {
    # PIDs of every simulation process, matched on the executable name.
    ps -eo pid,comm,args | awk '$2 ~ /^(px4|MicroXRCEAgent|flight_controll|parameter_bridg)$/ || ($2 ~ /^(ruby|gz)/ && /gz sim/) || ($2 ~ /^python/ && /gcs_heartbeat|ros2 run uav_autonomy/) || $2 ~ /^(debris_(depth_de|tracker|predicto|risk|avoidanc)|survivor_percep|survivor_locali)/ || ($2 ~ /^python/ && /scripts\/(sensor_study|sensor_integration|perception)\/|scripts\/avoidance\/(step_command|depth_gate)/) {print $1}'
}
restore_px4_params() {
    [ -d "$PARAM_BACKUP_DIR" ] || return
    for f in parameters.bson parameters_backup.bson; do
        [ -f "$PARAM_BACKUP_DIR/$f" ] && cp -p "$PARAM_BACKUP_DIR/$f" "$ROOTFS/$f"
    done
    if [ -f "$PARAM_BACKUP_DIR/sha_before" ]; then
        # Verify that PX4's stored parameters are back to what they were.
        local was now
        was=$(cat "$PARAM_BACKUP_DIR/sha_before")
        now=$(param_files_sha)
        {
            echo "stored_params_sha256_after_restore=$now"
            [ "$was" = "$now" ] && echo "restore_verified=yes" || echo "restore_verified=NO"
        } >> "$RUN_DIR/px4_param_state.txt"
    fi
    rm -rf "$PARAM_BACKUP_DIR"
}
cleanup() {
    (cd "$ROOTFS" && timeout 5 ../bin/px4-shutdown >/dev/null 2>&1)
    local pids
    pids=$(sim_pids)
    [ -n "$pids" ] && kill -INT $pids 2>/dev/null
    sleep 3
    pids=$(sim_pids)
    [ -n "$pids" ] && kill -9 $pids 2>/dev/null
    sleep 1
    restore_px4_params
}
trap cleanup EXIT
trap 'exit 130' INT TERM

before=$(ls -1t "$ROOTFS"/log/*/*.ulg 2>/dev/null | head -1)

{
    echo "date=$(date -Is)"
    echo "mode=$MODE world=$WORLD model=$MODEL project_model=$PROJECT_MODEL uav_name=$UAV_NAME headless=$HEADLESS nvidia_offload=$NVIDIA_OFFLOAD"
    echo "px4_commit=$(git -C "$PX4_DIR" rev-parse HEAD)"
    echo "ws_commit=$(git -C "$WS_DIR" rev-parse HEAD)"
} > "$RUN_DIR/trial_info.txt"

# A backup left behind means an earlier run was killed before it could
# restore PX4's parameter files: restore them now.
restore_px4_params
if [ -n "$UXRCE_SYNCT" ] || [ -n "$PX4_EXTRA_PARAMS" ]; then
    mkdir -p "$PARAM_BACKUP_DIR"
    cp -p "$ROOTFS/parameters.bson" "$ROOTFS/parameters_backup.bson" "$PARAM_BACKUP_DIR/"
fi
[ -n "$UXRCE_SYNCT" ] && export PX4_PARAM_UXRCE_DDS_SYNCT=$UXRCE_SYNCT
if [ -n "$PX4_EXTRA_PARAMS" ]; then
    param_files_sha > "$PARAM_BACKUP_DIR/sha_before"
    {
        echo "requested_overrides=$PX4_EXTRA_PARAMS"
        echo "stored_params_sha256_before=$(cat "$PARAM_BACKUP_DIR/sha_before")"
    } > "$RUN_DIR/px4_param_state.txt"
    for kv in $PX4_EXTRA_PARAMS; do
        export "PX4_PARAM_${kv%%=*}=${kv#*=}"
    done
fi
echo "px4_extra_params=${PX4_EXTRA_PARAMS:-none}" >> "$RUN_DIR/trial_info.txt"
echo "uxrce_dds_synct=${UXRCE_SYNCT:-px4-default}" >> "$RUN_DIR/trial_info.txt"
echo "[trial] starting Gazebo + PX4 (world=$WORLD)"
if [ -f "$WORLD_SDF" ]; then
    # Project-tracked world: start the Gazebo server ourselves, then attach
    # PX4 in standalone mode. Nothing has to be copied into the PX4 tree.
    (
        set +u
        # Models found here take precedence over PX4's (model:// lookups).
        [ -n "${EXTRA_RESOURCE_PATH:-}" ] && export GZ_SIM_RESOURCE_PATH="$EXTRA_RESOURCE_PATH"
        source "$ROOTFS/gz_env.sh"; set -u
        [ -f "$GZ_SERVER_CONFIG" ] && export GZ_SIM_SERVER_CONFIG_PATH="$GZ_SERVER_CONFIG"
        exec gz sim --verbose=3 -r -s "$WORLD_SDF"
    ) > "$RUN_DIR/gazebo.log" 2>&1 &
    PIDS+=($!)
    if [ "$HEADLESS" != "1" ]; then
        gz sim -g > "$RUN_DIR/gazebo_gui.log" 2>&1 &
        PIDS+=($!)
    fi
    if [ "$PROJECT_MODEL" != "none" ]; then
        # Spawn the project model, then let PX4 attach to it (no PX4 edits).
        for _ in $(seq 40); do
            gz service -l 2>/dev/null | grep -q "^/world/$WORLD/create$" && break
            sleep 0.5
        done
        gz service -s "/world/$WORLD/create" --reqtype gz.msgs.EntityFactory \
            --reptype gz.msgs.Boolean --timeout 5000 \
            --req "name: \"$UAV_NAME\", allow_renaming: false, sdf_filename: \"$MODELS_DIR/$PROJECT_MODEL/model.sdf\"" \
            > "$RUN_DIR/spawn.log" 2>&1
        sleep 1
        (
            cd "$ROOTFS" || exit 1
            PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=$WORLD PX4_SIM_MODEL=gz_x500 \
                PX4_GZ_MODEL_NAME=$UAV_NAME exec ../bin/px4 -d
        ) > "$RUN_DIR/px4.log" 2>&1 &
        PIDS+=($!)
    else
        (
            cd "$ROOTFS" || exit 1
            PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=$WORLD PX4_SIM_MODEL=$MODEL exec ../bin/px4 -d
        ) > "$RUN_DIR/px4.log" 2>&1 &
        PIDS+=($!)
    fi
else
    # World shipped inside PX4's Tools/simulation/gz/worlds.
    (
        cd "$ROOTFS" || exit 1
        [ "$HEADLESS" = "1" ] && export HEADLESS=1 || unset HEADLESS
        PX4_SIM_MODEL=$MODEL PX4_GZ_WORLD=$WORLD exec ../bin/px4 -d
    ) > "$RUN_DIR/px4.log" 2>&1 &
    PIDS+=($!)
fi

MicroXRCEAgent udp4 -p 8888 > "$RUN_DIR/xrce_agent.log" 2>&1 &
PIDS+=($!)

python3 "$WS_DIR/scripts/gcs_heartbeat.py" > "$RUN_DIR/gcs_heartbeat.log" 2>&1 &
PIDS+=($!)

# Wait for PX4 to finish its startup script.
for _ in $(seq 60); do
    grep -q 'Startup script returned successfully' "$RUN_DIR/px4.log" 2>/dev/null && break
    sleep 1
done
if ! grep -q 'Startup script returned successfully' "$RUN_DIR/px4.log"; then
    echo "[trial] PX4 did not finish starting; see $RUN_DIR/px4.log" >&2
    exit 4
fi
sleep "${SETTLE_SECONDS:-8}"   # let the EKF converge and the GCS link register

if [ -n "$PX4_EXTRA_PARAMS" ]; then
    # Read the values PX4 is actually running with.
    {
        echo "--- effective values in the running PX4"
        for kv in $PX4_EXTRA_PARAMS; do
            (cd "$ROOTFS" && timeout 5 ../bin/px4-param show "${kv%%=*}" 2>&1) | grep -E "^[x +*]*${kv%%=*} "
        done
    } >> "$RUN_DIR/px4_param_state.txt"
fi

if [ "$CAMERA_BRIDGE" = "1" ]; then
    ros2 run ros_gz_bridge parameter_bridge --ros-args \
        -p config_file:="$BRIDGE_CONFIG" \
        > "$RUN_DIR/camera_bridge.log" 2>&1 &
    PIDS+=($!)
fi

if [ "${PERCEPTION:-0}" = "1" ]; then
    # Perception nodes read sensor topics only and never publish to /fmu/in.
    ros2 run uav_autonomy debris_depth_detector ${DETECTOR_ARGS:-} \
        > "$RUN_DIR/debris_depth_detector.log" 2>&1 &
    PIDS+=($!)
    ros2 run uav_autonomy debris_tracker \
        > "$RUN_DIR/debris_tracker.log" 2>&1 &
    PIDS+=($!)
    ros2 run uav_autonomy debris_predictor \
        > "$RUN_DIR/debris_predictor.log" 2>&1 &
    PIDS+=($!)
    ros2 run uav_autonomy debris_risk ${RISK_ARGS:-} \
        > "$RUN_DIR/debris_risk.log" 2>&1 &
    PIDS+=($!)
    if [ "${AVOIDANCE:-off}" != "off" ]; then
        # Planner only proposes; flight_controller validates and commands.
        # AVOIDANCE=baseline runs it with enabled:=false (logs, never acts).
        avoid_enabled=true
        [ "$AVOIDANCE" = "baseline" ] && avoid_enabled=false
        ros2 run uav_autonomy debris_avoidance --ros-args \
            -p enabled:=$avoid_enabled ${AVOIDANCE_ARGS:-} \
            > "$RUN_DIR/debris_avoidance.log" 2>&1 &
        PIDS+=($!)
    fi
    # Four Python nodes starting at once can stall the simulator for longer
    # than the flight controller's 1 s telemetry guard; let them settle.
    sleep "${PERCEPTION_SETTLE_SECONDS:-6}"
fi

if [ "${SURVIVOR:-0}" = "1" ]; then
    # Reads the camera topics only; publishes detections, commands nothing.
    ros2 run uav_autonomy survivor_perception ${SURVIVOR_ARGS:-} \
        > "$RUN_DIR/survivor_perception.log" 2>&1 &
    PIDS+=($!)
    sleep 3
fi

if [ "${LOCALIZATION:-0}" = "1" ]; then
    echo "[trial] starting survivor localization"
    ros2 run uav_autonomy survivor_localization ${LOCALIZATION_ARGS:-} \
        > "$RUN_DIR/survivor_localization.log" 2>&1 &
    PIDS+=($!)
    sleep 1
fi

if [ -n "${PRE_HOOK:-}" ]; then
    bash -c "$PRE_HOOK" > "$RUN_DIR/pre_hook.log" 2>&1
fi

if [ "$MODE" != "none" ]; then
echo "[trial] starting flight_controller ($MODE)"
waypoint=false
[ "$MODE" = "waypoint" ] && waypoint=true
ros2 run uav_autonomy flight_controller --ros-args \
    -p waypoint_mission:=$waypoint ${CONTROLLER_ARGS:-} \
    > "$RUN_DIR/flight_controller.log" 2>&1 &
PIDS+=($!)
if [ -n "${DURING_HOOK:-}" ]; then
    bash -c "$DURING_HOOK" > "$RUN_DIR/during_hook.log" 2>&1 &
fi

outcome=TIMEOUT
for _ in $(seq "$FLIGHT_TIMEOUT"); do
    if grep -q 'FLIGHT COMPLETE' "$RUN_DIR/flight_controller.log"; then
        outcome=COMPLETE; break
    fi
    if grep -q 'FAILSAFE' "$RUN_DIR/flight_controller.log"; then
        outcome=FAILSAFE; sleep "${POST_FAILSAFE_WAIT:-20}"; break
    fi
    sleep 1
done
echo "[trial] controller outcome: $outcome"
echo "controller_outcome=$outcome" >> "$RUN_DIR/trial_info.txt"
fi
[ -n "${TRIAL_HOOK:-}" ] && bash -c "$TRIAL_HOOK"
sleep 3

cp "$HOME/.gz/rendering/ogre2.log" "$RUN_DIR/ogre2.log" 2>/dev/null
cleanup
trap - EXIT

if [ "$MODE" = "none" ]; then
    exit 0
fi

after=$(ls -1t "$ROOTFS"/log/*/*.ulg 2>/dev/null | head -1)
if [ -z "$after" ] || [ "$after" = "$before" ]; then
    echo "[trial] no new ULog was produced" >&2
    exit 5
fi
cp "$after" "$RUN_DIR/flight.ulg"

verify_args=(--mode "$MODE" --output "$RUN_DIR/verification.json")
[ -n "${VERIFY_ARGS:-}" ] && verify_args+=($VERIFY_ARGS)
[ "$MODE" = "waypoint" ] && verify_args+=("--waypoints=${WAYPOINTS:--4,-4;-4,-5.5;0,0}")
python3 "$WS_DIR/scripts/verify_ulog.py" "${verify_args[@]}" "$RUN_DIR/flight.ulg"
status=$?
echo "ulog_verification_exit=$status" >> "$RUN_DIR/trial_info.txt"
exit $status
