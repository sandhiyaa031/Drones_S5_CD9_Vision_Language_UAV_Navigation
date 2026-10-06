#!/usr/bin/env bash
# Phase 6.1 acceptance re-run and experiments (results/phase6_1/).
WS_DIR=$(cd "$(dirname "$0")/../.." && pwd)
export RESULTS_ROOT=$WS_DIR/results/phase6_1
R="$WS_DIR/scripts/avoidance/run_scenario.sh"
run() { echo "=== $(date +%T) $*"; "$R" "$@" | head -2; sleep 3; }
for s in T_29 T_35 T_40 F2_lateral; do run $s baseline; done
for s in A_none B_safe D_near_miss E_multiple G_climbing I_search F_lateral J_shaft J_late seed_11 seed_22 seed_33; do run $s avoid; done
for t in "" r2 r3; do run F2_lateral avoid $t; done
for s in T_29 T_35 T_40 T_44; do run $s avoid; run $s avoid r2; done
for s in GAP_100_0 GAP_250_0 GAP_750_0 GAP_500_400 GAP_750_400; do run $s avoid; done
run H_transit avoid; run H_transit avoid r2
RISK_ARGS="--ros-args -p uav_motion_model:=response" run H_transit avoid_risk_response
RISK_ARGS="--ros-args -p uav_motion_model:=response" run H_transit avoid_risk_response r2
RISK_ARGS="--ros-args -p uav_motion_model:=constant_acceleration" run H_transit avoid_risk_ca
echo "=== ALL DONE $(date +%T)"
