#!/usr/bin/env bash
# Fly every Phase 6 scenario, baseline and avoidance, one after another.
# Usage: scripts/avoidance/run_all.sh [variant ...]   (default: both)
WS_DIR=$(cd "$(dirname "$0")/../.." && pwd)
SCENARIOS=${SCENARIOS:-"A_none B_safe C_direct D_near_miss E_multiple F_lateral G_climbing H_transit I_search J_shaft J_late seed_11 seed_22 seed_33"}
for variant in ${@:-baseline avoid}; do
    for scenario in $SCENARIOS; do
        echo "=== $(date +%T) $variant $scenario"
        "$WS_DIR/scripts/avoidance/run_scenario.sh" "$scenario" "$variant" | head -3
        sleep 3
    done
done
echo "=== ALL DONE $(date +%T)"
