#!/usr/bin/env bash
#   ./monitor.sh                 # report completo
#   ./monitor.sh --watch-bt      # in più: cattura 20s di /behavior_tree_log
#                                #  (lancialo MENTRE il robot naviga/spinna!)

set -uo pipefail

REPORT="nav_report_$(date +%Y%m%d_%H%M%S).txt"
WATCH_BT=false
[ "${1:-}" = "--watch-bt" ] && WATCH_BT=true

log() { echo -e "$@" | tee -a "$REPORT"; }
section() { log "\n============================================================"; log "== $1"; log "============================================================"; }
run() {
  # run <descrizione> <timeout_sec> <comando...>
  local desc="$1"; local tmo="$2"; shift 2
  log "\n--- $desc"
  log "\$ $*"
  timeout "$tmo" "$@" 2>&1 | head -60 | tee -a "$REPORT"
  local rc=${PIPESTATUS[0]}
  [ "$rc" = "124" ] && log "[TIMEOUT dopo ${tmo}s]"
  [ "$rc" != "0" ] && [ "$rc" != "124" ] && log "[exit code: $rc]"
  return 0
}

log "NAV DIAGNOSTIC REPORT — $(date)"
log "host: $(hostname)  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"

# ------------------------------------------------------------------ 1. NODI
section "1. NODI ATTIVI (chi sta girando?)"
run "Tutti i nodi di navigazione" 10 bash -c "ros2 node list | grep -E 'amcl|bt_nav|planner|controller_server|behavior_server|map_server|smoother|velocity|lifecycle|twist_mux|mobile_base|laser|direct_laser' | sort"
run "DOPPIONI? (stesso nodo due volte = due stack attivi!)" 10 bash -c "ros2 node list | sort | uniq -d"

# ------------------------------------------------------------ 2. LIFECYCLE
section "2. STATO LIFECYCLE (devono essere 'active')"
for n in /bt_navigator /planner_server /controller_server /behavior_server /amcl /map_server; do
  run "lifecycle $n" 8 ros2 lifecycle get "$n"
done

# ------------------------------------------------------------------ 3. ACTION
section "3. ACTION SERVER"
run "Action di navigazione presenti" 10 bash -c "ros2 action list | grep -E 'navigate|follow|spin|backup|compute'"

# --------------------------------------------------------------- 4. SENSORI
section "4. SENSORI E TOPIC VITALI"
run "Topic scan disponibili" 10 bash -c "ros2 topic list | grep -i scan"
run "Frequenza /scan_raw (5s)" 8 ros2 topic hz /scan_raw --window 20
run "Frequenza /scan (5s, se esiste)" 8 ros2 topic hz /scan --window 20

# ------------------------------------------------------------------ 5. TF
section "5. TF (map->odom->base_footprint)"
run "map -> base_footprint" 8 bash -c "ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | head -8"
run "odom -> base_footprint (da fermo NON deve vibrare)" 8 bash -c "ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | head -8"

# ---------------------------------------------------------- 6. PARAMETRI CHIAVE
section "6. PARAMETRI CHIAVE (planner/controller/costmap/BT)"
run "Planner plugins" 8 ros2 param get /planner_server planner_plugins
run "Planner type" 8 ros2 param get /planner_server GridBased.plugin
run "Planner tolerance" 8 ros2 param get /planner_server GridBased.tolerance
run "Controller plugins" 8 ros2 param get /controller_server controller_plugins
run "Controller type (DWB? MPPI?)" 8 ros2 param get /controller_server FollowPath.plugin
run "BT XML in uso" 8 ros2 param get /bt_navigator default_nav_to_pose_bt_xml
run "GLOBAL robot_radius" 8 ros2 param get /global_costmap/global_costmap robot_radius
run "GLOBAL inflation_radius" 8 ros2 param get /global_costmap/global_costmap inflation_layer.inflation_radius
run "GLOBAL cost_scaling" 8 ros2 param get /global_costmap/global_costmap inflation_layer.cost_scaling_factor
run "GLOBAL layers" 8 ros2 param get /global_costmap/global_costmap plugins
run "LOCAL inflation_radius" 8 ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius
run "LOCAL layers" 8 ros2 param get /local_costmap/local_costmap plugins

# --------------------------------------------------------- 7. CATENA VELOCITÀ
section "7. CATENA cmd_vel (controller -> smoother -> twist_mux -> base)"
run "Topic cmd_vel esistenti" 10 bash -c "ros2 topic list | grep -i -E 'cmd_vel|nav_vel'"
run "twist_mux: input/output" 10 bash -c "ros2 node info /twist_mux 2>/dev/null | sed -n '/Subscribers/,/Publishers/p'"
run "Chi pubblica il comando base" 10 bash -c "for t in /cmd_vel /nav_vel /mobile_base_controller/cmd_vel /mobile_base_controller/cmd_vel_unstamped; do echo \"== \$t\"; ros2 topic info \$t 2>/dev/null | grep -E 'Type|count'; done"

# ------------------------------------------------------------ 8. LOCALIZZAZIONE
section "8. LOCALIZZAZIONE"
run "AMCL: posa stimata (1 msg)" 8 bash -c "ros2 topic echo /amcl_pose --once 2>/dev/null | head -20"
run "Particelle: topic presente?" 8 bash -c "ros2 topic list | grep -E 'particle'"

# ------------------------------------------------------- 9. COSTMAP SNAPSHOT
section "9. COSTMAP (pubblicano?)"
run "Global costmap freq (5s)" 8 ros2 topic hz /global_costmap/costmap --window 5
run "Local costmap freq (5s)" 8 ros2 topic hz /local_costmap/costmap --window 5

# ----------------------------------------------------------- 10. BT LOG LIVE
if $WATCH_BT; then
  section "10. BEHAVIOR TREE LOG (20s) — cosa fallisce prima di Spin?"
  log "(cattura in corso: manda ORA il goal / riproduci il problema...)"
  run "behavior_tree_log (20s)" 22 bash -c "timeout 20 ros2 topic echo /behavior_tree_log 2>/dev/null | grep -E 'node_name|current_status|previous_status' | head -200"
else
  section "10. BEHAVIOR TREE LOG — saltato"
  log "Rilancia con:  ./monitor.sh --watch-bt   MENTRE il robot naviga/spinna."
fi

section "FINE REPORT"
log "\nReport salvato in: $REPORT"
log "Invia questo file per la diagnosi."
