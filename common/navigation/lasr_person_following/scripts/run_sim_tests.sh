#!/bin/bash
# Runs the FollowPerson skill on simulation scenarios and prints a summary per run.
#   run_sim_tests.sh <prefix> [scenario ...]     (default: all scenarios)
# Needs start_sim_stack.sh first. A run whose reset fails (robot stuck in the sim)
# restarts the sim stack and is retried once. Results: $OUT/<prefix>_<sN>/
source "$(dirname "$0")/sim_env.sh"
PREFIX=${1:-run}; shift
SCEN_DIR=$WS/install/lasr_person_following/share/lasr_person_following/config/scenarios
SCENARIOS=("$@")
[ ${#SCENARIOS[@]} -eq 0 ] && SCENARIOS=($(ls "$SCEN_DIR" | sed 's/\.yaml$//'))
FOLLOWER="ros2 launch lasr_person_following person_following.launch.py use_sim_time:=true & sleep 3; ros2 run skills follow_person; wait"
for sc in "${SCENARIOS[@]}"; do
  run=${PREFIX}_${sc%%_*}
  echo "== $sc"
  for try in 1 2; do
    (cd "$WS"; $APPTAINER bash -c "$ROS_SETUP; ESTIMATE_TOPIC=/person_tracker/host_point MAX_DURATION=180 START_DELAY=12 \
      $WS/install/lasr_person_following/share/lasr_person_following/scripts/run_scenario.sh $SCEN_DIR/$sc.yaml '$FOLLOWER' $run $OUT") > "$OUT/last_run.txt" 2>&1
    grep -q '"error": "reset failed"' "$OUT/last_run.txt" || break
    echo "   reset failed: restarting the sim stack"; "$(dirname "$0")/start_sim_stack.sh" > /dev/null
  done
  tr -d '\n ' < "$OUT/$run/summary.json"; echo
  grep -h -E "FollowPerson: (host stationary|host lost)|PROCESS_RESPONSE' :|no progress" "$OUT/$run/follower.log" | sed 's/.*\]: //' | cut -c1-150
  (cd "$WS"; $APPTAINER $WS/install/lasr_vision_yolo/share/lasr_vision_yolo/venv/bin/python \
      $WS/install/lasr_person_following/share/lasr_person_following/scripts/plot_run.py "$OUT/$run" 2>/dev/null)
done
