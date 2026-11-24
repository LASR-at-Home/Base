#!/bin/bash
# Runs one person-following scenario in simulation and scores it (sim testing only).
#
# Run INSIDE the container, with the sim (nav.launch.py), sim_stubs and the follower's
# support services already running:
#   run_scenario.sh <scenario.yaml> <follower> <run_name> [out_dir]
#
# <follower> is a shell command that starts the follower under test, e.g.
#   "ros2 run skills follow_person"
# It is started after the robot is reset and the people are spawned, and is killed
# when the evaluator finishes.
#
# Env: START_DELAY  seconds between starting the follower and the host walking (25)
#      MAX_DURATION evaluator timeout in seconds (240)
set -u
SCENARIO=$1
FOLLOWER=$2
RUN=$3
OUT=${4:-/tmp/follow_eval}
START_DELAY=$(python3 -c "print(float(${START_DELAY:-25}))")
MAX_DURATION=$(python3 -c "print(float(${MAX_DURATION:-240}))")
mkdir -p "$OUT/$RUN"
LOG="$OUT/$RUN"

pids=()  # each background job runs in its own process group (setsid)
cleanup() {
    for p in "${pids[@]}"; do kill -INT -- "-$p" 2>/dev/null; done
    sleep 3
    for p in "${pids[@]}"; do kill -KILL -- "-$p" 2>/dev/null; done
    # stop the base in case the follower died mid-motion (a leftover Nav2 goal is
    # pre-empted by reset_episode at the start of the next run)
    timeout 3 ros2 topic pub -t 5 /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1
}
trap cleanup EXIT

echo "[run] resetting robot"
# hard timeout: a robot stuck in the sim must not hang a whole batch
if ! timeout 300 ros2 run lasr_person_following reset_episode > "$LOG/reset.log" 2>&1; then
    echo '{"error": "reset failed"}'
    exit 1
fi

# people listed in the scenario file
PEOPLE=$(python3 -c "import yaml,sys; print(' '.join(yaml.safe_load(open('$SCENARIO')).keys()))")
DISTRACTOR=""
for name in $PEOPLE; do
    mesh=standing
    if [ "$name" != "test_person" ]; then mesh=actor; DISTRACTOR=$name; fi
    setsid ros2 run lasr_person_following scripted_person --ros-args \
        -p name:=$name -p mesh:=$mesh -p scenario:=$SCENARIO \
        -p start_delay:=$START_DELAY -p use_sim_time:=true > "$LOG/person_$name.log" 2>&1 &
    pids+=($!)
done
sleep 4  # let the people spawn before the follower looks for the host

echo "[run] starting follower: $FOLLOWER"
setsid bash -c "$FOLLOWER" > "$LOG/follower.log" 2>&1 &
pids+=($!)

echo "[run] evaluating"
EVAL_ARGS=(-p run_name:=$RUN -p out_dir:=$OUT -p max_duration:=$MAX_DURATION -p use_sim_time:=true)
[ -n "$DISTRACTOR" ] && EVAL_ARGS+=(-p distractor:=$DISTRACTOR)
[ -n "${ESTIMATE_TOPIC:-}" ] && EVAL_ARGS+=(-p estimate_topic:=$ESTIMATE_TOPIC)
ros2 run lasr_person_following follow_evaluator --ros-args "${EVAL_ARGS[@]}" > "$LOG/evaluator.log" 2>&1
grep -h SUMMARY "$LOG/evaluator.log" | sed 's/.*SUMMARY //'
