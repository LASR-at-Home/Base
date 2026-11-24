#!/bin/bash
# Starts (or restarts) everything the follow tests need, headless (uses the GPU via
# apptainer --nv when there is one, see sim_env.sh):
#   Gazebo + Nav2 (nav.launch.py) with the follow-test Nav2 params, sim stubs
#   (point_head_action, transcribe_speech), the YOLO service and play_motion motions.
# Prints READY when done. Usage: start_sim_stack.sh
source "$(dirname "$0")/sim_env.sh"
SIM_PARAMS=$WS/install/simulation/share/simulation/config/nav2_params_scan_raw.yaml
# The sim's DWB max speed (0.26 m/s) is below walking pace and Humble's DWB can only
# lower its speed limits at runtime, not raise them above the configured value. Test
# with a configured max of 0.7; reset_episode lowers it to 0.26 for normal driving and
# follow_manager raises it again while following (exactly what happens on a robot whose
# configured max is above its normal driving speed).
# SIM_MAX_SPEED: configured maximum (follow_max_speed must not exceed it)
V=${SIM_MAX_SPEED:-0.7}
sed -e "s/max_vel_x: 0.26/max_vel_x: $V/" -e "s/max_speed_xy: 0.26/max_speed_xy: $V/" \
    -e "s/max_velocity: \[0.26, 0.0, 1.0\]/max_velocity: [$V, 0.0, 1.0]/" \
    -e "s/min_velocity: \[-0.26, 0.0, -1.0\]/min_velocity: [-$V, 0.0, -1.0]/" "$SIM_PARAMS" > "$FAST_NAV_PARAMS"

# Stop everything from a previous run. Leftover nodes (e.g. a second play_motion2) break
# the new stack, so kill every ROS process, not just the launch.
pkill -INT -f "ros2 launch [s]imulation"
pkill -f "lib/lasr_person_following/[a-z]"; pkill -f "lib/lasr_vision_yolo/[y]olo_service_node"
for i in $(seq 1 30); do pgrep -f "[g]zserver" >/dev/null || break; sleep 1; done
# anchored (^): match processes that ARE ROS executables, not shells that mention the paths
pkill -9 -f "^gzserver|^(/usr/bin/python3 )?(/opt/ros/humble/lib/|/opt/robocup_ws/install/|$WS/install/)|^/usr/bin/python3 /opt/ros/humble/bin/ros2 "
$APPTAINER bash -c "$ROS_SETUP; ros2 daemon stop" > /dev/null 2>&1
sleep 3

cd "$WS"
nohup $APPTAINER bash -c "$ROS_SETUP; ros2 launch simulation nav.launch.py gzclient:=False nav_params:=$FAST_NAV_PARAMS" > "$OUT/sim.log" 2>&1 &
for i in $(seq 1 120); do [ "$(grep -c 'Managed nodes are active' "$OUT/sim.log")" -ge 2 ] && break; sleep 3; done
sleep 10
pkill -f "lib/rviz2/[r]viz2"  # rviz is useless headless and costs a lot of CPU
nohup $APPTAINER bash -c "$ROS_SETUP; ros2 run lasr_person_following sim_stubs" > "$OUT/stubs.log" 2>&1 &
# without a GPU: 4 torch threads, more oversubscribes the CPU next to gzserver
nohup $APPTAINER bash -c "$ROS_SETUP; cd $WS; OMP_NUM_THREADS=4 ros2 launch lasr_vision_yolo service_launch.xml" > "$OUT/yolo.log" 2>&1 &
nohup $APPTAINER bash -c "$ROS_SETUP; ros2 launch skills load_motions.launch.py" > "$OUT/motions.log" 2>&1 &
for i in $(seq 1 60); do grep -q "YOLO service started" "$OUT/yolo.log" && grep -q "Sim stubs ready" "$OUT/stubs.log" && break; sleep 2; done
# the skill needs these: don't report READY on a half-started stack
if ! $APPTAINER bash -c "$ROS_SETUP; for i in \$(seq 1 30); do a=\$(ros2 action list 2>/dev/null); echo \"\$a\" | grep -qx /play_motion2 && echo \"\$a\" | grep -qx /navigate_to_pose && exit 0; sleep 2; done; exit 1" 2>/dev/null; then
  echo FAILED: /play_motion2 or /navigate_to_pose missing
  exit 1
fi
echo READY
