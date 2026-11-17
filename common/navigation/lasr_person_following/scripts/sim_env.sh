# Shared settings for the simulation test scripts. Override any of these in the
# environment. Everything runs inside the ros2.sif container.
WS=${WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../../.." && pwd)}   # workspace root
SIF=${SIF:-$WS/src/Base/ros2.sif}
OUT=${OUT:-/tmp/follow_eval}            # logs and results
FAST_NAV_PARAMS=${FAST_NAV_PARAMS:-$OUT/nav2_params_fast.yaml}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
mkdir -p "$OUT"
# bind /run/user so gzserver can render cameras on the X display (XAUTHORITY lives there)
APPTAINER="apptainer exec --bind /run/user/$(id -u) $SIF"
ROS_SETUP="export LIBGL_ALWAYS_SOFTWARE=1; export ROS_DOMAIN_ID=$ROS_DOMAIN_ID; source /opt/ros/humble/setup.bash; source /opt/robocup_ws/install/setup.bash; source $WS/install/setup.bash"
