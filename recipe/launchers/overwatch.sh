#!/bin/bash
# OVERWATCH drone side: start only the link to the operator console.
#   dts code workbench --recipe /home/ubuntu/drone-vision-sim/recipe -R <robot> -L overwatch
#
# YOLO runs on the console's machine, not here, and safe_hover is deliberately
# not started: the link is the only node that sends flight setpoints.

source /environment.sh
dt-launchfile-init
source /opt/ros/${ROS2_DISTRO}/setup.bash
source ${COLCON_WS}/install/setup.bash --extend

dt-set-module-healthy
dt-exec ros2 run drone_vision overwatch_link --ros-args \
    -p port:="${OVERWATCH_LINK_PORT:-8765}" \
    -p takeoff_altitude_m:="${OVERWATCH_TAKEOFF_ALTITUDE_M:-0.8}"
dt-launchfile-join
