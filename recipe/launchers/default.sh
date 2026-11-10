#!/bin/bash

source /environment.sh
dt-launchfile-init
source /opt/ros/${ROS2_DISTRO}/setup.bash
source ${COLCON_WS}/install/setup.bash --extend

# Optional per-experiment target selection. Edit packages/drone_vision/target.env in the project
# (bind-mounted into the workbench; YOLO_TARGET_CLASS_ID, YOLO_ALLOWED_CLASS_IDS) and restart the workbench.
if [ -f /code/catkin_ws/src/drone-vision-sim/packages/drone_vision/target.env ]; then
    source /code/catkin_ws/src/drone-vision-sim/packages/drone_vision/target.env
fi

# These nodes are deliberately perception-only: they subscribe to the camera
# and publish no MAVROS topic or flight command.
dt-set-module-healthy
dt-exec ros2 run drone_vision camera_probe
dt-exec ros2 run drone_vision yolo_detector --ros-args \
    -p model_path:=/code/src/drone-vision-sim/assets/yolov8n.onnx \
    -p target_class_id:="${YOLO_TARGET_CLASS_ID:--1}" \
    -p allowed_class_ids:="[${YOLO_ALLOWED_CLASS_IDS:-11,32}]" \
    -p confidence_threshold:="${YOLO_CONFIDENCE_THRESHOLD:-0.35}"
# Flight stays disabled unless explicitly enabled. The controller's YOLO
# search, centering, and approach options have short, validated defaults.
dt-exec ros2 run drone_vision safe_hover --ros-args \
    -p enabled:="${HOVER_ENABLED:-false}" \
    -p handoff_to_offboard:=true \
    -p object_search_enabled:=true \
    -p object_centering_enabled:=true \
    -p object_approach_enabled:=true
dt-launchfile-join
