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
# OpenCV color detector (COLOR_DETECTOR=true in target.env). Perception only.
if [ "${COLOR_DETECTOR:-true}" = "true" ]; then
    dt-exec ros2 run drone_vision color_detector --ros-args \
        -p min_area_fraction:="${COLOR_MIN_AREA_FRACTION:-0.002}"
fi
# Optional live browser view of an annotated stream (CAMERA_VIEWER=true in
# target.env). CAMERA_VIEWER_STREAM picks "yolo" (objects + their colors) or
# "color" (OpenCV color blobs). Viewer only; the detectors encode frames only
# while it is running.
if [ "${CAMERA_VIEWER:-false}" = "true" ]; then
    if [ "${CAMERA_VIEWER_STREAM:-yolo}" = "color" ]; then
        VIEWER_TOPIC=/drone_vision/color_image/compressed
    else
        VIEWER_TOPIC=/drone_vision/yolo_image/compressed
    fi
    dt-exec ros2 run drone_vision camera_view --ros-args \
        -p port:="${CAMERA_VIEWER_PORT:-8090}" \
        -p image_topic:="${VIEWER_TOPIC}"
fi
# Flight stays disabled unless explicitly enabled. The controller's YOLO
# search, centering, and approach options have short, validated defaults.
dt-exec ros2 run drone_vision safe_hover --ros-args \
    -p enabled:="${HOVER_ENABLED:-false}" \
    -p handoff_to_offboard:=true \
    -p object_search_enabled:=true \
    -p object_centering_enabled:=true \
    -p object_approach_enabled:=true
dt-launchfile-join
