#!/bin/bash

source /environment.sh
dt-launchfile-init
source /opt/ros/${ROS2_DISTRO}/setup.bash
source ${COLCON_WS}/install/setup.bash --extend

# Optional per-experiment target selection. Edit packages/drone_vision/target.env in the project
# (bind-mounted into the workbench; YOLO_TARGET_CLASS_ID, YOLO_ALLOWED_CLASS_IDS) and restart the workbench.
MOUNTED_PACKAGE=/code/catkin_ws/src/drone-vision-sim/packages/drone_vision
IMAGE_PACKAGE=/code/src/drone-vision-sim/packages/drone_vision
if [ -f "${MOUNTED_PACKAGE}/target.env" ]; then
    source "${MOUNTED_PACKAGE}/target.env"
fi
# The image copy is what Python imports. The workbench mount is newer when
# the package was edited after the image was built, so copy it over at start.
if [ -f "${MOUNTED_PACKAGE}/drone_vision/camera_view.py" ]; then
    cp -a "${MOUNTED_PACKAGE}/." "${IMAGE_PACKAGE}/"
    rm -rf "${IMAGE_PACKAGE}/drone_vision/__pycache__"
fi

# These nodes are deliberately perception-only: they subscribe to the camera
# and publish no MAVROS topic or flight command.
dt-set-module-healthy
# Keep the detector off the cores the Matrix physics loop needs.
export OMP_NUM_THREADS=2
export OPENBLAS_NUM_THREADS=2
export OPENCV_FOR_THREADS_NUM=2
dt-exec ros2 run drone_vision camera_probe
dt-exec ros2 run drone_vision yolo_detector --ros-args \
    -p model_path:=/code/src/drone-vision-sim/assets/yolov8n.onnx \
    -p target_class_id:="${YOLO_TARGET_CLASS_ID:--1}" \
    -p allowed_class_ids:="[${YOLO_ALLOWED_CLASS_IDS:-11,32}]" \
    -p confidence_threshold:="${YOLO_CONFIDENCE_THRESHOLD:-0.35}"
# Optional live browser view of the annotated YOLO stream (CAMERA_VIEWER=true in
# target.env). Viewer only; the detector encodes frames only while it is running.
if [ "${CAMERA_VIEWER:-false}" = "true" ]; then
    dt-exec ros2 run drone_vision camera_view --ros-args \
        -p port:="${CAMERA_VIEWER_PORT:-8090}"
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
