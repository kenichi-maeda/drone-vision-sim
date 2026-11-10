# syntax=docker/dockerfile:1.4
ARG ARCH
ARG DISTRO=ente
ARG DOCKER_REGISTRY=docker.io
FROM ${DOCKER_REGISTRY}/duckietown/dt-ros2-commons:${DISTRO}-${ARCH}

ARG DISTRO
ARG LAUNCHER=default
ARG TARGETARCH

ENV DT_MODULE_TYPE="exercise" \
    DT_MODULE_NAME="drone-vision-sim" \
    DT_MODULE_DESCRIPTION="Read-only Duckiedrone camera probe" \
    DT_MAINTAINER="Drone Vision Project" \
    DT_REPO_PATH="/code/src/drone-vision-sim" \
    DT_LAUNCH_PATH="/drone-vision-sim" \
    DT_LAUNCHER="${LAUNCHER}" \
    ROS_DOMAIN_ID=42

RUN mkdir -p "${DT_REPO_PATH}" "${DT_LAUNCH_PATH}"
WORKDIR "${DT_REPO_PATH}"

# OpenCV's ArUco module includes the AprilTag 36h11 detector. NumPy decodes
# the ROS compressed-image byte buffer without writing image files to disk.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-numpy \
    python3-opencv \
    ros-${ROS2_DISTRO}-mavros-msgs \
    && rm -rf /var/lib/apt/lists/*

COPY --from=meat ./packages/. "${DT_REPO_PATH}/packages/"
COPY --from=meat ./assets/yolov8n.onnx "${DT_REPO_PATH}/assets/yolov8n.onnx"
RUN . /opt/ros/${ROS2_DISTRO}/setup.sh && dt-colcon-build "${COLCON_WS}"

COPY --from=recipe ./launchers/. "${DT_LAUNCH_PATH}/"
RUN dt-install-launchers "${DT_LAUNCH_PATH}"
