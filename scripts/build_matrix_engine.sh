#!/usr/bin/env bash
set -euo pipefail

base_image="duckietown/dt-duckiematrix:ente-arm64v8"
backup_image="duckietown/dt-duckiematrix:ente-arm64v8-unpatched"
patched_image="duckietown/dt-duckiematrix:ente-arm64v8-yolo-patched"

if ! docker image inspect "${backup_image}" >/dev/null 2>&1; then
    docker image inspect "${base_image}" >/dev/null
    docker tag "${base_image}" "${backup_image}"
fi

docker build \
    --build-arg "MATRIX_BASE_IMAGE=${backup_image}" \
    --file recipe/matrix-engine/Dockerfile \
    --tag "${patched_image}" \
    .

# dts matrix run uses the canonical distro image name. Keep the unmodified
# image available under ${backup_image} so this can be reversed with docker tag.
docker tag "${patched_image}" "${base_image}"
echo "Built patched Matrix engine. Original image is preserved as ${backup_image}."
