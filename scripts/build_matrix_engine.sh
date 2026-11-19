#!/usr/bin/env bash
set -euo pipefail

case "$(uname -m)" in
  x86_64) dt_arch="amd64" ;;
  aarch64|arm64) dt_arch="arm64v8" ;;
  *) echo "unsupported architecture $(uname -m)" >&2; exit 1 ;;
esac
base_image="duckietown/dt-duckiematrix:ente-${dt_arch}"
backup_image="duckietown/dt-duckiematrix:ente-${dt_arch}-unpatched"
patched_image="duckietown/dt-duckiematrix:ente-${dt_arch}-yolo-patched"

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
