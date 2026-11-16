#!/usr/bin/env bash
# Build a patched Duckiematrix engine image under its own tag, leaving the
# stock image untouched.
#
# It starts a throwaway container from the stock engine image, applies all six
# project fixes with scripts/apply_matrix_bridge_fixes.sh (which skips patches
# that are already applied), and commits the result with the original
# entrypoint and command restored.
#
# Run it inside the workspace devcontainer from the project root:
#   bash scripts/build_patched_engine_image.sh                # builds ...-overwatch only
#   bash scripts/build_patched_engine_image.sh --as-default   # and makes it the image dts runs
#
# `dts matrix run --container-image` cannot be used here: with the native Mac
# renderer, dts refuses it on an ARM64 Linux host. So --as-default saves the
# stock image as ...-stock (once) and points the canonical tag at the patched
# image. Then always start the Matrix with --no-pull, otherwise dts may
# replace the canonical tag with a newer stock image:
#   dts matrix run --standalone --no-pull --map <map>
# To undo: docker tag duckietown/dt-duckiematrix:ente-arm64v8-stock duckietown/dt-duckiematrix:ente-arm64v8
set -euo pipefail

canonical="duckietown/dt-duckiematrix:ente-arm64v8"
stock_image="${canonical}-stock"
patched_image="${PATCHED_IMAGE:-${canonical}-overwatch}"
as_default=false
[ "${1:-}" = "--as-default" ] && as_default=true

# Always patch the stock image, never an already-patched canonical tag.
if docker image inspect "${stock_image}" >/dev/null 2>&1; then
    base_image="${stock_image}"
else
    base_image="${canonical}"
fi
base_image="${BASE_IMAGE:-${base_image}}"
work="ow-engine-patch-$$"

cleanup() { docker rm -f "${work}" >/dev/null 2>&1 || true; }
trap cleanup EXIT

docker image inspect "${base_image}" >/dev/null
entrypoint="$(docker image inspect "${base_image}" --format '{{json .Config.Entrypoint}}')"
cmd="$(docker image inspect "${base_image}" --format '{{json .Config.Cmd}}')"

docker run -d --name "${work}" --entrypoint sleep "${base_image}" infinity >/dev/null
bash "$(dirname "$0")/apply_matrix_bridge_fixes.sh" "${work}"
docker commit \
    --change "ENTRYPOINT ${entrypoint}" \
    --change "CMD ${cmd}" \
    "${work}" "${patched_image}" >/dev/null

echo "Built ${patched_image} from ${base_image}."

if ${as_default}; then
    if ! docker image inspect "${stock_image}" >/dev/null 2>&1; then
        docker tag "${base_image}" "${stock_image}"
        echo "Saved the stock engine as ${stock_image}."
    fi
    docker tag "${patched_image}" "${canonical}"
    echo "${canonical} now points at the patched engine. Start the Matrix with --no-pull."
fi
