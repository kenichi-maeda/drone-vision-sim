#!/usr/bin/env bash
# Run the project workbench on a virtual robot, using this repo's own recipe.
# Run it inside the workspace devcontainer:
#   bash scripts/workbench.sh                  # robot birdie, launcher overwatch
#   bash scripts/workbench.sh birdie default   # the original YOLO + safe_hover launcher
# It builds the project image on the robot first (cached after the first time).
#
# Why this wrapper exists: `dts code workbench --recipe <dir>` uses that recipe
# only for its first steps. The `devel run` step it calls next reloads the
# project from .dtproject and looks for the recipe in the dts cache
# (~/.duckietown/recipes/lx-dd-middleware-ros2-recipe/ente). When that cache
# entry is missing, the run fails with "No valid DTProject found". This script
# points a private recipes folder at the project's recipe/ for this one
# command, through DUCKIETOWN_RECIPES. The global dts cache is not touched.
set -euo pipefail

robot="${1:-birdie}"
launcher="${2:-overwatch}"
project="$(cd "$(dirname "$0")/.." && pwd)"

repository="$(sed -n 's/^RECIPE_REPOSITORY=//p' "${project}/.dtproject")"
branch="$(sed -n 's/^RECIPE_BRANCH=//p' "${project}/.dtproject")"
recipes="${HOME}/.cache/drone-vision-sim-recipes"
mkdir -p "${recipes}/${repository#*/}"
ln -sfn "${project}/recipe" "${recipes}/${repository#*/}/${branch}"
export DUCKIETOWN_RECIPES="${recipes}" DTSHELL_RECIPES="${recipes}"

cd "${project}"
# Build (or refresh) the project image on the robot first, so the workbench
# always runs the current code. Unchanged layers come from the Docker cache.
dts devel build --recipe "${project}/recipe" --force --username ubuntu -H "${robot}"
exec dts code workbench --recipe "${project}/recipe" -R "${robot}" -L "${launcher}"
