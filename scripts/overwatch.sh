#!/usr/bin/env bash
# Run the whole OVERWATCH sim session from the workspace devcontainer: no Mac
# host runner, no Mac Python. The Mac only needs a browser.
#
#   bash scripts/overwatch.sh up       # start everything that is not already running
#   bash scripts/overwatch.sh up --native   # same, but the Matrix opens as a native Mac window
#   bash scripts/overwatch.sh status   # what is running, plus the two URLs
#   bash scripts/overwatch.sh urls     # just the two URLs
#   bash scripts/overwatch.sh down     # stop the console, forwarder, workbench and Matrix (birdie keeps running)
#
# What `up` starts, in order (each step is skipped if it is already running):
#   1. the virtual drone `birdie`
#   2. the Matrix engine with maps/overwatch_arena, plus the renderer:
#      default: the WebGL renderer served on port 8800 (browser mode, nothing
#               is delegated to the Mac)
#      --native: the native renderer window on the Mac, launched through the
#               Mac host runner (needs a working host runner, see docs/interface.md)
#   3. `dts matrix attach birdie map_0/vehicle_0`
#   4. the project workbench with the overwatch launcher (scripts/workbench.sh)
#   5. the link port forwarder (scripts/forward_port.py, port 8765)
#   6. the OVERWATCH console (overwatch/.venv, port 8700)
#
# The drone camera is rendered by the Matrix renderer (browser tab or native
# window), so keep it open and visible while flying.
# Logs and PID files go to .run/ in the project root (gitignored).
set -euo pipefail

project="$(cd "$(dirname "$0")/.." && pwd)"
robot="${OVERWATCH_ROBOT:-birdie}"
map="${OVERWATCH_MAP:-${project}/maps/overwatch_arena}"
matrix_port="${OVERWATCH_MATRIX_PORT:-8800}"
console_port="${OVERWATCH_CONSOLE_PORT:-8700}"
link_port="${OVERWATCH_LINK_PORT:-8765}"
run="${project}/.run"
mkdir -p "${run}"

log() { printf '\033[1m[overwatch]\033[0m %s\n' "$*"; }
die() { printf '\033[31m[overwatch] %s\033[0m\n' "$*" >&2; exit 1; }

alive() { [ -f "${run}/$1.pid" ] && kill -0 "$(cat "${run}/$1.pid")" 2>/dev/null; }
listening() { (exec 3<>"/dev/tcp/127.0.0.1/$1") 2>/dev/null; }

wait_for() {  # wait_for <seconds> <description> <command...>
    local seconds="$1" what="$2"; shift 2
    for _ in $(seq 1 "${seconds}"); do "$@" && return 0; sleep 1; done
    die "Timed out after ${seconds}s waiting for ${what}."
}

# Start a long-running command detached, with a pseudo-terminal (dts and
# `docker run -it` refuse to run without one), logging to .run/<name>.log.
start_bg() {  # start_bg <name> <workdir> <command string>
    local name="$1" dir="$2" cmd="$3"
    : > "${run}/${name}.log"
    (cd "${dir}" && setsid script -qfec "${cmd}" "${run}/${name}.log" </dev/null >/dev/null 2>&1 &
     echo $! > "${run}/${name}.pid")
}

robot_up() { docker ps --format '{{.Names}}' | grep -qx "dts-virtual-${robot}"; }
engine_up() { docker ps --format '{{.Names}}' | grep -qx dts-matrix-engine; }
engine_healthy() { [ "$(docker inspect -f '{{.State.Health.Status}}' dts-matrix-engine 2>/dev/null)" = healthy ]; }
link_up() {
    docker exec "dts-virtual-${robot}" docker ps --format '{{.Names}}' 2>/dev/null \
        | grep -qx dts-run-drone-vision-sim
}
link_ready() {
    docker exec "dts-virtual-${robot}" docker logs dts-run-drone-vision-sim 2>&1 | grep -q 'OVERWATCH link on'
}

matrix_url() {
    local url
    url="$(sed 's/\x1b\[[0-9;]*m//g' "${run}/matrix.log" 2>/dev/null \
        | tr -d '\r' | grep -o 'Navigate to http[^ ]*' | tail -1 | sed 's/^Navigate to //; s/\.$//')"
    [ -n "${url}" ] || return 1
    echo "${url}" | sed -E 's#^http://[^/]+:#http://localhost:#'
}

native_launched() { grep -q 'Launching the native renderer on the host' "${run}/matrix.log" 2>/dev/null; }
native_failed() {
    grep -qE 'Host runner request failed|Host-side .dts matrix run. exited|ERROR' "${run}/matrix.log" 2>/dev/null
}

print_urls() {
    echo
    if [ -f "${run}/renderer" ] && [ "$(cat "${run}/renderer")" = native ]; then
        log "Matrix: native renderer window on the Mac"
    elif url="$(matrix_url)"; then
        log "Matrix (keep this tab open):  ${url}"
    else
        log "Matrix: no URL yet (see ${run}/matrix.log)"
    fi
    log "OVERWATCH console:            http://localhost:${console_port}/"
    echo
}

cmd_up() {
    local renderer=browser
    [ "${1:-}" = "--native" ] && renderer=native
    command -v dts >/dev/null || die "dts not found. Run this inside the workspace devcontainer."
    [ -x "${project}/overwatch/.venv/bin/python" ] || die \
        "Console venv missing. Create it once (see docs/interface.md, 'One time only')."

    # 1. virtual drone
    if robot_up; then log "1/6 ${robot} already running"
    else log "1/6 starting ${robot}"; dts duckiebot virtual start "${robot}"; fi

    # 2. Matrix engine + browser renderer
    if engine_up && alive matrix; then log "2/6 Matrix already running"
    else
        engine_up && die "A Matrix engine is running that this script did not start. Stop it first (docker stop dts-matrix-engine)."
        echo "${renderer}" > "${run}/renderer"
        if [ "${renderer}" = native ]; then
            log "2/6 starting the Matrix (${map##*/}, native renderer window on the Mac)"
            start_bg matrix "${project}" "dts matrix run --standalone --no-pull --no-tutorial --map '${map}'"
            wait_for 180 "the native renderer launch (see ${run}/matrix.log)" \
                bash -c "grep -qE 'Launching the native renderer|Host runner request failed|ERROR' '${run}/matrix.log'"
            sleep 3
            if native_failed; then
                sed 's/\x1b\[[0-9;]*m//g' "${run}/matrix.log" | grep -E 'failed|ERROR' >&2 || true
                die "The Mac host runner could not open the native renderer. Fix the runner (docs/interface.md, 'Native renderer') or run without --native."
            fi
        else
            log "2/6 starting the Matrix (${map##*/}, browser renderer on :${matrix_port})"
            start_bg matrix "${project}" "dts matrix run --standalone --no-pull --browser --host 0.0.0.0 \
--port ${matrix_port} --no-tutorial --map '${map}'"
            wait_for 180 "the Matrix renderer page" matrix_url
        fi
        wait_for 60 "the Matrix engine to become healthy" engine_healthy
    fi

    # 3. attach the drone to the spawn (harmless if it is already attached)
    log "3/6 attaching ${robot} to map_0/vehicle_0"
    dts matrix attach "${robot}" map_0/vehicle_0 >"${run}/attach.log" 2>&1 \
        || die "attach failed, see ${run}/attach.log"

    # 4. drone-side link (workbench, overwatch launcher)
    if link_up; then log "4/6 workbench already running on ${robot}"
    else
        log "4/6 building and starting the workbench (first build takes a few minutes)"
        start_bg workbench "${project}" "bash scripts/workbench.sh ${robot} overwatch"
        wait_for 900 "overwatch_link to start (see ${run}/workbench.log)" link_ready
    fi

    # 5. link port forwarder
    if listening "${link_port}"; then log "5/6 port ${link_port} already forwarded"
    else
        log "5/6 forwarding :${link_port} to ${robot}"
        start_bg forward "${project}" "python3 scripts/forward_port.py --robot ${robot} --port ${link_port}"
        wait_for 20 "the port forwarder" listening "${link_port}"
    fi

    # 6. OVERWATCH console
    if listening "${console_port}"; then log "6/6 console already running"
    else
        log "6/6 starting the OVERWATCH console (YOLO-World loads in ~30 s)"
        start_bg console "${project}/overwatch" ".venv/bin/python -m overwatch --port ${console_port}"
        wait_for 600 "the console (see ${run}/console.log)" listening "${console_port}"
    fi

    print_urls
}

cmd_status() {
    robot_up && log "${robot}: running" || log "${robot}: stopped"
    if engine_up; then log "Matrix engine: running ($(docker inspect -f '{{.State.Health.Status}}' dts-matrix-engine))"
    else log "Matrix engine: stopped"; fi
    link_up && log "workbench / overwatch_link: running" || log "workbench / overwatch_link: stopped"
    listening "${link_port}" && log "forwarder :${link_port}: listening" || log "forwarder :${link_port}: stopped"
    listening "${console_port}" && log "console :${console_port}: listening" || log "console :${console_port}: stopped"
    print_urls
}

stop_pid() {  # stop_pid <name>
    if alive "$1"; then
        local pid; pid="$(cat "${run}/$1.pid")"
        kill -- "-${pid}" 2>/dev/null || kill "${pid}" 2>/dev/null || true
    fi
    rm -f "${run}/$1.pid"
}

cmd_down() {
    log "stopping the console";   stop_pid console
    pkill -f "overwatch/.venv/bin/python -m overwatch" 2>/dev/null || true
    log "stopping the forwarder"; stop_pid forward
    pkill -f "scripts/forward_port.py --robot ${robot}" 2>/dev/null || true
    log "stopping the workbench"; stop_pid workbench
    docker exec "dts-virtual-${robot}" docker stop dts-run-drone-vision-sim >/dev/null 2>&1 || true
    log "stopping the Matrix";    stop_pid matrix
    docker stop dts-matrix-engine >/dev/null 2>&1 || true
    log "done (${robot} is still running; stop it with: dts duckiebot virtual stop ${robot})"
}

case "${1:-}" in
    up) cmd_up "${2:-}" ;;
    status) cmd_status ;;
    urls) print_urls ;;
    down) cmd_down ;;
    *) sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'; exit 1 ;;
esac
