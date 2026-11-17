# OVERWATCH operator console

> **Status (2026-11-17):** the whole sim session now runs from the workspace
> devcontainer with one command, `bash scripts/overwatch.sh up`. The Mac
> only needs a browser: no Mac host runner, Mac `dts` or Mac Python. The
> console connects to `overwatch_link` on `birdie` and runs YOLO-World on the
> devcontainer's CPU. The sim drone is `birdie`. The recon findings below were
> verified on 2026-11-15. This page will be updated as each capability is
> verified in the sim.

OVERWATCH is a browser-based operator console for the Duckiedrone. It shows
the drone's camera with live YOLO detections drawn on it. It lets the operator
designate one detected object as the target by mouse, keyboard or typed
command. It can also hand flight control to the operator through a manual
override. Autonomous pursuit of the target is out of scope for now: only its
interface, `get_target_state()`, will be provided.

The current phase is **simulation only** (Duckiematrix). The physical drone
will plug in later through the same two adapters.

## Contents

- [What has been done](#what-has-been-done)
- [Architecture](#architecture)
- [Recon findings](#recon-findings)
- [Using the console](#using-the-console)
- [Physical drone integration](#physical-drone-integration)
- [Open items and risks](#open-items-and-risks)

## What has been done

| Date | Work | Result |
| --- | --- | --- |
| 2026-11-15 | Read the Duckietown docs: Duckiedrone, Duckiematrix, ROS, `dts` (Ente), plus the Daffy DD24 manuals | Ente docs cover sim flight (PX4 + MAVROS) but not the drone camera. Physical-drone docs exist only for Daffy (ROS 1). |
| 2026-11-15 | Read `h2r/pidrone_pkg` source (the Daffy drone software) | Physical camera, control topics and safety heartbeat identified ([below](#physical-drone-daffy-docs-and-source-not-verified-on-hardware)). |
| 2026-11-15 | Live checks on a virtual DD24 in the workspace devcontainer | Camera topic, QoS, rate and MAVROS interfaces confirmed. The stock rosbridge was found to be broken. |
| 2026-11-15 | Four Matrix scene experiments using a scratch map (`/home/ubuntu/overwatch-recon-map` in the workspace, not in this repo) | Found which objects the drone camera can see ([below](#placing-objects-in-the-matrix)). |
| 2026-11-15 | Ran stock and open-vocabulary YOLO on captured sim frames, on the Mac's GPU | Chose the detector ([below](#yolo-on-sim-frames)). |
| 2026-11-16 | Design decisions agreed | Listed in [Decisions](#decisions). |
| 2026-11-16 | First sim session with the Mac native renderer and the Mac console | The link connected, and takeoff, OFFBOARD hold and LAND worked through `overwatch_link`. |
| 2026-11-17 | Moved the whole session into the devcontainer (`scripts/overwatch.sh`) | The Mac host runner broke when the Mac workspace folder was moved (`No such file or directory: '/Users/.../intro_robotics'`). The Matrix now uses the browser (WebGL) renderer, which is served from the devcontainer, and the console runs from a devcontainer venv with CPU PyTorch. All 92 console tests pass there. |

Work is on the branch `overwatch`.

### Decisions

1. **Perception runs in the devcontainer** (CPU) since 2026-11-17, next to
   the sim. It originally ran on the Mac's Apple GPU. `perception.device:
   auto` picks `mps`, `cuda` or `cpu`, whichever the machine has, so the same
   config still works on the Mac. The perception host is a config value
   (`perception.host: ws://HOST:8710`), so detection can move to a faster
   machine, a ground station or a companion computer later.
2. **Detector:** YOLO-World with per-box HSV color classification and no
   training to start. A recall eval of about 100 sim frames comes first.
   YOLO11n gets fine-tuned only if any class falls below about 80% recall at
   expected flight distances.
3. **Natural language:** a deterministic parser is the main path, and it asks
   a clarifying question when a command is ambiguous. An optional Claude
   fallback is off by default and runs only when the parser fails. It
   produces the same query schema, goes through the same validation, and can
   never reach flight control.
4. **Emergency stop:** KILL force-disarms by default (this is configurable)
   and is bound to one hard-to-hit key. A separate LAND NOW is the
   non-destructive abort. Every KILL and LAND NOW event is logged.
5. **Physical drone stack:** not yet decided (Daffy ROS 1 or Ente ROS 2 +
   PX4). Either way, LIVE sits behind the same feed and flight adapters.
6. **Repo:** work on branch `overwatch` in the workspace copy
   (`/home/ubuntu/drone-vision-sim`), which is now the copy that runs
   everything. `scripts/sync_to_workspace.py` is only needed if you still
   edit in a separate Mac checkout. Nothing is committed unless the lead asks.
7. **Sim drone:** `birdie`, a virtual DD24 in the workspace devcontainer.
8. **Everything runs in the devcontainer** (2026-11-17): Matrix engine,
   renderer (browser mode), workbench, forwarder and console. The Mac's role
   is a browser with two tabs.

## Architecture

```
          ┌──────── Ground station: workspace devcontainer (Python 3.12, FastAPI) ────────┐
 camera ──▶ FeedSource ──▶ Perception ──▶ TargetManager ──▶ UI server ⇄ browser console
 (SIM|LIVE)               YOLO + track      lock / LOST      (each frame + its detections
                          + color           get_target_state()   sent together, one frame ID)
                          CommandParser (natural language) ─┘        ▲ click / Tab / text / keys
 MAVROS ◀── FlightAdapter ◀── FlightControl (modes, dead-man) ◀──────┘
 (SIM|LIVE)
```

| Module | Responsibility | SIM | LIVE |
| --- | --- | --- | --- |
| Feed source | Delivers camera frames | `overwatch_link` node on `birdie`, over TCP | Ente drone: same link. Daffy drone: MJPEG adapter (stub). |
| Perception | YOLO-World detection, ByteTrack track IDs, HSV color | Devcontainer (CPU) | Ground station, or a remote host set in config |
| Target manager | Holds the lock, reports TARGET LOST, provides `get_target_state()` | Devcontainer | Ground station |
| Command parser | Turns typed commands into a validated target query | Devcontainer | Ground station |
| Flight control | Modes, key mapping, speed limits, dead-man timers | Devcontainer | Ground station |
| Flight adapter | Sends commands to the vehicle | `overwatch_link` → MAVROS | Ente drone: same link. Daffy drone: rosbridge adapter (stub). |
| UI | Browser page: video canvas, HUD, command box | Served from the devcontainer, viewed in the Mac browser | Same |

Switching SIM → LIVE changes only the feed adapter and the flight adapter.

Where each sim process runs (all in the workspace devcontainer, started by
`scripts/overwatch.sh up`):

| Process | Port (on the Mac as `localhost:<port>`) | Log |
| --- | --- | --- |
| Matrix engine (`dts-matrix-engine` container, host network) | 7501–7505 (engine and WebGL bridge) | `docker logs dts-matrix-engine` |
| Matrix renderer page (`dts matrix run --browser`) | 8800 | `.run/matrix.log` |
| Workbench, `overwatch_link` on `birdie` | 8765 inside `birdie` | `.run/workbench.log` |
| Port forwarder (`scripts/forward_port.py`) | 8765 | `.run/forward.log` |
| OVERWATCH console (`overwatch/.venv`) | 8700 | `.run/console.log` |

The drone camera image is drawn by the Matrix renderer, which in browser mode
runs **in the Mac browser tab**. With no Matrix tab open, the sim runs but
the camera publishes nothing, and the console shows FEED LOST.

**What the operator sees is what YOLO saw.** The browser receives each frame
together with its detections and target state, and shows only frames that
have been through inference. If inference falls behind the camera, older
frames are dropped, never queued, so delay cannot build up.

**Why the drone needs its own link node.** The virtual drone ships a rosbridge
websocket, but it is broken (see below). `overwatch_link` is a small ROS 2
node that runs in the project workbench on the drone. It sends camera JPEGs
up and takes flight commands down over one TCP connection. A port forwarder
in the devcontainer makes it reachable from the console at `127.0.0.1:8765`.

## Recon findings

### Simulated drone (verified live on a virtual DD24)

| Item | Finding |
| --- | --- |
| Camera topic | `/<robot>/camera_node/image/compressed`, so `/birdie/camera_node/image/compressed` for `birdie` |
| Message | `sensor_msgs/msg/CompressedImage`, JPEG, `frame_id: camera_color_optical_frame` |
| QoS | RELIABLE, KEEP_LAST (1), VOLATILE (matches the existing `yolo_detector.py`) |
| Image | 480 × 640 portrait, fisheye (equidistant model in `cameras.yaml`), pitched down toward the ground |
| Rate | 10–20 Hz, falling as the Matrix gets busier |
| Middleware | ROS 2 Jazzy, `rmw_zenoh_cpp`, `ROS_DOMAIN_ID=42` |
| Flight topics | `/mavros/setpoint_velocity/cmd_vel`, `/mavros/setpoint_position/local`, `/mavros/state`, `/mavros/extended_state`, `/mavros/local_position/pose` |
| Flight services | `/mavros/set_mode`, `/mavros/cmd/arming`, `/mavros/cmd/takeoff`, `/mavros/cmd/land`, `/mavros/cmd/command` |
| rosbridge | Container `ros2-rosbridge-websocket` is configured for port 9001, but loops on "Address already in use". Not usable. |
| Network | The Mac reaches ports the devcontainer listens on via `localhost`. It cannot reach the virtual drone's own IP, so a forwarder is needed (same approach as `scripts/view_camera.py`). |

Two problems were seen on the **stock** Matrix engine:

- PX4's position estimate had not converged; it read z = −0.71 m while the
  drone sat on the ground.
- The drone's starting heading differed between restarts.

The project's engine patches (`scripts/build_matrix_engine.sh`) are expected
to fix the first. That still has to be checked.

### Placing objects in the Matrix

| What was tried | Visible in the drone camera? |
| --- | --- |
| `markers.yaml` primitives (cube, sphere, cylinder, cone): a ring of nine 20 cm cubes | **No.** They load, but the drone camera never shows them. |
| Extra `vehicles.yaml` entries with `color: "#rrggbb"` (DB21J) | **Yes**, with colored bodies |
| Duckie citizen (`citizens.yaml`) | Yes |
| `decorations.yaml` | No usable prefabs in renderer v0.14.0 |

Other notes:

- The engine reads each layer from `<layer>.yaml` by filename.
- The ground is green grass, so the test arena sits on gray `asphalt` tiles;
  otherwise green targets would be hard to separate.

### YOLO on sim frames

Run on the Mac's Apple GPU:

| Model | Duckiebot | Duckie about 1 m away | Time per frame |
| --- | --- | --- | --- |
| YOLOv8n, COCO | "airplane" 0.23 | missed | 14–24 ms |
| YOLO11n, COCO | "truck" 0.31 | missed | 15–20 ms |
| YOLO11s, COCO | "truck" 0.63 | missed | 18–21 ms |
| YOLO-World v2-s, prompted "toy robot car" | **"toy robot car" 0.65**, no false positives | missed | 24 ms |

Stock COCO class names are wrong for these objects, which rules them out for
commands like "lock onto the red robot". The small duckie is missed by every
model, so a short fine-tune is likely.

Note that `assets/yolov8n.onnx` is gitignored and missing from both copies
of the repo, so the existing `yolo_detector` cannot run as checked out.

### Physical drone (Daffy docs and source, not verified on hardware)

| Item | Finding |
| --- | --- |
| Software | ROS 1 `pidrone_pkg`, Betaflight flight controller |
| Camera | `raspicam_node` publishing `/raspicam_node/image/compressed`, 320 × 240 at 30 fps |
| Camera over HTTP | `web_video_server` on port 8080: `/stream?topic=/raspicam_node/image&type=ros_compressed` |
| Commands | Through rosbridge (default port 9090, not verified): `/pidrone/desired/mode` (`pidrone_pkg/Mode`: `ARMED`, `DISARMED`, `FLYING`), `/pidrone/desired/twist` (`geometry_msgs/Twist`) |
| Safety | The flight controller disarms if `/pidrone/heartbeat/web_interface` (`std_msgs/Empty`) stops for 3 s |
| Stock web UI keys | `;` arm, Space disarm, `t` takeoff, I/J/K/L translate, W/S up/down, A/D yaw |

The Ente manual says the drone software is being ported to Ente. An Ente
drone would likely match the sim interfaces (ROS 2 + PX4 + MAVROS).

## Using the console

### Start a sim session

Everything runs in a terminal of the workspace devcontainer (VS Code
attached to the container). The Mac only needs a browser. Don't use the
Mac's own `dts` for virtual robots: they don't work on this Mac.

**One time only** (in the devcontainer, from `/home/ubuntu/drone-vision-sim`):

- If `dts duckiebot virtual list` does not show `birdie`, create it:
  `dts duckiebot virtual create -t duckiedrone -c DD24 birdie`
- Build the patched Matrix engine and make it the image `dts` runs:
  `bash scripts/build_patched_engine_image.sh --as-default`
- Create the console's venv. PyTorch comes from the CPU wheel index, which
  avoids gigabytes of unused CUDA packages:

  ```bash
  cd overwatch
  python3 -m venv .venv
  .venv/bin/pip install --index-url https://download.pytorch.org/whl/cpu \
      --extra-index-url https://pypi.org/simple torch==2.15.0 torchvision==0.29.2
  .venv/bin/pip install -r requirements.txt -r requirements-llm.txt
  .venv/bin/python -m pytest -q        # 92 tests should pass
  ```

  The first console start downloads YOLO-World's CLIP text encoder (338 MB)
  into the Ultralytics cache.

**Every session:**

```bash
cd /home/ubuntu/drone-vision-sim
bash scripts/overwatch.sh up
```

`up` starts, in order, whatever is not already running: `birdie`, the Matrix
engine with `maps/overwatch_arena` and the browser renderer on port 8800,
`dts matrix attach birdie map_0/vehicle_0`, the workbench with the overwatch
launcher (it builds the project image first), the link forwarder on port
8765, and the console on port 8700. At the end it prints two URLs. Open both
in the Mac browser:

1. **Matrix**: `http://localhost:8800/?...&token=.../` (the exact URL
   includes your Duckietown token; `bash scripts/overwatch.sh urls` prints it
   again). This tab **is the renderer**: it draws the 3D arena and the drone
   camera. Keep it open, and keep it visible in its own window, because
   browsers pause hidden tabs, and a paused tab means no camera frames.
2. **OVERWATCH console**: http://localhost:8700/

Other commands:

| Command | What it does |
| --- | --- |
| `bash scripts/overwatch.sh status` | Shows what is running, plus the URLs |
| `bash scripts/overwatch.sh urls` | Prints the two URLs |
| `bash scripts/overwatch.sh down` | Stops the console, forwarder, workbench and Matrix. `birdie` keeps running. |

Logs are in `.run/` in the project root (`matrix.log`, `workbench.log`,
`forward.log`, `console.log`). Settings can be overridden through
environment variables: `OVERWATCH_ROBOT`, `OVERWATCH_MAP`,
`OVERWATCH_MATRIX_PORT`, `OVERWATCH_CONSOLE_PORT`, `OVERWATCH_LINK_PORT`.

Manual steps, if you need to run a piece by hand (each in its own
devcontainer terminal):

1. `dts duckiebot virtual start birdie`
2. `dts matrix run --standalone --no-pull --browser --host 0.0.0.0 --port 8800 --no-tutorial --map /home/ubuntu/drone-vision-sim/maps/overwatch_arena`
   (`--no-pull` keeps `dts` from replacing the patched engine with a newer
   stock image)
3. `dts matrix attach birdie map_0/vehicle_0`
4. `bash scripts/workbench.sh birdie overwatch` (wraps `dts code workbench`,
   which on its own fails here with "No valid DTProject found"; the script
   explains why)
5. `python3 scripts/forward_port.py --robot birdie`
6. `cd overwatch && .venv/bin/python -m overwatch`

#### Native renderer (Mac window instead of a browser tab)

```bash
bash scripts/overwatch.sh up --native
```

Without `--browser`, `dts matrix run` in the devcontainer hands the renderer
to the **Mac host runner**: a small server on the Mac (port 61146) that the
devcontainer starts when VS Code opens it. It opens the native Duckiematrix
window on the Mac. Everything else (engine, workbench, console) still runs in
the devcontainer. The native renderer cannot run inside the devcontainer
itself: the Linux renderer is an x86-64 binary, and the devcontainer is ARM64
with no FEX emulator and no display.

The host runner remembers the Mac folder it was started from. If that
folder is moved or renamed, every native launch fails with:

```
Host runner request failed: [Errno 2] No such file or directory: '/Users/<you>/<old folder>'
```

That happened on 2026-11-17, when the workspace moved from
`~/Desktop/intro_robotics` to `~/Desktop/robotics/workspace`. To fix it,
restart the runner from the new folder, on the Mac:

1. In VS Code: `Cmd+Shift+P` → **Dev Containers: Reopen Folder Locally**.
2. In a Mac terminal, stop the old runner:
   `lsof -nP -iTCP:61146 -sTCP:LISTEN` gives its PID, then `kill <PID>`.
3. **File → Open Folder…** → `~/Desktop/robotics/workspace`, then
   `Cmd+Shift+P` → **Dev Containers: Reopen in Container** (or **Rebuild
   Container** if the old path still shows up).
4. Back in the devcontainer: `bash scripts/overwatch.sh down`, then
   `bash scripts/overwatch.sh up --native`.

Until the runner is fixed, `up --native` stops with the error above, and
plain `up` (browser renderer) still works.

To try the console without the simulator, run
`.venv/bin/python tools/fake_link.py --frames tools/sample_frames` (from
`overwatch/`) and then `.venv/bin/python -m overwatch`. It plays the frames
in a loop and pretends to fly. Stop it before a real session: both use port
8765.

### Screen layout

- **Video canvas:** the camera frame with each detection's box, class, color,
  confidence and track ID. Boxes can be clicked.
- **HUD:** FPS, inference latency, source (SIM, LIVE or REPLAY), control mode
  (AUTONOMOUS or MANUAL), link status, arm state, flight mode, the current
  manual command, and the lock status.
- **Target panel:** the locked target's ID, class, color and image position,
  shown as the live output of `get_target_state()`.
- **Command box:** typed targeting commands, and the console's replies,
  including clarifying questions.
- **Flight panel:** arm, takeoff, land, a LAND NOW button and a red KILL
  button, all always visible.

### Designate a target

| Method | How |
| --- | --- |
| Mouse | Click a box. |
| Keyboard | Tab or Shift+Tab moves a highlight through the detections. Enter locks the highlighted one. |
| Typed command | Press `/`, type a command, then press Enter. |
| Release | Backspace or the Release button |

Example commands:

- `lock onto the red robot`
- `target the leftmost blue object`
- `track the closest duckie`
- `the other one`: the previous candidates, minus the current or last-offered
  object
- `release`

When more than one object matches, the console asks instead of guessing. For
example: "Two red robots: #3 (left) and #8 (right). Which one?" Answer with
"the left one", "#3", "3" or "the blue one". The question expires after 15 s.

A few terms have specific meanings:

- Color is one of red, orange, yellow, green, blue or purple, or `unknown`
  when the box has no clear dominant color.
- "Closest" means the box whose bottom edge is lowest in the image, because
  the camera points down.

Once a target is locked, it gets a distinct reticle and keeps its track ID as
it moves. If it leaves the frame or stops being detected, the panel shows
**TARGET LOST**. If, within 3 s, exactly one object of the same class and
color appears, it is locked again automatically. If several match, the one
nearest the target's last position is highlighted and Enter locks it.

### Fly

AUTONOMOUS is the default mode. It sends no motion: if the drone is airborne,
it holds position. Pursuit is not implemented yet. MANUAL hands control to the
operator. Toggle between them with M or the HUD button.

| Action | Key | Notes |
| --- | --- | --- |
| Forward / back / left / right | W S A D or arrow keys | MANUAL only, relative to the drone's heading |
| Up / down | R / F | MANUAL only |
| Yaw left / right | Q / E | MANUAL only |
| Toggle MANUAL / AUTONOMOUS | M | |
| Arm / disarm | Y / H | Disarm only works on the ground |
| Takeoff / land | T / L | |
| **LAND NOW** | Space, or the HUD button | Works in any mode. Zeroes manual input and returns to AUTONOMOUS. Logged. |
| **KILL** (force-disarm) | **Shift+Esc**, or the red HUD button | Motors stop and the drone falls. Works even while typing. Returns to AUTONOMOUS. Logged. |

Releasing a movement key stops that motion. All keys and speed limits are set
in `overwatch/config/overwatch.yaml`. KILL and LAND NOW events go to a JSONL
log with the time, source, mode, arm state, altitude, target state and the
command's acknowledgement.

### Warnings and safety behavior

| Banner | Cause | What the console does |
| --- | --- | --- |
| FEED LOST | No new frame for 0.5 s | Holds velocity at zero and blocks movement keys |
| INFERENCE STALLED | Detection has not finished on a frame for 1 s | Same as FEED LOST |
| LINK LOST | No contact with `overwatch_link` | Same. The link also zeroes velocity on its side. |

Movement commands go out only while the console has a recent frame and the
browser keeps sending key state. Leaving the browser window, or typing in the
command box, sends zero velocity. Commands are never buffered or replayed. If
setpoints stop, PX4 leaves OFFBOARD mode and enters its own failsafe. LAND NOW
and KILL work in every state.

### `get_target_state()` (for future pursuit)

Planned return value:

```python
TargetState(
    status="TRACKING",          # NONE | TRACKING | LOST
    track_id=7,
    class_name="robot",
    color="red",
    confidence=0.71,
    bbox_xyxy_px=(120, 210, 260, 330),
    center_norm=(0.40, 0.42),   # 0..1 across the image
    error_norm=(-0.20, -0.16),  # -1..1 from image center
    area_norm=0.054,
    frame_id=18342,
    seconds_since_seen=0.0,
)
```

A pursuit controller will plug in behind this interface. It will run only in
AUTONOMOUS mode while a target is TRACKING and the feed is fresh.

## Physical drone integration

The console reaches a drone only through a feed adapter and a flight
adapter, which are selected in `overwatch.yaml`. An Ente/PX4 drone can reuse
`overwatch_link`, pointed at the real drone. A Daffy drone needs two adapters,
which will be stubbed with the mapping above:

- an MJPEG feed from `web_video_server`
- a rosbridge flight adapter that also sends the 3 s heartbeat

Which stack the physical drone runs is still to be decided (decision 5).

## Open items and risks

- **Physical drone stack** (decision 5) is not yet decided.
- **Patched Matrix engine:** all six project patches apply to the current
  engine (some with fuzz). `dts matrix run` refuses `--container-image` with
  the Mac renderer, so `scripts/build_patched_engine_image.sh --as-default`
  points the standard tag at the patched image and keeps the stock one as
  `ente-arm64v8-stock`. Always start the Matrix with `--no-pull`.
- **`birdie` must live in the workspace devcontainer.** A `dts-virtual-birdie`
  container on the Mac's own Docker holds the same ports (80, 8080, 9001,
  11411, 11911) and must be stopped first.
- **PX4 health in sim:** manual flight and the recall eval both depend on a
  converged position estimate.
- **Mac host runner (native renderer):** on 2026-11-17 it still pointed at
  the old `~/Desktop/intro_robotics` folder, so `up --native` fails until it
  is restarted from `~/Desktop/robotics/workspace` (see
  [Native renderer](#native-renderer-mac-window-instead-of-a-browser-tab)).
- **Browser renderer (since 2026-11-17):** the drone camera is rendered by
  the Matrix browser tab, so camera frames stop when that tab is closed,
  hidden or throttled. Its camera rate and fidelity have not yet been
  compared with the native Mac renderer's (10–20 Hz).
- **CPU perception:** YOLO-World on the devcontainer CPU is slower than on
  the Mac GPU (24 ms per frame). The console drops frames instead of queueing
  them, so expect a lower annotated frame rate, not growing delay. If it is
  too slow, run `python -m overwatch.perception.server` on a faster machine
  and set `perception.host` to it.
- **Small duckie recall** is likely below 80%, so a fine-tune is likely.
- **Assumed flight envelope** for the eval: 0.6–1.0 m altitude, 1–3 m from
  targets.
