# OVERWATCH operator console

> **Status (2026-11-16):** the console is built on branch `overwatch` and has
> been tested against a stand-in drone (`overwatch/tools/fake_link.py`), but
> **not yet in the simulator**. The sim drone is `birdie`. The recon findings
> below were verified on 2026-11-15. This page will be updated as each
> capability is verified in the sim.

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

Work is on the branch `overwatch`.

### Decisions

1. **Perception runs on the Mac** (Apple GPU), leaving the sim container to
   PX4 and the Matrix. The perception host is a config value, so it can move
   to a ground station or companion computer later.
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
6. **Repo:** work on branch `overwatch` in the Mac copy, and sync drone-side
   files into the workspace copy with `scripts/sync_to_workspace.py`
   (`--dry-run` supported). Nothing is committed unless the lead asks.
7. **Sim drone:** `birdie`, a virtual DD24 in the workspace devcontainer.
stil
## Architecture

```
                ┌─────────────── Mac ground station (Python 3.12, FastAPI) ───────────────┐
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
| Perception | YOLO-World detection, ByteTrack track IDs, HSV color | Mac | Mac, or a remote host set in config |
| Target manager | Holds the lock, reports TARGET LOST, provides `get_target_state()` | Mac | Mac |
| Command parser | Turns typed commands into a validated target query | Mac | Mac |
| Flight control | Modes, key mapping, speed limits, dead-man timers | Mac | Mac |
| Flight adapter | Sends commands to the vehicle | `overwatch_link` → MAVROS | Ente drone: same link. Daffy drone: rosbridge adapter (stub). |
| UI | Browser page: video canvas, HUD, command box | Mac | Mac |

Switching SIM → LIVE changes only the feed adapter and the flight adapter.

**What the operator sees is what YOLO saw.** The browser receives each frame
together with its detections and target state, and shows only frames that
have been through inference. If inference falls behind the camera, older
frames are dropped, never queued, so delay cannot build up.

**Why the drone needs its own link node.** The virtual drone ships a rosbridge
websocket, but it is broken (see below). `overwatch_link` is a small ROS 2
node that runs in the project workbench on the drone. It sends camera JPEGs
up and takes flight commands down over one TCP connection. A port forwarder
in the devcontainer makes it reachable from the Mac.

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

The sim drone is `birdie`, a virtual DD24 in the workspace devcontainer.
Run steps 1–5 in a devcontainer terminal, and step 6 on the Mac. Don't use the
Mac's own `dts` for virtual robots: they don't work on this Mac.

0. One time only:
   - If `dts duckiebot virtual list` does not show `birdie`, create it:
     `dts duckiebot virtual create -t duckiedrone -c DD24 birdie`
   - Build the patched Matrix engine and make it the image `dts` runs:
     `bash scripts/build_patched_engine_image.sh --as-default`
   - On the Mac, create the console's venv:
     `cd overwatch && /opt/homebrew/bin/python3.12 -m venv .venv && .venv/bin/pip install -r requirements.txt`
1. Start the virtual drone: `dts duckiebot virtual start birdie`
2. Start the Matrix with the arena map. `--no-pull` keeps `dts` from
   replacing the patched engine with a newer stock image:
   `dts matrix run --standalone --no-pull --map /home/ubuntu/drone-vision-sim/maps/overwatch_arena`
3. Attach the drone to the spawn: `dts matrix attach birdie map_0/vehicle_0`
4. Build the project image on `birdie` and start the drone-side link:
   `bash /home/ubuntu/drone-vision-sim/scripts/workbench.sh birdie overwatch`.
   This wraps `dts code workbench`, which on its own fails here with "No valid
   DTProject found" (the script explains why).
5. Forward the link port to the Mac: `python3 scripts/forward_port.py --robot birdie`
6. On the Mac: `cd overwatch && .venv/bin/python -m overwatch`, then open
   http://127.0.0.1:8700/.

To try the console without the simulator, run
`.venv/bin/python tools/fake_link.py --frames tools/sample_frames` (from
`overwatch/`) in place of steps 1–5. It plays the frames in a loop and
pretends to fly. Stop it before step 5 of a real session: both use port 8765.

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
- **Small duckie recall** is likely below 80%, so a fine-tune is likely.
- **Assumed flight envelope** for the eval: 0.6–1.0 m altitude, 1–3 m from
  targets.
