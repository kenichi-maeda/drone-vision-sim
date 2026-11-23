# Drone vision simulation

This project is a camera-guided visual-target approach prototype in Duckiematrix.
The pipeline is camera → stock YOLOv8n COCO detector (`assets/yolov8n.onnx`,
not fine-tuned) → PX4/MAVROS position setpoints for search, centering, and
approach. The weights are in this repository. `assets/yolov8n.pt` is the same
stock checkpoint, and `assets/yolov8n.onnx` is what the workbench loads. The
camera page chooses the object. The names are `duckie` (COCO 32,
`sports ball`, yellow), `stop sign` (COCO 11, red), and the ten colored bots
`red bot`, `orange bot`, `green bot`, `cyan bot`, `blue bot`, `purple bot`,
`pink bot`, `white bot`, `brown bot`, and `lime bot` (COCO 7, `truck`, plus
that color). Flight stays disabled until the page requests one. Image size is
only an approximate stopping cue; this is not metric ranging or obstacle
avoidance.

Detection is strictly camera-only YOLO. No Matrix object
pose, map coordinates, or simulator target labels feed detection or navigation.
The flight node does use the normal MAVROS local-position estimate, attitude,
and armed/landed state to hold position and publish setpoints; it does not use
the target's simulator pose. Do not enable a flight unless MAVROS is connected,
PX4 sensors are healthy, and camera-only YOLO is processing fresh frames. The
target may be absent at spawn; the bounded yaw search is responsible for
acquiring it.

## How it flies

The drone is controlled with position setpoints, not velocity. `safe_hover.py`
moves a target pose and PX4 flies to it.

What it sends to PX4 (through MAVROS):

- `/mavros/setpoint_position/local`: a `PoseStamped` (x, y, z and a yaw
  quaternion) published at 20 Hz. Every motion is a change to this one pose.
- `/mavros/set_mode`: switches between `AUTO.TAKEOFF`, `OFFBOARD` and
  `AUTO.LAND`.
- `/mavros/cmd/arming`: arms the drone.

Its inputs are the vehicle's own estimate from `/mavros/local_position/pose`,
the flight state, and the YOLO detection. It never reads simulator map poses.

The phases:

1. **Takeoff:** sets `AUTO.TAKEOFF` and arms, then holds a fixed thrust until
   the climb timer elapses or the camera shows the watched object move down
   the frame. Estimator height alone is not treated as a climb. It then
   streams setpoints and requests `OFFBOARD`. `MPC_TKO_SPEED` 0.8 and
   `MIS_TAKEOFF_ALT` 0.6 are RAM parameters, so set them again after every
   PX4 restart.
2. **Look around:** yaws in place and catalogs each colored object it sees.
3. **Recall:** turns back to the bearing of the name that was typed and waits
   there until that object is visible.
4. **Center:** yaws and nudges height until the object sits on the camera
   crosshair, both image errors within 0.22.
5. **Land beside it:** once the object is on the crosshair, the page says
   `Arrived at {label}` and the drone lands next to it, then disarms. The
   status is `Landed beside {label}`. It does not land on the object.

It also lands on a PX4 estimator or pose fault, or when the approach time
runs out. Height stays on the barometer (`EKF2_HGT_REF` 0). Leave GPS
altitude and vertical velocity off. Do not set `EKF2_GPS_CTRL` back to 5 or 7.

Exact rates, gains, and limits are in the parameter table in
[`docs/operations_notes.md`](docs/operations_notes.md#what-the-mission-parameters-mean).

## Start it yourself

Clone the repository and use this branch. Use one terminal for each
long-running command, and run only one Matrix engine. The robot name below is
`birdie123test`. Use a virtual name that is not a physical Duckiebot.

1. Start the virtual robot:

   ```bash
   dts duckiebot virtual start --no-pull birdie123test
   ```

2. Build the Matrix engine once so the drive page's keyboard hold is patched
   in, then start the map. Keep that renderer open.

   ```bash
   bash scripts/build_matrix_engine.sh
   dts matrix run --standalone --no-pull --map "$(pwd)/maps/tag_sandbox"
   ```

   After the map is visible, attach the virtual robot to the vehicle spawn:

   ```bash
   dts matrix attach birdie123test map_0/vehicle_0
   ```

   Do not fly while the engine bridge shows `rx_hits=0`.

3. Build and start the workbench from this checkout:

   ```bash
   dts devel build --recipe "$(pwd)/recipe" --force --username "$USER" -H birdie123test
   dts code workbench --recipe "$(pwd)/recipe" -R birdie123test -u "$USER"
   ```

   Leave this running. Do not start a second copy. The launcher starts the
   camera probe, the YOLO detector, and `safe_hover` with flight disabled.
   The model path is `assets/yolov8n.onnx`. Allowed classes come from
   `packages/drone_vision/target.env`.

4. Forward the camera page to this machine:

   ```bash
   python3 scripts/view_camera.py --robot birdie123test --port 8090
   ```

   Open `http://127.0.0.1:8090/`. **Fly to an object** lists the duckie, the
   ten colored bots, and the stop sign. Type one of those names and the
   simulated drone flies to it. The page beeps when the status begins with
   `Arrived`.
   `http://127.0.0.1:8090/drive` drives it from the keyboard: W and S move,
   A and D yaw, R climbs, F descends, and X resets to the spawn pose. Click
   the page first. Keys pressed in the Matrix window move the spectator.

Windows and Mac setup, the two cameras, and what a finished recording has to
show are in [`docs/running.md`](docs/running.md).

## Before enabling a flight

In a separate terminal, check PX4/MAVROS and camera/detector startup. Do not arm
if MAVROS is disconnected, the vehicle is already armed or airborne, local pose
is stale/non-finite, or the camera/detector is not producing live data.

```bash
docker exec dts-virtual-birdie123test docker exec ros2-mavros \
  bash -lc 'source /environment.sh && timeout 8s ros2 topic echo /mavros/state --once'

docker exec dts-virtual-birdie123test docker exec ros2-mavros \
  bash -lc 'source /environment.sh && timeout 8s ros2 topic echo /mavros/extended_state --once && timeout 8s ros2 topic echo /mavros/local_position/pose --once --qos-reliability best_effort'

docker exec dts-virtual-birdie123test docker logs --since 2m dt-px4 2>&1 \
  | grep -E 'Simulator connected|Ready for takeoff|poll timeout|ERROR|WARN' | tail -20

docker exec dts-virtual-birdie123test docker logs --since 2m dts-run-drone-vision-sim 2>&1 \
  | grep -E 'camera_probe.*frame=|yolo_detector|safe_hover' | tail -25

docker exec dts-virtual-birdie123test docker exec dts-run-drone-vision-sim \
  bash -lc 'source /environment.sh && timeout 8s ros2 topic echo /drone_vision/yolo_frame --once'
```

Expected state before a run: `connected: true`, `armed: false`, and
`landed_state: 1`, with a fresh local pose. PX4 should report `Simulator
connected` / `Ready for takeoff!` without stale sensor or attitude errors.
The workbench should show fresh camera frames and a recent
`/drone_vision/yolo_frame` stamp. A detection of the target class is required to
approach, but can be absent before the yaw search. If readiness fails, stop and use the
recovery notes below.

## Run the YOLO search-and-approach test

The launcher configures search, centering, and approach. Flight stays off
until the camera page sends a request. After the readiness checks above pass,
open `http://127.0.0.1:8090/` and type one of the object names.

Watch progress without following the fast, full terminal stream:

```bash
docker exec dts-virtual-birdie123test docker logs --since 5m dts-run-drone-vision-sim 2>&1 \
  | grep -E 'safe_hover|ERROR|AUTO.LAND' | tail -80
```

The expected phases are takeoff, a look around, a turn back to the typed
name, centering on the crosshair, arrival, and landing beside the object.
Verify after the run that MAVROS reports `armed: false` and `landed_state: 1`,
and that the page says `Landed beside` that name. The node is one-shot;
restart the workbench before another run.

The map places the duckie at `(0.22, 0.48)` and the stop sign at
`(-0.48, 0.62)`. The ten colored bots stand on a ring about 1.15 m from the
spawn. A grey duckiebot at `(0.0, 2.40)` is not a flight target. The flight
node does not read these coordinates.

If Python, the launcher, or the model changed, land the vehicle and stop the
old workbench first. Rebuild, then start it again using step 3:

```bash
dts devel build --recipe "$(pwd)/recipe" --force --username "$USER" -H birdie123test
```

For Matrix reset/recovery, logs, parameters, and known limitations, see
[`docs/operations_notes.md`](docs/operations_notes.md).

An earlier stop-sign recording is in
[`docs/stop-sign-flight.mp4`](docs/stop-sign-flight.mp4). New recordings need
both views described in [`docs/running.md`](docs/running.md): the overhead
camera, with the drone moving across the field, and the fisheye page where
the object name is typed.

## Watch the camera live

`CAMERA_VIEWER=true` is already set in `packages/drone_vision/target.env`.
Step 4 above forwards port 8090. The page shows the fisheye frame and a
straight frame (equidistant undistort, balance 0.7), the YOLO box, the flight
phase, and the object picker. A matching detection needs the COCO class and
the color: yellow for the duckie, red for the stop sign, and the named color
for each bot.
