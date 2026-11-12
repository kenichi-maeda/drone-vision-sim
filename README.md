# Drone vision simulation

This project is a camera-guided visual-target approach prototype in Duckiematrix.
The pipeline is camera → stock YOLOv8n COCO detector (`assets/yolov8n.onnx`,
not fine-tuned) → PX4/MAVROS position setpoints for search, centering, and
approach. The target COCO class is set in `packages/drone_vision/target.env`
(restart the workbench to apply it). The default is class 32 (`sports ball`),
which is how YOLO labels the yellow duckie prop and the only target verified
end to end. Flight is disabled by default. Image size is only an approximate
stopping cue; this is not metric ranging or obstacle avoidance.

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

1. **Takeoff:** sets `AUTO.TAKEOFF` and arms. PX4 climbs to the handoff height
   (about 0.4 m).
2. **Handoff:** streams setpoints at the current pose, then requests
   `OFFBOARD`.
3. **Search:** holds x, y and z and only yaws, through a little over one full
   turn, until YOLO confirms the target in several consecutive frames.
4. **Centering:** small yaw corrections proportional to the target's horizontal
   image error, rate-limited, with a short time limit.
5. **Approach:** moves the target position forward along the current heading.
   It slows as the target box grows and as the target drifts off-center, and it
   keeps correcting yaw while it goes.
6. **Landing:** sets `AUTO.LAND` and waits for ground contact and disarm.

Stop rules: it lands when the target box area reaches its goal growth (2.5x by
default) and the target is centered, after the approach time limit, or on a PX4
estimator or pose fault. If the target is lost mid-approach, it freezes
position, scans briefly once, then resumes or lands.

Exact rates, gains, and limits are in the parameter table in
[`docs/operations_notes.md`](docs/operations_notes.md#what-the-mission-parameters-mean).

## Start it yourself

Use one terminal for each long-running command. If Birdie is already running,
skip step 1. Run only one Matrix engine/window.

1. Start the virtual Duckiebot:

   ```bash
   dts duckiebot virtual start birdie
   ```

2. In another terminal, start the map and native Matrix renderer (no `--embedded`):

   ```bash
   dts matrix run --standalone --map /home/ubuntu/drone-vision-sim/maps/tag_sandbox
   ```

   Keep the native Matrix renderer open while the engine restarts. If it does
   not reconnect, relaunch the native renderer against `192.168.139.2` with
   `dts matrix run --engine 192.168.139.2 --renderer-id 0`.

   After the map is visible, attach Birdie to the vehicle spawn if it is not
   already attached:

   ```bash
   dts matrix attach birdie map_0/vehicle_0
   ```

3. In a third terminal, start the project workbench:

   ```bash
   cd /home/ubuntu/drone-vision-sim
   dts code workbench --recipe /home/ubuntu/drone-vision-sim/recipe -R birdie
   ```

   Leave this running. If the workbench is already running, do not start a
   second copy. The launcher starts the camera probe and the YOLO target
   detector. `safe_hover` starts disabled. The model is
   `assets/yolov8n.onnx`; the target class comes from
   `packages/drone_vision/target.env` (default `32`, COCO `sports ball`, which
   is how YOLO labels the duckie).

## Before enabling a flight

In a separate terminal, check PX4/MAVROS and camera/detector startup. Do not arm
if MAVROS is disconnected, the vehicle is already armed or airborne, local pose
is stale/non-finite, or the camera/detector is not producing live data.

```bash
docker exec dts-virtual-birdie docker exec ros2-mavros \
  bash -lc 'source /environment.sh && timeout 8s ros2 topic echo /mavros/state --once'

docker exec dts-virtual-birdie docker exec ros2-mavros \
  bash -lc 'source /environment.sh && timeout 8s ros2 topic echo /mavros/extended_state --once && timeout 8s ros2 topic echo /mavros/local_position/pose --once --qos-reliability best_effort'

docker exec dts-virtual-birdie docker logs --since 2m dt-px4 2>&1 \
  | grep -E 'Simulator connected|Ready for takeoff|poll timeout|ERROR|WARN' | tail -20

docker exec dts-virtual-birdie docker logs --since 2m dts-run-drone-vision-sim 2>&1 \
  | grep -E 'camera_probe.*frame=|yolo_detector|safe_hover' | tail -25

docker exec dts-virtual-birdie docker exec dts-run-drone-vision-sim \
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

The launcher configures the short YOLO search, centering, and approach phases.
Flight remains disabled unless `HOVER_ENABLED=true` is explicitly set when
starting the workbench. Run only after the readiness checks above pass:

```bash
docker exec dts-virtual-birdie docker exec dts-run-drone-vision-sim \
  bash -lc 'source /environment.sh && ros2 param set /safe_hover enabled true'
```

Watch progress without following the fast, full terminal stream:

```bash
docker exec dts-virtual-birdie docker logs --since 5m dts-run-drone-vision-sim 2>&1 \
  | grep -E 'safe_hover|ERROR|AUTO.LAND' | tail -80
```

The expected phases are takeoff, OFFBOARD handoff, target search, centering,
approach, one brief bounded reacquisition, and automatic landing. Forward speed
eases down as the target drifts off-center. If the target disappears during
approach, the drone holds position and scans for at most 2 seconds, then lands
if it cannot reacquire. Verify after the run that MAVROS reports `armed: false` and
`landed_state: 1`. The node is one-shot; restart the workbench before another
run.

The current test map places the duckie target at `(0.5, 0.9, 0)` and uses a 2.5x
visual-size growth goal. Its original pose, `(-0.35, 0.85, 0)`, is saved in
`maps/tag_sandbox/frames_baseline_2026-11-10_duckie_-0.35_0.85.yaml`.

If Python/package/launcher/model source changed, land the vehicle and stop the
old workbench first. Rebuild, then start it again using step 3:

```bash
cd /home/ubuntu/drone-vision-sim
dts devel build --recipe /home/ubuntu/drone-vision-sim/recipe --force --username ubuntu -H birdie
```

For Matrix reset/recovery, logs, parameters, and known limitations, see
[`docs/operations_notes.md`](docs/operations_notes.md).

## Watch the camera live (optional)

Set `CAMERA_VIEWER=true` in `packages/drone_vision/target.env` and restart the
workbench. Then run `python3 scripts/view_camera.py` on the sim machine and open
the URL it prints (port 8090) in a browser. The page shows the camera with the
YOLO box, plus the flight phase, confidence, error, and box area. It is
read-only.
