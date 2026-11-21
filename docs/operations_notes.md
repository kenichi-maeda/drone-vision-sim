# Operations notes

This page is the longer-form companion to the root README. Commands assume the
Orbstack/Duckietown host and the `ente` profile. Do not add `--embedded` and do
not start a second Matrix engine/window.

## Repeat a run from a clean spawn

The controller is one-shot. After it lands, stop the workbench (`Ctrl-C` in its
terminal) and relaunch it before another run. To reset the simulated drone and
map to their configured spawn, restart the Matrix map/engine; merely reattaching
Birdie is not a reliable reset of the scene.

1. Confirm Birdie is landed and disarmed (`landed_state: 1`, `armed: false`).
2. Stop the current Matrix session in its terminal with `Ctrl-C`. If you need a
   fresh spawn and `dts matrix run` reports that `dts-matrix-engine` is already
   running, inspect the exact engine before starting anything else:

   ```bash
   docker ps --filter name=dts-matrix-engine --format 'table {{.Names}}\t{{.Status}}'
   ```

   Only when Birdie is confirmed landed/disarmed and you intend to reset the
   scene, stop that one Matrix engine, then relaunch Matrix. Do not use an
   engine-name/port-offset workaround; that creates a second instance.

   ```bash
   docker stop dts-matrix-engine
   ```
3. Run one fresh Matrix session:

   ```bash
   dts matrix run --standalone --no-pull --map "$(pwd)/maps/tag_sandbox"
   ```

4. Attach the virtual vehicle to the map spawn, then start/restart the
   workbench:

   ```bash
   dts matrix attach birdie123test map_0/vehicle_0
   dts code workbench --recipe "$(pwd)/recipe" -R birdie123test -u "$USER"
   ```

If MAVROS has lost the simulator connection, do not arm. Re-attach the vehicle
and restart the MAVLink proxy and PX4:

```bash
dts matrix attach birdie123test map_0/vehicle_0
docker exec dts-virtual-birdie123test docker restart mavlink-proxy
docker exec dts-virtual-birdie123test docker restart dt-px4
```

Then repeat the README readiness checks. Look for PX4 `Simulator connected` and
`Ready for takeoff!`, MAVROS `connected: true`, a changing local pose, and fresh
camera frames. Start the native renderer with
`dts matrix run --engine 192.168.139.2 --renderer-id 0` if it did not reconnect.
Browser rendering is not required.

## What the mission parameters mean

The launcher starts the controller with `enabled=false`. The README enables
these options before setting `enabled=true`:

| Parameter | Current default | Effect |
| --- | ---: | --- |
| `hard_timeout_seconds` | `170` | Startup budget, including the slow takeoff spool. The watchdog adds search, centering, approach, and reacquisition. |
| `handoff_to_offboard` | `true` | Permit the AUTO.TAKEOFF to OFFBOARD handoff after the camera confirms the climb. |
| `object_search_enabled` | `true` | After takeoff, hold position and yaw toward the requested object. |
| `object_search_seconds` | `40` | Blind sweep only when that object was never seen. A sighting yaws toward its image error instead. |
| `object_centering_enabled` | `true` | Turn to reduce the target's horizontal image error. Yaw steps stay within 0.20 rad. |
| `object_centering_seconds` | `25` | Maximum time for image-based yaw centering before landing. |
| `object_approach_enabled` | `true` | Move forward while maintaining horizontal centering. |
| `object_approach_speed_m_s` | `0.08` | Maximum forward speed; controller slows as apparent size grows. |
| `object_approach_max_error_x` | `0.45` | Forward speed tapers between the center tolerance (`0.20`) and this image error; above it the drone holds position while turning. |
| `object_approach_size_growth_ratio` | `2.5` | Desired box-area growth. Landing also requires absolute area of at least 0.02, so growth of the tiny ground sliver does not land. |
| `object_approach_arrival_error_y` | `0.75` | Signed arrival cue. The target has to sit in the lower part of the forward image. |
| `object_approach_reacquire_seconds` | `8.0` | One bounded yaw scan after a target dropout. Translation is held. If not reacquired, land. |
| `object_approach_timeout_seconds` | `30` | Maximum approach time, not a desired flight duration. |
| `pose_stale_timeout_seconds` | `1.5` | After the estimator has risen 0.15 m, a pose older than this lands. Leave this value as it is. |

The flight node is `safe_hover.py`. YOLO reports the duckie as COCO class 32 and the stop sign as class 11 on
`/drone_vision/yolo_detection`. It is 2D image perception. The approach uses
relative box area as an approximate standoff cue, not a range sensor. The
forward camera looks ahead, so a ground object starts near the top of the
portrait frame and moves down as the drone closes in. That downward image
motion is the arrival cue.

Perception reads only `/<robot>/camera_node/image/compressed` (`birdie123test` in the verified run). Neither the
detector nor the controller reads Matrix entity poses, map coordinates, or
simulator target labels. The controller uses `/mavros/local_position/pose`,
MAVROS flight state, and the camera's YOLO detection topic; those are the
standard estimated vehicle state and image-derived target observation needed
to fly. The front ToF is not used by this mission.

The 2026-11-20 stop-sign run from the camera page climbed with the sign still
in frame, centered, commanded 1.53 m of approach with the box at 2.94x, then
hit the 30 s approach limit and landed disarmed. The duckie run from the same
page centered, approached, and landed with the duckie under the camera. The
controller makes one in-place yaw reacquisition attempt after a dropout. It
resumes only after stable detections, otherwise it lands. It never continues
forward without vision.

After every PX4 restart, set `MPC_TKO_SPEED` to 0.8 and `MIS_TAKEOFF_ALT` to
0.6 again. If `ekf2 status` shows no instance, run `ekf2 stop` and `ekf2 start`
on `/tmp/px4-sock-0` inside `dt-px4`, then start the pose stream with
`mavlink stream -r 30 -s LOCAL_POSITION_NED -u 14580` and
`mavlink stream -r 20 -s ATTITUDE -u 14580`. Confirm local position is updating
before the page starts a flight. `MPC_THR_HOVER` 0.70, `EKF2_OF_CTRL` 0,
`EKF2_MAG_CHECK` 0, `EKF2_REQ_HDRIFT` 0.6, `EKF2_REQ_VDRIFT` 0.6,
`SENS_EN_MAGSIM` 1, and `SENS_EN_BAROSIM` 1 should stay saved.

## Logs, images, and storage

Useful filtered logs (avoid scrolling through the live, high-rate stream):

```bash
docker exec dts-virtual-birdie123test docker logs --since 5m dts-run-drone-vision-sim 2>&1 \
  | grep -E 'safe_hover|yolo_detector|camera_probe.*frame=|ERROR' | tail -100
```

Camera probe images and the detector CSV are written inside the workbench
container under `/tmp`; they are bounded and are not copied to the project or
host unless you explicitly use `docker cp`. The probe keeps one diagnostic
image, up to 20 rolling camera samples, and at most 40 armed-flight JPEGs (not
continuous video). See `camera_probe.py` for the current filenames and limits.

## Stop the simulation

Let the mission finish its automatic landing and verify disarmed/landed before
stopping services. Then stop the workbench and virtual robot; stop the Matrix
session in its own terminal:

```bash
dts duckiebot virtual stop birdie123test
```

Use `Ctrl-C` in the Matrix terminal to close its renderer/session. Keep the
workbench terminal open while the mission is in flight so its controller and
landing safeguards remain active.
