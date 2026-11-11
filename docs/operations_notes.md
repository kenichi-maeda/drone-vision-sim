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
   dts matrix run --standalone --map /home/ubuntu/drone-vision-sim/maps/tag_sandbox
   ```

4. Attach the virtual vehicle to the map spawn, then start/restart the
   workbench:

   ```bash
   dts matrix attach birdie map_0/vehicle_0
   cd /home/ubuntu/drone-vision-sim
   dts code workbench --recipe /home/ubuntu/drone-vision-sim/recipe -R birdie
   ```

If MAVROS has lost the simulator connection, do not arm. Re-attach the vehicle
and restart the MAVLink proxy and PX4:

```bash
dts matrix attach birdie map_0/vehicle_0
docker exec dts-virtual-birdie docker restart mavlink-proxy
docker exec dts-virtual-birdie docker restart dt-px4
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
| `hard_timeout_seconds` | `70` | Startup budget for connection, takeoff, and OFFBOARD handoff; the mission watchdog adds the bounded search, centering, approach, and reacquisition phase budgets. |
| `handoff_to_offboard` | `true` | Permit the AUTO.TAKEOFF to OFFBOARD handoff. |
| `object_search_enabled` | `true` | After takeoff, hold position and yaw-scan for a stable target. |
| `object_search_seconds` | `22` | One yaw sweep of just over 360 degrees at the configured 0.31 rad/s, followed by automatic landing if YOLO does not acquire the sign. |
| `object_centering_enabled` | `true` | Turn to reduce the target's horizontal image error. |
| `object_centering_seconds` | `6` | Maximum time for image-based yaw centering before landing. |
| `object_approach_enabled` | `true` | Move forward while maintaining horizontal centering. |
| `object_approach_speed_m_s` | `0.08` | Maximum forward speed; controller slows as apparent size grows. |
| `object_approach_max_error_x` | `0.45` | Forward speed tapers between the center tolerance (`0.20`) and this image error; above it the drone holds position while turning. |
| `object_approach_size_growth_ratio` | `2.0` | Desired target box-area growth relative to acquisition; more growth means a closer visual-size stop, not a known metric distance. |
| `object_approach_reacquire_seconds` | `2.0` | One bounded yaw scan after a brief target dropout; translation is held. If not reacquired, land. |
| `object_approach_timeout_seconds` | `30` | Maximum approach time, not a desired flight duration. |

The flight node is `safe_hover.py`. YOLO reports a bounding box for COCO class 11 (stop sign) on
`/drone_vision/yolo_detection`. It is 2D image perception. The approach uses
relative box area as an approximate standoff cue, not a range sensor. Since the
camera points downward, vertical image error is not currently controlled.

Perception reads only `/birdie/camera_node/image/compressed`. Neither the
detector nor the controller reads Matrix entity poses, map coordinates, or
simulator target labels. The controller uses `/mavros/local_position/pose`,
MAVROS flight state, and the camera's YOLO detection topic; those are the
standard estimated vehicle state and image-derived target observation needed
to fly. The front ToF is not used by this mission.

The latest recorded reset-spawn run took off, searched, centered, and advanced
about 0.74 m. It reached about 2.32x apparent box area, then lost the sign and
landed via the target-loss safeguard before reaching the 3x cue. That is useful
end-to-end progress, but the final distance is not yet reliable. The controller
now makes one in-place yaw reacquisition attempt after the configured 2-second
loss grace period; it resumes only after stable detections, otherwise it lands.
It never continues forward without vision.

## Logs, images, and storage

Useful filtered logs (avoid scrolling through the live, high-rate stream):

```bash
docker exec dts-virtual-birdie docker logs --since 5m dts-run-drone-vision-sim 2>&1 \
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
dts duckiebot virtual stop birdie
```

Use `Ctrl-C` in the Matrix terminal to close its renderer/session. Keep the
workbench terminal open while the mission is in flight so its controller and
landing safeguards remain active.
