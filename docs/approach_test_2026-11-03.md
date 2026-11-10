# Alternate sign bearing and closer visual stop test

Before this test, the sign pose was `(x=0.75, y=0.75, z=0)` and the approach
area-growth goal was `3.0x`. Those are the fallback settings. The original pose
is also saved in `maps/tag_sandbox/frames_baseline_2026-11-03.yaml`; that backup
is not included by `main.yaml`.

The preceding flight reached the 3.0x goal after about 0.35 m of commanded
approach, with the final detection at 3.35x. That confirmed the approach ran,
but it did not provide a metric drone-to-sign distance.

The current test moves the sign to a new bearing and farther from the spawn:
`(x=1.35, y=0.54, z=0)`, about 1.45 m from spawn (previously about 1.06 m).
Its yaw is `0.78` rad (about 45 degrees), rotated farther toward the original
spawn after the prior angle looked insufficient. Position remains unchanged.
The approach goal is `4.5x` apparent box-area growth. The larger goal is meant
to bring the drone closer despite the increased starting range, but it is still
an image-size cue, not a measured distance. The YOLO box-area safety ceiling
remains in place.

To restore the earlier setup, set the sign pose in `frames.yaml` back to
`x: 0.75`, `y: 0.75` (leave z and orientation unchanged) and set
`object_approach_size_growth_ratio` in `safe_hover.py` back to `3.0`. Rebuild the
recipe image and restart the workbench after changing source.

The new map pose requires restarting Matrix to take effect. This test has not
yet been flown; verify the sign is detectable from the spawn before enabling
flight. If it is too small to detect, restore the saved pose or move it closer.

## YOLO-only visibility adjustment (2026-11-09)

The red-octagon detector fallback was removed. The archived camera-frame check
showed that COCO YOLOv8n did not reliably detect the sign at the previous
distance. The sign is now at `(0.90, 0.36, 0)` (previously `(1.35, 0.54, 0)`),
with the same yaw, and the image-area growth goal is `2.0x` to keep the visual
standoff conservative. Restore the previous pose and `4.5x` goal if this test
does not improve camera-only detection. This adjustment still requires live
YOLO verification before flight.

## Steering correction after the timeout run

The next flight commanded about 0.69 m of approach and reached 3.33x area
before the 60-second deadline. Detector logs showed repeated horizontal error
reversals; detections mainly came from the red-octagon fallback. The detector's
0.5-second log interval is not its publication rate.

Code inspection found that each fresh detection added yaw correction to the
previous commanded yaw. The accumulated offset was bounded to +/-0.35 rad, but
could still keep commanding a turn while the vehicle lagged. Centering and
approach now share one correction method based on measured yaw. Forward
increments follow measured heading, and paused ticks no longer contribute
elapsed movement time. Reacquisition resets the yaw reference to the newly
acquired heading. The 60-second deadline remains unchanged.

The controller now tapers forward speed as horizontal image error grows from
`0.20` to `0.45`, then holds position while turning. Previously it stopped at
`0.20`, and logs repeatedly crossed that bound. The approach log now reports
commanded distance and alignment factor separately. The 60-second timeout and
the 4.5x image-size goal still require a new flight to assess.

Offline regression checks passed for repeated observations without heading
motion, measured-heading updates, deadband settling, and angle wrapping.
This steering change still requires a rebuilt runtime and a flight from spawn;
the logs above do not validate the new controller.
