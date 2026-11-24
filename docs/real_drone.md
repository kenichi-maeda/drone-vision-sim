# Real drone (birdie)

This branch adds the physical Duckiedrone path on top of the `camera-views`
simulator work. The simulator robot is `birdie123test`. The aircraft in the
lab is `birdie`, a Duckietown DD24-B on the `ente` image. They do not share a
controller. `packages/drone_vision/drone_vision/safe_hover.py` arms, calls
`AUTO.TAKEOFF`, and flies position setpoints. This airframe has no GPS, no
barometer, and no position estimate PX4 can use for that, so that program
must not be started on birdie.

The program that has been flying birdie is
[`scripts/birdie_flight.py`](../scripts/birdie_flight.py). It sits beside
MAVROS inside the `ros2-mavros` container. It does not arm, it does not
publish a manual stick, and it does not request `AUTO.LAND`. The pilot arms
from Mission Control. The motors spin as soon as the vehicle is armed.

`scripts/real_camera.py` is an earlier laptop viewer for the same camera. It
only reads pictures. Leave it stopped while a flight page is open. The page
that actually publishes the duck lock is a separate program,
`~/classes/cs1951r/yolo-dashboard/server.py`, on port 8091. It is not in this
repository. The wire format it uses is documented below so the flight script
can be read on its own.

## Aircraft

- Host name `birdie`. The address moves with DHCP. `birdie.local` and the
  Brown DNS name can point at a stale address. Confirm with SSH and
  `hostname` before copying files. One address that answered during this work
  was `128.148.140.76`.
- Flight controller: Diatone Mamba F405 MK2, USB `/dev/ttyACM0`. The product
  string alternates between `PX4 OmnibusF4SD` and `DiatoneMambaF405 MK2`
  across a reboot. Do not open the serial port while MAVROS holds it.
- PX4 is the Duckietown tag `dd24-mamba-f405-mk2-v1.15.4-1`, commit
  `d8c6892a0d16792305bc66b0bd965c4e2b60232f`, board
  `boards/diatone/mamba-f405-mk2`. `CONFIG_BOARD_CONSTRAINED_FLASH` is set, so
  there is no logger module and MAVLink FTP of `/fs/microsd/log` fails. There
  is no flight-controller ulog.
- The board has no barometer and no magnetometer (`SYS_HAS_MAG` is 0 in the
  board defaults). The companion time-of-flight sensor is not the PX4 local
  altitude. There is no GPS. PX4 therefore has no local position and no local
  altitude.
- The board cannot measure current. Used milliamp-hours and remaining flight
  time stay empty. Voltage sag under throttle is what the battery failsafe
  sees. A 4S pack is empty at about 14.0 V (`BAT1_N_CELLS` 4, `BAT1_V_EMPTY`
  3.5, `BAT1_V_DIV` 11.0, `BAT1_CAPACITY` 1500). `CBRK_SUPPLY_CHK` stays
  `894281` because the board also cannot measure the 5 V supply. With that
  breaker at 0 the vehicle refuses to arm.
- Companion stack: ROS 2 Jazzy and MAVROS in
  `duckietown/dt-ros2-interface:ente-arm64v8`. Official bring-up is `dts` as a
  normal user. Do not run `sudo dts`.

Mission Control is `http://<robot-ip>/dashboard/robot/mission_control/`. On
that page, Space disarms, including in the air. The on-screen throttle is not
what this script flies. The dashboard KILL button is MAVLink command 400 with
param2 `21196`. That force-disarms and can latch a manual lockdown. It is not
command 185. Command 185 (`DO_FLIGHTTERMINATION`) is the commander path that
enters flight termination directly. Do not send it.

## What the flight script does

After the pilot arms, the script streams an attitude and a thrust on
`/mavros/setpoint_raw/attitude` (`AttitudeTarget`, best effort, type mask 7:
ignore body rates, use the quaternion and the thrust). It requests `OFFBOARD`
only after 1.2 seconds. The flight controller's spool-up
(`COM_SPOOLUP_TIME`) is 1.0 second, and a mode change during that second was
present on an early lock. The script keeps requesting `STABILIZED` while it
is disarmed and not in flight termination.

The quaternion is roll, pitch, and yaw. Roll and pitch commands stay at 0
except for a nose-down pitch of 0.03 rad during the approach. Level comes
from the board trim, not from a bias in this quaternion. Yaw is the IMU yaw
plus a short lead for the commanded turn rate. An earlier version integrated
the yaw command on its own. The drone stopped following it during a bounce,
and the setpoint kept turning, which is the lost heading that ended in the
cage corner.

Thrust is a fraction of full throttle, already scaled by MAVROS. The plugin
starts with `thrust_scaling` as NaN, and a NaN scale makes MAVROS drop every
attitude message, so the mode stays `STABILIZED` and the props stay at idle.
The value in `px4_config.yaml` does not reach the running node. The script
sets `thrust_scaling` to 1 at startup.

The height target is 0.50 m. Three feet is 0.91 m. Thrust starts at 46
percent and is not allowed above 50 percent. Above about 0.65 m, or while
climbing faster than about 0.4 m/s, it cuts toward 40 percent. Once it is
descending from that cut it holds about 44 percent instead of falling to the
floor. A height a little above 0.50 m eases down. A height under about 0.40 m
may climb back, slowly, and only up to 48 percent. With no height reading the
takeoff ramp stops at 48 percent. It does not cut to a descent after two
blind seconds, and it does not climb to 56 percent to chase a missing
reading.

The 8 second still hover counts only while the filtered height stays between
about 0.35 m and 0.65 m and the vertical speed is small. Outside that band
the script holds the heading and does not yaw. The search is one slow circle
(0.18 rad/s) measured from the IMU, so the circle ends when the aircraft has
actually turned, not when the command integral says it has. If the duck is
centered for 0.4 seconds, the scan stops and that IMU heading is kept. The
script then turns back to the best heading it saw, faces the duck, leans
forward, and lands when the duck is too close, when the mission reaches 120
seconds, or when a climb never leaves the ground. If the aircraft has already
been airborne and then sits under about 0.22 m for 5 seconds with the thrust
already high, it lands instead of pushing into the cage.

## Duck message

The laptop page publishes `std_msgs/Float32MultiArray` on
`/drone_vision/yolo_detection`. A sample older than 0.8 seconds is ignored.
The fields match the simulator detector:

| Index | Meaning |
| --- | --- |
| 0 | class id |
| 1 | confidence |
| 2 | center x, 0 to 1 |
| 3 | center y, 0 to 1 |
| 4 | horizontal error, positive to the right, about -1 to 1 |
| 5 | vertical error |
| 6 | box width, fraction of the image |
| 7 | box height, fraction of the image |
| 8 | range code |

Range codes: 1 in range, 2 too far, 3 too close, 4 touching the edge. The
script treats 1 and 2 as a heading it may approach, 3 as the landing cue, and
4 as "not a lock." Centered means the horizontal error is inside 0.18.

The live page is `http://127.0.0.1:8091/`. Frames come through an SSH tunnel
to the Pi rosbridge, `ws://127.0.0.1:9001`, topic
`/birdie/camera_node/image/compressed`. The bridge is asked for about 8 frames
a second. A frame older than about 400 ms is discarded without closing the
subscription. Closing that subscription has stopped `driver-camera`. Shown
frames are about 150 to 200 ms old. Do not run the extra viewers on ports
8092 and 8093 at the same time. Another rosbridge subscriber drops the flight
page to a few frames a second.

The detector is YOLOE (`yoloe-11s-seg.pt`) prompted for `rubber duck` and
`doll`. Stock COCO YOLO called the doll a teddy bear and missed the duck or
called it a traffic light. A yellow box that also contains an orange beak is
labeled rubber duck. A yellow tape measure can take the same label. Do not
arm toward that.

Mission Control's camera block polls `http://127.0.0.1:8091/frame.jpg`, so
the dashboard shows the boxes when that page is running.

## Height sensor

The script subscribes to `/birdie/altitude_node/altitude`
(`sensor_msgs/Range`). The bottom sensor is a VL53L1X on `/dev/i2c-23` at
`0x29`. A good chip id is `ea cc 10`. On the floor the raw range is about
0.018 m, and the altitude node publishes 0 because its minimum range is 0.04
m. The script ignores readings under 0.05 m, so a valid height appears only
after liftoff. One sample that jumps by more than 0.40 m is ignored. The same
reading three times is accepted, so a real climb is not frozen out. The
filter then follows the accepted range. The old filter rejected any jump over
0.25 m and never updated, so the log showed the cage ceiling while the thrust
loop still thought the drone was low and kept climbing.

The driver container can look healthy while its log says `I2C error. The
sensor is not responding.` A direct id read can succeed in that state and the
ROS topic can still be silent. Restart `driver-tof-bottom`, then
`ros2-tof-bottom` and `altitude`, and confirm the topic is publishing before
a flight that is supposed to hold 0.50 m. Do not leave the driver stopped.

The cage ceiling is about 2.0 to 2.1 m.

## Saved flight-controller parameters

These were read back after a reboot, so they are on the flight controller and
not only in RAM. Leave them there.

| Parameter | Saved value | Why |
| --- | --- | --- |
| `COM_LOW_BAT_ACT` | 0 (Warning) | Land falls through to flight termination on this board |
| `COM_FLTT_LOW_ACT` | 1 (Warning) | Return has the same fallthrough |
| `COM_RC_OVERRIDE` | 0 | Mission Control publishes throttle 0 many times a second. Override 1 chopped the climb and kicked the mode back to `STABILIZED` |
| `NAV_RCL_ACT` | 0 (Disabled) | A missing stick must not command a return |
| `COM_RC_IN_MODE` | 3 | Do not set this to 4 |
| `COM_OBL_RC_ACT` | 0 (Position) | Offboard loss enters a path that can stop at `STABILIZED` |
| `COM_OF_LOSS_T` | 1.0 | |
| `COM_SPOOLUP_TIME` | 1.0 | |
| `CBRK_FLIGHTTERM` | 121212 | Turns the failure-detector critical action into a warning. It does not stop the Land/Return fallthrough |
| `CBRK_SUPPLY_CHK` | 894281 | Required to arm |
| `FD_FAIL_P`, `FD_FAIL_R` | 0 | Disables the attitude failure check |
| `SENS_BOARD_Y_OFF` | 1.66 | Pitch trim |
| `SENS_BOARD_X_OFF` | -0.25 | Roll trim |

`COM_RCL_EXCEPT` is 4, which ignores RC loss in `OFFBOARD`.

## Why a low battery reading terminated the flight

PX4 custom mode `655360` is `0x000A0000`. The main mode lives in bits 16–23,
and 10 is `PX4_CUSTOM_MAIN_MODE_TERMINATION`. Heartbeat `system_status` 8 is
`MAV_STATE_FLIGHT_TERMINATION`. The motors are off and the vehicle is
disarmed. The script will not ask for `STABILIZED` in that state, because a
failsafe terminate does not clear on a mode change. Command 246 with param1 1
reboots the flight controller. USB drops, then `/dev/ttyACM0` returns. Set
`STABILIZED` after the port is back and wait until the heartbeat is disarmed,
mode `STABILIZED`, status 3.

The failsafe action selector falls through PosCtrl, AltCtrl, Stabilized,
Hold, Return, Land, Descend, and then Terminate when the selected action
cannot run. Land needs local position and local altitude. Descend needs
attitude, rates, and local altitude. This board has neither local position
nor local altitude, so Land and Descend cannot run and the selector ends at
Terminate. The comment in that firmware's `framework.cpp` says that if
Stabilized is not available the result is Terminate. Battery Land and
remaining-time Return enter the switch below Stabilized, so they skip it.

`COM_LOW_BAT_ACT` was 2 (Land) and `COM_FLTT_LOW_ACT` was 3 (Return). Both
are now the warning values in the table above. The hop that died about a
second after takeoff climbed through about 0.07 m and 0.17 m at 47 to 52
percent, then locked. The pack was near 15 V at rest. Under throttle the
voltage sags, the current sensor does not exist, and the sag is read as an
empty pack. After the motors stop, the voltage recovers, so a later reading
near 15 V does not disprove the sag. The next hop disarmed normally instead
of locking. That is one flight, not a long proof. Do not put Land or Return
back.

Offboard-loss action Position enters higher in the same switch and can stop
at Stabilized, so loss of the script alone should not be what terminates.
Command 176 (`DO_SET_MODE`) shows up in the MAVROS log as an unexpected
command with result 0. An unknown custom mode in that handler does not enter
termination. No command 185 was seen on the flights that locked.

`CBRK_FLIGHTTERM` 121212 does not cover this path. During the first second
after arm, a critical failure detector disarms instead of terminating.
`FD_FAIL_P` and `FD_FAIL_R` at 0 disable the attitude check.

## Why the hover bounced into the cage

Thrust on this airframe is very sensitive to the pack.

- On a tired pack, 50 percent held about 0.15 m.
- On a full pack, 50 percent climbed from under 0.1 m to about 1 m in a
  second and reached the cage ceiling, about 2.1 m.
- The hop after the height sensor came back reached a logged 2.10 m while the
  thrust was still 50 percent, then swung between about 42 and 56 percent.
  Each time the reading looked low the script added thrust, and each time it
  looked high the script cut to the floor of the slew. That is the up-down
  loop. An earlier loop was the same bug at a smaller step, 48 percent versus
  45 percent around 0.25 m.
- A hover that the pilot liked held with no height reading, a little above
  the target. Two seconds later the script stepped from 50 percent to 34
  percent and stayed there, so it could not climb back. That blind cut is
  gone.

The current limit is 50 percent, and the cut starts at 0.65 m rather than
waiting for 0.91 m. Do not restore a thrust step that depends on crossing a
height line, and do not put the climb back up to 56 percent.

## Trim

A slow slide in `STABILIZED` with the sticks centered is the level reference.
`SENS_BOARD_Y_OFF` is pitch and `SENS_BOARD_X_OFF` is roll. Leave the rotor
mixer and the rate PIDs alone. Read the live values before writing, add a
small delta, and stay within a few degrees of the level calibration (Y about
0.95, X about -0.66).

Checked on this aircraft against the PX4 attitude, disarmed and sitting
still:

- Raising Y makes the PX4 pitch more negative. The controller holds the nose
  up, which counters a slide forward. Raising Y too far makes it slide
  backward.
- Raising X makes the PX4 roll more negative. The controller holds the right
  wing down, which counters a slide to the drone's left. Raising X too far
  makes it slide right.

The MAVROS IMU pitch has the opposite sign from that PX4 pitch. Use the
MAVLink attitude when checking a trim change. Set the parameters with
`ParamSetV2` (type 3, double, force set) and save with command 245 param1 1.
Do the check disarmed in `STABILIZED`. Do not write trim while the status is
8 or the vehicle is armed.

History of the saved values: Y 1.7 slid backward. Y 1.4 and Y 1.55 slid
forward. Y 1.62 was the half step. The pilot then reported a slight slide
forward and a slight slide left, so Y went from 1.62 to 1.66 and X from -0.30
to -0.25. Those are the saved values. The bounce flights were too short and
too high to justify another trim step. Do not put that correction in the
attitude quaternion.

## Running it

Copy the script into the companion and start one process. A second copy also
publishes setpoints. The shell line matches `birdie_flight`, so kill the
python process and its shell parent, then confirm one pair is left.

```bash
docker cp birdie_flight.py ros2-mavros:/tmp/birdie_flight.py
docker exec ros2-mavros bash -lc 'ps -eo pid,cmd | awk "/birdie_flight/ && !/awk/ {print \$1}" | xargs -r kill -9'
docker exec -d ros2-mavros bash -lc 'source /opt/ros/jazzy/setup.bash && python3 -u /tmp/birdie_flight.py >/tmp/birdie-flight.log 2>&1'
```

`/tmp` inside the container survives a container restart. The log is
`/tmp/birdie-flight.log`. Ready to arm means the log is the disarmed waiting
line, the heartbeat is `STABILIZED` with status 3, and the downward height
topic is publishing. Status 0 after a reboot usually means Mission Control is
not publishing the joystick. Open the mission-control page and wait for
status 3. Status 8 means it is locked. Do not arm. Reboot the flight
controller with command 246, then set `STABILIZED` and confirm status 3.

A flight needs a TA, goggles, and the props clear. Space disarms. The script
will climb when the pilot arms.

The last state written down after an emergency stop was disarmed, with the
mode string still `OFFBOARD` and system status 0, and the flight script
stopped. That stop was command 400 param2 `21196`, and the controller
accepted it (`success` true, result 0). Set `STABILIZED` and see status 3
before another arm.

## Files on this branch

- `scripts/birdie_flight.py` is the physical flight script.
- `scripts/real_camera.py` is the earlier read-only camera page on port 8093.
  Its bridge address is a DHCP address from one session. Leave the process
  stopped during a flight.
- `packages/drone_vision/drone_vision/yolo_infer.py` is the ROS-free YOLOv8n
  pass used by that camera page. The flight page uses YOLOE instead, because
  YOLOv8n does not name the rubber duck.
