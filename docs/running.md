# Running the simulation

The commands are the same on Windows and macOS. They run in a Unix shell:
Terminal on a Mac, and Ubuntu 24.04 under WSL on Windows. Use branch
`camera-views`. The virtual robot name is `birdie123test`. Use a virtual name
that is not a physical Duckiebot. Do not arm, fly, or power a real drone.

Run one Matrix engine. Do not start a flight while the engine bridge shows
`rx_hits=0`.

## Windows

1. Install Ubuntu 24.04 as a WSL2 distro.
2. In that distro, install Docker and the Duckietown shell (`dts`), then log in.
3. Clone this repository in the Ubuntu filesystem and check out `camera-views`.
4. Run the start commands from that directory. `dts matrix run` opens the
   Matrix renderer as a window on the Windows desktop.
5. From Ubuntu, forward the camera page:

   ```bash
   python3 scripts/view_camera.py --robot birdie123test --port 8090
   ```

6. Open `http://127.0.0.1:8090/` in a browser on Windows.

## macOS

1. Install Docker Desktop and the Duckietown shell (`dts`), then log in.
2. Clone this repository and check out `camera-views`.
3. Run the start commands in Terminal. The Matrix renderer opens as a native window.
4. Forward the camera page with the same `scripts/view_camera.py` command and
   open `http://127.0.0.1:8090/`.

## Start

From the repository directory, one terminal per long-running command:

```bash
dts duckiebot virtual start --no-pull birdie123test
bash scripts/build_matrix_engine.sh
dts matrix run --standalone --no-pull --map "$(pwd)/maps/tag_sandbox"
dts matrix attach birdie123test map_0/vehicle_0
dts devel build --recipe "$(pwd)/recipe" --force --username "$USER" -H birdie123test
dts code workbench --recipe "$(pwd)/recipe" -R birdie123test -u "$USER"
python3 scripts/view_camera.py --robot birdie123test --port 8090
```

Skip the engine build when that image is already local. Skip the workbench
build when this checkout is already the image inside the virtual robot.
Restart the workbench after a code change. After every PX4 restart, set
`MPC_TKO_SPEED` to 0.8 and `MIS_TAKEOFF_ALT` to 0.6 again. Height stays on
the barometer (`EKF2_HGT_REF` 0). Leave GPS altitude and vertical velocity
off, and do not set `EKF2_GPS_CTRL` back to 5 or 7.

If `ekf2 status` shows no available instance, run `ekf2 start` once while the
vehicle is disarmed, then start the pose stream:

```bash
mavlink stream -u 14580 -s LOCAL_POSITION_NED -r 30
mavlink stream -u 14580 -s ATTITUDE -r 20
```

Confirm attitude and local position are healthy before a flight. A string
search for `healthy` also matches `unhealthy`.

## The two views

Every recording needs both of these for the whole flight.

**Fisheye, from the drone.** `http://127.0.0.1:8090/`, Fisheye selected. This
is the drone's front camera. Type one of the names below and press Fly. The
page lists what the camera currently recognizes. The object has to sit on the
crosshair when the status says `Arrived at {label}`, and the status then has
to reach `Landed beside {label}`.

**Overhead, so the drone's path is visible.** The map places a fixed camera
at `map_0/overhead_0`, west of the spawn, 3.6 m up, pitched down across the
ring. In the Matrix window, open the view menu at the top right and choose
that camera. The drone should travel through the shot from takeoff to
landing, with the ring of objects still in frame. Mayor's View is the free
spectator. Keys there move the spectator. The drive page
`http://127.0.0.1:8090/drive` is what flies the drone by hand: click it
first, then W and S move, A and D yaw, R climbs, F descends, and X resets to
the spawn.

Restart the Matrix session after a map change so the overhead camera is loaded.

## Objects

Type one of these on the fisheye page:

`duckie`, `red bot`, `orange bot`, `green bot`, `cyan bot`, `blue bot`,
`purple bot`, `pink bot`, `white bot`, `brown bot`, `lime bot`, `stop sign`.

The grey duckiebot farther ahead is not a flight target.

## What a finished video shows

One file per object. For that flight the overhead view shows the drone take
off, turn, fly to the object, and land beside it. The fisheye view shows the
typed name and the object on the crosshair at arrival, then the landed
status. Keep a take only when both views are in the file, the file plays, and
the page reached `Landed beside` that name.
