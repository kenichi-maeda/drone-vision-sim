"""Two browser views of the simulated drone.

/ is the front camera, in fisheye or with the fisheye removed.
/drive moves the drone through the Matrix keyboard input.
Neither view arms PX4 by itself.
"""

from __future__ import annotations

import json
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from time import monotonic, sleep, time
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray, String

from drone_vision.drive_link import COMMAND_FIELDS, DriveLink, empty_command
from drone_vision.vision_lib import rectify


ANNOTATED_TOPIC = "/drone_vision/yolo_image/compressed"
DETECTION_TOPIC = "/drone_vision/yolo_detection"
OBJECTS_TOPIC = "/drone_vision/yolo_objects"
PHASE_TOPIC = "/drone_vision/calibration_phase"
STALE_AFTER_S = 1.0
BOUNDARY = "frame"
DRIVE_URLS = [
    os.environ.get("MATRIX_DATA_IN", ""),
    "ws://172.17.0.1:7505",
    "ws://host.docker.internal:7505",
    "ws://127.0.0.1:7505",
]

STYLE = """
  body { margin: 0; background: #111; color: #ddd; font: 14px/1.4 system-ui, sans-serif; }
  nav { display: flex; gap: 8px; align-items: center; padding: 12px 16px; background: #1b1b1b; }
  a, button { color: #eee; background: #333; border: 0; border-radius: 6px; padding: 8px 12px; text-decoration: none; font: inherit; }
  button.active, a.active { background: #2f6fed; }
  main { display: flex; flex-wrap: wrap; gap: 16px; padding: 16px; align-items: flex-start; }
  img { max-width: min(100%, 480px); background: #000; }
  table { border-collapse: collapse; }
  th, td { text-align: left; padding: 4px 10px 4px 0; }
  th { color: #888; font-weight: 500; }
  .ok { color: #8d8; }
  .bad { color: #f88; }
  .pad { display: grid; grid-template-columns: repeat(3, 88px); gap: 8px; }
  p { max-width: 42rem; color: #aaa; }
"""

PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Drone camera</title>
<style>""" + STYLE + """</style>
</head>
<body>
<nav>
  <a class="active" href="/">Camera</a>
  <a href="/drive">Drive</a>
  <button id="fisheye" class="active" type="button">Fisheye</button>
  <button id="straight" type="button">Straight</button>
</nav>
<main>
  <img id="cam" alt="front camera">
  <div>
    <table>
      <tr><th>Phase</th><td id="phase">-</td></tr>
      <tr><th>Lens</th><td id="lens">fisheye</td></tr>
      <tr><th>Specified</th><td id="specified">-</td></tr>
    </table>
    <table id="objects">
      <tr><th>Object</th><th>Class</th><th>Color</th><th>Match</th></tr>
    </table>
  </div>
</main>
<script>
  let lens = "fisheye";
  const text = (id, value) => { document.getElementById(id).textContent = value; };
  function setLens(next) {
    lens = next;
    text("lens", next);
    document.getElementById("fisheye").classList.toggle("active", next === "fisheye");
    document.getElementById("straight").classList.toggle("active", next === "straight");
  }
  document.getElementById("fisheye").onclick = () => setLens("fisheye");
  document.getElementById("straight").onclick = () => setLens("straight");
  function addCell(row, value, className) {
    const cell = document.createElement("td");
    cell.textContent = value;
    if (className) cell.className = className;
    row.appendChild(cell);
  }
  async function poll() {
    try {
      const status = await (await fetch("/status")).json();
      const objects = status.objects || [];
      text("phase", status.phase || "idle");
      const matched = new Set(objects.filter((item) => item.color_ok).map((item) => item.name));
      const ready = matched.has("sports_ball") && matched.has("stop_sign");
      text("specified", ready ? "duckie and stop sign match" : "waiting for the duckie and the stop sign");
      const table = document.getElementById("objects");
      while (table.rows.length > 1) table.deleteRow(1);
      for (const item of objects) {
        const row = table.insertRow();
        addCell(row, item.label || item.name);
        addCell(row, item.name + " " + Number(item.confidence).toFixed(2));
        addCell(row, item.color ? item.color + " " + Number(item.color_confidence).toFixed(2) : "-");
        addCell(row, item.expected_color ? (item.color_ok ? "yes" : "expected " + item.expected_color) : "-", item.color_ok ? "ok" : "bad");
      }
    } catch (error) {
      text("phase", "no connection");
    }
    setTimeout(poll, 250);
  }
  poll();
  const cam = document.getElementById("cam");
  const showNext = () => { cam.src = "/frame.jpg?lens=" + lens + "&t=" + Date.now(); };
  cam.onload = () => setTimeout(showNext, 30);
  cam.onerror = () => setTimeout(showNext, 1000);
  showNext();
</script>
</body>
</html>
"""

DRIVE_PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Drive the drone</title>
<style>""" + STYLE + """</style>
</head>
<body>
<nav>
  <a href="/">Camera</a>
  <a class="active" href="/drive">Drive</a>
</nav>
<main>
  <div>
    <p>This view moves the simulated drone. W and S drive, A and D yaw, R or the up arrow climbs, F or the down arrow descends, and X returns it to the spawn pose. It has to be off the ground before W moves it forward. Click this page first so the keys land here. The Matrix window is the outside view of the same drone.</p>
    <div class="pad">
      <button type="button" data-key="left">A yaw left</button>
      <button type="button" data-key="forward">W forward</button>
      <button type="button" data-key="right">D yaw right</button>
      <button type="button" data-key="ascend">R up</button>
      <button type="button" data-key="backward">S back</button>
      <button type="button" data-key="descend">F down</button>
      <span></span>
      <button type="button" id="reset">X reset</button>
    </div>
    <p id="state">idle</p>
  </div>
</main>
<script>
  const held = {forward:false, backward:false, left:false, right:false, ascend:false, descend:false, reset:false};
  const names = {KeyW:"forward", KeyS:"backward", KeyA:"left", KeyD:"right", KeyR:"ascend", ArrowUp:"ascend", KeyF:"descend", ArrowDown:"descend"};
  const state = document.getElementById("state");
  async function send() {
    try {
      const response = await fetch("/drive/command", {method:"POST", headers:{"Content-Type":"application/json"}, body: JSON.stringify(held)});
      const body = await response.json();
      const active = Object.entries(held).filter((entry) => entry[1]).map((entry) => entry[0]);
      state.textContent = (active.length ? active.join(", ") : "idle") + (body.drive_error ? " (" + body.drive_error + ")" : "");
    } catch (error) {
      state.textContent = "no connection";
    }
  }
  setInterval(send, 100);
  addEventListener("keydown", (event) => {
    if (event.code === "KeyX") { held.reset = true; event.preventDefault(); return; }
    const name = names[event.code];
    if (!name) return;
    held[name] = true;
    event.preventDefault();
  });
  addEventListener("keyup", (event) => {
    if (event.code === "KeyX") { held.reset = false; return; }
    const name = names[event.code];
    if (!name) return;
    held[name] = false;
    event.preventDefault();
  });
  for (const button of document.querySelectorAll("[data-key]")) {
    const name = button.dataset.key;
    button.onpointerdown = (event) => { held[name] = true; event.preventDefault(); };
    button.onpointerup = () => { held[name] = false; };
    button.onpointerleave = () => { held[name] = false; };
  }
  document.getElementById("reset").onclick = async () => {
    held.reset = true;
    await send();
    setTimeout(() => { held.reset = false; }, 400);
  };
</script>
</body>
</html>
"""


class DriveRelay:
    """Repeat the latest drive command while a key is held."""

    def __init__(self) -> None:
        self.command = empty_command()
        self.reset_until = 0.0
        self.connected = False
        self.error = ""
        self._lock = threading.Lock()
        self._stop = False
        self._was_active = False
        threading.Thread(target=self._loop, daemon=True).start()

    def set_command(self, payload: dict) -> None:
        with self._lock:
            for name in COMMAND_FIELDS:
                self.command[name] = bool(payload.get(name))
            if self.command["reset"]:
                self.reset_until = time() + 0.4

    def snapshot(self) -> dict[str, bool]:
        with self._lock:
            command = dict(self.command)
            if time() < self.reset_until:
                command["reset"] = True
            return command

    def _loop(self) -> None:
        link = None
        url_index = 0
        urls = [url for url in DRIVE_URLS if url]
        while not self._stop:
            command = self.snapshot()
            active = any(command.values())
            if not active and not self._was_active:
                sleep(0.05)
                continue
            if not active:
                command = empty_command()
            try:
                if link is None:
                    url = urls[url_index % len(urls)]
                    link = DriveLink(url)
                    link.connect()
                    self.connected = True
                    self.error = ""
                link.send(command)
                self._was_active = active
            except Exception as exc:
                self.connected = False
                self.error = str(exc)
                if link is not None:
                    link.close()
                link = None
                url_index += 1
                sleep(0.4)
                continue
            sleep(0.1)

    def close(self) -> None:
        self._stop = True


class CameraView(Node):
    """Buffers the latest annotated frame and status, and serves them over HTTP."""

    def __init__(self) -> None:
        super().__init__("camera_view")
        cv2.setNumThreads(1)
        self.declare_parameter("port", 8090)
        port = int(self.get_parameter("port").value)

        self._frame_ready = threading.Condition()
        self._frame: bytes | None = None
        self._frame_number = 0
        self._phase = ""
        self._detection: list[float] | None = None
        self._detection_at = 0.0
        self._objects: list[dict] = []
        self._objects_at = 0.0
        self._drive = DriveRelay()

        self.create_subscription(CompressedImage, ANNOTATED_TOPIC, self._on_frame, QoSProfile(depth=1))
        self.create_subscription(Float32MultiArray, DETECTION_TOPIC, self._on_detection, 10)
        self.create_subscription(String, OBJECTS_TOPIC, self._on_objects, 10)
        self.create_subscription(String, PHASE_TOPIC, self._on_phase, 10)

        self._server = ThreadingHTTPServer(("0.0.0.0", port), self._make_handler())
        self._server.daemon_threads = True
        threading.Thread(target=self._server.serve_forever, daemon=True).start()
        self.get_logger().info(
            f"Camera view on port {port}: / is the fisheye camera, /drive moves the drone."
        )

    def _on_frame(self, message: CompressedImage) -> None:
        with self._frame_ready:
            self._frame = bytes(message.data)
            self._frame_number += 1
            self._frame_ready.notify_all()

    def _on_detection(self, message: Float32MultiArray) -> None:
        self._detection = list(message.data)
        self._detection_at = monotonic()

    def _on_objects(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        objects = payload.get("objects", [])
        if isinstance(objects, list):
            self._objects = objects
            self._objects_at = monotonic()

    def _on_phase(self, message: String) -> None:
        self._phase = message.data

    def _status(self) -> dict:
        detection = None
        data = self._detection
        if data is not None and len(data) >= 8 and monotonic() - self._detection_at <= STALE_AFTER_S:
            detection = {
                "class_id": int(data[0]),
                "confidence": data[1],
                "error_x": data[4],
                "error_y": data[5],
                "width": data[6],
                "height": data[7],
            }
        objects = self._objects if monotonic() - self._objects_at <= STALE_AFTER_S else []
        return {
            "phase": self._phase,
            "detection": detection,
            "objects": objects,
            "drive_connected": self._drive.connected,
            "drive_error": self._drive.error,
        }

    def _frame_bytes(self, lens: str) -> bytes | None:
        frame = self._frame
        if frame is None:
            return None
        if lens != "straight":
            return frame
        image = cv2.imdecode(np.frombuffer(frame, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            return frame
        straight = rectify(image)
        ok, encoded = cv2.imencode(".jpg", straight, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            return frame
        return encoded.tobytes()

    def _next_frame(self, last_number: int) -> tuple[bytes, int] | None:
        """Block until a frame newer than last_number arrives (or time out)."""
        with self._frame_ready:
            self._frame_ready.wait_for(lambda: self._frame_number != last_number, timeout=5.0)
            if self._frame is None or self._frame_number == last_number:
                return None
            return self._frame, self._frame_number

    def _make_handler(self) -> type[BaseHTTPRequestHandler]:
        view = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_args) -> None:
                pass

            def _send(self, content_type: str, body: bytes) -> None:
                self.send_response(200)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self) -> None:
                path = urlparse(self.path).path
                if path == "/":
                    self._send("text/html; charset=utf-8", PAGE.encode())
                elif path == "/drive":
                    self._send("text/html; charset=utf-8", DRIVE_PAGE.encode())
                elif path == "/status":
                    self._send("application/json", json.dumps(view._status()).encode())
                elif path == "/frame.jpg":
                    query = parse_qs(urlparse(self.path).query)
                    lens = query.get("lens", ["fisheye"])[0]
                    frame = view._frame_bytes(lens)
                    if frame is None:
                        self.send_error(503, "no frame yet")
                    else:
                        self._send("image/jpeg", frame)
                elif path == "/stream":
                    self._stream()
                else:
                    self.send_error(404)

            def do_POST(self) -> None:
                path = urlparse(self.path).path
                if path != "/drive/command":
                    self.send_error(404)
                    return
                length = int(self.headers.get("Content-Length", "0"))
                raw = self.rfile.read(length) if length else b"{}"
                try:
                    payload = json.loads(raw.decode("utf-8"))
                except json.JSONDecodeError:
                    self.send_error(400, "expected JSON")
                    return
                if not isinstance(payload, dict):
                    self.send_error(400, "expected an object")
                    return
                view._drive.set_command(payload)
                self._send("application/json", json.dumps(view._status()).encode())

            def _stream(self) -> None:
                self.send_response(200)
                self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                last_number = -1
                try:
                    while True:
                        item = view._next_frame(last_number)
                        if item is None:
                            continue
                        jpeg, last_number = item
                        self.wfile.write(
                            f"--{BOUNDARY}\r\nContent-Type: image/jpeg\r\n"
                            f"Content-Length: {len(jpeg)}\r\n\r\n".encode()
                        )
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass

        return Handler

    def destroy_node(self) -> bool:
        self._drive.close()
        self._server.shutdown()
        self._server.server_close()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CameraView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
