"""Live browser view of the annotated YOLO camera stream. Viewer only.

Serves the detector's annotated frames (latest frame and MJPEG), plus a small
status page with the flight phase and the target error. It publishes nothing and sends no
flight commands.
"""

from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from time import monotonic

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray, String


ANNOTATED_TOPIC = "/drone_vision/yolo_image/compressed"
DETECTION_TOPIC = "/drone_vision/yolo_detection"
PHASE_TOPIC = "/drone_vision/calibration_phase"
STALE_AFTER_S = 1.0
BOUNDARY = "frame"

PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Drone camera</title>
<style>
  body { margin: 0; background: #111; color: #ddd; font: 14px/1.4 system-ui, sans-serif; }
  main { display: flex; flex-wrap: wrap; gap: 16px; padding: 16px; align-items: flex-start; }
  img { height: min(80vh, 640px); background: #000; }
  dl { margin: 0; display: grid; grid-template-columns: auto auto; gap: 4px 16px; }
  dt { color: #888; }
  dd { margin: 0; font-variant-numeric: tabular-nums; }
</style>
</head>
<body>
<main>
  <img id="cam" alt="camera">
  <dl>
    <dt>Phase</dt><dd id="phase">-</dd>
    <dt>Target</dt><dd id="seen">-</dd>
    <dt>Confidence</dt><dd id="confidence">-</dd>
    <dt>Error x</dt><dd id="error_x">-</dd>
    <dt>Box area</dt><dd id="area">-</dd>
  </dl>
</main>
<script>
  const text = (id, value) => { document.getElementById(id).textContent = value; };
  async function poll() {
    try {
      const status = await (await fetch("/status")).json();
      const d = status.detection;
      text("phase", status.phase || "-");
      text("seen", d ? "detected" : "not detected");
      text("confidence", d ? d.confidence.toFixed(2) : "-");
      text("error_x", d ? d.error_x.toFixed(3) : "-");
      text("area", d ? (d.width * d.height * 100).toFixed(2) + " %" : "-");
    } catch (error) {
      text("phase", "no connection");
    }
    setTimeout(poll, 250);
  }
  poll();

  // Fetch the latest frame repeatedly instead of holding one stream open: it
  // recovers by itself if the workbench restarts or a connection stalls.
  const cam = document.getElementById("cam");
  const showNext = () => { cam.src = "/frame.jpg?" + Date.now(); };
  cam.onload = () => setTimeout(showNext, 30);
  cam.onerror = () => setTimeout(showNext, 1000);
  showNext();
</script>
</body>
</html>
"""


class CameraView(Node):
    """Buffers the latest annotated frame and status, and serves them over HTTP."""

    def __init__(self) -> None:
        super().__init__("camera_view")
        self.declare_parameter("port", 8090)
        port = int(self.get_parameter("port").value)

        self._frame_ready = threading.Condition()
        self._frame: bytes | None = None
        self._frame_number = 0
        self._phase = ""
        self._detection: list[float] | None = None
        self._detection_at = 0.0

        self.create_subscription(CompressedImage, ANNOTATED_TOPIC, self._on_frame, QoSProfile(depth=1))
        self.create_subscription(Float32MultiArray, DETECTION_TOPIC, self._on_detection, 10)
        self.create_subscription(String, PHASE_TOPIC, self._on_phase, 10)

        self._server = ThreadingHTTPServer(("0.0.0.0", port), self._make_handler())
        self._server.daemon_threads = True
        threading.Thread(target=self._server.serve_forever, daemon=True).start()
        self.get_logger().info(f"Camera view serving on port {port} (/, /frame.jpg, /stream, /status). Viewer only.")

    def _on_frame(self, message: CompressedImage) -> None:
        with self._frame_ready:
            self._frame = bytes(message.data)
            self._frame_number += 1
            self._frame_ready.notify_all()

    def _on_detection(self, message: Float32MultiArray) -> None:
        self._detection = list(message.data)
        self._detection_at = monotonic()

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
        return {"phase": self._phase, "detection": detection}

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
            def log_message(self, *_args) -> None:  # keep the node log quiet
                pass

            def _send(self, content_type: str, body: bytes) -> None:
                self.send_response(200)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self) -> None:
                path = self.path.split("?", 1)[0]
                if path == "/":
                    self._send("text/html; charset=utf-8", PAGE.encode())
                elif path == "/status":
                    self._send("application/json", json.dumps(view._status()).encode())
                elif path == "/frame.jpg":
                    frame = view._frame
                    if frame is None:
                        self.send_error(503, "no frame yet")
                    else:
                        self._send("image/jpeg", frame)
                elif path == "/stream":
                    self._stream()
                else:
                    self.send_error(404)

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
                            continue  # no new frame yet; keep the connection open
                        jpeg, last_number = item
                        self.wfile.write(
                            f"--{BOUNDARY}\r\nContent-Type: image/jpeg\r\n"
                            f"Content-Length: {len(jpeg)}\r\n\r\n".encode()
                        )
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass  # the browser closed the stream

        return Handler

    def destroy_node(self) -> bool:
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
