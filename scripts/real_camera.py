#!/usr/bin/env python3
"""Run the camera-views detector on the physical birdie camera.

The picture and the class/color table are the same checks as the simulator
page. This process only reads the camera. It does not arm the drone or
publish a flight setpoint.
"""

from __future__ import annotations

import base64
import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
from websocket import WebSocketTimeoutException, create_connection

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "packages" / "drone_vision"))

from drone_vision.vision_lib import SPECIFIED_OBJECTS, rectify
from drone_vision.yolo_infer import draw_rows, infer, load_net, row_for


HOST = "127.0.0.1"
PORT = 8093
BRIDGE = "ws://128.148.140.76:9001"
TOPIC = "/birdie/camera_node/image/compressed"
MODEL = ROOT / "assets" / "yolov8n.onnx"
CONFIDENCE = 0.08
NMS = 0.45
WATCHED = [11, 32]
VIEW_MAX_AREA = 0.45

latest_bgr = None
arrivals: list[float] = []
rows: list[dict] = []
infer_ms = 0.0
lock = threading.Lock()

PAGE = """<!doctype html>
<meta charset="utf-8">
<title>birdie camera</title>
<style>
  body { margin: 0; background: #111; color: #ddd; font: 14px/1.4 system-ui, sans-serif; }
  nav { display: flex; gap: 8px; align-items: center; padding: 12px 16px; background: #1b1b1b; }
  button { color: #eee; background: #333; border: 0; border-radius: 6px; padding: 8px 12px; font: inherit; }
  button.active { background: #2f6fed; }
  main { display: flex; flex-wrap: wrap; gap: 16px; padding: 16px; align-items: flex-start; }
  img { max-width: min(100%, 480px); background: #000; }
  table { border-collapse: collapse; }
  th, td { text-align: left; padding: 4px 10px 4px 0; }
  th { color: #888; font-weight: 500; }
  .ok { color: #8d8; }
  .bad { color: #f88; }
  #fps { font-size: 42px; font-weight: 700; }
  p { max-width: 36rem; color: #aaa; }
</style>
<nav>
  <button id="fisheye" class="active" type="button">Fisheye</button>
  <button id="straight" type="button">Straight</button>
</nav>
<main>
  <img id="cam" alt="front camera">
  <div>
    <div id="fps">–</div>
    <table>
      <tr><th>Drone</th><td>birdie, real camera</td></tr>
      <tr><th>Lens</th><td id="lens">fisheye</td></tr>
      <tr><th>Detect</th><td id="detect">starting</td></tr>
      <tr><th>Flight</th><td>off</td></tr>
    </table>
    <table id="objects">
      <tr><th>Object</th><th>Class</th><th>Color</th><th>Match</th></tr>
    </table>
    <p>Same YOLOv8n check as the simulator: a duckie is sports ball plus yellow, a stop sign is stop sign plus red. Flight stays off on this drone.</p>
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
      const status = await (await fetch("/status", {cache: "no-store"})).json();
      text("fps", status.fps.toFixed(1) + " fps");
      text("detect", status.infer_ms.toFixed(0) + " ms");
      document.title = status.fps.toFixed(1) + " fps — birdie";
      const table = document.getElementById("objects");
      while (table.rows.length > 1) table.deleteRow(1);
      for (const item of status.objects) {
        const row = table.insertRow();
        addCell(row, item.label || item.name);
        addCell(row, item.name + " " + Number(item.confidence).toFixed(2));
        addCell(row, item.color ? item.color + " " + Number(item.color_confidence).toFixed(2) : "-");
        addCell(row, item.expected_color ? (item.color_ok ? "yes" : "expected " + item.expected_color) : "-", item.color_ok ? "ok" : "bad");
      }
    } catch (error) {
      text("detect", "no connection");
    }
    setTimeout(poll, 250);
  }
  poll();
  const cam = document.getElementById("cam");
  const showNext = () => { cam.src = "/frame.jpg?lens=" + lens + "&t=" + Date.now(); };
  cam.onload = () => setTimeout(showNext, 30);
  cam.onerror = () => setTimeout(showNext, 500);
  showNext();
</script>
"""


def rate(stamps, now):
    fresh = [stamp for stamp in stamps if stamp >= now - 1.0]
    if len(fresh) < 2:
        return 0.0, fresh
    return (len(fresh) - 1) / (fresh[-1] - fresh[0]), fresh


def camera_loop():
    global latest_bgr, arrivals
    while True:
        try:
            ws = create_connection(BRIDGE, timeout=8)
            ws.settimeout(5)
            ws.send(json.dumps({
                "op": "subscribe",
                "topic": TOPIC,
                "queue_length": 1,
                "throttle_rate": 0,
            }))
            print("camera connected", flush=True)
            while True:
                try:
                    raw = ws.recv()
                except WebSocketTimeoutException:
                    raise TimeoutError("camera stalled")
                if '"publish"' not in raw[:120]:
                    continue
                payload = json.loads(raw).get("msg", {}).get("data")
                if not payload:
                    continue
                blob = payload if isinstance(payload, str) else None
                if blob is None:
                    continue
                image = cv2.imdecode(np.frombuffer(base64.b64decode(blob), np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    continue
                now = time.time()
                with lock:
                    latest_bgr = image
                    arrivals.append(now)
                    if len(arrivals) > 120:
                        arrivals = arrivals[-90:]
        except Exception as exc:
            print(f"camera reconnect: {exc}", flush=True)
            time.sleep(1)


def infer_loop(net):
    global rows, infer_ms
    last_log = 0.0
    seen = None
    while True:
        with lock:
            image = None if latest_bgr is None else latest_bgr.copy()
        if image is None:
            time.sleep(0.05)
            continue
        started = time.perf_counter()
        detections = infer(net, image, CONFIDENCE, NMS, WATCHED)
        elapsed = (time.perf_counter() - started) * 1000.0
        area_limit = float(image.shape[0] * image.shape[1])
        visible = []
        for detection in detections:
            box = detection[1]
            area = (box[2] - box[0]) * (box[3] - box[1]) / area_limit
            if area <= VIEW_MAX_AREA:
                visible.append(detection)
        built = [row_for(image, detection) for detection in visible]
        with lock:
            rows = built
            infer_ms = elapsed
        now = time.time()
        signature = tuple((row["name"], row["confidence"], row["color"]) for row in built)
        if signature != seen or now - last_log > 2.0:
            seen = signature
            last_log = now
            summary = ", ".join(
                f"{row['name']} {row['confidence']:.2f} {row['color'] or '-'}" for row in built
            ) or "none"
            print(f"detect {elapsed:.0f} ms: {summary}", flush=True)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        return

    def _send(self, content_type, body):
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/":
            self._send("text/html; charset=utf-8", PAGE.encode())
            return
        if path == "/status":
            now = time.time()
            with lock:
                fps, fresh = rate(arrivals, now)
                arrivals[:] = fresh
                payload = {
                    "fps": round(fps, 1),
                    "infer_ms": round(infer_ms, 1),
                    "objects": [
                        {key: value for key, value in row.items() if key not in ("x_min", "y_min", "x_max", "y_max")}
                        for row in rows
                    ],
                    "flight": "off",
                    "available": [spec["label"] for spec in SPECIFIED_OBJECTS.values()],
                }
            self._send("application/json", json.dumps(payload).encode())
            return
        if path == "/frame.jpg":
            lens = parse_qs(urlparse(self.path).query).get("lens", ["fisheye"])[0]
            with lock:
                image = None if latest_bgr is None else latest_bgr.copy()
                drawn_rows = list(rows)
            if image is None:
                self.send_error(503, "no frame yet")
                return
            painted = draw_rows(image, drawn_rows)
            if lens == "straight":
                painted = rectify(painted)
            ok, encoded = cv2.imencode(".jpg", painted, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok:
                self.send_error(500, "encode failed")
                return
            self._send("image/jpeg", encoded.tobytes())
            return
        self.send_error(404)

    def do_POST(self):
        body = json.dumps({
            "ok": False,
            "error": "Flight stays off on the real drone.",
        }).encode()
        self.send_response(403)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def main():
    cv2.setNumThreads(2)
    net = load_net(str(MODEL))
    threading.Thread(target=camera_loop, daemon=True).start()
    threading.Thread(target=infer_loop, args=(net,), daemon=True).start()
    server = ThreadingHTTPServer((HOST, PORT), Handler)
    print(f"open http://{HOST}:{PORT}/", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
