"""FastAPI app: the browser console (static files + one websocket) and a small
HTTP API that scripted end-to-end tests use to act like an operator."""

from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager
from pathlib import Path
from time import monotonic

import cv2
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse, Response
from fastapi.staticfiles import StaticFiles

from ..annotate import annotate
from ..app import Runtime, UIClient

STATIC = Path(__file__).parent / "static"


def create_app(cfg: dict) -> FastAPI:
    runtime_holder: dict[str, Runtime] = {}

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        runtime = Runtime(cfg)
        runtime_holder["rt"] = runtime
        await runtime.start()
        try:
            yield
        finally:
            await runtime.stop()

    app = FastAPI(title="OVERWATCH", lifespan=lifespan)

    def rt() -> Runtime:
        return runtime_holder["rt"]

    @app.websocket("/ws")
    async def websocket(ws: WebSocket) -> None:
        await ws.accept()
        runtime = rt()
        client = UIClient(ws.send_bytes, ws.send_text)
        runtime.clients.add(client)
        client.push_json({"type": "config", "keys": cfg["keys"], "source": runtime.source_name,
                          "classes": runtime.vocab.class_names,
                          "estop_action": cfg["flight"].get("estop_action", "kill")})
        for line in list(runtime.log_lines)[-20:]:
            client.push_json(line)
        sender = asyncio.create_task(client.run())
        try:
            while True:
                message = await ws.receive_json()
                if isinstance(message, dict):
                    reply = await runtime.handle_ui(message, source="key")
                    if reply is not None:
                        client.push_json(reply)
        except (WebSocketDisconnect, RuntimeError):
            pass
        finally:
            client.closed = True
            runtime.clients.discard(client)
            # A browser that goes away must not leave motion keys "held".
            runtime.control.clear_input()
            sender.cancel()

    @app.get("/api/status")
    async def status() -> dict:
        return rt().status()

    @app.post("/api/ui")
    async def ui_message(message: dict) -> JSONResponse:
        """Same messages the browser sends over the websocket (for scripted tests)."""
        reply = await rt().handle_ui(message, source="api")
        return JSONResponse(reply or {"ok": True})

    @app.post("/api/record")
    async def record(body: dict) -> dict:
        runtime = rt()
        if body.get("action") == "start":
            folder = runtime.recorder.start(body.get("name"), body.get("tag", ""), body.get("interval_s"))
            runtime.notify("info", f"Recording to {folder}")
            return {"recording": str(folder)}
        if body.get("action") == "tag":
            runtime.recorder.set_tag(body.get("tag", ""))
            return {"tag": body.get("tag", "")}
        summary = runtime.recorder.stop()
        runtime.notify("info", f"Recording stopped: {summary['frames']} frames")
        return summary

    @app.get("/api/snapshot.jpg")
    async def snapshot() -> Response:
        runtime = rt()
        if runtime.displayed is None or runtime.displayed_result is None:
            return Response(status_code=503, content=b"no frame yet")
        now = monotonic()
        image = annotate(runtime.displayed.jpeg, runtime.displayed_result,
                         runtime.targets.get_target_state(now).to_dict(), runtime.targets.cursor,
                         [b["text"] for b in runtime.banners(now, runtime.health(now))])
        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 90])
        return Response(content=encoded.tobytes(), media_type="image/jpeg", headers={"Cache-Control": "no-store"})

    @app.get("/api/events")
    async def events() -> list:
        return list(rt().events.recent)

    app.mount("/", StaticFiles(directory=STATIC, html=True), name="static")
    return app
