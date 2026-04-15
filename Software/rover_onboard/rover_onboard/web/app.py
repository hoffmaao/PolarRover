"""Operator-facing web UI served by the rover's WiFi applet.

The operator joins the rover's access point in the field (AccessPopup
fallback AP, or an infrastructure network if one exists), opens
http://<rover-ip>:5000/, and sees:

- A live video feed from the OAK-D Pro (or whichever frame source is
  wired into the rover_onboard.vision pipeline)
- A rover state panel that updates over WebSocket from the StatusBroker

The app is a small FastAPI application to stay consistent with
rover_sim_startup. It's created via ``create_app(broker, frame_source)``
so the service layer can wire in whichever concrete backends are live
(real sources on the rover, nulls in tests).
"""

from typing import Optional


def create_app(
    broker,
    frame_source=None,
    templates_dir: Optional[str] = None,
):
    """Build a FastAPI application bound to the given StatusBroker and
    optional FrameSource. Import is lazy so the package remains
    importable without the 'web' extra."""
    try:
        from fastapi import FastAPI, WebSocket, WebSocketDisconnect
        from fastapi.responses import HTMLResponse, StreamingResponse
        from fastapi.staticfiles import StaticFiles
        from fastapi.templating import Jinja2Templates
        from starlette.requests import Request
    except ImportError as e:
        raise RuntimeError(
            "The web UI requires the 'web' extra. "
            "Install with: pip install rover_onboard[web]"
        ) from e

    import asyncio
    import json
    from pathlib import Path

    app = FastAPI(title="Nisse")

    tpl_dir = Path(templates_dir) if templates_dir else (
        Path(__file__).parent / "templates"
    )
    templates = Jinja2Templates(directory=str(tpl_dir))

    @app.get("/", response_class=HTMLResponse)
    async def index(request: Request):
        snap = broker.latest()
        return templates.TemplateResponse(
            "index.html",
            {"request": request, "snapshot": snap.to_dict()},
        )

    @app.get("/api/state")
    async def api_state():
        return broker.latest().to_dict()

    @app.websocket("/ws/state")
    async def ws_state(ws: WebSocket):
        await ws.accept()
        last_ts = -1.0
        try:
            while True:
                snap = broker.latest()
                if snap.timestamp != last_ts:
                    last_ts = snap.timestamp
                    await ws.send_text(json.dumps(snap.to_dict()))
                await asyncio.sleep(0.1)
        except WebSocketDisconnect:
            return

    @app.get("/video_feed")
    async def video_feed(source: str = "oakd_color"):
        if frame_source is None:
            return HTMLResponse("no frame source attached", status_code=503)
        return StreamingResponse(
            _mjpeg_stream(frame_source),
            media_type="multipart/x-mixed-replace; boundary=frame",
        )

    return app


async def _mjpeg_stream(frame_source):
    """Minimal MJPEG multipart stream. Encoding is deferred to OpenCV when
    the 'vision' extra is installed; otherwise we stop immediately with
    no frames, and the browser shows a broken image."""
    import asyncio

    try:
        import cv2  # type: ignore
    except ImportError:
        return

    boundary = b"--frame\r\n"
    while True:
        frame = frame_source.next_frame()
        if frame is None:
            await asyncio.sleep(0.05)
            continue
        ok, buf = cv2.imencode(".jpg", frame.image)
        if not ok:
            continue
        payload = (
            boundary
            + b"Content-Type: image/jpeg\r\n\r\n"
            + bytes(buf)
            + b"\r\n"
        )
        yield payload
        await asyncio.sleep(0.033)  # ~30 fps cap
