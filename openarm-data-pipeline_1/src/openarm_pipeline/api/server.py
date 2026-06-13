"""FastAPI app (tasks 4 REST + 5 dashboard).

Endpoints
---------
GET  /                              dashboard
GET  /api/state                     live joint/sync/recording snapshot (JSON)
GET  /api/episodes                  list episodes + metadata
GET  /api/episodes/{id}             one episode's metadata
GET  /api/episodes/{id}/download    download the raw .mcap
POST /api/recording/start           arm + begin recording   {notes?}
POST /api/recording/stop            stop + finalize episode
GET  /api/cameras/{name}/preview    latest JPEG for one camera
WS   /ws                            ~10 Hz live state push
"""

from __future__ import annotations

import asyncio
import contextlib
from pathlib import Path

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, HTMLResponse, Response

from openarm_pipeline.recorder import Recorder

STATIC = Path(__file__).resolve().parent / "static"


@contextlib.asynccontextmanager
async def lifespan(app: FastAPI):
    rec = Recorder(mock=True)
    await rec.start()
    app.state.recorder = rec
    try:
        yield
    finally:
        await rec.stop()


app = FastAPI(title="OpenArm 2.0 Data Collection", lifespan=lifespan)


def _rec(app: FastAPI) -> Recorder:
    return app.state.recorder


@app.get("/", response_class=HTMLResponse)
async def dashboard() -> str:
    return (STATIC / "dashboard.html").read_text()


@app.get("/api/state")
async def state():
    return _rec(app).live_state()


@app.get("/api/episodes")
async def episodes():
    return {"episodes": _rec(app).store.list_all()}


@app.get("/api/episodes/{episode_id}")
async def episode(episode_id: str):
    meta = _rec(app).store.metadata(episode_id)
    if meta is None:
        raise HTTPException(status_code=404, detail=f"no episode {episode_id!r}")
    return meta


@app.get("/api/episodes/{episode_id}/download")
async def download(episode_id: str):
    path = _rec(app).store.path(episode_id)
    if path is None:
        raise HTTPException(status_code=404, detail=f"no episode {episode_id!r}")
    return FileResponse(
        path, media_type="application/octet-stream", filename=f"{episode_id}.mcap"
    )


@app.post("/api/recording/start")
async def start_recording(body: dict | None = None):
    notes = (body or {}).get("notes", "")
    return _rec(app).start_recording(notes=notes)


@app.post("/api/recording/stop")
async def stop_recording():
    return _rec(app).stop_recording()


@app.get("/api/cameras/{name}/preview")
async def camera_preview(name: str):
    cam = _rec(app).cameras.get(name)
    if cam is None or cam.latest is None:
        raise HTTPException(status_code=404, detail=f"no frame for camera {name!r}")
    return Response(content=cam.latest.jpeg, media_type="image/jpeg")


@app.websocket("/ws")
async def ws(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            await websocket.send_json(_rec(app).live_state())
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        pass
