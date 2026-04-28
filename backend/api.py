"""FastAPI app: REST + WebSocket live state + watchdog."""
from __future__ import annotations

import asyncio
import logging
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Optional

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from .can_bus import autodetect_channel, open_bus
from .config import AppConfig
from .gripper import Gripper
from .motion import LimitViolation, Motion
from .teach import TeachError, TeachRecorder

log = logging.getLogger("arctos.api")

FRONTEND_DIR = Path(__file__).resolve().parent.parent / "frontend"
CONFIG_PATH = Path(__file__).resolve().parent.parent / "config.yaml"


# ---------------- Request models ----------------

class JogStartReq(BaseModel):
    can_id: int = Field(ge=1, le=6)
    direction: int = Field(ge=0, le=1)
    speed_pct: float = Field(ge=-1.0, le=1.0)


class JogStopReq(BaseModel):
    can_id: int = Field(ge=1, le=6)


class MoveReq(BaseModel):
    can_id: int = Field(ge=1, le=6)
    degrees: float
    speed_pct: float = Field(0.5, gt=0.0, le=1.0)


class EnableReq(BaseModel):
    on: bool = True


class MicrostepsReq(BaseModel):
    can_id: int = Field(ge=1, le=6)
    microsteps: int = Field(ge=1, le=256)


class CurrentReq(BaseModel):
    can_id: int = Field(ge=1, le=6)
    milliamps: int = Field(ge=0, le=5200)


class GripperReq(BaseModel):
    position: int = Field(ge=0, le=255)


class CaptureReq(BaseModel):
    dwell_ms: int = Field(0, ge=0, le=600_000)
    speed_pct: float = Field(0.5, gt=0.0, le=1.0)


class WaypointPatch(BaseModel):
    dwell_ms: Optional[int] = Field(None, ge=0, le=600_000)
    speed_pct: Optional[float] = Field(None, gt=0.0, le=1.0)
    gripper: Optional[int] = Field(None, ge=0, le=255)


class ReorderReq(BaseModel):
    to: int = Field(ge=0)


class ProgramNameReq(BaseModel):
    name: str = Field(min_length=1, max_length=64)


# ---------------- App wiring ----------------

class AppState:
    cfg: AppConfig
    motion: Motion
    gripper: Gripper
    teach: TeachRecorder
    last_heartbeat: float = 0.0
    ws_connected: int = 0

    def __init__(self) -> None:
        self.cfg = AppConfig.load(CONFIG_PATH)
        channel = self.cfg.can.channel or (autodetect_channel() if self.cfg.can.backend == "slcan" else None)
        bus = open_bus(self.cfg.can.backend, channel=channel, bitrate=self.cfg.can.bitrate)
        self.motion = Motion(self.cfg, bus)
        self.gripper = Gripper(self.cfg.gripper, bus)
        self.teach = TeachRecorder(motion=self.motion, gripper=self.gripper)
        log.info(
            "motion ready, backend=%s, gripper=%s",
            self.cfg.can.backend,
            "on" if self.cfg.gripper.enabled else "off",
        )


state: Optional[AppState] = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    global state
    state = AppState()
    watchdog = asyncio.create_task(_watchdog_loop())
    try:
        yield
    finally:
        watchdog.cancel()
        state.motion.bus.shutdown()


app = FastAPI(title="arctos-pi", lifespan=lifespan)


def _motion() -> Motion:
    assert state is not None
    return state.motion


def _gripper() -> Gripper:
    assert state is not None
    return state.gripper


def _teach() -> TeachRecorder:
    assert state is not None
    return state.teach


# ---------------- REST endpoints ----------------

@app.get("/api/state")
def get_state():
    return {
        "axes": _motion().state_dict(),
        "gripper": _gripper().state_dict(),
        "teach": _teach_summary(),
        "backend": state.cfg.can.backend,
    }


def _teach_summary() -> dict:
    """Lightweight teach summary for the WS broadcast — full waypoint list
    is fetched on demand via GET /api/teach to keep the 5 Hz tick small."""
    t = _teach()
    return {"count": len(t.waypoints), "loaded_name": t.loaded_name, "dirty": t.dirty}


@app.get("/api/config")
def get_config():
    g = state.cfg.gripper
    return {
        "backend": state.cfg.can.backend,
        "axes": [
            {
                "can_id": ax.can_id, "name": ax.name,
                "gear_ratio": ax.gear_ratio, "pulses_per_rev": ax.pulses_per_rev,
                "invert": ax.invert, "max_speed": ax.max_speed,
                "soft_limit_min": ax.soft_limit_min, "soft_limit_max": ax.soft_limit_max,
                "default_current_ma": ax.default_current_ma,
                "default_microsteps": ax.default_microsteps,
            }
            for ax in state.cfg.axes
        ],
        "gripper": {
            "enabled": g.enabled, "can_id": g.can_id,
            "open_position": g.open_position, "close_position": g.close_position,
            "default_position": g.default_position,
        },
    }


@app.post("/api/enable")
def enable_all(req: EnableReq):
    _motion().enable_all(req.on)
    return {"ok": True}


@app.post("/api/estop")
def estop():
    _motion().emergency_stop()
    return {"ok": True}


@app.post("/api/jog/start")
def jog_start(req: JogStartReq):
    _motion().jog_start(req.can_id, req.direction, req.speed_pct)
    return {"ok": True}


@app.post("/api/jog/stop")
def jog_stop(req: JogStopReq):
    _motion().jog_stop(req.can_id)
    return {"ok": True}


@app.post("/api/jog/stop_all")
def jog_stop_all():
    _motion().jog_stop_all()
    return {"ok": True}


@app.post("/api/move")
def move(req: MoveReq):
    try:
        _motion().move_to_degrees(req.can_id, req.degrees, req.speed_pct)
    except LimitViolation as e:
        raise HTTPException(status_code=400, detail=str(e))
    return {"ok": True}


@app.post("/api/microsteps")
def microsteps(req: MicrostepsReq):
    try:
        _motion().set_microsteps(req.can_id, req.microsteps)
    except PermissionError as e:
        raise HTTPException(status_code=403, detail=str(e))
    return {"ok": True}


@app.post("/api/current")
def current(req: CurrentReq):
    try:
        _motion().set_current(req.can_id, req.milliamps)
    except PermissionError as e:
        raise HTTPException(status_code=403, detail=str(e))
    return {"ok": True}


@app.post("/api/refresh")
def refresh():
    _motion().request_all_positions()
    return {"ok": True}


# ---------------- Gripper ----------------

@app.post("/api/gripper")
def gripper_set(req: GripperReq):
    try:
        sent = _gripper().set_position(req.position)
    except RuntimeError as e:
        raise HTTPException(status_code=409, detail=str(e))
    return {"ok": True, "position": sent}


@app.post("/api/gripper/open")
def gripper_open():
    try:
        sent = _gripper().open()
    except RuntimeError as e:
        raise HTTPException(status_code=409, detail=str(e))
    return {"ok": True, "position": sent}


@app.post("/api/gripper/close")
def gripper_close():
    try:
        sent = _gripper().close()
    except RuntimeError as e:
        raise HTTPException(status_code=409, detail=str(e))
    return {"ok": True, "position": sent}


# ---------------- Teach / record ----------------

@app.get("/api/teach")
def teach_state():
    return _teach().state_dict()


@app.post("/api/teach/capture")
def teach_capture(req: CaptureReq):
    try:
        wp = _teach().capture(dwell_ms=req.dwell_ms, speed_pct=req.speed_pct)
    except TeachError as e:
        raise HTTPException(status_code=400, detail=str(e))
    return {"ok": True, "waypoint": wp.to_dict(), "count": len(_teach().waypoints)}


@app.post("/api/teach/clear")
def teach_clear():
    _teach().clear()
    return {"ok": True}


@app.delete("/api/teach/{idx}")
def teach_delete(idx: int):
    try:
        _teach().delete(idx)
    except TeachError as e:
        raise HTTPException(status_code=400, detail=str(e))
    return {"ok": True}


@app.patch("/api/teach/{idx}")
def teach_patch(idx: int, req: WaypointPatch):
    try:
        wp = _teach().update(idx, dwell_ms=req.dwell_ms, speed_pct=req.speed_pct, gripper=req.gripper)
    except TeachError as e:
        raise HTTPException(status_code=400, detail=str(e))
    return {"ok": True, "waypoint": wp.to_dict()}


@app.post("/api/teach/{idx}/reorder")
def teach_reorder(idx: int, req: ReorderReq):
    try:
        _teach().reorder(idx, req.to)
    except TeachError as e:
        raise HTTPException(status_code=400, detail=str(e))
    return {"ok": True}


@app.get("/api/teach/programs")
def teach_list_programs():
    return {"programs": _teach().list_programs()}


@app.post("/api/teach/save")
def teach_save(req: ProgramNameReq):
    try:
        path = _teach().save(req.name)
    except TeachError as e:
        raise HTTPException(status_code=400, detail=str(e))
    return {"ok": True, "name": req.name, "path": str(path)}


@app.post("/api/teach/load")
def teach_load(req: ProgramNameReq):
    try:
        _teach().load(req.name)
    except TeachError as e:
        raise HTTPException(status_code=404, detail=str(e))
    return {"ok": True, "count": len(_teach().waypoints)}


@app.delete("/api/teach/programs/{name}")
def teach_delete_program(name: str):
    try:
        _teach().delete_program(name)
    except TeachError as e:
        raise HTTPException(status_code=404, detail=str(e))
    return {"ok": True}


# ---------------- WebSocket + watchdog ----------------

class WsHub:
    """Broadcasts state to all connected clients. Each connection pings at
    heartbeat_ms; if no pings from any client during a jog, motion stops.
    """
    def __init__(self) -> None:
        self.clients: set[WebSocket] = set()
        self._lock = asyncio.Lock()

    async def connect(self, ws: WebSocket) -> None:
        await ws.accept()
        async with self._lock:
            self.clients.add(ws)

    async def disconnect(self, ws: WebSocket) -> None:
        async with self._lock:
            self.clients.discard(ws)

    async def broadcast(self, payload: dict) -> None:
        dead = []
        for ws in list(self.clients):
            try:
                await ws.send_json(payload)
            except Exception:
                dead.append(ws)
        for ws in dead:
            await self.disconnect(ws)


hub = WsHub()


@app.websocket("/api/ws")
async def ws_endpoint(ws: WebSocket):
    await hub.connect(ws)
    state.ws_connected += 1
    state.last_heartbeat = time.monotonic()
    try:
        await ws.send_json({"type": "hello", "backend": state.cfg.can.backend})
        while True:
            msg = await ws.receive_json()
            if msg.get("type") == "ping":
                state.last_heartbeat = time.monotonic()
                await ws.send_json({"type": "pong", "t": state.last_heartbeat})
    except WebSocketDisconnect:
        pass
    finally:
        await hub.disconnect(ws)
        state.ws_connected = max(0, state.ws_connected - 1)


async def _watchdog_loop():
    """Broadcast state regularly; stop motion if heartbeat is stale
    AND there are active jog commands — in MVP we conservatively stop-all.
    Also polls every axis for pulses each tick so the UI shows live position.
    """
    interval = 0.2
    timeout_s = (state.cfg.server.heartbeat_ms / 1000.0) * 5  # 5x heartbeat
    # Prime the pump so the first broadcast isn't all zeros.
    try:
        _motion().request_all_positions()
    except Exception:
        log.exception("initial position poll failed")
    while True:
        await asyncio.sleep(interval)
        try:
            await hub.broadcast({
                "type": "state",
                "axes": _motion().state_dict(),
                "gripper": _gripper().state_dict(),
                "teach": _teach_summary(),
            })
            if state.ws_connected > 0:
                stale = (time.monotonic() - state.last_heartbeat) > timeout_s
                if stale:
                    log.warning("WS heartbeat stale (>%.2fs); stopping jogs", timeout_s)
                    _motion().jog_stop_all()
                    state.last_heartbeat = time.monotonic()
            # Kick off reads for the next tick; replies land via _on_frame
            # before the next broadcast, so state stays fresh at ~5 Hz.
            _motion().request_all_positions()
        except Exception:
            log.exception("watchdog iteration failed")


# ---------------- Frontend ----------------

@app.get("/")
def root():
    index = FRONTEND_DIR / "index.html"
    if index.exists():
        return FileResponse(index)
    raise HTTPException(404, "frontend not built")


app.mount("/static", StaticFiles(directory=str(FRONTEND_DIR)), name="static")
