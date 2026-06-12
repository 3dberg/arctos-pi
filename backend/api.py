"""FastAPI app: REST + WebSocket live state + watchdog."""
from __future__ import annotations

import asyncio
import logging
import os
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Optional

from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

try:
    from can.exceptions import CanError
except ImportError:  # python-can not installed in mock-only test envs
    class CanError(Exception):  # type: ignore[no-redef]
        pass

from .can_bus import autodetect_channel, open_bus
from .config import AppConfig
from .gripper import Gripper
from .motion import HomingError, LimitViolation, Motion, NotHomedError
from .ros_client import RosClient, import_error, ros_available
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
    duration_s: float = Field(1.0, gt=0.0, le=120.0)  # ROS-mode single-joint move time


class EnableReq(BaseModel):
    on: bool = True


class MicrostepsReq(BaseModel):
    can_id: int = Field(ge=1, le=6)
    microsteps: int = Field(ge=1, le=256)


class CurrentReq(BaseModel):
    can_id: int = Field(ge=1, le=6)
    milliamps: int = Field(ge=0, le=5200)


class WorkModeReq(BaseModel):
    can_id: int = Field(ge=1, le=6)
    mode: int = Field(ge=0, le=5)  # 0..2=CR_*, 3..5=SR_*; jog requires SR (mode 5 = SR_vFOC)


class GearRatioReq(BaseModel):
    can_id: int = Field(ge=1, le=6)
    gear_ratio: float = Field(gt=0.0, le=10000.0)  # output turns per motor turn


class JointZeroReq(BaseModel):
    can_id: int = Field(ge=1, le=6)
    # The joint's TRUE physical angle right now (URDF/joint convention), used to
    # calibrate home_offset_deg so displayed/entered degrees are joint angles.
    angle_deg: float = Field(0.0, ge=-3600.0, le=3600.0)


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


class HomeReq(BaseModel):
    can_id: int = Field(ge=1, le=6)


class IoReq(BaseModel):
    can_id: int = Field(ge=1, le=6)


class HomeDirReq(BaseModel):
    can_id: int = Field(ge=1, le=6)
    ccw: bool  # False = CW (home_dir 0), True = CCW (home_dir 1)


class RosMoveReq(BaseModel):
    joints_deg: dict[str, float]
    duration_s: float = Field(3.0, gt=0.0, le=120.0)


class RosRunReq(BaseModel):
    seg_time_s: float = Field(2.0, gt=0.0, le=60.0)


# ---------------- App wiring ----------------

class AppState:
    cfg: AppConfig
    motion: Motion
    gripper: Gripper
    teach: TeachRecorder
    ros: Optional[RosClient] = None
    backend: str = ""
    last_heartbeat: float = 0.0
    ws_connected: int = 0

    def __init__(self) -> None:
        self.cfg = AppConfig.load(CONFIG_PATH)
        backend = self._effective_backend()
        channel = self.cfg.can.channel or (autodetect_channel() if backend == "slcan" else None)
        bus = open_bus(backend, channel=channel, bitrate=self.cfg.can.bitrate)
        self.motion = Motion(self.cfg, bus)
        self.gripper = Gripper(self.cfg.gripper, bus)
        self.teach = TeachRecorder(motion=self.motion, gripper=self.gripper)
        self.ros = self._maybe_start_ros()
        self.backend = backend
        log.info(
            "motion ready, backend=%s, gripper=%s, ros=%s",
            backend,
            "on" if self.cfg.gripper.enabled else "off",
            "on" if self.ros else "off",
        )

    def _effective_backend(self) -> str:
        """Enforce the single-CAN-owner rule. When running as a ROS2 client
        (ARCTOS_ROS set and rclpy importable), the arctos_bridge node owns can0,
        so this process must NOT open the hardware bus too — doing both pushes
        the bus into ERROR-PASSIVE and floods parse errors. Downgrade a real
        backend to dry_run so the local Motion/Gripper still exist (REST/WS stay
        up) while real control flows through the /api/ros/* endpoints -> bridge.
        """
        backend = self.cfg.can.backend
        if backend in ("slcan", "socketcan") and os.environ.get("ARCTOS_ROS") and ros_available():
            log.warning(
                "ARCTOS_ROS set: NOT opening %s directly (arctos_bridge owns can0). "
                "Local Motion runs dry_run; drive the robot via the Motion/MoveIt tab.",
                backend,
            )
            return "dry_run"
        return backend

    def _maybe_start_ros(self) -> Optional[RosClient]:
        """Start the optional in-process ROS2 client when ARCTOS_ROS is set and
        rclpy is importable. Failure is non-fatal — the standard CAN control
        path keeps working and the /api/ros/* endpoints report unavailability.
        """
        if not os.environ.get("ARCTOS_ROS"):
            return None
        if not ros_available():
            log.warning("ARCTOS_ROS set but rclpy unavailable: %s", import_error())
            return None
        controller = os.environ.get("ARCTOS_ROS_CONTROLLER", "arctos_arm_controller")
        try:
            client = RosClient(controller_name=controller)
            log.info("ROS2 client started (controller=%s)", controller)
            return client
        except Exception:
            log.exception("ROS2 client failed to start")
            return None


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
        if state.ros is not None:
            state.ros.close()
        state.motion.bus.shutdown()


app = FastAPI(title="arctos-pi", lifespan=lifespan)


async def _can_unavailable_response(exc: BaseException) -> JSONResponse:
    log.error("CAN bus error on request: %s: %s", type(exc).__name__, exc)
    return JSONResponse(
        status_code=503,
        content={
            "detail": (
                f"CAN bus unavailable: {exc}. Bring it back up with: "
                "`sudo ip link set can0 up type can bitrate 500000`. "
                "The app will reconnect automatically."
            )
        },
    )


@app.exception_handler(CanError)
async def _can_error_handler(_req: Request, exc: CanError):
    """python-can's typed errors (CanOperationError, CanInterfaceNotImplemented,
    etc.). Reached only when SocketCanBus.send's auto-reopen also failed.
    """
    return await _can_unavailable_response(exc)


@app.exception_handler(OSError)
async def _os_error_handler(_req: Request, exc: OSError):
    """Raw kernel errors (ENETDOWN, ENODEV, EBADF) that some python-can
    versions don't wrap. Same recovery story as CanError above.
    """
    return await _can_unavailable_response(exc)


@app.exception_handler(ValueError)
async def _value_error_handler(req: Request, exc: ValueError):
    """select() against a closed CAN socket fd=-1 raises ValueError. Treat
    it as a CAN-unavailable case ONLY when it came from the CAN stack —
    a generic ValueError elsewhere should still get FastAPI's normal 500.
    """
    msg = str(exc)
    if "file descriptor" in msg or "fd" in msg.lower():
        return await _can_unavailable_response(exc)
    log.exception("unhandled ValueError on %s", req.url.path)
    return JSONResponse(status_code=500, content={"detail": str(exc)})


def _motion() -> Motion:
    assert state is not None
    return state.motion


def _gripper() -> Gripper:
    assert state is not None
    return state.gripper


def _teach() -> TeachRecorder:
    assert state is not None
    return state.teach


def _ros() -> RosClient:
    assert state is not None
    if state.ros is None:
        if not ros_available():
            detail = f"ROS2 not available in this environment ({import_error()})."
        else:
            detail = ("ROS2 client not enabled. Start the server with ARCTOS_ROS=1 "
                      "in a sourced ROS2 env, with the arctos_bridge node running.")
        raise HTTPException(status_code=503, detail=detail)
    return state.ros


def _ros_mode() -> bool:
    """True when the in-process ROS2 client is active. In this mode the local
    CAN bus is dry_run (the arctos_bridge owns can0), so motion endpoints route
    through ROS and live joint positions come from /joint_states."""
    return state is not None and state.ros is not None


def _axis_name(can_id: int) -> str:
    return state.cfg.axis_by_id(can_id).name


def _require_ros_homed(joint_names) -> None:
    """ROS-path block-until-homed gate. Raises 409 if any involved home_enabled
    joint isn't homed yet (per the bridge-published homed status). The bridge
    also rejects un-homed trajectory goals; this just gives a clean message."""
    if not state.cfg.require_home_before_move:
        return
    homed = state.ros.homed() if state.ros is not None else {}
    by_name = {ax.name: ax for ax in state.cfg.axes}
    unhomed = [n for n in joint_names
               if (ax := by_name.get(n)) is not None and ax.home_enabled and not homed.get(n, False)]
    if unhomed:
        raise HTTPException(status_code=409, detail=f"not homed: {', '.join(unhomed)} — home before moving")


def _axes_state() -> dict:
    """Per-axis state for the UI. In ROS mode the local dry_run Motion has no
    real positions, so overlay the live joint angles the bridge publishes."""
    axes = _motion().state_dict()
    if _ros_mode():
        js = state.ros.joint_states()
        if js:
            deg = dict(zip(js["name"], js["position_deg"]))
            for name, st in axes.items():
                if name in deg:
                    st["degrees"] = deg[name]
        # The bridge owns the real Motion, so homed + live home-switch state come
        # from it (the local dry_run Motion never homes and can't read IO).
        homed = state.ros.homed()
        switch = state.ros.home_switch()
        for name, st in axes.items():
            if name in homed:
                st["is_homed"] = homed[name]
            if name in switch:
                st["home_switch"] = switch[name]
    return axes


# ---------------- REST endpoints ----------------

@app.get("/api/state")
def get_state():
    return {
        "axes": _axes_state(),
        "gripper": _gripper().state_dict(),
        "teach": _teach_summary(),
        "backend": state.backend,
        "ros_mode": _ros_mode(),
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
        "backend": state.backend,
        "axes": [
            {
                "can_id": ax.can_id, "name": ax.name,
                "gear_ratio": ax.gear_ratio, "pulses_per_rev": ax.pulses_per_rev,
                "home_offset_deg": ax.home_offset_deg,
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
    # In ROS mode the bridge owns the motors; route enable/disable to it.
    if _ros_mode():
        try:
            _ros().enable(req.on)
        except Exception as e:
            raise HTTPException(status_code=503, detail=f"ROS enable failed: {e}")
        return {"ok": True, "via": "ros"}
    _motion().enable_all(req.on)
    if state.cfg.gripper.enabled:
        _gripper().set_enabled(req.on)
    return {"ok": True}


@app.post("/api/estop")
def estop():
    if _ros_mode():
        try:
            _ros().estop()
        except Exception as e:
            raise HTTPException(status_code=503, detail=f"ROS estop failed: {e}")
        return {"ok": True, "via": "ros"}
    _motion().emergency_stop()
    return {"ok": True}


@app.post("/api/jog/start")
def jog_start(req: JogStartReq):
    # Same jog surface in both modes. In ROS mode it publishes a JointJog to the
    # bridge (signed velocity = direction * speed_pct); the bridge has a deadman
    # stop, so the UI must keep republishing while the button is held.
    if _ros_mode():
        vel = req.speed_pct if req.direction == 1 else -req.speed_pct
        _ros().jog(_axis_name(req.can_id), vel)
        return {"ok": True, "via": "ros"}
    _motion().jog_start(req.can_id, req.direction, req.speed_pct)
    return {"ok": True}


@app.post("/api/jog/stop")
def jog_stop(req: JogStopReq):
    if _ros_mode():
        _ros().jog_stop([_axis_name(req.can_id)])
        return {"ok": True, "via": "ros"}
    _motion().jog_stop(req.can_id)
    return {"ok": True}


@app.post("/api/jog/stop_all")
def jog_stop_all():
    if _ros_mode():
        _ros().jog_stop([ax.name for ax in state.cfg.axes])
        return {"ok": True, "via": "ros"}
    _motion().jog_stop_all()
    return {"ok": True}


@app.post("/api/move")
def move(req: MoveReq):
    """Absolute move of one joint to a target angle. Direct mode: an MKS relative
    move via the local Motion. ROS mode: a single-joint FollowJointTrajectory goal
    to the bridge (the local bus is dry-run there), so the 'Go to angle' control
    drives the real robot in both deployments. The differential wrist holds the
    partner joint at its current angle, so moving J5/J6 alone is still coordinated.
    """
    if _ros_mode():
        name = _axis_name(req.can_id)
        _require_ros_homed([name])
        try:
            return {"ok": True, "via": "ros",
                    **_ros().move_to_joints({name: req.degrees}, req.duration_s)}
        except HTTPException:
            raise
        except Exception as e:
            raise HTTPException(status_code=503, detail=f"ROS move failed: {e}")
    try:
        _motion().move_to_degrees(req.can_id, req.degrees, req.speed_pct)
    except NotHomedError as e:
        raise HTTPException(status_code=409, detail=str(e))
    except LimitViolation as e:
        raise HTTPException(status_code=400, detail=str(e))
    return {"ok": True}


@app.post("/api/home")
def home(req: HomeReq):
    # In ROS mode the bridge owns the motors; route per-axis homing to it.
    if _ros_mode():
        try:
            return {"ok": True, "via": "ros", **_ros().home_axis(_axis_name(req.can_id))}
        except HTTPException:
            raise
        except Exception as e:
            raise HTTPException(status_code=503, detail=f"ROS home failed: {e}")
    try:
        _motion().home_axis(req.can_id)
    except HomingError as e:
        raise HTTPException(status_code=409, detail=str(e))
    return {"ok": True}


@app.post("/api/home/all")
def home_all():
    if _ros_mode():
        try:
            return {"ok": True, "via": "ros", **_ros().home_all()}
        except HTTPException:
            raise
        except Exception as e:
            raise HTTPException(status_code=503, detail=f"ROS home-all failed: {e}")
    try:
        _motion().home_all()
    except HomingError as e:
        raise HTTPException(status_code=409, detail=str(e))
    return {"ok": True}


@app.post("/api/home/dir")
def home_dir(req: HomeDirReq):
    """Live seek-direction override for one axis (validation aid). ccw=False is
    CW (home_dir 0), True is CCW (home_dir 1)."""
    if _ros_mode():
        try:
            result = _ros().set_home_dir(_axis_name(req.can_id), req.ccw)
        except HTTPException:
            raise
        except Exception as e:
            raise HTTPException(status_code=503, detail=f"ROS set_dir failed: {e}")
        # Mirror locally so the UI's direction indicator reflects the change.
        state.cfg.axis_by_id(req.can_id).home_dir = 1 if req.ccw else 0
        return {"ok": True, "via": "ros", **result}
    state.cfg.axis_by_id(req.can_id).home_dir = 1 if req.ccw else 0
    return {"ok": True, "home_dir": 1 if req.ccw else 0}


@app.post("/api/io")
def read_io(req: IoReq):
    """Read the home-switch state for one axis. Bench aid for verifying the
    sensor's reverse-logic level. In ROS mode the bridge owns the bus, so this
    returns the bridge-published switch state (reverse-logic already applied)."""
    if _ros_mode():
        name = _axis_name(req.can_id)
        sw = _ros().home_switch().get(name)
        if sw is None:
            raise HTTPException(status_code=504, detail="no home-switch reading from bridge yet")
        return {"ok": True, "via": "ros", "io": {"home_switch": sw}}
    io = _motion().read_io(req.can_id)
    if io is None:
        raise HTTPException(status_code=504, detail="no IO reply from driver")
    return {"ok": True, "io": io}


@app.post("/api/microsteps")
def microsteps(req: MicrostepsReq):
    if _ros_mode():
        try:
            return {"ok": True, "via": "ros",
                    **_ros().set_microsteps(_axis_name(req.can_id), req.microsteps)}
        except HTTPException:
            raise
        except Exception as e:
            raise HTTPException(status_code=503, detail=f"ROS microsteps failed: {e}")
    try:
        _motion().set_microsteps(req.can_id, req.microsteps)
    except PermissionError as e:
        raise HTTPException(status_code=403, detail=str(e))
    return {"ok": True}


@app.post("/api/current")
def current(req: CurrentReq):
    if _ros_mode():
        try:
            return {"ok": True, "via": "ros",
                    **_ros().set_current(_axis_name(req.can_id), req.milliamps)}
        except HTTPException:
            raise
        except Exception as e:
            raise HTTPException(status_code=503, detail=f"ROS current failed: {e}")
    try:
        _motion().set_current(req.can_id, req.milliamps)
    except PermissionError as e:
        raise HTTPException(status_code=403, detail=str(e))
    return {"ok": True}


@app.post("/api/work_mode")
def work_mode(req: WorkModeReq):
    # Flash-persisting write (CMD 0x82). The UI confirms before calling this;
    # jog needs SR_* (mode 5 = SR_vFOC), CR_* silently drops motion. In ROS mode
    # the bridge owns the bus, so route the write to it (the local Motion is
    # dry-run there and the write would never reach the driver).
    if _ros_mode():
        try:
            return {"ok": True, "via": "ros",
                    **_ros().set_work_mode(_axis_name(req.can_id), req.mode)}
        except HTTPException:
            raise
        except Exception as e:
            raise HTTPException(status_code=503, detail=f"ROS work_mode failed: {e}")
    try:
        _motion().set_work_mode(req.can_id, req.mode)
    except PermissionError as e:
        raise HTTPException(status_code=403, detail=str(e))
    return {"ok": True}


@app.post("/api/gear_ratio")
def gear_ratio(req: GearRatioReq):
    """Live-adjust an axis gear ratio (the degrees<->pulses scale). SOFTWARE-ONLY
    — no driver/flash write — so MoveIt and jog motion can be matched to the real
    robot on the bench, then the tuned value copied into config.yaml. In ROS mode
    the bridge owns the conversion, so route it there (and mirror locally so
    /api/config reflects the change)."""
    if _ros_mode():
        try:
            result = _ros().set_gear_ratio(_axis_name(req.can_id), req.gear_ratio)
        except HTTPException:
            raise
        except Exception as e:
            raise HTTPException(status_code=503, detail=f"ROS gear_ratio failed: {e}")
        # Mirror locally so the UI's /api/config shows the live value.
        state.cfg.axis_by_id(req.can_id).gear_ratio = req.gear_ratio
        return {"ok": True, "via": "ros", **result}
    try:
        _motion().set_gear_ratio(req.can_id, req.gear_ratio)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    return {"ok": True, "gear_ratio": req.gear_ratio}


@app.post("/api/joint_zero")
def joint_zero(req: JointZeroReq):
    """Calibrate a joint's zero reference: declare "this joint is physically at
    angle_deg right now". Sets home_offset_deg so the UI shows (and 'Go to angle'
    accepts) true JOINT angles instead of travel-from-the-home-switch. SOFTWARE-
    ONLY like /api/gear_ratio — copy the returned home_offset_deg into config.yaml
    to persist it. In ROS mode the bridge owns the conversion: compute the offset
    from its published /joint_states angle and push it as a bridge param (mirrored
    locally so /api/config stays truthful)."""
    ax = state.cfg.axis_by_id(req.can_id)
    if _ros_mode():
        js = state.ros.joint_states()
        deg = dict(zip(js["name"], js["position_deg"])) if js else {}
        if ax.name not in deg:
            raise HTTPException(status_code=503,
                                detail="no /joint_states from the bridge yet — is arctos_bridge running?")
        new_offset = ax.home_offset_deg + (req.angle_deg - deg[ax.name])
        try:
            result = _ros().set_home_offset(ax.name, new_offset)
        except HTTPException:
            raise
        except Exception as e:
            raise HTTPException(status_code=503, detail=f"ROS joint_zero failed: {e}")
        ax.home_offset_deg = new_offset
        return {"ok": True, "via": "ros", "home_offset_deg": round(new_offset, 3), **result}
    new_offset = _motion().calibrate_joint_zero(req.can_id, req.angle_deg)
    return {"ok": True, "home_offset_deg": round(new_offset, 3)}


@app.post("/api/refresh")
def refresh():
    _motion().request_all_positions()
    return {"ok": True}


# ---------------- ROS2 / MoveIt (optional) ----------------
#
# These wrap the in-process ROS2 client. They are additive: the standard CAN
# control endpoints above keep working whether or not ROS is enabled. The
# arctos_bridge node owns the CAN bus on a ROS deployment (single-owner rule),
# so on the Pi these become the primary motion path and direct /api/move etc.
# would be configured against a dry_run/disabled local bus.

@app.get("/api/ros/status")
def ros_status():
    if state.ros is None:
        return {
            "available": False,
            "enabled": False,
            "rclpy": ros_available(),
            "detail": import_error() if not ros_available() else "set ARCTOS_ROS=1 to enable",
        }
    try:
        return {"available": True, "enabled": True, **state.ros.status()}
    except Exception as e:  # pragma: no cover - needs a live ROS graph
        raise HTTPException(status_code=503, detail=str(e))


@app.post("/api/ros/estop")
def ros_estop():
    try:
        return _ros().estop()
    except HTTPException:
        raise
    except Exception as e:  # pragma: no cover - needs a live ROS graph
        raise HTTPException(status_code=503, detail=str(e))


@app.post("/api/ros/enable")
def ros_enable(req: EnableReq):
    try:
        return _ros().enable(req.on)
    except HTTPException:
        raise
    except Exception as e:  # pragma: no cover - needs a live ROS graph
        raise HTTPException(status_code=503, detail=str(e))


@app.post("/api/ros/move")
def ros_move(req: RosMoveReq):
    """Joint-space move via the FollowJointTrajectory action (no collision
    planning). Collision-aware MoveIt planning (MoveItPy) is a future endpoint.
    """
    client = _ros()  # 503 first if ROS is unavailable
    _require_ros_homed(list(req.joints_deg.keys()))
    try:
        return client.move_to_joints(req.joints_deg, req.duration_s)
    except HTTPException:
        raise
    except Exception as e:  # pragma: no cover - needs a live ROS graph
        raise HTTPException(status_code=503, detail=str(e))


@app.post("/api/ros/run_program")
def ros_run_program(req: RosRunReq):
    """Replay the currently loaded/captured teach waypoints as a single timed
    JointTrajectory through ROS2. Load a saved program first via /api/teach/load.
    """
    client = _ros()
    joint_names = [ax.name for ax in state.cfg.axes]
    waypoints = [{"joints": wp.joints, "dwell_ms": wp.dwell_ms} for wp in _teach().waypoints]
    if not waypoints:
        raise HTTPException(status_code=400, detail="no waypoints loaded")
    _require_ros_homed(joint_names)
    try:
        return client.run_waypoints(joint_names, waypoints, seg_time_s=req.seg_time_s)
    except HTTPException:
        raise
    except Exception as e:  # pragma: no cover - needs a live ROS graph
        raise HTTPException(status_code=503, detail=str(e))


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
        await ws.send_json({"type": "hello", "backend": state.backend})
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
                "axes": _axes_state(),
                "ros_mode": _ros_mode(),
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
            # Live home-switch state for the validation UI. Only in direct mode —
            # in ROS mode the local bus is dry_run and the bridge publishes it.
            if not _ros_mode():
                _motion().request_all_io()
        except Exception:
            log.exception("watchdog iteration failed")


# ---------------- Frontend ----------------

@app.get("/")
def root():
    index = FRONTEND_DIR / "index.html"
    if not index.exists():
        raise HTTPException(404, "frontend not built")
    # Cache-bust app.js by its mtime so the touchscreen browser always loads the
    # current UI (it otherwise caches /static/app.js across deploys), and tell it
    # not to cache the HTML shell itself.
    html = index.read_text()
    appjs = FRONTEND_DIR / "app.js"
    if appjs.exists():
        html = html.replace('src="/static/app.js"', f'src="/static/app.js?v={int(appjs.stat().st_mtime)}"')
    return HTMLResponse(html, headers={"Cache-Control": "no-store"})


app.mount("/static", StaticFiles(directory=str(FRONTEND_DIR)), name="static")
