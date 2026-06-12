"""Optional in-process ROS2 client so the FastAPI/touchscreen UI is a ROS2
remote control without a second front-end.

Design constraints:
  * rclpy is imported lazily. If it isn't installed (the standalone non-ROS
    deployment, and the test suite) `ros_available()` returns False and the
    API surfaces a clear "ROS not available" status instead of crashing.
  * The node runs on a background MultiThreadedExecutor thread; the FastAPI
    request handlers stay synchronous and block on rclpy futures with a
    timeout (no nested spinning, no event-loop coupling).

Scope (M5): mirror /joint_states, call the arctos_bridge estop/enable
services, and execute joint-space trajectories (move-to-joint-goal and
teach-program replay) via the FollowJointTrajectory action that the
arctos_bridge node / joint_trajectory_controller serves. Collision-aware
MoveIt planning (MoveItPy) is a documented next step, not wired here.

This client is the FastAPI side of the "single CAN owner" rule: it does NOT
open the CAN bus — the arctos_bridge node owns the hardware and this talks to
it over ROS2.
"""
from __future__ import annotations

import math
import threading
from typing import Optional

try:  # rclpy + message types only exist in a sourced ROS2 env
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile

    from builtin_interfaces.msg import Duration
    from control_msgs.action import FollowJointTrajectory
    from control_msgs.msg import JointJog
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParameters
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Int32
    from std_srvs.srv import SetBool, Trigger
    from trajectory_msgs.msg import JointTrajectoryPoint

    _IMPORT_ERROR: Optional[Exception] = None
except Exception as exc:  # pragma: no cover - exercised only outside a ROS env
    _IMPORT_ERROR = exc


def ros_available() -> bool:
    return _IMPORT_ERROR is None


def import_error() -> Optional[str]:
    return None if _IMPORT_ERROR is None else f"{type(_IMPORT_ERROR).__name__}: {_IMPORT_ERROR}"


class RosUnavailable(RuntimeError):
    pass


class RosClient:
    """Lifetime-managed ROS2 client. Construct once (app startup), close once."""

    def __init__(
        self,
        controller_name: str = "arctos_arm_controller",
        estop_service: str = "/arctos_bridge/estop",
        enable_service: str = "/arctos_bridge/enable",
        jog_topic: str = "/arctos_bridge/joint_jog",
        home_service_prefix: str = "/arctos_bridge/home_",
        home_all_service: str = "/arctos_bridge/home_all",
        homed_topic: str = "/arctos_bridge/homed_status",
        set_dir_prefix: str = "/arctos_bridge/set_dir_",
        home_switch_topic: str = "/arctos_bridge/home_switch_status",
        set_params_service: str = "/arctos_bridge/set_parameters",
    ) -> None:
        if not ros_available():
            raise RosUnavailable(f"rclpy not importable: {import_error()}")

        self._action_name = f"/{controller_name}/follow_joint_trajectory"
        self._home_prefix = home_service_prefix
        self._set_dir_prefix = set_dir_prefix
        rclpy.init()
        self._node: Node = rclpy.create_node("arctos_fastapi_client")

        self._latest: Optional[dict] = None
        self._latest_lock = threading.Lock()
        self._homed_mask = 0
        self._switch_mask = 0

        self._node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self._estop_cli = self._node.create_client(Trigger, estop_service)
        self._enable_cli = self._node.create_client(SetBool, enable_service)
        self._traj_cli = ActionClient(self._node, FollowJointTrajectory, self._action_name)
        self._jog_pub = self._node.create_publisher(JointJog, jog_topic, 10)

        # Per-joint home / set-dir services are created lazily by name; home-all up front.
        self._home_all_cli = self._node.create_client(Trigger, home_all_service)
        self._home_clis: dict = {}
        self._set_dir_clis: dict = {}
        # Driver config (work mode / current / microsteps) is routed through the
        # bridge node's standard parameter service so /api/work_mode etc. reach
        # the real drivers in ROS mode (the local dry-run Motion can't).
        self._set_params_cli = self._node.create_client(SetParameters, set_params_service)
        # Latched Int32 bitmasks (bit i = joint i), matching the /joint_states
        # name order from the same bridge node: homed state + live home-switch.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._node.create_subscription(Int32, homed_topic, self._on_homed, latched)
        self._node.create_subscription(Int32, home_switch_topic, self._on_switch, latched)

        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True, name="ros-client")
        self._thread.start()

    # ---- subscriptions ----

    def _on_js(self, msg) -> None:
        with self._latest_lock:
            self._latest = {
                "name": list(msg.name),
                "position_rad": list(msg.position),
                "position_deg": [round(math.degrees(p), 3) for p in msg.position],
            }

    def joint_states(self) -> Optional[dict]:
        with self._latest_lock:
            return dict(self._latest) if self._latest is not None else None

    def _on_homed(self, msg) -> None:
        with self._latest_lock:
            self._homed_mask = int(msg.data)

    def _on_switch(self, msg) -> None:
        with self._latest_lock:
            self._switch_mask = int(msg.data)

    def _mask_to_joints(self, mask: int) -> dict:
        with self._latest_lock:
            names = list(self._latest["name"]) if self._latest is not None else []
        return {n: bool(mask & (1 << i)) for i, n in enumerate(names)}

    def homed(self) -> dict:
        """Map the bridge's homed bitmask onto joint names using the latest
        /joint_states ordering (both come from the same bridge node). Returns
        {} until joint_states has arrived — callers treat that as un-homed."""
        with self._latest_lock:
            mask = self._homed_mask
        return self._mask_to_joints(mask)

    def home_switch(self) -> dict:
        """{joint_name: bool} live home-switch state (reverse-logic applied by the bridge)."""
        with self._latest_lock:
            mask = self._switch_mask
        return self._mask_to_joints(mask)

    # ---- helpers ----

    @staticmethod
    def _wait(future, timeout: float):
        """Block the calling (FastAPI) thread until the executor-driven future
        completes. The executor spins on its own thread, so we just wait."""
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            raise TimeoutError("ROS request timed out")
        return future.result()

    # ---- services ----

    def status(self) -> dict:
        return {
            "available": True,
            "action": self._action_name,
            "action_server_ready": self._traj_cli.server_is_ready(),
            "joint_states": self.joint_states(),
        }

    def estop(self, timeout: float = 2.0) -> dict:
        if not self._estop_cli.wait_for_service(timeout_sec=timeout):
            raise TimeoutError("estop service unavailable")
        resp = self._wait(self._estop_cli.call_async(Trigger.Request()), timeout)
        return {"success": resp.success, "message": resp.message}

    def enable(self, on: bool, timeout: float = 2.0) -> dict:
        if not self._enable_cli.wait_for_service(timeout_sec=timeout):
            raise TimeoutError("enable service unavailable")
        req = SetBool.Request()
        req.data = bool(on)
        resp = self._wait(self._enable_cli.call_async(req), timeout)
        return {"success": resp.success, "message": resp.message}

    # ---- homing (per-joint + all; the bridge runs the GoHome under the interlock) ----

    def home_axis(self, joint_name: str, wait_ready: float = 2.0,
                  timeout: float = 10.0) -> dict:
        cli = self._home_clis.get(joint_name)
        if cli is None:
            cli = self._node.create_client(Trigger, f"{self._home_prefix}{joint_name}")
            self._home_clis[joint_name] = cli
        if not cli.wait_for_service(timeout_sec=wait_ready):
            raise TimeoutError(f"home service for {joint_name} unavailable")
        resp = self._wait(cli.call_async(Trigger.Request()), timeout)
        return {"success": resp.success, "message": resp.message}

    def home_all(self, wait_ready: float = 2.0, timeout: float = 10.0) -> dict:
        if not self._home_all_cli.wait_for_service(timeout_sec=wait_ready):
            raise TimeoutError("home_all service unavailable")
        resp = self._wait(self._home_all_cli.call_async(Trigger.Request()), timeout)
        return {"success": resp.success, "message": resp.message}

    def set_home_dir(self, joint_name: str, ccw: bool,
                     wait_ready: float = 2.0, timeout: float = 5.0) -> dict:
        """Live seek-direction override for one joint (ccw=False -> CW/0, True -> CCW/1)."""
        cli = self._set_dir_clis.get(joint_name)
        if cli is None:
            cli = self._node.create_client(SetBool, f"{self._set_dir_prefix}{joint_name}")
            self._set_dir_clis[joint_name] = cli
        if not cli.wait_for_service(timeout_sec=wait_ready):
            raise TimeoutError(f"set_dir service for {joint_name} unavailable")
        req = SetBool.Request()
        req.data = bool(ccw)
        resp = self._wait(cli.call_async(req), timeout)
        return {"success": resp.success, "message": resp.message}

    # ---- driver config (work mode / current / microsteps via the bridge's param service) ----

    def _set_axis_param(self, joint_name: str, kind: str, value, as_float: bool = False,
                        wait_ready: float = 2.0, timeout: float = 5.0) -> dict:
        """Set one driver param on the bridge (axis.<joint>.<kind> = value). The
        bridge applies it (flash write for work_mode/current/microsteps; a plain
        software conversion factor for gear_ratio) and reports success via the
        SetParametersResult. `as_float` selects a DOUBLE param (gear_ratio) vs the
        default INTEGER param."""
        if not self._set_params_cli.wait_for_service(timeout_sec=wait_ready):
            raise TimeoutError("set_parameters service unavailable")
        req = SetParameters.Request()
        if as_float:
            pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
        else:
            pv = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=int(value))
        req.parameters = [Parameter(name=f"axis.{joint_name}.{kind}", value=pv)]
        resp = self._wait(self._set_params_cli.call_async(req), timeout)
        res = resp.results[0] if resp.results else None
        ok = bool(res.successful) if res is not None else False
        msg = (res.reason if res is not None else "") or f"{joint_name} {kind}={value}"
        return {"success": ok, "message": msg}

    def set_work_mode(self, joint_name: str, mode: int, **kw) -> dict:
        return self._set_axis_param(joint_name, "work_mode", mode, **kw)

    def set_current(self, joint_name: str, milliamps: int, **kw) -> dict:
        return self._set_axis_param(joint_name, "current_ma", milliamps, **kw)

    def set_microsteps(self, joint_name: str, microsteps: int, **kw) -> dict:
        return self._set_axis_param(joint_name, "microsteps", microsteps, **kw)

    def set_gear_ratio(self, joint_name: str, ratio: float, **kw) -> dict:
        return self._set_axis_param(joint_name, "gear_ratio", ratio, as_float=True, **kw)

    def set_home_offset(self, joint_name: str, offset_deg: float, **kw) -> dict:
        """Joint-zero calibration: the joint angle assigned to the home-switch
        position (software-only conversion shift, like gear_ratio)."""
        return self._set_axis_param(joint_name, "home_offset_deg", offset_deg, as_float=True, **kw)

    # ---- manual jog (fire-and-forget topic; bridge has a deadman stop) ----

    def jog(self, joint_name: str, velocity: float) -> None:
        """Publish a hold-to-run jog for one joint. velocity in [-1, 1]: sign is
        direction, magnitude is the fraction of the axis max_speed. The UI must
        republish at ~10 Hz while held; the bridge auto-stops on silence."""
        msg = JointJog()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.joint_names = [joint_name]
        msg.velocities = [float(velocity)]
        self._jog_pub.publish(msg)

    def jog_stop(self, joint_names: list[str]) -> None:
        """Immediately stop the given joints (zero velocity)."""
        msg = JointJog()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.joint_names = list(joint_names)
        msg.velocities = [0.0] * len(joint_names)
        self._jog_pub.publish(msg)

    # ---- trajectory execution ----

    def _send_trajectory(self, joint_names: list[str], points: list[dict], timeout: float) -> dict:
        """points: list of {positions_rad: [...], time_from_start_s: float}."""
        if not self._traj_cli.wait_for_server(timeout_sec=timeout):
            raise TimeoutError(f"action server {self._action_name} unavailable")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names
        for p in points:
            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in p["positions_rad"]]
            t = float(p["time_from_start_s"])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1.0) * 1e9))
            goal.trajectory.points.append(pt)

        send_future = self._traj_cli.send_goal_async(goal)
        goal_handle = self._wait(send_future, timeout)
        if not goal_handle.accepted:
            return {"accepted": False, "error_code": None, "error_string": "goal rejected"}

        # Execution can take a while; give it generous time beyond the last point.
        exec_timeout = timeout + (points[-1]["time_from_start_s"] if points else 0.0) + 5.0
        result = self._wait(goal_handle.get_result_async(), exec_timeout).result
        return {
            "accepted": True,
            "error_code": int(result.error_code),
            "error_string": result.error_string,
        }

    def move_to_joints(self, joints_deg: dict[str, float], duration_s: float = 3.0,
                       timeout: float = 5.0) -> dict:
        names = list(joints_deg.keys())
        points = [{
            "positions_rad": [math.radians(joints_deg[n]) for n in names],
            "time_from_start_s": max(0.1, duration_s),
        }]
        return self._send_trajectory(names, points, timeout)

    def run_waypoints(self, joint_names: list[str], waypoints: list[dict],
                      seg_time_s: float = 2.0, timeout: float = 5.0) -> dict:
        """waypoints: teach-program format [{joints: {J1: deg, ...}, dwell_ms}].
        Converts to a single timed JointTrajectory (degrees -> radians)."""
        points = []
        t = 0.0
        for wp in waypoints:
            joints = wp.get("joints", {})
            t += seg_time_s
            points.append({
                "positions_rad": [math.radians(joints.get(n, 0.0)) for n in joint_names],
                "time_from_start_s": t,
            })
            t += wp.get("dwell_ms", 0) / 1000.0
        if not points:
            return {"accepted": False, "error_string": "no waypoints"}
        return self._send_trajectory(joint_names, points, timeout)

    # ---- lifecycle ----

    def close(self) -> None:
        try:
            self._executor.shutdown()
        except Exception:
            pass
        try:
            self._node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
