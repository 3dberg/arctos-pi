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

    from builtin_interfaces.msg import Duration
    from control_msgs.action import FollowJointTrajectory
    from sensor_msgs.msg import JointState
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
    ) -> None:
        if not ros_available():
            raise RosUnavailable(f"rclpy not importable: {import_error()}")

        self._action_name = f"/{controller_name}/follow_joint_trajectory"
        rclpy.init()
        self._node: Node = rclpy.create_node("arctos_fastapi_client")

        self._latest: Optional[dict] = None
        self._latest_lock = threading.Lock()

        self._node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self._estop_cli = self._node.create_client(Trigger, estop_service)
        self._enable_cli = self._node.create_client(SetBool, enable_service)
        self._traj_cli = ActionClient(self._node, FollowJointTrajectory, self._action_name)

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
