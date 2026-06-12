"""arctos_bridge — reuse the tested Python Motion/CAN/MKS stack from ROS2.

Why a node instead of a ros2_control hardware interface: ros2_control hardware
plugins are loaded via pluginlib (C++ shared libraries), so the existing,
tested Python MKS/CAN code (backend.mks / backend.can_bus / backend.motion)
cannot be a hardware plugin directly. Rather than re-implement and fork that
logic in C++, this node wraps it and exposes the exact ROS2 surface MoveIt2
(and, later, an AI agent) needs:

  * publishes  sensor_msgs/JointState  on /joint_states   (positions in rad)
  * serves     control_msgs/action/FollowJointTrajectory
                 on <controller_name>/follow_joint_trajectory
  * serves     std_srvs/Trigger        on ~/estop
  * serves     std_srvs/SetBool        on ~/enable

MoveIt's moveit_controllers.yaml points at the same action name, so the MoveIt
config is identical whether execution goes through this bridge (real Python
stack: MockBus on a dev box, slcan/socketcan on the Pi) or through a
ros2_control joint_trajectory_controller backed by mock_components (pure sim).

This node is the SINGLE owner of the CAN bus. Do not run another process that
also opens the bus on real hardware.
"""
from __future__ import annotations

import math
import threading
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointJog
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from std_srvs.srv import SetBool, Trigger

from backend.can_bus import open_bus
from backend.config import AppConfig
from backend.motion import HomingError, LimitViolation, Motion


def _duration_to_sec(d: Duration) -> float:
    return d.sec + d.nanosec * 1e-9


class ArctosBridge(Node):
    def __init__(self) -> None:
        super().__init__("arctos_bridge")

        self.declare_parameter("config_path", "")
        self.declare_parameter("can_backend", "")  # override config.can.backend if set
        self.declare_parameter("controller_name", "arctos_arm_controller")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("default_speed_pct", 0.4)

        config_path = self.get_parameter("config_path").value
        backend_override = self.get_parameter("can_backend").value
        self._controller = self.get_parameter("controller_name").value
        self._default_speed = float(self.get_parameter("default_speed_pct").value)

        self._cfg = AppConfig.load(Path(config_path)) if config_path else AppConfig.default_six_axis()
        if backend_override:
            self._cfg.can.backend = backend_override

        self.get_logger().info(
            f"opening CAN backend={self._cfg.can.backend} channel={self._cfg.can.channel}"
        )
        self._bus = open_bus(self._cfg.can.backend, self._cfg.can.channel, self._cfg.can.bitrate)
        self._motion = Motion(self._cfg, self._bus)

        # name <-> can_id maps
        self._joint_names = [ax.name for ax in self._cfg.axes]
        self._can_by_name = {ax.name: ax.can_id for ax in self._cfg.axes}

        # The bridge owns the real bus, so it is the legitimate writer of driver
        # flash (work mode / current / microsteps). The FastAPI-side dry-run
        # Motion keeps allow_flash_writes=False so stray local writes still error.
        self._motion.allow_flash_writes = True
        self._apply_startup_driver_config()
        self._setup_driver_params()

        self._motion.enable_all(True)

        # ---- manual jog state ----
        # Hold-to-run: the UI republishes JointJog at ~10 Hz while a button is
        # held and sends zero velocity (or stops sending) on release. The
        # deadman timer stops all jogs if no fresh message arrives within
        # _jog_timeout, so a dropped client / closed tab can't leave an axis
        # running. Jog and trajectory execution are mutually exclusive.
        self._jog_lock = threading.Lock()
        self._jogging: set[int] = set()
        self._last_jog = 0.0
        self._jog_timeout = 0.35
        self._traj_active = False

        cb = ReentrantCallbackGroup()
        # The periodic CAN pollers (joint_states, home-switch, homed) and the
        # jog deadman share one mutually-exclusive group so they run one-at-a-
        # time instead of stacking executor threads that all hit the bus at
        # once. The bus layer serializes+paces transmits regardless; this just
        # keeps the bridge from offering bursts. Services/action stay on the
        # reentrant group so a long trajectory never blocks the pollers.
        poll_cb = MutuallyExclusiveCallbackGroup()

        self._js_pub = self.create_publisher(JointState, "/joint_states", 10)
        rate = float(self.get_parameter("publish_rate_hz").value)
        self._timer = self.create_timer(1.0 / rate, self._publish_joint_states, callback_group=poll_cb)

        self._action = ActionServer(
            self,
            FollowJointTrajectory,
            f"{self._controller}/follow_joint_trajectory",
            execute_callback=self._execute_trajectory,
            goal_callback=self._on_goal,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=cb,
        )

        self.create_service(Trigger, "~/estop", self._on_estop, callback_group=cb)
        self.create_service(SetBool, "~/enable", self._on_enable, callback_group=cb)

        # Homing: one Trigger per joint (~/home_<name>) plus ~/home_all, and a
        # per-joint SetBool (~/set_dir_<name>) live seek-direction override.
        # Per-joint std_srvs keeps the bridge dependency-free (no custom srv).
        self.create_service(Trigger, "~/home_all", self._on_home_all, callback_group=cb)
        for name in self._joint_names:
            self.create_service(
                Trigger, f"~/home_{name}",
                lambda req, resp, n=name: self._on_home_axis(n, resp),
                callback_group=cb,
            )
            self.create_service(
                SetBool, f"~/set_dir_{name}",
                lambda req, resp, n=name: self._on_set_dir(n, req, resp),
                callback_group=cb,
            )

        # Latched bitmask topics (bit i = joint i): homed state, and live
        # home-switch state (IN_1 with the axis's reverse-logic applied). The
        # FastAPI client gets the current value immediately on (re)connect.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._homed_pub = self.create_publisher(Int32, "~/homed_status", latched)
        self._last_homed_mask = -1
        self.create_timer(0.2, self._publish_homed, callback_group=poll_cb)
        self._switch_pub = self.create_publisher(Int32, "~/home_switch_status", latched)
        self._last_switch_mask = -1
        # Poll the home sensors so the validation UI shows live IN_1 state.
        self.create_timer(0.25, self._poll_and_publish_switches, callback_group=poll_cb)

        # Manual jog from the FastAPI UI (control_msgs/JointJog on ~/joint_jog).
        self.create_subscription(JointJog, "~/joint_jog", self._on_joint_jog, 10, callback_group=cb)
        self.create_timer(0.1, self._jog_deadman, callback_group=poll_cb)

        self.get_logger().info(
            f"arctos_bridge ready: joints={self._joint_names} "
            f"action={self._controller}/follow_joint_trajectory, jog=~/joint_jog, "
            f"home=~/home_<joint>|~/home_all"
        )

    # ---- driver config (flash-persisting: work mode / current / microsteps) ----

    def _apply_startup_driver_config(self) -> None:
        """Push per-axis default_work_mode / hold current to the drivers once at
        startup so the hardware is deterministic. Drivers boot in unknown
        factory modes; a wrong/open-loop mode causes idle hum + heat (J3/J4).
        Flash writes — only for axes that opt in (value is not None)."""
        for ax in self._cfg.axes:
            try:
                if ax.default_work_mode is not None:
                    self._motion.set_work_mode(ax.can_id, ax.default_work_mode)
                    self.get_logger().info(f"{ax.name}: set work_mode={ax.default_work_mode}")
                if ax.hold_current_ma is not None:
                    self._motion.set_current(ax.can_id, ax.hold_current_ma)
                    self.get_logger().info(f"{ax.name}: set current={ax.hold_current_ma}mA")
            except Exception as e:
                self.get_logger().warn(f"{ax.name}: startup driver config failed: {e}")

    def _setup_driver_params(self) -> None:
        """Expose per-axis driver settings as ROS params so they can be changed
        live: `ros2 param set /arctos_bridge axis.J3.work_mode 5` — also how the
        FastAPI UI routes /api/work_mode|current|microsteps in ROS mode. Declare
        BEFORE registering the callback so these declarations don't trigger
        flash writes; only later sets do."""
        self._param_targets: dict[str, tuple[int, str]] = {}
        for ax in self._cfg.axes:
            specs = {
                # -1 = "unset at boot" sentinel for work_mode (param type is int)
                "work_mode": ax.default_work_mode if ax.default_work_mode is not None else -1,
                "current_ma": ax.default_current_ma,
                "microsteps": ax.default_microsteps,
            }
            for kind, default in specs.items():
                pname = f"axis.{ax.name}.{kind}"
                self.declare_parameter(pname, int(default))
                self._param_targets[pname] = (ax.can_id, kind)
            # Software-only DOUBLE params (no flash write): gear_ratio is the
            # deg<->pulse scale factor; home_offset_deg is the joint angle
            # assigned to the home-switch position (joint-zero calibration).
            # Both can be tuned live to match MoveIt/UI angles to the real robot.
            for kind, default in (("gear_ratio", ax.gear_ratio),
                                  ("home_offset_deg", ax.home_offset_deg)):
                pname = f"axis.{ax.name}.{kind}"
                self.declare_parameter(pname, float(default))
                self._param_targets[pname] = (ax.can_id, kind)
        self.add_on_set_parameters_callback(self._on_set_params)

    def _on_set_params(self, params) -> SetParametersResult:
        """Apply a driver-config param change to the real driver (flash write).
        Refused while busy (moving/homing). Unrelated params are accepted."""
        busy = self._busy_reason()
        for p in params:
            target = self._param_targets.get(p.name)
            if target is None:
                continue
            can_id, kind = target
            # gear_ratio / home_offset_deg are software-only conversion factors
            # (no driver/flash write). Still gated on busy so the conversion
            # can't change underneath a running trajectory/jog, which would
            # jump the commanded target.
            if kind in ("gear_ratio", "home_offset_deg"):
                if busy:
                    return SetParametersResult(successful=False, reason=f"cannot set {p.name}: {busy}")
                try:
                    if kind == "gear_ratio":
                        self._motion.set_gear_ratio(can_id, float(p.value))
                    else:
                        self._motion.set_home_offset(can_id, float(p.value))
                except Exception as e:
                    return SetParametersResult(successful=False, reason=f"{p.name}: {e}")
                continue
            value = int(p.value)
            if kind == "work_mode" and value < 0:
                continue  # sentinel: not configured, nothing to write
            if busy:
                return SetParametersResult(successful=False, reason=f"cannot set {p.name}: {busy}")
            try:
                if kind == "work_mode":
                    self._motion.set_work_mode(can_id, value)
                elif kind == "current_ma":
                    self._motion.set_current(can_id, value)
                elif kind == "microsteps":
                    self._motion.set_microsteps(can_id, value)
            except Exception as e:
                return SetParametersResult(successful=False, reason=f"{p.name}: {e}")
        return SetParametersResult(successful=True)

    # ---- manual jog ----

    def _homing_active(self) -> bool:
        if self._motion.homing_all_in_progress:
            return True
        return any(self._motion._state[c].homing_in_progress for c in self._can_by_name.values())

    def _busy_reason(self) -> str:
        """Why a new motion (home) can't start right now — one motion source at a time."""
        if self._traj_active:
            return "trajectory running"
        with self._jog_lock:
            if self._jogging:
                return "manual jog active"
        if self._homing_active():
            return "homing already in progress"
        return ""

    def _on_joint_jog(self, msg: JointJog) -> None:
        """Apply per-joint speed-mode jog. velocity sign = direction, magnitude
        = fraction of the axis max_speed (0..1). Ignored while a trajectory or a
        homing seek is running.
        """
        if self._traj_active or self._homing_active():
            return
        try:
            vels = list(msg.velocities)
            with self._jog_lock:
                self._last_jog = time.monotonic()
                for i, name in enumerate(msg.joint_names):
                    can_id = self._can_by_name.get(name)
                    if can_id is None:
                        continue
                    vel = vels[i] if i < len(vels) else 0.0
                    if abs(vel) < 1e-3:
                        self._motion.jog_stop(can_id)
                        self._jogging.discard(can_id)
                    else:
                        direction = 1 if vel > 0 else 0
                        self._motion.jog_start(can_id, direction, min(1.0, abs(vel)))
                        self._jogging.add(can_id)
        except Exception as e:  # never let a CAN hiccup kill the node
            self.get_logger().warn(f"jog skipped (CAN?): {e}", throttle_duration_sec=5.0)

    def _jog_deadman(self) -> None:
        try:
            with self._jog_lock:
                if self._jogging and (time.monotonic() - self._last_jog) > self._jog_timeout:
                    # Normal stop-on-release path (the UI relies on this), so log
                    # it quietly + throttled — every jog release would otherwise
                    # flood the console at WARN and look like a fault.
                    self.get_logger().info("jog deadman: jog stream ended; stopping all jogs",
                                           throttle_duration_sec=10.0)
                    self._motion.jog_stop_all()
                    self._jogging.clear()
        except Exception as e:
            self.get_logger().warn(f"jog deadman error (CAN?): {e}", throttle_duration_sec=5.0)

    def _on_goal(self, goal):
        # Refuse trajectories while a manual jog is active — one motion source
        # at a time. The UI's mode switch normally prevents this; this is the
        # safety backstop.
        with self._jog_lock:
            if self._jogging:
                self.get_logger().warn("rejecting trajectory goal: manual jog active")
                return GoalResponse.REJECT
        if self._homing_active():
            self.get_logger().warn("rejecting trajectory goal: homing in progress")
            return GoalResponse.REJECT
        # Block-until-homed gate: refuse goals touching an un-homed axis.
        unhomed = self._unhomed(list(goal.trajectory.joint_names))
        if unhomed:
            self.get_logger().warn(f"rejecting trajectory goal: not homed: {unhomed}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _unhomed(self, joint_names) -> list:
        """Names of involved home_enabled joints that aren't homed yet."""
        if not self._cfg.require_home_before_move:
            return []
        out = []
        for n in joint_names:
            can_id = self._can_by_name.get(n)
            if can_id is None:
                continue
            ax = self._cfg.axis_by_id(can_id)
            if ax.home_enabled and not self._motion._state[can_id].is_homed:
                out.append(n)
        return out

    # ---- state publishing ----

    def _publish_joint_states(self) -> None:
        # A CAN error here must NOT propagate: an unhandled exception in a ROS
        # timer callback is fatal to the node. Log (throttled) and skip the tick;
        # SocketCanBus.send auto-reopens can0 when it returns.
        try:
            # While any axis is homing, do NOT issue bus reads here: the homing
            # worker is already polling the bus, and a congested send would hold
            # the CAN tx lock and stall this timer — which is exactly what froze
            # the UI (joint_states stopped publishing). Publish from cache
            # instead; the homing axis still updates because its worker reads it.
            if not self._homing_active():
                self._motion.request_all_positions()  # MockBus auto-responds; hw via rx thread
            state = self._motion.state_dict()
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(self._joint_names)
            msg.position = [math.radians(state[name]["degrees"]) for name in self._joint_names]
            self._js_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"joint_states tick skipped (CAN?): {e}", throttle_duration_sec=5.0)

    def _publish_homed(self) -> None:
        mask = 0
        for i, name in enumerate(self._joint_names):
            if self._motion._state[self._can_by_name[name]].is_homed:
                mask |= (1 << i)
        if mask != self._last_homed_mask:
            self._last_homed_mask = mask
            self._homed_pub.publish(Int32(data=mask))

    def _poll_and_publish_switches(self) -> None:
        try:
            # Request fresh IO; replies cache in _state[].last_io (one cycle
            # latency). Skip the bus reads while homing for the same reason as
            # _publish_joint_states — publish the last cached mask instead.
            if not self._homing_active():
                self._motion.request_all_io()
            mask = 0
            for i, name in enumerate(self._joint_names):
                if self._motion.home_switch(self._can_by_name[name]):
                    mask |= (1 << i)
            if mask != self._last_switch_mask:
                self._last_switch_mask = mask
                self._switch_pub.publish(Int32(data=mask))
        except Exception as e:
            self.get_logger().warn(f"home-switch poll skipped (CAN?): {e}", throttle_duration_sec=5.0)

    # ---- trajectory execution ----

    def _execute_trajectory(self, goal_handle):
        # Block manual jog for the duration of the program.
        with self._jog_lock:
            self._motion.jog_stop_all()
            self._jogging.clear()
            self._traj_active = True
        try:
            return self._run_trajectory(goal_handle)
        finally:
            self._traj_active = False

    def _run_trajectory(self, goal_handle):
        traj = goal_handle.request.trajectory
        names = list(traj.joint_names)
        result = FollowJointTrajectory.Result()

        unknown = [n for n in names if n not in self._can_by_name]
        if unknown:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            result.error_string = f"unknown joints: {unknown}"
            return result

        start = time.monotonic()
        for point in traj.points:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.error_string = "canceled"
                return result

            # honor time_from_start so playback speed matches the plan
            target_t = _duration_to_sec(point.time_from_start)
            sleep_for = target_t - (time.monotonic() - start)
            if sleep_for > 0:
                time.sleep(sleep_for)

            degrees_per_axis = {
                self._can_by_name[n]: math.degrees(pos)
                for n, pos in zip(names, point.positions)
            }
            try:
                # move_all_degrees applies the J5/J6 differential coupling (if
                # configured) internally; we just pass per-joint degrees as usual.
                self._motion.move_all_degrees(degrees_per_axis, speed_pct=self._default_speed)
            except LimitViolation as exc:
                self._motion.jog_stop_all()
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                result.error_string = str(exc)
                return result
            except Exception as exc:  # CAN drop etc — abort cleanly, don't crash the node
                try:
                    self._motion.jog_stop_all()
                except Exception:
                    pass
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                result.error_string = f"execution error: {exc}"
                return result

            fb = FollowJointTrajectory.Feedback()
            fb.joint_names = names
            fb.desired = point
            goal_handle.publish_feedback(fb)

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    # ---- services ----

    def _on_estop(self, _req, resp):
        try:
            self._motion.emergency_stop()
            resp.success = True
            resp.message = "emergency stop sent to all axes"
        except Exception as e:
            resp.success = False
            resp.message = f"estop error (CAN?): {e}"
        return resp

    def _on_enable(self, req, resp):
        try:
            self._motion.enable_all(bool(req.data))
            resp.success = True
            resp.message = f"axes {'enabled' if req.data else 'disabled'}"
        except Exception as e:
            resp.success = False
            resp.message = f"enable error (CAN?): {e}"
        return resp

    # ---- homing (fire-and-forget; progress/result come back on ~/homed_status) ----

    def _on_home_axis(self, joint_name: str, resp):
        can_id = self._can_by_name.get(joint_name)
        if can_id is None:
            resp.success = False
            resp.message = f"unknown joint {joint_name}"
            return resp
        busy = self._busy_reason()
        if busy:
            resp.success = False
            resp.message = f"cannot home {joint_name}: {busy}"
            return resp
        try:
            self._motion.home_axis(can_id)  # spawns a background seek; returns at once
            resp.success = True
            resp.message = f"homing {joint_name}"
        except HomingError as exc:
            resp.success = False
            resp.message = str(exc)
        return resp

    def _on_home_all(self, _req, resp):
        busy = self._busy_reason()
        if busy:
            resp.success = False
            resp.message = f"cannot home all: {busy}"
            return resp
        try:
            self._motion.home_all()  # sequences home_enabled axes on its own thread
            resp.success = True
            resp.message = "homing all axes (sequencing in background)"
        except HomingError as exc:
            resp.success = False
            resp.message = str(exc)
        return resp

    def _on_set_dir(self, joint_name: str, req, resp):
        """Live seek-direction override (data: False=CW/0, True=CCW/1). Bring-up
        aid: validate/flip direction without editing config + restarting."""
        can_id = self._can_by_name.get(joint_name)
        if can_id is None:
            resp.success = False
            resp.message = f"unknown joint {joint_name}"
            return resp
        self._cfg.axis_by_id(can_id).home_dir = 1 if req.data else 0
        resp.success = True
        resp.message = f"{joint_name} home_dir={'CCW' if req.data else 'CW'}"
        return resp

    def destroy_node(self) -> bool:
        try:
            self._bus.shutdown()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArctosBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
