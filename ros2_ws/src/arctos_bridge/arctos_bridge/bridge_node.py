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
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger

from backend.can_bus import open_bus
from backend.config import AppConfig
from backend.motion import LimitViolation, Motion


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

        self._motion.enable_all(True)

        cb = ReentrantCallbackGroup()

        self._js_pub = self.create_publisher(JointState, "/joint_states", 10)
        rate = float(self.get_parameter("publish_rate_hz").value)
        self._timer = self.create_timer(1.0 / rate, self._publish_joint_states, callback_group=cb)

        self._action = ActionServer(
            self,
            FollowJointTrajectory,
            f"{self._controller}/follow_joint_trajectory",
            execute_callback=self._execute_trajectory,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=cb,
        )

        self.create_service(Trigger, "~/estop", self._on_estop, callback_group=cb)
        self.create_service(SetBool, "~/enable", self._on_enable, callback_group=cb)

        self.get_logger().info(
            f"arctos_bridge ready: joints={self._joint_names} "
            f"action={self._controller}/follow_joint_trajectory"
        )

    # ---- state publishing ----

    def _publish_joint_states(self) -> None:
        self._motion.request_all_positions()  # MockBus auto-responds; hw via rx thread
        state = self._motion.state_dict()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_names)
        msg.position = [math.radians(state[name]["degrees"]) for name in self._joint_names]
        self._js_pub.publish(msg)

    # ---- trajectory execution ----

    def _execute_trajectory(self, goal_handle):
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
                self._motion.move_all_degrees(degrees_per_axis, speed_pct=self._default_speed)
            except LimitViolation as exc:
                self._motion.jog_stop_all()
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                result.error_string = str(exc)
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
        self._motion.emergency_stop()
        resp.success = True
        resp.message = "emergency stop sent to all axes"
        return resp

    def _on_enable(self, req, resp):
        self._motion.enable_all(bool(req.data))
        resp.success = True
        resp.message = f"axes {'enabled' if req.data else 'disabled'}"
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
