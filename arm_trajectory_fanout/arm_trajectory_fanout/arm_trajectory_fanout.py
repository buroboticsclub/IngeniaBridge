#!/usr/bin/env python3
from __future__ import annotations

import threading
from typing import Dict, List, Tuple

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ArmTrajectoryFanout(Node):
    def __init__(self) -> None:
        super().__init__("arm_trajectory_fanout")

        self.declare_parameter("controller_action", "/arm_controller/follow_joint_trajectory")
        self.declare_parameter("joint_mappings", ["joint0:/joint0_position_controller/follow_joint_trajectory"])
        self.declare_parameter("allow_unmapped_joints", True)
        self.declare_parameter("client_timeout_sec", 5.0)
        self.declare_parameter("result_timeout_padding_sec", 3.0)

        self.controller_action = str(self.get_parameter("controller_action").value)
        self.allow_unmapped_joints = bool(self.get_parameter("allow_unmapped_joints").value)
        self.client_timeout_sec = float(self.get_parameter("client_timeout_sec").value)
        self.result_timeout_padding_sec = float(self.get_parameter("result_timeout_padding_sec").value)

        self.joint_to_action = self._parse_joint_mappings(
            list(self.get_parameter("joint_mappings").value)
        )
        if not self.joint_to_action:
            raise RuntimeError("arm_trajectory_fanout needs at least one joint_mappings entry")

        self.cb_group = ReentrantCallbackGroup()
        self.child_clients: Dict[str, ActionClient] = {
            joint: ActionClient(
                self,
                FollowJointTrajectory,
                action_name,
                callback_group=self.cb_group,
            )
            for joint, action_name in self.joint_to_action.items()
        }

        self._active_child_goals = []
        self._active_lock = threading.Lock()

        self.server = ActionServer(
            self,
            FollowJointTrajectory,
            self.controller_action,
            execute_callback=self._execute,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self.cb_group,
        )

        self.get_logger().info(f"Fanout action server ready: {self.controller_action}")
        self.get_logger().info(f"Joint mappings: {self.joint_to_action}")
        self.get_logger().warning(
            "allow_unmapped_joints=true: unmapped joints in full-arm trajectories will be ignored"
            if self.allow_unmapped_joints
            else "allow_unmapped_joints=false: every commanded joint must be mapped"
        )

    def _parse_joint_mappings(self, entries: List[str]) -> Dict[str, str]:
        mapping: Dict[str, str] = {}
        for raw in entries:
            item = str(raw).strip()
            if not item:
                continue
            if ":" not in item:
                raise ValueError(f"Bad joint_mappings entry '{item}', expected '<joint>:<action>'")
            joint, action = item.split(":", 1)
            joint = joint.strip()
            action = action.strip()
            if not joint or not action.startswith("/"):
                raise ValueError(f"Bad joint_mappings entry '{item}'")
            mapping[joint] = action
        return mapping

    def _goal_cb(self, goal_request) -> GoalResponse:
        names = list(goal_request.trajectory.joint_names)
        if not names or not goal_request.trajectory.points:
            self.get_logger().warning("Rejecting empty trajectory goal")
            return GoalResponse.REJECT

        mapped = [j for j in names if j in self.joint_to_action]
        unmapped = [j for j in names if j not in self.joint_to_action]

        if not mapped:
            self.get_logger().warning(f"Rejecting trajectory: no mapped joints in {names}")
            return GoalResponse.REJECT

        if unmapped and not self.allow_unmapped_joints:
            self.get_logger().warning(f"Rejecting trajectory with unmapped joints: {unmapped}")
            return GoalResponse.REJECT

        if unmapped:
            self.get_logger().warning(
                f"Accepting trajectory but ignoring unmapped joints: {unmapped}; mapped={mapped}"
            )
        else:
            self.get_logger().info(f"Accepting trajectory for mapped joints: {mapped}")

        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle) -> CancelResponse:
        self.get_logger().warning("Cancel requested; forwarding to active child goals")
        with self._active_lock:
            child_goals = list(self._active_child_goals)
        for child in child_goals:
            try:
                child.cancel_goal_async()
            except Exception:
                pass
        return CancelResponse.ACCEPT

    def _wait_future(self, future, timeout_sec: float, label: str):
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout_sec):
            raise TimeoutError(f"{label} timed out after {timeout_sec:.2f}s")
        return future.result()

    def _trajectory_duration_sec(self, traj: JointTrajectory) -> float:
        if not traj.points:
            return 0.0
        last = traj.points[-1].time_from_start
        return float(last.sec) + float(last.nanosec) * 1e-9

    def _make_single_joint_goal(self, parent_goal, joint_name: str, joint_index: int):
        src = parent_goal.trajectory

        dst_goal = FollowJointTrajectory.Goal()
        dst_goal.trajectory = JointTrajectory()
        dst_goal.trajectory.header = src.header
        dst_goal.trajectory.joint_names = [joint_name]

        for point in src.points:
            dst_point = JointTrajectoryPoint()
            if point.positions and len(point.positions) > joint_index:
                dst_point.positions = [float(point.positions[joint_index])]
            if point.velocities and len(point.velocities) > joint_index:
                dst_point.velocities = [float(point.velocities[joint_index])]
            if point.accelerations and len(point.accelerations) > joint_index:
                dst_point.accelerations = [float(point.accelerations[joint_index])]
            if point.effort and len(point.effort) > joint_index:
                dst_point.effort = [float(point.effort[joint_index])]
            dst_point.time_from_start = point.time_from_start
            dst_goal.trajectory.points.append(dst_point)

        return dst_goal

    def _execute(self, goal_handle):
        parent_goal = goal_handle.request
        names = list(parent_goal.trajectory.joint_names)

        mapped: List[Tuple[str, int]] = [
            (joint, names.index(joint))
            for joint in names
            if joint in self.joint_to_action
        ]
        unmapped = [joint for joint in names if joint not in self.joint_to_action]

        result = FollowJointTrajectory.Result()

        if unmapped and not self.allow_unmapped_joints:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            result.error_string = f"Unmapped joints: {unmapped}"
            return result

        if not mapped:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            result.error_string = f"No mapped joints in trajectory names={names}"
            return result

        self.get_logger().info(
            f"Executing fanout: parent_joints={names}, mapped={[j for j, _ in mapped]}, ignored={unmapped}"
        )

        child_goal_handles = []
        try:
            for joint, idx in mapped:
                client = self.child_clients[joint]
                action_name = self.joint_to_action[joint]

                if not client.wait_for_server(timeout_sec=self.client_timeout_sec):
                    raise TimeoutError(f"Child action unavailable for {joint}: {action_name}")

                child_goal = self._make_single_joint_goal(parent_goal, joint, idx)
                child_handle = self._wait_future(
                    client.send_goal_async(child_goal),
                    self.client_timeout_sec,
                    f"send_goal {joint}",
                )

                if not child_handle.accepted:
                    raise RuntimeError(f"Child goal rejected for {joint}: {action_name}")

                self.get_logger().info(f"Child goal accepted for {joint}")
                child_goal_handles.append((joint, child_handle))

            with self._active_lock:
                self._active_child_goals = [handle for _, handle in child_goal_handles]

            wait_sec = (
                self._trajectory_duration_sec(parent_goal.trajectory)
                + self.result_timeout_padding_sec
                + self.client_timeout_sec
            )

            child_results = []
            for joint, child_handle in child_goal_handles:
                wrapped = self._wait_future(
                    child_handle.get_result_async(),
                    wait_sec,
                    f"get_result {joint}",
                )
                child_results.append((joint, wrapped))
                self.get_logger().info(
                    f"Child result {joint}: status={getattr(wrapped, 'status', None)} "
                    f"error_code={getattr(wrapped.result, 'error_code', None)}"
                )

            with self._active_lock:
                self._active_child_goals = []

            bad = [
                (joint, wrapped.result.error_code, wrapped.result.error_string)
                for joint, wrapped in child_results
                if wrapped.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL
            ]

            if bad:
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                result.error_string = f"Child trajectory failures: {bad}"
                return result

            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            result.error_string = (
                f"Fanout succeeded for mapped joints {[j for j, _ in mapped]}; "
                f"ignored unmapped joints {unmapped}"
            )
            return result

        except Exception as exc:
            with self._active_lock:
                self._active_child_goals = []
            self.get_logger().error(f"Fanout execution failed: {exc}")
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            result.error_string = str(exc)
            return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmTrajectoryFanout()
    executor = MultiThreadedExecutor(num_threads=4)
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
