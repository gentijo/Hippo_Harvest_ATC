import math
import os
import random
import uuid

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from hippo_harvest_sim.layout_data import (
    cell_to_pose,
    generate_tables,
    robot_home_cell,
    robot_home_yaw,
    robot_return_aisle_entry_cell,
    robot_return_stage_transit_cell,
    robot_return_transit_cell,
    robot_staging_cell,
    workspace_approach_cell,
)
from hippo_harvest_sim.telemetry import RunContextPublisher, Telemetry


class MultiRobotOrchestrator(Node):
    def __init__(self) -> None:
        super().__init__("multi_robot_orchestrator")

        # Orchestration parameters for the multi-robot Nav2 demo.
        self.declare_parameter("robot_count", 10)
        self.declare_parameter("task_waypoint_count", 2)
        self.declare_parameter("shuffle_seed", 42)
        self.declare_parameter("goal_timeout_sec", 45.0)
        self.declare_parameter("return_goal_timeout_sec", 75.0)
        self.declare_parameter("start_stagger_sec", 4.0)
        self.declare_parameter("diagnostic_period_sec", 5.0)

        self.robot_count = int(self.get_parameter("robot_count").value)
        self.task_waypoint_count = int(self.get_parameter("task_waypoint_count").value)
        shuffle_seed = int(self.get_parameter("shuffle_seed").value)
        self.goal_timeout_sec = float(self.get_parameter("goal_timeout_sec").value)
        self.return_goal_timeout_sec = float(self.get_parameter("return_goal_timeout_sec").value)
        self.start_stagger_sec = float(self.get_parameter("start_stagger_sec").value)
        self.diagnostic_period_sec = float(self.get_parameter("diagnostic_period_sec").value)
        self.run_id = os.environ.get("HIPPO_RUN_ID", "").strip() or uuid.uuid4().hex
        self.telemetry = Telemetry("hippo_harvest_sim", "multi_robot_orchestrator", self.get_logger())
        self.run_span = self.telemetry.start_span(
            "simulation.run",
            run_id=self.run_id,
            robot_count=self.robot_count,
            task_waypoint_count=self.task_waypoint_count,
        )

        # Shuffle workspace assignment deterministically so repeated runs are stable.
        self.random = random.Random(shuffle_seed)
        self.workspace_ids = [table["id"] for table in generate_tables()]
        self.robot_states = {}
        self.start_time_ns = None
        self.next_diagnostic_snapshot_sec = 0.0
        task_sequences = self._build_task_sequences()
        robot_traceparents = {
            f"robot{robot_index}": Telemetry.make_traceparent()
            for robot_index in range(1, self.robot_count + 1)
        }
        self.run_context_publisher = RunContextPublisher(self, self.run_id, robot_traceparents)
        atc_goal_qos = QoSProfile(depth=1)
        atc_goal_qos.reliability = ReliabilityPolicy.RELIABLE
        atc_goal_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        for robot_index in range(1, self.robot_count + 1):
            robot_name = f"robot{robot_index}"

            # Build the full route for this robot: staging, assigned workspaces,
            # return-lane waypoints, staging again, then home.
            home_x, home_y = cell_to_pose(robot_home_cell(robot_index, self.robot_count))
            home_yaw = robot_home_yaw(robot_index, self.robot_count)
            stage_x, stage_y = cell_to_pose(robot_staging_cell(robot_index, self.robot_count))
            workspace_ids = task_sequences[robot_index - 1]
            return_aisle_x, return_aisle_y = cell_to_pose(robot_return_aisle_entry_cell(workspace_ids[-1]))
            return_transit_x, return_transit_y = cell_to_pose(robot_return_transit_cell(workspace_ids[-1]))
            return_stage_transit_x, return_stage_transit_y = cell_to_pose(
                robot_return_stage_transit_cell(robot_index, self.robot_count)
            )
            goals = [{"label": f"{robot_name}_stage", "pose": self._pose_stamped(stage_x, stage_y, 0.0)}]
            goals.extend(
                {"label": workspace_id, "pose": self._goal_pose_for_workspace(workspace_id)} for workspace_id in workspace_ids
            )
            goals.append(
                {
                    "label": f"{robot_name}_return_aisle",
                    "pose": self._pose_stamped(return_aisle_x, return_aisle_y, -math.pi / 2.0),
                }
            )
            goals.append(
                {
                    "label": f"{robot_name}_return_transit",
                    "pose": self._pose_stamped(return_transit_x, return_transit_y, math.pi),
                }
            )
            goals.append(
                {
                    "label": f"{robot_name}_return_stage_transit",
                    "pose": self._pose_stamped(return_stage_transit_x, return_stage_transit_y, math.pi / 2.0),
                }
            )
            goals.append({"label": f"{robot_name}_return_stage", "pose": self._pose_stamped(stage_x, stage_y, 0.0)})
            goals.append({"label": robot_name, "pose": self._pose_stamped(home_x, home_y, home_yaw)})
            self.robot_states[robot_name] = {
                # Sends NavigateToPose action goals to each robot namespace.
                "action_client": ActionClient(self, NavigateToPose, f"/{robot_name}/navigate_to_pose"),
                # Publishes the decoded current goal for observability tools such
                # as the Air Traffic Control RQT panel.
                "atc_goal_name_pub": self.create_publisher(String, f"/{robot_name}/atc/current_goal_name", atc_goal_qos),
                "atc_goal_pose_pub": self.create_publisher(PoseStamped, f"/{robot_name}/atc/current_goal", atc_goal_qos),
                "goal_handle": None,
                "goal_index": 0,
                "goal_in_flight": False,
                "goal_start_time_ns": None,
                "goals": goals,
                "active_goal_span": None,
                # Start robots at different times to reduce immediate traffic conflicts.
                "release_delay_ns": int((robot_index - 1) * self.start_stagger_sec * 1e9),
                "ready": False,
                "ready_query_in_flight": False,
                # Workspace reservations keep two robots from targeting the same
                # workspace at the same time.
                "reserved_workspace": None,
                "result_future": None,
                # Queries the robot's bt_navigator lifecycle state before sending goals.
                "state_client": self.create_client(GetState, f"/{robot_name}/bt_navigator/get_state"),
                "traceparent": robot_traceparents[robot_name],
                "wait_logged_for_workspace": None,
                "wait_logged_for_return_lane": False,
                "goals_dispatched": 0,
                "goals_succeeded": 0,
                "goals_failed": 0,
                "goals_timed_out": 0,
                "cancel_requests": 0,
            }

        # Main scheduler: checks readiness, dispatches goals, and handles results.
        self.timer = self.create_timer(1.0, self._tick)
        self._log("info", f"Simulation telemetry run_id={self.run_id}")

    def _tick(self) -> None:
        # Establish a common start time so release_delay_ns is relative to this node.
        if self.start_time_ns is None:
            self.start_time_ns = self.get_clock().now().nanoseconds

        for robot_name, robot_state in self.robot_states.items():
            # Wait for each robot's Nav2 stack to become active before dispatching.
            if not robot_state["ready"]:
                self._request_nav_ready(robot_name, robot_state)
                continue

            # Stagger robot starts to reduce congestion at the beginning of the run.
            elapsed_ns = self.get_clock().now().nanoseconds - self.start_time_ns
            if elapsed_ns < robot_state["release_delay_ns"]:
                continue

            # While a goal is active, poll its result or timeout.
            if robot_state["goal_in_flight"]:
                self._check_goal_result(robot_name, robot_state)
                continue

            # A robot with no remaining goals is done.
            if robot_state["goal_index"] >= len(robot_state["goals"]):
                continue

            self._dispatch_goal(robot_name, robot_state)

        self._publish_diagnostic_snapshot_if_due()

    def _build_task_sequences(self) -> list[list[str]]:
        # Round-robin the shuffled workspaces so each robot gets task_waypoint_count
        # workspace goals without every robot taking the same sequence.
        shuffled = list(self.workspace_ids)
        self.random.shuffle(shuffled)
        sequences = [[] for _ in range(self.robot_count)]

        for task_index in range(self.task_waypoint_count):
            for robot_index in range(self.robot_count):
                workspace_index = (robot_index + task_index * self.robot_count) % len(shuffled)
                sequences[robot_index].append(shuffled[workspace_index])

        return sequences

    def _request_nav_ready(self, robot_name: str, robot_state: dict) -> None:
        # Avoid stacking multiple lifecycle service requests for the same robot.
        if robot_state["ready_query_in_flight"]:
            return

        state_client = robot_state["state_client"]
        if not state_client.wait_for_service(timeout_sec=0.0):
            return

        robot_state["ready_query_in_flight"] = True
        future = state_client.call_async(GetState.Request())
        future.add_done_callback(lambda done: self._on_state_response(robot_name, robot_state, done))

    def _on_state_response(self, robot_name: str, robot_state: dict, future) -> None:
        # Mark the robot ready only when bt_navigator reports the active state.
        robot_state["ready_query_in_flight"] = False
        try:
            response = future.result()
        except Exception as exc:
            self._log("warning", f"{robot_name}: failed to query bt_navigator state: {exc}", robot_name)
            return

        if response.current_state.label == "active":
            robot_state["ready"] = True
            with self.telemetry.start_as_current_span(
                "simulation.robot_nav_ready",
                run_id=self.run_id,
                robot_id=robot_name,
                traceparent=robot_state["traceparent"],
            ) as span:
                self._set_robot_traceparent(robot_name, Telemetry.traceparent_from_span(span))
                self._log("info", f"{robot_name}: Nav2 is active", robot_name)

    def _dispatch_goal(self, robot_name: str, robot_state: dict) -> None:
        # Send the next NavigateToPose goal if the action server is available.
        action_client = robot_state["action_client"]
        if not action_client.wait_for_server(timeout_sec=0.0):
            return

        goal_spec = robot_state["goals"][robot_state["goal_index"]]
        goal_label = goal_spec["label"]
        # Workspace goals are mutually exclusive across robots.
        if self._is_workspace_label(goal_label) and self._workspace_reserved_by_other(goal_label, robot_name):
            if robot_state["wait_logged_for_workspace"] != goal_label:
                robot_state["wait_logged_for_workspace"] = goal_label
                self._log("info", f"{robot_name}: waiting for {goal_label} to clear", robot_name, goal_label=goal_label)
            return
        robot_state["wait_logged_for_workspace"] = None
        robot_state["wait_logged_for_return_lane"] = False
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_spec["pose"]
        robot_state["reserved_workspace"] = goal_label if self._is_workspace_label(goal_label) else None
        self._publish_current_goal(robot_state, goal_label, goal_spec["pose"])
        self._log(
            "info",
            f"{robot_name}: dispatching goal {robot_state['goal_index'] + 1}/{len(robot_state['goals'])} "
            f"[{goal_label}]",
            robot_name,
            goal_label=goal_label,
            goal_index=robot_state["goal_index"] + 1,
            goal_count=len(robot_state["goals"]),
        )
        robot_state["goals_dispatched"] += 1
        robot_state["active_goal_span"] = self.telemetry.start_span(
            "simulation.navigate_goal",
            run_id=self.run_id,
            robot_id=robot_name,
            traceparent=robot_state["traceparent"],
            goal_label=goal_label,
            goal_index=robot_state["goal_index"] + 1,
            goal_count=len(robot_state["goals"]),
        )
        self._set_robot_traceparent(robot_name, Telemetry.traceparent_from_span(robot_state["active_goal_span"]))
        robot_state["active_goal_span"].add_event("goal_dispatched")
        future = action_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda done: self._on_goal_response(robot_name, robot_state, done))

    def _on_goal_response(self, robot_name: str, robot_state: dict, future) -> None:
        # Convert an accepted goal into an in-flight result future.
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._log("error", f"{robot_name}: failed to send goal: {exc}", robot_name)
            robot_state["active_goal_span"].record_exception(exc)
            robot_state["active_goal_span"].set_attribute("goal.accepted", False)
            self._end_active_goal_span(robot_state)
            return

        if not goal_handle.accepted:
            self._log("warning", f"{robot_name}: goal was rejected", robot_name)
            robot_state["active_goal_span"].set_attribute("goal.accepted", False)
            robot_state["active_goal_span"].add_event("goal_rejected")
            robot_state["reserved_workspace"] = None
            self._end_active_goal_span(robot_state)
            return

        robot_state["active_goal_span"].set_attribute("goal.accepted", True)
        robot_state["active_goal_span"].add_event("goal_accepted")
        robot_state["goal_handle"] = goal_handle
        robot_state["result_future"] = goal_handle.get_result_async()
        robot_state["goal_in_flight"] = True
        robot_state["goal_start_time_ns"] = self.get_clock().now().nanoseconds

    def _check_goal_result(self, robot_name: str, robot_state: dict) -> None:
        # Poll the in-flight action result and enforce per-goal timeouts.
        result_future = robot_state["result_future"]
        if result_future is None:
            return

        if not result_future.done():
            if robot_state["goal_start_time_ns"] is not None:
                elapsed_sec = (self.get_clock().now().nanoseconds - robot_state["goal_start_time_ns"]) / 1e9
                goal_spec = robot_state["goals"][robot_state["goal_index"]]
                timeout_sec = self._timeout_for_goal(goal_spec["label"])
                if elapsed_sec >= timeout_sec:
                    self._log(
                        "warning",
                        f"{robot_name}: goal [{goal_spec['label']}] timed out after {elapsed_sec:.0f}s, returning home",
                        robot_name,
                        goal_label=goal_spec["label"],
                        elapsed_sec=elapsed_sec,
                    )
                    robot_state["active_goal_span"].set_attribute("goal.timed_out", True)
                    robot_state["active_goal_span"].set_attribute("goal.elapsed_sec", elapsed_sec)
                    robot_state["active_goal_span"].add_event("goal_timeout")
                    robot_state["goals_timed_out"] += 1
                    # Timed-out robots skip remaining work and head to the final
                    # home goal after requesting cancellation.
                    self._cancel_active_goal(robot_name, robot_state)
                    robot_state["goal_index"] = len(robot_state["goals"]) - 1
                    robot_state["wait_logged_for_workspace"] = None
                    self._clear_goal_state(robot_name, robot_state)
            return

        goal_spec = robot_state["goals"][robot_state["goal_index"]]
        try:
            result = result_future.result()
        except Exception as exc:
            self._log(
                "error",
                f"{robot_name}: goal result failed for [{goal_spec['label']}]: {exc}",
                robot_name,
                goal_label=goal_spec["label"],
            )
            self._clear_goal_state(robot_name, robot_state)
            return

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self._log("info", f"{robot_name}: reached [{goal_spec['label']}]", robot_name, goal_label=goal_spec["label"])
            robot_state["active_goal_span"].set_attribute("goal.status", "succeeded")
            robot_state["active_goal_span"].add_event("goal_succeeded")
            robot_state["goals_succeeded"] += 1
            robot_state["goal_index"] += 1
        else:
            self._log(
                "warning",
                f"{robot_name}: goal [{goal_spec['label']}] finished with status {result.status}",
                robot_name,
                goal_label=goal_spec["label"],
                status=result.status,
            )
            robot_state["active_goal_span"].set_attribute("goal.status", result.status)
            robot_state["active_goal_span"].add_event("goal_finished_unsuccessfully")
            robot_state["goals_failed"] += 1
            # On failure, skip the remaining task/return sequence and send the
            # robot to the final home goal if it is not already there.
            if robot_state["goal_index"] < len(robot_state["goals"]) - 1:
                robot_state["goal_index"] = len(robot_state["goals"]) - 1

        self._clear_goal_state(robot_name, robot_state)

    def _cancel_active_goal(self, robot_name: str, robot_state: dict) -> None:
        # Best-effort cancellation for goals that exceed their timeout.
        goal_handle = robot_state["goal_handle"]
        if goal_handle is None:
            return

        try:
            cancel_future = goal_handle.cancel_goal_async()
        except Exception as exc:
            self._log("warning", f"{robot_name}: failed to request goal cancel: {exc}", robot_name)
            return

        robot_state["cancel_requests"] += 1
        cancel_future.add_done_callback(lambda done: self._on_cancel_response(robot_name, done))

    def _on_cancel_response(self, robot_name: str, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self._log("warning", f"{robot_name}: cancel request failed: {exc}", robot_name)
            return

        if len(response.goals_canceling) > 0:
            self._log("info", f"{robot_name}: canceled timed-out goal", robot_name)
        else:
            self._log("warning", f"{robot_name}: timed-out goal was not accepted for cancel", robot_name)

    @staticmethod
    def _is_workspace_label(label: str) -> bool:
        return label.startswith("ws")

    @staticmethod
    def _is_return_goal_label(label: str) -> bool:
        return "_return_" in label

    def _timeout_for_goal(self, goal_label: str) -> float:
        # Return/parking movements get a longer timeout than workspace visits.
        if (
            goal_label.endswith("_return_aisle")
            or goal_label.endswith("_return_transit")
            or goal_label.endswith("_return_stage_transit")
            or goal_label.endswith("_return_stage")
            or goal_label.startswith("robot")
        ):
            return self.return_goal_timeout_sec
        return self.goal_timeout_sec

    def _workspace_reserved_by_other(self, workspace_label: str, robot_name: str) -> bool:
        # True if another robot has already dispatched a goal for this workspace.
        for other_robot_name, other_state in self.robot_states.items():
            if other_robot_name == robot_name:
                continue
            if other_state["reserved_workspace"] == workspace_label:
                return True
        return False

    def _clear_goal_state(self, robot_name: str, robot_state: dict) -> None:
        # Reset per-goal state after success, failure, or timeout.
        robot_state["goal_handle"] = None
        robot_state["goal_in_flight"] = False
        robot_state["goal_start_time_ns"] = None
        robot_state["reserved_workspace"] = None
        robot_state["result_future"] = None
        robot_state["wait_logged_for_return_lane"] = False
        self._publish_current_goal(robot_state, "", None)
        self._end_active_goal_span(robot_state)

    def _publish_diagnostic_snapshot_if_due(self) -> None:
        if self.diagnostic_period_sec <= 0.0:
            return
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec < self.next_diagnostic_snapshot_sec:
            return
        self.next_diagnostic_snapshot_sec = now_sec + self.diagnostic_period_sec

        ready_count = sum(1 for state in self.robot_states.values() if state["ready"])
        in_flight = [name for name, state in self.robot_states.items() if state["goal_in_flight"]]
        completed = [
            name
            for name, state in self.robot_states.items()
            if state["goal_index"] >= len(state["goals"]) and not state["goal_in_flight"]
        ]
        waiting_for_nav = [name for name, state in self.robot_states.items() if not state["ready"]]
        waiting_for_workspace = [
            name
            for name, state in self.robot_states.items()
            if state["wait_logged_for_workspace"] is not None
        ]

        longest_goal_robot = ""
        longest_goal_label = ""
        longest_goal_elapsed_sec = 0.0
        for robot_name, state in self.robot_states.items():
            if not state["goal_in_flight"] or state["goal_start_time_ns"] is None:
                continue
            elapsed_sec = (self.get_clock().now().nanoseconds - state["goal_start_time_ns"]) / 1e9
            if elapsed_sec > longest_goal_elapsed_sec:
                longest_goal_robot = robot_name
                goal_index = state["goal_index"]
                longest_goal_label = state["goals"][goal_index]["label"] if goal_index < len(state["goals"]) else ""
                longest_goal_elapsed_sec = elapsed_sec

        self._log(
            "info",
            "Orchestrator diagnostic snapshot",
            ready_robot_count=ready_count,
            waiting_for_nav_count=len(waiting_for_nav),
            waiting_for_nav_robots=",".join(sorted(waiting_for_nav)),
            in_flight_goal_count=len(in_flight),
            in_flight_robots=",".join(sorted(in_flight)),
            completed_robot_count=len(completed),
            completed_robots=",".join(sorted(completed)),
            waiting_for_workspace_count=len(waiting_for_workspace),
            waiting_for_workspace_robots=",".join(sorted(waiting_for_workspace)),
            total_goals_dispatched=sum(state["goals_dispatched"] for state in self.robot_states.values()),
            total_goals_succeeded=sum(state["goals_succeeded"] for state in self.robot_states.values()),
            total_goals_failed=sum(state["goals_failed"] for state in self.robot_states.values()),
            total_goals_timed_out=sum(state["goals_timed_out"] for state in self.robot_states.values()),
            total_cancel_requests=sum(state["cancel_requests"] for state in self.robot_states.values()),
            longest_goal_robot_id=longest_goal_robot,
            longest_goal_label=longest_goal_label,
            longest_goal_elapsed_sec=longest_goal_elapsed_sec,
        )

    @staticmethod
    def _end_active_goal_span(robot_state: dict) -> None:
        span = robot_state.get("active_goal_span")
        if span is None:
            return
        span.end()
        robot_state["active_goal_span"] = None

    def _goal_pose_for_workspace(self, workspace_id: str) -> PoseStamped:
        # Workspace task goals target the generated approach cell for that workspace.
        goal_x, goal_y = cell_to_pose(workspace_approach_cell(workspace_id))
        return self._pose_stamped(goal_x, goal_y, 0.0)

    @staticmethod
    def _publish_current_goal(robot_state: dict, goal_label: str, goal_pose: PoseStamped | None) -> None:
        # Publish a human-readable goal label plus the goal pose so monitoring tools
        # do not have to display opaque Nav2 action UUIDs.
        name_msg = String()
        name_msg.data = goal_label
        robot_state["atc_goal_name_pub"].publish(name_msg)
        if goal_pose is not None:
            robot_state["atc_goal_pose_pub"].publish(goal_pose)

    def _pose_stamped(self, x: float, y: float, yaw: float) -> PoseStamped:
        # Helper for map-frame Nav2 goals with yaw-only orientation.
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _log(self, level: str, message: str, robot_name: str = "", **attrs) -> None:
        traceparent = ""
        if robot_name:
            robot_state = self.robot_states.get(robot_name, {})
            traceparent = robot_state.get("traceparent", "")
        self.telemetry.log(
            message,
            level=level,
            run_id=self.run_id,
            robot_id=robot_name,
            traceparent=traceparent,
            **attrs,
        )

    def _set_robot_traceparent(self, robot_name: str, traceparent: str) -> None:
        if not robot_name or not traceparent:
            return
        robot_state = self.robot_states.get(robot_name)
        if robot_state is not None:
            robot_state["traceparent"] = traceparent
        self.run_context_publisher.update_robot_traceparent(robot_name, traceparent)

    def destroy_node(self) -> bool:
        self.run_span.add_event("simulation_run_shutdown")
        self.run_span.end()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = MultiRobotOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
