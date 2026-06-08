import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

from hippo_harvest_sim.layout_data import cell_to_pose, is_in_bounds, planning_occupied_cells, workspace_approach_cell


def safe_staging_cell(cell: tuple[int, int], south_offset: int = 10) -> tuple[int, int]:
    x, y = cell
    occupied = planning_occupied_cells()
    for candidate_y in range(y - south_offset, -1, -1):
        candidate = (x, candidate_y)
        if is_in_bounds(candidate) and candidate not in occupied:
            return candidate
    raise RuntimeError(f"Could not find a staging cell south of {cell}")


class Nav2WorkAreaGoalNode(Node):
    def __init__(self) -> None:
        super().__init__("nav2_work_area_goal")
        ws2 = workspace_approach_cell("ws2")
        ws10 = workspace_approach_cell("ws10")
        ws2_exit = safe_staging_cell(ws2)
        self.goals = [
            {"label": "ws2", "cell": ws2},
            {"label": "ws2_exit", "cell": ws2_exit},
            {"label": "east_aisle_upper", "cell": (90, ws2_exit[1])},
            {"label": "east_aisle_lower", "cell": (90, ws10[1])},
            {"label": "ws10", "cell": ws10},
        ]
        self.goal_index = 0
        self.goal_in_flight = False
        self.nav2_ready = False
        self.goal_handle = None
        self.result_future = None
        self.nav_action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.bt_state_client = self.create_client(GetState, "bt_navigator/get_state")
        self.timer = self.create_timer(1.0, self._tick)

    def _tick(self) -> None:
        if not self.nav2_ready:
            self.nav2_ready = self._navigator_is_active()
            if not self.nav2_ready:
                return
            self.get_logger().info("Nav2 is active and ready for goals")

        if self.goal_in_flight:
            self._check_goal_result()
            return

        if self.goal_index >= len(self.goals):
            return

        self._dispatch_goal()

    def _navigator_is_active(self) -> bool:
        if not self.bt_state_client.wait_for_service(timeout_sec=0.0):
            self.get_logger().info("bt_navigator/get_state service not available yet")
            return False

        future = self.bt_state_client.call_async(GetState.Request())
        future.add_done_callback(self._on_state_response)
        return False

    def _on_state_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(f"Failed to query bt_navigator state: {exc}")
            return

        state = response.current_state.label
        if state == "active":
            self.nav2_ready = True
        else:
            self.get_logger().info(f"bt_navigator state is {state}, waiting for active")

    def _dispatch_goal(self) -> None:
        if not self.nav_action_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().info("navigate_to_pose action server not available yet")
            return

        goal_spec = self.goals[self.goal_index]
        goal_x, goal_y = cell_to_pose(goal_spec["cell"])
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.orientation.z = math.sin((-math.pi / 2.0) / 2.0)
        goal.pose.orientation.w = math.cos((-math.pi / 2.0) / 2.0)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal

        self.get_logger().info(
            f"Dispatching Nav2 goal {self.goal_index + 1}/{len(self.goals)} "
            f"[{goal_spec['label']}] at ({goal_x:.3f}, {goal_y:.3f})"
        )
        send_goal_future = self.nav_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to send goal: {exc}")
            return

        if not goal_handle.accepted:
            self.get_logger().error(f"Goal {self.goal_index + 1} was rejected")
            return

        self.goal_handle = goal_handle
        self.result_future = goal_handle.get_result_async()
        self.goal_in_flight = True

    def _check_goal_result(self) -> None:
        if self.result_future is None or not self.result_future.done():
            return

        try:
            result = self.result_future.result()
        except Exception as exc:
            self.get_logger().error(f"Goal result failed: {exc}")
            self.goal_in_flight = False
            self.result_future = None
            self.goal_handle = None
            return

        goal_spec = self.goals[self.goal_index]
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Goal {self.goal_index + 1} [{goal_spec['label']}] reached successfully")
            self.goal_index += 1
        else:
            self.get_logger().warning(
                f"Goal {self.goal_index + 1} [{goal_spec['label']}] finished with status {status}"
            )

        self.goal_in_flight = False
        self.result_future = None
        self.goal_handle = None


def main() -> None:
    rclpy.init()
    node = Nav2WorkAreaGoalNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
