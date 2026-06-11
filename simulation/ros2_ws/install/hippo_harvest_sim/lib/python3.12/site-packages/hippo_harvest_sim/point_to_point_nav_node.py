import heapq
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from hippo_harvest_sim.layout_data import (
    GRID_RESOLUTION_M,
    cell_to_pose,
    default_start_cell,
    generate_tables,
    is_in_bounds,
    planning_occupied_cells,
    workspace_approach_cell,
)
from hippo_harvest_sim.telemetry import RunContextSubscriber, Telemetry


class PointToPointNavNode(Node):
    def __init__(self) -> None:
        super().__init__("point_to_point_nav_node")
        self.robot_name = self.get_namespace().strip("/") or "robot"
        self.run_context = RunContextSubscriber(self)
        self.telemetry = Telemetry("hippo_harvest_sim", "point_to_point_nav_node", self.get_logger())

        # Publishes raw velocity commands for the simple robot node to limit/execute.
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Publishes the initial pose consumed by synthetic localization.
        self.start_pose_pub = self.create_publisher(PoseStamped, "/nav/start_pose", 10)

        # Publishes visualization/helper poses for the route start and current target.
        self.waypoint_a_pub = self.create_publisher(PoseStamped, "/waypoint_a", 10)
        self.waypoint_b_pub = self.create_publisher(PoseStamped, "/waypoint_b", 10)

        # Publishes marker arrays and planned path output for RViz/debugging.
        self.marker_pub = self.create_publisher(MarkerArray, "/waypoint_markers", 10)
        self.plan_pub = self.create_publisher(Path, "/nav_plan", 10)

        # Listens to the synthetic localization pose and drives from that estimate.
        self.pose_sub = self.create_subscription(PoseStamped, "/synthetic_pose", self.on_pose, 10)

        # Controller/update loop at 10 Hz.
        self.timer = self.create_timer(0.1, self.on_timer)

        # Controller and planning constants for the simple point-to-point route.
        self.grid_resolution = GRID_RESOLUTION_M
        self.goal_tolerance = 0.04
        self.max_linear_speed = 0.10
        self.max_angular_speed = 0.6
        self.heading_tolerance = 0.08
        self.current_pose = None
        self.started = False
        self.completed = False
        self.workspace_tables = generate_tables()
        self.occupied = planning_occupied_cells()

        # Hard-coded route: start at the default cell, visit ws2, then ws10.
        self.start_cell = default_start_cell()
        self.target_workspace_ids = ["ws2", "ws10"]
        self.target_cells = [workspace_approach_cell(workspace_id) for workspace_id in self.target_workspace_ids]
        self.current_target_index = 0
        self.current_path_cells = []
        self.current_path_index = 0

        self.a_pose = cell_to_pose(self.start_cell)
        self.b_pose = cell_to_pose(self.target_cells[0])
        self._plan_current_leg(self.start_cell)

        self._log(
            "info",
            f"Start cell {self.start_cell} -> {self.a_pose}; route targets: "
            f"{self.target_workspace_ids[0]} via {self.target_cells[0]}, "
            f"then {self.target_workspace_ids[1]} via {self.target_cells[1]}",
            start_cell=str(self.start_cell),
        )

    def _make_pose(self, x: float, y: float) -> PoseStamped:
        # Helper for map-frame PoseStamped messages with no yaw rotation.
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.w = 1.0
        return msg

    def _publish_start_pose(self) -> None:
        # Re-publish the start pose until localization sends its first pose update.
        self.start_pose_pub.publish(self._make_pose(self.a_pose[0], self.a_pose[1]))

    def _publish_waypoint_poses(self) -> None:
        # waypoint_a is the route start; waypoint_b is the active target workspace.
        self.waypoint_a_pub.publish(self._make_pose(self.a_pose[0], self.a_pose[1]))
        current_goal_pose = cell_to_pose(self.target_cells[self.current_target_index])
        self.waypoint_b_pub.publish(self._make_pose(current_goal_pose[0], current_goal_pose[1]))

    def _publish_waypoint_markers(self) -> None:
        # MarkerArray output: start marker, route target markers, all workstation
        # approach points, and text labels for RViz.
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()

        start_marker = Marker()
        start_marker.header.frame_id = "map"
        start_marker.header.stamp = now
        start_marker.ns = "waypoints"
        start_marker.id = 0
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        start_marker.pose.position.x = self.a_pose[0]
        start_marker.pose.position.y = self.a_pose[1]
        start_marker.pose.orientation.w = 1.0
        start_marker.scale.x = 0.18
        start_marker.scale.y = 0.18
        start_marker.scale.z = 0.08
        start_marker.color.a = 1.0
        start_marker.color.r = 0.1
        start_marker.color.g = 0.8
        start_marker.color.b = 0.1
        markers.markers.append(start_marker)

        start_label = Marker()
        start_label.header.frame_id = "map"
        start_label.header.stamp = now
        start_label.ns = "waypoint_labels"
        start_label.id = 100
        start_label.type = Marker.TEXT_VIEW_FACING
        start_label.action = Marker.ADD
        start_label.pose.position.x = self.a_pose[0]
        start_label.pose.position.y = self.a_pose[1]
        start_label.pose.position.z = 0.18
        start_label.pose.orientation.w = 1.0
        start_label.scale.z = 0.14
        start_label.color.a = 1.0
        start_label.color.r = 1.0
        start_label.color.g = 1.0
        start_label.color.b = 1.0
        start_label.text = "START"
        markers.markers.append(start_label)

        for idx, workspace_id in enumerate(self.target_workspace_ids, start=1):
            target_cell = self.target_cells[idx - 1]
            target_pose = cell_to_pose(target_cell)
            goal_marker = Marker()
            goal_marker.header.frame_id = "map"
            goal_marker.header.stamp = now
            goal_marker.ns = "route_targets"
            goal_marker.id = idx
            goal_marker.type = Marker.SPHERE
            goal_marker.action = Marker.ADD
            goal_marker.pose.position.x = target_pose[0]
            goal_marker.pose.position.y = target_pose[1]
            goal_marker.pose.orientation.w = 1.0
            goal_marker.scale.x = 0.18
            goal_marker.scale.y = 0.18
            goal_marker.scale.z = 0.08
            goal_marker.color.a = 1.0
            goal_marker.color.r = 0.9
            goal_marker.color.g = 0.1
            goal_marker.color.b = 0.1
            markers.markers.append(goal_marker)

            label = Marker()
            label.header.frame_id = "map"
            label.header.stamp = now
            label.ns = "route_target_labels"
            label.id = 100 + idx
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = target_pose[0]
            label.pose.position.y = target_pose[1]
            label.pose.position.z = 0.18
            label.pose.orientation.w = 1.0
            label.scale.z = 0.14
            label.color.a = 1.0
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.text = workspace_id.upper()
            markers.markers.append(label)

        for index, table in enumerate(self.workspace_tables, start=1):
            pose_x, pose_y = cell_to_pose(table["waypoint_cell"])

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = now
            marker.ns = "workstations"
            marker.id = 1000 + index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = pose_x
            marker.pose.position.y = pose_y
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.10
            marker.scale.y = 0.10
            marker.scale.z = 0.06
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.85
            marker.color.b = 0.10
            markers.markers.append(marker)

            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = now
            text_marker.ns = "workstation_labels"
            text_marker.id = 2000 + index
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = pose_x
            text_marker.pose.position.y = pose_y
            text_marker.pose.position.z = 0.14
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.10
            text_marker.color.a = 1.0
            text_marker.color.r = 0.0
            text_marker.color.g = 0.0
            text_marker.color.b = 0.0
            text_marker.text = table["id"]
            markers.markers.append(text_marker)

        self.marker_pub.publish(markers)

    def _neighbors(self, cell):
        # Four-connected grid neighbors that are in bounds and not planning obstacles.
        x, y = cell
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            nxt = (x + dx, y + dy)
            if is_in_bounds(nxt) and nxt not in self.occupied:
                yield nxt

    @staticmethod
    def _heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def _a_star(self, start, goal):
        # A* over grid cells using Manhattan distance and unit movement cost.
        frontier = []
        heapq.heappush(frontier, (0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}

        while frontier:
            _, current = heapq.heappop(frontier)
            if current == goal:
                break

            for nxt in self._neighbors(current):
                new_cost = cost_so_far[current] + 1
                if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                    cost_so_far[nxt] = new_cost
                    priority = new_cost + self._heuristic(goal, nxt)
                    heapq.heappush(frontier, (priority, nxt))
                    came_from[nxt] = current

        if goal not in came_from:
            raise RuntimeError(f"No path found from {start} to {goal}")

        path = []
        current = goal
        while current is not None:
            path.append(current)
            current = came_from[current]
        path.reverse()
        return path

    def _publish_plan(self):
        # Publish the current grid path as a nav_msgs/Path in map-frame meters.
        plan = Path()
        plan.header.frame_id = "map"
        plan.header.stamp = self.get_clock().now().to_msg()
        for cell in self.current_path_cells:
            pose = cell_to_pose(cell)
            plan.poses.append(self._make_pose(pose[0], pose[1]))
        self.plan_pub.publish(plan)

    def _plan_current_leg(self, start_cell):
        # Re-plan from the supplied start cell to the active target workspace.
        target_cell = self.target_cells[self.current_target_index]
        self.current_path_cells = self._a_star(start_cell, target_cell)
        self.current_path_index = 1 if len(self.current_path_cells) > 1 else 0
        self.b_pose = cell_to_pose(target_cell)
        self._publish_plan()
        self._log(
            "info",
            f"Planned path to {self.target_workspace_ids[self.current_target_index]} with "
            f"{len(self.current_path_cells)} cells",
            target_workspace=self.target_workspace_ids[self.current_target_index],
            path_cell_count=len(self.current_path_cells),
        )

    def _distance_to_current_goal(self):
        # Euclidean distance from the current localization pose to the active target.
        goal_x, goal_y = cell_to_pose(self.target_cells[self.current_target_index])
        dx = goal_x - self.current_pose.pose.position.x
        dy = goal_y - self.current_pose.pose.position.y
        return math.hypot(dx, dy)

    def _advance_goal_if_needed(self):
        # When the active workspace is reached, either finish or plan the next leg.
        distance_to_goal = self._distance_to_current_goal()
        if distance_to_goal > self.goal_tolerance:
            return False

        self._log(
            "info",
            f"Reached {self.target_workspace_ids[self.current_target_index]} at distance {distance_to_goal:.3f} m",
            target_workspace=self.target_workspace_ids[self.current_target_index],
            distance_to_goal_m=distance_to_goal,
        )
        if self.current_target_index == len(self.target_workspace_ids) - 1:
            self.completed = True
            self.cmd_pub.publish(Twist())
            self._log("info", "Reached final target and stopped")
            return True

        self.current_target_index += 1
        next_start = self.current_path_cells[min(self.current_path_index, len(self.current_path_cells) - 1)]
        self._plan_current_leg(next_start)
        return True

    def on_pose(self, msg: PoseStamped) -> None:
        # Synthetic localization callback. The first pose means the start pose was accepted.
        self.current_pose = msg
        if not self.started:
            self.started = True
            self._log("info", "Navigation received first localization update")

    def on_timer(self) -> None:
        # Always refresh visualization topics, even before navigation starts.
        self._publish_waypoint_poses()
        self._publish_waypoint_markers()
        self._publish_plan()

        if not self.started:
            self._publish_start_pose()
            return

        # Stop commanding once the route is done or before any pose has arrived.
        if self.completed or self.current_pose is None:
            return

        if self._advance_goal_if_needed():
            return

        if self.current_path_index >= len(self.current_path_cells):
            # If the path is exhausted but the final target is not reached, stop
            # instead of driving without a target cell.
            if self._distance_to_current_goal() <= self.goal_tolerance:
                self._advance_goal_if_needed()
            else:
                self.cmd_pub.publish(Twist())
            return

        target_cell = self.current_path_cells[self.current_path_index]
        target_pose = cell_to_pose(target_cell)
        cmd = Twist()

        # Pure pursuit-style controller toward the next grid-cell center.
        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y
        yaw = self._yaw_from_pose(self.current_pose)
        dx = target_pose[0] - x
        dy = target_pose[1] - y
        distance = math.hypot(dx, dy)

        if distance <= self.goal_tolerance:
            self.current_path_index += 1
            self.cmd_pub.publish(Twist())
            return

        target_yaw = math.atan2(dy, dx)
        yaw_error = self._normalize_angle(target_yaw - yaw)

        if abs(yaw_error) > self.heading_tolerance:
            # Turn in place until roughly aligned with the next path segment.
            cmd.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, 2.0 * yaw_error))
            cmd.linear.x = 0.0
        else:
            # Drive forward while applying a smaller heading correction.
            cmd.angular.z = max(-0.2, min(0.2, 1.5 * yaw_error))
            cmd.linear.x = min(self.max_linear_speed, 1.5 * distance)

        self.cmd_pub.publish(cmd)

    @staticmethod
    def _yaw_from_pose(msg: PoseStamped) -> float:
        z = msg.pose.orientation.z
        w = msg.pose.orientation.w
        return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _log(self, level: str, message: str, **attrs) -> None:
        self.telemetry.log(
            message,
            level=level,
            run_id=self.run_context.run_id,
            robot_id=self.robot_name,
            traceparent=self.run_context.robot_traceparent(self.robot_name),
            **attrs,
        )


def main() -> None:
    rclpy.init()
    node = PointToPointNavNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
