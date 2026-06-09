import csv
import math
import os
import random

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.node import Node


class RandomWorkAreaGoalNode(Node):
    def __init__(self) -> None:
        super().__init__("random_work_area_goal")

        # BasicNavigator wraps Nav2 action/service clients for a single robot.
        self.navigator = BasicNavigator()

        # Goals are loaded from goals.csv rather than ROS parameters.
        self.goals = self._load_goals()

        # Timer starts a random goal whenever the node is idle.
        self.timer = self.create_timer(1.0, self._tick)
        self.active = False

    def _load_goals(self):
        # Prefer the installed package share path, with a local source-tree fallback.
        share_dir = get_package_share_directory("hippo_harvest_sim")
        candidate_paths = [
            os.path.join(share_dir, "maps", "goals.csv"),
            "/opt/code/HippoHarvest/simulation/layout/goals.csv",
        ]
        goals = []
        csv_path = next((path for path in candidate_paths if os.path.exists(path)), None)
        if csv_path is None:
            self.get_logger().warning(
                "No goals.csv found in either the installed package share directory "
                "or the local simulation/layout directory."
            )
            return goals

        # goals.csv rows are expected to include goal_id, x_m, y_m, and yaw_rad.
        with open(csv_path, newline="", encoding="ascii") as csv_file:
            reader = csv.DictReader(csv_file)
            for row in reader:
                goals.append(row)
        return goals

    def _tick(self) -> None:
        # Nothing to dispatch if no CSV goals were found or a navigation task is active.
        if not self.goals or self.active:
            return

        # Choose one random goal row and convert it to a map-frame PoseStamped.
        selected = random.choice(self.goals)
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(selected["x_m"])
        goal.pose.position.y = float(selected["y_m"])
        yaw = float(selected["yaw_rad"])
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f"Dispatching goal {selected['goal_id']} at "
            f"({goal.pose.position.x:.3f}, {goal.pose.position.y:.3f})"
        )
        self.navigator.goToPose(goal)
        self.active = True

        # Block inside this timer callback until Nav2 reports the task complete.
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.2)

        result = self.navigator.getResult()
        self.get_logger().info(f"Navigation result: {result}")
        self.active = False


def main() -> None:
    rclpy.init()
    node = RandomWorkAreaGoalNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
