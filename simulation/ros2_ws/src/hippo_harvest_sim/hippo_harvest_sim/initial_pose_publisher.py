import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from hippo_harvest_sim.layout_data import cell_to_pose, robot_home_cell


class InitialPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__("initial_pose_publisher")

        # start_pose_topic is consumed by the synthetic localization node as its
        # one-shot initial pose input.
        self.declare_parameter("start_pose_topic", "nav/start_pose")

        # robot_index and robot_count choose which generated home cell this robot
        # should start from in the shared layout.
        self.declare_parameter("robot_index", 1)
        self.declare_parameter("robot_count", 10)

        # Initial yaw in radians, written into the published PoseStamped orientation.
        self.declare_parameter("start_yaw", 0.0)

        start_pose_topic = str(self.get_parameter("start_pose_topic").value)
        robot_index = int(self.get_parameter("robot_index").value)
        robot_count = int(self.get_parameter("robot_count").value)
        self.start_yaw = float(self.get_parameter("start_yaw").value)

        # Publishes geometry_msgs/PoseStamped on the configured start-pose topic.
        self.publisher = self.create_publisher(PoseStamped, start_pose_topic, 10)

        # Publish the start pose a few times so late-starting subscribers still
        # have a chance to receive it, then stop the timer.
        self.publish_count = 0
        self.max_publishes = 5
        self.timer = self.create_timer(1.0, self.on_timer)

        # Convert the assigned grid home cell into map-frame meters.
        x, y = cell_to_pose(robot_home_cell(robot_index, robot_count))
        self.start_x = x
        self.start_y = y

    def on_timer(self) -> None:
        if self.publish_count >= self.max_publishes:
            self.timer.cancel()
            return

        # PoseStamped output: initial map-frame pose for this robot.
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = self.start_x
        msg.pose.position.y = self.start_y
        msg.pose.orientation.z = math.sin(self.start_yaw / 2.0)
        msg.pose.orientation.w = math.cos(self.start_yaw / 2.0)
        self.publisher.publish(msg)
        self.publish_count += 1

        if self.publish_count == 1:
            self.get_logger().info(
                f"Publishing initial pose at ({self.start_x:.3f}, {self.start_y:.3f})"
            )


def main() -> None:
    rclpy.init()
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
