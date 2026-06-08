import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from hippo_harvest_sim.layout_data import cell_to_pose


class InitialPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__("initial_pose_publisher")
        self.publisher = self.create_publisher(PoseStamped, "/nav/start_pose", 10)
        self.publish_count = 0
        self.max_publishes = 5
        self.timer = self.create_timer(1.0, self.on_timer)

        x, y = cell_to_pose((20, 20))
        self.start_x = x
        self.start_y = y
        self.start_yaw = math.atan2(2.1875 - y, 1.8875 - x)

    def on_timer(self) -> None:
        if self.publish_count >= self.max_publishes:
            self.timer.cancel()
            return

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
    finally:
        node.destroy_node()
        rclpy.shutdown()
