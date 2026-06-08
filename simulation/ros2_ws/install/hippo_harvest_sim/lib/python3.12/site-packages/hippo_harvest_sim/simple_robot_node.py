import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class SimpleRobotNode(Node):
    def __init__(self) -> None:
        super().__init__("simple_robot_node")
        self.max_linear_speed = 0.10
        self.max_angular_speed = 0.6
        self.command_sub = self.create_subscription(Twist, "/cmd_vel", self.on_command, 10)
        self.executed_pub = self.create_publisher(Twist, "/executed_cmd_vel", 10)
        self.get_logger().info("Simple robot node ready")

    def on_command(self, msg: Twist) -> None:
        executed = Twist()
        executed.linear.x = max(-self.max_linear_speed, min(self.max_linear_speed, msg.linear.x))
        executed.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, msg.angular.z))

        if abs(executed.linear.x) < 1e-4:
            executed.linear.x = 0.0
        if abs(executed.angular.z) < 1e-4:
            executed.angular.z = 0.0

        self.executed_pub.publish(executed)


def main() -> None:
    rclpy.init()
    node = SimpleRobotNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
