import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class SimpleRobotNode(Node):
    def __init__(self) -> None:
        super().__init__("simple_robot_node")

        # Motion-limit parameters. Incoming commands are clipped to these values
        # before they are republished as executed commands.
        self.declare_parameter("max_linear_speed", 0.10)
        self.declare_parameter("max_angular_speed", 0.6)

        # command_topic is the requested velocity input; executed_command_topic is
        # the output that downstream synthetic localization should integrate.
        self.declare_parameter("command_topic", "cmd_vel")
        self.declare_parameter("executed_command_topic", "executed_cmd_vel")

        # Used only for a startup log so namespaced robots identify themselves.
        self.declare_parameter("robot_name", self.get_namespace().strip("/") or "robot")

        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        command_topic = str(self.get_parameter("command_topic").value)
        executed_command_topic = str(self.get_parameter("executed_command_topic").value)
        robot_name = str(self.get_parameter("robot_name").value)

        # Listens for desired robot velocity commands.
        self.command_sub = self.create_subscription(Twist, command_topic, self.on_command, 10)

        # Publishes the limited command that the simple robot model actually accepts.
        self.executed_pub = self.create_publisher(Twist, executed_command_topic, 10)
        self.get_logger().info(f"Simple robot node ready for {robot_name}")

    def on_command(self, msg: Twist) -> None:
        # This simple robot model supports forward linear velocity and yaw rate only.
        executed = Twist()
        executed.linear.x = max(-self.max_linear_speed, min(self.max_linear_speed, msg.linear.x))
        executed.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, msg.angular.z))

        # Drop tiny values to exact zero so downstream integration does not drift
        # from numerical noise or near-zero controller output.
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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
