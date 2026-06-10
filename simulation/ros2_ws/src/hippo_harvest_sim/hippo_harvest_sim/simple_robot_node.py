import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool

from hippo_harvest_sim.telemetry import RunContextSubscriber, Telemetry


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
        self.declare_parameter("pause_topic", "atc/pause")

        # Used only for a startup log so namespaced robots identify themselves.
        self.declare_parameter("robot_name", self.get_namespace().strip("/") or "robot")

        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        command_topic = str(self.get_parameter("command_topic").value)
        executed_command_topic = str(self.get_parameter("executed_command_topic").value)
        pause_topic = str(self.get_parameter("pause_topic").value)
        self.robot_name = str(self.get_parameter("robot_name").value)
        self.paused_by_atc = False
        self.run_context = RunContextSubscriber(self)
        self.telemetry = Telemetry("hippo_harvest_sim", "simple_robot_node", self.get_logger())

        # Listens for desired robot velocity commands.
        self.command_sub = self.create_subscription(Twist, command_topic, self.on_command, 10)

        # Listens for centralized traffic-manager pause/resume decisions.
        self.pause_sub = self.create_subscription(Bool, pause_topic, self.on_pause, 10)

        # Publishes the limited command that the simple robot model actually accepts.
        self.executed_pub = self.create_publisher(Twist, executed_command_topic, 10)
        self._log("info", f"Simple robot node ready for {self.robot_name}")

    def on_pause(self, msg: Bool) -> None:
        was_paused = self.paused_by_atc
        self.paused_by_atc = msg.data
        if self.paused_by_atc != was_paused:
            with self.telemetry.start_as_current_span(
                "simulation.robot_pause_state_changed",
                run_id=self.run_context.run_id,
                robot_id=self.robot_name,
                traceparent=self.run_context.robot_traceparent(self.robot_name),
                paused=self.paused_by_atc,
            ) as span:
                self.run_context.update_robot_traceparent(self.robot_name, Telemetry.traceparent_from_span(span))
                state = "paused" if self.paused_by_atc else "resumed"
                self._log("info", f"{self.robot_name}: ATC {state} robot", paused=self.paused_by_atc)
        if self.paused_by_atc and not was_paused:
            self.executed_pub.publish(Twist())

    def on_command(self, msg: Twist) -> None:
        if self.paused_by_atc:
            self.executed_pub.publish(Twist())
            return

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
    node = SimpleRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
