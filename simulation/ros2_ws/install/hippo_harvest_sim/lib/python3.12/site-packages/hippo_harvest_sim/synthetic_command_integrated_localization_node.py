import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from hippo_harvest_sim.layout_data import GRID_HEIGHT_CELLS, GRID_RESOLUTION_M, GRID_WIDTH_CELLS
from hippo_harvest_sim.telemetry import RunContextSubscriber, Telemetry


class SyntheticCommandIntegratedLocalizationNode(Node):
    def __init__(self) -> None:
        super().__init__("synthetic_command_integrated_localization_node")
        default_robot_name = self.get_namespace().strip("/") or "robot"

        # Topic parameters. Relative topic names are resolved inside the node namespace;
        # the default marker topics are absolute, so they are shared/global unless overridden.
        self.declare_parameter("pose_topic", "synthetic_pose")
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("cell_topic", "robot_cell")
        self.declare_parameter("path_topic", "robot_path")
        self.declare_parameter("robot_marker_topic", "/robot_marker")
        self.declare_parameter("robot_marker_array_topic", "/robot_marker_array")
        self.declare_parameter("executed_command_topic", "executed_cmd_vel")
        self.declare_parameter("start_pose_topic", "nav/start_pose")
        self.declare_parameter("pause_topic", "atc/pause")

        # Frame/identity parameters. base_frame_id is used as the odometry child
        # frame and the child frame of the map -> base transform.
        self.declare_parameter("base_frame_id", f"{default_robot_name}/base_link")

        # robot_name is kept as configurable metadata for launch/multi-robot setup,
        # but this node currently does not use it beyond storing the value.
        self.declare_parameter("robot_name", default_robot_name)

        # robot_index gives this robot stable visualization marker IDs.
        self.declare_parameter("robot_index", 1)

        pose_topic = str(self.get_parameter("pose_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        cell_topic = str(self.get_parameter("cell_topic").value)
        path_topic = str(self.get_parameter("path_topic").value)
        robot_marker_topic = str(self.get_parameter("robot_marker_topic").value)
        robot_marker_array_topic = str(self.get_parameter("robot_marker_array_topic").value)
        executed_command_topic = str(self.get_parameter("executed_command_topic").value)
        start_pose_topic = str(self.get_parameter("start_pose_topic").value)
        pause_topic = str(self.get_parameter("pause_topic").value)
        self.base_frame_id = str(self.get_parameter("base_frame_id").value)
        self.robot_name = str(self.get_parameter("robot_name").value)
        self.robot_index = int(self.get_parameter("robot_index").value)
        self.run_context = RunContextSubscriber(self)
        self.telemetry = Telemetry(
            "hippo_harvest_sim",
            "synthetic_command_integrated_localization_node",
            self.get_logger(),
        )

        # Publishes the current synthetic pose as geometry_msgs/PoseStamped in map.
        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)

        # Publishes nav_msgs/Odometry with the current pose and most recent command
        # velocities. The odom parent frame is map; child_frame_id is base_frame_id.
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)

        # Publishes the robot's current grid cell as a PointStamped. point.x and
        # point.y hold integer cell coordinates converted to floats.
        self.cell_pub = self.create_publisher(PointStamped, cell_topic, 10)

        # Publishes a rolling nav_msgs/Path of recent poses for visualization/debugging.
        self.path_pub = self.create_publisher(Path, path_topic, 10)

        # Publishes a single RViz marker for the robot body.
        self.robot_marker_pub = self.create_publisher(Marker, robot_marker_topic, 10)

        # Publishes a marker array containing the robot body and heading arrow.
        self.robot_marker_array_pub = self.create_publisher(MarkerArray, robot_marker_array_topic, 10)

        # Broadcasts the synthetic map -> base_frame_id transform each update.
        self.tf_broadcaster = TransformBroadcaster(self)

        # Listens to the command that was actually executed and integrates linear.x
        # and angular.z into the synthetic pose estimate.
        self.command_sub = self.create_subscription(Twist, executed_command_topic, self.on_command, 10)

        # Listens for the initial localization pose. Only the first message is used.
        self.start_sub = self.create_subscription(PoseStamped, start_pose_topic, self.on_start_pose, 10)

        # Listens to ATC pause/resume decisions so visualization can show paused
        # robots in orange and active robots in blue.
        self.pause_sub = self.create_subscription(Bool, pause_topic, self.on_pause, 10)

        # The synthetic localization update loop runs at 30 Hz.
        self.timer_period_sec = 1.0 / 30.0
        self.timer = self.create_timer(self.timer_period_sec, self.on_timer)

        # Current synthetic robot state in the map frame.
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.paused_by_atc = False

        # Path messages are published in map and capped later to avoid unbounded growth.
        self.path = Path()
        self.path.header.frame_id = "map"

        # Grid geometry comes from layout_data rather than ROS parameters.
        self.cell_resolution = GRID_RESOLUTION_M
        self.grid_width_limit = GRID_WIDTH_CELLS * GRID_RESOLUTION_M
        self.grid_height_limit = GRID_HEIGHT_CELLS * GRID_RESOLUTION_M
        self.grid_epsilon = 1e-6

        # Visualization size for the robot body marker.
        self.robot_diameter = 0.125

        # The node does not publish localization output until start_pose initializes it.
        self.initialized = False
        self.last_status_log_ns = 0
        self.last_update_ns = None

    def on_start_pose(self, msg: PoseStamped) -> None:
        # This node treats start_pose as a one-shot initialization signal; later
        # messages are ignored so the integrated pose is not reset during motion.
        if self.initialized:
            return
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.yaw = self._yaw_from_quaternion(msg.pose.orientation.z, msg.pose.orientation.w)
        self.initialized = True
        self.last_update_ns = self.get_clock().now().nanoseconds
        self._log("info", f"Localization initialized at ({self.x:.3f}, {self.y:.3f})", x=self.x, y=self.y)

    def on_command(self, msg: Twist) -> None:
        # Only forward speed and yaw rate are modeled. Other Twist fields are ignored.
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def on_pause(self, msg: Bool) -> None:
        self.paused_by_atc = msg.data
        if self.paused_by_atc:
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0

    def on_timer(self) -> None:
        # Do not publish pose/odom/cell/path/markers until an initial pose arrives.
        if not self.initialized:
            return

        now_clock = self.get_clock().now()
        now_ns = now_clock.nanoseconds
        if self.last_update_ns is None:
            dt = self.timer_period_sec
        else:
            dt = (now_ns - self.last_update_ns) / 1e9
        self.last_update_ns = now_ns
        dt = min(max(dt, 0.0), 0.1)

        if self.paused_by_atc:
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0

        # Dead-reckon from the last executed command. This is synthetic localization,
        # not sensor fusion: no covariance, noise, slip, or measurements are applied.
        self.yaw += self.angular_velocity * dt
        self.x += self.linear_velocity * math.cos(self.yaw) * dt
        self.y += self.linear_velocity * math.sin(self.yaw) * dt

        # Keep the synthetic pose inside the known grid.
        self.x = min(max(self.x, 0.0), self.grid_width_limit - self.grid_epsilon)
        self.y = min(max(self.y, 0.0), self.grid_height_limit - self.grid_epsilon)

        now = now_clock.to_msg()

        # PoseStamped output: current pose in map.
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = now
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.orientation.z = math.sin(self.yaw / 2.0)
        pose_msg.pose.orientation.w = math.cos(self.yaw / 2.0)
        self.pose_pub.publish(pose_msg)

        # Odometry output: same pose plus the latest command velocities.
        odom_msg = Odometry()
        odom_msg.header.frame_id = "map"
        odom_msg.header.stamp = now
        odom_msg.child_frame_id = self.base_frame_id
        odom_msg.pose.pose = pose_msg.pose
        odom_msg.twist.twist.linear.x = self.linear_velocity
        odom_msg.twist.twist.angular.z = self.angular_velocity
        self.odom_pub.publish(odom_msg)

        # Grid-cell output: floor the continuous map position into layout cells.
        cell_x = int(math.floor(self.x / self.cell_resolution))
        cell_y = int(math.floor(self.y / self.cell_resolution))
        cell_msg = PointStamped()
        cell_msg.header.frame_id = "map"
        cell_msg.header.stamp = now
        cell_msg.point.x = float(cell_x)
        cell_msg.point.y = float(cell_y)
        self.cell_pub.publish(cell_msg)

        # Path output: append the latest pose and keep only the most recent 2000.
        self.path.header.stamp = now
        self.path.poses.append(pose_msg)
        if len(self.path.poses) > 2000:
            self.path.poses = self.path.poses[-2000:]
        self.path_pub.publish(self.path)

        # TF output: publish map -> base_frame_id for consumers that use tf2.
        transform = TransformStamped()
        transform.header.frame_id = "map"
        transform.header.stamp = now
        transform.child_frame_id = self.base_frame_id
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation = pose_msg.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

        # Body marker: blue sphere normally, orange while ATC is holding this robot.
        robot_marker = Marker()
        robot_marker.header.frame_id = "map"
        robot_marker.header.stamp = now
        robot_marker.ns = "robot"
        robot_marker.id = self.robot_index * 10
        robot_marker.type = Marker.SPHERE
        robot_marker.action = Marker.ADD
        robot_marker.pose.position.x = self.x
        robot_marker.pose.position.y = self.y
        robot_marker.pose.position.z = 0.06
        robot_marker.pose.orientation.w = 1.0
        robot_marker.scale.x = self.robot_diameter
        robot_marker.scale.y = self.robot_diameter
        robot_marker.scale.z = 0.08
        robot_marker.color.a = 1.0
        if self.paused_by_atc:
            robot_marker.color.r = 1.0
            robot_marker.color.g = 0.45
            robot_marker.color.b = 0.0
        else:
            robot_marker.color.r = 0.0
            robot_marker.color.g = 0.35
            robot_marker.color.b = 1.0
        self.robot_marker_pub.publish(robot_marker)

        # Heading marker: black arrow aligned with the current pose orientation.
        heading_marker = Marker()
        heading_marker.header.frame_id = "map"
        heading_marker.header.stamp = now
        heading_marker.ns = "robot"
        heading_marker.id = self.robot_index * 10 + 1
        heading_marker.type = Marker.ARROW
        heading_marker.action = Marker.ADD
        heading_marker.pose = pose_msg.pose
        heading_marker.pose.position.z = 0.11
        heading_marker.scale.x = 0.10
        heading_marker.scale.y = 0.04
        heading_marker.scale.z = 0.04
        heading_marker.color.a = 1.0
        heading_marker.color.r = 0.0
        heading_marker.color.g = 0.0
        heading_marker.color.b = 0.0

        # MarkerArray consumers get both the body and heading in one message.
        marker_array = MarkerArray()
        marker_array.markers = [robot_marker, heading_marker]
        self.robot_marker_array_pub.publish(marker_array)

        # Rate-limited status log for humans watching the simulation.
        if now_clock.nanoseconds - self.last_status_log_ns >= 1_000_000_000:
            self.last_status_log_ns = now_clock.nanoseconds
            self._log(
                "info",
                f"Pose ({self.x:.3f}, {self.y:.3f}) yaw {self.yaw:.2f} cell ({cell_x}, {cell_y}) "
                f"cmd v={self.linear_velocity:.3f} w={self.angular_velocity:.3f}",
                x=self.x,
                y=self.y,
                yaw=self.yaw,
                cell_x=cell_x,
                cell_y=cell_y,
                linear_velocity=self.linear_velocity,
                angular_velocity=self.angular_velocity,
            )

    @staticmethod
    def _yaw_from_quaternion(z: float, w: float) -> float:
        return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)

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
    node = SyntheticCommandIntegratedLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
