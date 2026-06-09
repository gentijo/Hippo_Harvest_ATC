import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from hippo_harvest_sim.layout_data import GRID_HEIGHT_CELLS, GRID_RESOLUTION_M, GRID_WIDTH_CELLS


class SyntheticCommandIntegratedLocalizationNode(Node):
    def __init__(self) -> None:
        super().__init__("synthetic_command_integrated_localization_node")
        default_robot_name = self.get_namespace().strip("/") or "robot"
        self.declare_parameter("pose_topic", "synthetic_pose")
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("cell_topic", "robot_cell")
        self.declare_parameter("path_topic", "robot_path")
        self.declare_parameter("robot_marker_topic", "/robot_marker")
        self.declare_parameter("robot_marker_array_topic", "/robot_marker_array")
        self.declare_parameter("executed_command_topic", "executed_cmd_vel")
        self.declare_parameter("start_pose_topic", "nav/start_pose")
        self.declare_parameter("base_frame_id", f"{default_robot_name}/base_link")
        self.declare_parameter("robot_name", default_robot_name)
        self.declare_parameter("robot_index", 1)

        pose_topic = str(self.get_parameter("pose_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        cell_topic = str(self.get_parameter("cell_topic").value)
        path_topic = str(self.get_parameter("path_topic").value)
        robot_marker_topic = str(self.get_parameter("robot_marker_topic").value)
        robot_marker_array_topic = str(self.get_parameter("robot_marker_array_topic").value)
        executed_command_topic = str(self.get_parameter("executed_command_topic").value)
        start_pose_topic = str(self.get_parameter("start_pose_topic").value)
        self.base_frame_id = str(self.get_parameter("base_frame_id").value)
        self.robot_name = str(self.get_parameter("robot_name").value)
        self.robot_index = int(self.get_parameter("robot_index").value)

        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.cell_pub = self.create_publisher(PointStamped, cell_topic, 10)
        self.path_pub = self.create_publisher(Path, path_topic, 10)
        self.robot_marker_pub = self.create_publisher(Marker, robot_marker_topic, 10)
        self.robot_marker_array_pub = self.create_publisher(MarkerArray, robot_marker_array_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.command_sub = self.create_subscription(Twist, executed_command_topic, self.on_command, 10)
        self.start_sub = self.create_subscription(PoseStamped, start_pose_topic, self.on_start_pose, 10)
        self.timer_period_sec = 1.0 / 30.0
        self.timer = self.create_timer(self.timer_period_sec, self.on_timer)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.path = Path()
        self.path.header.frame_id = "map"
        self.cell_resolution = GRID_RESOLUTION_M
        self.grid_width_limit = GRID_WIDTH_CELLS * GRID_RESOLUTION_M
        self.grid_height_limit = GRID_HEIGHT_CELLS * GRID_RESOLUTION_M
        self.grid_epsilon = 1e-6
        self.robot_diameter = 0.125
        self.initialized = False
        self.last_status_log_ns = 0
        self.last_update_ns = None

    def on_start_pose(self, msg: PoseStamped) -> None:
        if self.initialized:
            return
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.yaw = self._yaw_from_quaternion(msg.pose.orientation.z, msg.pose.orientation.w)
        self.initialized = True
        self.last_update_ns = self.get_clock().now().nanoseconds
        self.get_logger().info(f"Localization initialized at ({self.x:.3f}, {self.y:.3f})")

    def on_command(self, msg: Twist) -> None:
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def on_timer(self) -> None:
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
        self.yaw += self.angular_velocity * dt
        self.x += self.linear_velocity * math.cos(self.yaw) * dt
        self.y += self.linear_velocity * math.sin(self.yaw) * dt
        self.x = min(max(self.x, 0.0), self.grid_width_limit - self.grid_epsilon)
        self.y = min(max(self.y, 0.0), self.grid_height_limit - self.grid_epsilon)

        now = now_clock.to_msg()
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = now
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.orientation.z = math.sin(self.yaw / 2.0)
        pose_msg.pose.orientation.w = math.cos(self.yaw / 2.0)
        self.pose_pub.publish(pose_msg)

        odom_msg = Odometry()
        odom_msg.header.frame_id = "map"
        odom_msg.header.stamp = now
        odom_msg.child_frame_id = self.base_frame_id
        odom_msg.pose.pose = pose_msg.pose
        odom_msg.twist.twist.linear.x = self.linear_velocity
        odom_msg.twist.twist.angular.z = self.angular_velocity
        self.odom_pub.publish(odom_msg)

        cell_x = int(math.floor(self.x / self.cell_resolution))
        cell_y = int(math.floor(self.y / self.cell_resolution))
        cell_msg = PointStamped()
        cell_msg.header.frame_id = "map"
        cell_msg.header.stamp = now
        cell_msg.point.x = float(cell_x)
        cell_msg.point.y = float(cell_y)
        self.cell_pub.publish(cell_msg)

        self.path.header.stamp = now
        self.path.poses.append(pose_msg)
        if len(self.path.poses) > 2000:
            self.path.poses = self.path.poses[-2000:]
        self.path_pub.publish(self.path)

        transform = TransformStamped()
        transform.header.frame_id = "map"
        transform.header.stamp = now
        transform.child_frame_id = self.base_frame_id
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation = pose_msg.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

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
        robot_marker.color.r = 0.0
        robot_marker.color.g = 0.85
        robot_marker.color.b = 1.0
        self.robot_marker_pub.publish(robot_marker)

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

        marker_array = MarkerArray()
        marker_array.markers = [robot_marker, heading_marker]
        self.robot_marker_array_pub.publish(marker_array)

        if now_clock.nanoseconds - self.last_status_log_ns >= 1_000_000_000:
            self.last_status_log_ns = now_clock.nanoseconds
            self.get_logger().info(
                f"Pose ({self.x:.3f}, {self.y:.3f}) yaw {self.yaw:.2f} cell ({cell_x}, {cell_y}) "
                f"cmd v={self.linear_velocity:.3f} w={self.angular_velocity:.3f}"
            )

    @staticmethod
    def _yaw_from_quaternion(z: float, w: float) -> float:
        return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


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
