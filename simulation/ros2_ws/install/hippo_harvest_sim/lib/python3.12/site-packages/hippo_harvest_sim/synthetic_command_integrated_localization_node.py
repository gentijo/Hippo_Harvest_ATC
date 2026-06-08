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
        self.pose_pub = self.create_publisher(PoseStamped, "/synthetic_pose", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.cell_pub = self.create_publisher(PointStamped, "/robot_cell", 10)
        self.path_pub = self.create_publisher(Path, "/robot_path", 10)
        self.robot_marker_pub = self.create_publisher(Marker, "/robot_marker", 10)
        self.robot_marker_array_pub = self.create_publisher(MarkerArray, "/robot_marker_array", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.command_sub = self.create_subscription(Twist, "/executed_cmd_vel", self.on_command, 10)
        self.start_sub = self.create_subscription(PoseStamped, "/nav/start_pose", self.on_start_pose, 10)
        self.timer = self.create_timer(0.1, self.on_timer)

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

    def on_start_pose(self, msg: PoseStamped) -> None:
        if self.initialized:
            return
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.yaw = self._yaw_from_quaternion(msg.pose.orientation.z, msg.pose.orientation.w)
        self.initialized = True
        self.get_logger().info(f"Localization initialized at ({self.x:.3f}, {self.y:.3f})")

    def on_command(self, msg: Twist) -> None:
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def on_timer(self) -> None:
        if not self.initialized:
            return

        dt = 0.1
        self.yaw += self.angular_velocity * dt
        self.x += self.linear_velocity * math.cos(self.yaw) * dt
        self.y += self.linear_velocity * math.sin(self.yaw) * dt
        self.x = min(max(self.x, 0.0), self.grid_width_limit - self.grid_epsilon)
        self.y = min(max(self.y, 0.0), self.grid_height_limit - self.grid_epsilon)

        now_clock = self.get_clock().now()
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
        odom_msg.child_frame_id = "base_link"
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
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation = pose_msg.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

        robot_marker = Marker()
        robot_marker.header.frame_id = "map"
        robot_marker.header.stamp = now
        robot_marker.ns = "robot"
        robot_marker.id = 0
        robot_marker.type = Marker.SPHERE
        robot_marker.action = Marker.ADD
        robot_marker.pose.position.x = self.x
        robot_marker.pose.position.y = self.y
        robot_marker.pose.position.z = 0.0
        robot_marker.pose.orientation.w = 1.0
        robot_marker.scale.x = self.robot_diameter
        robot_marker.scale.y = self.robot_diameter
        robot_marker.scale.z = 0.05
        robot_marker.color.a = 1.0
        robot_marker.color.r = 0.0
        robot_marker.color.g = 0.85
        robot_marker.color.b = 1.0
        self.robot_marker_pub.publish(robot_marker)

        heading_marker = Marker()
        heading_marker.header.frame_id = "map"
        heading_marker.header.stamp = now
        heading_marker.ns = "robot"
        heading_marker.id = 1
        heading_marker.type = Marker.ARROW
        heading_marker.action = Marker.ADD
        heading_marker.pose = pose_msg.pose
        heading_marker.pose.position.z = 0.02
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
    finally:
        node.destroy_node()
        rclpy.shutdown()
