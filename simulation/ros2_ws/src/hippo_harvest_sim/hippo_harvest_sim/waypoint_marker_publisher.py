import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from hippo_harvest_sim.layout_data import (
    cell_to_pose,
    default_start_cell,
    generate_tables,
    workspace_approach_cell,
)


class WaypointMarkerPublisher(Node):
    def __init__(self) -> None:
        super().__init__("waypoint_marker_publisher")
        self.waypoint_a_pub = self.create_publisher(PoseStamped, "/waypoint_a", 10)
        self.waypoint_b_pub = self.create_publisher(PoseStamped, "/waypoint_b", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/waypoint_markers", 10)
        self.start_pose = cell_to_pose(default_start_cell())
        self.spawn_pose = cell_to_pose((20, 20))
        self.target_workspace_ids = ["ws2", "ws10"]
        self.target_cells = [workspace_approach_cell(workspace_id) for workspace_id in self.target_workspace_ids]
        self.workspace_tables = generate_tables()
        self.timer = self.create_timer(1.0, self.on_timer)

    def _make_pose(self, x: float, y: float) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.w = 1.0
        return msg

    def on_timer(self) -> None:
        self.waypoint_a_pub.publish(self._make_pose(self.start_pose[0], self.start_pose[1]))
        current_goal_pose = cell_to_pose(self.target_cells[0])
        self.waypoint_b_pub.publish(self._make_pose(current_goal_pose[0], current_goal_pose[1]))
        self._publish_waypoint_markers()

    def _publish_waypoint_markers(self) -> None:
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()

        start_marker = Marker()
        start_marker.header.frame_id = "map"
        start_marker.header.stamp = now
        start_marker.ns = "waypoints"
        start_marker.id = 0
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        start_marker.pose.position.x = self.start_pose[0]
        start_marker.pose.position.y = self.start_pose[1]
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
        start_label.pose.position.x = self.start_pose[0]
        start_label.pose.position.y = self.start_pose[1]
        start_label.pose.position.z = 0.18
        start_label.pose.orientation.w = 1.0
        start_label.scale.z = 0.14
        start_label.color.a = 1.0
        start_label.color.r = 1.0
        start_label.color.g = 1.0
        start_label.color.b = 1.0
        start_label.text = "START"
        markers.markers.append(start_label)

        spawn_marker = Marker()
        spawn_marker.header.frame_id = "map"
        spawn_marker.header.stamp = now
        spawn_marker.ns = "waypoints"
        spawn_marker.id = 50
        spawn_marker.type = Marker.CUBE
        spawn_marker.action = Marker.ADD
        spawn_marker.pose.position.x = self.spawn_pose[0]
        spawn_marker.pose.position.y = self.spawn_pose[1]
        spawn_marker.pose.orientation.w = 1.0
        spawn_marker.scale.x = 0.12
        spawn_marker.scale.y = 0.12
        spawn_marker.scale.z = 0.08
        spawn_marker.color.a = 1.0
        spawn_marker.color.r = 0.1
        spawn_marker.color.g = 0.6
        spawn_marker.color.b = 1.0
        markers.markers.append(spawn_marker)

        spawn_label = Marker()
        spawn_label.header.frame_id = "map"
        spawn_label.header.stamp = now
        spawn_label.ns = "waypoint_labels"
        spawn_label.id = 150
        spawn_label.type = Marker.TEXT_VIEW_FACING
        spawn_label.action = Marker.ADD
        spawn_label.pose.position.x = self.spawn_pose[0]
        spawn_label.pose.position.y = self.spawn_pose[1]
        spawn_label.pose.position.z = 0.18
        spawn_label.pose.orientation.w = 1.0
        spawn_label.scale.z = 0.12
        spawn_label.color.a = 1.0
        spawn_label.color.r = 0.8
        spawn_label.color.g = 0.95
        spawn_label.color.b = 1.0
        spawn_label.text = "SPAWN"
        markers.markers.append(spawn_label)

        for idx, workspace_id in enumerate(self.target_workspace_ids, start=1):
            target_pose = cell_to_pose(self.target_cells[idx - 1])

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


def main() -> None:
    rclpy.init()
    node = WaypointMarkerPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
