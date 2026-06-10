import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from hippo_harvest_sim.layout_data import (
    cell_to_pose,
    generate_tables,
    robot_home_cells,
    workspace_approach_cell,
)


class WaypointMarkerPublisher(Node):
    def __init__(self) -> None:
        super().__init__("waypoint_marker_publisher")

        # Number of robot home markers to generate along the west side of the map.
        self.declare_parameter("robot_count", 10)
        self.robot_count = int(self.get_parameter("robot_count").value)

        # Publishes helper waypoint poses for consumers/visualizers that expect them.
        self.waypoint_a_pub = self.create_publisher(PoseStamped, "/waypoint_a", 10)
        self.waypoint_b_pub = self.create_publisher(PoseStamped, "/waypoint_b", 10)

        # Publishes all route/home/workspace markers as one MarkerArray.
        self.marker_pub = self.create_publisher(MarkerArray, "/waypoint_markers", 10)

        # Layout-derived home cells and workspace approach cells.
        self.home_cells = robot_home_cells(self.robot_count)
        self.target_workspace_ids = [table["id"] for table in generate_tables()]
        self.target_cells = [workspace_approach_cell(workspace_id) for workspace_id in self.target_workspace_ids]
        self.workspace_tables = generate_tables()

        # Periodic republish keeps RViz displays populated after restarts.
        self.timer = self.create_timer(1.0, self.on_timer)

    def _make_pose(self, x: float, y: float) -> PoseStamped:
        # Helper for map-frame PoseStamped messages with no yaw rotation.
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.w = 1.0
        return msg

    def on_timer(self) -> None:
        # waypoint_a is the first robot home; waypoint_b is the first workspace target.
        first_home_pose = cell_to_pose(self.home_cells[0])
        self.waypoint_a_pub.publish(self._make_pose(first_home_pose[0], first_home_pose[1]))
        current_goal_pose = cell_to_pose(self.target_cells[0])
        self.waypoint_b_pub.publish(self._make_pose(current_goal_pose[0], current_goal_pose[1]))
        self._publish_waypoint_markers()

    def _publish_waypoint_markers(self) -> None:
        # MarkerArray output includes home positions, workspace approach targets,
        # workstation points, and text labels for RViz.
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()

        for index, home_cell in enumerate(self.home_cells, start=1):
            # Green markers label each robot's home cell.
            home_pose = cell_to_pose(home_cell)

            home_marker = Marker()
            home_marker.header.frame_id = "map"
            home_marker.header.stamp = now
            home_marker.ns = "waypoints"
            home_marker.id = index
            home_marker.type = Marker.SPHERE
            home_marker.action = Marker.ADD
            home_marker.pose.position.x = home_pose[0]
            home_marker.pose.position.y = home_pose[1]
            home_marker.pose.orientation.w = 1.0
            home_marker.scale.x = 0.16
            home_marker.scale.y = 0.16
            home_marker.scale.z = 0.08
            home_marker.color.a = 1.0
            home_marker.color.r = 0.1
            home_marker.color.g = 0.8
            home_marker.color.b = 0.1
            markers.markers.append(home_marker)

            home_label = Marker()
            home_label.header.frame_id = "map"
            home_label.header.stamp = now
            home_label.ns = "waypoint_labels"
            home_label.id = 100 + index
            home_label.type = Marker.TEXT_VIEW_FACING
            home_label.action = Marker.ADD
            home_label.pose.position.x = home_pose[0]
            home_label.pose.position.y = home_pose[1]
            home_label.pose.position.z = 0.18
            home_label.pose.orientation.w = 1.0
            home_label.scale.z = 0.10
            home_label.color.a = 1.0
            home_label.color.r = 1.0
            home_label.color.g = 1.0
            home_label.color.b = 1.0
            home_label.text = f"robot{index}"
            markers.markers.append(home_label)

        for idx, workspace_id in enumerate(self.target_workspace_ids, start=1):
            # Red route-target markers show each workspace approach cell.
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
            # Yellow workstation markers show the generated workspace waypoint cells.
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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
