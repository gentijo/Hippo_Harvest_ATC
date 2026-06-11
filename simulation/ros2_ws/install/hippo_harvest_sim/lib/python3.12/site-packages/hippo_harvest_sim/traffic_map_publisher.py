import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from hippo_harvest_sim.telemetry import RunContextSubscriber, Telemetry


@dataclass
class RobotPose:
    pose: PoseStamped
    received_sec: float


class TrafficMapPublisher(Node):
    """Publishes robot-occupied costmap layers for Nav2.

    This node belongs to the simulator, not ATC. It lets robots plan around
    currently occupied robot footprints even when the ATC guard process is not
    running. ATC can still pause/resume robots independently.
    """

    def __init__(self) -> None:
        super().__init__("traffic_map_publisher")

        self.declare_parameter("robot_count", 10)
        self.declare_parameter("robot_prefix", "robot")
        self.declare_parameter("pose_topic_suffix", "synthetic_pose")
        self.declare_parameter("publish_period_sec", 0.2)
        self.declare_parameter("stale_pose_sec", 2.0)
        self.declare_parameter("dynamic_obstacle_radius_m", 0.09)
        self.declare_parameter("diagnostic_period_sec", 5.0)
        self.declare_parameter("grid_resolution_m", 0.025)
        self.declare_parameter("grid_width_cells", 180)
        self.declare_parameter("grid_height_cells", 135)

        self.robot_count = int(self.get_parameter("robot_count").value)
        self.robot_prefix = str(self.get_parameter("robot_prefix").value)
        self.pose_topic_suffix = str(self.get_parameter("pose_topic_suffix").value).strip("/")
        publish_period_sec = float(self.get_parameter("publish_period_sec").value)
        self.stale_pose_sec = float(self.get_parameter("stale_pose_sec").value)
        self.dynamic_obstacle_radius_m = float(self.get_parameter("dynamic_obstacle_radius_m").value)
        self.diagnostic_period_sec = float(self.get_parameter("diagnostic_period_sec").value)
        self.grid_resolution_m = float(self.get_parameter("grid_resolution_m").value)
        self.grid_width_cells = int(self.get_parameter("grid_width_cells").value)
        self.grid_height_cells = int(self.get_parameter("grid_height_cells").value)
        self.run_context = RunContextSubscriber(self)
        self.telemetry = Telemetry("hippo_harvest_sim", "traffic_map_publisher", self.get_logger())

        traffic_map_qos = QoSProfile(depth=1)
        traffic_map_qos.reliability = ReliabilityPolicy.RELIABLE
        traffic_map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.poses: dict[str, RobotPose] = {}
        self.robot_names = [f"{self.robot_prefix}{index}" for index in range(1, self.robot_count + 1)]
        self.robot_publishers = {
            robot_name: self.create_publisher(OccupancyGrid, f"/{robot_name}/atc/traffic_map", traffic_map_qos)
            for robot_name in self.robot_names
        }
        self.fleet_publisher = self.create_publisher(OccupancyGrid, "/atc/traffic_map", traffic_map_qos)
        self.next_diagnostic_snapshot_sec = 0.0
        self.last_obstacle_cell_count = 0

        for robot_name in self.robot_names:
            self.create_subscription(
                PoseStamped,
                f"/{robot_name}/{self.pose_topic_suffix}",
                lambda msg, name=robot_name: self._on_pose(name, msg),
                10,
            )

        self.timer = self.create_timer(publish_period_sec, self._publish_maps)
        self.telemetry.log(
            "Publishing simulator traffic maps",
            robot_count=self.robot_count,
            obstacle_radius_m=self.dynamic_obstacle_radius_m,
            grid_resolution_m=self.grid_resolution_m,
            grid_width_cells=self.grid_width_cells,
            grid_height_cells=self.grid_height_cells,
        )

    def _on_pose(self, robot_name: str, msg: PoseStamped) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        self.poses[robot_name] = RobotPose(pose=msg, received_sec=now_sec)

    def _publish_maps(self) -> None:
        fresh_names = [robot_name for robot_name in self.robot_names if self._pose_is_fresh(robot_name)]
        for robot_name in self.robot_names:
            obstacle_names = [name for name in fresh_names if name != robot_name]
            self.robot_publishers[robot_name].publish(self._make_traffic_map(obstacle_names))
        fleet_map = self._make_traffic_map(fresh_names)
        self.last_obstacle_cell_count = sum(1 for cell in fleet_map.data if cell > 0)
        self.fleet_publisher.publish(fleet_map)
        self._publish_diagnostic_snapshot_if_due(fresh_names)

    def _make_traffic_map(self, obstacle_names: list[str]) -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.map_load_time = msg.header.stamp
        msg.info.resolution = self.grid_resolution_m
        msg.info.width = self.grid_width_cells
        msg.info.height = self.grid_height_cells
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.orientation.w = 1.0

        data = [0] * (self.grid_width_cells * self.grid_height_cells)
        radius_cells = max(1, int(math.ceil(self.dynamic_obstacle_radius_m / self.grid_resolution_m)))
        for robot_name in obstacle_names:
            robot_pose = self.poses.get(robot_name)
            if robot_pose is None:
                continue

            pose = robot_pose.pose.pose
            center_x = int(math.floor(pose.position.x / self.grid_resolution_m))
            center_y = int(math.floor(pose.position.y / self.grid_resolution_m))
            for y in range(max(0, center_y - radius_cells), min(self.grid_height_cells, center_y + radius_cells + 1)):
                for x in range(max(0, center_x - radius_cells), min(self.grid_width_cells, center_x + radius_cells + 1)):
                    cell_x_m = (x + 0.5) * self.grid_resolution_m
                    cell_y_m = (y + 0.5) * self.grid_resolution_m
                    dx = cell_x_m - pose.position.x
                    dy = cell_y_m - pose.position.y
                    if math.hypot(dx, dy) <= self.dynamic_obstacle_radius_m:
                        data[y * self.grid_width_cells + x] = 100

        msg.data = data
        return msg

    def _pose_is_fresh(self, robot_name: str) -> bool:
        robot_pose = self.poses.get(robot_name)
        if robot_pose is None:
            return False
        now_sec = self.get_clock().now().nanoseconds / 1e9
        return now_sec - robot_pose.received_sec <= self.stale_pose_sec

    def _publish_diagnostic_snapshot_if_due(self, fresh_names: list[str]) -> None:
        if self.diagnostic_period_sec <= 0.0:
            return
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec < self.next_diagnostic_snapshot_sec:
            return
        self.next_diagnostic_snapshot_sec = now_sec + self.diagnostic_period_sec

        stale_names = [name for name in self.robot_names if name not in fresh_names]
        max_pose_age_sec = 0.0
        for robot_pose in self.poses.values():
            max_pose_age_sec = max(max_pose_age_sec, now_sec - robot_pose.received_sec)

        self.telemetry.log(
            "Traffic map diagnostic snapshot",
            run_id=self.run_context.run_id,
            fresh_robot_count=len(fresh_names),
            stale_robot_count=len(stale_names),
            stale_robots=",".join(stale_names),
            obstacle_cell_count=self.last_obstacle_cell_count,
            obstacle_radius_m=self.dynamic_obstacle_radius_m,
            max_pose_age_sec=max_pose_age_sec,
        )


def main(args=None):
    rclpy.init(args=args)
    node = TrafficMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
