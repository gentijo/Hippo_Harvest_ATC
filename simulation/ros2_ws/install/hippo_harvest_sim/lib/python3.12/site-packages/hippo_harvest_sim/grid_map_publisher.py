import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from hippo_harvest_sim.layout_data import (
    GRID_HEIGHT_CELLS,
    GRID_RESOLUTION_M,
    GRID_WIDTH_CELLS,
    hard_occupied_cells,
)


class GridMapPublisher(Node):
    def __init__(self) -> None:
        super().__init__("grid_map_publisher")
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(OccupancyGrid, "/map", qos)
        self.timer = self.create_timer(1.0, self.publish_map)
        self.width = GRID_WIDTH_CELLS
        self.height = GRID_HEIGHT_CELLS
        self.resolution = GRID_RESOLUTION_M
        self.data = [0] * (self.width * self.height)
        for x, y in hard_occupied_cells():
            self.data[y * self.width + x] = 100

    def publish_map(self) -> None:
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.map_load_time = self.get_clock().now().to_msg()
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = self.data
        self.publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = GridMapPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
