"""RQT observability plugin for the air-traffic-control system.

ADR 0003 companion story:
the coordinator makes decisions from pose and goal state, and this panel shows
the same fleet story to a human operator. It listens to the robot feeds, turns
them into a live nearest-neighbor table, and highlights the moments where the
fleet is converging or colliding.

Tasks this plugin performs while ATC is running:
- subscribe to pose, goal, and Nav2 status topics
- compute nearest neighbors and distance trends
- flag stale feeds and collision clusters
- show the action log emitted by the centralized traffic manager

This is the observability side of UC1-UC7: it does not intervene, but it makes
the same conflict stories visible during a demo or test run.
"""

import math
import re
import uuid
from dataclasses import dataclass, field
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped
from qt_gui.plugin import Plugin
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QColor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from python_qt_binding.QtWidgets import (
    QAbstractItemView,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QLineEdit,
    QPushButton,
    QSplitter,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)
from std_msgs.msg import Bool

from air_traffic_control.telemetry import RunContextSubscriber, Telemetry


TREND_EPSILON_M = 0.005
STALE_AFTER_SEC = 2.0
COLLISION_DISTANCE_M = 0.13

COMMAND_QOS = QoSProfile(depth=1)
COMMAND_QOS.reliability = ReliabilityPolicy.RELIABLE
COMMAND_QOS.durability = DurabilityPolicy.VOLATILE

STATE_QOS = QoSProfile(depth=1)
STATE_QOS.reliability = ReliabilityPolicy.RELIABLE
STATE_QOS.durability = DurabilityPolicy.TRANSIENT_LOCAL


@dataclass
class RobotObservation:
    """Current observable state for one robot in the traffic-control panel."""

    name: str
    pose: Optional[PoseStamped] = None
    pose_time_sec: Optional[float] = None
    goal_pose: Optional[PoseStamped] = None
    goal_pose_time_sec: Optional[float] = None
    goal_name: str = ""
    goal_name_time_sec: Optional[float] = None
    active_goal_id: str = ""
    active_goal_status: str = ""
    nearest_robot: str = ""
    nearest_distance_m: Optional[float] = None
    previous_nearest_distance_m: Optional[float] = None
    distance_trend: str = "unknown"
    subscriptions: list = field(default_factory=list)


class AirTrafficControlPlugin(Plugin):
    """RQT grid for collecting and displaying ATC observability data.

    This pass is intentionally read-only. It does not pause robots or modify Nav2;
    it only collects the data needed for the first centralized traffic-control
    decisions described in the project ADRs.
    """

    def __init__(self, context) -> None:
        super().__init__(context)
        self.setObjectName("AirTrafficControlPlugin")

        # The plugin can run inside an existing ROS session or bootstrap its own
        # minimal rclpy context when launched standalone.
        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_rclpy = True
        else:
            self._owns_rclpy = False

        self.node = rclpy.create_node("air_traffic_control_rqt_plugin")
        self.run_context = RunContextSubscriber(self.node)
        self.telemetry = Telemetry("air_traffic_control", "air_traffic_control_rqt_plugin", self.node.get_logger())
        self.robots: dict[str, RobotObservation] = {}
        self.robot_count = 20
        self.robot_prefix = "robot"
        self.pose_topic_suffix = "synthetic_pose"
        self.active_collision_groups: set[tuple[str, ...]] = set()
        self.atc_enabled = True
        self.atc_enabled_topic = "/atc/control_enabled"
        self.atc_enabled_state_sub = self.node.create_subscription(
            Bool,
            "/atc/enabled",
            self._on_atc_enabled_state,
            STATE_QOS,
        )
        self.atc_enabled_pub = self.node.create_publisher(Bool, self.atc_enabled_topic, COMMAND_QOS)
        self.atc_event_sub = self.node.create_subscription(String, "/atc/events", self._on_atc_event, 10)

        # Build the dashboard first so the widget appears immediately, then
        # attach subscriptions and refresh timers.
        self.widget = QWidget()
        self.widget.setWindowTitle("Air Traffic Control Monitor")
        self._build_ui()
        context.add_widget(self.widget)

        self._configure_robot_subscriptions()

        self.spin_timer = QTimer(self.widget)
        self.spin_timer.timeout.connect(self._spin_ros_once)
        self.spin_timer.start(20)

        self.table_timer = QTimer(self.widget)
        self.table_timer.timeout.connect(self._refresh_table)
        self.table_timer.start(250)

    def _build_ui(self) -> None:
        root_layout = QVBoxLayout()
        controls_layout = QHBoxLayout()

        # --- Top control strip: tune the panel live without restarting it. ---
        controls_layout.addWidget(QLabel("Robots"))
        self.robot_count_spin = QSpinBox()
        self.robot_count_spin.setRange(1, 200)
        self.robot_count_spin.setValue(self.robot_count)
        controls_layout.addWidget(self.robot_count_spin)

        controls_layout.addWidget(QLabel("Prefix"))
        self.robot_prefix_edit = QLineEdit(self.robot_prefix)
        controls_layout.addWidget(self.robot_prefix_edit)

        controls_layout.addWidget(QLabel("Pose suffix"))
        self.pose_suffix_edit = QLineEdit(self.pose_topic_suffix)
        controls_layout.addWidget(self.pose_suffix_edit)

        self.apply_button = QPushButton("Apply")
        self.apply_button.clicked.connect(self._on_apply_clicked)
        controls_layout.addWidget(self.apply_button)

        self.clear_log_button = QPushButton("Clear Log")
        self.clear_log_button.clicked.connect(self._on_clear_log_clicked)
        controls_layout.addWidget(self.clear_log_button)

        self.atc_toggle_button = QPushButton("ATC Enabled")
        self.atc_toggle_button.setCheckable(True)
        self.atc_toggle_button.setChecked(True)
        self.atc_toggle_button.toggled.connect(self._on_atc_toggle_clicked)
        controls_layout.addWidget(self.atc_toggle_button)
        controls_layout.addStretch(1)

        self.summary_label = QLabel()
        controls_layout.addWidget(self.summary_label)
        root_layout.addLayout(controls_layout)

        # --- Main split view: robot table on top, event log underneath. ---
        self.main_splitter = QSplitter(Qt.Vertical)
        self.table = QTableWidget(0, 10)
        self.table.setHorizontalHeaderLabels(
            [
                "Robot",
                "Pose",
                "X m",
                "Y m",
                "Yaw rad",
                "Current Nav Goal",
                "Nearest",
                "Distance m",
                "Trend",
                "Age s",
            ]
        )
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setAlternatingRowColors(True)
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.main_splitter.addWidget(self.table)

        # --- Log panel: highlight collision clusters and ATC action history. ---
        log_panel = QWidget()
        log_layout = QVBoxLayout()
        log_layout.setContentsMargins(0, 0, 0, 0)
        log_layout.addWidget(QLabel("Crash / ATC action log"))
        self.collision_log = QListWidget()
        self.collision_log.setMinimumHeight(self.collision_log.fontMetrics().lineSpacing() * 3 + 30)
        self.collision_log.setAlternatingRowColors(True)
        self.collision_log.setSelectionMode(QAbstractItemView.NoSelection)
        self.collision_log.setStyleSheet("QListWidget::item { padding: 4px; border-bottom: 1px solid #d0d0d0; }")
        log_layout.addWidget(self.collision_log)
        log_panel.setLayout(log_layout)
        self.main_splitter.addWidget(log_panel)
        self.main_splitter.setStretchFactor(0, 1)
        self.main_splitter.setStretchFactor(1, 0)
        self.main_splitter.setSizes([700, self.collision_log.fontMetrics().lineSpacing() * 3 + 45])
        root_layout.addWidget(self.main_splitter)

        self.widget.setLayout(root_layout)

    def _on_apply_clicked(self) -> None:
        self.robot_count = int(self.robot_count_spin.value())
        self.robot_prefix = self.robot_prefix_edit.text().strip() or "robot"
        self.pose_topic_suffix = self.pose_suffix_edit.text().strip().strip("/") or "synthetic_pose"
        self._configure_robot_subscriptions()

    def _on_clear_log_clicked(self) -> None:
        self.collision_log.clear()
        self.active_collision_groups.clear()

    def _on_atc_toggle_clicked(self, checked: bool) -> None:
        self._publish_atc_enabled(checked)

    def _on_atc_enabled_state(self, msg: Bool) -> None:
        self.atc_enabled = bool(msg.data)
        self._sync_atc_toggle_button()

    def _publish_atc_enabled(self, enabled: bool) -> None:
        self.atc_enabled = bool(enabled)
        msg = Bool()
        msg.data = self.atc_enabled
        self.atc_enabled_pub.publish(msg)
        self._sync_atc_toggle_button()

    def _sync_atc_toggle_button(self) -> None:
        if hasattr(self, "atc_toggle_button"):
            self.atc_toggle_button.blockSignals(True)
            self.atc_toggle_button.setChecked(self.atc_enabled)
            self.atc_toggle_button.setText("ATC Enabled" if self.atc_enabled else "ATC Disabled")
            self.atc_toggle_button.blockSignals(False)

    def _configure_robot_subscriptions(self) -> None:
        # Tear down the previous subscription set before rebuilding it with the
        # current robot count / naming pattern.
        for robot in self.robots.values():
            for subscription in robot.subscriptions:
                self.node.destroy_subscription(subscription)

        self.robots.clear()
        self.active_collision_groups.clear()
        atc_goal_qos = QoSProfile(depth=1)
        atc_goal_qos.reliability = ReliabilityPolicy.RELIABLE
        atc_goal_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # UC7 observability: each robot contributes a bundle of topics that let
        # the operator read the fleet story without touching the controller.
        for index in range(1, self.robot_count + 1):
            robot_name = f"{self.robot_prefix}{index}"
            robot = RobotObservation(name=robot_name)

            # Pose input from the simulator's synthetic localization node.
            pose_topic = f"/{robot_name}/{self.pose_topic_suffix}"
            robot.subscriptions.append(
                self.node.create_subscription(
                    PoseStamped,
                    pose_topic,
                    lambda msg, name=robot_name: self._on_pose(name, msg),
                    10,
                )
            )

            # Nav2 action status provides active goal UUIDs and state, even though
            # it does not expose the original target pose. That is still enough for
            # a read-only panel to show whether a robot has work in flight.
            status_topic = f"/{robot_name}/navigate_to_pose/_action/status"
            robot.subscriptions.append(
                self.node.create_subscription(
                    GoalStatusArray,
                    status_topic,
                    lambda msg, name=robot_name: self._on_goal_status(name, msg),
                    10,
                )
            )

            # Optional goal-pose topics. These keep the panel ready for future
            # goal-publishing shims without changing the UI contract.
            robot.subscriptions.append(
                self.node.create_subscription(
                    PoseStamped,
                    f"/{robot_name}/atc/current_goal",
                    lambda msg, name=robot_name: self._on_goal_pose(name, msg),
                    atc_goal_qos,
                )
            )
            for goal_topic in (f"/{robot_name}/nav/current_goal", f"/{robot_name}/goal_pose"):
                robot.subscriptions.append(
                    self.node.create_subscription(
                        PoseStamped,
                        goal_topic,
                        lambda msg, name=robot_name: self._on_goal_pose(name, msg),
                        10,
                    )
                )

            # Human-readable goal-name topics let the table display names such as
            # `ws2` or `robot4_return_transit` instead of opaque Nav2 UUIDs.
            robot.subscriptions.append(
                self.node.create_subscription(
                    String,
                    f"/{robot_name}/atc/current_goal_name",
                    lambda msg, name=robot_name: self._on_goal_name(name, msg),
                    atc_goal_qos,
                )
            )
            robot.subscriptions.append(
                self.node.create_subscription(
                    String,
                    f"/{robot_name}/nav/current_goal_name",
                    lambda msg, name=robot_name: self._on_goal_name(name, msg),
                    10,
                )
            )

            self.robots[robot_name] = robot

        self.table.setRowCount(self.robot_count)
        self._refresh_table()

    def _spin_ros_once(self) -> None:
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def _on_pose(self, robot_name: str, msg: PoseStamped) -> None:
        robot = self.robots.get(robot_name)
        if robot is None:
            return

        robot.pose = msg
        robot.pose_time_sec = self.node.get_clock().now().nanoseconds / 1e9

    def _on_goal_pose(self, robot_name: str, msg: PoseStamped) -> None:
        robot = self.robots.get(robot_name)
        if robot is None:
            return

        robot.goal_pose = msg
        robot.goal_pose_time_sec = self.node.get_clock().now().nanoseconds / 1e9

    def _on_goal_name(self, robot_name: str, msg: String) -> None:
        robot = self.robots.get(robot_name)
        if robot is None:
            return

        robot.goal_name = msg.data.strip()
        robot.goal_name_time_sec = self.node.get_clock().now().nanoseconds / 1e9
        if not robot.goal_name:
            robot.goal_pose = None
            robot.goal_pose_time_sec = None

    def _on_goal_status(self, robot_name: str, msg: GoalStatusArray) -> None:
        robot = self.robots.get(robot_name)
        if robot is None:
            return

        active_statuses = [
            status for status in msg.status_list if status.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
        ]
        if not active_statuses:
            robot.active_goal_id = ""
            robot.active_goal_status = ""
            return

        latest = active_statuses[-1]
        robot.active_goal_id = self._format_goal_id(latest.goal_info.goal_id.uuid)
        robot.active_goal_status = self._status_label(latest.status)

    def _on_atc_event(self, msg: String) -> None:
        robot_id = self._first_robot_mentioned(msg.data)
        traceparent = self.run_context.robot_traceparent(robot_id)
        with self.telemetry.start_as_current_span(
            "atc.rqt_event_observed",
            run_id=self.run_context.run_id,
            robot_id=robot_id,
            traceparent=traceparent,
            event_text=msg.data,
        ) as span:
            updated_traceparent = Telemetry.traceparent_from_span(span)
            if updated_traceparent:
                self.run_context.update_robot_traceparent(robot_id, updated_traceparent)
                traceparent = updated_traceparent
            self.telemetry.log(
                msg.data,
                run_id=self.run_context.run_id,
                robot_id=robot_id,
                traceparent=traceparent,
                event_text=msg.data,
            )
        self._append_log_text(msg.data)

    def _update_nearest_neighbors(self) -> None:
        # The table mirrors the ADR's nearest-neighbor view so the operator can
        # see which robots are converging before the traffic manager intervenes.
        robot_items = list(self.robots.items())
        for robot_name, robot in robot_items:
            nearest_name = ""
            nearest_distance = None

            if robot.pose is not None:
                x = robot.pose.pose.position.x
                y = robot.pose.pose.position.y
                for other_name, other in robot_items:
                    if other_name == robot_name or other.pose is None:
                        continue

                    dx = other.pose.pose.position.x - x
                    dy = other.pose.pose.position.y - y
                    distance = math.hypot(dx, dy)
                    if nearest_distance is None or distance < nearest_distance:
                        nearest_distance = distance
                        nearest_name = other_name

            robot.nearest_robot = nearest_name
            robot.previous_nearest_distance_m = robot.nearest_distance_m
            robot.nearest_distance_m = nearest_distance
            robot.distance_trend = self._distance_trend(
                robot.previous_nearest_distance_m,
                robot.nearest_distance_m,
            )

    def _refresh_table(self) -> None:
        self._update_nearest_neighbors()
        self._update_collision_log()
        now_sec = self.node.get_clock().now().nanoseconds / 1e9
        active_pose_count = 0

        # Render the ATC story row-by-row: pose freshness, goal state, closest
        # peer, and whether the gap is shrinking, widening, or holding steady.
        for row, robot_name in enumerate(sorted(self.robots.keys(), key=self._robot_sort_key)):
            robot = self.robots[robot_name]
            pose_age = None if robot.pose_time_sec is None else now_sec - robot.pose_time_sec
            pose_fresh = pose_age is not None and pose_age <= STALE_AFTER_SEC
            if pose_fresh:
                active_pose_count += 1

            yaw = self._yaw_from_pose(robot.pose) if robot.pose is not None else None
            values = [
                robot.name,
                "fresh" if pose_fresh else "missing" if robot.pose is None else "stale",
                self._format_float(robot.pose.pose.position.x if robot.pose is not None else None),
                self._format_float(robot.pose.pose.position.y if robot.pose is not None else None),
                self._format_float(yaw),
                self._goal_text(robot),
                robot.nearest_robot or "-",
                self._format_float(robot.nearest_distance_m),
                robot.distance_trend,
                self._format_float(pose_age),
            ]

            for column, value in enumerate(values):
                item = self.table.item(row, column)
                if item is None:
                    item = QTableWidgetItem()
                    self.table.setItem(row, column, item)
                item.setText(value)
                item.setTextAlignment(Qt.AlignCenter)
                self._style_item(item, robot, column, pose_fresh)

        self.summary_label.setText(f"Pose feeds: {active_pose_count}/{self.robot_count}")

    def _goal_text(self, robot: RobotObservation) -> str:
        if robot.goal_name:
            return robot.goal_name
        if robot.goal_pose is not None:
            x = robot.goal_pose.pose.position.x
            y = robot.goal_pose.pose.position.y
            return f"pose ({x:.2f}, {y:.2f})"
        if robot.active_goal_id:
            return f"{robot.active_goal_status} {robot.active_goal_id}"
        return "-"

    def _style_item(self, item: QTableWidgetItem, robot: RobotObservation, column: int, pose_fresh: bool) -> None:
        item.setBackground(QColor("white"))
        if column == 1 and not pose_fresh:
            item.setBackground(QColor(255, 235, 235))
        elif column == 8 and robot.distance_trend == "decreasing":
            item.setBackground(QColor(255, 245, 200))
        elif column == 8 and robot.distance_trend == "increasing":
            item.setBackground(QColor(225, 245, 225))

    def _update_collision_log(self) -> None:
        # UC1-UC5: the log only grows when a new connected collision cluster
        # appears, so the operator can read each new traffic incident once.
        collision_groups = self._collision_groups()
        current_groups = {tuple(group) for group in collision_groups}

        for group in collision_groups:
            group_key = tuple(group)
            if group_key in self.active_collision_groups:
                continue
            self._append_collision_log(group)

        self.active_collision_groups = current_groups

    def _collision_groups(self) -> list[list[str]]:
        # Build an undirected graph of collisions and collapse it into connected
        # components so multi-robot incidents are reported as a single event.
        robot_names = sorted(self.robots.keys(), key=self._robot_sort_key)
        colliding_pairs = []

        for left_index, left_name in enumerate(robot_names):
            left_robot = self.robots[left_name]
            if left_robot.pose is None:
                continue

            for right_name in robot_names[left_index + 1 :]:
                right_robot = self.robots[right_name]
                if right_robot.pose is None:
                    continue

                distance = self._pose_distance(left_robot.pose, right_robot.pose)
                if distance <= COLLISION_DISTANCE_M:
                    colliding_pairs.append((left_name, right_name))

        return self._connected_collision_components(colliding_pairs)

    @staticmethod
    def _connected_collision_components(pairs: list[tuple[str, str]]) -> list[list[str]]:
        adjacency: dict[str, set[str]] = {}
        for left, right in pairs:
            adjacency.setdefault(left, set()).add(right)
            adjacency.setdefault(right, set()).add(left)

        groups = []
        visited = set()
        for robot_name in sorted(adjacency.keys()):
            if robot_name in visited:
                continue

            stack = [robot_name]
            group = []
            visited.add(robot_name)
            while stack:
                current = stack.pop()
                group.append(current)
                for neighbor in adjacency[current]:
                    if neighbor not in visited:
                        visited.add(neighbor)
                        stack.append(neighbor)
            groups.append(sorted(group))

        return groups

    def _append_collision_log(self, robot_names: list[str]) -> None:
        robots = [self.robots[name] for name in robot_names if self.robots[name].pose is not None]
        if not robots:
            return

        crash_x = sum(robot.pose.pose.position.x for robot in robots if robot.pose is not None) / len(robots)
        crash_y = sum(robot.pose.pose.position.y for robot in robots if robot.pose is not None) / len(robots)
        min_distance = self._minimum_pair_distance(robots)
        robot_details = ", ".join(f"{robot.name} goal={self._goal_text(robot)}" for robot in robots)
        message = (
            f"Collision detected: robots [{', '.join(robot.name for robot in robots)}], "
            f"closest separation {min_distance:.3f} m, crash location ({crash_x:.3f}, {crash_y:.3f}), "
            f"goals: {robot_details}"
        )

        self._append_log_text(message)

        robot_id = robots[0].name if robots else ""
        traceparent = self.run_context.robot_traceparent(robot_id)
        with self.telemetry.start_as_current_span(
            "atc.rqt_collision_observed",
            run_id=self.run_context.run_id,
            robot_id=robot_id,
            traceparent=traceparent,
            crash_x=crash_x,
            crash_y=crash_y,
            min_distance_m=min_distance,
        ) as span:
            updated_traceparent = Telemetry.traceparent_from_span(span)
            if updated_traceparent:
                self.run_context.update_robot_traceparent(robot_id, updated_traceparent)
                traceparent = updated_traceparent
            self.telemetry.log(
                message,
                run_id=self.run_context.run_id,
                robot_id=robot_id,
                traceparent=traceparent,
                min_distance_m=min_distance,
            )

    def _append_log_text(self, message: str) -> None:
        item = QListWidgetItem(message)
        self.collision_log.addItem(item)
        self.collision_log.scrollToBottom()

    @staticmethod
    def _minimum_pair_distance(robots: list[RobotObservation]) -> float:
        minimum = math.inf
        for left_index, left_robot in enumerate(robots):
            for right_robot in robots[left_index + 1 :]:
                if left_robot.pose is None or right_robot.pose is None:
                    continue
                minimum = min(minimum, AirTrafficControlPlugin._pose_distance(left_robot.pose, right_robot.pose))
        return minimum if math.isfinite(minimum) else 0.0

    @staticmethod
    def _pose_distance(left: PoseStamped, right: PoseStamped) -> float:
        dx = right.pose.position.x - left.pose.position.x
        dy = right.pose.position.y - left.pose.position.y
        return math.hypot(dx, dy)

    @staticmethod
    def _distance_trend(previous: Optional[float], current: Optional[float]) -> str:
        if previous is None or current is None:
            return "unknown"
        delta = current - previous
        if delta > TREND_EPSILON_M:
            return "increasing"
        if delta < -TREND_EPSILON_M:
            return "decreasing"
        return "stable"

    @staticmethod
    def _yaw_from_pose(msg: Optional[PoseStamped]) -> Optional[float]:
        if msg is None:
            return None
        z = msg.pose.orientation.z
        w = msg.pose.orientation.w
        return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)

    @staticmethod
    def _format_float(value: Optional[float]) -> str:
        if value is None:
            return "-"
        return f"{value:.3f}"

    @staticmethod
    def _format_goal_id(goal_uuid) -> str:
        raw = bytes(goal_uuid)
        if len(raw) != 16:
            return "-"
        return str(uuid.UUID(bytes=raw))[:8]

    @staticmethod
    def _status_label(status: int) -> str:
        labels = {
            GoalStatus.STATUS_ACCEPTED: "accepted",
            GoalStatus.STATUS_EXECUTING: "executing",
            GoalStatus.STATUS_CANCELING: "canceling",
            GoalStatus.STATUS_SUCCEEDED: "succeeded",
            GoalStatus.STATUS_CANCELED: "canceled",
            GoalStatus.STATUS_ABORTED: "aborted",
        }
        return labels.get(status, str(status))

    @staticmethod
    def _robot_sort_key(name: str):
        prefix = name.rstrip("0123456789")
        suffix = name[len(prefix) :]
        if suffix.isdigit():
            return (prefix, int(suffix))
        return (name, 0)

    def _first_robot_mentioned(self, text: str) -> str:
        for robot_name in sorted(self.robots.keys(), key=self._robot_sort_key):
            if re.search(rf"(?<![A-Za-z0-9_]){re.escape(robot_name)}(?![A-Za-z0-9_])", text):
                return robot_name
        return ""

    def shutdown_plugin(self) -> None:
        self.spin_timer.stop()
        self.table_timer.stop()
        for robot in self.robots.values():
            for subscription in robot.subscriptions:
                self.node.destroy_subscription(subscription)
        self.node.destroy_subscription(self.atc_enabled_state_sub)
        self.node.destroy_subscription(self.atc_event_sub)
        self.robots.clear()
        self.node.destroy_node()
        if self._owns_rclpy and rclpy.ok():
            rclpy.shutdown()

    def save_settings(self, plugin_settings, instance_settings) -> None:
        instance_settings.set_value("robot_count", self.robot_count_spin.value())
        instance_settings.set_value("robot_prefix", self.robot_prefix_edit.text())
        instance_settings.set_value("pose_topic_suffix", self.pose_suffix_edit.text())

    def restore_settings(self, plugin_settings, instance_settings) -> None:
        robot_count = instance_settings.value("robot_count", self.robot_count)
        robot_prefix = instance_settings.value("robot_prefix", self.robot_prefix)
        pose_suffix = instance_settings.value("pose_topic_suffix", self.pose_topic_suffix)

        self.robot_count_spin.setValue(int(robot_count))
        self.robot_prefix_edit.setText(str(robot_prefix))
        self.pose_suffix_edit.setText(str(pose_suffix))
        self._on_apply_clicked()
