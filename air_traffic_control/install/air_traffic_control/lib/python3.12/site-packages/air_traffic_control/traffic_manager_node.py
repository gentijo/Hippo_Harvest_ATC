import json
import math
import os
import re
from dataclasses import dataclass, field
from typing import Optional

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

from air_traffic_control.telemetry import RunContextSubscriber, Telemetry


COMMAND_QOS = QoSProfile(depth=1)
COMMAND_QOS.reliability = ReliabilityPolicy.RELIABLE
COMMAND_QOS.durability = DurabilityPolicy.VOLATILE

STATE_QOS = QoSProfile(depth=1)
STATE_QOS.reliability = ReliabilityPolicy.RELIABLE
STATE_QOS.durability = DurabilityPolicy.TRANSIENT_LOCAL


@dataclass
class RobotState:
    name: str
    pose: Optional[PoseStamped] = None
    goal_pose: Optional[PoseStamped] = None
    goal_name: str = ""
    paused: bool = False
    paused_since_sec: Optional[float] = None
    yielding_to: set[str] = field(default_factory=set)


class CentralizedTrafficManager(Node):
    """Centralized local conflict supervisor for the simulator robots.

    ADR 0003 use-case story:
    A robot enters a shared aisle or intersection, ATC watches the fleet state,
    notices the gap closing, chooses one robot to yield, marks that robot as a
    protected occupied zone, and then releases it once the path is safe again.

    Tasks this node performs while ATC is running:
    - subscribe to robot pose and goal topics
    - measure pairwise separation and nearest-neighbor trends
    - select a deterministic yielder when conflict risk rises
    - publish pause state and protected-zone markers
    - unwind yield cycles and deadlocks so the fleet can keep moving

    ADR 0003 coverage notes:
    - UC1 head-on aisle conflict: direct implementation
    - UC2 intersection merge conflict: direct implementation through the same
      closing-distance and goal-area arbitration
    - UC3 rear-approach / overtaking conflict: direct implementation through the
      protected-zone machinery
    - UC4 stopped robot near a table or workstation: represented by the paused
      robot as a temporary obstacle in the traffic map
    - UC5 congestion near home / dispatch area: direct implementation through
      cycle breaking and deadlock release
    - UC6 short backing maneuver with limited lidar coverage: represented by the
      same protected-zone and map-driven shielding logic
    - UC7 visually ambiguous sensing conditions: direct implementation because
      the coordinator reasons over pose and goal state instead of lidar returns
    """

    def __init__(self) -> None:
        super().__init__("centralized_traffic_manager")

        self.declare_parameter("robot_count", 10)
        self.declare_parameter("robot_prefix", "robot")
        self.declare_parameter("pose_topic_suffix", "synthetic_pose")
        self.declare_parameter("goal_pose_topic_suffix", "atc/current_goal")
        self.declare_parameter("goal_name_topic_suffix", "atc/current_goal_name")
        self.declare_parameter("pause_topic_suffix", "atc/pause")
        self.declare_parameter("atc_enabled_command_topic", "/atc/control_enabled")
        self.declare_parameter("atc_enabled_state_topic", "/atc/enabled")
        self.declare_parameter("check_period_sec", 0.2)
        self.declare_parameter("safety_distance_m", 0.35)
        self.declare_parameter("clear_distance_m", 0.55)
        self.declare_parameter("closing_epsilon_m", 0.005)
        self.declare_parameter("protected_zone_radius_m", 0.09)
        self.declare_parameter("protected_zone_enforcement_margin_m", 0.06)
        self.declare_parameter("dynamic_obstacle_radius_m", 0.09)
        self.declare_parameter("publish_traffic_map", False)
        self.declare_parameter("max_pause_sec", 8.0)
        self.declare_parameter("deadlock_release_sec", 3.0)
        self.declare_parameter("stale_pose_sec", 2.0)
        self.declare_parameter("diagnostic_period_sec", 5.0)
        self.declare_parameter("grid_resolution_m", 0.025)
        self.declare_parameter("grid_width_cells", 180)
        self.declare_parameter("grid_height_cells", 135)
        self.declare_parameter("diagnostic_log_path", "/tmp/air_traffic_control/atc_diagnostics.jsonl")

        self.robot_count = int(self.get_parameter("robot_count").value)
        self.robot_prefix = str(self.get_parameter("robot_prefix").value)
        self.pose_topic_suffix = str(self.get_parameter("pose_topic_suffix").value).strip("/")
        self.goal_pose_topic_suffix = str(self.get_parameter("goal_pose_topic_suffix").value).strip("/")
        self.goal_name_topic_suffix = str(self.get_parameter("goal_name_topic_suffix").value).strip("/")
        self.pause_topic_suffix = str(self.get_parameter("pause_topic_suffix").value).strip("/")
        self.atc_enabled_command_topic = str(self.get_parameter("atc_enabled_command_topic").value).strip()
        self.atc_enabled_state_topic = str(self.get_parameter("atc_enabled_state_topic").value).strip()
        check_period_sec = float(self.get_parameter("check_period_sec").value)
        self.safety_distance_m = float(self.get_parameter("safety_distance_m").value)
        self.clear_distance_m = float(self.get_parameter("clear_distance_m").value)
        self.closing_epsilon_m = float(self.get_parameter("closing_epsilon_m").value)
        self.protected_zone_radius_m = float(self.get_parameter("protected_zone_radius_m").value)
        self.protected_zone_enforcement_margin_m = float(
            self.get_parameter("protected_zone_enforcement_margin_m").value
        )
        self.dynamic_obstacle_radius_m = float(self.get_parameter("dynamic_obstacle_radius_m").value)
        self.publish_traffic_map = bool(self.get_parameter("publish_traffic_map").value)
        self.max_pause_sec = float(self.get_parameter("max_pause_sec").value)
        self.deadlock_release_sec = float(self.get_parameter("deadlock_release_sec").value)
        self.stale_pose_sec = float(self.get_parameter("stale_pose_sec").value)
        self.diagnostic_period_sec = float(self.get_parameter("diagnostic_period_sec").value)
        self.grid_resolution_m = float(self.get_parameter("grid_resolution_m").value)
        self.grid_width_cells = int(self.get_parameter("grid_width_cells").value)
        self.grid_height_cells = int(self.get_parameter("grid_height_cells").value)
        self.diagnostic_log_path = str(self.get_parameter("diagnostic_log_path").value)
        self.run_context = RunContextSubscriber(self)
        self.telemetry = Telemetry("air_traffic_control", "centralized_traffic_manager", self.get_logger())

        self.robots: dict[str, RobotState] = {}
        self.pause_publishers = {}
        self.traffic_map_publishers = {}
        self.pose_times = {}
        self.previous_pair_distances: dict[tuple[str, str], float] = {}
        self.active_yields: dict[str, set[str]] = {}
        self.reported_yield_cycles: set[tuple[str, ...]] = set()
        self.deadlock_release_robot: Optional[str] = None
        self.deadlock_release_until_sec = 0.0
        self.reported_deadlock_release_events: set[tuple[str, str, str]] = set()
        self.last_evaluation_summary = {}
        self.next_diagnostic_snapshot_sec = 0.0
        self.atc_enabled = True
        self.marker_pub = self.create_publisher(MarkerArray, "/atc/protected_zones", 10)
        self.event_pub = self.create_publisher(String, "/atc/events", 10)
        self.atc_enabled_pub = self.create_publisher(Bool, self.atc_enabled_state_topic, STATE_QOS)
        self.atc_enabled_sub = self.create_subscription(
            Bool,
            self.atc_enabled_command_topic,
            self._on_atc_enabled_command,
            COMMAND_QOS,
        )
        traffic_map_qos = QoSProfile(depth=1)
        traffic_map_qos.reliability = ReliabilityPolicy.RELIABLE
        traffic_map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.traffic_map_pub = (
            self.create_publisher(OccupancyGrid, "/atc/traffic_map", traffic_map_qos)
            if self.publish_traffic_map
            else None
        )
        self._initialize_diagnostic_log()
        self._publish_atc_enabled_state()

        for index in range(1, self.robot_count + 1):
            robot_name = f"{self.robot_prefix}{index}"
            self.robots[robot_name] = RobotState(name=robot_name)
            self.pause_publishers[robot_name] = self.create_publisher(
                Bool,
                f"/{robot_name}/{self.pause_topic_suffix}",
                10,
            )
            if self.publish_traffic_map:
                self.traffic_map_publishers[robot_name] = self.create_publisher(
                    OccupancyGrid,
                    f"/{robot_name}/atc/traffic_map",
                    traffic_map_qos,
                )
            self.create_subscription(
                PoseStamped,
                f"/{robot_name}/{self.pose_topic_suffix}",
                lambda msg, name=robot_name: self._on_pose(name, msg),
                10,
            )
            self.create_subscription(
                PoseStamped,
                f"/{robot_name}/{self.goal_pose_topic_suffix}",
                lambda msg, name=robot_name: self._on_goal_pose(name, msg),
                10,
            )
            self.create_subscription(
                String,
                f"/{robot_name}/{self.goal_name_topic_suffix}",
                lambda msg, name=robot_name: self._on_goal_name(name, msg),
                10,
            )

        self.timer = self.create_timer(check_period_sec, self._tick)
        self._log(
            "info",
            f"Centralized traffic manager monitoring {self.robot_count} robots; "
            f"pause below {self.safety_distance_m:.2f} m, resume above {self.clear_distance_m:.2f} m"
        )

    def _on_pose(self, robot_name: str, msg: PoseStamped) -> None:
        self.robots[robot_name].pose = msg
        self.pose_times[robot_name] = self.get_clock().now().nanoseconds / 1e9

    def _on_goal_pose(self, robot_name: str, msg: PoseStamped) -> None:
        self.robots[robot_name].goal_pose = msg

    def _on_goal_name(self, robot_name: str, msg: String) -> None:
        self.robots[robot_name].goal_name = msg.data.strip()
        if not self.robots[robot_name].goal_name:
            self.robots[robot_name].goal_pose = None

    def _tick(self) -> None:
        self._publish_atc_enabled_state()
        if not self.atc_enabled:
            self._publish_monitor_only_outputs()
            self._publish_diagnostic_snapshot_if_due()
            return
        self._evaluate_conflicts()
        self._publish_pause_states()
        self._publish_protected_zones()
        if self.publish_traffic_map:
            self._publish_traffic_map()
        self._publish_diagnostic_snapshot_if_due()

    def _on_atc_enabled_command(self, msg: Bool) -> None:
        self._set_atc_enabled(bool(msg.data))

    def _set_atc_enabled(self, enabled: bool) -> None:
        if self.atc_enabled == enabled:
            return

        self.atc_enabled = enabled
        self.previous_pair_distances = {}
        self.deadlock_release_robot = None
        self.deadlock_release_until_sec = 0.0
        self.reported_yield_cycles.clear()
        self.reported_deadlock_release_events.clear()

        if not enabled:
            self._clear_all_action_state()
            self._publish_event(
                "ATC disabled: monitor-only mode active; pause commands and protected zones cleared",
                event_type="atc_disabled",
            )
        else:
            self._publish_event(
                "ATC enabled: corrective action restored",
                event_type="atc_enabled",
            )

        self._publish_atc_enabled_state()

    def _publish_atc_enabled_state(self) -> None:
        msg = Bool()
        msg.data = self.atc_enabled
        self.atc_enabled_pub.publish(msg)

    def _clear_all_action_state(self) -> None:
        for robot in self.robots.values():
            robot.paused = False
            robot.paused_since_sec = None
            robot.yielding_to.clear()
        self.active_yields = {}
        self._publish_pause_states()
        self._publish_protected_zones()
        if self.publish_traffic_map:
            self._publish_clear_traffic_map()

    def _publish_monitor_only_outputs(self) -> None:
        self._publish_pause_states()
        self._publish_protected_zones()
        if self.publish_traffic_map:
            self._publish_clear_traffic_map()

    def _evaluate_conflicts(self) -> None:
        # UC1 and UC2: watch all fresh robot pairs, find closing motion, and
        # decide whether the pair is headed into a corridor or intersection
        # conflict that needs one robot to yield.
        fresh_names = [name for name in sorted(self.robots.keys(), key=self._robot_sort_key) if self._pose_is_fresh(name)]
        desired_yields: dict[str, set[str]] = {}
        current_distances = {}
        pair_count = 0
        near_pair_count = 0
        closing_pair_count = 0
        preemptive_pair_count = 0
        still_needed_pair_count = 0
        deadlock_release_pair_count = 0
        now_sec = self.get_clock().now().nanoseconds / 1e9
        self._expire_deadlock_release(now_sec)

        for left_index, left_name in enumerate(fresh_names):
            for right_name in fresh_names[left_index + 1 :]:
                pair_count += 1
                pair = (left_name, right_name)
                distance = self._distance_between(left_name, right_name)
                current_distances[pair] = distance
                if distance <= self.clear_distance_m:
                    near_pair_count += 1

                if self._yield_still_needed(pair, distance):
                    still_needed_pair_count += 1
                    yielder = self._existing_yielder(pair)
                    if yielder is not None:
                        other = self._other_robot(pair, yielder)
                        if self._pause_duration_sec(yielder) >= self.max_pause_sec:
                            deadlock_release_pair_count += 1
                            released_robot = self._deadlock_release_robot(yielder, other, now_sec)
                            if released_robot is None:
                                desired_yields.setdefault(yielder, set()).add(other)
                                continue
                            blocked_robot = self._other_robot(pair, released_robot)
                            desired_yields.setdefault(blocked_robot, set()).add(released_robot)
                            event_key = (released_robot, blocked_robot, yielder)
                            if event_key not in self.reported_deadlock_release_events:
                                self.reported_deadlock_release_events.add(event_key)
                                self._publish_event(
                                    f"ATC deadlock watchdog: {yielder} has been paused for "
                                    f"{self._pause_duration_sec(yielder):.1f}s while near {other}; "
                                    f"granting release priority to {released_robot} so {blocked_robot} pauses "
                                    f"and {released_robot} is retasked to {self._goal_label(released_robot)}",
                                    robot_id=blocked_robot,
                                    event_type="deadlock_watchdog",
                                    related_robot_id=released_robot,
                                    deadlock_release_robot_id=released_robot,
                                    deadlock_release_until_sec=self.deadlock_release_until_sec,
                                )
                        else:
                            desired_yields.setdefault(yielder, set()).add(other)
                    continue

                previous = self.previous_pair_distances.get(pair)
                closing = previous is not None and distance < previous - self.closing_epsilon_m
                if closing:
                    closing_pair_count += 1
                if distance <= self.safety_distance_m and (closing or self._approaching_same_goal_area(left_name, right_name)):
                    preemptive_pair_count += 1
                    yielder = self._select_yielder(left_name, right_name)
                    proceeds = right_name if yielder == left_name else left_name
                    desired_yields.setdefault(yielder, set()).add(proceeds)
                    if yielder not in self.active_yields or proceeds not in self.active_yields[yielder]:
                        location = self._midpoint(left_name, right_name)
                        self._publish_event(
                            f"ATC preemptive action: {left_name} and {right_name} separation {distance:.3f} m "
                            f"near ({location[0]:.3f}, {location[1]:.3f}); "
                            f"{yielder} pauses for {proceeds}; protected occupied ring radius "
                            f"{self.protected_zone_radius_m:.2f} m added around {yielder}; "
                            f"goals: {left_name}={self._goal_label(left_name)}, {right_name}={self._goal_label(right_name)}",
                            robot_id=yielder,
                            event_type="preemptive_pause",
                            related_robot_id=proceeds,
                            separation_m=distance,
                        )

        self._add_protected_zone_yields(desired_yields)
        self._apply_deadlock_release_priority(desired_yields, fresh_names)
        self._break_yield_cycles(desired_yields)
        self.last_evaluation_summary = {
            "fresh_robot_count": len(fresh_names),
            "pair_count": pair_count,
            "near_pair_count": near_pair_count,
            "closing_pair_count": closing_pair_count,
            "preemptive_pair_count": preemptive_pair_count,
            "still_needed_pair_count": still_needed_pair_count,
            "deadlock_release_pair_count": deadlock_release_pair_count,
            "desired_yield_count": sum(len(yields) for yields in desired_yields.values()),
            "desired_paused_robot_count": len(desired_yields),
        }
        self.previous_pair_distances = current_distances
        self._apply_yields(desired_yields)

    def _expire_deadlock_release(self, now_sec: float) -> None:
        if self.deadlock_release_robot is not None and now_sec >= self.deadlock_release_until_sec:
            self.deadlock_release_robot = None
            self.deadlock_release_until_sec = 0.0
            self.reported_deadlock_release_events.clear()

    def _deadlock_release_robot(self, yielder: str, other: str, now_sec: float) -> Optional[str]:
        if self.deadlock_release_robot is None:
            self.deadlock_release_robot = yielder
            self.deadlock_release_until_sec = now_sec + self.deadlock_release_sec
        if self.deadlock_release_robot in (yielder, other):
            return self.deadlock_release_robot
        return None

    def _apply_deadlock_release_priority(self, desired_yields: dict[str, set[str]], fresh_names: list[str]) -> None:
        released_robot = self.deadlock_release_robot
        if released_robot is None or released_robot not in fresh_names:
            return

        desired_yields.pop(released_robot, None)
        for blockers in desired_yields.values():
            blockers.discard(released_robot)
        for robot_name in [name for name, blockers in desired_yields.items() if not blockers]:
            desired_yields.pop(robot_name, None)

        for robot_name in fresh_names:
            if robot_name == released_robot:
                continue
            distance = self._distance_between(released_robot, robot_name)
            if distance <= self.clear_distance_m:
                desired_yields.setdefault(robot_name, set()).add(released_robot)

    def _yield_still_needed(self, pair: tuple[str, str], distance: float) -> bool:
        if distance >= self.clear_distance_m:
            return False
        return self._existing_yielder(pair) is not None

    def _existing_yielder(self, pair: tuple[str, str]) -> Optional[str]:
        left, right = pair
        if right in self.active_yields.get(left, set()):
            return left
        if left in self.active_yields.get(right, set()):
            return right
        return None

    @staticmethod
    def _other_robot(pair: tuple[str, str], robot_name: str) -> str:
        return pair[1] if pair[0] == robot_name else pair[0]

    def _select_yielder(self, left_name: str, right_name: str) -> str:
        left_remaining = self._distance_to_goal(left_name)
        right_remaining = self._distance_to_goal(right_name)

        if left_remaining is not None and right_remaining is not None:
            if abs(left_remaining - right_remaining) > 1e-6:
                return left_name if left_remaining > right_remaining else right_name
        elif left_remaining is None and right_remaining is not None:
            return left_name
        elif right_remaining is None and left_remaining is not None:
            return right_name

        return max(left_name, right_name, key=self._robot_sort_key)

    def _apply_yields(self, desired_yields: dict[str, set[str]]) -> None:
        previous_paused = {name for name, robot in self.robots.items() if robot.paused}
        self.active_yields = desired_yields
        paused_now = set(desired_yields.keys())
        now_sec = self.get_clock().now().nanoseconds / 1e9

        for robot_name, robot in self.robots.items():
            robot.yielding_to = desired_yields.get(robot_name, set())
            robot.paused = robot_name in paused_now
            if robot.paused and robot.paused_since_sec is None:
                robot.paused_since_sec = now_sec
            elif not robot.paused:
                robot.paused_since_sec = None

        for robot_name in sorted(previous_paused - paused_now, key=self._robot_sort_key):
            self._publish_event(
                f"ATC clear: condition cleared; {robot_name} is being tasked to "
                f"{self._goal_label(robot_name)}; protected occupied ring removed and pause released",
                robot_id=robot_name,
                event_type="pause_released",
            )
        for robot_name in sorted(paused_now - previous_paused, key=self._robot_sort_key):
            yielding_to = ", ".join(sorted(desired_yields[robot_name], key=self._robot_sort_key))
            self._publish_event(
                f"ATC pause command sent: {robot_name} yielding to {yielding_to}",
                robot_id=robot_name,
                event_type="pause_command",
                yielding_to=yielding_to,
            )

    def _add_protected_zone_yields(self, desired_yields: dict[str, set[str]]) -> None:
        # UC3, UC4, and UC6: once a robot is paused, treat its occupied circle as
        # a temporary obstacle so rear-approachers, queued robots, and backing
        # maneuvers stay clear of the same space.
        enforcement_distance = self.protected_zone_radius_m + self.protected_zone_enforcement_margin_m
        protected_names = set(desired_yields.keys())
        allowed_through_by_protected_robot = {
            protected_name: set(desired_yields.get(protected_name, set())) for protected_name in protected_names
        }
        protected_robots = [
            self.robots[robot_name]
            for robot_name in protected_names
            if self.robots[robot_name].pose is not None
        ]
        if not protected_robots:
            return

        for robot_name, robot in self.robots.items():
            if robot_name in protected_names or robot.pose is None:
                continue

            for protected_robot in protected_robots:
                if robot_name in allowed_through_by_protected_robot.get(protected_robot.name, set()):
                    continue

                distance = self._pose_distance(robot.pose, protected_robot.pose)
                if distance > enforcement_distance:
                    continue

                already_held = protected_robot.name in self.active_yields.get(robot_name, set())
                desired_yields.setdefault(robot_name, set()).add(protected_robot.name)
                if not already_held:
                    self._publish_event(
                        f"ATC protected-zone stop: {robot_name} attempted to enter {protected_robot.name}'s "
                        f"occupied ring at {distance:.3f} m; {robot_name} paused and will be retasked to "
                        f"{self._goal_label(robot_name)} after the zone clears",
                        robot_id=robot_name,
                        event_type="protected_zone_stop",
                        related_robot_id=protected_robot.name,
                        separation_m=distance,
                )
                break

    def _break_yield_cycles(self, desired_yields: dict[str, set[str]]) -> None:
        # UC5: when multiple robots yield to each other in a loop, release one
        # robot deterministically so the congestion can unwind instead of
        # stalling the whole group.
        current_cycle_keys = set()
        while True:
            cycles = self._yield_cycles(desired_yields)
            if not cycles:
                break

            cycle = cycles[0]
            current_cycle_keys.add(cycle)
            released_robot = self._select_cycle_release_robot(cycle)
            blockers = sorted(desired_yields.get(released_robot, set()), key=self._robot_sort_key)
            desired_yields.pop(released_robot, None)

            if cycle not in self.reported_yield_cycles:
                self._publish_event(
                    f"ATC yield-cycle breaker: cycle {' -> '.join(cycle)} detected; "
                    f"releasing {released_robot} and keeping peers paused so the cluster can unwind",
                    robot_id=released_robot,
                    event_type="yield_cycle_breaker",
                    cycle=" -> ".join(cycle),
                    released_robot_id=released_robot,
                    released_robot_goal=self._goal_label(released_robot),
                    released_robot_distance_to_goal_m=self._distance_to_goal(released_robot),
                    released_robot_previous_blockers=",".join(blockers),
                )

        self.reported_yield_cycles &= current_cycle_keys
        self.reported_yield_cycles |= current_cycle_keys

    def _yield_cycles(self, desired_yields: dict[str, set[str]]) -> list[tuple[str, ...]]:
        cycles = set()

        def visit(start: str, current: str, path: list[str]) -> None:
            for next_robot in sorted(desired_yields.get(current, set()), key=self._robot_sort_key):
                if next_robot not in desired_yields:
                    continue
                if next_robot == start:
                    cycles.add(self._canonical_cycle(path))
                    continue
                if next_robot in path:
                    continue
                visit(start, next_robot, path + [next_robot])

        for robot_name in sorted(desired_yields.keys(), key=self._robot_sort_key):
            visit(robot_name, robot_name, [robot_name])

        return sorted(cycles, key=lambda cycle: (len(cycle), cycle))

    def _canonical_cycle(self, cycle: list[str]) -> tuple[str, ...]:
        rotations = []
        for index in range(len(cycle)):
            rotations.append(tuple(cycle[index:] + cycle[:index]))
        return min(rotations, key=lambda rotated: [self._robot_sort_key(name) for name in rotated])

    def _select_cycle_release_robot(self, cycle: tuple[str, ...]) -> str:
        return min(
            cycle,
            key=lambda robot_name: (
                self._distance_to_goal(robot_name) if self._distance_to_goal(robot_name) is not None else math.inf,
                -self._pause_duration_sec(robot_name),
                self._robot_sort_key(robot_name),
            ),
        )

    def _publish_pause_states(self) -> None:
        for robot_name, robot in self.robots.items():
            msg = Bool()
            msg.data = robot.paused
            self.pause_publishers[robot_name].publish(msg)

    def _publish_protected_zones(self) -> None:
        # Publish the paused robots as visible occupied zones in RViz so the
        # chosen use case is obvious during a demo.
        now = self.get_clock().now().to_msg()
        marker_array = MarkerArray()

        for marker_index, robot in enumerate(
            [robot for robot in self.robots.values() if robot.paused and robot.pose is not None],
            start=1,
        ):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = now
            marker.ns = "atc_protected_zones"
            marker.id = marker_index * 2
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = robot.pose.pose.position.x
            marker.pose.position.y = robot.pose.pose.position.y
            marker.pose.position.z = 0.02
            marker.pose.orientation.w = 1.0
            marker.scale.x = self.protected_zone_radius_m * 2.0
            marker.scale.y = self.protected_zone_radius_m * 2.0
            marker.scale.z = 0.04
            marker.color.a = 0.35
            marker.color.r = 1.0
            marker.color.g = 0.25
            marker.color.b = 0.0
            marker_array.markers.append(marker)

            ring_marker = Marker()
            ring_marker.header.frame_id = "map"
            ring_marker.header.stamp = now
            ring_marker.ns = "atc_protected_zones"
            ring_marker.id = marker_index * 2 + 1
            ring_marker.type = Marker.LINE_STRIP
            ring_marker.action = Marker.ADD
            ring_marker.pose.orientation.w = 1.0
            ring_marker.scale.x = 0.035
            ring_marker.color.a = 1.0
            ring_marker.color.r = 0.0
            ring_marker.color.g = 0.0
            ring_marker.color.b = 0.0
            for step in range(49):
                angle = 2.0 * math.pi * step / 48.0
                point = Point()
                point.x = robot.pose.pose.position.x + self.protected_zone_radius_m * math.cos(angle)
                point.y = robot.pose.pose.position.y + self.protected_zone_radius_m * math.sin(angle)
                point.z = 0.09
                ring_marker.points.append(point)
            marker_array.markers.append(ring_marker)

        clear_marker = Marker()
        clear_marker.header.frame_id = "map"
        clear_marker.header.stamp = now
        clear_marker.ns = "atc_protected_zones"
        clear_marker.id = 10_000
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.insert(0, clear_marker)
        self.marker_pub.publish(marker_array)

    def _publish_traffic_map(self) -> None:
        # UC3, UC4, and UC7: expose paused robots and fresh fleet positions as a
        # shared occupancy grid instead of relying on lidar interpretation.
        fresh_names = [name for name in sorted(self.robots.keys(), key=self._robot_sort_key) if self._pose_is_fresh(name)]
        for robot_name in sorted(self.robots.keys(), key=self._robot_sort_key):
            obstacle_names = [name for name in fresh_names if name != robot_name]
            self.traffic_map_publishers[robot_name].publish(self._make_traffic_map(obstacle_names))
        if self.traffic_map_pub is not None:
            self.traffic_map_pub.publish(self._make_traffic_map(fresh_names))

    def _publish_clear_traffic_map(self) -> None:
        clear_names = []
        for robot_name in sorted(self.robots.keys(), key=self._robot_sort_key):
            self.traffic_map_publishers[robot_name].publish(self._make_traffic_map(clear_names))
        if self.traffic_map_pub is not None:
            self.traffic_map_pub.publish(self._make_traffic_map(clear_names))

    def _publish_diagnostic_snapshot_if_due(self) -> None:
        if self.diagnostic_period_sec <= 0.0:
            return
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec < self.next_diagnostic_snapshot_sec:
            return
        self.next_diagnostic_snapshot_sec = now_sec + self.diagnostic_period_sec

        paused = [name for name, robot in self.robots.items() if robot.paused]
        stale = [name for name in self.robots if not self._pose_is_fresh(name)]
        active_edges = self._yield_edges(self.active_yields)
        longest_pause_robot = ""
        longest_pause_sec = 0.0
        for robot_name in paused:
            pause_sec = self._pause_duration_sec(robot_name)
            if pause_sec > longest_pause_sec:
                longest_pause_robot = robot_name
                longest_pause_sec = pause_sec

        self._publish_event(
            "ATC diagnostic snapshot",
            event_type="diagnostic_snapshot",
            snapshot_fresh_robot_count=len(self.robots) - len(stale),
            stale_robot_count=len(stale),
            stale_robots=",".join(sorted(stale, key=self._robot_sort_key)),
            paused_robot_count=len(paused),
            paused_robots=",".join(sorted(paused, key=self._robot_sort_key)),
            active_yield_count=sum(len(yields) for yields in self.active_yields.values()),
            active_yield_edge_count=len(active_edges),
            active_yield_cycle_count=len(self._yield_cycles(self.active_yields)),
            longest_pause_robot_id=longest_pause_robot,
            longest_pause_sec=longest_pause_sec,
            deadlock_release_robot_id=self.deadlock_release_robot or "",
            **self.last_evaluation_summary,
        )

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
        obstacle_radius = max(self.protected_zone_radius_m, self.dynamic_obstacle_radius_m)
        radius_cells = max(1, int(math.ceil(obstacle_radius / self.grid_resolution_m)))
        for robot_name in obstacle_names:
            robot = self.robots[robot_name]
            if robot.pose is None:
                continue

            center_x = int(math.floor(robot.pose.pose.position.x / self.grid_resolution_m))
            center_y = int(math.floor(robot.pose.pose.position.y / self.grid_resolution_m))
            for y in range(max(0, center_y - radius_cells), min(self.grid_height_cells, center_y + radius_cells + 1)):
                for x in range(max(0, center_x - radius_cells), min(self.grid_width_cells, center_x + radius_cells + 1)):
                    cell_x_m = (x + 0.5) * self.grid_resolution_m
                    cell_y_m = (y + 0.5) * self.grid_resolution_m
                    dx = cell_x_m - robot.pose.pose.position.x
                    dy = cell_y_m - robot.pose.pose.position.y
                    if math.hypot(dx, dy) <= obstacle_radius:
                        data[y * self.grid_width_cells + x] = 100

        msg.data = data
        return msg

    def _pose_is_fresh(self, robot_name: str) -> bool:
        pose_time = self.pose_times.get(robot_name)
        if pose_time is None:
            return False
        return self.get_clock().now().nanoseconds / 1e9 - pose_time <= self.stale_pose_sec

    def _distance_between(self, left_name: str, right_name: str) -> float:
        left_pose = self.robots[left_name].pose
        right_pose = self.robots[right_name].pose
        dx = right_pose.pose.position.x - left_pose.pose.position.x
        dy = right_pose.pose.position.y - left_pose.pose.position.y
        return math.hypot(dx, dy)

    @staticmethod
    def _pose_distance(left_pose: PoseStamped, right_pose: PoseStamped) -> float:
        dx = right_pose.pose.position.x - left_pose.pose.position.x
        dy = right_pose.pose.position.y - left_pose.pose.position.y
        return math.hypot(dx, dy)

    def _pause_duration_sec(self, robot_name: str) -> float:
        paused_since = self.robots[robot_name].paused_since_sec
        if paused_since is None:
            return 0.0
        return self.get_clock().now().nanoseconds / 1e9 - paused_since

    def _distance_to_goal(self, robot_name: str) -> Optional[float]:
        robot = self.robots[robot_name]
        if robot.pose is None or robot.goal_pose is None:
            return None
        dx = robot.goal_pose.pose.position.x - robot.pose.pose.position.x
        dy = robot.goal_pose.pose.position.y - robot.pose.pose.position.y
        return math.hypot(dx, dy)

    def _approaching_same_goal_area(self, left_name: str, right_name: str) -> bool:
        left = self.robots[left_name]
        right = self.robots[right_name]
        if left.goal_pose is None or right.goal_pose is None:
            return False
        goal_dx = right.goal_pose.pose.position.x - left.goal_pose.pose.position.x
        goal_dy = right.goal_pose.pose.position.y - left.goal_pose.pose.position.y
        return math.hypot(goal_dx, goal_dy) <= self.clear_distance_m

    def _midpoint(self, left_name: str, right_name: str) -> tuple[float, float]:
        left_pose = self.robots[left_name].pose
        right_pose = self.robots[right_name].pose
        return (
            (left_pose.pose.position.x + right_pose.pose.position.x) / 2.0,
            (left_pose.pose.position.y + right_pose.pose.position.y) / 2.0,
        )

    def _goal_label(self, robot_name: str) -> str:
        robot = self.robots[robot_name]
        if robot.goal_name:
            return robot.goal_name
        if robot.goal_pose is not None:
            return f"pose({robot.goal_pose.pose.position.x:.2f},{robot.goal_pose.pose.position.y:.2f})"
        return "unknown"

    def _publish_event(self, text: str, robot_id: str = "", **attributes) -> None:
        span_robot_id = robot_id or self._first_robot_mentioned(text)
        traceparent = self.run_context.robot_traceparent(span_robot_id)
        attributes.setdefault("paused_robot_count", sum(1 for robot in self.robots.values() if robot.paused))
        attributes.setdefault("active_yield_count", sum(len(yields) for yields in self.active_yields.values()))
        attributes.setdefault(
            "active_yield_cycles",
            ";".join(" -> ".join(cycle) for cycle in self._yield_cycles(self.active_yields)),
        )
        with self.telemetry.start_as_current_span(
            "atc.event",
            run_id=self.run_context.run_id,
            robot_id=span_robot_id,
            traceparent=traceparent,
            event_text=text,
            **attributes,
        ) as span:
            updated_traceparent = Telemetry.traceparent_from_span(span)
            if updated_traceparent:
                self.run_context.update_robot_traceparent(span_robot_id, updated_traceparent)
                traceparent = updated_traceparent
            span.add_event("atc_decision", {"event.text": text})
            self.telemetry.log(
                text,
                run_id=self.run_context.run_id,
                robot_id=span_robot_id,
                traceparent=traceparent,
                event_text=text,
                **attributes,
            )
        msg = String()
        msg.data = text
        self.event_pub.publish(msg)
        self._write_diagnostic_log(text)

    def _initialize_diagnostic_log(self) -> None:
        log_dir = os.path.dirname(self.diagnostic_log_path)
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
        with open(self.diagnostic_log_path, "w", encoding="ascii") as log_file:
            log_file.write("")
        self._log("info", f"ATC diagnostic log: {self.diagnostic_log_path}", diagnostic_log_path=self.diagnostic_log_path)

    def _write_diagnostic_log(self, event_text: str) -> None:
        entry = {
            "time_sec": self.get_clock().now().nanoseconds / 1e9,
            "event": event_text,
            "parameters": {
                "safety_distance_m": self.safety_distance_m,
                "clear_distance_m": self.clear_distance_m,
                "closing_epsilon_m": self.closing_epsilon_m,
                "protected_zone_radius_m": self.protected_zone_radius_m,
                "dynamic_obstacle_radius_m": self.dynamic_obstacle_radius_m,
                "publish_traffic_map": self.publish_traffic_map,
                "max_pause_sec": self.max_pause_sec,
                "deadlock_release_sec": self.deadlock_release_sec,
                "stale_pose_sec": self.stale_pose_sec,
                "diagnostic_period_sec": self.diagnostic_period_sec,
            },
            "evaluation_summary": self.last_evaluation_summary,
            "fresh_robot_count": sum(1 for robot_name in self.robots if self._pose_is_fresh(robot_name)),
            "paused_robot_count": sum(1 for robot in self.robots.values() if robot.paused),
            "stale_robots": [
                robot_name
                for robot_name in sorted(self.robots.keys(), key=self._robot_sort_key)
                if not self._pose_is_fresh(robot_name)
            ],
            "active_yields": {
                robot_name: sorted(list(yielding_to), key=self._robot_sort_key)
                for robot_name, yielding_to in self.active_yields.items()
            },
            "active_yield_edges": self._yield_edges(self.active_yields),
            "active_yield_cycles": [" -> ".join(cycle) for cycle in self._yield_cycles(self.active_yields)],
            "deadlock_release_robot": self.deadlock_release_robot,
            "deadlock_release_until_sec": self.deadlock_release_until_sec,
            "protected_zone_authorized_passers": {
                robot_name: sorted(list(yielding_to), key=self._robot_sort_key)
                for robot_name, yielding_to in self.active_yields.items()
            },
            "robots": {},
        }

        for robot_name in sorted(self.robots.keys(), key=self._robot_sort_key):
            robot = self.robots[robot_name]
            pose_age = None
            if robot_name in self.pose_times:
                pose_age = self.get_clock().now().nanoseconds / 1e9 - self.pose_times[robot_name]
            entry["robots"][robot_name] = {
                "paused": robot.paused,
                "paused_since_sec": robot.paused_since_sec,
                "pause_duration_sec": self._pause_duration_sec(robot_name),
                "yielding_to": sorted(list(robot.yielding_to), key=self._robot_sort_key),
                "pose": self._pose_dict(robot.pose),
                "pose_age_sec": pose_age,
                "goal_name": robot.goal_name,
                "goal_pose": self._pose_dict(robot.goal_pose),
                "distance_to_goal_m": self._distance_to_goal(robot_name),
            }

        with open(self.diagnostic_log_path, "a", encoding="ascii") as log_file:
            log_file.write(json.dumps(entry, sort_keys=True) + "\n")

    def _yield_edges(self, yields: dict[str, set[str]]) -> list[dict]:
        edges = []
        for yielder in sorted(yields.keys(), key=self._robot_sort_key):
            for blocker in sorted(yields[yielder], key=self._robot_sort_key):
                distance = None
                if yielder in self.robots and blocker in self.robots:
                    if self.robots[yielder].pose is not None and self.robots[blocker].pose is not None:
                        distance = self._distance_between(yielder, blocker)
                edges.append(
                    {
                        "yielder": yielder,
                        "blocked_by": blocker,
                        "distance_m": distance,
                        "yielder_goal": self._goal_label(yielder),
                        "blocked_by_goal": self._goal_label(blocker),
                    }
                )
        return edges

    def _first_robot_mentioned(self, text: str) -> str:
        for robot_name in sorted(self.robots.keys(), key=self._robot_sort_key):
            if re.search(rf"(?<![A-Za-z0-9_]){re.escape(robot_name)}(?![A-Za-z0-9_])", text):
                return robot_name
        return ""

    def _log(self, level: str, message: str, robot_id: str = "", **attrs) -> None:
        self.telemetry.log(
            message,
            level=level,
            run_id=self.run_context.run_id,
            robot_id=robot_id,
            traceparent=self.run_context.robot_traceparent(robot_id),
            **attrs,
        )

    @staticmethod
    def _pose_dict(pose: Optional[PoseStamped]) -> Optional[dict]:
        if pose is None:
            return None
        return {
            "x": pose.pose.position.x,
            "y": pose.pose.position.y,
            "yaw": math.atan2(
                2.0 * pose.pose.orientation.w * pose.pose.orientation.z,
                1.0 - 2.0 * pose.pose.orientation.z * pose.pose.orientation.z,
            ),
        }

    @staticmethod
    def _robot_sort_key(name: str):
        prefix = name.rstrip("0123456789")
        suffix = name[len(prefix) :]
        if suffix.isdigit():
            return (prefix, int(suffix))
        return (name, 0)


def main() -> None:
    rclpy.init()
    node = CentralizedTrafficManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
