from air_traffic_control.traffic_manager_node import CentralizedTrafficManager, RobotState
from geometry_msgs.msg import PoseStamped


def make_pose(x: float, y: float) -> PoseStamped:
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = 1.0
    return pose


def make_manager() -> CentralizedTrafficManager:
    manager = CentralizedTrafficManager.__new__(CentralizedTrafficManager)
    manager.robots = {}
    manager.active_yields = {}
    manager.reported_yield_cycles = set()
    manager.reported_deadlock_release_events = set()
    manager.previous_pair_distances = {}
    manager.deadlock_release_robot = None
    manager.deadlock_release_until_sec = 0.0
    manager.atc_enabled = True
    manager.publish_traffic_map = False
    manager.protected_zone_radius_m = 0.09
    manager.protected_zone_enforcement_margin_m = 0.06
    manager.clear_distance_m = 0.55
    manager.max_pause_sec = 8.0
    manager.deadlock_release_sec = 3.0
    manager.atc_enabled_pub = type("Pub", (), {"publish": lambda self, msg: None})()
    manager._publish_event = lambda *args, **kwargs: None
    manager._publish_pause_states = lambda: None
    manager._publish_protected_zones = lambda: None
    manager._publish_clear_traffic_map = lambda: None
    return manager


def test_select_yielder_prefers_robot_with_longer_remaining_distance() -> None:
    manager = make_manager()
    manager.robots = {
        "robot1": RobotState(name="robot1", pose=make_pose(0.0, 0.0), goal_pose=make_pose(10.0, 0.0)),
        "robot2": RobotState(name="robot2", pose=make_pose(0.0, 0.0), goal_pose=make_pose(2.0, 0.0)),
    }

    assert manager._select_yielder("robot1", "robot2") == "robot1"


def test_select_yielder_prefers_robot_with_defined_goal_when_other_is_missing() -> None:
    manager = make_manager()
    manager.robots = {
        "robot1": RobotState(name="robot1", pose=make_pose(0.0, 0.0), goal_pose=None),
        "robot2": RobotState(name="robot2", pose=make_pose(0.0, 0.0), goal_pose=make_pose(2.0, 0.0)),
    }

    assert manager._select_yielder("robot1", "robot2") == "robot1"


def test_break_yield_cycles_releases_a_single_robot_to_unwind_the_cycle() -> None:
    manager = make_manager()
    manager.robots = {
        "robot1": RobotState(name="robot1", pose=make_pose(0.0, 0.0), goal_pose=make_pose(5.0, 0.0)),
        "robot2": RobotState(name="robot2", pose=make_pose(1.0, 0.0), goal_pose=make_pose(3.0, 0.0)),
        "robot3": RobotState(name="robot3", pose=make_pose(2.0, 0.0), goal_pose=make_pose(2.1, 0.0)),
    }
    manager._distance_to_goal = lambda robot_name: {"robot1": 5.0, "robot2": 2.0, "robot3": 0.1}[robot_name]
    manager._pause_duration_sec = lambda robot_name: {"robot1": 1.0, "robot2": 2.0, "robot3": 3.0}[robot_name]

    desired_yields = {
        "robot1": {"robot2"},
        "robot2": {"robot3"},
        "robot3": {"robot1"},
    }

    manager._break_yield_cycles(desired_yields)

    assert desired_yields == {"robot1": {"robot2"}, "robot2": {"robot3"}}
    assert manager._yield_cycles(desired_yields) == []


def test_add_protected_zone_yields_blocks_a_nearby_robot() -> None:
    events = []
    manager = make_manager()
    manager._publish_event = lambda text, **attrs: events.append((text, attrs))
    manager.robots = {
        "robot1": RobotState(name="robot1", pose=make_pose(0.0, 0.0)),
        "robot2": RobotState(name="robot2", pose=make_pose(0.12, 0.0)),
    }

    desired_yields = {"robot1": set()}

    manager._add_protected_zone_yields(desired_yields)

    assert desired_yields["robot2"] == {"robot1"}
    assert any("protected-zone stop" in text for text, _ in events)


def test_disabling_atc_clears_pause_state_and_removes_yields() -> None:
    manager = make_manager()
    manager.robots = {
        "robot1": RobotState(
            name="robot1",
            pose=make_pose(0.0, 0.0),
            goal_pose=make_pose(5.0, 0.0),
            paused=True,
            paused_since_sec=12.3,
            yielding_to={"robot2"},
        ),
        "robot2": RobotState(name="robot2", pose=make_pose(1.0, 0.0)),
    }
    manager.active_yields = {"robot1": {"robot2"}}

    manager._set_atc_enabled(False)

    assert manager.atc_enabled is False
    assert manager.active_yields == {}
    assert manager.robots["robot1"].paused is False
    assert manager.robots["robot1"].paused_since_sec is None
    assert manager.robots["robot1"].yielding_to == set()
