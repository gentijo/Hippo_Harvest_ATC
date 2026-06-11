from air_traffic_control.air_traffic_control_plugin import AirTrafficControlPlugin, RobotObservation
from geometry_msgs.msg import PoseStamped


def make_pose(x: float, y: float) -> PoseStamped:
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = 1.0
    return pose


def test_distance_trend_classifies_motion_direction() -> None:
    assert AirTrafficControlPlugin._distance_trend(None, 1.0) == "unknown"
    assert AirTrafficControlPlugin._distance_trend(1.0, 1.02) == "increasing"
    assert AirTrafficControlPlugin._distance_trend(1.0, 0.98) == "decreasing"
    assert AirTrafficControlPlugin._distance_trend(1.0, 1.001) == "stable"


def test_minimum_pair_distance_returns_the_closest_robot_pair() -> None:
    robots = [
        RobotObservation(name="robot1", pose=make_pose(0.0, 0.0)),
        RobotObservation(name="robot2", pose=make_pose(0.3, 0.4)),
        RobotObservation(name="robot3", pose=make_pose(5.0, 5.0)),
    ]

    assert AirTrafficControlPlugin._minimum_pair_distance(robots) == 0.5


def test_format_goal_id_truncates_uuid_bytes() -> None:
    raw = bytes.fromhex("1234567890abcdef1234567890abcdef")
    assert AirTrafficControlPlugin._format_goal_id(raw) == "12345678"
    assert AirTrafficControlPlugin._format_goal_id(b"bad") == "-"


def test_first_robot_mentioned_uses_sorted_robot_order() -> None:
    plugin = AirTrafficControlPlugin.__new__(AirTrafficControlPlugin)
    plugin.robots = {
        "robot10": object(),
        "robot2": object(),
        "robot1": object(),
    }

    assert plugin._first_robot_mentioned("robot10 and robot2 and robot1") == "robot1"


class FakeButton:
    def __init__(self) -> None:
        self.checked = None
        self.text = None
        self.blocked = False

    def blockSignals(self, blocked: bool) -> None:
        self.blocked = blocked

    def setChecked(self, checked: bool) -> None:
        self.checked = checked

    def setText(self, text: str) -> None:
        self.text = text


class FakePublisher:
    def __init__(self) -> None:
        self.published = []

    def publish(self, msg) -> None:
        self.published.append(msg.data)


def test_atc_toggle_publish_and_sync_updates_button_state() -> None:
    plugin = AirTrafficControlPlugin.__new__(AirTrafficControlPlugin)
    plugin.atc_enabled = True
    plugin.atc_enabled_pub = FakePublisher()
    plugin.atc_toggle_button = FakeButton()

    plugin._publish_atc_enabled(False)

    assert plugin.atc_enabled is False
    assert plugin.atc_enabled_pub.published == [False]
    assert plugin.atc_toggle_button.checked is False
    assert plugin.atc_toggle_button.text == "ATC Disabled"


def test_atc_enabled_state_callback_updates_local_state() -> None:
    class Msg:
        def __init__(self, data: bool) -> None:
            self.data = data

    plugin = AirTrafficControlPlugin.__new__(AirTrafficControlPlugin)
    plugin.atc_enabled = False
    plugin.atc_toggle_button = FakeButton()

    plugin._on_atc_enabled_state(Msg(True))

    assert plugin.atc_enabled is True
    assert plugin.atc_toggle_button.checked is True
    assert plugin.atc_toggle_button.text == "ATC Enabled"
