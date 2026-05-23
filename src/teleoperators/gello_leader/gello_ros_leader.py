import logging
import os
import time
from threading import Event, Thread

from lerobot.teleoperators import Teleoperator
from lerobot.types import RobotAction
from src.teleoperators.gello_leader.config_gello_ros_leader import GelloRosLeaderConfig

logger = logging.getLogger(__name__)

ARM_JOINT_NAMES = tuple(f"fr3_joint{i}" for i in range(1, 8))
ROBOT_DOF = len(ARM_JOINT_NAMES) + 1


def _namespaced(base_topic: str, namespace: str) -> str:
    """Prepend namespace to a topic path."""
    if not namespace:
        return base_topic
    ns = namespace.strip("/")
    topic = base_topic.lstrip("/")
    return f"/{ns}/{topic}"


class GelloRosLeader(Teleoperator):
    """Passive teleoperator that listens to GELLO ROS topics.

    In bypass mode, records GELLO actions to the dataset without re-publishing
    them to the robot (avoids conflict with the live control pipeline).

    Supports bridge mode (Python 3.10 subprocess) for Python 3.12+ compatibility.
    """

    config_class = GelloRosLeaderConfig
    name = "gello_ros_leader"
    is_passive = True

    def __init__(self, config: GelloRosLeaderConfig | None = None):
        super().__init__(config)
        self.config = config if config else GelloRosLeaderConfig()
        self._connected = False
        self._node = None
        self._executor = None
        self._spin_stop = Event()
        self._spin_thread = None

        self.num_dofs = ROBOT_DOF
        self._last_arm_cmd = [0.0] * len(ARM_JOINT_NAMES)
        self._last_gripper_cmd = 0.0
        self._last_gripper_raw_msg_at = 0.0

        self._ns = getattr(self.config, "topic_namespace", "")
        self._use_bridge = getattr(self.config, "use_bridge", True)
        self._bridge_client = None

    @property
    def action_features(self) -> dict[str, type]:
        return {f"joint_positions_{i}": float for i in range(ROBOT_DOF)}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        return

    def configure(self) -> None:
        return

    def send_feedback(self, feedback: dict) -> None:
        return

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        if self._connected:
            return

        if self._use_bridge:
            self._connect_bridge()
            return

        # Direct ROS mode
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float32

        if not rclpy.ok():
            domain = getattr(self.config, "ros_domain_id", None)
            if domain is not None:
                os.environ["ROS_DOMAIN_ID"] = str(domain)
            rclpy.init()

        arm_cmd_topic = _namespaced(self.config.arm_command_topic, self._ns)
        gripper_cmd_topic = _namespaced(self.config.gripper_command_topic, self._ns)
        gripper_raw_topic = _namespaced(self.config.gripper_raw_topic, self._ns)

        self._node = Node("gello_listener_for_lerobot")
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        self._node.create_subscription(JointState, arm_cmd_topic, self._arm_cb, 10)
        self._node.create_subscription(Float32, gripper_cmd_topic, self._gripper_percent_cb, 10)
        self._node.create_subscription(Float32, gripper_raw_topic, self._gripper_raw_cb, 10)

        logger.info(
            "Gello listener: ns=%s, arm=%s, gripper_percent=%s, gripper_raw=%s",
            self._ns or "none",
            arm_cmd_topic,
            gripper_cmd_topic,
            gripper_raw_topic,
        )

        self._spin_stop.clear()
        self._spin_thread = Thread(target=self._spin_ros, name="gello_listener_spin", daemon=True)
        self._spin_thread.start()

        time.sleep(0.5)
        self._connected = True

    def _connect_bridge(self) -> None:
        from src.ros_bridge_client import get_bridge_client

        self._bridge_client = get_bridge_client(namespace=self._ns)
        self._bridge_client.connect()
        self._connected = True

    def _spin_ros(self) -> None:
        import rclpy
        while not self._spin_stop.is_set() and rclpy.ok() and self._node is not None:
            try:
                if self._executor is not None:
                    self._executor.spin_once(timeout_sec=0.05)
            except (RuntimeError, ValueError) as e:
                logger.warning("gello listener spin interrupted: %s", e)
                break

    def _arm_cb(self, msg) -> None:
        if hasattr(msg, "position") and len(msg.position) >= len(ARM_JOINT_NAMES):
            positions_by_name = dict(zip(msg.name, msg.position))
            self._last_arm_cmd = [
                float(positions_by_name.get(joint_name, 0.0))
                for joint_name in ARM_JOINT_NAMES
            ]

    def _gripper_percent_cb(self, msg) -> None:
        if time.time() - self._last_gripper_raw_msg_at < 0.5:
            return
        open_percent = max(0.0, min(1.0, float(msg.data)))
        max_closed = float(getattr(self.config, "gripper_max_closed_position", 0.085))
        self._last_gripper_cmd = max_closed * (1.0 - open_percent)

    def _gripper_raw_cb(self, msg) -> None:
        self._last_gripper_cmd = float(msg.data)
        self._last_gripper_raw_msg_at = time.time()

    def get_action(self) -> RobotAction:
        if self._use_bridge and self._bridge_client is not None:
            return dict(self._bridge_client.get_action())

        action = {}
        for i, val in enumerate(self._last_arm_cmd):
            action[f"joint_positions_{i}"] = float(val)
        action["joint_positions_7"] = float(self._last_gripper_cmd)
        return action

    def disconnect(self) -> None:
        if not self._connected:
            return

        if self._use_bridge:
            if self._bridge_client is not None:
                self._bridge_client.disconnect()
                self._bridge_client = None
            self._connected = False
            return

        self._spin_stop.set()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=1.0)

        if self._node is not None:
            if self._executor is not None:
                self._executor.remove_node(self._node)
                self._executor.shutdown(timeout_sec=0.5)
                self._executor = None
            self._node.destroy_node()
            self._node = None

        self._connected = False
