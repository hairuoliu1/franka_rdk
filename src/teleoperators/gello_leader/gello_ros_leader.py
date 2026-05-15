import logging
import time
from threading import Event, Thread

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

from lerobot.teleoperators import Teleoperator
from lerobot.types import RobotAction
from src.teleoperators.gello_leader.config_gello_ros_leader import GelloRosLeaderConfig

logger = logging.getLogger(__name__)

class GelloRosLeader(Teleoperator):
    """
    一个被动监听的遥操作类（旁路监听模式）。
    它在 ROS 环境中窃听 GELLO 发出的机器手臂和夹爪控制话题，
    供 lerobot_record 记录这些动作用于日后大模型训练，但不会真正通过它下发动作指令到机器人。
    """
    config_class = GelloRosLeaderConfig
    name = "gello_ros_leader"
    is_passive = True  # 核心标志，阻止 lerobot_record 重发动作。

    def __init__(self, config: GelloRosLeaderConfig | None = None):
        super().__init__(config)
        self.config = config if config else GelloRosLeaderConfig()
        self._connected = False
        self._node = None
        self._executor = None
        self._spin_stop = Event()
        self._spin_thread = None

        self.num_dofs = 8
        self._last_arm_cmd = [0.0] * 7
        self._last_gripper_cmd = 0.0

    @property
    def action_features(self) -> dict[str, type]:
        return {f"joint_positions_{i}": float for i in range(8)}

    @property
    def feedback_features(self) -> dict[str, type]:
        # We don't support feedback like force feedback in passive mode anyway.
        return {}

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def send_feedback(self, feedback: dict) -> None:
        pass

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self):
        if self._connected:
            return

        if not rclpy.ok():
            rclpy.init()

        self._node = Node("gello_listener_for_lerobot")
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        
        # 订阅由 GELLO 等外部控制器发布的真实 Target Topics
        self._node.create_subscription(
            JointState, 
            self.config.arm_command_topic, 
            self._arm_cb, 
            10
        )
        self._node.create_subscription(
            Float32, 
            self.config.gripper_command_topic, 
            self._gripper_cb, 
            10
        )

        self._spin_stop.clear()
        self._spin_thread = Thread(target=self._spin_ros, name="gello_listener_spin", daemon=True)
        self._spin_thread.start()

        # 等待第一帧指令或者直接放行，避免录制死锁
        time.sleep(0.5)

        self._connected = True

    def _spin_ros(self):
        while not self._spin_stop.is_set() and rclpy.ok() and self._node is not None:
            try:
                if self._executor is not None:
                    self._executor.spin_once(timeout_sec=0.05)
            except (RuntimeError, ValueError) as e:
                logger.warning(f"gello listener spin interrupted: {e}")
                break

    def _arm_cb(self, msg: JointState):
        if hasattr(msg, 'position') and len(msg.position) >= 7:
            # 安全读取: 防止 ROS2 底层的乱序发送问题，这里不要直接按照顺序读
            # 我们根据 joint name 来匹配，确保拿到正确的关节位置
            arm_positions = []
            for i in range(1, 8):
                joint_name = f"fr3_joint{i}"
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    arm_positions.append(float(msg.position[idx]))
                else:
                    arm_positions.append(0.0)
            self._last_arm_cmd = arm_positions

    def _gripper_cb(self, msg: Float32):
        self._last_gripper_cmd = float(msg.data)

    def get_action(self) -> RobotAction:
        """
        获取当前窃听到的 Action 给 LeRobot。
        因为你的 FrankaFr3RobotiqGripper 期望的是 action_features 格式为：
        ['joint_positions_0', ..., 'joint_positions_7']。
        """
        action = {}
        for i, val in enumerate(self._last_arm_cmd):
            action[f"joint_positions_{i}"] = float(val)
        
        # 将夹爪也拼凑进去
        action["joint_positions_7"] = float(self._last_gripper_cmd)
        
        return action

    def disconnect(self):
        if not self._connected:
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

    def __del__(self):
        pass # 安全退出在外层做好了
