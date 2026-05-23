from dataclasses import dataclass, field

from lerobot.robots.config import RobotConfig
from src.robots.franka_fr3_robotiq_gripper.config_franka_fr3_robotiq_gripper import (
    FrankaFr3RobotiqGripperConfigBase,
)


@RobotConfig.register_subclass("bi_franka_fr3_robotiq_gripper")
@dataclass
class BiFrankaFr3RobotiqGripperConfig(RobotConfig):
    """Configuration for bimanual Franka FR3 with Robotiq grippers.

    Wraps two single-arm configs: left_arm_config and right_arm_config.
    """

    left_arm_config: FrankaFr3RobotiqGripperConfigBase = field(
        default_factory=FrankaFr3RobotiqGripperConfigBase
    )
    right_arm_config: FrankaFr3RobotiqGripperConfigBase = field(
        default_factory=lambda: FrankaFr3RobotiqGripperConfigBase(topic_namespace="right")
    )
