from dataclasses import dataclass, field

from lerobot.teleoperators.config import TeleoperatorConfig
from src.teleoperators.gello_leader.config_gello_ros_leader import GelloRosLeaderConfigBase


@TeleoperatorConfig.register_subclass("bi_gello_ros_leader")
@dataclass
class BiGelloRosLeaderConfig(TeleoperatorConfig):
    """Configuration for bimanual GELLO ROS leader.

    Wraps two single-arm GELLO configs: left_arm_config and right_arm_config.
    """

    left_arm_config: GelloRosLeaderConfigBase = field(default_factory=GelloRosLeaderConfigBase)
    right_arm_config: GelloRosLeaderConfigBase = field(
        default_factory=lambda: GelloRosLeaderConfigBase(topic_namespace="right")
    )
