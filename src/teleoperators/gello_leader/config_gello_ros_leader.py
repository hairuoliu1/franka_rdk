from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@dataclass
class GelloRosLeaderConfigBase:
    """Configuration for GelloROSLeader, a passive listener teleoperator.

    Listens to GELLO's ROS topics for arm and gripper commands without
    re-publishing them to the robot (bypass/passive mode).

    Single-arm instances default to the left arm namespace. Bimanual wrappers
    keep the left arm default and override the right arm namespace to "right".
    """

    # ROS 2 domain ID for cross-machine communication. Default None uses env var or ROS 2 default (0).
    ros_domain_id: int | None = None

    # Topic namespace. Single-arm defaults to the left arm.
    topic_namespace: str = "left"

    # Use Python 3.10 ROS bridge subprocess instead of direct rclpy import.
    use_bridge: bool = True

    arm_command_topic: str = "/gello/joint_states"
    gripper_command_topic: str = "/gripper/gripper_client/target_gripper_width_percent"
    gripper_raw_topic: str = "/gello/gripper_position"
    gripper_max_closed_position: float = 0.085


@TeleoperatorConfig.register_subclass("gello_ros_leader")
@dataclass
class GelloRosLeaderConfig(TeleoperatorConfig, GelloRosLeaderConfigBase):
    pass
