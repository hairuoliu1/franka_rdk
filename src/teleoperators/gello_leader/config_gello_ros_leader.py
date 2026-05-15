from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig

@TeleoperatorConfig.register_subclass("gello_ros_leader")
@dataclass
class GelloRosLeaderConfig(TeleoperatorConfig):
    """
    Configuration for GelloROSLeader, a passive listener teleoperator that listens to GELLO's ROS topics for arm and gripper commands.
    This is used for recording GELLO's commands without actually sending them to the robot.
    """
    arm_command_topic: str = "/gello/joint_states"
    gripper_command_topic: str = "/gripper/gripper_client/target_gripper_width_percent"
