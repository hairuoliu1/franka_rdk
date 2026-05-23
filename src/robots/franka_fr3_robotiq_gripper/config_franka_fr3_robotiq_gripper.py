from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.robots.config import RobotConfig


@dataclass
class FrankaFr3RobotiqGripperConfigBase:
    """Configuration for Franka FR3 robot with Robotiq gripper and OpenCV cameras.

    Supports cross-machine ROS 2 communication via DDS discovery.
    Single-arm instances default to the left arm namespace. Bimanual wrappers
    keep the left arm default and override the right arm namespace to "right".
    """

    # ROS 2 cross-machine communication: set this to the same value on both
    # the local (camera) machine and the remote (robot/Gello) machine.
    # Default None uses the ROS_DOMAIN_ID env var or ROS 2 default (0).
    ros_domain_id: int | None = None

    # Remote robot machine IP for DDS peer discovery fallback.
    # Only needed if multicast discovery fails between machines.
    # When set, configures Cyclone DDS to use explicit peer discovery.
    remote_ip: str | None = None

    # Topic namespace. Single-arm defaults to the left arm.
    # Topics become: /{namespace}/franka/joint_states, /{namespace}/gello/joint_states, etc.
    topic_namespace: str = "left"

    # Gripper configuration
    use_gripper: bool = True
    gripper_command_topic: str = "/gripper/gripper_client/target_gripper_width_percent"
    gripper_state_topic: str = "/gripper/joint_states"
    gripper_state_joint_index: int = 0
    # LeRobot observation/action values use raw Robotiq units.
    # The ROS gripper command topic still receives open-width percent.
    # On this setup, raw 0.0=open and raw 0.085=max closed.
    gripper_max_closed_position: float = 0.085

    # Arm state configuration
    arm_state_topic: str = "/franka/joint_states"
    arm_command_topic: str = "/gello/joint_states"

    # Joint name prefix for state topic (e.g. "left_" → "left_fr3_joint1").
    # Empty string for standard fr3_jointX names (used by Gello).
    state_joint_prefix: str = ""  # auto-derived from topic_namespace if empty

    # Force torque sensor configuration
    use_ft_sensor: bool = False  # disabled: franka_robot_state_broadcaster not running
    ft_sensor_topic: str = "/franka_robot_state_broadcaster/external_wrench_in_base_frame"

    # Control method: "joint" or "tcp"
    control_method: str = "joint"

    # Use Python 3.10 ROS bridge subprocess instead of direct rclpy import.
    # Required when running in Python 3.12+ with ROS 2 Humble (Python 3.10 ABI).
    use_bridge: bool = True

    # Camera configuration
    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "left_camera": OpenCVCameraConfig(
                index_or_path=0,
                width=640,
                height=480,
                fps=30,
                warmup_s=3,
                fourcc="MJPG",
            ),
            "right_camera": OpenCVCameraConfig(
                index_or_path=2,
                width=640,
                height=480,
                fps=30,
                warmup_s=3,
                fourcc="MJPG",
            ),
        }
    )


@RobotConfig.register_subclass("franka_fr3_robotiq_gripper")
@dataclass
class FrankaFr3RobotiqGripperConfig(RobotConfig, FrankaFr3RobotiqGripperConfigBase):
    pass
