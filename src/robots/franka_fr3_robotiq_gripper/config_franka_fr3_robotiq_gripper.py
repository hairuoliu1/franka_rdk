from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig
from lerobot.robots.config import RobotConfig


@RobotConfig.register_subclass("franka_fr3_robotiq_gripper")
@dataclass
class FrankaFr3RobotiqGripperConfig(RobotConfig):
    """Configuration for Franka FR3 robot with Robotiq gripper and RealSense cameras."""

    # connection
    # robot_ip: str = "172.16.0.2"
    # gripper_port: str = "/dev/ttyUSB1"
    # cameras

    # Gripper configuration
    # gripper_limits: list[float] = field(
    #     default_factory=lambda: [0.01, 0.80]
    # )  # min and max opening in meters
    # gripper_threshold: float = 0.5
    # binarize_gripper: bool = False
    use_gripper: bool = True
    gripper_command_topic: str = "/gripper/gripper_client/target_gripper_width_percent"
    gripper_state_topic: str = "/gripper/joint_states"
    gripper_state_joint_index: int = 0
    gripper_state_invert: bool = True # True if 0% command corresponds to max opening, False if 0% command corresponds to fully closed
    # arm_state configuration
    arm_state_topic: str = "/franka/joint_states"
    arm_command_topic: str = "/gello/joint_states"

    # Force torque sensor configuration
    use_ft_sensor: bool = True
    ft_sensor_topic: str = "/robotiq_force_torque_sensor_broadcaster/wrench"

    # Control method: "joint" or "tcp"
    control_method: str = "joint"

    # Camera configuration (RealSense D405 and D415)
    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "top": RealSenseCameraConfig(
                serial_number_or_name="311122062207",  # Update with actual serial number
                width=640,
                height=480,
                fps=30,
            ),
            "wrist": RealSenseCameraConfig(
                serial_number_or_name="352122272067",  # Update with actual serial number
                width=640,
                height=480,
                fps=30,
            ),
        }
    )

    # width = 640, height = 480, fps = 30

    # # Initialization configuration
    # init: bool = True
    # init_method: str = "joint"  # "joint" or "tcp"
    # init_tcp_positions: list[float] = field(
    #     default_factory=lambda: [0.787, 0.184, 0.512, 2.194, 2.190, 0.072, 0.0]
    # )
    # init_joint_positions: list[float] = field(
    #     default_factory=lambda: np.deg2rad([90, -90, 90, -90, -90, -180, 0]).tolist()
    # )
