import pkgutil
import importlib
import logging

def register_local_plugins() -> None:
    # robots
    from src.robots.franka_fr3_robotiq_gripper.franka_fr3_robotiq_gripper import FrankaFr3RobotiqGripper
    from src.robots.franka_fr3_robotiq_gripper.config_franka_fr3_robotiq_gripper import FrankaFr3RobotiqGripperConfig

    # teleoperators
    from src.teleoperators.gello_leader.gello_ros_leader import GelloRosLeader
    from src.teleoperators.gello_leader.config_gello_ros_leader import GelloRosLeaderConfig

    # policies
    # from src.policies.xvla_client.modeling_xvla_client import XVLAClientPolicy
    # from src.policies.xvla_client.configuration_xvla_client import XVLAClientConfig
    
    try:
        from src.policies.openpi_client.modeling_openpi_client import OpenPIClientPolicy
        from src.policies.openpi_client.configuration_openpi_client import OpenPIClientConfig
    except ImportError as e:
        logging.warning(f"Could not import openpi_client policy: {e}")

