import importlib
import logging


PLUGIN_MODULES = (
    "src.robots.franka_fr3_robotiq_gripper.franka_fr3_robotiq_gripper",
    "src.robots.franka_fr3_robotiq_gripper.config_franka_fr3_robotiq_gripper",
    "src.robots.bi_franka_fr3_robotiq_gripper.bi_franka_fr3_robotiq_gripper",
    "src.robots.bi_franka_fr3_robotiq_gripper.config_bi_franka_fr3_robotiq_gripper",
    "src.teleoperators.gello_leader.gello_ros_leader",
    "src.teleoperators.gello_leader.config_gello_ros_leader",
    "src.teleoperators.bi_gello_leader.bi_gello_leader",
    "src.teleoperators.bi_gello_leader.config_bi_gello_leader",
)

OPTIONAL_PLUGIN_MODULES = (
    "src.policies.openpi_client.modeling_openpi_client",
    "src.policies.openpi_client.configuration_openpi_client",
)


def register_local_plugins() -> None:
    """Import local modules so their LeRobot registries are populated."""
    for module_name in PLUGIN_MODULES:
        importlib.import_module(module_name)

    for module_name in OPTIONAL_PLUGIN_MODULES:
        try:
            importlib.import_module(module_name)
        except ImportError as exc:
            logging.warning("Could not import optional plugin %s: %s", module_name, exc)
