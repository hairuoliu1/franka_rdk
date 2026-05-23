import logging
from dataclasses import fields
from typing import Any

from lerobot.teleoperators import Teleoperator
from lerobot.types import RobotAction

from .config_bi_gello_leader import BiGelloRosLeaderConfig
from src.teleoperators.gello_leader.gello_ros_leader import GelloRosLeader
from src.teleoperators.gello_leader.config_gello_ros_leader import (
    GelloRosLeaderConfig,
    GelloRosLeaderConfigBase,
)

logger = logging.getLogger(__name__)


class BiGelloRosLeader(Teleoperator):
    """Bimanual GELLO ROS teleoperator.

    Wraps two GelloRosLeader instances (left + right arm).
    All action keys are prefixed with 'left_' or 'right_'.
    Runs in passive/bypass mode and records actions without commanding the robot.
    """

    config_class = BiGelloRosLeaderConfig
    name = "bi_gello_ros_leader"
    is_passive = True

    def __init__(self, config: BiGelloRosLeaderConfig | None = None):
        self.config = config or BiGelloRosLeaderConfig()
        super().__init__(self.config)

        self.left_arm = GelloRosLeader(self._arm_config(self.config.left_arm_config, id_suffix="left"))
        self.right_arm = GelloRosLeader(
            self._arm_config(self.config.right_arm_config, id_suffix="right")
        )

    def _arm_config(
        self,
        arm_config: GelloRosLeaderConfigBase,
        *,
        id_suffix: str,
    ) -> GelloRosLeaderConfig:
        values = {field.name: getattr(arm_config, field.name) for field in fields(GelloRosLeaderConfigBase)}
        return GelloRosLeaderConfig(
            id=f"{self.config.id}_{id_suffix}" if self.config.id else None,
            calibration_dir=self.config.calibration_dir,
            **values,
        )

    @staticmethod
    def _prefix_items(values: dict[str, Any], prefix: str) -> dict[str, Any]:
        return {f"{prefix}_{key}": value for key, value in values.items()}

    # --- Feature definitions ---

    @property
    def action_features(self) -> dict[str, type]:
        features: dict[str, type] = {}
        features.update(self._prefix_items(self.left_arm.action_features, "left"))
        features.update(self._prefix_items(self.right_arm.action_features, "right"))
        return features

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    # --- Lifecycle ---

    @property
    def is_connected(self) -> bool:
        return self.left_arm.is_connected and self.right_arm.is_connected

    def connect(self) -> None:
        if self.is_connected:
            return
        try:
            logger.info("Connecting left Gello...")
            self.left_arm.connect()
            logger.info("Connecting right Gello...")
            self.right_arm.connect()
            logger.info("Bimanual Gello connected")
        except Exception:
            logger.exception("Bimanual Gello connection failed; cleaning up connected listeners")
            self.disconnect()
            raise

    def disconnect(self) -> None:
        logger.info("Disconnecting left Gello...")
        self.left_arm.disconnect()
        logger.info("Disconnecting right Gello...")
        self.right_arm.disconnect()

    # --- Calibration (no-op) ---

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        return

    def configure(self) -> None:
        return

    def send_feedback(self, feedback: dict) -> None:
        return

    # --- Action ---

    def get_action(self) -> RobotAction:
        left_action = self.left_arm.get_action()
        right_action = self.right_arm.get_action()

        action: RobotAction = {}
        action.update(self._prefix_items(left_action, "left"))
        action.update(self._prefix_items(right_action, "right"))
        return action
