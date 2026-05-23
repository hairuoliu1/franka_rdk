import logging
from dataclasses import fields
from pathlib import Path
from typing import Any

from lerobot.robots.robot import Robot
from lerobot.types import RobotAction, RobotObservation

from .config_bi_franka_fr3_robotiq_gripper import BiFrankaFr3RobotiqGripperConfig
from src.robots.franka_fr3_robotiq_gripper.franka_fr3_robotiq_gripper import (
    FrankaFr3RobotiqGripper,
)
from src.robots.franka_fr3_robotiq_gripper.config_franka_fr3_robotiq_gripper import (
    FrankaFr3RobotiqGripperConfig,
    FrankaFr3RobotiqGripperConfigBase,
)

logger = logging.getLogger(__name__)


class BiFrankaFr3RobotiqGripper(Robot):
    """Bimanual Franka FR3 robot with Robotiq grippers.

    Wraps two FrankaFr3RobotiqGripper instances (left + right arm).
    Joint/state keys are prefixed with 'left_' or 'right_'.
    Camera keys pass through as-is. Users must give them unique names
    across both arms (e.g. 'left_wrist', 'right_wrist', 'top').
    """

    config_class = BiFrankaFr3RobotiqGripperConfig
    name = "bi_franka_fr3_robotiq_gripper"

    def __init__(self, config: BiFrankaFr3RobotiqGripperConfig):
        super().__init__(config)
        self.config = config

        self.left_arm = FrankaFr3RobotiqGripper(
            self._arm_config(config.left_arm_config, id_suffix="left")
        )
        self.right_arm = FrankaFr3RobotiqGripper(
            self._arm_config(config.right_arm_config, id_suffix="right")
        )

        # Merge cameras from both arms. Camera keys must be unique.
        self.cameras = {**self.left_arm.cameras, **self.right_arm.cameras}

    def _arm_config(
        self,
        arm_config: FrankaFr3RobotiqGripperConfigBase,
        *,
        id_suffix: str,
    ) -> FrankaFr3RobotiqGripperConfig:
        values = {
            field.name: getattr(arm_config, field.name)
            for field in fields(FrankaFr3RobotiqGripperConfigBase)
        }
        return FrankaFr3RobotiqGripperConfig(
            id=f"{self.config.id}_{id_suffix}" if self.config.id else None,
            calibration_dir=self.config.calibration_dir,
            **values,
        )

    def _is_camera(self, key: str, arm: FrankaFr3RobotiqGripper) -> bool:
        return key in arm.cameras

    def _prefix_arm_items(
        self,
        values: dict[str, Any],
        *,
        arm: FrankaFr3RobotiqGripper,
        prefix: str,
    ) -> dict[str, Any]:
        return {
            key if self._is_camera(key, arm) else f"{prefix}_{key}": value
            for key, value in values.items()
        }

    @staticmethod
    def _prefix_items(values: dict[str, Any], prefix: str) -> dict[str, Any]:
        return {f"{prefix}_{key}": value for key, value in values.items()}

    @staticmethod
    def _strip_prefixed_items(values: dict[str, Any], prefix: str) -> dict[str, Any]:
        key_prefix = f"{prefix}_"
        return {
            key[len(key_prefix) :]: value
            for key, value in values.items()
            if key.startswith(key_prefix)
        }

    # --- Feature definitions ---

    @property
    def observation_features(self) -> dict[str, Any]:
        features: dict[str, Any] = {}
        features.update(
            self._prefix_arm_items(self.left_arm.observation_features, arm=self.left_arm, prefix="left")
        )
        features.update(
            self._prefix_arm_items(self.right_arm.observation_features, arm=self.right_arm, prefix="right")
        )
        return features

    @property
    def action_features(self) -> dict[str, Any]:
        features: dict[str, Any] = {}
        features.update(self._prefix_items(self.left_arm.action_features, "left"))
        features.update(self._prefix_items(self.right_arm.action_features, "right"))
        return features

    # --- Lifecycle ---

    @property
    def is_connected(self) -> bool:
        return self.left_arm.is_connected and self.right_arm.is_connected

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            return
        try:
            logger.info("Connecting left arm...")
            self.left_arm.connect()
            logger.info("Connecting right arm...")
            self.right_arm.connect()
            logger.info("Bimanual robot connected")
        except Exception:
            logger.exception("Bimanual robot connection failed; cleaning up connected arms")
            self.disconnect()
            raise

    def disconnect(self) -> None:
        logger.info("Disconnecting left arm...")
        self.left_arm.disconnect()
        logger.info("Disconnecting right arm...")
        self.right_arm.disconnect()

    # --- Calibration (no-op) ---

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        return

    def configure(self) -> None:
        return

    # --- Observation ---

    def get_observation(self) -> RobotObservation:
        left_obs = self.left_arm.get_observation()
        right_obs = self.right_arm.get_observation()

        obs: RobotObservation = {}
        obs.update(self._prefix_arm_items(left_obs, arm=self.left_arm, prefix="left"))
        obs.update(self._prefix_arm_items(right_obs, arm=self.right_arm, prefix="right"))
        return obs

    # --- Action ---

    def send_action(self, action: RobotAction) -> RobotAction:
        left_action = self._strip_prefixed_items(action, "left")
        right_action = self._strip_prefixed_items(action, "right")

        left_sent = self.left_arm.send_action(left_action)
        right_sent = self.right_arm.send_action(right_action)

        sent: RobotAction = {}
        sent.update(self._prefix_items(left_sent, "left"))
        sent.update(self._prefix_items(right_sent, "right"))
        return sent

    # --- Calibration persistence (no-op stubs) ---

    def _load_calibration(self, fpath: Path | None = None) -> None:
        return

    def _save_calibration(self, fpath: Path | None = None) -> None:
        return

    def __str__(self) -> str:
        return f"BimanualFrankaFR3(id={self.config.id})"
