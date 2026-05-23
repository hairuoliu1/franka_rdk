# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import time
from pathlib import Path
from threading import Event, Thread

import draccus
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

from lerobot.motors import MotorCalibration
from lerobot.types import RobotAction, RobotObservation
from lerobot.robots.robot import Robot
from lerobot.cameras.utils import make_cameras_from_configs

from .config_franka_fr3_robotiq_gripper import FrankaFr3RobotiqGripperConfig

logger = logging.getLogger(__name__)


class FrankaFr3RobotiqGripper(Robot):
    """
    The base abstract class for all LeRobot-compatible robots.

    This class provides a standardized interface for interacting with physical robots.
    Subclasses must implement all abstract methods and properties to be usable.

    Attributes:
        config_class (RobotConfig): The expected configuration class for this robot.
        name (str): The unique robot name used to identify this robot type.
    """

    config_class = FrankaFr3RobotiqGripperConfig
    name = "franka_fr3_robotiq_gripper"

    def __init__(self, config: FrankaFr3RobotiqGripperConfig):
        super().__init__(config)
        self.config = config
        self.num_dofs = 8
        self._connected = False
        self._joint_state_msg: JointState | None = None
        self._gripper_state_msg: JointState | None = None
        self._joint_sub = None
        self._gripper_sub = None
        self._arm_pub = None
        self._gripper_pub = None
        self._node: Node | None = None
        self._spin_stop = Event()
        self._spin_thread: Thread | None = None
        self._last_sent_action = {f"joint_positions_{i}": 0.0 for i in range(self.num_dofs)}
        self.cameras = make_cameras_from_configs(config.cameras)

    def __str__(self) -> str:
        return f"{self.id} {self.__class__.__name__}"

    def __enter__(self):
        """
        Context manager entry.
        Automatically connects to the camera.
        """
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        """
        Context manager exit.
        Automatically disconnects, ensuring resources are released even on error.
        """
        self.disconnect()

    def __del__(self) -> None:
        """
        Destructor safety net.
        Attempts to disconnect if the object is garbage collected without cleanup.
        """
        try:
            if self.is_connected:
                self.disconnect()
        except Exception:  # nosec B110
            pass

    # 需要验证
    def _joint_callback(self, msg: JointState) -> None:
        self._joint_state_msg = msg

    def _gripper_joint_callback(self, msg: JointState) -> None:
        self._gripper_state_msg = msg

    def _get_gripper_observation(self) -> float:
        """Read gripper state from joint states topic with fallback to last commanded value."""
        if self._gripper_state_msg is not None and self._gripper_state_msg.position:
            idx = int(getattr(self.config, "gripper_state_joint_index", 0))
            if 0 <= idx < len(self._gripper_state_msg.position):
                return float(self._gripper_state_msg.position[idx])

        return float(self._last_sent_action.get("joint_positions_7", 0.0))

    def _spin_ros(self) -> None:
        while not self._spin_stop.is_set() and rclpy.ok() and self._node is not None:
            rclpy.spin_once(self._node, timeout_sec=0.05)

    @property
    def observation_features(self) -> dict:
        """
        A dictionary describing the structure and types of the observations produced by the robot.
        Its structure (keys) should match the structure of what is returned by :pymeth:`get_observation`.
        Values for the dict should either be:
            - The type of the value if it's a simple value, e.g. `float` for single proprioceptive value (a joint's position/velocity)
            - A tuple representing the shape if it's an array-type value, e.g. `(height, width, channel)` for images

        Note: this property should be able to be called regardless of whether the robot is connected or not.
        """
        features = {f"joint_positions_{i}": float for i in range(8)}
        if hasattr(self.config, "cameras") and self.config.cameras:
            for cam_name, cam_cfg in self.config.cameras.items():
                features[cam_name] = (cam_cfg.height, cam_cfg.width, 3)
        return features

    @property
    def action_features(self) -> dict:
        """
        A dictionary describing the structure and types of the actions expected by the robot. Its structure
        (keys) should match the structure of what is passed to :pymeth:`send_action`. Values for the dict
        should be the type of the value if it's a simple value, e.g. `float` for single proprioceptive value
        (a joint's goal position/velocity)

        Note: this property should be able to be called regardless of whether the robot is connected or not.
        """
        return {f"joint_positions_{i}": float for i in range(8)}

    @property
    def is_connected(self) -> bool:
        """
        Whether the robot is currently connected or not. If `False`, calling :pymeth:`get_observation` or
        :pymeth:`send_action` should raise an error.
        """
        return self._connected

    def connect(self, calibrate: bool = True) -> None:
        """
        Establish communication with the robot.

        Args:
            calibrate (bool): If True, automatically calibrate the robot after connecting if it's not
                calibrated or needs calibration (this is hardware-dependant).
        """
        if self._connected:
            return

        if not rclpy.ok():
            rclpy.init()

        self._node = Node("franka_fr3_robotiq_gripper")
        self._joint_sub = self._node.create_subscription(
            JointState,
            self.config.arm_state_topic,
            self._joint_callback,
            10,
        )
        self._gripper_sub = self._node.create_subscription(
            JointState,
            self.config.gripper_state_topic,
            self._gripper_joint_callback,
            10,
        )
        self._arm_pub = self._node.create_publisher(JointState, self.config.arm_command_topic, 10)
        self._gripper_pub = self._node.create_publisher(
            Float32,
            self.config.gripper_command_topic,
            10,
        )

        for cam in self.cameras.values():
            cam.connect()

        self._spin_stop.clear()
        self._spin_thread = Thread(target=self._spin_ros, name="franka_ros_spin", daemon=True)
        self._spin_thread.start()

        t0 = time.time()
        # 允许夹爪启动不依赖机械臂状态，放宽连接检查，没接到joint_state也不阻塞
        while self._gripper_state_msg is None and time.time() - t0 < 2.0:
            time.sleep(0.02)

        self._connected = True

    @property
    def is_calibrated(self) -> bool:
        """Whether the robot is currently calibrated or not. Should be always `True` if not applicable"""
        return True

    def calibrate(self) -> None:
        """
        Calibrate the robot if applicable. If not, this should be a no-op.

        This method should collect any necessary data (e.g., motor offsets) and update the
        :pyattr:`calibration` dictionary accordingly.
        """
        return

    def _load_calibration(self, fpath: Path | None = None) -> None:
        """
        Helper to load calibration data from the specified file.

        Args:
            fpath (Path | None): Optional path to the calibration file. Defaults to `self.calibration_fpath`.
        """
        fpath = self.calibration_fpath if fpath is None else fpath
        with open(fpath) as f, draccus.config_type("json"):
            self.calibration = draccus.load(dict[str, MotorCalibration], f)

    def _save_calibration(self, fpath: Path | None = None) -> None:
        """
        Helper to save calibration data to the specified file.

        Args:
            fpath (Path | None): Optional path to save the calibration file. Defaults to `self.calibration_fpath`.
        """
        fpath = self.calibration_fpath if fpath is None else fpath
        with open(fpath, "w") as f, draccus.config_type("json"):
            draccus.dump(self.calibration, f, indent=4)

    def configure(self) -> None:
        """
        Apply any one-time or runtime configuration to the robot.
        This may include setting motor parameters, control modes, or initial state.
        """
        return

    def get_observation(self) -> RobotObservation:
        """
        Retrieve the current observation from the robot.

        Returns:
            RobotObservation: A flat dictionary representing the robot's current sensory state. Its structure
                should match :pymeth:`observation_features`.
        """

        if not self._connected:
            raise RuntimeError("Robot is not connected. Call connect() first.")

        if self._joint_state_msg is None:
            arm = np.zeros(7, dtype=np.float32)
        else:
            arm_positions = []
            for i in range(1, 8):
                joint_name = f"fr3_joint{i}"
                if joint_name in self._joint_state_msg.name:
                    idx = self._joint_state_msg.name.index(joint_name)
                    arm_positions.append(self._joint_state_msg.position[idx])
                else:
                    arm_positions.append(0.0)
            arm = np.array(arm_positions, dtype=np.float32)

        gripper = self._get_gripper_observation()
        joints = np.append(arm, gripper)

        obs: RobotObservation = {}
        for i, value in enumerate(joints):
            obs[f"joint_positions_{i}"] = float(value)

        for cam_name, cam in self.cameras.items():
            obs[cam_name] = cam.read()

        return obs

    def send_action(self, action: RobotAction) -> RobotAction:
        """
        Send an action command to the robot.

        Args:
            action (RobotAction): Dictionary representing the desired action. Its structure should match
                :pymeth:`action_features`.

        Returns:
            RobotAction: The action actually sent to the motors potentially clipped or modified, e.g. by
                safety limits on velocity.
        """
        if not self._connected or self._node is None or self._arm_pub is None:
            raise RuntimeError("Robot is not connected. Call connect() first.")

        arm = np.array([float(action[f"joint_positions_{i}"]) for i in range(7)], dtype=np.float32)

        arm_msg = JointState()
        arm_msg.header.stamp = self._node.get_clock().now().to_msg()
        arm_msg.header.frame_id = "fr3_link0"
        arm_msg.name = [
            "fr3_joint1",
            "fr3_joint2",
            "fr3_joint3",
            "fr3_joint4",
            "fr3_joint5",
            "fr3_joint6",
            "fr3_joint7",
        ]
        arm_msg.position = [float(v) for v in arm]
        self._arm_pub.publish(arm_msg)

        sent: RobotAction = {f"joint_positions_{i}": float(arm[i]) for i in range(7)}
        if self.config.use_gripper:
            raw_gripper = float(action.get("joint_positions_7", self._last_sent_action["joint_positions_7"]))
            max_closed = getattr(self.config, "gripper_max_closed_position", None)
            if max_closed is not None:
                raw_gripper = float(np.clip(raw_gripper, 0.0, float(max_closed)))
            if self._gripper_pub is not None:
                g_msg = Float32()
                if max_closed is not None and float(max_closed) > 0.0:
                    g_msg.data = 1.0 - (raw_gripper / float(max_closed))
                else:
                    g_msg.data = raw_gripper
                self._gripper_pub.publish(g_msg)
            sent["joint_positions_7"] = raw_gripper

        self._last_sent_action.update(sent)
        return sent

    def disconnect(self) -> None:
        """Disconnect from the robot and perform any necessary cleanup."""
        if not self._connected:
            return

        for cam in self.cameras.values():
            cam.disconnect()

        self._spin_stop.set()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=1.0)

        if self._node is not None:
            self._node.destroy_node()
            self._node = None

        self._joint_sub = None
        self._gripper_sub = None
        self._arm_pub = None
        self._gripper_pub = None
        self._connected = False
