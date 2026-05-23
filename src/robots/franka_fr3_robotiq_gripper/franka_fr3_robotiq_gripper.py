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
import os
import time
from pathlib import Path
from threading import Event, Thread

import draccus
import numpy as np

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.motors import MotorCalibration
from lerobot.robots.robot import Robot
from lerobot.types import RobotAction, RobotObservation

from .config_franka_fr3_robotiq_gripper import FrankaFr3RobotiqGripperConfig

logger = logging.getLogger(__name__)

ARM_JOINT_NAMES = tuple(f"fr3_joint{i}" for i in range(1, 8))
ROBOT_DOF = len(ARM_JOINT_NAMES) + 1
WRENCH_AXES = ("fx", "fy", "fz", "tx", "ty", "tz")
CAMERA_MAX_AGE_MS = 500


def _namespaced(base_topic: str, namespace: str) -> str:
    """Prepend namespace to a topic path."""
    if not namespace:
        return base_topic
    ns = namespace.strip("/")
    topic = base_topic.lstrip("/")
    return f"/{ns}/{topic}"


class FrankaFr3RobotiqGripper(Robot):
    """LeRobot-compatible robot for Franka FR3 with Robotiq gripper.

    Supports two modes:
    - Direct ROS 2 (rclpy): requires Python 3.10 (ROS 2 Humble ABI).
    - Bridge mode: uses a Python 3.10 subprocess for ROS, communicates via
      shared memory file. Required for Python 3.12+.
    """

    config_class = FrankaFr3RobotiqGripperConfig
    name = "franka_fr3_robotiq_gripper"

    def __init__(self, config: FrankaFr3RobotiqGripperConfig):
        super().__init__(config)
        self.config = config
        self.num_dofs = ROBOT_DOF
        self._connected = False

        # Bridge mode
        self._use_bridge = getattr(self.config, "use_bridge", True)
        self._bridge_client = None

        # Direct ROS mode (requires Python 3.10)
        self._joint_state_msg = None
        self._gripper_state_msg = None
        self._wrench_msg = None
        self._joint_sub = None
        self._gripper_sub = None
        self._wrench_sub = None
        self._arm_pub = None
        self._gripper_pub = None
        self._node = None
        self._spin_stop = Event()
        self._spin_thread = None

        self._last_sent_action = {f"joint_positions_{i}": 0.0 for i in range(self.num_dofs)}
        self.cameras = make_cameras_from_configs(config.cameras)

        # Resolve namespace and joint name prefix
        self._ns = getattr(self.config, "topic_namespace", "")
        prefix = getattr(self.config, "state_joint_prefix", "")
        if not prefix and self._ns:
            prefix = self._ns.rstrip("_") + "_"
        self._state_joint_prefix = prefix

    def __str__(self) -> str:
        return f"{self.id} {self.__class__.__name__}"

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.disconnect()

    def __del__(self) -> None:
        try:
            if self.is_connected:
                self.disconnect()
        except Exception:
            pass

    # --- Bridge mode helpers ---

    def _connect_bridge(self):
        from src.ros_bridge_client import get_bridge_client

        domain = getattr(self.config, "ros_domain_id", None)
        if domain is not None:
            os.environ["ROS_DOMAIN_ID"] = str(domain)
        self._bridge_client = get_bridge_client(namespace=self._ns)
        connected_cameras = []
        try:
            self._bridge_client.connect()
            for cam in self.cameras.values():
                cam.connect()
                connected_cameras.append(cam)
            self._connected = True
        except Exception:
            for cam in reversed(connected_cameras):
                try:
                    cam.disconnect()
                except Exception:
                    pass
            if self._bridge_client is not None:
                self._bridge_client.disconnect()
                self._bridge_client = None
            raise

    def _get_obs_bridge(self) -> RobotObservation:
        obs = dict(self._bridge_client.get_observation())
        for cam_name, cam in self.cameras.items():
            obs[cam_name] = self._read_camera_frame(cam)
        return obs

    def _send_action_bridge(self, action: RobotAction) -> RobotAction:
        # In bridge mode we only record, never command the robot.
        sent = {
            f"joint_positions_{i}": float(action[f"joint_positions_{i}"])
            for i in range(ROBOT_DOF)
        }
        self._last_sent_action.update(sent)
        return sent

    def _disconnect_bridge(self):
        if self._bridge_client is not None:
            self._bridge_client.disconnect()
            self._bridge_client = None

    # --- Direct ROS mode ---

    def _joint_callback(self, msg):
        self._joint_state_msg = msg

    def _gripper_joint_callback(self, msg):
        self._gripper_state_msg = msg

    def _wrench_callback(self, msg):
        self._wrench_msg = msg

    def _spin_ros(self) -> None:
        import rclpy
        while not self._spin_stop.is_set() and rclpy.ok() and self._node is not None:
            rclpy.spin_once(self._node, timeout_sec=0.05)

    def _get_gripper_observation(self) -> float:
        if self._gripper_state_msg is not None and self._gripper_state_msg.position:
            idx = int(getattr(self.config, "gripper_state_joint_index", 0))
            if 0 <= idx < len(self._gripper_state_msg.position):
                return float(self._gripper_state_msg.position[idx])
        return float(self._last_sent_action.get("joint_positions_7", 0.0))

    def _read_arm_observation(self) -> np.ndarray:
        if self._joint_state_msg is None:
            return np.zeros(len(ARM_JOINT_NAMES), dtype=np.float32)

        positions_by_name = dict(zip(self._joint_state_msg.name, self._joint_state_msg.position))
        arm_positions = []
        for joint_name in ARM_JOINT_NAMES:
            prefixed_name = f"{self._state_joint_prefix}{joint_name}"
            value = positions_by_name.get(prefixed_name, positions_by_name.get(joint_name, 0.0))
            arm_positions.append(value)
        return np.array(arm_positions, dtype=np.float32)

    def _read_camera_frame(self, cam):
        try:
            return cam.read_latest(max_age_ms=CAMERA_MAX_AGE_MS)
        except (NotImplementedError, AttributeError, TimeoutError, RuntimeError):
            return cam.read()

    # --- Public interface ---

    @property
    def observation_features(self) -> dict:
        features = {f"joint_positions_{i}": float for i in range(ROBOT_DOF)}
        if getattr(self.config, "use_ft_sensor", False):
            for w in WRENCH_AXES:
                features[f"wrench_{w}"] = float
        if hasattr(self.config, "cameras") and self.config.cameras:
            for cam_name, cam_cfg in self.config.cameras.items():
                features[cam_name] = (cam_cfg.height, cam_cfg.width, 3)
        return features

    @property
    def action_features(self) -> dict:
        return {f"joint_positions_{i}": float for i in range(ROBOT_DOF)}

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self, calibrate: bool = True) -> None:
        if self._connected:
            return

        if self._use_bridge:
            self._connect_bridge()
            return

        # Direct ROS 2 mode
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState as JointStateMsg
        from geometry_msgs.msg import WrenchStamped as WrenchStampedMsg
        from std_msgs.msg import Float32 as Float32Msg

        if not rclpy.ok():
            domain = getattr(self.config, "ros_domain_id", None)
            if domain is not None:
                os.environ["ROS_DOMAIN_ID"] = str(domain)
            rclpy.init()

        ns = self._ns
        arm_state_topic = _namespaced(self.config.arm_state_topic, ns)
        gripper_state_topic = _namespaced(self.config.gripper_state_topic, ns)
        arm_command_topic = _namespaced(self.config.arm_command_topic, ns)
        gripper_command_topic = _namespaced(self.config.gripper_command_topic, ns)

        node_name = f"franka_fr3_{ns}" if ns else "franka_fr3_robotiq_gripper"
        self._node = Node(node_name)

        self._joint_sub = self._node.create_subscription(
            JointStateMsg,
            arm_state_topic,
            self._joint_callback,
            10,
        )
        self._gripper_sub = self._node.create_subscription(
            JointStateMsg,
            gripper_state_topic,
            self._gripper_joint_callback,
            10,
        )
        if getattr(self.config, "use_ft_sensor", False):
            ft_topic = _namespaced(self.config.ft_sensor_topic, ns)
            self._wrench_sub = self._node.create_subscription(
                WrenchStampedMsg,
                ft_topic,
                self._wrench_callback,
                10,
            )

        self._arm_pub = self._node.create_publisher(JointStateMsg, arm_command_topic, 10)
        self._gripper_pub = self._node.create_publisher(Float32Msg, gripper_command_topic, 10)

        logger.info(
            "Connecting %s: ns=%s, arm_state=%s, arm_cmd=%s, gripper_state=%s, gripper_cmd=%s",
            self.name,
            ns or "none",
            arm_state_topic,
            arm_command_topic,
            gripper_state_topic,
            gripper_command_topic,
        )

        for cam in self.cameras.values():
            cam.connect()

        self._spin_stop.clear()
        self._spin_thread = Thread(target=self._spin_ros, name="franka_ros_spin", daemon=True)
        self._spin_thread.start()

        t0 = time.time()
        while self._joint_state_msg is None and time.time() - t0 < 3.0:
            time.sleep(0.02)

        self._connected = True

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        return

    def _load_calibration(self, fpath: Path | None = None) -> None:
        fpath = self.calibration_fpath if fpath is None else fpath
        with open(fpath) as f, draccus.config_type("json"):
            self.calibration = draccus.load(dict[str, MotorCalibration], f)

    def _save_calibration(self, fpath: Path | None = None) -> None:
        fpath = self.calibration_fpath if fpath is None else fpath
        with open(fpath, "w") as f, draccus.config_type("json"):
            draccus.dump(self.calibration, f, indent=4)

    def configure(self) -> None:
        return

    def get_observation(self) -> RobotObservation:
        if not self._connected:
            raise RuntimeError("Robot is not connected. Call connect() first.")

        if self._use_bridge:
            return self._get_obs_bridge()

        # Direct ROS mode
        arm = self._read_arm_observation()
        gripper = self._get_gripper_observation()
        joints = np.append(arm, gripper)

        obs: RobotObservation = {}
        for i, value in enumerate(joints):
            obs[f"joint_positions_{i}"] = float(value)

        if getattr(self.config, "use_ft_sensor", False):
            if self._wrench_msg is None:
                for w in WRENCH_AXES:
                    obs[f"wrench_{w}"] = 0.0
            else:
                f = self._wrench_msg.wrench.force
                t = self._wrench_msg.wrench.torque
                obs["wrench_fx"] = float(f.x)
                obs["wrench_fy"] = float(f.y)
                obs["wrench_fz"] = float(f.z)
                obs["wrench_tx"] = float(t.x)
                obs["wrench_ty"] = float(t.y)
                obs["wrench_tz"] = float(t.z)

        for cam_name, cam in self.cameras.items():
            obs[cam_name] = self._read_camera_frame(cam)

        return obs

    def send_action(self, action: RobotAction) -> RobotAction:
        if not self._connected:
            raise RuntimeError("Robot is not connected. Call connect() first.")

        if self._use_bridge:
            return self._send_action_bridge(action)

        # Direct ROS mode
        if self._node is None or self._arm_pub is None:
            raise RuntimeError("Robot is not connected. Call connect() first.")

        from sensor_msgs.msg import JointState as JointStateMsg
        from std_msgs.msg import Float32 as Float32Msg

        arm = np.array(
            [float(action[f"joint_positions_{i}"]) for i in range(len(ARM_JOINT_NAMES))],
            dtype=np.float32,
        )

        arm_msg = JointStateMsg()
        arm_msg.header.stamp = self._node.get_clock().now().to_msg()
        arm_msg.header.frame_id = "fr3_link0"
        arm_msg.name = list(ARM_JOINT_NAMES)
        arm_msg.position = [float(v) for v in arm]
        self._arm_pub.publish(arm_msg)

        sent: RobotAction = {
            f"joint_positions_{i}": float(arm[i])
            for i in range(len(ARM_JOINT_NAMES))
        }
        if self.config.use_gripper:
            raw_gripper = float(action.get("joint_positions_7", self._last_sent_action["joint_positions_7"]))
            max_closed = getattr(self.config, "gripper_max_closed_position", None)
            if max_closed is not None:
                raw_gripper = float(np.clip(raw_gripper, 0.0, float(max_closed)))
            if self._gripper_pub is not None:
                g_msg = Float32Msg()
                if max_closed is not None and float(max_closed) > 0.0:
                    g_msg.data = 1.0 - (raw_gripper / float(max_closed))
                else:
                    g_msg.data = raw_gripper
                self._gripper_pub.publish(g_msg)
            sent["joint_positions_7"] = raw_gripper

        self._last_sent_action.update(sent)
        return sent

    def disconnect(self) -> None:
        if not self._connected:
            return

        for cam in self.cameras.values():
            cam.disconnect()

        if self._use_bridge:
            self._disconnect_bridge()
            self._connected = False
            return

        # Direct ROS mode cleanup
        self._spin_stop.set()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=1.0)

        if self._node is not None:
            self._node.destroy_node()
            self._node = None

        self._joint_sub = None
        self._gripper_sub = None
        self._wrench_sub = None
        self._arm_pub = None
        self._gripper_pub = None
        self._connected = False
