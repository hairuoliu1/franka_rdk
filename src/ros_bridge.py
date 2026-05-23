#!/usr/bin/env python3.10
"""
ROS 2 bridge: subscribes to arm state, gripper state, wrench, and Gello teleop
topics on the given namespace, then writes the latest state as JSON to a shared
memory file every tick.

Usage:
    python3.10 src/ros_bridge.py --namespace left --fps 30
"""

import argparse
import json
import os
import signal
import time

from geometry_msgs.msg import WrenchStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

ARM_JOINT_NAMES = tuple(f"fr3_joint{i}" for i in range(1, 8))
WRENCH_AXES = ("fx", "fy", "fz", "tx", "ty", "tz")
ROBOTIQ_MAX_CLOSED_POSITION = 0.085


class LerobotRosBridge(Node):
    def __init__(self, namespace: str):
        ns = namespace.strip("/") if namespace else ""
        node_name = f"lerobot_ros_bridge_{ns}" if ns else "lerobot_ros_bridge"
        super().__init__(node_name)
        prefix = f"/{ns}" if ns else ""
        now = time.time()

        self._arm_msg: JointState | None = None
        self._gripper_msg: JointState | None = None
        self._wrench_msg: WrenchStamped | None = None
        self._gello_arm_msg: JointState | None = None
        self._gello_gripper_percent_msg: Float32 | None = None
        self._gello_gripper_raw_msg: Float32 | None = None
        self._namespace = ns
        self._started_at = now
        self._counts = {
            "arm": 0,
            "gripper": 0,
            "wrench": 0,
            "gello_arm": 0,
            "gello_gripper": 0,
        }
        self._last_seen = {key: None for key in self._counts}

        # Robot state topics
        self.create_subscription(
            JointState,
            f"{prefix}/franka/joint_states",
            self._arm_cb,
            10,
        )
        self.create_subscription(
            JointState,
            f"{prefix}/gripper/joint_states",
            self._gripper_cb,
            10,
        )
        self.create_subscription(
            WrenchStamped,
            f"{prefix}/franka_robot_state_broadcaster/external_wrench_in_base_frame",
            self._wrench_cb,
            10,
        )

        # Gello teleop topics
        self.create_subscription(
            JointState,
            f"{prefix}/gello/joint_states",
            self._gello_arm_cb,
            10,
        )
        self.create_subscription(
            Float32,
            f"{prefix}/gripper/gripper_client/target_gripper_width_percent",
            self._gello_gripper_percent_cb,
            10,
        )
        self.create_subscription(
            Float32,
            f"{prefix}/gello/gripper_position",
            self._gello_gripper_raw_cb,
            10,
        )

        self._state_joint_prefix = f"{ns}_" if ns else ""

        self.get_logger().info(
            f"Bridge started: ns={ns or 'none'}, prefix={prefix}, gripper=raw"
        )

    def _mark_seen(self, source: str) -> None:
        self._counts[source] += 1
        self._last_seen[source] = time.time()

    def _arm_cb(self, msg: JointState) -> None:
        self._arm_msg = msg
        self._mark_seen("arm")

    def _gripper_cb(self, msg: JointState) -> None:
        self._gripper_msg = msg
        self._mark_seen("gripper")

    def _wrench_cb(self, msg: WrenchStamped) -> None:
        self._wrench_msg = msg
        self._mark_seen("wrench")

    def _gello_arm_cb(self, msg: JointState) -> None:
        self._gello_arm_msg = msg
        self._mark_seen("gello_arm")

    def _gello_gripper_percent_cb(self, msg: Float32) -> None:
        self._gello_gripper_percent_msg = Float32(data=msg.data)
        self._mark_seen("gello_gripper")

    def _gello_gripper_raw_cb(self, msg: Float32) -> None:
        self._gello_gripper_raw_msg = Float32(data=msg.data)
        self._mark_seen("gello_gripper")

    def _read_joint_positions(
        self,
        msg: JointState | None,
        *,
        allow_state_prefix: bool,
    ) -> list[float]:
        if msg is None or not msg.name:
            return [0.0] * len(ARM_JOINT_NAMES)

        positions_by_name = dict(zip(msg.name, msg.position))
        positions = []
        for joint_name in ARM_JOINT_NAMES:
            candidate_names = [joint_name]
            if allow_state_prefix and self._state_joint_prefix:
                candidate_names.insert(0, f"{self._state_joint_prefix}{joint_name}")
            positions.append(
                next(
                    (
                        float(positions_by_name[name])
                        for name in candidate_names
                        if name in positions_by_name
                    ),
                    0.0,
                )
            )
        return positions

    def _read_arm_joints(self) -> list[float]:
        """Extract 7 arm joint positions from the latest state message.

        This must represent Franka state only. Do not fall back to Gello action,
        otherwise a missing robot state topic can be silently recorded as state.
        """
        return self._read_joint_positions(self._arm_msg, allow_state_prefix=True)

    def _read_gripper(self) -> float:
        if self._gripper_msg is not None and self._gripper_msg.position:
            return float(self._gripper_msg.position[0])
        return 0.0

    def _read_wrench(self) -> dict[str, float]:
        if self._wrench_msg is None:
            return {f"wrench_{axis}": 0.0 for axis in WRENCH_AXES}
        f = self._wrench_msg.wrench.force
        t = self._wrench_msg.wrench.torque
        return {
            "wrench_fx": float(f.x),
            "wrench_fy": float(f.y),
            "wrench_fz": float(f.z),
            "wrench_tx": float(t.x),
            "wrench_ty": float(t.y),
            "wrench_tz": float(t.z),
        }

    def _read_gello_arm(self) -> list[float]:
        """Extract 7 arm joint positions from the latest Gello message."""
        return self._read_joint_positions(self._gello_arm_msg, allow_state_prefix=False)

    def _read_gello_gripper(self) -> float:
        if self._gello_gripper_raw_msg is not None:
            return float(self._gello_gripper_raw_msg.data)
        if self._gello_gripper_percent_msg is not None:
            open_percent = max(0.0, min(1.0, float(self._gello_gripper_percent_msg.data)))
            return ROBOTIQ_MAX_CLOSED_POSITION * (1.0 - open_percent)
        return 0.0

    def get_state(self) -> dict:
        """Return the full observation + action snapshot as a dict."""
        now = time.time()
        arm = self._read_arm_joints()
        gripper = self._read_gripper()
        gello_arm = self._read_gello_arm()
        gello_gripper = self._read_gello_gripper()

        state = {
            "observation": {
                **{f"joint_positions_{i}": arm[i] for i in range(len(ARM_JOINT_NAMES))},
                "joint_positions_7": gripper,
                **self._read_wrench(),
            },
            "action": {
                **{f"joint_positions_{i}": gello_arm[i] for i in range(len(ARM_JOINT_NAMES))},
                "joint_positions_7": gello_gripper,
            },
            "metadata": {
                "namespace": self._namespace,
                "started_at": self._started_at,
                "timestamp": now,
                "gripper_convention": "raw_robotiq_position",
                "counts": dict(self._counts),
                "received": {key: count > 0 for key, count in self._counts.items()},
                "age_s": {
                    key: None if last is None else now - last
                    for key, last in self._last_seen.items()
                },
            },
        }
        return state


def main():
    parser = argparse.ArgumentParser(description="ROS 2 → LeRobot bridge")
    parser.add_argument("--namespace", default="left", help="Topic namespace (left, right)")
    parser.add_argument("--fps", type=int, default=30, help="Output rate")
    parser.add_argument(
        "--output",
        default=None,
        help="Output file path (default: /dev/shm/lerobot_state_<ns>.json)",
    )
    args = parser.parse_args()

    rclpy.init()

    bridge = LerobotRosBridge(args.namespace)
    out_path = args.output or f"/dev/shm/lerobot_state_{args.namespace}.json"
    period = 1.0 / args.fps

    print(f"Bridge writing to {out_path} @ {args.fps} Hz", flush=True)

    running = True

    def _shutdown(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        while running and rclpy.ok():
            rclpy.spin_once(bridge, timeout_sec=0.01)
            state = bridge.get_state()
            # Atomic write: write to temp file, then rename
            tmp_path = out_path + ".tmp"
            with open(tmp_path, "w") as f:
                json.dump(state, f)
            os.replace(tmp_path, out_path)
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()
        # Clean up
        for p in (out_path, out_path + ".tmp"):
            try:
                os.unlink(p)
            except OSError:
                pass


if __name__ == "__main__":
    main()
