import sys
from pathlib import Path
# 把项目的根目录临时加到环境变量里，方便读取 src 下的包
sys.path.append(str(Path(__file__).resolve().parent.parent))

#!/usr/bin/env python3
"""Quick camera connectivity test using LeRobot camera configs.

This script validates that cameras defined in FrankaFr3RobotiqGripperConfig
can be connected and read through LeRobot's camera abstraction.
"""

import argparse
import time


def load_config_class():
    """Load config class with a fallback for direct script execution."""
    try:
        from src.robots.franka_fr3_robotiq_gripper.config_franka_fr3_robotiq_gripper import (
            FrankaFr3RobotiqGripperConfig,
        )

        return FrankaFr3RobotiqGripperConfig
    except ImportError:
        from config_franka_fr3_robotiq_gripper import FrankaFr3RobotiqGripperConfig

        return FrankaFr3RobotiqGripperConfig


def summarize_frame(frame):
    """Return a short human-readable summary for a frame object."""
    shape = getattr(frame, "shape", None)
    dtype = getattr(frame, "dtype", None)
    if shape is not None and dtype is not None:
        return f"shape={tuple(shape)}, dtype={dtype}"
    return f"type={type(frame)}"


def main() -> int:
    parser = argparse.ArgumentParser(description="Test LeRobot camera read for Franka config.")
    parser.add_argument("--frames", type=int, default=10, help="Number of frames to read per camera.")
    parser.add_argument(
        "--interval",
        type=float,
        default=0.1,
        help="Sleep interval (seconds) between reads.",
    )
    args = parser.parse_args()

    from lerobot.cameras.utils import make_cameras_from_configs

    ConfigClass = load_config_class()
    config = ConfigClass()
    cameras = make_cameras_from_configs(config.cameras)

    print("=== LeRobot Camera Test ===")
    print(f"Configured cameras: {list(cameras.keys())}")

    try:
        print("Connecting cameras...")
        for name, cam in cameras.items():
            cam.connect()
            print(f"[OK] Connected: {name}")

        print("Reading frames...")
        for i in range(args.frames):
            for name, cam in cameras.items():
                frame = cam.read()
                print(f"frame {i + 1:03d} | {name}: {summarize_frame(frame)}")
            time.sleep(max(args.interval, 0.0))

        print("[PASS] Camera read test completed.")
        return 0

    except Exception as exc:
        print(f"[FAIL] Camera test failed: {exc}")
        return 1

    finally:
        print("Disconnecting cameras...")
        for name, cam in cameras.items():
            try:
                cam.disconnect()
                print(f"[OK] Disconnected: {name}")
            except Exception as exc:
                print(f"[WARN] Failed to disconnect {name}: {exc}")


if __name__ == "__main__":
    raise SystemExit(main())
