#!/usr/bin/env python3
"""End-to-end recording test for bimanual Franka FR3 state and 3 cameras.

Default output root is LeRobot's default cache:
    $HF_LEROBOT_HOME/<repo_id>

Run:
    python test/test_recording.py

Useful overrides:
    python test/test_recording.py --ros-domain-id 42
    python test/test_recording.py --episode-s 3 --fps 15
    python test/test_recording.py --left-camera /dev/video0 --right-camera /dev/video2
    python test/test_recording.py --zed-serial 12345678
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import shutil
import sys
import tempfile
import time
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / "src", ROOT / "lerobot" / "src"):
    path_str = str(path)
    if path_str not in sys.path:
        sys.path.insert(0, path_str)

os.environ.setdefault("HF_HOME", "/tmp/lerobot_hf_home")
os.environ.setdefault("HF_DATASETS_CACHE", "/tmp/lerobot_hf_datasets_cache")

from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.datasets.feature_utils import build_dataset_frame, combine_feature_dicts
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.pipeline_features import aggregate_pipeline_dataset_features, create_initial_features
from lerobot.processor import make_default_processors
from lerobot.utils.constants import ACTION, HF_LEROBOT_HOME, OBS_STR

from src.robots.bi_franka_fr3_robotiq_gripper.bi_franka_fr3_robotiq_gripper import (
    BiFrankaFr3RobotiqGripper,
)
from src.robots.bi_franka_fr3_robotiq_gripper.config_bi_franka_fr3_robotiq_gripper import (
    BiFrankaFr3RobotiqGripperConfig,
)
from src.robots.franka_fr3_robotiq_gripper.config_franka_fr3_robotiq_gripper import (
    FrankaFr3RobotiqGripperConfig,
)
from src.teleoperators.bi_gello_leader.bi_gello_leader import BiGelloRosLeader
from src.teleoperators.bi_gello_leader.config_bi_gello_leader import BiGelloRosLeaderConfig
from src.teleoperators.gello_leader.config_gello_ros_leader import GelloRosLeaderConfig


LOGGER = logging.getLogger("test_recording")

DEFAULT_REPO_ID = "test/bimanual_franka_recording"
DEFAULT_TASK = "bimanual franka recording test"
ARM_JOINT_COUNT = 7
ROBOT_DOF = ARM_JOINT_COUNT + 1
GELLO_RECEIVED_KEYS = ("gello_arm", "gello_gripper")


def _parse_camera_id(value: str) -> int | str:
    return int(value) if value.isdigit() else value


def _parse_size(value: str) -> tuple[int, int]:
    try:
        width, height = value.lower().split("x", maxsplit=1)
        return int(width), int(height)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("Expected WIDTHxHEIGHT, for example 640x480") from exc


def _default_ros_domain() -> int:
    value = os.getenv("ROS_DOMAIN_ID", "0")
    try:
        return int(value)
    except ValueError:
        return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record one bimanual Franka FR3 test episode.")
    parser.add_argument("--repo-id", default=DEFAULT_REPO_ID, help="LeRobot dataset repo id.")
    parser.add_argument(
        "--root",
        type=Path,
        default=None,
        help="Optional dataset root. Omit this to use LeRobot's default path.",
    )
    parser.add_argument(
        "--keep-existing",
        action="store_true",
        help="Fail if the dataset root already exists. Default behavior replaces this test dataset.",
    )
    parser.add_argument("--task", default=DEFAULT_TASK, help="Task string stored in the dataset.")
    parser.add_argument("--episode-s", type=float, default=10.0, help="Recording duration in seconds.")
    parser.add_argument("--fps", type=int, default=30, help="Recording FPS.")
    parser.add_argument("--ros-domain-id", type=int, default=_default_ros_domain(), help="ROS_DOMAIN_ID.")
    parser.add_argument("--left-namespace", default="left", help="Left arm ROS namespace.")
    parser.add_argument("--right-namespace", default="right", help="Right arm ROS namespace.")
    parser.add_argument(
        "--remote-ip",
        default=None,
        help="Optional DDS peer IP. Sets CYCLONEDDS_URI for cross-machine discovery fallback.",
    )
    parser.add_argument("--ros-timeout-s", type=float, default=10.0, help="Seconds to wait for ROS state.")
    parser.add_argument(
        "--allow-missing-ros",
        action="store_true",
        help="Record even if left/right Franka state topics are not received.",
    )
    parser.add_argument(
        "--require-gello",
        action="store_true",
        help="Also require GELLO action topics before recording.",
    )
    parser.add_argument("--no-cameras", action="store_true", help="Skip camera connection and video capture.")
    parser.add_argument("--left-camera", default="0", help="OpenCV id/path for left camera.")
    parser.add_argument(
        "--right-camera",
        default="2",
        help="OpenCV id/path for right camera.",
    )
    parser.add_argument("--zed-serial", default=None, help="Optional ZED serial number for middle camera.")
    parser.add_argument(
        "--left-size",
        type=_parse_size,
        default=(640, 480),
        help="Left OpenCV camera WIDTHxHEIGHT.",
    )
    parser.add_argument(
        "--right-size",
        type=_parse_size,
        default=(640, 480),
        help="Right OpenCV camera WIDTHxHEIGHT.",
    )
    parser.add_argument("--zed-size", type=_parse_size, default=(672, 376), help="Middle ZED WIDTHxHEIGHT.")
    parser.add_argument("--camera-warmup-s", type=float, default=3.0, help="OpenCV camera warmup seconds.")
    parser.add_argument("--vcodec", default="auto", help="Video codec passed to LeRobotDataset.create().")
    parser.add_argument(
        "--streaming-encoding",
        action="store_true",
        help="Encode video while recording instead of after save_episode().",
    )
    parser.add_argument("--encoder-threads", type=int, default=2, help="Encoder threads per video.")
    parser.add_argument(
        "--serial-video-encoding",
        action="store_true",
        help="Encode camera videos serially during save_episode().",
    )
    return parser.parse_args()


def _dataset_root(repo_id: str, root: Path | None) -> Path:
    return root if root is not None else HF_LEROBOT_HOME / repo_id


def _configure_dds_peer(remote_ip: str | None) -> str | None:
    if not remote_ip:
        return None

    cyclonedds_uri = (
        "<CycloneDDS>"
        "<Domain><General><AllowMulticast>false</AllowMulticast></General>"
        f"<Discovery><Peers><Peer Address=\"{remote_ip}\"/></Peers></Discovery>"
        "</Domain></CycloneDDS>"
    )
    tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".xml", delete=False, prefix="cyclonedds_")
    tmp.write(cyclonedds_uri)
    tmp.close()
    os.environ["CYCLONEDDS_URI"] = f"file://{tmp.name}"
    return tmp.name


def _camera_config(index_or_path: str, size: tuple[int, int], fps: int, warmup_s: float) -> OpenCVCameraConfig:
    width, height = size
    return OpenCVCameraConfig(
        index_or_path=_parse_camera_id(index_or_path),
        width=width,
        height=height,
        fps=fps,
        warmup_s=warmup_s,
        fourcc="MJPG",
    )


def _zed_camera_config(
    serial_number: str | None,
    size: tuple[int, int],
    fps: int,
    warmup_s: float,
) -> Any:
    from lerobot.cameras.zed.configuration_zed import ZEDCameraConfig

    width, height = size
    return ZEDCameraConfig(
        serial_number=serial_number,
        width=width,
        height=height,
        fps=fps,
        warmup_s=int(warmup_s),
    )


def _build_camera_configs(args: argparse.Namespace) -> tuple[dict[str, Any], dict[str, Any]]:
    if args.no_cameras:
        return {}, {}

    left_cameras = {
        "left_camera": _camera_config(args.left_camera, args.left_size, args.fps, args.camera_warmup_s),
    }
    right_cameras = {
        "middle_zed": _zed_camera_config(
            args.zed_serial,
            args.zed_size,
            args.fps,
            args.camera_warmup_s,
        ),
        "right_camera": _camera_config(
            args.right_camera,
            args.right_size,
            args.fps,
            args.camera_warmup_s,
        ),
    }
    return left_cameras, right_cameras


def _arm_robot_config(
    args: argparse.Namespace,
    *,
    namespace: str,
    cameras: dict[str, Any],
) -> FrankaFr3RobotiqGripperConfig:
    return FrankaFr3RobotiqGripperConfig(
        ros_domain_id=args.ros_domain_id,
        topic_namespace=namespace,
        remote_ip=args.remote_ip,
        use_bridge=True,
        use_ft_sensor=False,
        cameras=cameras,
    )


def build_robot_config(args: argparse.Namespace) -> BiFrankaFr3RobotiqGripperConfig:
    left_cameras, right_cameras = _build_camera_configs(args)
    return BiFrankaFr3RobotiqGripperConfig(
        left_arm_config=_arm_robot_config(args, namespace=args.left_namespace, cameras=left_cameras),
        right_arm_config=_arm_robot_config(args, namespace=args.right_namespace, cameras=right_cameras),
    )


def _arm_teleop_config(args: argparse.Namespace, namespace: str) -> GelloRosLeaderConfig:
    return GelloRosLeaderConfig(
        ros_domain_id=args.ros_domain_id,
        topic_namespace=namespace,
        use_bridge=True,
    )


def build_teleop_config(args: argparse.Namespace) -> BiGelloRosLeaderConfig:
    return BiGelloRosLeaderConfig(
        left_arm_config=_arm_teleop_config(args, args.left_namespace),
        right_arm_config=_arm_teleop_config(args, args.right_namespace),
    )


def _robot_arms(robot: BiFrankaFr3RobotiqGripper) -> tuple[tuple[str, Any], tuple[str, Any]]:
    return (("left", robot.left_arm), ("right", robot.right_arm))


def _bridge_metadata(robot: BiFrankaFr3RobotiqGripper) -> dict[str, dict[str, Any]]:
    result = {}
    for side, arm in _robot_arms(robot):
        client = getattr(arm, "_bridge_client", None)
        result[side] = client.get_metadata() if client is not None else {}
    return result


def _received_all(received: dict[str, Any], keys: tuple[str, ...]) -> bool:
    return all(bool(received.get(key)) for key in keys)


def wait_for_ros_data(
    robot: BiFrankaFr3RobotiqGripper,
    *,
    timeout_s: float,
    require_gello: bool,
) -> dict[str, dict[str, Any]]:
    deadline = time.time() + timeout_s
    last_status: dict[str, dict[str, Any]] = {}

    while time.time() < deadline:
        last_status = _bridge_metadata(robot)
        left_received = last_status.get("left", {}).get("received", {})
        right_received = last_status.get("right", {}).get("received", {})

        has_left_arm = _received_all(left_received, ("arm",))
        has_right_arm = _received_all(right_received, ("arm",))
        has_gello = _received_all(left_received, GELLO_RECEIVED_KEYS) and _received_all(
            right_received,
            GELLO_RECEIVED_KEYS,
        )

        if has_left_arm and has_right_arm and (has_gello or not require_gello):
            return last_status

        time.sleep(0.2)

    raise RuntimeError(
        "Timed out waiting for ROS data. Required Franka state topics are "
        "/<left_namespace>/franka/joint_states and /<right_namespace>/franka/joint_states. "
        f"Last bridge status:\n{json.dumps(last_status, indent=2, ensure_ascii=False)}"
    )


def build_dataset(
    args: argparse.Namespace,
    robot: BiFrankaFr3RobotiqGripper,
    dataset_features: dict[str, dict],
    use_videos: bool,
) -> LeRobotDataset:
    resolved_root = _dataset_root(args.repo_id, args.root)
    if resolved_root.exists():
        if args.keep_existing:
            raise FileExistsError(f"Dataset root already exists: {resolved_root}")
        LOGGER.info("Removing existing test dataset root: %s", resolved_root)
        shutil.rmtree(resolved_root)

    dataset = LeRobotDataset.create(
        args.repo_id,
        args.fps,
        root=args.root,
        robot_type=robot.name,
        features=dataset_features,
        use_videos=use_videos,
        image_writer_processes=0,
        image_writer_threads=4 * len(robot.cameras),
        vcodec=args.vcodec,
        streaming_encoding=args.streaming_encoding,
        encoder_threads=args.encoder_threads,
    )
    LOGGER.info("Dataset root: %s", dataset.root)
    return dataset


def build_dataset_features(
    robot: BiFrankaFr3RobotiqGripper,
    teleop_action_processor: Any,
    robot_observation_processor: Any,
    use_videos: bool,
) -> dict[str, dict]:
    action_features = aggregate_pipeline_dataset_features(
        pipeline=teleop_action_processor,
        initial_features=create_initial_features(action=robot.action_features),
        use_videos=use_videos,
    )
    observation_features = aggregate_pipeline_dataset_features(
        pipeline=robot_observation_processor,
        initial_features=create_initial_features(observation=robot.observation_features),
        use_videos=use_videos,
    )
    return combine_feature_dicts(action_features, observation_features)


def verify_dataset(root: Path, repo_id: str, expected_frames: int, expected_videos: int) -> None:
    loaded = LeRobotDataset(repo_id, root=root, download_videos=False)
    if loaded.num_episodes < 1:
        raise RuntimeError("Dataset verification failed: no saved episodes.")
    if loaded.num_frames != expected_frames:
        raise RuntimeError(
            f"Dataset verification failed: frames={loaded.num_frames}, expected={expected_frames}."
        )

    if expected_videos:
        videos = sorted(root.glob("videos/**/*.mp4"))
        missing_or_empty = [path for path in videos if path.stat().st_size <= 0]
        if len(videos) != expected_videos or missing_or_empty:
            raise RuntimeError(
                "Dataset verification failed: expected "
                f"{expected_videos} non-empty mp4 files, got {len(videos)}. "
                f"Empty files: {missing_or_empty}"
            )


def _safe_disconnect(name: str, obj: Any) -> None:
    if obj is None:
        return
    try:
        obj.disconnect()
    except Exception as exc:
        LOGGER.warning("Failed to disconnect %s: %s", name, exc)


def _log_frame_progress(obs: dict[str, Any], frame_idx: int, num_frames: int) -> None:
    left = [round(float(obs[f"left_joint_positions_{i}"]), 4) for i in range(ROBOT_DOF)]
    right = [round(float(obs[f"right_joint_positions_{i}"]), 4) for i in range(ROBOT_DOF)]
    LOGGER.info("Frame %s/%s left=%s right=%s", frame_idx + 1, num_frames, left, right)


def main() -> int:
    logging.basicConfig(
        level=logging.INFO,
        format="%(levelname)s %(name)s: %(message)s",
        force=True,
    )
    args = parse_args()

    os.environ["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    dds_config_path = _configure_dds_peer(args.remote_ip)

    LOGGER.info("ROS_DOMAIN_ID=%s", os.environ["ROS_DOMAIN_ID"])
    if dds_config_path:
        LOGGER.info("CYCLONEDDS_URI=%s", os.environ["CYCLONEDDS_URI"])
    LOGGER.info("Default dataset root resolves to: %s", _dataset_root(args.repo_id, args.root))

    robot: BiFrankaFr3RobotiqGripper | None = None
    teleop: BiGelloRosLeader | None = None
    dataset: LeRobotDataset | None = None

    try:
        robot = BiFrankaFr3RobotiqGripper(build_robot_config(args))
        LOGGER.info("Connecting bimanual robot bridge and cameras...")
        robot.connect()
        LOGGER.info("Robot connected. Cameras: %s", list(robot.cameras))

        if args.allow_missing_ros:
            LOGGER.warning("Skipping ROS data wait because --allow-missing-ros was set.")
        else:
            status = wait_for_ros_data(
                robot,
                timeout_s=args.ros_timeout_s,
                require_gello=args.require_gello,
            )
            LOGGER.info("ROS data received: %s", json.dumps(status, ensure_ascii=False))

        teleop = BiGelloRosLeader(build_teleop_config(args))
        LOGGER.info("Connecting passive GELLO listeners...")
        teleop.connect()

        use_videos = not args.no_cameras and len(robot.cameras) > 0
        teleop_action_processor, _, robot_observation_processor = make_default_processors()
        dataset_features = build_dataset_features(
            robot,
            teleop_action_processor,
            robot_observation_processor,
            use_videos,
        )
        LOGGER.info("Dataset features: %s", list(dataset_features))

        dataset = build_dataset(args, robot, dataset_features, use_videos)

        num_frames = max(1, int(round(args.episode_s * args.fps)))
        LOGGER.info("Recording %s frames at %s fps...", num_frames, args.fps)

        start_t = time.perf_counter()
        for frame_idx in range(num_frames):
            loop_start = time.perf_counter()

            obs = robot.get_observation()
            obs_processed = robot_observation_processor(obs)
            observation_frame = build_dataset_frame(dataset.features, obs_processed, prefix=OBS_STR)

            action = teleop.get_action()
            action_processed = teleop_action_processor((action, obs))
            action_frame = build_dataset_frame(dataset.features, action_processed, prefix=ACTION)

            frame = {**observation_frame, **action_frame, "task": args.task}
            dataset.add_frame(frame)

            if frame_idx == 0 or (frame_idx + 1) % args.fps == 0 or frame_idx == num_frames - 1:
                _log_frame_progress(obs, frame_idx, num_frames)

            elapsed = time.perf_counter() - loop_start
            time.sleep(max((1.0 / args.fps) - elapsed, 0.0))

        LOGGER.info("Saving episode...")
        dataset.save_episode(parallel_encoding=not args.serial_video_encoding)
        dataset.finalize()

        elapsed_total = time.perf_counter() - start_t
        LOGGER.info(
            "Saved %s frames from %s cameras in %.1fs at %.1f fps.",
            num_frames,
            len(robot.cameras),
            elapsed_total,
            num_frames / elapsed_total,
        )

        verify_dataset(dataset.root, args.repo_id, num_frames, expected_videos=len(robot.cameras))
        LOGGER.info("Verified dataset: %s", dataset.root)
        return 0

    finally:
        if dataset is not None and not getattr(dataset, "_is_finalized", True):
            try:
                if dataset.has_pending_frames():
                    dataset.clear_episode_buffer()
                dataset.finalize()
            except Exception as exc:
                LOGGER.warning("Dataset cleanup failed: %s", exc)

        _safe_disconnect("teleop", teleop)
        _safe_disconnect("robot", robot)

        if dds_config_path:
            try:
                os.unlink(dds_config_path)
            except OSError:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
