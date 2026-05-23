"""
Bridge client for LeRobot: reads robot state and action from a shared-memory
JSON file written by the Python 3.10 `ros_bridge.py` process.

This allows Python 3.12 code to use ROS 2 data without importing rclpy (which
requires Python 3.10 ABI in ROS 2 Humble).

Uses a singleton per namespace so robot and teleop share one bridge process.
"""

import json
import logging
import os
import subprocess
import time
from pathlib import Path
from threading import Lock

logger = logging.getLogger(__name__)

_instances: dict[str, "RosBridgeClient"] = {}
_lock = Lock()
PYTHON310 = "/usr/bin/python3.10"


def _without_conda_paths(value: str) -> str:
    """Return a path-list value with conda entries removed."""
    if not value:
        return value

    parts = [
        part
        for part in value.split(os.pathsep)
        if part and "/conda" not in part and "/miniconda" not in part
    ]
    return os.pathsep.join(parts)


def _bridge_env() -> dict[str, str]:
    """Build an environment for the Python 3.10 ROS bridge subprocess."""
    env = os.environ.copy()
    for key in ("PYTHONHOME", "CONDA_PREFIX", "CONDA_DEFAULT_ENV"):
        env.pop(key, None)

    for key in ("LD_LIBRARY_PATH", "PATH", "PYTHONPATH"):
        if key in env:
            env[key] = _without_conda_paths(env[key])

    log_dir = Path(env.get("ROS_LOG_DIR", "/tmp/lerobot_ros_logs"))
    log_dir.mkdir(parents=True, exist_ok=True)
    env["ROS_LOG_DIR"] = str(log_dir)
    return env


def get_bridge_client(
    namespace: str = "left",
    fps: int = 30,
) -> "RosBridgeClient":
    """Return a shared RosBridgeClient for the given namespace."""
    with _lock:
        domain = os.getenv("ROS_DOMAIN_ID", "")
        key = f"{namespace}:{fps}:{domain}"
        if key not in _instances:
            _instances[key] = RosBridgeClient(namespace=namespace, fps=fps)
        return _instances[key]


class RosBridgeClient:
    """Reads robot observation and action from the ROS bridge output file.

    Use get_bridge_client() to get a shared singleton instance.
    """

    def __init__(self, namespace: str = "left", fps: int = 30):
        self._ns = namespace
        self._fps = fps
        self._state_path = f"/dev/shm/lerobot_state_{namespace}.json"
        self._bridge_proc: subprocess.Popen | None = None
        self._connected = False

    @property
    def state_path(self) -> str:
        return self._state_path

    def _find_bridge_script(self) -> Path:
        candidates = (
            Path(__file__).resolve().with_name("ros_bridge.py"),
            Path.cwd() / "src" / "ros_bridge.py",
            Path("/home/franka/franka_rdk/src/ros_bridge.py"),
        )
        for candidate in candidates:
            if candidate.exists():
                return candidate
        raise FileNotFoundError(f"ros_bridge.py not found in any of: {list(candidates)}")

    def connect(self) -> None:
        if self._connected:
            return

        bridge_script = self._find_bridge_script()

        try:
            os.unlink(self._state_path)
        except FileNotFoundError:
            pass

        cmd = [
            PYTHON310,
            str(bridge_script),
            "--namespace",
            self._ns,
            "--fps",
            str(self._fps),
            "--output",
            self._state_path,
        ]
        logger.info("Starting ROS bridge: %s", " ".join(cmd))

        self._bridge_proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            env=_bridge_env(),
        )

        # Wait for the state file to appear (bridge writes first frame)
        t0 = time.time()
        while not os.path.exists(self._state_path) and time.time() - t0 < 5.0:
            # Check if bridge died
            if self._bridge_proc.poll() is not None:
                out = self._bridge_proc.stdout.read() if self._bridge_proc.stdout else ""
                raise RuntimeError(f"ROS bridge exited early (code={self._bridge_proc.returncode}): {out}")
            time.sleep(0.1)

        if not os.path.exists(self._state_path):
            raise RuntimeError(f"ROS bridge did not create state file within 5s: {self._state_path}")

        logger.info("ROS bridge connected: ns=%s, state=%s", self._ns, self._state_path)
        self._connected = True

    def _read_state(self) -> dict:
        """Read the latest state dict from the shared file."""
        try:
            with open(self._state_path, "r") as f:
                return json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            return {}

    def get_observation(self) -> dict:
        state = self._read_state()
        return state.get("observation", {})

    def get_action(self) -> dict:
        state = self._read_state()
        return state.get("action", {})

    def get_metadata(self) -> dict:
        state = self._read_state()
        return state.get("metadata", {})

    def disconnect(self) -> None:
        if self._bridge_proc is not None:
            self._bridge_proc.terminate()
            try:
                self._bridge_proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self._bridge_proc.kill()
            self._bridge_proc = None
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    def __del__(self):
        self.disconnect()
