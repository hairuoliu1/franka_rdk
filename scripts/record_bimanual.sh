#!/usr/bin/env bash
set -euo pipefail

REPO_DIR=/home/franka/franka_rdk
PYTHON_BIN=/home/franka/miniconda3/envs/lerobot/bin/python

cd "$REPO_DIR"

export LEROBOT_FRANKA_RDK_ROOT="${LEROBOT_FRANKA_RDK_ROOT:-$REPO_DIR}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  exec "$PYTHON_BIN" -m lerobot.scripts.lerobot_record --help
fi

exec "$PYTHON_BIN" - "$@" <<'PY'
import sys

from lerobot.scripts.lerobot_record import DatasetRecordConfig, RecordConfig, record
from lerobot.robots.bi_franka_fr3_robotiq_gripper import BiFrankaFr3RobotiqGripperConfig
from lerobot.teleoperators.bi_gello_leader import BiGelloRosLeaderConfig
from lerobot.utils.import_utils import register_third_party_plugins

register_third_party_plugins()

cfg = RecordConfig(
    robot=BiFrankaFr3RobotiqGripperConfig(),
    teleop=BiGelloRosLeaderConfig(),
    dataset=DatasetRecordConfig(
        repo_id="local/bimanual_franka_recording",
        num_episodes=1,
        single_task="bimanual franka recording",
        fps=30,
        episode_time_s=30,
        reset_time_s=0,
        video=True,
        streaming_encoding=True,
        encoder_threads=2,
        vcodec="auto",
        push_to_hub=False,
    ),
    display_data=False,
    play_sounds=False,
)

record(cfg)
PY
