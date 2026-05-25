#!/usr/bin/env bash
set -euo pipefail

REPO_DIR=/home/franka/franka_rdk
PYTHON_BIN=/home/franka/miniconda3/envs/lerobot/bin/python

cd "$REPO_DIR"

export LEROBOT_FRANKA_RDK_ROOT="${LEROBOT_FRANKA_RDK_ROOT:-$REPO_DIR}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

for arg in "$@"; do
  if [[ "$arg" == "--help" || "$arg" == "-h" ]]; then
    exec "$PYTHON_BIN" -m lerobot.scripts.lerobot_record --help
  fi
done

exec "$PYTHON_BIN" -m lerobot.scripts.lerobot_record \
  --robot.type=bi_franka_fr3_robotiq_gripper \
  --teleop.type=bi_gello_ros_leader \
  --dataset.repo_id=local/bimanual_franka_recording \
  --dataset.num_episodes=10 \
  --dataset.single_task="bimanual franka recording" \
  --dataset.fps=30 \
  --dataset.episode_time_s=300 \
  --dataset.reset_time_s=0 \
  --dataset.video=true \
  --dataset.streaming_encoding=true \
  --dataset.encoder_threads=2 \
  --dataset.vcodec=auto \
  --dataset.push_to_hub=false \
  --display_data=true \
  --play_sounds=true \
  "$@"
