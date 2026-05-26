#!/usr/bin/env bash
set -euo pipefail

REPO_DIR=/home/franka/franka_rdk
CONDA_BASE="${CONDA_BASE:-$HOME/miniconda3}"
CONDA_ENV_NAME="${CONDA_ENV_NAME:-lerobot}"
TASK_SLUG="${TASK_SLUG:-${TASK_NAME:-fold_the_box}}"
TASK_DESCRIPTION="${TASK_DESCRIPTION:-fold the box}"
RUN_TIMESTAMP="${RUN_TIMESTAMP:-$(date +%Y%m%d_%H%M%S)}"
DATASET_REPO_ID="${DATASET_REPO_ID:-local/${TASK_SLUG}_${RUN_TIMESTAMP}}"
HF_HOME_DIR="${HF_HOME:-$HOME/.cache/huggingface}"
HF_LEROBOT_HOME_DIR="${HF_LEROBOT_HOME:-$HF_HOME_DIR/lerobot}"
DATASET_ROOT="${DATASET_ROOT:-$HF_LEROBOT_HOME_DIR/$TASK_SLUG/$RUN_TIMESTAMP}"

cd "$REPO_DIR"

export LEROBOT_FRANKA_RDK_ROOT="${LEROBOT_FRANKA_RDK_ROOT:-$REPO_DIR}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

CONDA_SH="$CONDA_BASE/etc/profile.d/conda.sh"
if [[ ! -f "$CONDA_SH" ]]; then
  echo "Conda activation script not found: $CONDA_SH" >&2
  exit 1
fi

# shellcheck disable=SC1090
source "$CONDA_SH"
conda activate "$CONDA_ENV_NAME"

for arg in "$@"; do
  if [[ "$arg" == "--help" || "$arg" == "-h" ]]; then
    exec python -m lerobot.scripts.lerobot_record --help
  fi
done

echo "Task: $TASK_SLUG"
echo "Dataset: $DATASET_REPO_ID"
echo "Root: $DATASET_ROOT"
echo "Run timestamp: $RUN_TIMESTAMP"

exec python -m lerobot.scripts.lerobot_record \
  --robot.type=bi_franka_fr3_robotiq_gripper \
  --teleop.type=bi_gello_ros_leader \
  --dataset.repo_id="$DATASET_REPO_ID" \
  --dataset.root="$DATASET_ROOT" \
  --dataset.num_episodes=1 \
  --dataset.single_task="$TASK_DESCRIPTION" \
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
