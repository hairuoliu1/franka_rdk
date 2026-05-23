#!/usr/bin/env bash
set -euo pipefail

REPO_DIR=/home/franka/franka_rdk
PYTHON_BIN=/home/franka/miniconda3/envs/lerobot/bin/python
CONFIG_FILE="$REPO_DIR/script/record_bimanual.yaml"

cd "$REPO_DIR"

exec "$PYTHON_BIN" -m src.lerobot_record --config_path="$CONFIG_FILE" "$@"
