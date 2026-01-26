#!/usr/bin/env bash
set -euo pipefail

SESSION="bikelab_interfaces2"
WIN_NAME="camera_glasses"

G3_DIR="/home/ubuntu/ros2_ws/src/glasses3-pylib"

G3_PY="$HOME/.pyenv/versions/g3env/bin/python"

command -v tmux >/dev/null 2>&1 || { echo "tmux not found. Install: sudo apt install -y tmux"; exit 2; }
command -v ros2 >/dev/null 2>&1 || { echo "ros2 not found in PATH"; exit 2; }

if ! tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "tmux session '$SESSION' not found."
  exit 2
fi

if [[ ! -x "$G3_PY" ]]; then
  echo "g3env python not found/executable at: $G3_PY"
  echo "Fix by setting G3_PY to the correct path (check: pyenv prefix g3env)."
  exit 2
fi

if [[ ! -d "$G3_DIR" ]]; then
  echo "Directory not found: $G3_DIR"
  exit 2
fi

# Avoid duplicate window
if tmux list-windows -t "$SESSION" -F '#W' | grep -qx "$WIN_NAME"; then
  echo "tmux window '$WIN_NAME' already exists in session '$SESSION'."
  exit 0
fi

tmux new-window -t "$SESSION" -n "$WIN_NAME"

# Pane 0: camera publisher (system python unaffected)
tmux send-keys -t "$SESSION:$WIN_NAME.0" \
  "echo '[CAM] ros2 run camera_streamer camera_publisher'; ros2 run camera_streamer camera_publisher" C-m

# Split -> Pane 1: glasses save_record with venv python only
tmux split-window -v -t "$SESSION:$WIN_NAME"

tmux send-keys -t "$SESSION:$WIN_NAME.1" \
  "echo '[G3] Using g3env python: $G3_PY'; cd '$G3_DIR' && '$G3_PY' ./tests/save_record.py" C-m

tmux select-layout -t "$SESSION:$WIN_NAME" even-vertical

echo "Started in tmux session '$SESSION' window '$WIN_NAME'."
echo "Attach: tmux attach -t $SESSION"
