#!/usr/bin/env bash
set -euo pipefail

SESSION="bikelab_interfaces2"
WIN_NAME="camera_glasses"

G3_DIR="/home/ubuntu/ros2_ws/src/bikelab_interfaces2/glasses3-pylib"
G3_PY="$HOME/.pyenv/versions/g3env/bin/python"

CAM_DIR="$HOME/ros2_ws/src/bikelab_interfaces2"
CAM_CMD="cd '$CAM_DIR' && python3 log_camera_to_frame.py"

PCAP_ONELINER="sudo tcpdump -i eth0 -n -B 4096 'udp and ((host 192.168.1.200 and (port 2000 or port 2001)) or (host 192.168.1.201 and (port 2010 or port 2011)) or (host 192.168.1.202 and (port 2020 or port 2021)))' -w /mnt/bikelab_data/rs_3lidars_\$(date +%Y%m%d_%H%M%S).pcap"

command -v tmux >/dev/null 2>&1 || { echo "tmux not found. Install: sudo apt install -y tmux"; exit 2; }

[[ -x "$G3_PY" ]] || { echo "g3env python not found/executable at: $G3_PY"; exit 2; }
[[ -d "$G3_DIR" ]] || { echo "Directory not found: $G3_DIR"; exit 2; }
[[ -d "$CAM_DIR" ]] || { echo "Camera script directory not found: $CAM_DIR"; exit 2; }

echo "[PRE] Killing VSCode ripgrep (if running)..."
pkill -f ".vscode-server/.*/ripgrep/bin/rg" >/dev/null 2>&1 || true
pkill -f "@vscode/ripgrep/bin/rg" >/dev/null 2>&1 || true

echo "Tip: run 'sudo -v' once before starting to avoid sudo password prompts inside tmux."

# Create session if needed
if ! tmux has-session -t "$SESSION" 2>/dev/null; then
  tmux new-session -d -s "$SESSION" -n "$WIN_NAME"
else
  # If window already exists, don't duplicate
  if tmux list-windows -t "$SESSION" -F '#W' | grep -qx "$WIN_NAME"; then
    echo "tmux window '$WIN_NAME' already exists in session '$SESSION'."
    echo "Attach: tmux attach -t $SESSION"
    exit 0
  fi
  tmux new-window -t "$SESSION" -n "$WIN_NAME"
fi

# --- Get pane IDs robustly ---
# Pane A = initial pane in the new window
PANE_CAM="$(tmux display-message -p -t "$SESSION:$WIN_NAME" '#{pane_id}')"

# Split vertically -> Pane B (glasses)
PANE_G3="$(tmux split-window -v -t "$PANE_CAM" -P -F '#{pane_id}')"

# Split horizontally from camera pane -> Pane C (pcap)
PANE_PCAP="$(tmux split-window -h -t "$PANE_CAM" -P -F '#{pane_id}')"

# Layout
tmux select-layout -t "$SESSION:$WIN_NAME" tiled

# --- Send commands (use bash -lc to ensure normal shell parsing) ---
tmux send-keys -t "$PANE_CAM" "echo '[CAM] $CAM_CMD'; bash -lc \"$CAM_CMD\"" C-m

tmux send-keys -t "$PANE_G3" \
  "echo '[G3] Using g3env python: $G3_PY'; cd '$G3_DIR' && '$G3_PY' ./tests/save_record.py" C-m

tmux send-keys -t "$PANE_PCAP" \
  "echo '[PCAP] tcpdump start'; bash -lc \"$PCAP_ONELINER\"" C-m

echo "Started tmux session '$SESSION' window '$WIN_NAME'."
echo "Attach: tmux attach -t $SESSION"
echo "Debug: panes are CAM=$PANE_CAM G3=$PANE_G3 PCAP=$PANE_PCAP"