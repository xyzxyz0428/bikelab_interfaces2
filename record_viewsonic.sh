#!/bin/bash
set -euo pipefail

DEV="/dev/video1"
FPS=30
W=1280
H=720

# 建议：设置一个默认录制时长，最稳（秒）。留空则录到 Ctrl+C
DURATION=""   # 例如 "300"=5分钟；""=手动停止

TS="$(date +'%Y-%m-%d_%H-%M-%S')"
DEFAULT_DIR="/media/bikelab2/TRANSCEND/bikelab_data/video"
DEFAULT_FILE="$DEFAULT_DIR/viewsonic_${W}x${H}_${FPS}fps_${TS}.avi"  # 用 avi 更抗中断

ARG="${1:-}"
if [[ -z "$ARG" ]]; then
  OUTFILE="$DEFAULT_FILE"
elif [[ "$ARG" == *.mkv || "$ARG" == *.mp4 || "$ARG" == *.avi ]]; then
  OUTFILE="$ARG"
else
  OUTFILE="$ARG/viewsonic_${W}x${H}_${FPS}fps_${TS}.avi"
fi

OUTDIR="$(dirname "$OUTFILE")"
mkdir -p "$OUTDIR"

echo "[1/2] Apply v4l2 controls on $DEV ..."

v4l2-ctl -d "$DEV" --set-ctrl=auto_exposure=0
v4l2-ctl -d "$DEV" --set-ctrl=gain=0
v4l2-ctl -d "$DEV" --set-ctrl=brightness=1
v4l2-ctl -d "$DEV" --set-ctrl=contrast=6
v4l2-ctl -d "$DEV" --set-ctrl=gamma=222
v4l2-ctl -d "$DEV" --set-ctrl=white_balance_automatic=0
v4l2-ctl -d "$DEV" --set-ctrl=white_balance_temperature=5200
v4l2-ctl -d "$DEV" --set-ctrl=saturation=80
v4l2-ctl -d "$DEV" --set-ctrl=sharpness=6
v4l2-ctl -d "$DEV" --set-ctrl=backlight_compensation=1

v4l2-ctl -d "$DEV" --set-fmt-video=width=$W,height=$H,pixelformat=MJPG || true
v4l2-ctl -d "$DEV" --set-parm=$FPS || true

echo "[2/2] Recording to: $OUTFILE"
if [[ -n "$DURATION" ]]; then
  echo "Will stop automatically after ${DURATION}s."
else
  echo "Press Ctrl+C to stop."
fi

FFPID=""

cleanup() {
  if [[ -n "${FFPID}" ]] && kill -0 "$FFPID" 2>/dev/null; then
    echo ""
    echo "[!] Ctrl+C received. Letting ffmpeg flush for 1s..."
    sleep 1
    kill -INT "$FFPID" 2>/dev/null || true
    wait "$FFPID" 2>/dev/null || true
  fi
}
trap cleanup INT TERM

# 录制：MJPG 直接 copy 到 AVI（最省 CPU、最抗中断）
# 如果 mjpeg 协商失败，去掉 -input_format mjpeg
if [[ -n "$DURATION" ]]; then
  ffmpeg -hide_banner -loglevel warning \
    -f v4l2 -framerate "$FPS" -video_size "${W}x${H}" -input_format mjpeg \
    -i "$DEV" -t "$DURATION" \
    -c copy "$OUTFILE" &
else
  ffmpeg -hide_banner -loglevel warning \
    -f v4l2 -framerate "$FPS" -video_size "${W}x${H}" -input_format mjpeg \
    -i "$DEV" \
    -c copy "$OUTFILE" &
fi

FFPID=$!
wait "$FFPID" 2>/dev/null || true

echo "Saved: $OUTFILE"
