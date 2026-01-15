#!/bin/bash
set -euo pipefail

# =========================
# User settings
# =========================
DEV="/dev/video1"

# 录制格式：识别优先建议 YUYV；CPU/IO优先可用 MJPG
# 可选：YUYV 或 MJPG
PIX="YUYV"

# 识别优先建议：640x480 或 800x600；过曝严重就先降分辨率
W=640
H=480
FPS=30

# 录制时长：留空=按 Ctrl+C 停；建议采集用固定时长更稳
DURATION=""   # e.g. "300" for 5 min

# 默认保存路径
DEFAULT_DIR="/media/bikelab2/TRANSCEND/bikelab_data/video"

# profile: sunny / normal
PROFILE="${1:-normal}"

# 输出路径参数（第2个参数）
ARG_OUT="${2:-}"

# =========================
# Output file handling
# =========================
TS="$(date +'%Y-%m-%d_%H-%M-%S')"
EXT="avi"   # AVI 最抗中断，采集更稳

DEFAULT_FILE="$DEFAULT_DIR/viewsonic_${PROFILE}_${W}x${H}_${FPS}fps_${TS}.${EXT}"

if [[ -z "$ARG_OUT" ]]; then
  OUTFILE="$DEFAULT_FILE"
elif [[ "$ARG_OUT" == *.mkv || "$ARG_OUT" == *.mp4 || "$ARG_OUT" == *.avi ]]; then
  OUTFILE="$ARG_OUT"
else
  OUTFILE="$ARG_OUT/viewsonic_${PROFILE}_${W}x${H}_${FPS}fps_${TS}.${EXT}"
fi

OUTDIR="$(dirname "$OUTFILE")"
mkdir -p "$OUTDIR"

# =========================
# Apply V4L2 controls
# =========================
apply_controls() {
  echo "[1/2] Apply v4l2 controls on $DEV (profile=$PROFILE, pix=$PIX, ${W}x${H}@${FPS}) ..."

  # 基础：尽量压住过曝、稳定颜色
  v4l2-ctl -d "$DEV" --set-ctrl=auto_exposure=0
  v4l2-ctl -d "$DEV" --set-ctrl=gain=0
  v4l2-ctl -d "$DEV" --set-ctrl=brightness=0
  v4l2-ctl -d "$DEV" --set-ctrl=backlight_compensation=0

  # 固定白平衡（室外）
  v4l2-ctl -d "$DEV" --set-ctrl=white_balance_automatic=0
  v4l2-ctl -d "$DEV" --set-ctrl=white_balance_temperature=5200

  # 分 profile 的“识别向”参数
  case "$PROFILE" in
    sunny)
      v4l2-ctl -d "$DEV" --set-ctrl=contrast=14
      v4l2-ctl -d "$DEV" --set-ctrl=gamma=150
      v4l2-ctl -d "$DEV" --set-ctrl=saturation=50
      v4l2-ctl -d "$DEV" --set-ctrl=sharpness=9
      ;;
    normal)
      v4l2-ctl -d "$DEV" --set-ctrl=contrast=11
      v4l2-ctl -d "$DEV" --set-ctrl=gamma=180
      v4l2-ctl -d "$DEV" --set-ctrl=saturation=70
      v4l2-ctl -d "$DEV" --set-ctrl=sharpness=10
      ;;
    *)
      echo "Unknown PROFILE: $PROFILE (use sunny|normal)"
      exit 1
      ;;
  esac

  # 设置格式/帧率（不支持就忽略）
  v4l2-ctl -d "$DEV" --set-fmt-video=width=$W,height=$H,pixelformat=$PIX || true
  v4l2-ctl -d "$DEV" --set-parm=$FPS || true
}

# =========================
# Record with ffmpeg
# =========================
record_video() {
  echo "[2/2] Recording to: $OUTFILE"
  if [[ -n "$DURATION" ]]; then
    echo "Will stop automatically after ${DURATION}s."
  else
    echo "Press Ctrl+C to stop."
  fi

  local FFPID=""
  cleanup() {
    if [[ -n "$FFPID" ]] && kill -0 "$FFPID" 2>/dev/null; then
      echo ""
      echo "[!] Ctrl+C received. Letting ffmpeg flush for 1s..."
      sleep 1
      kill -INT "$FFPID" 2>/dev/null || true
      wait "$FFPID" 2>/dev/null || true
    fi
  }
  trap cleanup INT TERM

  if [[ "$PIX" == "MJPG" ]]; then
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
  else
    if [[ -n "$DURATION" ]]; then
      ffmpeg -hide_banner -loglevel warning \
        -f v4l2 -framerate "$FPS" -video_size "${W}x${H}" \
        -i "$DEV" -t "$DURATION" \
        -c copy "$OUTFILE" &
    else
      ffmpeg -hide_banner -loglevel warning \
        -f v4l2 -framerate "$FPS" -video_size "${W}x${H}" \
        -i "$DEV" \
        -c copy "$OUTFILE" &
    fi
  fi

  FFPID=$!
  wait "$FFPID" 2>/dev/null || true
  echo "Saved: $OUTFILE"
}

apply_controls
record_video
