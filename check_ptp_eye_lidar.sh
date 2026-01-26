#!/usr/bin/env bash
set -euo pipefail

# ---------------------------
# 1) PTP slave check
# ---------------------------
echo "# 1) PTP slave check (TIME_STATUS_NP):"
echo "#    true  = gmPresent is true AND |offsetFromMaster| < 1,000,000 ns (1 ms)"
echo "#    false = otherwise"

ptp_slave="$(
  sudo pmc -u -b 0 "GET TIME_STATUS_NP" 2>/dev/null \
  | awk '
      /gmPresent/ {gm=$2}
      /offsetFromMaster/ {off=$2}
      END {
        # off is ns; accept gmPresent as 1/true; offset within +/-1e6 ns
        if ((gm=="1" || gm=="true") && off<1000000 && off>-1000000) print "true";
        else print "false";
      }'
)"
echo "$ptp_slave"

echo

# ---------------------------
# 2) Eyetracker NTP sync check
# ---------------------------
echo "# 2) Eyetracker NTP sync check (HTTP endpoint):"
echo "#    true  = endpoint indicates synchronized"
echo "#    false = endpoint indicates NOT synchronized, or request failed"

eye_resp="$(curl -fsS --max-time 2 "http://192.168.1.166/rest/system.ntp-is-synchronized" 2>/dev/null || true)"

# Try to interpret common responses: true/false, 1/0, JSON containing "true"/"false"
eye_ok="false"
if echo "$eye_resp" | grep -qiE 'true|1'; then
  eye_ok="true"
elif echo "$eye_resp" | grep -qiE 'false|0'; then
  eye_ok="false"
else
  # unknown format or empty -> false
  eye_ok="false"
fi

echo "$eye_ok"
echo "#    raw response: ${eye_resp:-<empty/no response>}"

echo

# ---------------------------
# 3) Start LiDARs (keep running) + Time sync check
# ---------------------------
SESSION="bikelab_interfaces2"

# thresholds for time check
THRESH="0.1"        # seconds; pass if |now - stamp| < THRESH
TIMEOUT_S="3"       # seconds; max wait for one message per topic
WAIT_AFTER_START="2" # seconds; wait after launching before checking topics

topics=(/rslidar_points_200 /rslidar_points_201 /rslidar_points_202)

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux not found. Install it with: sudo apt update && sudo apt install -y tmux"
  exit 2
fi

# 3A) Start LiDAR launch in detached tmux session
if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "# 3A) tmux session '$SESSION' already exists (not starting another)."
else
  tmux new-session -d -s "$SESSION" -n lidar \
    "echo '[LiDAR] ros2 launch rslidar_sdk start.py'; ros2 launch rslidar_sdk start.py"
  echo "# 3A) started tmux session '$SESSION' (detached). Attach: tmux attach -t $SESSION"
fi

# 3B) Time sync check via ROS topic timestamps
echo
echo "# 3B) LiDAR time alignment check vs local clock"
echo "#     true  = |now - header.stamp| < ${THRESH}s  (LiDAR timestamps aligned to this PC)"
echo "#     false = no message within ${TIMEOUT_S}s OR timestamp far from local time"

sleep "$WAIT_AFTER_START"

for topic in "${topics[@]}"; do
  # Get one message (up to TIMEOUT_S seconds). Parse header stamp and compare to local time.
  out="$(
    timeout "${TIMEOUT_S}" ros2 topic echo --once "${topic}" 2>/dev/null \
    | awk -v now="$(date +%s.%N)" -v th="${THRESH}" '
        { sub(/\r$/, "", $0) }

        # Grab first sec/nanosec encountered (PointCloud2 header.stamp)
        /[[:space:]]sec:/     && sec==""  { sec=$2; next }
        /[[:space:]]nanosec:/ && nsec=="" {
          nsec=$2
          stamp = sec + nsec/1e9
          dt = now - stamp; if (dt<0) dt=-dt
          ok = (dt < th) ? "true" : "false"
          printf("dt=%.6f s %s (now=%.9f stamp=%.9f)\n", dt, ok, now, stamp)
          exit
        }

        END {
          if (sec=="" || nsec=="") {
            printf("dt=NA false (no message / parse failed)\n")
          }
        }
      '
  )" || true

  ok="$(echo "$out" | awk '{print $2}')"
  dt="$(echo "$out" | awk '{sub("dt=","",$1); print $1}')"

  if [[ "$ok" == "true" ]]; then
    echo "  - ${topic}: dt=${dt} -> true  (aligned)"
  else
    echo "  - ${topic}: ${out}  (NOT aligned / no data)"
  fi
done
