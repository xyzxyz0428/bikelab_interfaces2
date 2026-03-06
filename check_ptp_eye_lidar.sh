#!/usr/bin/env bash
set -euo pipefail

# ---------------------------
# 1) PTP slave check
# ---------------------------
echo "# 1) PTP slave check (TIME_STATUS_NP):"
echo "#    true  = gmPresent is true AND |master_offset| < 1,000,0000 ns (10 ms)"
echo "#    false = otherwise"

ptp_slave="$(
  sudo pmc -u -b 0 "GET PORT_DATA_SET" "GET TIME_STATUS_NP" 2>/dev/null | awk '
/portState/     {ps=$2}
/gmPresent/     {gm=$2}
/master_offset/ {mo=$2}
END {
  ok=(ps=="SLAVE" && (gm=="1"||gm=="true") && mo!="" && mo<10000000 && mo>-10000000)
  printf("PTP_lock=%s portState=%s gmPresent=%s master_offset_ns=%s\n",
         ok?"true":"false", (ps?ps:"NA"), (gm?gm:"NA"), (mo?mo:"NA"))
}'
)"
echo "$ptp_slave"

# Extract PTP boolean (true/false) from the printed line
ptp_ok="$(echo "$ptp_slave" | awk '
{
  for (i=1; i<=NF; i++) {
    if ($i ~ /^PTP_lock=/) {
      split($i, a, "=")
      print a[2]
      exit
    }
  }
}')"

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
  eye_ok="false"
fi

echo "$eye_ok"
echo "#    raw response: ${eye_resp:-<empty/no response>}"

echo


