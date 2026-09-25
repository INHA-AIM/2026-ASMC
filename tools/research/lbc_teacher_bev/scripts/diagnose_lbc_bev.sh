#!/usr/bin/env bash
# Full LBC + MORAI diagnostic — save report for sharing
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
# shellcheck source=lbc_env.sh
source "$SCRIPT_DIR/lbc_env.sh"
OUT_DIR="$LBC_DATA_ROOT/diag"
TS=$(date +"%Y%m%d_%H%M%S")
REPORT="$OUT_DIR/lbc_report_${TS}.txt"

mkdir -p "$OUT_DIR"
cd "$WS_ROOT"
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"

echo "Writing report to: $REPORT"
echo ""

python3 "$LBC_SCRIPTS/lbc_bev_diagnose.py" \
  --aim-ws "$WS_ROOT" \
  --output "$REPORT" \
  --ego-wait 8 \
  --no-print 2>&1 | tee "$OUT_DIR/last_run_${TS}.log"

echo ""
echo "=============================================="
echo " Send this file:"
echo "   $REPORT"
echo "=============================================="
cat "$REPORT"
