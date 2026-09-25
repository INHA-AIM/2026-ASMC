#!/usr/bin/env bash
# LBC BEV live preview without RViz/rqt (saves PNG)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
# shellcheck source=lbc_env.sh
source "$SCRIPT_DIR/lbc_env.sh"
OUT_DIR="$LBC_DATA_ROOT/diag/live"
WAIT_SEC="${1:-30}"

cd "$WS_ROOT"
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"

python3 -c "from scipy.spatial import cKDTree" 2>/dev/null || pip3 install -q scipy

mkdir -p "$OUT_DIR"
echo "Saving BEV PNGs to: $OUT_DIR"
echo ""

"$WS_ROOT/scripts/wait_morai_topic.sh" /Ego_topic "$WAIT_SEC"

rosparam set /lbc_bev_visualizer/save_snapshots true
rosparam set /lbc_bev_visualizer/snapshot_dir "$OUT_DIR"
rosparam set /lbc_bev_visualizer/draw_ego_marker false
rosparam set /lbc_bev_visualizer/debug true

python3 "$LBC_SCRIPTS/lbc_bev_visualizer.py" _draw_ego_marker:=false _save_snapshots:=true
