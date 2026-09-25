#!/usr/bin/env bash
# LBC BEV live view — optional RViz/rqt
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
# shellcheck source=lbc_env.sh
source "$SCRIPT_DIR/lbc_env.sh"
RVIZ_CFG="$LBC_TOOL_ROOT/config/lbc_bev.rviz"
WAIT_SEC="${1:-120}"
VIEWER="${VIEWER:-none}"
LAUNCH_GUI="${LAUNCH_GUI:-1}"

_ensure_scipy() {
  python3 -c "from scipy.spatial import cKDTree" 2>/dev/null && return 0
  apt-get update -qq
  DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends python3-scipy \
    >/dev/null 2>&1 || pip3 install -q scipy
}

cd "$WS_ROOT"
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"

export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-root}"
mkdir -p "$XDG_RUNTIME_DIR" && chmod 700 "$XDG_RUNTIME_DIR"
export QT_X11_NO_MITSHM=1
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"

_ensure_scipy

echo "=============================================="
echo " LBC BEV (official colors, no ego marker)"
echo "=============================================="
echo "  No GUI: ./scripts/snapshot_lbc_bev.sh"
echo "  VIEWER=rviz|rqt|none"
echo "=============================================="

"$WS_ROOT/scripts/wait_morai_topic.sh" /Ego_topic "$WAIT_SEC"

VIEWER_PID=
if [ "$LAUNCH_GUI" = "1" ] && [ -n "${DISPLAY:-}" ] && [ "$VIEWER" != "none" ]; then
  if [ "$VIEWER" = "rqt" ]; then
    rosrun rqt_image_view rqt_image_view /lbc_bev/image_full &
    VIEWER_PID=$!
  elif [ "$VIEWER" = "rviz" ]; then
    rosrun rviz rviz -d "$RVIZ_CFG" &
    VIEWER_PID=$!
  fi
  sleep 2
fi

cleanup() {
  kill "$VIEWER_PID" 2>/dev/null || true
  kill "$VIZ_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

rosparam set /lbc_bev_visualizer/draw_ego_marker false 2>/dev/null || true
python3 "$LBC_SCRIPTS/lbc_bev_visualizer.py" _debug:=true _draw_ego_marker:=false &
VIZ_PID=$!
wait "$VIZ_PID"
