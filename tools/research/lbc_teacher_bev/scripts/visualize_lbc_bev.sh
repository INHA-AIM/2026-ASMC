#!/usr/bin/env bash
# LBC BEV live viewer — MORAI Ioniq 5 (rear-axle /Ego_topic)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
# shellcheck source=lbc_env.sh
source "$SCRIPT_DIR/lbc_env.sh"
WAIT_SEC="${1:-120}"

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

if [ -z "${DISPLAY:-}" ]; then
  echo "[WARN] DISPLAY is not set. imshow needs X11 (WSL: export DISPLAY=:0)"
fi

echo "=============================================="
echo " LBC BEV imshow (Ioniq 5 rear-axle ego)"
echo "=============================================="

_ensure_scipy

"$WS_ROOT/scripts/wait_morai_topic.sh" /Ego_topic "$WAIT_SEC"
exec python3 "$LBC_SCRIPTS/lbc_bev_visualizer.py" \
  _use_imshow:=true \
  _pixels_ahead:=0 \
  _draw_ego_imshow:=true \
  _publish_images:=false \
  _skip_overrun_frames:=true \
  _filter_objects_radius_m:=45 \
  _object_bev_margin_px:=50 \
  _max_vehicles:=80 \
  _max_pedestrians:=40
