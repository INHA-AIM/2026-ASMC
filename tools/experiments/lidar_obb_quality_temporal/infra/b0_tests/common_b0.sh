#!/bin/bash
# B0 integration harness (G4) - shared settings. Adapted from analysis/tests/skeletons/common_b0.sh.
# Source this file; never run ROS outside ns_exec_b0.sh.
#
# Isolation contract (analysis/tests/evidence/e03_ns_probe.txt, logs/g3 rostest/runtime runs):
#   unshare --user --map-root-user --net --pid --fork --mount-proc
#   + ip link set lo up                       (only lo, 0 IPv4/IPv6 routes)
#   + private binfmt_misc instance            (WSLInterop handler absent -> Windows exe cannot run)
#   + tmpfs /run/WSL, tmpfs /tmp, WSL/ROS env unset
#   ports: private master port in [20000,29999], never 11311/11312/11400
set -u
R=/home/autonav/2026AIM/Lidar_experimments/obb_quality_temporal_20261018_170239/logs/b0
WT=/home/autonav/2026AIM/Lidar_experimments/obb_quality_temporal_20261018_170239/repo
NS=$WT/src/lidar+sensorfusion
MAIN=$NS/src/Main
BAG=/home/autonav/2026AIM/data/tunnel_yolo_fusion.bag
BAG_SHA_EXPECTED=a8225512015ee19c834dcb5fc67cec4b4905048f5b194b7b5de79bdadaf00444   # precheck/bag_sha256_start.txt
SK=/home/autonav/2026AIM/Lidar_experimments/obb_quality_temporal_20261018_170239/tools/b0_tests
CAT=$SK/fault_catalogue_g4.json
RT=${B0_RT:-$R/logs/g4/runtime}         # per-run directories
LB=${B0_LB:-$R/logs/g4/matrix}          # driver logs + summary
mkdir -p "$RT/gen" "$LB" "$RT/ros_home"
NESTED_DEVEL=${B0_NESTED_DEVEL:?set B0_NESTED_DEVEL to the variant devel}

PORT_FILE=/home/autonav/2026AIM/Lidar_experimments/obb_quality_temporal_20261018_170239/logs/b0/master_port.txt
if [ ! -s "$PORT_FILE" ]; then
  while :; do p=$(( 20000 + RANDOM % 10000 )); case $p in 11311|11312|11400) continue;; esac
    ss -Htuln 2>/dev/null | awk '{print $5}' | grep -q ":$p\$" && continue; echo "$p" > "$PORT_FILE"; break; done
fi
PORT=$(cat "$PORT_FILE")
case $PORT in 11311|11312|11400) echo "FORBIDDEN PORT $PORT"; exit 99;; esac
[ "$PORT" -ge 20000 ] && [ "$PORT" -le 29999 ] || { echo "PORT OUT OF RANGE $PORT"; exit 99; }

ros_ps() {
  ps -u autonav -o pid=,ppid=,etime=,comm=,args= 2>/dev/null | \
    awk '$4 ~ /^(rosmaster|roscore|roslaunch|rosout|rostest|rosbag|record|play|main|strace|rostopic|rosparam|rosnode|camera_objects_|planning_.*|tracked_wire_br)$/ || ($4 ~ /^python3$/ && $0 ~ /b0_(feeder|recorder|udp_sink)|ros/)' | cut -c1-220 || true
}
host_listeners() { ss -Htuln 2>/dev/null | awk -v p=":$PORT$" '$5 ~ /:(11311|11312|11400|7510)$/ || $5 ~ p'; }
