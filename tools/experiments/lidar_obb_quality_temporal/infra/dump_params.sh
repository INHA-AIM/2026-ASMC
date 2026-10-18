#!/bin/bash
# roslaunch --dump-params of the production V2 profile (Main/main_integration.launch, planner_udp_enabled:=false)
# for one build variant. Pure launch-file parsing: no master, no node is started.
# usage: dump_params.sh VARIANT OUT.yaml
set -u
VARIANT=$1; OUT=$2
RR=/home/autonav/2026AIM/Lidar_experimments/obb_quality_temporal_20261018_170239
D=${SRC_ROOT:-$RR/repo}/src/lidar+sensorfusion/build/exp_$VARIANT/devel
env -i HOME=/home/autonav USER=autonav LANG=C.UTF-8 LC_ALL=C.UTF-8 PATH=/usr/local/bin:/usr/bin:/bin \
    ROS_HOME=$RR/logs/build/ros_home ROS_MASTER_URI=http://127.0.0.1:65530 \
    bash --noprofile --norc -c "set +u; source /opt/ros/noetic/setup.bash; source '$D/setup.bash'; roslaunch --dump-params Main main_integration.launch planner_udp_enabled:=false" > "$OUT"
rc=$?
grep -c "^/Main/" "$OUT" | sed "s/^/main_params=/"
exit $rc
