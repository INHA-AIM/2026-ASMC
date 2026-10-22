#!/bin/bash
# Outer catkin workspace build inside the experiment worktree (git-ignored build/ devel/ only).
# Same command semantics as scripts/build_ws.sh (production CI), CMAKE_BUILD_TYPE unset (as production build).
set -u
RR=/home/autonav/2026AIM/Lidar_experimments/track_competition_stage1_20261022_134204
W=$RR/repo
L=$RR/logs/build
mkdir -p "$L/ros_home/log"
REG="-DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON -DCMAKE_FIND_PACKAGE_NO_PACKAGE_REGISTRY=ON"
git -C "$W" status --porcelain=v1 -uall > "$L/outer_git_status_before.txt"
t0=$(date +%s)
env -i GIT_OPTIONAL_LOCKS=0 HOME=/home/autonav USER=autonav LOGNAME=autonav LANG=C.UTF-8 LC_ALL=C.UTF-8 TERM=dumb \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    ROS_HOME="$L/ros_home" ROS_LOG_DIR="$L/ros_home/log" ROS_MASTER_URI=http://127.0.0.1:65530 \
    MAKEFLAGS= CMAKE_BUILD_PARALLEL_LEVEL=4 \
    bash --noprofile --norc -c "source /opt/ros/noetic/setup.bash; cd '$W' && ./scripts/build_ws.sh -j4 -l5 $REG" > "$L/outer_build.log" 2>&1
rc=$?
echo "outer_build rc=$rc secs=$(( $(date +%s)-t0 ))" | tee "$L/outer_build.rc"
git -C "$W" status --porcelain=v1 -uall > "$L/outer_git_status_after.txt"
