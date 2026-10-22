#!/bin/bash
# Nested (lidar+sensorfusion) Main build into a separate git-ignored prefix of the experiment worktree:
#   src/lidar+sensorfusion/build/exp_<VARIANT>/{build,devel}
# OPT variant: identical to build_nested.sh but CMAKE_BUILD_TYPE=Release with -O2 only (no NDEBUG) for the realtime comparison.
# --only-pkg-with-deps Main, outer worktree devel sourced. Optional SAN=1 adds ASan+UBSan (RelWithDebInfo -O1 -g,
# same recipe as the B0 audit s40_build_asan.sh).
# usage: build_nested.sh VARIANT [SAN]
set -u
VARIANT=$1; SAN=${2:-0}
RR=/home/autonav/2026AIM/Lidar_experimments/track_competition_stage1_20261022_134204
W=${SRC_ROOT:-$RR/repo}   # SRC_ROOT: e.g. a git-archive export of another commit
OUTER=${OUTER_DEVEL:-$RR/repo/devel}
NS=$W/src/lidar+sensorfusion
B=$NS/build/exp_$VARIANT/build; D=$NS/build/exp_$VARIANT/devel
L=$RR/logs/build; mkdir -p "$L/ros_home/log" "$B" "$D"
REG="-DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON -DCMAKE_FIND_PACKAGE_NO_PACKAGE_REGISTRY=ON"
CFG="-DCATKIN_ENABLE_TESTING=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE=-O2 -DCMAKE_C_FLAGS_RELEASE=-O2"
if [ "$SAN" = 1 ]; then
  S="-fsanitize=address,undefined -fno-omit-frame-pointer ${SAN_EXTRA_CXX:-}"
  CFG="$CFG -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_FLAGS_RELWITHDEBINFO='-O1 -g' -DCMAKE_C_FLAGS_RELWITHDEBINFO='-O1 -g' -DCMAKE_CXX_FLAGS='$S' -DCMAKE_C_FLAGS='$S' -DCMAKE_EXE_LINKER_FLAGS='-fsanitize=address,undefined' -DCMAKE_SHARED_LINKER_FLAGS='-fsanitize=address,undefined'"
fi
MAIN_CM=$NS/src/Main/CMakeLists.txt
TARGETS="main lidar_dataset_replay lidar_continuous_replay warmup_motion_evidence_test $(sed -n -E 's/^\s*catkin_add_gtest\(\s*([A-Za-z0-9_]+).*/\1/p; s/^\s*add_rostest_gtest\(\s*([A-Za-z0-9_]+).*/\1/p' "$MAIN_CM" | tr '\n' ' ')"
HEAD=$(git -C "$W" rev-parse HEAD)
git -C "$W" status --porcelain=v1 -uall > "$L/nested_${VARIANT}_git_status_before.txt"
LOG=$L/nested_${VARIANT}.log
echo "variant=$VARIANT san=$SAN head=$HEAD start=$(date -Is)" > "$LOG"
echo "targets=$TARGETS" >> "$LOG"
t0=$(date +%s)
env -i GIT_OPTIONAL_LOCKS=0 HOME=/home/autonav USER=autonav LOGNAME=autonav LANG=C.UTF-8 LC_ALL=C.UTF-8 TERM=dumb \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    ROS_HOME="$L/ros_home" ROS_LOG_DIR="$L/ros_home/log" ROS_MASTER_URI=http://127.0.0.1:65530 \
    MAKEFLAGS= CMAKE_BUILD_PARALLEL_LEVEL=4 \
    bash --noprofile --norc -c "set +u; source /opt/ros/noetic/setup.bash; source '$OUTER/setup.bash'; cd '$NS' && catkin_make --build '$B' -DCATKIN_DEVEL_PREFIX='$D' --only-pkg-with-deps Main --force-cmake -j${BUILD_JOBS:-4} -l${BUILD_LOAD:-5} $REG $CFG && source '$D/setup.bash' && cd '$B' && make -j${BUILD_JOBS:-4} -l${BUILD_LOAD:-5} $TARGETS" >> "$LOG" 2>&1
rc=$?
echo "nested_$VARIANT rc=$rc secs=$(( $(date +%s)-t0 )) head=$HEAD" | tee "$L/nested_${VARIANT}.rc"
git -C "$W" status --porcelain=v1 -uall > "$L/nested_${VARIANT}_git_status_after.txt"
exit $rc
