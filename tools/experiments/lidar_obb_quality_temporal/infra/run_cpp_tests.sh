#!/bin/bash
# Every Main gtest of the variant's build (catkin_add_gtest list parsed from the given CMakeLists) + warmup plain
# test + outer planning_inputs gtests/planning_motion_test + asmc_lidar ctest and plain tests, each binary inside
# 'unshare --user --map-root-user --net' with ROS_MASTER_URI pointing at a dead port (same as the B0 audit s20).
# usage: run_cpp_tests.sh VARIANT OUTDIR CMAKELISTS [SAN(0|1)]
set -u
VARIANT=$1; LB=$2; CM=$3; SAN=${4:-0}
RR=/home/autonav/2026AIM/Lidar_experimments/obb_quality_temporal_20261018_170239
W=${SRC_ROOT:-$RR/repo}; NS=$W/src/lidar+sensorfusion   # SRC_ROOT: exported tree of another commit
NSD=$NS/build/exp_$VARIANT/devel; NSB=$NS/build/exp_$VARIANT/build; OD=$W/devel; OB=$W/build
mkdir -p "$LB/ros_home/log"
SUM=$LB/cpp_tests_summary.tsv
printf "kind\tname\tbinary\trc\tms\ttests\tfailures\terrors\tdisabled\tskipped\tasan_files\tubsan_files\n" > "$SUM"
san_env() {
  [ "$SAN" = 1 ] || return 0
  echo "export ASAN_OPTIONS='log_path=$1/asan:detect_leaks=1:handle_abort=1:abort_on_error=0:fast_unwind_on_malloc=0:malloc_context_size=40' UBSAN_OPTIONS='log_path=$1/ubsan:print_stacktrace=1:halt_on_error=0';"
}
run_env() {
  local setup=$1 sandir=$2; shift 2
  unshare --user --map-root-user --net env -i GIT_OPTIONAL_LOCKS=0 HOME=/home/autonav USER=autonav LOGNAME=autonav LANG=C.UTF-8 LC_ALL=C.UTF-8 TERM=dumb \
      PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
      ROS_HOME="$LB/ros_home" ROS_LOG_DIR="$LB/ros_home/log" \
      bash --noprofile --norc -c "set +u; source /opt/ros/noetic/setup.bash; source '$setup'; export ROS_MASTER_URI=http://127.0.0.1:65530; $(san_env "$sandir") $*"
}
xml_attr() { grep -m1 -o "<testsuites[^>]*" "$1" | grep -o " $2=\"[0-9]*\"" | grep -o '[0-9]*'; }
run_one() {
  local kind=$1 name=$2 bin=$3 cwd=$4 setup=$5; shift 5
  local log=$LB/${kind}_$name.log sd=$LB/san/$name t0 t1 rc tests=- fails=- errs=- dis=- skp=-
  mkdir -p "$sd"
  if [ ! -x "$bin" ]; then echo "MISSING $bin" > "$log"; printf "%s\t%s\t%s\tMISSING\t0\t-\t-\t-\t-\t-\t0\t0\n" "$kind" "$name" "$bin" >> "$SUM"; echo "$kind $name MISSING"; return; fi
  local pre=""; [ "$SAN" = 1 ] && pre="setarch $(uname -m) -R"
  t0=$(date +%s%3N)
  run_env "$setup" "$sd" "cd '$cwd' && timeout 1200 $pre '$bin' $*" > "$log" 2>&1
  rc=$?
  t1=$(date +%s%3N)
  if [ -f "$LB/gtest_$name.xml" ] && [ "$kind" = gtest ]; then
    tests=$(xml_attr "$LB/gtest_$name.xml" tests); fails=$(xml_attr "$LB/gtest_$name.xml" failures)
    errs=$(xml_attr "$LB/gtest_$name.xml" errors); dis=$(xml_attr "$LB/gtest_$name.xml" disabled)
    skp=$(grep -c 'result="skipped"\|<skipped' "$LB/gtest_$name.xml")
  fi
  printf "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" "$kind" "$name" "$bin" "$rc" "$((t1-t0))" "${tests:--}" "${fails:--}" "${errs:--}" "${dis:--}" "${skp:--}" \
    "$(ls "$sd" | grep -c '^asan')" "$(ls "$sd" | grep -c '^ubsan')" >> "$SUM"
  echo "$kind $name rc=$rc tests=$tests failures=$fails"
}
echo "start $(date -Is) variant=$VARIANT san=$SAN NSD=$NSD"
for t in $(sed -n -E 's/^\s*catkin_add_gtest\(\s*([A-Za-z0-9_]+).*/\1/p' "$CM"); do
  rm -f "$LB/gtest_$t.xml"
  run_one gtest "$t" "$NSD/lib/Main/$t" "$NSB/Main" "$NSD/setup.bash" "--gtest_output=xml:$LB/gtest_$t.xml"
done
run_one plain warmup_motion_evidence_test "$NSD/lib/Main/warmup_motion_evidence_test" "$NSB/Main" "$NSD/setup.bash"
if [ "$SAN" != 1 ]; then
  for t in $(sed -n -E 's/^\s*catkin_add_gtest\(\s*([A-Za-z0-9_]+).*/\1/p' "$W/src/planning_inputs/CMakeLists.txt"); do
    rm -f "$LB/gtest_$t.xml"
    run_one gtest "$t" "$OD/lib/planning_inputs/$t" "$OB/planning_inputs" "$OD/setup.bash" "--gtest_output=xml:$LB/gtest_$t.xml"
  done
  run_one plain planning_motion_test "$OD/lib/planning_inputs/planning_motion_test" "$OB/planning_inputs" "$OD/setup.bash"
  for t in asmc_lidar_geometry_boundary_test asmc_lidar_frame_time_core_test asmc_lidar_imu_selection_test; do
    run_one plain "$t" "$OD/lib/asmc_lidar/$t" "$OB/asmc_lidar" "$OD/setup.bash"
  done
fi
echo "== totals"
awk -F'\t' 'NR>1 && $1=="gtest" {n++; if ($6 ~ /^[0-9]+$/) t+=$6; if ($7 ~ /^[0-9]+$/) f+=$7; if($4!=0) bad++} END {printf "gtest binaries=%d tests=%d failures=%d nonzero_rc=%d\n", n, t, f, bad+0}' "$SUM"
echo "done $(date -Is)"
