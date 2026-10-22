#!/bin/bash
# Run every gtest executable of one build variant (inside ns_batch: private roscore available).
# usage: run_gtests.sh VARIANT OUT_DIR
V=$1; O=$2
RR=/home/autonav/2026AIM/Lidar_experimments/track_competition_stage1_20261022_134204
D=$RR/repo/src/lidar+sensorfusion/build/exp_$V/devel/lib/Main
mkdir -p $O
: > $O/summary.txt
for b in $(ls $D | grep -E '^test_|_test$' | sort); do
  [ -x "$D/$b" ] || continue
  ( cd /tmp && timeout 600 "$D/$b" --gtest_output=xml:$O/gtest_$b.xml > $O/$b.log 2>&1 ); rc=$?
  pass=$(grep -c '^\[       OK \]' $O/$b.log); fail=$(grep -c '^\[  FAILED  \].*(' $O/$b.log)
  echo "$b rc=$rc ok=$pass failed=$fail" >> $O/summary.txt
done
cat $O/summary.txt
