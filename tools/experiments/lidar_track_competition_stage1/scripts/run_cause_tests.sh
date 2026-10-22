#!/bin/bash
# Run the stage-1 cause tests (normal and TRACKING_OPTIONS_FORCE_OFF=1) and record exit codes.
RR=/home/autonav/2026AIM/Lidar_experimments/track_competition_stage1_20261022_134204
D=$RR/repo/src/lidar+sensorfusion/build/exp_cand/devel
O=$RR/work/tests; mkdir -p $O
set +u; source /opt/ros/noetic/setup.bash; source $D/setup.bash
B=$D/lib/Main/test_tracking_frame_options
FILTER=${1:-*}
$B --gtest_filter="$FILTER" --gtest_output=xml:$O/cause_tests_normal.xml > $O/cause_tests_normal.log 2>&1
echo "normal rc=$?" | tee $O/cause_tests_rc.txt
TRACKING_OPTIONS_FORCE_OFF=1 $B --gtest_filter="$FILTER" --gtest_output=xml:$O/cause_tests_force_off.xml > $O/cause_tests_force_off.log 2>&1
echo "force_off rc=$?" | tee -a $O/cause_tests_rc.txt
tail -30 $O/cause_tests_normal.log
echo ===== force_off
grep -E "OK \]|FAILED|PASSED|Failure|Expected|Which is|value of" $O/cause_tests_force_off.log | head -40
