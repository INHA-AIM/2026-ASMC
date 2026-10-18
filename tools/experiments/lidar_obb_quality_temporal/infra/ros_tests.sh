#!/bin/bash
# Main rostests (frame_boundary, frame_boundary_v2, main_integration_gating, camera_objects_adapter), each in its
# own fresh isolated namespace with a private master (ns_batch.sh), then the 27-scenario GPS-denial truth-table
# matrix through the real Main (tools/b0_tests/40_integration_matrix.sh, planner_udp_enabled:=false).
# usage: ros_tests.sh VARIANT OUTDIR [gate]   (gate: matrix through b0_test_wrapper_gate.launch, gate enabled)
set -u
VARIANT=$1; OUT=$2; MODE=${3:-}
RR=/home/autonav/2026AIM/Lidar_experimments/obb_quality_temporal_20261018_170239
T=$RR/tools
D=$RR/repo/src/lidar+sensorfusion/build/exp_$VARIANT/devel
mkdir -p "$OUT"
SUM=$OUT/ros_tests_summary.tsv; printf "name\trc\tresult_line\n" > "$SUM"
if [ "$MODE" != gate ]; then
  for t in frame_boundary frame_boundary_v2 main_integration_gating camera_objects_adapter; do
    J=$OUT/job_$t.txt
    echo "cd $OUT && rostest --reuse-master Main $t.test > $OUT/rostest_$t.log 2>&1; echo \$? > $OUT/rostest_$t.rc" > "$J"
    bash "$T/ns_batch.sh" "ros_${VARIANT}_$t" "$D/setup.bash" "$J" 1 600 > "$OUT/ns_$t.log" 2>&1
    cp "$RR/logs/ns/ros_${VARIANT}_$t/udp_snmp.json" "$OUT/udp_$t.json"
    printf "%s\t%s\t%s\n" "$t" "$(cat $OUT/rostest_$t.rc 2>/dev/null)" "$(grep -a -E 'SUMMARY|RESULT|Ran [0-9]+ test' $OUT/rostest_$t.log | tr '\n' ' ' | cut -c1-200)" >> "$SUM"
  done
fi
export B0_NESTED_DEVEL=$D B0_RT=$OUT/matrix_runtime B0_LB=$OUT/matrix_logs B0_PREFIX=b0_normal
if [ "$MODE" = gate ]; then export B0_PREFIX=b0_gate B0_WRAP=$T/b0_tests/b0_test_wrapper_gate.launch; fi
mkdir -p "$OUT/matrix_runtime" "$OUT/matrix_logs"
bash "$T/b0_tests/40_integration_matrix.sh" integration > "$OUT/matrix_driver.log" 2>&1
echo "matrix rc=$?" >> "$OUT/matrix_driver.log"
cat "$SUM"; cat "$OUT/matrix_logs/40_matrix_integration.tsv"
