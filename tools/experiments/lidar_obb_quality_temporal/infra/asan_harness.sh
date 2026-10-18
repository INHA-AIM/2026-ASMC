#!/bin/bash
# ASan+UBSan replay of the real LidarProcess->tracking->output path: the ASan-built lidar_dataset_replay with candidate features ON (ON_ARGS) and OFF
# on a few scenes inside one isolated namespace; plus a positive control (1 leak + 1 UB shift) that must report.
# usage: [ON_ARGS="--set k=v ..."] asan_harness.sh VARIANT OUTDIR SCENE...   (mode on = candidate features enabled via ON_ARGS)
set -u
VARIANT=$1; O=$2; shift 2
RR=/home/autonav/2026AIM/Lidar_experimments/obb_quality_temporal_20261018_170239
D=${SRC_ROOT:-$RR/repo}/src/lidar+sensorfusion/build/exp_$VARIANT/devel
BIN=$D/lib/Main/lidar_dataset_replay; IN=$RR/work/inputs; export PYTHONDONTWRITEBYTECODE=1
mkdir -p "$O/selftest"
cat > "$O/selftest/selftest.cpp" <<'CPP'
#include <cstdio>
int main(int argc, char**) { int* leak = new int[4]; leak[0] = argc; int s = 31 + argc; int ub = 1 << s;
  std::printf("%d %d\n", leak[0], ub); leak = nullptr; return 0; }
CPP
g++ -O0 -g -fsanitize=address,undefined -fno-omit-frame-pointer "$O/selftest/selftest.cpp" -o "$O/selftest/selftest"
bash "$RR/tools/dump_params.sh" "$VARIANT" "$O/params.yaml" > /dev/null 2>&1
J=$O/jobs.txt; : > "$J"
san() { echo "ASAN_OPTIONS=log_path=$1/asan:detect_leaks=1:handle_abort=1:abort_on_error=0:fast_unwind_on_malloc=0:malloc_context_size=40 UBSAN_OPTIONS=log_path=$1/ubsan:print_stacktrace=1:halt_on_error=0"; }
mkdir -p "$O/san_selftest"; echo "$(san $O/san_selftest) setarch $(uname -m) -R $O/selftest/selftest > $O/selftest.log 2>&1; echo \$? > $O/selftest.rc" >> "$J"
for s in "$@"; do
  for mode in on off; do
    G=""; [ $mode = on ] && G="${ON_ARGS:-}"
    mkdir -p "$O/san_${s}_$mode"
    echo "$(san $O/san_${s}_$mode) setarch $(uname -m) -R $BIN --frames $IN/$s/frames.csv --scene $s --params $O/params.yaml --set imu_yaw_is_true_enu=false $G --out $O/${s}_$mode.jsonl > $O/${s}_$mode.log 2>&1; echo \$? > $O/${s}_$mode.rc" >> "$J"
  done
done
bash "$RR/tools/ns_batch.sh" "asan_harness_$VARIANT" "$D/setup.bash" "$J" 4 7200 > "$O/ns.log" 2>&1
for d in "$O"/san_*; do
  n=$(basename "$d"); files=$(ls "$d" 2>/dev/null | wc -l)
  kinds=$(cat "$d"/* 2>/dev/null | grep -o -E "ERROR: (AddressSanitizer|LeakSanitizer): [a-z-]+|runtime error" | sort | uniq -c | tr '\n' ';')
  printf '{"report_files": %d, "kinds": "%s"}\n' "$files" "$kinds" > "$O/$n.summary.json"
  echo "$n files=$files $kinds"
done
cat "$RR/logs/ns/asan_harness_$VARIANT/udp_snmp.json"
