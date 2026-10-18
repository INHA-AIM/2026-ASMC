#!/bin/bash
# Run one experiment arm over the frozen scenes (+ the static GT bag) inside ONE isolated namespace (ns_batch.sh),
# then evaluate every scene and compute the set metrics. Run it from an immutable snapshot (launch.sh).
# usage: run_arm.sh RUN_NAME VARIANT PARALLEL [extra harness args, e.g. --set lidar_obb_quality_enabled=true]
# env:   ROLE_FILTER=development|holdout|highspeed (only scenes of that manifest role, static bag skipped)
#        SCENES="scenA scenB"  (explicit scene list, overrides ROLE_FILTER, static bag skipped)
#        POINTS=1|0 (write cluster point dumps, default 1)   CF=1|0 (clustering counterfactual in eval, default 0)
#        EVAL_PAR (default 7)   METRICS=1|0 (default 1)
set -u
NAME=$1; VARIANT=$2; PAR=$3; shift 3; EXTRA="$*"
RR=/home/autonav/2026AIM/Lidar_experimments/obb_quality_temporal_20261018_170239
T=$RR/tools; E=$RR/repo/tools/experiments/lidar_obb_quality_temporal
D=$RR/repo/src/lidar+sensorfusion/build/exp_$VARIANT/devel
BIN=$D/lib/Main/lidar_dataset_replay
IN=$RR/work/inputs; SB=$RR/work/static_bag
ROLE_FILTER=${ROLE_FILTER:-}; SCENES=${SCENES:-}; POINTS=${POINTS:-1}; CF=${CF:-0}; EVAL_PAR=${EVAL_PAR:-7}; METRICS=${METRICS:-1}
export PYTHONDONTWRITEBYTECODE=1
O=$RR/work/runs/$NAME; S=$O/samples
[ -e "$O" ] && { echo "run dir exists: $O"; exit 3; }
mkdir -p "$O/out" "$S"
PARAMS=$O/params.yaml
bash "$T/dump_params.sh" "$VARIANT" "$PARAMS" > "$O/dump_params.log" 2>&1 || { echo "param dump failed"; exit 4; }
sha256sum "$BIN" "$D/lib/libmyHeader.so" "$PARAMS" > "$O/binary_sha256.txt"
echo "name=$NAME variant=$VARIANT head=$(git -C $RR/repo rev-parse HEAD) extra=[$EXTRA] par=$PAR role=[$ROLE_FILTER] scenes=[$SCENES] points=$POINTS cf=$CF start=$(date -Is)" > "$O/run_meta.txt"
if [ -n "$SCENES" ]; then LIST=$SCENES; else
  LIST=$(python3 -B -c "import json;m=json.load(open('$IN/INPUT_MANIFEST.json'));r='$ROLE_FILTER';print(' '.join(sorted([s for s in m['scenes'] if not r or m['scenes'][s]['role']==r],key=lambda s:-m['scenes'][s]['n_frames'])))")
fi
PTS() { [ "$POINTS" = 1 ] && echo "--points-out $O/out/$1.pts.bin"; }
JOBS=$O/jobs.txt; : > "$JOBS"
for sc in $LIST; do
  echo "$BIN --frames $IN/$sc/frames.csv --scene $sc --params $PARAMS --set imu_yaw_is_true_enu=false $EXTRA --out $O/out/$sc.jsonl $(PTS $sc) > $O/out/$sc.log 2>&1; echo \$? > $O/out/$sc.rc" >> "$JOBS"
done
STATIC=0; [ -z "$ROLE_FILTER" ] && [ -z "$SCENES" ] && STATIC=1
[ "$STATIC" = 1 ] && echo "$BIN --frames $SB/static_bag/frames.csv --scene static_bag --params $PARAMS --set imu_yaw_is_true_enu=false $EXTRA --out $O/out/static_bag.jsonl $(PTS static_bag) > $O/out/static_bag.log 2>&1; echo \$? > $O/out/static_bag.rc" >> "$JOBS"
bash "$T/ns_batch.sh" "$NAME" "$D/setup.bash" "$JOBS" "$PAR" 14400 > "$O/ns_batch.log" 2>&1
echo "ns_batch rc=$? $(tail -1 $O/ns_batch.log | cut -c1-400)" >> "$O/run_meta.txt"
cp "$RR/logs/ns/$NAME/udp_snmp.json" "$O/udp_snmp.json"
echo "nonzero_rc_files=$(grep -L '^0$' $O/out/*.rc | wc -l) total_rc=$(ls $O/out/*.rc | wc -l)" >> "$O/run_meta.txt"
grep -h DATASET_REPLAY_TOTAL $O/out/*.log > "$O/replay_totals.txt"
CFA=""; [ "$CF" = 1 ] && CFA="--counterfactual"
EJ=$O/eval_jobs.txt; : > "$EJ"
for sc in $LIST; do
  PA=""; [ -f "$O/out/$sc.pts.bin" ] && PA="--points $O/out/$sc.pts.bin"
  echo "cd $O && python3 -B $E/eval_lib.py scene --contract $E/EVAL_CONTRACT.json --inputs $IN --pred $O/out/$sc.jsonl $PA $CFA --scene $sc --out $S/$sc.pkl >> $O/eval.log 2>&1" >> "$EJ"
done
if [ "$STATIC" = 1 ]; then
  PA=""; [ -f "$O/out/static_bag.pts.bin" ] && PA="--points $O/out/static_bag.pts.bin"
  echo "cd $O && python3 -B $E/eval_lib.py scene --contract $E/EVAL_CONTRACT.json --inputs $SB --pred $O/out/static_bag.jsonl $PA $CFA --scene static_bag --out $S/static_bag.pkl >> $O/eval.log 2>&1" >> "$EJ"
fi
xargs -P "$EVAL_PAR" -I{} -d '\n' bash -c '{}' < "$EJ"
echo "eval pkls=$(ls $S/*.pkl 2>/dev/null | wc -l) expected=$(wc -l < $EJ)" >> "$O/run_meta.txt"
if [ "$METRICS" = 1 ] && [ "$STATIC" = 1 ]; then
  (cd $O && python3 -B $E/eval_lib.py metrics --contract $E/EVAL_CONTRACT.json --samples $S --manifest $IN/INPUT_MANIFEST.json --out $O/METRICS.json > $O/metrics.log 2>&1)
  echo "metrics rc=$?" >> "$O/run_meta.txt"
fi
echo "end=$(date -Is)" >> "$O/run_meta.txt"
cat "$O/run_meta.txt"
