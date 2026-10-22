#!/bin/bash
# Run one arm (one binary variant + one input mode + extra --set args) over scenes inside ONE isolated
# namespace (ns_batch.sh: private roscore, lo only, UDP counted), then evaluate every scene with the
# frozen evaluator (eval_lib.py v2, EVAL_CONTRACT 71b4ef6f). Run it from a snapshot (launch.sh).
# usage: run_arm2.sh RUN_NAME VARIANT PAR MODE [extra harness args...]
#   MODE: historical  -> exp03 frozen inputs frames.csv (synthetic IMU/CAN/GPS from the evaluation ego pose; = P0)
#         competition -> $RR/work/inputs_comp/<sc>/frames_comp.csv with --input-mode competition (allowed channels only)
#         synthetic   -> $RR/work/synthetic/<sc>/frames_comp.csv with --input-mode competition (DIAGNOSTIC_ONLY)
# env: SCENES="a b" (default: all 114 + static_bag; synthetic: all syn*)  POINTS=1|0 (default 1)
#      TRACE_SCENES="a b" (track trace for these scenes)  TRACE_IDS=csv  EVAL_PAR (default 6)
set -u
NAME=$1; VARIANT=$2; PAR=$3; MODE=$4; shift 4; EXTRA="$*"
RR=/home/autonav/2026AIM/Lidar_experimments/track_competition_stage1_20261022_134204
E3=/home/autonav/2026AIM/Lidar_experimments/obb_init_turn_follow_20261019_032258
T=$RR/scripts; E=$RR/repo/tools/experiments/lidar_obb_quality_temporal
D=$RR/repo/src/lidar+sensorfusion/build/exp_$VARIANT/devel
BIN=$D/lib/Main/lidar_dataset_replay
POINTS=${POINTS:-1}; EVAL_PAR=${EVAL_PAR:-6}; TRACE_SCENES=${TRACE_SCENES:-}; TRACE_IDS=${TRACE_IDS:-}
export PYTHONDONTWRITEBYTECODE=1
case $MODE in
  historical)  IN=$E3/work/inputs; SBIN=$E3/work/static_bag; FR=frames.csv; MODEARG="" ;;
  competition) IN=$RR/work/inputs_comp; SBIN=$RR/work/inputs_comp; FR=frames_comp.csv; MODEARG="--input-mode competition" ;;
  synthetic)   IN=$RR/work/synthetic; SBIN=""; FR=frames_comp.csv; MODEARG="--input-mode competition" ;;
  *) echo "bad mode $MODE"; exit 2 ;;
esac
# IN_OVERRIDE: alternative inputs dir with the same layout (e.g. isolation variants)
if [ -n "${IN_OVERRIDE:-}" ]; then IN=$IN_OVERRIDE; SBIN=$IN_OVERRIDE; fi
O=$RR/work/runs/$NAME; S=$O/samples
[ -e "$O" ] && { echo "run dir exists: $O"; exit 3; }
mkdir -p "$O/out" "$S" "$O/trace"
PARAMS=$O/params.yaml
bash "$T/dump_params.sh" "$VARIANT" "$PARAMS" > "$O/dump_params.log" 2>&1 || { echo "param dump failed"; exit 4; }
sha256sum "$BIN" "$D/lib/libmyHeader.so" "$PARAMS" > "$O/binary_sha256.txt"
HEADC=$(git -C $RR/repo rev-parse HEAD); DIRTY=$(git -C $RR/repo status --porcelain=v1 | wc -l)
if [ -n "${SCENES:-}" ]; then LIST=$SCENES; else
  if [ "$MODE" = synthetic ]; then LIST=$(cd $IN && ls -d syn* | tr '\n' ' ');
  else LIST="$(python3 -B -c "import json;m=json.load(open('$E3/work/inputs/INPUT_MANIFEST.json'));print(' '.join(sorted(m['scenes'],key=lambda s:-m['scenes'][s]['n_frames'])))") static_bag"; fi
fi
echo "name=$NAME variant=$VARIANT mode=$MODE head=$HEADC worktree_dirty_lines=$DIRTY extra=[$EXTRA] par=$PAR scenes=[$LIST] points=$POINTS trace=[$TRACE_SCENES] start=$(date -Is)" > "$O/run_meta.txt"
git -C $RR/repo status --porcelain=v1 > "$O/worktree_status.txt"; git -C $RR/repo diff > "$O/worktree_diff.patch"
JOBS=$O/jobs.txt; : > "$JOBS"
for sc in $LIST; do
  if [ "$sc" = static_bag ] && [ "$MODE" = historical ]; then FRP=$SBIN/static_bag/$FR; else FRP=$IN/$sc/$FR; fi
  PA=""; [ "$POINTS" = 1 ] && PA="--points-out $O/out/$sc.pts.bin"
  TA=""; case " $TRACE_SCENES " in *" $sc "*) TA="--track-trace $O/trace/$sc.trace.jsonl --run-id $NAME:$sc"; [ -n "$TRACE_IDS" ] && TA="$TA --track-trace-ids $TRACE_IDS";; esac
  echo "$BIN --frames $FRP --scene $sc --params $PARAMS --set imu_yaw_is_true_enu=false $MODEARG $EXTRA $TA --out $O/out/$sc.jsonl $PA > $O/out/$sc.log 2>&1; echo \$? > $O/out/$sc.rc" >> "$JOBS"
done
bash "$T/ns_batch.sh" "$NAME" "$D/setup.bash" "$JOBS" "$PAR" 14400 > "$O/ns_batch.log" 2>&1
echo "ns_batch rc=$? $(tail -1 $O/ns_batch.log | cut -c1-400)" >> "$O/run_meta.txt"
cp "$RR/logs/ns/$NAME/udp_snmp.json" "$O/udp_snmp.json"
echo "nonzero_rc_files=$(grep -L '^0$' $O/out/*.rc | wc -l) total_rc=$(ls $O/out/*.rc | wc -l)" >> "$O/run_meta.txt"
grep -h DATASET_REPLAY_TOTAL $O/out/*.log > "$O/replay_totals.txt"
EJ=$O/eval_jobs.txt; : > "$EJ"
for sc in $LIST; do
  PA=""; [ -f "$O/out/$sc.pts.bin" ] && PA="--points $O/out/$sc.pts.bin"
  EIN=$IN; [ "$sc" = static_bag ] && EIN=$SBIN
  echo "cd $O && python3 -B $E/eval_lib.py scene --contract $E/EVAL_CONTRACT.json --inputs $EIN --pred $O/out/$sc.jsonl $PA --scene $sc --out $S/$sc.pkl >> $O/eval.log 2>&1" >> "$EJ"
done
xargs -P "$EVAL_PAR" -I{} -d '\n' bash -c '{}' < "$EJ"
echo "eval pkls=$(ls $S/*.pkl 2>/dev/null | wc -l) expected=$(wc -l < $EJ)" >> "$O/run_meta.txt"
echo "end=$(date -Is)" >> "$O/run_meta.txt"
echo "RUN_ARM2_DONE" >> "$O/run_meta.txt"
cat "$O/run_meta.txt"
