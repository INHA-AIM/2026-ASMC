#!/bin/bash
# Post-processing of queue A results (read-only on runs; writes analysis outputs).
set -u
RR=/home/autonav/2026AIM/Lidar_experimments/track_competition_stage1_20261022_134204
E3=/home/autonav/2026AIM/Lidar_experimments/obb_init_turn_follow_20261019_032258
E6=/home/autonav/2026AIM/Lidar_experimments/obb_measurement_identity_20261021_045723
X=$E3/repo/tools/experiments/lidar_obb_init_turn_follow
EV=$RR/repo/tools/experiments/lidar_obb_quality_temporal
S=$RR/scripts; W=$RR/work; A=$W/analysis; mkdir -p $A
export PYTHONDONTWRITEBYTECODE=1
cd $W
# 1. ARM.json (frozen pre-holdout evaluator, contract e82f3bcf) for every real-data arm
for r in CB CAND_A CAND_B CAND_AB; do
  [ -f runs/$r/ARM.json ] || ( python3 -B $X/preholdout_eval.py arm --pre $X/PREHOLDOUT_EVAL_CONTRACT.json --root $E3/repo --samples runs/$r/samples --out runs/$r/ARM.json > runs/$r/arm.log 2>&1; echo "arm $r rc=$?" ) &
done
wait
# 2. supplementary metrics (real data; evaluation pose only for ego categories)
python3 -B $S/stage1_metrics.py $A/stage1_metrics_all.json $E3/work/inputs P0=$E3/work/runs/p0 E06=$E6/work/runs/S3_F4L3 \
  ID_OFF_hist=runs/ID_OFF_hist CB=runs/CB CAND_A=runs/CAND_A CAND_B=runs/CAND_B CAND_AB=runs/CAND_AB > $A/stage1_metrics_all.log 2>&1
# 3. comparison tables
python3 -B $S/compare_arms.py $A/COMPARE_HIST_vs_COMP.json $A/COMPARE_HIST_vs_COMP.md P0 \
  P0=$E3/work/runs/p0:$A/stage1_metrics_all.json E06=$E6/work/runs/S3_F4L3:$A/stage1_metrics_all.json CB=runs/CB:$A/stage1_metrics_all.json
python3 -B $S/compare_arms.py $A/COMPARE_CANDIDATES.json $A/COMPARE_CANDIDATES.md CB \
  CB=runs/CB:$A/stage1_metrics_all.json CAND_A=runs/CAND_A:$A/stage1_metrics_all.json CAND_B=runs/CAND_B:$A/stage1_metrics_all.json CAND_AB=runs/CAND_AB:$A/stage1_metrics_all.json
# 4. reference-point study
for r in CB CAND_AB; do python3 -B $S/diag_reference_point.py runs/$r $E3/work/inputs $A/reference_point_$r.json > $A/reference_point_$r.log 2>&1; done
# 5. forbidden-field isolation
mkdir -p $A/isolation
for v in forbid_perturb forbid_removed allowed_perturb; do
  python3 -B $EV/identity_check.py runs/ISO_orig/out runs/ISO_$v/out $A/isolation/IDENTITY_orig_vs_$v.json > $A/isolation/$v.log 2>&1
done
# 6. synthetic DIAGNOSTIC_ONLY report
python3 -B $S/syn_report.py $A/SYNTHETIC_REPORT.json $W/synthetic SYN_BASE=runs/SYN_BASE SYN_A=runs/SYN_A SYN_B=runs/SYN_B SYN_AB=runs/SYN_AB > $A/SYNTHETIC_REPORT.txt 2>&1
# 7. trace neutrality on real scenes (trace ON vs P0 reference subset)
R=$W/ref_subset_TRC; rm -rf $R; mkdir -p $R
for sc in scen93 scen21 scen58 static_bag; do ln -s $E3/work/runs/p0/out/$sc.jsonl $R/$sc.jsonl; ln -s $E3/work/runs/p0/out/$sc.pts.bin $R/$sc.pts.bin; done
python3 -B $EV/identity_check.py $R runs/TRC_hist/out $A/IDENTITY_TRACE_ON_vs_P0.json > $A/identity_trace_on.log 2>&1
echo POST_STAGE1_DONE > $A/POST_DONE
