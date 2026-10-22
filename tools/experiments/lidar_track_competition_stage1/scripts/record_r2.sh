#!/bin/bash
# Record the R2 continuation (2026-10-22 23:15 KST session) on the experiment branch: copy the new
# results/scripts into the repo record prefix (same layout as commit 5c12c346) and commit. Local only, no push.
set -eu
RR=/home/autonav/2026AIM/Lidar_experimments/track_competition_stage1_20261022_134204
P=$RR/repo/tools/experiments/lidar_track_competition_stage1
cp -n $RR/work/analysis/reference_point_raw_TRC_CB.json $RR/work/analysis/reference_point_raw_TRC_AB.json $RR/work/analysis/speed_lag_split.json $RR/results/ 2>/dev/null || true
mkdir -p $P/results/trace_examples $P/scripts
for f in COMPETITION_CHANNEL_IDENTITY.json DIAGNOSIS_ADDENDUM_KO.md FIRST_STAGE_REPORT_KO_R2.md INPUT_CONTRACT_ADDENDUM_KO.md \
         STAGE1_REVERIFY_20261022T2315.json reference_point_raw_TRC_CB.json reference_point_raw_TRC_AB.json speed_lag_split.json; do
  cp $RR/results/$f $P/results/$f
done
cp $RR/results/trace_examples/REFPOINT_*.csv $P/results/trace_examples/
cp $RR/scripts/diag_reference_point_raw.py $RR/scripts/diag_speed_lag_split.py $RR/scripts/record_r2.sh $P/scripts/
cd $RR/repo
git add tools/experiments/lidar_track_competition_stage1
cat > $RR/work/commit3_msg.txt <<'EOF'
docs(lidar): record stage-1 R2: reverify, channel identity correction, reference-point and lag analyses

Continuation of the stage-1 record (5c12c346) after the stage-1 instruction was re-issued:
- STAGE1_REVERIFY_20261022T2315.json: manifest SHA256 of all built binaries re-checked (all same), worktree clean, no live instance
- INPUT_CONTRACT_ADDENDUM_KO.md + COMPETITION_CHANNEL_IDENTITY.json: Competition Vehicle Status is PUBSUB 295
  (#MoraiInfo$ 181 B) -> 9099 -> /Ego_topic; 787 is Ego Detail Status (105 B, judging only). Withdraws the R1
  P1 remap-violation claim; keeps P2 (velocity norm). Records hazard H3 (local MORAI save routes 770 -> 9099)
- DIAGNOSIS_ADDENDUM_KO.md: raw L-shape OBB centre vs point mean vs GT (bias halves, stationary jitter unchanged
  0.54 m p95); speed-lag split by ego motion (A+B lag increase confined to ego accel/decel, co-moving segments)
- FIRST_STAGE_REPORT_KO_R2.md: revised entry document for GPT review
- scripts: diag_reference_point_raw.py, diag_speed_lag_split.py, record_r2.sh (read-only analyses, no code change)

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>
EOF
git commit -q -F $RR/work/commit3_msg.txt
git log --oneline -1
git status --short | wc -l
