#!/bin/bash
# Stage-1 run queue (sequential, each arm in its own isolated namespace via run_arm2.sh).
set -u
RR=/home/autonav/2026AIM/Lidar_experimments/track_competition_stage1_20261022_134204
S=$RR/scripts; L=$RR/logs/queue_stage1.log
A="--set lidar_kf_reacquire_elapsed_init=true"
B="--set lidar_kf_ego_translation_comp=true"
P=${PAR:-6}
run() { echo "START $1 $(date -Is)" >> $L; shift; "$@" >> $L 2>&1; echo "END rc=$? $(date -Is)" >> $L; }
echo "QUEUE_START $(date -Is) load=$(cat /proc/loadavg)" >> $L
# 1. default-off identity of the instrumented candidate binary vs P0 (historical inputs, all 115)
run ID_OFF_hist bash $S/launch.sh $S/run_arm2.sh ID_OFF_hist cand $P historical
# 2. Competition baseline (candidate binary, all options off, competition inputs)
run CB bash $S/launch.sh $S/run_arm2.sh CB cand $P competition
# 3. forbidden-field isolation (scen58, scen93)
for v in orig forbid_perturb forbid_removed allowed_perturb; do
  run ISO_$v env SCENES="scen58 scen93" IN_OVERRIDE=$RR/work/isolation/$v POINTS=0 bash $S/launch.sh $S/run_arm2.sh ISO_$v cand $P competition
done
# 4. candidates
run CAND_A bash $S/launch.sh $S/run_arm2.sh CAND_A cand $P competition $A
run CAND_B bash $S/launch.sh $S/run_arm2.sh CAND_B cand $P competition $B
run CAND_AB bash $S/launch.sh $S/run_arm2.sh CAND_AB cand $P competition $A $B
# 5. synthetic DIAGNOSTIC_ONLY arms with full traces
SYN=$(cd $RR/work/synthetic && ls -d syn* | tr '\n' ' ')
run SYN_BASE env TRACE_SCENES="$SYN" bash $S/launch.sh $S/run_arm2.sh SYN_BASE cand $P synthetic
run SYN_A env TRACE_SCENES="$SYN" bash $S/launch.sh $S/run_arm2.sh SYN_A cand $P synthetic $A
run SYN_B env TRACE_SCENES="$SYN" bash $S/launch.sh $S/run_arm2.sh SYN_B cand $P synthetic $B
run SYN_AB env TRACE_SCENES="$SYN" bash $S/launch.sh $S/run_arm2.sh SYN_AB cand $P synthetic $A $B
# 6. trace neutrality on real scenes (historical inputs, trace ON) and diagnostic traces of the competition arms
run TRC_hist env SCENES="scen93 scen21 scen58 static_bag" TRACE_SCENES="scen93 scen21 scen58 static_bag" bash $S/launch.sh $S/run_arm2.sh TRC_hist cand $P historical
echo "QUEUE_DONE $(date -Is)" >> $L
