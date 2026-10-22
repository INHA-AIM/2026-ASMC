#!/bin/bash
# Stage-1 queue B: waits for queue A, then diagnostic traces on real scenes and the 15 Hz real-rate runs.
set -u
RR=/home/autonav/2026AIM/Lidar_experimments/track_competition_stage1_20261022_134204
S=$RR/scripts; L=$RR/logs/queue_stage1b.log
A="--set lidar_kf_reacquire_elapsed_init=true"
B="--set lidar_kf_ego_translation_comp=true"
DIAG="static_bag scen75 scen92 scen16 scen10 scen50 scen56 scen21 scen39"
run() { echo "START $1 $(date -Is)" >> $L; shift; "$@" >> $L 2>&1; echo "END rc=$? $(date -Is)" >> $L; }
echo "WAIT_QUEUE_A $(date -Is)" >> $L
until grep -q QUEUE_DONE $RR/logs/queue_stage1.log; do sleep 20; done
echo "QUEUE_B_START $(date -Is) load=$(cat /proc/loadavg)" >> $L
run TRC_CB env SCENES="$DIAG" TRACE_SCENES="$DIAG" POINTS=0 bash $S/launch.sh $S/run_arm2.sh TRC_CB cand 6 competition
run TRC_AB env SCENES="$DIAG" TRACE_SCENES="$DIAG" POINTS=0 bash $S/launch.sh $S/run_arm2.sh TRC_AB cand 6 competition $A $B
# ---- 15 Hz real-rate runs of the real Main node (one fresh namespace + roscore per run), ABAB order
RT=$RR/work/realtime; mkdir -p $RT
rt() {   # name variant scene trace [rosparams...]
  local name=$1 var=$2 sc=$3 tr=$4; shift 4
  local D=$RR/repo/src/lidar+sensorfusion/build/exp_$var/devel
  echo "bash $S/rt_job.sh $RT/$name $RR/work/inputs_comp/$sc/frames_comp.csv 15 40 $tr $*" > $RT/$name.job
  echo "load_before=$(cat /proc/loadavg)" > $RT/$name.load
  run RT_$name bash $S/ns_batch.sh rt_$name $D/setup.bash $RT/$name.job 1 300
  echo "load_after=$(cat /proc/loadavg)" >> $RT/$name.load
}
rt O0_off_scen21_r1   cand    scen21 0
rt O2_off_scen21_r1   cand_o2 scen21 0
rt O0_trace_scen21    cand    scen21 1
rt O2_trace_scen21    cand_o2 scen21 1
rt O0_AB_scen21       cand    scen21 0 lidar_kf_reacquire_elapsed_init=true lidar_kf_ego_translation_comp=true
rt O0_off_scen21_r2   cand    scen21 0
rt O2_off_scen21_r2   cand_o2 scen21 0
rt O0_off_scen75      cand    scen75 0
rt O2_off_scen75      cand_o2 scen75 0
rt O0_trace_scen75    cand    scen75 1
python3 -B $S/rt_analyze.py $RT/O0_off_scen21_r1 $RT/O2_off_scen21_r1 $RT/O0_trace_scen21 $RT/O2_trace_scen21 $RT/O0_AB_scen21 \
   $RT/O0_off_scen21_r2 $RT/O2_off_scen21_r2 $RT/O0_off_scen75 $RT/O2_off_scen75 $RT/O0_trace_scen75 > $RT/RT_SUMMARY.json 2> $RT/rt_analyze.err
echo "QUEUE_B_DONE $(date -Is)" >> $L
