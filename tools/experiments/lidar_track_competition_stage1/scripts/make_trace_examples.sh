#!/bin/bash
# Consecutive-frame GT -> cluster -> association -> KF -> OBB -> output chains for the diagnostic objects.
RR=/home/autonav/2026AIM/Lidar_experimments/track_competition_stage1_20261022_134204
O=$RR/results/trace_examples; mkdir -p $O
IN=$RR/work/inputs_comp
# scene gt_tid from to label
while read sc gt lo hi label; do
  for arm in TRC_CB TRC_AB; do
    python3 -B $RR/scripts/trace_link.py $RR/work/runs/$arm $IN $sc $O/${arm}_${sc}_gt${gt}_${label} --gt-tid $gt --from $lo --to $hi
  done
done <<EOF
scen16 22 555 580 ego_turn_parked_vehicle
scen10 16 685 712 ego_turn_parked_vehicle
scen92 8 352 400 ego_straight_parked_vehicle
scen56 3 0 61 target_accel_vehicle
scen50 0 249 300 target_turn_vehicle
scen21 6 115 145 ped_next_to_vehicle
scen21 1 115 145 vehicle_stopping_next_to_ped
scen39 12 535 565 short_occlusion_reacquire
scen75 16 284 330 ego_stopped_parked_vehicle
EOF
ls $O | wc -l
