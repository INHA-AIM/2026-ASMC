#!/bin/bash
# Stage-1 P0 build: outer workspace (own devel) then nested Main variant exp_p0 at HEAD 39cffc57 (P0 options).
set -u
RR=/home/autonav/2026AIM/Lidar_experimments/track_competition_stage1_20261022_134204
S=$RR/scripts
echo "start $(date -Is) head=$(git -C $RR/repo rev-parse HEAD)" > $RR/logs/build_p0_all.log
bash $S/build_outer.sh >> $RR/logs/build_p0_all.log 2>&1
echo "outer rc=$?" >> $RR/logs/build_p0_all.log
OUTER_DEVEL=$RR/repo/devel BUILD_JOBS=4 BUILD_LOAD=8 bash $S/build_nested.sh p0 >> $RR/logs/build_p0_all.log 2>&1
echo "nested rc=$?" >> $RR/logs/build_p0_all.log
echo "BUILD_P0_ALL_DONE $(date -Is)" >> $RR/logs/build_p0_all.log
