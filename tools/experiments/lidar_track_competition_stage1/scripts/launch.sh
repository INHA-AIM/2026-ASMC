#!/bin/bash
# Run a tool script from an immutable snapshot copy (bash reads scripts incrementally; editing a running
# script must never change a run in progress). usage: launch.sh SCRIPT [ARGS...]
set -u
RR=/home/autonav/2026AIM/Lidar_experimments/track_competition_stage1_20261022_134204
S=$1; shift
D=$RR/work/script_snapshots; mkdir -p "$D"
C=$D/$(date +%Y%m%d_%H%M%S_%N)_$(basename "$S")
cp "$S" "$C"; chmod 444 "$C"
echo "snapshot=$C sha256=$(sha256sum "$C" | cut -d' ' -f1)" >&2
exec bash "$C" "$@"
