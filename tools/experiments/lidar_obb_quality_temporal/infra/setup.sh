#!/bin/bash
# Experiment 02 bootstrap: verify the protected source repo, create RUN_ROOT + local branch + worktree.
set -euo pipefail
export GIT_OPTIONAL_LOCKS=0 PYTHONDONTWRITEBYTECODE=1
SRC=/home/autonav/2026AIM/2026-ASMC-nogps
BASE=916e4d9efb38f9066aa4b079df05d16ef011c063
EXP=/home/autonav/2026AIM/Lidar_experimments
BOOT=$(cd "$(dirname "$0")" && pwd)

for i in 1 2 3 4 5; do
  TS=$(TZ=Asia/Seoul date +%Y%m%d_%H%M%S)
  RUN_ROOT=$EXP/obb_quality_temporal_$TS
  BR=exp/lidar-obb-quality-temporal-$TS
  if [ ! -e "$RUN_ROOT" ] && ! git -C "$SRC" show-ref --verify --quiet "refs/heads/$BR"; then break; fi
  sleep 1
done
mkdir "$RUN_ROOT"
mkdir -p "$RUN_ROOT/results" "$RUN_ROOT/logs" "$RUN_ROOT/tools" "$RUN_ROOT/work"
cp "$BOOT"/source_snapshot.py "$BOOT"/dataset_integrity.py "$BOOT"/runtime_state.sh "$BOOT"/setup.sh "$RUN_ROOT/tools/"
{
echo "RUN_ROOT=$RUN_ROOT"; echo "TS=$TS"; echo "BRANCH=$BR"; echo "WORKTREE=$RUN_ROOT/repo"
echo "RESULTS=$RUN_ROOT/results"; echo "WORK=$RUN_ROOT/work"; echo "LOGS=$RUN_ROOT/logs"; echo "BASE=$BASE"
} > "$RUN_ROOT/RUN_ENV"
cat "$RUN_ROOT/RUN_ENV"

set +e
python3 -B "$RUN_ROOT/tools/source_snapshot.py" "$RUN_ROOT/results/SOURCE_SNAPSHOT_BEFORE.json" 2>&1 | tee "$RUN_ROOT/logs/source_snapshot_before.log"
rc=${PIPESTATUS[0]}
set -e
if [ "$rc" != "0" ]; then
  echo "BLOCKED_SOURCE_MOVED_OR_DIRTY rc=$rc" | tee "$RUN_ROOT/results/BLOCKED.txt"
  exit 3
fi
bash "$RUN_ROOT/tools/runtime_state.sh" "$RUN_ROOT/results/RUNTIME_STATE_BEFORE.txt"
git -C "$SRC" worktree add -b "$BR" "$RUN_ROOT/repo" "$BASE" 2>&1 | tee "$RUN_ROOT/logs/worktree_add.log"
echo "worktree_head=$(git -C "$RUN_ROOT/repo" rev-parse HEAD) branch=$(git -C "$RUN_ROOT/repo" branch --show-current)"
git -C "$RUN_ROOT/repo" status --porcelain=v1 | head
git -C "$RUN_ROOT/repo" submodule status 2>&1 | head
