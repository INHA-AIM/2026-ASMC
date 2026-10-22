#!/bin/bash
# Read-only snapshot of protected evidence (past experiment results, repo HEADs, worktree status).
# usage: protected_snapshot.sh <out_json>
set -u
OUT="$1"
LE=/home/autonav/2026AIM/Lidar_experimments
TMP=$(mktemp -d)
{
  echo "{"
  echo "\"taken_at\": \"$(date -Iseconds)\","
  echo "\"experiments\": {"
  first=1
  for d in obb_innovation_gate_20261018_113148 obb_quality_temporal_20261018_170239 obb_init_turn_follow_20261019_032258 obb_provisional_turn_v2_20261019_202308 obb_geometry_decoupled_20261020_200235 obb_measurement_identity_20261021_045723; do
    R=$LE/$d
    res_hash=$(cd $R/results && find . -type f -print0 | sort -z | xargs -0 sha256sum | sha256sum | cut -d" " -f1)
    res_n=$(find $R/results -type f | wc -l)
    head=$(git -C $R/repo rev-parse HEAD 2>/dev/null)
    st=$(git -C $R/repo status --porcelain=v1 2>/dev/null | sha256sum | cut -d" " -f1)
    stn=$(git -C $R/repo status --porcelain=v1 2>/dev/null | wc -l)
    [ $first -eq 1 ] || echo ","
    first=0
    echo "\"$d\": {\"results_tree_sha256\": \"$res_hash\", \"results_files\": $res_n, \"repo_head\": \"$head\", \"repo_status_sha256\": \"$st\", \"repo_status_lines\": $stn}"
  done
  echo "},"
  echo "\"repos\": {"
  first=1
  for r in /home/autonav/2026AIM/2026-ASMC /home/autonav/2026AIM/2026-ASMC-nogps /home/autonav/2026AIM/Yangtest; do
    head=$(git -C $r rev-parse HEAD 2>/dev/null)
    br=$(git -C $r rev-parse --abbrev-ref HEAD 2>/dev/null)
    idx=$(sha256sum $r/.git/index 2>/dev/null | cut -d" " -f1)
    [ -f $r/.git ] && idx=$(sha256sum $(git -C $r rev-parse --git-dir)/index | cut -d" " -f1)
    st=$(git -C $r status --porcelain=v1 2>/dev/null | sha256sum | cut -d" " -f1)
    [ $first -eq 1 ] || echo ","
    first=0
    echo "\"$r\": {\"head\": \"$head\", \"branch\": \"$br\", \"index_sha256\": \"$idx\", \"status_sha256\": \"$st\"}"
  done
  echo "}"
  echo "}"
} > "$OUT"
rm -rf $TMP
python3 -B -c "import json,sys; json.load(open(sys.argv[1])); print('valid json')" "$OUT"
