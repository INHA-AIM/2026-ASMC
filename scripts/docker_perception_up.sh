#!/usr/bin/env bash
# Perception ML 학습 컨테이너 제어
set -euo pipefail
COMPOSE_DIR="$(dirname "$(readlink -f "$0")")/../docker/perception-train"
export ASMC="${ASMC:-$(dirname "$(readlink -f "$0")")/..}"
export ASMC_DATA="${ASMC_DATA:-$(dirname "$(readlink -f "$ASMC")")/data}"
export TWINLITE_SOURCE="${TWINLITE_SOURCE:-$(dirname "$(readlink -f "$ASMC")")/external/baselines/TwinLiteNetPlus}"
mkdir -p "$ASMC_DATA"
COMPOSE_ARGS=(-f "$COMPOSE_DIR/docker-compose.yaml")
mapfile -t original_roots < <(python3 - "${ASMC_DATA_LOCATIONS:-$ASMC/src/data_collection/config/data_locations.json}" <<'PY'
import json,sys
with open(sys.argv[1]) as stream:
    locations=json.load(stream)
print(locations['aim_directory_wsl'])
print(locations['capture_archive_wsl'])
PY
)
export ASMC_AIM_ROOT="${ASMC_AIM_ROOT:-${original_roots[0]}}"
export ASMC_MORAI_ARCHIVE="${ASMC_MORAI_ARCHIVE:-${original_roots[1]}}"
if [[ -d "$ASMC_AIM_ROOT" && -d "$ASMC_MORAI_ARCHIVE" ]]; then
  COMPOSE_ARGS+=(-f "$COMPOSE_DIR/docker-compose.data.yaml")
  echo "[ORIGINALS] AIM=$ASMC_AIM_ROOT; Capture archive=$ASMC_MORAI_ARCHIVE (ro)"
else
  echo "[ORIGINALS] local AIM/archive paths not both available; external raw mounts omitted"
fi

cmd="${1:-up}"
echo "[DATA] $ASMC_DATA -> /data (ro)"
echo "[TWINLITE] $TWINLITE_SOURCE -> /opt/baselines/TwinLiteNetPlus (ro)"
case "$cmd" in
  build)
    docker compose "${COMPOSE_ARGS[@]}" build
    ;;
  up)
    docker compose "${COMPOSE_ARGS[@]}" up -d --force-recreate
    docker ps --filter name=asmc-perception-train
    ;;
  down)
    docker compose "${COMPOSE_ARGS[@]}" down
    ;;
  rebuild)
    docker compose "${COMPOSE_ARGS[@]}" build --no-cache
    docker compose "${COMPOSE_ARGS[@]}" up -d --force-recreate
    ;;
  *)
    echo "Usage: $0 {build|up|down|rebuild}"
    exit 1
    ;;
esac
