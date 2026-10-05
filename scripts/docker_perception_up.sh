#!/usr/bin/env bash
# Perception ML 학습 컨테이너 제어
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
COMPOSE_DIR="$REPO_ROOT/docker/perception-train"
export ASMC="$REPO_ROOT"
export ASMC_DATA="${ASMC_DATA:-$(dirname "$REPO_ROOT")/data}"
export TWINLITE_SOURCE="${TWINLITE_SOURCE:-$(dirname "$REPO_ROOT")/external/baselines/TwinLiteNetPlus}"
[ -d "$ASMC_DATA" ] || {
  echo "[FAIL] ASMC_DATA directory does not exist: $ASMC_DATA" >&2
  exit 1
}
[ -f "$TWINLITE_SOURCE/model/model.py" ] || {
  echo "[FAIL] TwinLite source is incomplete: $TWINLITE_SOURCE" >&2
  exit 1
}

cmd="${1:-up}"
echo "[DATA] $ASMC_DATA -> /data (ro)"
echo "[TWINLITE] $TWINLITE_SOURCE -> /opt/baselines/TwinLiteNetPlus (ro)"
case "$cmd" in
  build)
    docker compose -f "$COMPOSE_DIR/docker-compose.yaml" build
    ;;
  up)
    docker compose -f "$COMPOSE_DIR/docker-compose.yaml" up -d --force-recreate
    docker ps --filter name=asmc-perception-train
    ;;
  down)
    docker compose -f "$COMPOSE_DIR/docker-compose.yaml" down
    ;;
  rebuild)
    docker compose -f "$COMPOSE_DIR/docker-compose.yaml" build --no-cache
    docker compose -f "$COMPOSE_DIR/docker-compose.yaml" up -d --force-recreate
    ;;
  *)
    echo "Usage: $0 {build|up|down|rebuild}"
    exit 1
    ;;
esac
