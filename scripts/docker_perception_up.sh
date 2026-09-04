#!/usr/bin/env bash
# Perception ML 학습 컨테이너 제어
set -euo pipefail
COMPOSE_DIR="$(dirname "$(readlink -f "$0")")/../docker/perception-train"
export ASMC="${ASMC:-$(dirname "$(readlink -f "$0")")/..}"
export ASMC_DATA="${ASMC_DATA:-$(dirname "$(readlink -f "$ASMC")")/data}"
export TWINLITE_SOURCE="${TWINLITE_SOURCE:-$(dirname "$(readlink -f "$ASMC")")/external/baselines/TwinLiteNetPlus}"
mkdir -p "$ASMC_DATA"

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
