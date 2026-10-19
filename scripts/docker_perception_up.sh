#!/usr/bin/env bash
# Perception ML 학습 컨테이너 제어
set -euo pipefail
COMPOSE_DIR="$(dirname "$(readlink -f "$0")")/../docker/perception-train"
REPO_ROOT="$(cd "$COMPOSE_DIR/../.." && pwd)"
source "$REPO_ROOT/scripts/lib/storage.sh"
asmc_storage_load "$REPO_ROOT"
asmc_storage_require_overlay "${ASMC_TRAIN_STORAGE_COMPOSE:-}"
case "${1:-up}" in
  up|rebuild) asmc_storage_prepare_data "$REPO_ROOT" ;;
esac
COMPOSE_ARGS=(-f "$COMPOSE_DIR/docker-compose.yaml")

if [[ -n "${ASMC_TRAIN_STORAGE_COMPOSE:-}" ]]; then
  COMPOSE_ARGS+=(-f "$ASMC_TRAIN_STORAGE_COMPOSE")
fi

cmd="${1:-up}"
echo "[DATA] $ASMC_DATA -> /data (ro)"
echo "[TWINLITE] $TWINLITE_SOURCE -> /opt/baselines/TwinLiteNetPlus (ro)"
case "$cmd" in
  config)
    docker compose "${COMPOSE_ARGS[@]}" config
    ;;
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
    echo "Usage: $0 {config|build|up|down|rebuild}"
    exit 1
    ;;
esac
