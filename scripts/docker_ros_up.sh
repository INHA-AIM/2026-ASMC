#!/usr/bin/env bash
# ROS Noetic 개발 컨테이너 제어
set -euo pipefail
COMPOSE_DIR="$(dirname "$(readlink -f "$0")")/../docker/ros-noetic"
export ASMC="${ASMC:-$(dirname "$(readlink -f "$0")")/..}"

# Docker Desktop+Windows: Dest UDP가 compose에 있어도 Windows 리스너가 빠지는 경우가 있음 (특히 9299).
_check_win_udp() {
  local port="$1"
  command -v powershell.exe >/dev/null 2>&1 || return 0
  powershell.exe -NoProfile -Command \
    "if (Get-NetUDPEndpoint -LocalPort $port -ErrorAction SilentlyContinue) { exit 0 } else { exit 1 }" \
    >/dev/null 2>&1
}

_verify_sensor_udp() {
  local missing=()
  local p
  for p in 9291 9299 9301 9303; do
    if ! _check_win_udp "$p"; then
      missing+=("$p")
    fi
  done
  if ((${#missing[@]})); then
    echo "[WARN] Windows UDP Dest not listening: ${missing[*]}"
    echo "       (cam/gps/imu OK but lidar=0 often means 9299 missing)"
    return 1
  fi
  echo "[OK] Windows UDP Dest open: 9291 9299 9301 9303"
  return 0
}

cmd="${1:-up}"
case "$cmd" in
  build)
    docker compose -f "$COMPOSE_DIR/docker-compose.yaml" build
    ;;
  up)
    docker compose -f "$COMPOSE_DIR/docker-compose.yaml" up -d --force-recreate
    docker ps --filter name=asmc-ros-noetic
    sleep 2
    if ! _verify_sensor_udp; then
      echo "[INFO] Recreating once to re-bind UDP..."
      docker compose -f "$COMPOSE_DIR/docker-compose.yaml" up -d --force-recreate
      sleep 2
      _verify_sensor_udp || echo "[WARN] Still missing — retry: $0 up  or restart Docker Desktop"
    fi
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
