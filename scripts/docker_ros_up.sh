#!/usr/bin/env bash
# ROS Noetic 개발 컨테이너 제어
set -euo pipefail
COMPOSE_DIR="$(dirname "$(readlink -f "$0")")/../docker/ros-noetic"
export ASMC="${ASMC:-$(dirname "$(readlink -f "$0")")/..}"

# Docker Desktop+Windows: Dest UDP가 compose에 있어도 Windows 리스너가 빠지는 경우가 있음 (특히 9299).
#
# 주의 — 이 검사는 "지금 리스너가 있다"만 본다. 컨테이너 쪽 9299에 리스너가 없는 동안
# MORAI가 스트림을 쏘면 Docker Desktop이 ~30s 후 포워딩을 철거하므로, 깨질 예정인 상태도
# 여기서는 통과한다. lidar 수신 여부는 반드시 브릿지 로그의 `rx ... lidar=` 로 확인할 것.
# 상세: docs/network-udp-ros.md §6.1
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
  # "open" ≠ "동작함". 위 _check_win_udp 주석 참고.
  echo "[OK] Windows UDP Dest listening (preliminary): 9291 9299 9301 9303"
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
      if ! _verify_sensor_udp; then
        echo "[WARN] Still missing — Docker Desktop 재시작이 필요하다:"
        echo "       \"/mnt/c/Program Files/Docker/Docker/resources/bin/docker.exe\" desktop restart"
        echo "       (WSL 안의 'docker desktop restart' 는 실패하면서 성공처럼 보인다)"
        echo "       상세: docs/network-udp-ros.md §6.1"
      fi
    fi
    echo "[NEXT] 지체 없이 브릿지를 실행할 것 — 리스너 없이 방치하면 9299 포워딩이 ~30s 후 죽는다."
    echo "       docker exec -it asmc-ros-noetic bash"
    echo "       source devel/setup.bash && ./scripts/udp_bridge.sh"
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
