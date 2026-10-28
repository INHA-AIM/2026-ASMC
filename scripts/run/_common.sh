#!/usr/bin/env bash
# A~E 공용: ROS 컨테이너에 접속해 workspace 환경을 잡고 명령을 실행한다.
# 호스트 터미널에서 실행한다. 컨테이너가 꺼져 있으면 먼저 켠다.
set -euo pipefail

CONTAINER="asmc-ros-noetic"
REPO_ROOT="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../.." && pwd)"

ensure_container() {
  if ! docker inspect "$CONTAINER" >/dev/null 2>&1; then
    echo "[run] $CONTAINER 없음: docker_ros_up.sh up 으로 생성"
    "$REPO_ROOT/scripts/docker_ros_up.sh" up
  elif [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER")" != "true" ]; then
    echo "[run] $CONTAINER 시작"
    docker start "$CONTAINER" >/dev/null
  fi
  # RViz 등 GUI용. 실패해도 진행한다.
  xhost +local:root >/dev/null 2>&1 || true
}

# $1: 표시 이름, $2: 컨테이너 안에서 실행할 명령
run_in_container() {
  ensure_container
  echo "[run] $1: $2"
  printf '\033]0;%s\007' "$1"  # 터미널 탭 제목
  docker exec -it -e DISPLAY="${DISPLAY:-}" "$CONTAINER" bash -lc "
    cd /root/ws &&
    source /opt/ros/noetic/setup.bash &&
    source devel/setup.bash &&
    $2"
}
