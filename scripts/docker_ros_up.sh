#!/usr/bin/env bash
# Build, run, and validate the ROS Noetic development container.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
COMPOSE_FILE="$REPO_ROOT/docker/ros-noetic/docker-compose.yaml"
RTX50_COMPOSE_FILE="$REPO_ROOT/docker/ros-noetic/docker-compose.rtx50.yaml"
CONTAINER="asmc-ros-noetic"

export ASMC="${ASMC:-$REPO_ROOT}"
export ASMC_DATA="${ASMC_DATA:-$(dirname "$REPO_ROOT")/data}"
export TWINLITE_SOURCE="${TWINLITE_SOURCE:-$(dirname "$REPO_ROOT")/external/baselines/TwinLiteNetPlus}"
mkdir -p "$ASMC_DATA"

COMPOSE=(docker compose -f "$COMPOSE_FILE")
COMPOSE_RTX50=(docker compose -f "$COMPOSE_FILE" -f "$RTX50_COMPOSE_FILE")

die() {
  echo "[FAIL] $*" >&2
  exit 1
}

require_running_container() {
  docker inspect "$CONTAINER" >/dev/null 2>&1 \
    || die "$CONTAINER does not exist. Run: $0 up"
  [ "$(docker inspect "$CONTAINER" --format '{{.State.Running}}')" = "true" ] \
    || die "$CONTAINER exists but is stopped. Run: $0 up"
}

assert_host_mode() {
  require_running_container
  local mode
  mode="$(docker inspect "$CONTAINER" --format '{{.HostConfig.NetworkMode}}')"
  [ "$mode" = "host" ] || die "expected host network mode, got: $mode"
  echo "[OK] container network mode: host"
}

render_config() {
  local rendered
  rendered="$("${COMPOSE[@]}" config)"
  grep -q '^    network_mode: host$' <<<"$rendered" \
    || die "compose does not contain network_mode: host"
  if grep -q '^    ports:$' <<<"$rendered"; then
    die "host-mode compose must not publish ports"
  fi
  grep -q '^      TWINLITE_ROOT: /opt/baselines/TwinLiteNetPlus$' <<<"$rendered" \
    || die "compose does not set the TwinLite runtime root"
  grep -q '^        target: /opt/baselines/TwinLiteNetPlus$' <<<"$rendered" \
    || die "compose does not mount the TwinLite runtime source"
  echo "[OK] compose uses host mode, no published ports, and the TwinLite runtime mount"
  if [ "${1:-}" = "show" ]; then
    printf '%s\n' "$rendered"
  fi
}

require_twinlite_source() {
  [ -d "$TWINLITE_SOURCE" ] \
    || die "TwinLite source does not exist: $TWINLITE_SOURCE"
  [ -f "$TWINLITE_SOURCE/model/model.py" ] \
    || die "TwinLite source is incomplete (missing model/model.py): $TWINLITE_SOURCE"
}

render_rtx50_config() {
  local rendered
  rendered="$("${COMPOSE_RTX50[@]}" config)"
  grep -q '^    image: asmc/ros-noetic:rtx50$' <<<"$rendered" \
    || die "RTX 50 compose overlay did not select asmc/ros-noetic:rtx50"
  grep -q '^      dockerfile: Dockerfile.rtx50$' <<<"$rendered" \
    || die "RTX 50 compose overlay did not select Dockerfile.rtx50"
  grep -q '^    network_mode: host$' <<<"$rendered" \
    || die "RTX 50 compose does not preserve host network mode"
  if grep -q '^    ports:$' <<<"$rendered"; then
    die "RTX 50 host-mode compose must not publish ports"
  fi
  echo "[OK] RTX 50 overlay preserves host mode and selects the derived image"
}

inspect_mode() {
  assert_host_mode
  docker inspect "$CONTAINER" --format \
    'mode={{.HostConfig.NetworkMode}} networks={{json .NetworkSettings.Networks}} published={{json .NetworkSettings.Ports}}'
}

check_udp_loopback() {
  assert_host_mode
  command -v powershell.exe >/dev/null 2>&1 \
    || die "powershell.exe is required for the Windows/container UDP check"

  local port="${1:-39199}"
  [[ "$port" =~ ^[0-9]+$ ]] && ((port >= 1024 && port < 65535)) \
    || die "check port must be an integer in 1024..65534"

  local token="asmc-host-${port}-$$-$(date +%s)"
  local result_file="/tmp/asmc-host-udp-check-${port}.json"
  docker exec "$CONTAINER" rm -f "$result_file"
  docker exec -d "$CONTAINER" python3 /root/ws/scripts/probe_host_udp.py \
    listen --port "$port" --token "$token" --timeout 8 \
    --result-file "$result_file"
  sleep 1

  powershell.exe -NoProfile -Command \
    "\$c=New-Object System.Net.Sockets.UdpClient; \$b=[Text.Encoding]::UTF8.GetBytes('$token'); 1..3 | ForEach-Object { [void]\$c.Send(\$b,\$b.Length,'127.0.0.1',$port); Start-Sleep -Milliseconds 50 }; \$c.Close()" \
    >/dev/null

  local i
  for i in $(seq 1 50); do
    if docker exec "$CONTAINER" test -s "$result_file"; then
      docker exec "$CONTAINER" cat "$result_file"
      break
    fi
    sleep 0.1
  done
  docker exec "$CONTAINER" test -s "$result_file" \
    || die "no Windows -> container UDP result; enable Docker Desktop host networking"
  docker exec "$CONTAINER" grep -q '"received": true' "$result_file" \
    || die "Windows -> container UDP payload mismatch or timeout"
  docker exec "$CONTAINER" rm -f "$result_file"
  echo "[OK] Windows -> container UDP on 127.0.0.1:$port"

  local reverse_port=$((port + 1))
  local reverse_token="${token}-reverse"
  local reverse_output
  reverse_output="$(mktemp /tmp/asmc-host-reverse.XXXXXX)"
  powershell.exe -NoProfile -Command \
    "\$c=New-Object System.Net.Sockets.UdpClient($reverse_port); \$c.Client.ReceiveTimeout=8000; \$ep=New-Object System.Net.IPEndPoint([System.Net.IPAddress]::Any,0); try { \$b=\$c.Receive([ref]\$ep); [Console]::Out.Write([Text.Encoding]::UTF8.GetString(\$b)) } catch { exit 3 } finally { \$c.Close() }" \
    >"$reverse_output" 2>&1 &
  local receiver_pid=$!
  sleep 1
  docker exec "$CONTAINER" python3 /root/ws/scripts/probe_host_udp.py \
    send --host 127.0.0.1 --port "$reverse_port" --token "$reverse_token"
  if ! wait "$receiver_pid"; then
    cat "$reverse_output" >&2
    rm -f "$reverse_output"
    die "container -> Windows UDP check timed out"
  fi
  if ! grep -Fq "$reverse_token" "$reverse_output"; then
    cat "$reverse_output" >&2
    rm -f "$reverse_output"
    die "container -> Windows UDP payload mismatch"
  fi
  rm -f "$reverse_output"
  echo "[OK] container -> Windows UDP on 127.0.0.1:$reverse_port"
}

verify_topics() {
  assert_host_mode
  local duration="${1:-8}"
  local profile="${2:-runtime}"
  local simulator_release="${3:-molitcomp03}"
  local require_competition_status="${4:-false}"
  [[ "$duration" =~ ^[0-9]+([.][0-9]+)?$ ]] \
    || die "duration must be a positive number"
  [[ "$simulator_release" == "molitcomp03" || "$simulator_release" == "26r1" ]] \
    || die "simulator release must be molitcomp03 or 26r1"
  [[ "$require_competition_status" == "true" || "$require_competition_status" == "false" ]] \
    || die "require_competition_status must be true or false"
  if [ "$require_competition_status" = "true" ] && [ "$simulator_release" != "molitcomp03" ]; then
    die "Competition Vehicle Status field gate is only defined for molitcomp03"
  fi
  local competition_arg=""
  if [ "$require_competition_status" = "true" ]; then
    competition_arg="--require-competition-status"
  fi
  docker exec "$CONTAINER" bash -lc \
    "cd /root/ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash && python3 scripts/verify_morai_topics.py --profile $profile --duration $duration --simulator-release $simulator_release $competition_arg"
}

print_next() {
  cat <<'EOF'
[NEXT] Start the UDP bridge in a separate foreground terminal:
  docker exec -it asmc-ros-noetic bash
  source devel/setup.bash
  ./scripts/udp_bridge.sh

Keep MORAI Host/Destination IP at 127.0.0.1 and keep every port unchanged.
Field candidate: ./scripts/udp_bridge.sh --transport lan2lan
EOF
}

cmd="${1:-up}"
echo "[DATA] $ASMC_DATA -> /data (rw)"
echo "[TWINLITE] $TWINLITE_SOURCE -> /opt/baselines/TwinLiteNetPlus (ro)"
case "$cmd" in
  config)
    render_config "${2:-}"
    ;;
  build)
    render_config
    "${COMPOSE[@]}" build
    ;;
  up)
    render_config
    require_twinlite_source
    "${COMPOSE[@]}" up -d --force-recreate
    inspect_mode
    print_next
    ;;
  status)
    inspect_mode
    ;;
  check)
    check_udp_loopback "${2:-39199}"
    ;;
  verify)
    verify_topics "${2:-8}" competition "${3:-molitcomp03}" false
    ;;
  verify-field)
    verify_topics "${2:-8}" competition "${3:-molitcomp03}" true
    ;;
  verify-gt)
    verify_topics "${2:-8}" development "${3:-molitcomp03}"
    ;;
  down)
    "${COMPOSE[@]}" down
    ;;
  rebuild)
    render_config
    require_twinlite_source
    "${COMPOSE[@]}" build --no-cache
    "${COMPOSE[@]}" up -d --force-recreate
    inspect_mode
    print_next
    ;;
  build-rtx50)
    render_config
    render_rtx50_config
    # 파생 이미지가 오래된 asmc/ros-noetic:dev를 상속하지 않도록 기본 이미지를
    # 항상 먼저 갱신한다. Docker layer cache가 있어 변경 없는 cu118은 재다운로드하지 않는다.
    "${COMPOSE[@]}" build
    "${COMPOSE_RTX50[@]}" build
    ;;
  up-rtx50)
    render_config
    render_rtx50_config
    require_twinlite_source
    "${COMPOSE_RTX50[@]}" up -d --force-recreate
    inspect_mode
    print_next
    ;;
  rebuild-rtx50)
    render_config
    render_rtx50_config
    require_twinlite_source
    # 공용 RTX 40 이미지를 먼저 만든 뒤 RTX 50 파생 이미지를 no-cache로 만든다.
    "${COMPOSE[@]}" build
    "${COMPOSE_RTX50[@]}" build --no-cache
    "${COMPOSE_RTX50[@]}" up -d --force-recreate
    inspect_mode
    print_next
    ;;
  *)
    echo "Usage: $0 {config [show]|build|up|rebuild|build-rtx50|up-rtx50|rebuild-rtx50|status|check [port]|verify [seconds] [molitcomp03|26r1]|verify-field [seconds] [molitcomp03]|verify-gt [seconds] [molitcomp03|26r1]|down}"
    exit 2
    ;;
esac
