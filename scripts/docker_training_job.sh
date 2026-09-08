#!/usr/bin/env bash
# Run a training command independently from the invoking VS Code/WSL terminal.
set -euo pipefail

CONTAINER="${ASMC_TRAIN_CONTAINER:-asmc-perception-train}"
ACTION="${1:-}"
JOB_NAME="${2:-}"

usage() {
  echo "Usage:"
  echo "  $0 start <job-name> -- <command> [args...]"
  echo "  $0 status <job-name>"
  echo "  $0 logs <job-name> [line-count]"
}

if [[ ! "$JOB_NAME" =~ ^[a-zA-Z0-9][a-zA-Z0-9._-]*$ ]]; then
  usage
  exit 2
fi

JOB_DIR="/root/ws/artifacts/perception_eval/jobs/$JOB_NAME"

case "$ACTION" in
  start)
    shift 2
    if [[ "${1:-}" == "--" ]]; then
      shift
    fi
    if [[ "$#" -eq 0 ]]; then
      usage
      exit 2
    fi
    if ! docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -qx true; then
      echo "[ERROR] container is not running: $CONTAINER"
      exit 1
    fi
    docker exec "$CONTAINER" bash -lc "test ! -e '$JOB_DIR'"
    quoted_command=""
    for argument in "$@"; do
      printf -v quoted_argument '%q' "$argument"
      quoted_command+="$quoted_argument "
    done
    docker exec "$CONTAINER" bash -lc "mkdir -p '$JOB_DIR' && printf '%s\n' '$quoted_command' > '$JOB_DIR/command.txt'"
    docker exec -d "$CONTAINER" bash -lc "
      cd /root/ws
      echo \$\$ > '$JOB_DIR/pid'
      date -Iseconds > '$JOB_DIR/started_at'
      set +e
      $quoted_command > '$JOB_DIR/console.log' 2>&1
      code=\$?
      printf '%s\n' \"\$code\" > '$JOB_DIR/exit_code'
      date -Iseconds > '$JOB_DIR/finished_at'
      exit \"\$code\"
    "
    echo "[STARTED] $JOB_NAME"
    echo "[LOG] artifacts/perception_eval/jobs/$JOB_NAME/console.log"
    ;;
  status)
    docker exec "$CONTAINER" bash -lc "
      if [[ -f '$JOB_DIR/exit_code' ]]; then
        echo \"completed exit_code=\$(cat '$JOB_DIR/exit_code')\"
      elif [[ -f '$JOB_DIR/pid' ]] && kill -0 \"\$(cat '$JOB_DIR/pid')\" 2>/dev/null; then
        echo \"running pid=\$(cat '$JOB_DIR/pid')\"
      elif [[ -d '$JOB_DIR' ]]; then
        echo interrupted_without_exit_code
      else
        echo job_not_found
        exit 1
      fi
    "
    ;;
  logs)
    LINE_COUNT="${3:-40}"
    if [[ ! "$LINE_COUNT" =~ ^[1-9][0-9]*$ ]]; then
      echo "[ERROR] line-count must be a positive integer"
      exit 2
    fi
    docker exec "$CONTAINER" bash -lc "tail -n '$LINE_COUNT' '$JOB_DIR/console.log'"
    ;;
  *)
    usage
    exit 2
    ;;
esac
