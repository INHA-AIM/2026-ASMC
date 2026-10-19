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

case "$ACTION" in start|status|logs) ;; *) usage; exit 2 ;; esac
if [[ ! "$JOB_NAME" =~ ^[a-zA-Z0-9][a-zA-Z0-9._-]*$ ]]; then usage; exit 2; fi
if [[ "$ACTION" == start && ( "$#" -lt 3 || ( "${3:-}" == -- && "$#" -lt 4 ) ) ]]; then
  usage; exit 2
fi
if [[ "$ACTION" == logs && ! "${3:-40}" =~ ^[1-9][0-9]*$ ]]; then
  echo "[ERROR] line-count must be a positive integer" >&2; exit 2
fi
if ! docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -qx true; then
  echo "[ERROR] container is not running: $CONTAINER" >&2; exit 1
fi

if [[ -n "${ASMC_TRAIN_JOB_ROOT:-}" ]]; then
  JOB_DIR="$ASMC_TRAIN_JOB_ROOT/$JOB_NAME"
else
  # New managed jobs use the writable training mount. Historical jobs remain
  # readable via their original repository aliases; never silently mix two runs.
  JOB_DIR="$(docker exec "$CONTAINER" python3 -c '
import sys
from pathlib import Path
sys.path.insert(0,"/root/ws/src/data_collection/src")
from data_collection.storage_paths import default_artifact_root
action,name=sys.argv[1:]
current=default_artifact_root(Path("/root/ws"))/"perception_eval/jobs"/name
legacy=Path("/root/ws/artifacts/perception_eval/jobs")/name
existing={str(p.resolve()):p for p in (current,legacy) if p.exists()}
if action=="start" and existing:raise SystemExit("Job exists; choose a new job name")
if len(existing)>1:raise SystemExit("Ambiguous job; set ASMC_TRAIN_JOB_ROOT explicitly")
print(next(iter(existing.values())) if existing else current)
' "$ACTION" "$JOB_NAME")"
fi
[[ "$JOB_DIR" = /* ]] || { echo "[ERROR] job directory must be absolute" >&2; exit 2; }

case "$ACTION" in
  start)
    shift 2
    if [[ "${1:-}" == -- ]]; then shift; fi
    docker exec "$CONTAINER" python3 -c 'import sys; sys.path.insert(0,"/root/ws/src/data_collection/src"); from data_collection.storage_paths import validate_bulk_output; validate_bulk_output(sys.argv[1])' "$JOB_DIR"
    quoted_command=""
    for argument in "$@"; do
      printf -v quoted_argument '%q' "$argument"
      quoted_command+="$quoted_argument "
    done
    # Paths and commands are positional arguments, including spaces/apostrophes.
    docker exec "$CONTAINER" bash -lc '
      set -eu
      mkdir -p -- "$(dirname "$1")"
      mkdir -- "$1"
      printf "%s\n" "$2" > "$1/command.txt"
    ' _ "$JOB_DIR" "$quoted_command"
    docker exec -d "$CONTAINER" bash -lc '
      cd /root/ws
      echo $$ > "$1/pid"
      date -Iseconds > "$1/started_at"
      set +e
      eval "$2" > "$1/console.log" 2>&1
      code=$?
      printf "%s\n" "$code" > "$1/exit_code"
      date -Iseconds > "$1/finished_at"
      exit "$code"
    ' _ "$JOB_DIR" "$quoted_command"
    echo "[STARTED] $JOB_NAME"
    echo "[LOG] $JOB_DIR/console.log"
    ;;
  status)
    docker exec "$CONTAINER" bash -lc '
      if [[ -f "$1/exit_code" ]]; then
        echo "completed exit_code=$(cat "$1/exit_code")"
      elif [[ -f "$1/pid" ]] && kill -0 "$(cat "$1/pid")" 2>/dev/null; then
        echo "running pid=$(cat "$1/pid")"
      elif [[ -d "$1" ]]; then
        echo interrupted_without_exit_code
      else
        echo job_not_found; exit 1
      fi
    ' _ "$JOB_DIR"
    ;;
  logs)
    docker exec "$CONTAINER" bash -lc 'tail -n "$2" -- "$1/console.log"' _ "$JOB_DIR" "${3:-40}"
    ;;
esac
