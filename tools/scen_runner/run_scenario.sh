#!/usr/bin/env bash
set -euo pipefail
SCEN_STACK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$SCEN_STACK_DIR/run_validation.sh" --with-planning "$@"
