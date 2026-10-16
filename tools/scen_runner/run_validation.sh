#!/usr/bin/env bash
set -euo pipefail
SCEN_VALIDATION_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="$SCEN_VALIDATION_DIR${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -m scen_runner.ui.validation_gui "$@"
