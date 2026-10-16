#!/usr/bin/env bash
set -euo pipefail
SCEN_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="$SCEN_SCRIPT_DIR${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -m scen_runner.morai.dll_patch "$@"
