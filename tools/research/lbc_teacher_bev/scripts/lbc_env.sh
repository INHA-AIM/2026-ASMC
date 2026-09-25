#!/usr/bin/env bash
# Shared environment for the LBC teacher-BEV research tools.
_LBC_ENV_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../../../../scripts/asmc_env.sh
source "$_LBC_ENV_DIR/../../../../scripts/asmc_env.sh"
export WS_ROOT="$ASMC_WS_ROOT"
export LBC_TOOL_ROOT="$WS_ROOT/tools/research/lbc_teacher_bev"
export LBC_SCRIPTS="$LBC_TOOL_ROOT/scripts"
export LBC_DATA_ROOT="${ASMC_DATA:-$(dirname "$WS_ROOT")/data}/lbc_teacher_bev"
