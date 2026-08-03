#!/usr/bin/env bash
# DEPRECATED name — rosbridge diagnostic (NOT UDP).
echo "[deprecated] scripts/diagnose_morai_bridge.sh → scripts/diagnose_morai_rosbridge.sh" >&2
exec "$(cd "$(dirname "$(readlink -f "$0")")" && pwd)/diagnose_morai_rosbridge.sh" "$@"
