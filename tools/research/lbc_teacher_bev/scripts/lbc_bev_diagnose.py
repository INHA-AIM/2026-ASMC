#!/usr/bin/env python3
"""Run LBC BEV diagnostics and save a shareable report."""
from __future__ import annotations

import argparse
import sys
from datetime import datetime
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from lbc_bev.diagnostics import build_report, save_report
from lbc_bev.ws_root import asmc_ws_root, lbc_data_root


def main() -> int:
    parser = argparse.ArgumentParser(description="LBC BEV + MORAI diagnostic report")
    parser.add_argument(
        "--aim-ws", default=str(asmc_ws_root()), help="ASMC repository root"
    )
    parser.add_argument(
        "--output",
        default="",
        help="report file path (default: $ASMC_DATA/lbc_teacher_bev/diag/lbc_report_TIMESTAMP.txt)",
    )
    parser.add_argument("--ego-wait", type=float, default=5.0, help="seconds to wait for /Ego_topic")
    parser.add_argument("--no-print", action="store_true", help="only write file, no stdout")
    args = parser.parse_args()

    ws = Path(args.aim_ws).expanduser()
    if args.output:
        out_path = Path(args.output).expanduser()
    else:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = lbc_data_root() / "diag" / f"lbc_report_{ts}.txt"

    text = build_report(ws_root=ws, ego_timeout=args.ego_wait)
    save_report(out_path, text)

    if not args.no_print:
        print(text)
    print(f"\n>>> Report saved: {out_path}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
