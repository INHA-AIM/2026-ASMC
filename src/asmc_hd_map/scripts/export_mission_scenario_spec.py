#!/usr/bin/env python3
"""Export the current mission scenario spec as JSON for non-Python consumers.

The spec is rebuilt from the committed metadata, compact product, MGeo link_set
and the editable scenario source on every run; the export is a snapshot and is
not tracked in git.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
from typing import Optional, Sequence


def main(argv: Optional[Sequence[str]] = None) -> int:
    root = Path(__file__).resolve().parents[3]
    package_src = root / "src/asmc_hd_map/src"
    if str(package_src) not in sys.path:
        sys.path.insert(0, str(package_src))
    from asmc_hd_map.mission_scenario import build_mission_scenario_spec

    data_root = Path(os.environ.get("ASMC_DATA") or root.parent / "data")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=root)
    parser.add_argument("--source", type=Path)
    parser.add_argument(
        "--out", type=Path, default=data_root / "hd_map_exports/kcity2025_mission_scenario_spec.json"
    )
    args = parser.parse_args(argv)
    document = build_mission_scenario_spec(args.repo_root, args.source)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(document, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    print("wrote %s: zones=%d warnings=%d" % (args.out, len(document["zones"]), len(document["warnings"])))
    for warning in document["warnings"]:
        print("WARNING " + warning, file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
