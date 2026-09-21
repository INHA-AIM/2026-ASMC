#!/usr/bin/env python3
"""Generate the P2-B route-compact lane/drivable product."""

import argparse
import copy
import json
from pathlib import Path

import numpy as np

from asmc_hd_map.compact_drivable import (
    build_compact_drivable,
    canonical_sha256,
    load_yaml,
    render_report,
    sha256_file,
    write_product,
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", default="/root/ws")
    parser.add_argument(
        "--config",
        default="src/asmc_hd_map/config/kcity2025_compact_drivable_source.yaml",
    )
    parser.add_argument("--check", action="store_true", help="verify committed outputs without writing")
    args = parser.parse_args()
    root = Path(args.repo_root).resolve()
    source = (root / args.config).resolve()
    config = load_yaml(source)
    if args.check:
        product_path = (root / str(config["outputs"]["product"])).resolve()
        grid_path = (root / str(config["outputs"]["grid"])).resolve()
        committed = json.loads(product_path.read_text(encoding="utf-8"))
        generated, arrays = build_compact_drivable(root, source)
        committed_core = copy.deepcopy(committed)
        generated_core = copy.deepcopy(generated)
        committed_core.pop("product_sha256", None)
        generated_core.pop("product_sha256", None)
        committed_core["grid"].pop("sha256", None)
        if committed_core != generated_core:
            raise RuntimeError("committed compact product metadata/geometry drift")
        with np.load(str(grid_path), allow_pickle=False) as archive:
            if not np.array_equal(arrays["occupancy"], archive["occupancy"]):
                raise RuntimeError("committed compact occupancy grid drift")
        if committed["grid"]["sha256"] != sha256_file(grid_path):
            raise RuntimeError("committed compact occupancy SHA mismatch")
        expected = canonical_sha256(
            {key: value for key, value in committed.items() if key != "product_sha256"}
        )
        if committed["product_sha256"] != expected:
            raise RuntimeError("committed compact product self-hash mismatch")
        print("compact drivable outputs are deterministic and current")
        return 0
    product = write_product(root, source)
    report = (root / str(config["outputs"]["report"])).resolve()
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text(render_report(product), encoding="utf-8")
    print("wrote %s" % config["outputs"]["product"])
    print("wrote %s" % config["outputs"]["grid"])
    print("wrote %s" % config["outputs"]["report"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
