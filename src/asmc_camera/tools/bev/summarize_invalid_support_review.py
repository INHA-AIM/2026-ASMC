#!/usr/bin/env python3
"""Build compact contact sheets from sparse invalid-support review overlays."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from asmc_camera.offline.bev.invalid_support_review import (
    HEALTHY_REVIEW_CATEGORY,
    INVALID_REVIEW_CATEGORIES,
)


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_entries(root):
    entries = []
    sources = []
    for manifest_path in sorted(root.glob("*/invalid_support_review.json")):
        document = json.loads(manifest_path.read_text(encoding="utf-8"))
        if document.get("invalid_geometry_planning_authority") is not False:
            raise ValueError(
                "review manifest must explicitly deny planning authority: {}".format(
                    manifest_path
                )
            )
        run_dir = manifest_path.parent
        sources.append(
            {
                "run_id": document.get("run_id"),
                "path": str(manifest_path.resolve()),
                "sha256": _sha256(manifest_path),
            }
        )
        for value in document.get("overlays", []):
            path = run_dir / value["overlay"]
            if not path.is_file():
                raise ValueError("review overlay is missing: {}".format(path))
            item = dict(value)
            item["source_overlay"] = str(path.resolve())
            entries.append(item)
    if not sources:
        raise ValueError("no invalid-support review manifests under {}".format(root))
    return entries, sources


def _tile(entry, width=320, height=500):
    image = cv2.imread(entry["source_overlay"], cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("cannot decode {}".format(entry["source_overlay"]))
    scale = min(float(width) / image.shape[1], float(height - 44) / image.shape[0])
    resized = cv2.resize(
        image,
        (
            max(1, int(round(image.shape[1] * scale))),
            max(1, int(round(image.shape[0] * scale))),
        ),
        interpolation=cv2.INTER_AREA,
    )
    tile = np.full((height, width, 3), 22, dtype=np.uint8)
    x0 = (width - resized.shape[1]) // 2
    tile[44 : 44 + resized.shape[0], x0 : x0 + resized.shape[1]] = resized
    run_id = str(entry["run_id"])
    label = "{}  f{}".format(run_id.split("_kcity2025_")[-1], entry["frame_index"])
    cv2.putText(
        tile, label[:51], (6, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.37,
        (240, 240, 240), 1, cv2.LINE_AA,
    )
    active = ",".join(entry.get("active_categories", [])) or "none"
    cv2.putText(
        tile, active[:58], (6, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.32,
        (180, 180, 180), 1, cv2.LINE_AA,
    )
    return tile


def _contact_sheet(entries, path, columns=4):
    if not entries:
        return None
    tiles = [_tile(value) for value in entries]
    blank = np.full_like(tiles[0], 22)
    rows = []
    for index in range(0, len(tiles), columns):
        row = tiles[index : index + columns]
        row.extend([blank] * (columns - len(row)))
        rows.append(np.concatenate(row, axis=1))
    sheet = np.concatenate(rows, axis=0)
    if not cv2.imwrite(str(path), sheet, [cv2.IMWRITE_JPEG_QUALITY, 92]):
        raise OSError("failed to write contact sheet: {}".format(path))
    return path.name


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-root", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--max-per-category", type=int, default=8)
    parser.add_argument("--columns", type=int, default=4)
    arguments = parser.parse_args()
    if arguments.max_per_category <= 0 or arguments.columns <= 0:
        raise SystemExit("max-per-category and columns must be positive")
    arguments.output_dir.mkdir(parents=True, exist_ok=False)
    entries, sources = _load_entries(arguments.input_root.resolve())
    category_order = INVALID_REVIEW_CATEGORIES + (HEALTHY_REVIEW_CATEGORY,)
    sheets = {}
    selected = []
    for category in category_order:
        candidates = sorted(
            (
                value
                for value in entries
                if value.get("selected_category") == category
            ),
            key=lambda value: (value["run_id"], int(value["frame_index"])),
        )[: arguments.max_per_category]
        if not candidates:
            continue
        filename = "contact_sheet_{}.jpg".format(category)
        sheets[category] = _contact_sheet(
            candidates, arguments.output_dir / filename, arguments.columns
        )
        selected.extend(candidates)
    overview = []
    for category in category_order:
        overview.extend(
            [
                value
                for value in selected
                if value.get("selected_category") == category
            ][:2]
        )
    overview_name = _contact_sheet(
        overview, arguments.output_dir / "contact_sheet_overview.jpg", arguments.columns
    )
    output = {
        "schema_version": "asmc-invalid-support-overlay-prereview-1.0.0",
        "created_at_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "status": "visual_prereview_only_no_support_policy_change",
        "measurement_only": True,
        "invalid_geometry_planning_authority": False,
        "interpretation": (
            "red curves are retained mathematical family geometry where local "
            "camera support is invalid; they are not validated low-confidence lanes"
        ),
        "input_root": str(arguments.input_root.resolve()),
        "sources": sources,
        "selected_overlays": selected,
        "contact_sheets": sheets,
        "overview": overview_name,
        "next_gate": (
            "compare invalid geometry with role-matched map pseudo-GT using "
            "source-time Ego pose interpolation, then review threshold and worst cases"
        ),
    }
    (arguments.output_dir / "review_manifest.json").write_text(
        json.dumps(output, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


if __name__ == "__main__":
    main()
