#!/usr/bin/env python3
"""Verify and summarize frozen research evidence; never run inference or write data.

This reader replaces no F1 probe. Historical timing/geometry remains historical;
the original executors are pinned in the import manifest, not imported at runtime.
Only the selected immutable evidence is hashed, not current documentation/code.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MANIFEST = ROOT / "config/baselines/lidar_research_import_v1.json"
EVIDENCE = Path("config/baselines/evidence")


def evidence_path(root, value):
    path = Path(value)
    if path.is_absolute() or ".." in path.parts or EVIDENCE not in path.parents:
        raise ValueError("Evidence path outside repository evidence tree: " + value)
    resolved = (root / path).resolve()
    if (root / EVIDENCE).resolve() not in resolved.parents:
        raise ValueError("Evidence symlink escapes evidence tree: " + value)
    return resolved


def verify(root, manifest, verify_git=False):
    """Check all files before returning summaries. Missing data is never success."""
    if manifest.get("schema_version") != 1 or not manifest.get("groups"):
        raise ValueError("Expected nonempty research import schema v1")
    summaries, seen, group_ids = {}, set(), set()
    for group in manifest["groups"]:
        group_id = group["id"]
        if group_id in group_ids:
            raise ValueError("Duplicate evidence group: " + group_id)
        group_ids.add(group_id)
        entries = group["files"]
        paths = {entry["path"] for entry in entries}
        if not entries or not {group["summary"], group["gallery"]} <= paths:
            raise ValueError("Summary/gallery not covered by manifest: " + group_id)
        for entry in entries:
            name = entry["path"]
            if name in seen:
                raise ValueError("Duplicate evidence file: " + name)
            seen.add(name)
            data = evidence_path(root, name).read_bytes()
            digest = hashlib.sha256(data).hexdigest()
            if len(data) != entry["bytes"] or digest != entry["sha256"]:
                raise ValueError("Historical evidence changed: " + name)
            if verify_git:
                source = group["source_commit"] + ":" + entry["source_path"]
                # No shell interpolation; a missing historical revision is a failure.
                blob = subprocess.check_output(
                    ["git", "rev-parse", source], cwd=root, text=True).strip()
                original = subprocess.check_output(["git", "show", source], cwd=root)
                if blob != entry["source_blob"] or original != data:
                    raise ValueError("Original Git blob differs: " + name)
        summaries[group_id] = json.loads(evidence_path(root, group["summary"]).read_text())
    return summaries


def build_report(manifest, summaries):
    classical = summaries["lidar_fusion_shadow_v1"]
    learned = summaries["lidar_learned_candidates_v1"]
    e8 = classical["models"]["e8"]
    reuse = learned["segmentation_reuse"]
    return {
        "status": "verified_historical_evidence_only",
        "runtime_acceptance": False,
        "source_commits": [g["source_commit"] for g in manifest["groups"]],
        "verified_files": sum(len(g["files"]) for g in manifest["groups"]),
        "verified_bytes": sum(e["bytes"] for g in manifest["groups"] for e in g["files"]),
        "experiments": [classical["experiment"], learned["experiment"]],
        "gpu": classical["gpu"],
        "independent_accuracy_gt": False,
        "e8_support": {
            "scope": e8["scope"],
            "geometry_unknown_boxes": e8["box_supported_but_geometry_unknown"],
            "detections_without_raw_support": e8["detections_without_raw_support"],
            "active_frame_geometry_ms": e8["active_frame_geometry_ms"],
        },
        "learned_models": {
            name: {"scope": model["scope"],
                   "reported_latency_ms": model.get("total_ms", model.get("latency_ms"))}
            for name, model in learned["models"].items()
        },
        "segmentation_label_evidence": {
            "images": reuse["images"],
            "train_positive_annotations": reuse["positive_annotations_by_evidence"]["train"],
            "complete_positive_frames_with_mask_metadata":
                reuse["complete_positive_frames_with_mask_metadata"],
            "sample_inspection_only": reuse["inspection"],
            "limitations": reuse["limitations"],
        },
        "limitations": {"classical": classical["limitations"], "learned": learned["limitations"]},
        "galleries": [g["gallery"] for g in manifest["groups"]],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--verify-git", action="store_true",
                        help="Also compare original blobs (requires preserved research Git history)")
    args = parser.parse_args()
    try:
        manifest = json.loads(args.manifest.read_text())
        summaries = verify(ROOT, manifest, args.verify_git)
        report = build_report(manifest, summaries)
    except (OSError, ValueError, KeyError, TypeError, subprocess.CalledProcessError) as exc:
        print("Research review failed: " + str(exc), file=sys.stderr)
        return 1
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
