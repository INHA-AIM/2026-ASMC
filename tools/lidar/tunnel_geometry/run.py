#!/usr/bin/env python3
"""Run isolated policies on frozen inputs, compare OFF parity, and score known truth."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

import numpy as np
import yaml

from evidence import camera_evidence
from fixtures import canonical
from metrics import score, promotion_gate
from git_evidence import git_output

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "camera_sync"))
from run_evaluation import digest_file
from run_geometry import geometry_only

MODES = ("baseline", "policy_only", "geometry", "candidate_off")
HISTORICAL = ("labels", "kept_indices", "objects", "leveling", "leveled_xyzi_fnv1a64",
              "clusters", "final_objects", "protected_indices", "protected_points",
              "ground_removed", "wall_removed", "ceiling_removed", "kept_points")


def old_shape(actual, expected):
    if isinstance(expected, dict):
        return {k: old_shape(actual[k], v) for k, v in expected.items()}
    if isinstance(expected, list):
        if len(actual) != len(expected):
            return actual
        return [old_shape(a, b) for a, b in zip(actual, expected)]
    return actual


def percentiles(values):
    return dict(zip(("p50", "p95", "p99"), [float(x) for x in np.percentile(values, [50, 95, 99])])) if values else {}


def run(args):
    if args.output.exists():
        raise ValueError("output exists; preserve previous results")
    repo = Path(__file__).resolve().parents[3]
    evaluator_commit = git_output(repo, ["rev-parse", "HEAD"]).strip()
    config = yaml.safe_load(args.config.read_text())
    source_rows = [json.loads(s) for s in args.snapshots.open()]
    if len({r["input_id"] for r in source_rows}) != len(source_rows):
        raise ValueError("duplicate snapshot ID")
    truth_rows = {r["input_id"]: r for r in map(json.loads, args.truth.open())} if args.truth else {}
    reference = iter(map(json.loads, args.reference_geometry.open())) if args.reference_geometry else None
    args.output.mkdir(parents=True)
    inputs = [("snapshots", args.snapshots), ("setting", args.setting), ("params", args.params),
              ("config", args.config), ("executable", args.executable)]
    if args.truth:
        inputs.append(("truth", args.truth))
    if args.reference_geometry:
        inputs.append(("reference_geometry", args.reference_geometry))
    library = args.executable.parent / "libl1b4_policy.so"
    if library.exists():
        inputs.append(("policy_library", library))
    input_evidence = {name: {"path": str(p.resolve()), "bytes": p.stat().st_size,
                             "sha256": digest_file(p)} for name, p in inputs}
    tools_evidence = {p.name: digest_file(p) for p in Path(__file__).parent.iterdir() if p.is_file()}
    cloud_evidence = {}
    for source in source_rows:
        if not source["geometry_valid"]:
            continue
        path = (args.snapshots.parent / source["cloud_path"]).resolve()
        if str(path) not in cloud_evidence:
            cloud_evidence[str(path)] = {"bytes": path.stat().st_size, "sha256": digest_file(path)}
        if source.get("cloud_sha256") and cloud_evidence[str(path)]["sha256"] != source["cloud_sha256"]:
            raise ValueError("raw cloud SHA changed")
    log = args.output / "cpp.log"
    with log.open("w") as stream:
        subprocess.run([str(args.executable), str(args.snapshots), str(args.setting), str(args.params),
                        str(args.config), str(args.output / "cpp.jsonl"), str(args.output / "clouds"),
                        args.decision], check=True, stdout=stream, stderr=subprocess.STDOUT)
    counts = Counter()
    states = Counter()
    scene_counts = defaultdict(lambda: {m: Counter() for m in MODES})
    times = defaultdict(list)
    key_scores = {}
    all_scores = []
    negative_groups = defaultdict(dict)
    diagnostic_cases = []
    stable_digest = hashlib.sha256()
    with (args.output / "cpp.jsonl").open() as results, (args.output / "geometry.jsonl").open("w") as target:
        for source in source_rows:
            line = next(results, None)
            if line is None:
                raise ValueError("missing evaluator output")
            result = json.loads(line)
            if result["input_id"] != source["input_id"]:
                raise ValueError("scan order/identity changed")
            old = next(reference, None) if reference else None
            if reference and (old is None or old["input_id"] != result["input_id"]):
                raise ValueError("historical scan order differs")
            counts["frames"] += 1
            if "skipped_reason" in result:
                counts["skipped"] += 1
                if old and old.get("skipped_reason") != result["skipped_reason"]:
                    counts["historical_parity_errors"] += 1
            else:
                counts["valid"] += 1
                result["camera_evidence"] = camera_evidence(source[args.decision])
                states[result["camera_evidence"]["state"]] += 1
                if any(x.get("used_future") or x.get("arrival_ns", 0) > source.get("scan_arrival_ns", 2**63 - 1)
                       for x in source.get("pose_evidence", {}).values()):
                    counts["future_pose_uses"] += 1
                if result["scan_stamp_ns"] != source["scan_stamp_ns"]:
                    counts["stamp_errors"] += 1
                counts["intensity_errors"] += result["intensity_preservation_errors"]
                counts["point_count_errors"] += not result["point_count_preserved"]
                counts["lineage_errors"] += result["lineage_errors"]
                if geometry_only(result["baseline"]) != geometry_only(result["candidate_off"]):
                    counts["candidate_off_errors"] += 1
                if old:
                    keys = {k: old[args.decision][k] for k in HISTORICAL}
                    try:
                        compared = old_shape(result["baseline"], keys)
                        counts["historical_parity_errors"] += compared != keys
                    except (KeyError, TypeError):
                        counts["historical_parity_errors"] += 1
                for mode in MODES:
                    if result["measurement"]:
                        for key in ("wall_removed", "ground_removed", "ceiling_removed", "kept_points", "clusters", "final_objects"):
                            scene_counts[result["scene"]][mode][key] += result[mode][key]
                        for key, value in result[mode]["timing_ms"].items():
                            times[mode + "/" + key].append(value)
                if result["measurement"]:
                    times["shared/frozen_copy"].append(result["frozen_copy_ms"])
                    times["shared/lineage"].append(result["lineage_ms"])
                gt = truth_rows.get(result["input_id"])
                if gt and result["measurement"]:
                    points = np.fromfile(args.output / "clouds" / result["voxel_path"], dtype="<f4").reshape(-1, 4)
                    scores = {m: score(result[m], gt["labels"], result["pre_wall_labels"], points)
                              for m in MODES}
                    entry = {"input_id": result["input_id"], "kind": gt.get("kind"),
                             "family": source.get("fixture_family"), "camera_state": result["camera_evidence"]["state"],
                             "shape": gt.get("shape"), "side": gt.get("side"), "scores": scores}
                    all_scores.append(entry)
                    if scores["baseline"]["accepted_target"] and not scores["geometry"]["accepted_target"]:
                        counts["baseline_success_losses"] += 1
                    if gt.get("key_gate"):
                        key_scores[gt["kind"]] = {"geometry": scores["geometry"],
                                                 "missing_reference": scores["policy_only"]}
                    state = result["camera_evidence"]["state"]
                    if source.get("fixture_family") in ("analytic_first_return", "dense_negative_variants") and state in \
                            ("valid_empty", "unsupported_only", "missing", "protection_off", "invalid_stamp"):
                        group = (gt.get("kind"), gt.get("side"), gt.get("shape"), source.get("fixture_family"))
                        negative_groups[group][state] = result["geometry"]["labels"]
                if result["measurement"] and len(diagnostic_cases) < 3 and not gt:
                    diagnostic_cases.append(result["input_id"])
            stable = canonical(geometry_only(result)) + "\n"
            stable_digest.update(stable.encode())
            target.write(canonical(result) + "\n")
        if next(results, None) is not None or (reference and next(reference, None) is not None):
            raise ValueError("extra evaluator or historical frames")
    for group in negative_groups.values():
        counts["negative_camera_mask_differences"] += max(0, len({tuple(x) for x in group.values()}) - 1)
    summary = {"schema": "asmc-l1b4-results-v1", "counts": dict(counts), "camera_states": dict(states),
               "timing_ms": {k: percentiles(v) for k, v in times.items()},
               "by_scene": {s: {m: dict(v) for m, v in modes.items()} for s, modes in scene_counts.items()},
               "key_scores": key_scores, "fixture_scores": all_scores,
               "negative_camera_groups": len(negative_groups), "diagnostic_cases": diagnostic_cases,
               "geometry_sha256": stable_digest.hexdigest()}
    gate = promotion_gate(key_scores, counts["baseline_success_losses"]) if args.truth else \
           {"passed": False, "reasons": ["no_accuracy_ground_truth"]}
    for key in ("candidate_off_errors", "historical_parity_errors", "future_pose_uses", "stamp_errors",
                "intensity_errors", "point_count_errors", "negative_camera_mask_differences", "lineage_errors"):
        if counts[key]:
            gate["passed"] = False
            gate["reasons"].append(key)
    summary["promotion_gate"] = gate
    (args.output / "summary.json").write_text(json.dumps(summary, sort_keys=True, indent=2) + "\n")
    provenance = args.executable.parent / "reference_provenance.json"
    shutil.copyfile(provenance, args.output / provenance.name)
    for name, path in inputs:
        if digest_file(path) != input_evidence[name]["sha256"]:
            raise ValueError("run input changed during evaluation: " + name)
    for name, expected in tools_evidence.items():
        if digest_file(Path(__file__).parent / name) != expected:
            raise ValueError("evaluator source changed during evaluation: " + name)
    manifest = {"schema": "asmc-l1b4-run-v1", "runtime_connected": False, "decision": args.decision,
                "reference_revision": config["reference_revision"], "geometry_sha256": summary["geometry_sha256"],
                "evaluator_commit": evaluator_commit,
                "inputs": input_evidence, "source_clouds": cloud_evidence,
                "tools_sha256": tools_evidence,
                "summary_sha256": digest_file(args.output / "summary.json"),
                "geometry_file_sha256": digest_file(args.output / "geometry.jsonl"),
                "derived_clouds_sha256": {p.name: digest_file(p) for p in (args.output / "clouds").iterdir() if p.is_file()},
                "provenance_sha256": digest_file(args.output / provenance.name),
                "point_origin": "Frozen leveled sensor-origin coordinates, not base_link or map",
                "limits": ["Filter wall labels and object counts are not accuracy ground truth",
                           "Common fit isolates deletion policy, not end-to-end independent wall estimation",
                           "Shared preprocessing plus candidate stages is an offline timing estimate",
                           "No tracker, ROS publish or concurrent GPU workload in timing"]}
    (args.output / "manifest.json").write_text(json.dumps(manifest, sort_keys=True, indent=2) + "\n")
    print(canonical({k: summary[k] for k in ("counts", "geometry_sha256", "promotion_gate")}))
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("snapshots", "setting", "params", "config", "executable", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--truth", type=Path)
    parser.add_argument("--reference-geometry", type=Path)
    parser.add_argument("--decision", choices=("baseline", "candidate"), default="baseline")
    run(parser.parse_args())
