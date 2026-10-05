#!/usr/bin/env python3
"""Isolated clustering comparison. Capture admission failure blocks ALL AOS calls."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from tools.lidar.tunnel_geometry.git_evidence import git_output

if __package__:
    from .projection import canonical, project, sha
else:
    from projection import canonical, project, sha


def stable(value):
    if isinstance(value, dict):
        return {k: stable(v) for k, v in value.items()
                if k != "timing_ms" and not k.endswith("_ms")}
    if isinstance(value, list):
        return [stable(v) for v in value]
    return value


def old_shape(actual, expected):
    if isinstance(expected, dict):
        return {k: old_shape(actual[k], v) for k, v in expected.items()}
    if isinstance(expected, list) and len(actual) == len(expected):
        return [old_shape(a, b) for a, b in zip(actual, expected)]
    return actual


def admission(kind, audit, config, source_manifest):
    if kind != "capture":
        return False
    if audit is None or source_manifest is None:
        raise ValueError("Capture requires a bound, complete projection audit")
    m = json.loads((audit / "manifest.json").read_text())
    for name, digest in m["artifacts"].items():
        if sha(audit / name) != digest:
            raise ValueError("projection audit artifact changed: " + name)
    if m["config_sha256"] != sha(config) or m["admission_sha256"] != sha(source_manifest):
        raise ValueError("projection audit input binding changed")
    return json.loads((audit / "summary.json").read_text())["admitted"]


def analytic_admitted(source, points, config):
    # The analytic generator, not a fabricated ring/time field, establishes
    # beam directions. Dense arbitrary fixtures have no projection admission.
    if source.get("fixture_family") not in ("analytic_first_return", "l1b5_analytic_first_return"):
        return False
    result = project(points, config["aos"])
    return bool(len(points) and np.all(result["reasons"] == "eligible"))


def run(args):
    if args.output.exists():
        raise ValueError("output exists; preserve previous experiment")
    config = yaml.safe_load(args.config.read_text())
    repo = Path(__file__).resolve().parents[3]
    tool_commit = git_output(repo, ["rev-parse", "HEAD"]).strip()
    tools_at_start = {p.name: sha(p) for p in Path(__file__).parent.iterdir() if p.is_file()}
    capture_ok = admission(args.kind, args.projection_audit, args.config, args.source_manifest)
    source_rows = [json.loads(s) for s in args.snapshots.open()]
    if len({r["input_id"] for r in source_rows}) != len(source_rows):
        raise ValueError("duplicate source identity")
    args.output.mkdir(parents=True)
    prepared = []
    clouds = {}
    for original in source_rows:
        row = dict(original)
        if not row.get("geometry_valid"):
            row["projection_admitted"] = False
            prepared.append(row)
            continue
        path = (args.snapshots.parent / row["cloud_path"]).resolve()
        if row.get("geometry_valid"):
            if str(path) not in clouds:
                clouds[str(path)] = {"bytes": path.stat().st_size, "sha256": sha(path)}
            if row.get("cloud_sha256") and row["cloud_sha256"] != clouds[str(path)]["sha256"]:
                raise ValueError("source cloud changed")
        row["cloud_path"] = str(path)
        row["projection_admitted"] = capture_ok if args.kind == "capture" else (
            analytic_admitted(row, np.fromfile(path, dtype="<f4").reshape(-1, 4), config)
            if args.kind == "synthetic" and row.get("geometry_valid") else False)
        prepared.append(row)
    snapshot = args.output / "snapshots.jsonl"
    snapshot.write_text("".join(canonical(r) + "\n" for r in prepared))
    with (args.output / "cpp.log").open("w") as log:
        subprocess.run([str(args.executable), str(snapshot), str(args.setting), str(args.params),
                        str(args.config), str(args.output / "geometry.jsonl"), str(args.output / "clouds"),
                        args.decision], stdout=log, stderr=subprocess.STDOUT, check=True)
    old_file = args.reference_geometry.open() if args.reference_geometry else None
    old = iter(map(json.loads, old_file)) if old_file else None
    counts = Counter()
    times = defaultdict(list)
    geometry_digest = hashlib.sha256()
    for source, line in zip(prepared, (args.output / "geometry.jsonl").open()):
        r = json.loads(line)
        prior = next(old, None) if old else None
        counts["calls"] += 1
        if source["input_id"] != r["input_id"]:
            raise ValueError("scan identity or order changed")
        if prior is not None and prior["input_id"] != r["input_id"]:
            raise ValueError("reference input order differs")
        geometry_digest.update((canonical(stable(r)) + "\n").encode())
        if "skipped_reason" in r:
            counts["skipped"] += 1
            continue
        if source["scan_stamp_ns"] != r.get("scan_stamp_ns"):
            raise ValueError("observation stamp changed")
        if not r["measurement"]:
            continue
        counts["measured"] += 1
        arrival = source.get("scan_arrival_ns", source[args.decision].get("scan_arrival_ns"))
        counts["future_pose_uses"] += any(e.get("used_future") or
            (arrival is not None and e.get("arrival_ns", 0) > arrival)
            for e in source.get("pose_evidence", {}).values())
        decision = source[args.decision]
        yolo_arrival = decision.get("selected_yolo_arrival_ns")
        counts["future_camera_uses"] += bool(yolo_arrival is not None and
            yolo_arrival > decision.get("decision_ns", arrival))
        counts["off_parity_errors"] += stable(r["baseline"]) != stable(r["candidate_off"])
        counts["lineage_errors"] += r["lineage_errors"]
        counts["intensity_errors"] += r["intensity_preservation_errors"]
        counts["count_errors"] += not r["point_count_preserved"]
        if old:
            if prior is None:
                raise ValueError("historical output ended early")
            expected = stable(prior.get("baseline", prior.get(args.decision)))
            counts["historical_parity_errors"] += old_shape(stable(r["baseline"]), expected) != expected
        before = np.fromfile(source["cloud_path"], dtype="<f4").reshape(-1, 4)
        after = np.fromfile(args.output / "clouds" / r["raw_path"], dtype="<f4").reshape(-1, 4)
        if len(before) != len(after) or not np.array_equal(before[:, 3].view("u4"), after[:, 3].view("u4")):
            raise ValueError("raw point count/intensity changed")
        counts["aos_evaluated"] += r["travel_aos"]["status"] == "evaluated"
        if args.kind == "capture" and not capture_ok and r["travel_aos"]["status"] != "blocked_projection_admission":
            raise ValueError("Capture AOS executed despite global admission failure")
        for mode in ("baseline", "raw_euclidean", "travel_aos"):
            for key, value in r[mode].get("timing_ms", {}).items():
                times[mode + "/" + key].append(value)
        for key in ("preprocessing_ms", "lineage_ms", "frozen_copy_ms"):
            times[key].append(r[key])
    if old_file:
        if next(old, None) is not None:
            raise ValueError("historical output has extra rows")
        old_file.close()
    with (args.output / "geometry.jsonl").open() as f:
        if sum(1 for _ in f) != len(prepared) or counts["calls"] != len(prepared):
            raise ValueError("missing or extra evaluation calls")
    error_keys = ("off_parity_errors", "historical_parity_errors", "lineage_errors", "intensity_errors", "count_errors",
                  "future_pose_uses", "future_camera_uses")
    summary = {"schema": "asmc-l1b5a-run-v1", "kind": args.kind, "counts": dict(counts),
               "stable_geometry_sha256": geometry_digest.hexdigest(),
               "projection_admitted_capture": capture_ok,
               "timing_ms": {k: dict(zip(("p50", "p95", "p99"), np.percentile(v, [50, 95, 99]).tolist()))
                             for k, v in sorted(times.items())},
               "passed_preservation_and_parity": not any(counts[k] for k in error_keys),
               "deployment_accepted": False}
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    bound = {"snapshots": args.snapshots, "setting": args.setting, "params": args.params,
             "config": args.config, "executable": args.executable,
             "reference_provenance": args.executable.parent / "reference_provenance.json",
             "frozen_filter_library": args.executable.parent / "libl1b5_frozen_filter.so"}
    for name in ("reference_geometry", "source_manifest"):
        if getattr(args, name):
            bound[name] = getattr(args, name)
    artifacts = {str(p.relative_to(args.output)): {"bytes": p.stat().st_size, "sha256": sha(p)}
                 for p in args.output.rglob("*") if p.is_file()}
    manifest = {"schema": "asmc-l1b5a-run-manifest-v1", "runtime_connected": False,
                "kind": args.kind, "inputs": {k: {"path": str(p), "sha256": sha(p)} for k, p in bound.items()},
                "source_clouds": clouds, "artifacts": artifacts,
                "tool_sha256": tools_at_start}
    manifest["reference_provenance"] = json.loads(bound["reference_provenance"].read_text())
    manifest["tool_commit"] = tool_commit
    (args.output / "manifest.json").write_text(json.dumps(manifest, sort_keys=True, indent=2) + "\n")
    print(canonical(summary))
    if not summary["passed_preservation_and_parity"]:
        raise ValueError("preservation/parity regression; inspect output")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("snapshots", "executable", "setting", "params", "config", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    for name in ("projection-audit", "source-manifest", "reference-geometry"):
        parser.add_argument("--" + name, type=Path)
    parser.add_argument("--kind", choices=("capture", "synthetic", "stationary"), required=True)
    parser.add_argument("--decision", default="baseline")
    run(parser.parse_args())
