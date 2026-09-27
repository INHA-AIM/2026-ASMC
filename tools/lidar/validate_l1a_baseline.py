#!/usr/bin/env python3
"""Verify the pinned L1-A experiment evidence without changing ROS or bags.

Run from any directory. The baseline's source revisions are read from Git objects,
not the current checkout, so a later runtime edit does not rewrite history.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import yaml

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools" / "audit"))
from validate_p1b_contract import Constant, Field, MessageSpec, ros1_md5  # noqa: E402

BASELINE = REPO / "config" / "baselines" / "lidar_l1a_20260926.yaml"


def git_command(*args: str) -> list[str]:
    # Object reads with an explicit Git directory work on the Noetic 2.25
    # bind mount without writing the user's global safe.directory setting.
    return ["git", f"--git-dir={REPO / '.git'}", *args]


def digest_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def git_bytes(revision: str, path: str) -> bytes:
    return subprocess.check_output(
        git_command("show", f"{revision}:{path}"), cwd=REPO,
        stderr=subprocess.DEVNULL,
    )


def remote_message_spec(revision: str, path: str) -> MessageSpec:
    constants = []
    fields = []
    for raw in git_bytes(revision, path).decode("utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        type_name, declaration = line.split(None, 1)
        if "=" in declaration:
            name, value = declaration.split("=", 1)
            constants.append(Constant(type_name, name.strip(), value.strip()))
        else:
            fields.append(Field(type_name, declaration.strip()))
    return MessageSpec(Path(path).stem, Path(path), tuple(constants), tuple(fields))


def md5_at_revision(revision: str, directory: str, name: str) -> str:
    paths = subprocess.check_output(
        git_command("ls-tree", "-r", "--name-only", revision, directory),
        cwd=REPO, text=True,
    ).splitlines()
    specs = {Path(path).stem: remote_message_spec(revision, path)
             for path in paths if path.endswith(".msg")}
    changed = "DetectedObject" if directory == "src/asmc_msgs/msg" else "TrackedObject"
    specs[changed] = remote_message_spec(revision, f"{directory}/{changed}.msg")
    if name not in (changed, changed + "Array"):
        raise ValueError(f"unexpected message: {name}")
    return ros1_md5(name, specs)


def safe_child(root: Path, relative: str) -> Path:
    path = (root / relative).resolve()
    try:
        path.relative_to(root.resolve())
    except ValueError:
        raise ValueError(f"path escapes root: {relative}")
    return path


def check_document(document: dict, data_root: Path | None) -> list[str]:
    errors = []

    def expect(actual, expected, label):
        if actual != expected:
            errors.append(f"{label}: expected {expected!r}, got {actual!r}")

    expect(document.get("schema"), "asmc-lidar-l1a-baseline-v1", "schema")
    revisions = document["source_revisions"]
    for name in ("main", "son_structural_filter", "seojun_planning"):
        revision = revisions[name]
        result = subprocess.run(
            git_command("cat-file", "-e", f"{revision}^{{commit}}"), cwd=REPO,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False,
        )
        expect(result.returncode, 0, f"revision {name}")

    for item in document["environment"]["repo_artifacts"]:
        path = item["path"]
        try:
            content = git_bytes(revisions["main"], path)
            expect(hashlib.sha256(content).hexdigest(), item["sha256"], f"repo SHA {path}")
        except subprocess.CalledProcessError:
            errors.append(f"missing pinned repo artifact {path}")

    dirs = {"Msg": "src/lidar+sensorfusion/src/Msg/msg",
            "asmc_msgs": "src/asmc_msgs/msg"}
    for variant, revision in (("main", revisions["main"]),
                              ("seojun", revisions["seojun_planning"])):
        for type_name, expected in document["wire_md5"][variant].items():
            package, name = type_name.split("/", 1)
            try:
                actual = md5_at_revision(revision, dirs[package], name)
                expect(actual, expected, f"{variant} MD5 {type_name}")
            except (KeyError, subprocess.CalledProcessError, ValueError) as exc:
                errors.append(f"cannot resolve {variant} {type_name}: {exc}")

    if data_root is not None:
        for item in document["artifacts"]:
            try:
                path = safe_child(data_root, item["path"])
            except ValueError as exc:
                errors.append(str(exc))
                continue
            if not path.is_file():
                errors.append(f"missing data artifact {path}")
                continue
            expect(path.stat().st_size, item["bytes"], f"bytes {item['path']}")
            expect(digest_file(path), item["sha256"], f"SHA {item['path']}")

        base = safe_child(data_root, "lidar_assessment/20260926")
        if (base / "stationary_v2/manifest.json").is_file():
            bag_manifest = json.loads((base / "stationary_v2/manifest.json").read_text())
            expect(bag_manifest["bags"][0]["sha256"], document["artifacts"][0]["sha256"],
                   "bag manifest SHA")
            expect(bag_manifest["restore_complete"], True, "Ego restoration")
            expect(bag_manifest["collection_complete"], True, "scene collection")
            expect([scene["name"] for scene in bag_manifest["scenes"]],
                   document["evidence"]["stationary_scenes"], "scene order")
        if (base / "offline_v2_scene_summary.json").is_file():
            offline = json.loads((base / "offline_v2_scene_summary.json").read_text())
            expect(offline["all_frames"]["frames"], document["evidence"]["offline_frames"],
                   "offline frame count")
            expect(offline["evaluator_summary"]["processing_delay_s"],
                   document["evidence"]["offline_processing_delay_s"], "offline delay")
            for scene in ("shadow_A126_entry", "shadow_A126_middle"):
                expect(offline["scenes"][scene]["conditioned_on_protection_available"]["false"]["frames"],
                       document["evidence"]["offline_tunnel_camera_protection_unavailable"],
                       f"offline {scene} camera unavailable")
        if (base / "stationary_v2_scene_audit_final.json").is_file():
            live = json.loads((base / "stationary_v2_scene_audit_final.json").read_text())
            for scene, pinned in document["evidence"]["live_tunnel"].items():
                measured = live["scenes"][scene]["topics"]["/lidar/tunnel/diagnostics"][
                    "diagnostics"]["tunnel_experimental_filter"]
                expect(measured["messages"], pinned["diagnostic_frames"], f"live {scene} frames")
                expect(measured["categorical"]["protection_available"]["false"],
                       pinned["camera_protection_unavailable"], f"live {scene} camera unavailable")
                expect(measured["numeric"]["yolo_offset_ms"]["p95"],
                       pinned["yolo_offset_p95_ms"], f"live {scene} YOLO p95")
        if (base / "ground_benchmark_v1/report.json").is_file():
            ground = json.loads((base / "ground_benchmark_v1/report.json").read_text())
            pinned = document["evidence"]["ground_benchmark"]
            expect(ground["samples"], pinned["samples"], "ground samples")
            for algorithm, field in (("patchworkpp_sensor180", "patchworkpp_sensor180_retained"),
                                     ("pcl_plane_reference", "pcl_plane_reference_retained")):
                by_semantic = ground["results"][algorithm]["obstacle_by_semantic"]
                expect(sum(v["retained"] for label, v in by_semantic.items() if label != "109"),
                       pinned[field], f"{algorithm} non-109 retained")
                expect(sum(v["points"] for label, v in by_semantic.items() if label != "109"),
                       pinned["actor_obstacle_points_excluding_109"], f"{algorithm} non-109 total")
                for label, metric in (("164", "low_obstacle"), ("118", "pedestrian")):
                    expect(by_semantic[label]["points"], pinned[f"{metric}_{label}_points"],
                           f"{algorithm} {label} points")
                    prefix = "patchworkpp" if algorithm.startswith("patchwork") else "pcl"
                    expect(by_semantic[label]["retained"], pinned[f"{prefix}_{metric}_retained"],
                           f"{algorithm} {label} retained")

    missions = document.get("missions", [])
    fixture = document["first_change_fixture"]
    fixture_path = safe_child(REPO, fixture["path"])
    if not fixture_path.is_file():
        errors.append(f"missing first-change fixture {fixture_path}")
    else:
        expect(fixture_path.stat().st_size, fixture["bytes"], "fixture bytes")
        expect(digest_file(fixture_path), fixture["sha256"], "fixture SHA")
    expect(len(missions), 9, "mission evaluation rows")
    if len({item["id"] for item in missions}) != len(missions):
        errors.append("duplicate mission evaluation ID")
    regression = document.get("regression_cases", [])
    if not regression or any(not item.get("state") or not item.get("input") for item in regression):
        errors.append("regression input/status incomplete")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, default=BASELINE)
    parser.add_argument("--data-root", type=Path, default=REPO.parent / "data")
    parser.add_argument("--skip-data", action="store_true", help="check pinned Git/schema only")
    args = parser.parse_args()
    document = yaml.safe_load(args.baseline.read_text())
    errors = check_document(document, None if args.skip_data else args.data_root)
    if errors:
        for error in errors:
            print("FAIL:", error, file=sys.stderr)
        return 1
    print("L1-A baseline verified: revisions, wire MD5, mission matrix" +
          ("" if args.skip_data else ", evidence SHA and measured counts"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
