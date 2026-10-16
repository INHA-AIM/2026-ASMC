#!/usr/bin/env python3
"""Build and verify the deterministic P4-A perception runtime baseline.

This generator belongs to input commit ``1bd8ec9``.  After P4-C6 package
closeout it is retained only to reproduce that historical commit in a
separate worktree; it is not a current-tree inventory generator.

P4-A is an inventory-only gate.  It must not move packages, rewrite launch
files, change model artifacts, or alter runtime behavior.  The manifest makes
the current split runtime and its known migration risks reviewable before the
physical ``asmc_perception`` consolidation starts.
"""

from __future__ import annotations

import argparse
import difflib
import hashlib
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Sequence

import yaml


SCHEMA_VERSION = 1
BASELINE_ID = "p4a_perception_runtime_v1"
INPUT_COMMIT = "1bd8ec9"
MANIFEST_REL = Path("config/baselines/perception_runtime_baseline_v1.yaml")
REPORT_REL = Path("docs/records/camera/perception-p4a-runtime-v1.md")

MODEL_ARTIFACTS = (
    {
        "id": "camera_twinlite_v6",
        "path": "artifacts/perception_eval/runs/"
        "twinlite_medium_morai_v6_road_marking_init_b_b12_e18_20260908/"
        "best_mean.pt",
        "selected": True,
        "consumer": "three_view_lane_pipeline.launch",
        "sha256": "a26311b3af4e92880f632509eff3c81defc1b95135f5b463c7a08e5b6e23d24b",
        "size_bytes": 8141406,
    },
    {
        "id": "traffic_light_resnet18",
        "path": "src/traffic_light/best.pt",
        "selected": True,
        "consumer": "traffic_light/live_infer.py",
        "sha256": "70ddaf222e7bc6ff2b4748c4b07c1b24dec72a2671f39bc18e0b243ddd93c0d5",
        "size_bytes": 44795403,
    },
    {
        "id": "lidar_camera_yolo11m",
        "path": "src/lidar+sensorfusion/src/Main/YOLOWeight/yolo11m.pt",
        "selected": True,
        "consumer": "Main/main.launch",
        "sha256": "d5ffc1a674953a08e11a8d21e022781b1b23a19b730afc309290bd9fb5305b95",
        "size_bytes": 40684120,
    },
    {
        "id": "lidar_camera_yolo11m_duplicate",
        "path": "src/lidar+sensorfusion/src/Main/YOLOWeight/yolo11m(1).pt",
        "selected": False,
        "consumer": None,
        "sha256": "6cf89c17d9a1122b9b6f3f6abe7574bf3f80cc4653e549d1bfd102ec8e421bf9",
        "size_bytes": 19155482,
    },
)

PACKAGE_SCOPES = (
    ("camera_semantic", "src/perception/camera_semantic_perception", "catkin_team_runtime", "asmc_perception/camera"),
    ("drivable_bev", "src/perception/drivable_bev", "catkin_team_mixed", "asmc_perception/bev_lane + tools/perception"),
    ("traffic_light", "src/traffic_light", "catkin_team_runtime", "asmc_perception/traffic_light"),
    ("data_collection", "src/data_collection", "catkin_team_mixed", "asmc_perception/recorder + tools/perception"),
    ("lidar_main", "src/lidar+sensorfusion/src/Main", "nested_catkin_team_runtime", "asmc_perception/lidar_fusion"),
    ("lidar_msg", "src/lidar+sensorfusion/src/Msg", "nested_catkin_team_duplicate", "asmc_msgs"),
    ("lidar_morai_msgs", "src/lidar+sensorfusion/src/morai_msgs", "nested_catkin_external_duplicate", "pinned MORAI-ROS_morai_msgs"),
    ("velodyne_vendor", "src/lidar+sensorfusion/src/velodyne/velodyne", "nested_catkin_upstream_vendor", "system dependency or vendor"),
    ("velodyne_driver_vendor", "src/lidar+sensorfusion/src/velodyne/velodyne_driver", "nested_catkin_upstream_vendor", "system dependency or vendor"),
    ("velodyne_laserscan_vendor", "src/lidar+sensorfusion/src/velodyne/velodyne_laserscan", "nested_catkin_upstream_vendor", "system dependency or vendor"),
    ("velodyne_msgs_vendor", "src/lidar+sensorfusion/src/velodyne/velodyne_msgs", "nested_catkin_upstream_vendor", "system dependency or vendor"),
    ("velodyne_pcl_vendor", "src/lidar+sensorfusion/src/velodyne/velodyne_pcl", "nested_catkin_upstream_vendor", "system dependency or vendor"),
    ("velodyne_pointcloud_vendor", "src/lidar+sensorfusion/src/velodyne/velodyne_pointcloud", "nested_catkin_upstream_vendor", "system dependency or vendor"),
    ("twinlite_morai", "src/perception/twinlite_morai", "non_catkin_backend_offline", "asmc_perception backend + tools/perception"),
    ("morai_3d_detection", "src/perception/morai_3d_detection", "non_catkin_research", "tools/perception candidate"),
    ("learning", "src/learning", "legacy_placeholder", "delete after reference gate"),
    ("learning_by_cheating", "src/learning_by_cheating", "legacy_runtime_research", "replace then delete; preserve in git history"),
)

DUPLICATE_MESSAGES = {
    "TrackedObject": "6c96135eb03a3ee93bba474fa835f57d",
    "TrackedObjectArray": "e127b2aabd7494ad01a5a56b005bf07e",
    "Yolo_bbox": "605f5dd6f8f921f2dc15199c760840f9",
    "Yolo_bbox_arr": "9604a9d8104d38c3bd9830382a3571d0",
}

SOURCE_ASSERTIONS = {
    "src/perception/drivable_bev/launch/three_view_lane_pipeline.launch": (
        "ASMC_sensor_set_v2.json",
        "twinlite_v6_shadow_boundary_contract_v1",
    ),
    "src/lidar+sensorfusion/src/Main/main.launch": (
        '<remap from="/Competition_topic" to="/Ego_topic"/>',
        "../../../link_node/out/road_mask.png",
        "YOLOWeight/yolo11m.pt",
    ),
    "src/lidar+sensorfusion/src/Main/setting.yaml": (
        "ASMC_sensor_set_v1.json",
        "Hz: 15",
        "Hz: 30",
    ),
    "src/lidar+sensorfusion/src/Main/src/Parser/Parser.cpp": (
        'subscribe("/velodyne_points"',
        'advertise<sensor_msgs::CompressedImage>("/camera/image/compressed"',
        'advertise<morai_msgs::CtrlCmd>("/ctrl_cmd_0"',
    ),
    "src/lidar+sensorfusion/src/Main/src/PlannerInterface/PlannerObjectRosPublisher.cpp": (
        '"/lidar/tracked_objects_map"',
        '"/last_lidar"',
        '"waize_katri_2025-10-23"',
    ),
    "src/traffic_light/live_infer.py": (
        'default="/cam_extra/image_jpeg/compressed"',
        'default="/traffic_light/state"',
        "std_msgs.msg import String",
    ),
}

TEXT_SUFFIXES = {
    ".bash", ".cc", ".cfg", ".cmake", ".cpp", ".cxx", ".h", ".hpp",
    ".json", ".launch", ".md", ".msg", ".py", ".sh", ".txt", ".xml",
    ".yaml", ".yml",
}


class BaselineError(RuntimeError):
    """A required P4-A input or invariant is missing."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_tree_sha256(repo_root: Path, paths: Iterable[Path]) -> str:
    digest = hashlib.sha256()
    for path in sorted(paths, key=lambda value: value.as_posix()):
        relative = path.relative_to(repo_root).as_posix()
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(sha256_file(path).encode("ascii"))
        digest.update(b"\n")
    return digest.hexdigest()


def tracked_files(repo_root: Path, prefixes: Sequence[str]) -> List[Path]:
    # The repository is bind-mounted into the root-owned ROS container while
    # the host checkout belongs to the developer.  Explicit git-dir/work-tree
    # mode is read-only and avoids mutating the container's global
    # safe.directory configuration.
    command = [
        "git", f"--git-dir={repo_root / '.git'}", f"--work-tree={repo_root}",
        "ls-files", "-z", "--", *prefixes,
    ]
    try:
        output = subprocess.check_output(command)
    except (OSError, subprocess.CalledProcessError) as exc:
        raise BaselineError("git ls-files is required for the deterministic P4-A audit") from exc
    paths = []
    for raw in output.split(b"\0"):
        if not raw:
            continue
        path = repo_root / raw.decode("utf-8")
        if path.is_file():
            paths.append(path)
    return sorted(set(paths), key=lambda value: value.as_posix())


def package_metadata(path: Path) -> Mapping[str, Any] | None:
    package_xml = path / "package.xml"
    if not package_xml.is_file():
        return None
    root = ET.parse(package_xml).getroot()
    dependencies = sorted(
        {
            (element.text or "").strip()
            for tag in (
                "depend", "build_depend", "build_export_depend", "exec_depend",
                "test_depend",
            )
            for element in root.findall(tag)
            if (element.text or "").strip()
        }
    )
    return {
        "name": (root.findtext("name") or "").strip(),
        "version": (root.findtext("version") or "").strip(),
        "dependencies": dependencies,
        "package_xml_sha256": sha256_file(package_xml),
    }


def audit_scopes(repo_root: Path) -> Mapping[str, Any]:
    items = []
    all_files: List[Path] = []
    for scope_id, relative, classification, destination in PACKAGE_SCOPES:
        root = repo_root / relative
        if not root.exists():
            raise BaselineError(f"missing perception scope: {relative}")
        files = tracked_files(repo_root, [relative])
        if not files:
            raise BaselineError(f"perception scope has no tracked files: {relative}")
        all_files.extend(files)
        items.append(
            {
                "id": scope_id,
                "path": relative,
                "classification": classification,
                "target": destination,
                "tracked_file_count": len(files),
                "launch_file_count": sum(path.suffix == ".launch" for path in files),
                "python_file_count": sum(path.suffix == ".py" for path in files),
                "cpp_file_count": sum(path.suffix in {".cc", ".cpp", ".cxx"} for path in files),
                "tree_sha256": canonical_tree_sha256(repo_root, files),
                "package": package_metadata(root),
            }
        )
    return {
        "scope_count": len(items),
        "tracked_file_count": len(set(all_files)),
        "tree_sha256": canonical_tree_sha256(repo_root, set(all_files)),
        "items": items,
    }


def audit_launches(repo_root: Path) -> Mapping[str, Any]:
    prefixes = [item[1] for item in PACKAGE_SCOPES]
    items = []
    node_count = 0
    for path in tracked_files(repo_root, prefixes):
        if path.suffix != ".launch":
            continue
        try:
            root = ET.parse(path).getroot()
        except ET.ParseError as exc:
            raise BaselineError(f"invalid launch XML {path}: {exc}") from exc
        nodes = []
        for node in root.iter("node"):
            nodes.append(
                {
                    "package": node.get("pkg", ""),
                    "type": node.get("type", ""),
                    "name": node.get("name", ""),
                    "required": node.get("required", "false"),
                }
            )
        nodes.sort(key=lambda value: (value["package"], value["type"], value["name"]))
        includes = sorted(
            include.get("file", "")
            for include in root.iter("include")
            if include.get("file")
        )
        node_count += len(nodes)
        items.append(
            {
                "path": path.relative_to(repo_root).as_posix(),
                "nodes": nodes,
                "includes": includes,
                "sha256": sha256_file(path),
            }
        )
    return {"file_count": len(items), "node_declaration_count": node_count, "items": items}


def audit_entrypoints(repo_root: Path) -> Mapping[str, Any]:
    prefixes = [item[1] for item in PACKAGE_SCOPES]
    items = []
    for path in tracked_files(repo_root, prefixes):
        if path.name != "CMakeLists.txt":
            continue
        text = path.read_text(encoding="utf-8", errors="replace")
        for match in re.finditer(r"\badd_executable\s*\(\s*([^\s\)]+)", text):
            items.append(
                {
                    "kind": "cmake_add_executable",
                    "name": match.group(1),
                    "source": path.relative_to(repo_root).as_posix(),
                }
            )
        for match in re.finditer(
            r"\bcatkin_install_python\s*\(\s*PROGRAMS(.*?)DESTINATION",
            text,
            flags=re.DOTALL,
        ):
            block = re.sub(r"#[^\n]*", "", match.group(1))
            for token in re.split(r"\s+", block.strip()):
                if token and not token.startswith("${"):
                    items.append(
                        {
                            "kind": "catkin_install_python",
                            "name": token,
                            "source": path.relative_to(repo_root).as_posix(),
                        }
                    )
    # traffic_light/run.sh is the only non-catkin wrapper currently presented
    # as an operator-facing inference entrypoint.
    wrapper = repo_root / "src/traffic_light/run.sh"
    if wrapper.is_file():
        items.append(
            {
                "kind": "operator_wrapper",
                "name": "run.sh",
                "source": wrapper.relative_to(repo_root).as_posix(),
            }
        )
    items.sort(key=lambda value: (value["source"], value["kind"], value["name"]))
    return {"count": len(items), "items": items}


def endpoint(
    topic: str,
    message_type: str,
    producer: str,
    role: str,
    frame: str,
    timestamp: str,
    deployment_allowed: bool = True,
) -> Mapping[str, Any]:
    return {
        "topic": topic,
        "type": message_type,
        "producer": producer,
        "role": role,
        "frame_policy": frame,
        "timestamp_policy": timestamp,
        "deployment_allowed": deployment_allowed,
    }


def audit_topics() -> Mapping[str, Any]:
    inputs = [
        endpoint("/image_jpeg/compressed", "sensor_msgs/CompressedImage", "asmc_bridge", "front_rgb_input", "camera_front", "source header; bridge receive-time fallback"),
        endpoint("/cam_left/image_jpeg/compressed", "sensor_msgs/CompressedImage", "asmc_bridge", "left_rgb_input", "camera_left", "source header; bridge receive-time fallback"),
        endpoint("/cam_right/image_jpeg/compressed", "sensor_msgs/CompressedImage", "asmc_bridge", "right_rgb_input", "camera_right", "source header; bridge receive-time fallback"),
        endpoint("/cam_extra/image_jpeg/compressed", "sensor_msgs/CompressedImage", "asmc_bridge", "traffic_light_rgb_input", "camera_extra", "source header; bridge receive-time fallback"),
        endpoint("/velodyne_points", "sensor_msgs/PointCloud2", "asmc_bridge", "lidar_input", "velodyne", "source header; bridge receive-time fallback"),
        endpoint("/gps", "morai_msgs/GPSMessage", "asmc_bridge", "localization_input", "message has no Header contract", "bridge receive time is not carried in message"),
        endpoint("/imu", "sensor_msgs/Imu", "asmc_bridge", "orientation_input", "imu_link", "source header; bridge receive-time fallback"),
        endpoint("/Ego_topic", "morai_msgs/EgoVehicleStatus", "asmc_bridge", "vehicle_input_and_Main_compatibility_remap", "map-like simulator coordinates; no Header", "message has no Header"),
    ]
    outputs: List[Mapping[str, Any]] = []
    for view in ("front", "left", "right"):
        camera_frame = f"camera_{view}"
        for suffix in ("drivable/probability", "lane/probability", "road_marking/class_id", "road_marking/confidence"):
            outputs.append(endpoint(f"/perception/camera/{view}/{suffix}", "sensor_msgs/Image", "camera_semantic_perception", "camera_semantic", camera_frame, "preserve input image header"))
        for suffix in ("class_id", "confidence"):
            outputs.append(endpoint(f"/perception/camera/{view}/road_marking/model_grid/{suffix}", "sensor_msgs/Image", "camera_semantic_perception", "camera_model_grid", camera_frame, "preserve input image header"))
        for suffix in ("drivable_probability", "road_marking/class_id", "road_marking/confidence", "coverage"):
            outputs.append(endpoint(f"/perception/bev/debug/{view}/{suffix}", "sensor_msgs/Image", "drivable_bev", "per_view_bev_debug", "base_link", "preserve semantic source stamp"))
    outputs.extend(
        [
            endpoint("/perception/camera/diagnostics", "diagnostic_msgs/DiagnosticArray", "camera_semantic_perception", "diagnostic", "not geometric", "publish/receive time"),
            endpoint("/perception/bev/camera/drivable_probability", "sensor_msgs/Image", "drivable_bev", "fused_bev", "base_link", "newest synchronized source stamp"),
            endpoint("/perception/bev/camera/road_marking/class_id", "sensor_msgs/Image", "drivable_bev", "fused_bev", "base_link", "newest synchronized source stamp"),
            endpoint("/perception/bev/camera/road_marking/confidence", "sensor_msgs/Image", "drivable_bev", "fused_bev", "base_link", "newest synchronized source stamp"),
            endpoint("/perception/bev/camera/coverage", "sensor_msgs/Image", "drivable_bev", "fused_bev", "base_link", "newest synchronized source stamp"),
            endpoint("/perception/bev/camera/source_count", "sensor_msgs/Image", "drivable_bev", "fused_bev", "base_link", "newest synchronized source stamp"),
            endpoint("/perception/bev/diagnostics", "diagnostic_msgs/DiagnosticArray", "drivable_bev", "diagnostic", "not geometric", "publish/receive time"),
            endpoint("/perception/lane/boundary_observations", "asmc_msgs/LaneBoundaryObservationArray", "drivable_bev", "typed_lane_handoff", "base_link", "synchronized camera source stamp"),
            endpoint("/perception/lane/boundary_family", "asmc_msgs/LaneBoundaryFamily", "drivable_bev", "provisional_candidate", "base_link", "synchronized camera source stamp", False),
            endpoint("/perception/lane/boundaries", "visualization_msgs/MarkerArray", "drivable_bev", "legacy_visualization", "base_link", "synchronized camera source stamp", False),
            endpoint("/perception/lane/diagnostics", "diagnostic_msgs/DiagnosticArray", "drivable_bev", "diagnostic", "not geometric", "publish/receive time"),
            endpoint("/traffic_light/state", "std_msgs/String", "traffic_light", "traffic_light_label", "not geometric; unheadered", "no source stamp; stale timeout is local monotonic time"),
            endpoint("/traffic_light/detail", "std_msgs/String JSON", "traffic_light", "traffic_light_detail", "not geometric; unheadered", "no source stamp; inference timing only"),
            endpoint("/camera/image/compressed", "sensor_msgs/CompressedImage", "Main", "LiDAR_fusion_camera_compatibility_relay", "preserve front camera frame", "preserve front camera header"),
            endpoint("/YOLO/BBox", "Msg/Yolo_bbox_arr", "Main/yolo_detection_node", "LiDAR_camera_fusion_internal", "image frame", "preserve relayed image header"),
            endpoint("/lidar/tracked_objects_map", "Msg/TrackedObjectArray", "Main", "tracked_objects", "map", "LiDAR source stamp"),
            endpoint("/lidar/tracking_markers_map", "visualization_msgs/MarkerArray", "Main", "tracked_object_visualization", "map", "LiDAR source stamp", False),
            endpoint("/last_lidar", "Msg/TrackedObjectArray", "Main", "legacy_filtered_objects", "map", "LiDAR source stamp"),
            endpoint("/last_lidar_markers", "visualization_msgs/MarkerArray", "Main", "legacy_filtered_visualization", "velodyne", "LiDAR source stamp", False),
            endpoint("/ctrl_cmd_0", "morai_msgs/CtrlCmd", "Main", "legacy_control_output_not_perception_contract", "not geometric", "unheadered", False),
        ]
    )
    outputs.sort(key=lambda item: item["topic"])
    return {
        "coordinate_contract": {
            "camera_bev_and_lane": "base_link, x forward, y left, z up",
            "lidar_tracked_objects": "map; positions and velocity are absolute in array header frame",
            "hd_map_context": "map; consumers perform the map/base_link transform exactly once",
        },
        "inputs": inputs,
        "outputs": outputs,
        "input_count": len(inputs),
        "output_count": len(outputs),
    }


def audit_models(repo_root: Path) -> Mapping[str, Any]:
    items = []
    for expected in MODEL_ARTIFACTS:
        path = repo_root / str(expected["path"])
        if not path.is_file():
            raise BaselineError(f"missing model artifact: {expected['path']}")
        actual_sha = sha256_file(path)
        actual_size = path.stat().st_size
        if actual_sha != expected["sha256"] or actual_size != expected["size_bytes"]:
            raise BaselineError(
                f"model artifact drift: {expected['path']} "
                f"sha={actual_sha} size={actual_size}"
            )
        items.append(dict(expected))
    return {
        "policy": "runtime selects by explicit path; migration target is external artifact manifest with SHA-256",
        "items": items,
        "selected_count": sum(bool(item["selected"]) for item in items),
        "unselected_duplicate_count": sum(not bool(item["selected"]) for item in items),
    }


def audit_duplicate_messages(repo_root: Path) -> Mapping[str, Any]:
    items = []
    for name, ros_md5 in sorted(DUPLICATE_MESSAGES.items()):
        path = repo_root / "src/lidar+sensorfusion/src/Msg/msg" / f"{name}.msg"
        if not path.is_file():
            raise BaselineError(f"missing duplicate message: {path}")
        items.append(
            {
                "type": f"Msg/{name}",
                "ros1_md5": ros_md5,
                "source": path.relative_to(repo_root).as_posix(),
                "source_sha256": sha256_file(path),
                "migration": f"additive asmc_msgs equivalent then consumer cutover; preserve {ros_md5}",
            }
        )
    return {"count": len(items), "items": items}


def audit_bag_profiles(repo_root: Path) -> Mapping[str, Any]:
    root = repo_root / "src/data_collection/config/bag_profiles"
    paths = {path.stem: path for path in sorted(root.glob("*.yaml"))}
    raw_values = {
        stem: (yaml.safe_load(path.read_text(encoding="utf-8")) or {})
        for stem, path in paths.items()
    }

    def resolve(stem: str, stack: Sequence[str] = ()) -> Mapping[str, Any]:
        if stem not in raw_values:
            raise BaselineError(f"bag profile extends missing parent: {stem}")
        if stem in stack:
            raise BaselineError(f"bag profile inheritance cycle: {' -> '.join((*stack, stem))}")
        child = dict(raw_values[stem])
        parent_name = child.get("extends")
        if not parent_name:
            return child
        merged = dict(resolve(str(parent_name), (*stack, stem)))
        merged.update(child)
        return merged

    items = []
    for stem, path in sorted(paths.items()):
        raw = raw_values[stem]
        value = resolve(stem)
        items.append(
            {
                "name": str(value.get("name", path.stem)),
                "path": path.relative_to(repo_root).as_posix(),
                "extends": raw.get("extends"),
                "kind": value.get("kind", "inherited"),
                "sensor_set": value.get("sensor_set", "inherited"),
                "privileged_gt": bool(value.get("privileged_gt", False)),
                "deployment_allowed": bool(value.get("deployment_allowed", False)),
                "required_topic_count": len(value.get("required_topics", [])),
                "optional_topic_count": len(value.get("optional_topics", [])),
                "sha256": sha256_file(path),
            }
        )
    if not items:
        raise BaselineError("no bag profiles found")
    return {
        "count": len(items),
        "deployment_allowed_count": sum(item["deployment_allowed"] for item in items),
        "privileged_gt_count": sum(item["privileged_gt"] for item in items),
        "items": items,
    }


def verify_sources(repo_root: Path) -> List[Mapping[str, Any]]:
    results = []
    for relative, tokens in sorted(SOURCE_ASSERTIONS.items()):
        path = repo_root / relative
        if not path.is_file():
            raise BaselineError(f"missing asserted source: {relative}")
        text = path.read_text(encoding="utf-8", errors="replace")
        missing = [token for token in tokens if token not in text]
        if missing:
            raise BaselineError(f"source assertion failed for {relative}: {missing}")
        results.append({"path": relative, "sha256": sha256_file(path), "assertion_count": len(tokens)})
    return results


def external_reference_audit(repo_root: Path) -> Mapping[str, Any]:
    candidates = tracked_files(repo_root, ["src", "scripts", "tools", "config"])
    self_paths = {
        MANIFEST_REL.as_posix(),
        "tools/audit/build_perception_runtime_baseline.py",
        "tools/audit/test_perception_runtime_baseline.py",
    }
    rules = (
        ("learning_by_cheating", ("src/learning_by_cheating/", "lbc_bev"), "src/learning_by_cheating/"),
        ("duplicate_Msg", ("<Msg/", "Msg::", "from Msg.msg", "import Msg.msg"), "src/lidar+sensorfusion/src/Msg/"),
        ("nested_morai_msgs", ("src/lidar+sensorfusion/src/morai_msgs",), "src/lidar+sensorfusion/src/morai_msgs/"),
        ("legacy_link_node_mask", ("link_node/out/road_mask", "../../../link_node/out/"), "src/asmc_hd_map/development/won_tae_link_node/"),
    )
    results = []
    for rule_id, tokens, excluded_prefix in rules:
        blocking_matches = []
        reference_matches = []
        blocking_occurrences = 0
        reference_occurrences = 0
        for path in candidates:
            relative = path.relative_to(repo_root).as_posix()
            if (
                relative in self_paths
                or relative.startswith(excluded_prefix)
                or path.suffix.lower() not in TEXT_SUFFIXES
            ):
                continue
            text = path.read_text(encoding="utf-8", errors="replace")
            count = sum(text.count(token) for token in tokens)
            if count:
                reference_only = (
                    path.suffix.lower() == ".md"
                    or relative.startswith("config/baselines/")
                    or relative.startswith("tools/audit/")
                )
                if reference_only:
                    reference_occurrences += count
                    reference_matches.append(relative)
                else:
                    blocking_occurrences += count
                    blocking_matches.append(relative)
        results.append(
            {
                "id": rule_id,
                "blocking_external_file_count": len(blocking_matches),
                "blocking_occurrence_count": blocking_occurrences,
                "blocking_files": sorted(blocking_matches),
                "reference_only_file_count": len(reference_matches),
                "reference_only_occurrence_count": reference_occurrences,
                "reference_only_files": sorted(reference_matches),
            }
        )
    return {"rules": results}


def build_manifest(repo_root: Path) -> Mapping[str, Any]:
    repo_root = repo_root.resolve()
    assertions = verify_sources(repo_root)
    manifest = {
        "schema_version": SCHEMA_VERSION,
        "baseline_id": BASELINE_ID,
        "input_commit": INPUT_COMMIT,
        "purpose": "P4-A perception package/topic/frame/timestamp/model/oracle migration baseline",
        "determinism": {
            "timestamps_omitted": True,
            "paths": "repository-relative",
            "tree_inventory": "git tracked files only",
        },
        "protected_scope": {
            "package_moves": False,
            "package_deletions": False,
            "runtime_behavior_changes": False,
            "model_artifact_changes": False,
            "message_changes": False,
            "planning_owned_source_changes": False,
        },
        "runtime_scopes": audit_scopes(repo_root),
        "launch_inventory": audit_launches(repo_root),
        "declared_entrypoints": audit_entrypoints(repo_root),
        "topic_contract": audit_topics(),
        "rate_contract": {
            "bridge_configured_or_nominal_hz": {
                "front_rgb": 20.0,
                "left_rgb": 20.0,
                "right_rgb": 20.0,
                "traffic_light_rgb": 10.0,
                "vlp16_configured": 15.0,
                "gps": 5.0,
                "imu_nominal": 50.0,
            },
            "bridge_local_observed_hz": {
                "imu_approx_range": [42.0, 44.0],
                "vlp16_complete_scan_approx": 12.0,
            },
            "vlp16_acceptance": "configured/consumer loop 15 Hz; approximately 12 Hz complete scans accepted without duplication",
            "three_view_lane_model_grid_max_hz": 15.0,
            "camera_bev_fused_max_hz": 10.0,
        },
        "duplicate_messages": audit_duplicate_messages(repo_root),
        "model_artifacts": audit_models(repo_root),
        "bag_profiles": audit_bag_profiles(repo_root),
        "oracle_policy": {
            "deployment_forbidden_inputs": [
                "MORAI Capture Mode or gRPC truth",
                "Semantic/Instance GT camera or LiDAR",
                "/Object_topic or simulator object truth",
                "simulator TrafficLight status",
                "disallowed global pose during GPS-shadow operation",
                "GT-derived masks, pseudo-GT, or map overlays used as live perception input",
            ],
            "allowed_use": "offline development/evaluation profiles explicitly marked privileged_gt=true and deployment_allowed=false",
        },
        "known_integration_risks": [
            {
                "id": "camera_calibration_v2_provenance",
                "severity": "high",
                "fact": "operational three-view lane launch and cameras_v2.yaml name v2 while bridge deployment baseline is sensor set v3",
                "p4_action": "prove geometry parity or create a v3-named calibration before changing runtime default",
            },
            {
                "id": "lidar_stale_drivable_mask",
                "severity": "critical",
                "fact": "Main loads ../../../link_node/out/road_mask and map id waize_katri_2025-10-23 instead of the final asmc_hd_map contract",
                "p4_action": "replace with /hd_map vector/grid consumer and run object-filter parity",
            },
            {
                "id": "nested_message_and_morai_duplicates",
                "severity": "high",
                "fact": "Main depends on nested Msg and a second morai_msgs package",
                "p4_action": "migrate TrackedObject/Yolo messages additively to asmc_msgs and pin the outer morai_msgs dependency",
            },
            {
                "id": "traffic_light_unheadered_output",
                "severity": "high",
                "fact": "state/detail are std_msgs/String without source timestamp, validity, or typed confidence",
                "p4_action": "define typed additive output and retain String compatibility until planning cutover",
            },
            {
                "id": "lidar_legacy_side_effects",
                "severity": "medium",
                "fact": "Main relays /camera/image/compressed and publishes /ctrl_cmd_0 in addition to perception results",
                "p4_action": "separate perception ownership; preserve relay compatibility only while YOLO consumer requires it",
            },
            {
                "id": "lidar_legacy_sensor_rate_provenance",
                "severity": "medium",
                "fact": "Main/setting.yaml names sensor set v1 and camera/GPS 30 Hz although the bridge baseline is v3 at 20/5 Hz; LiDAR loop is correctly 15 Hz",
                "p4_action": "make bridge topics authoritative and validate queues/timing with measured v3 rates before moving Main",
            },
            {
                "id": "bag_profiles_v2_provenance",
                "severity": "medium",
                "fact": "deployment-allowed camera_raw, lidar_raw, and full_raw profiles still declare sensor set v2 while the bridge deployment default is v3",
                "p4_action": "add explicit v3 recording profiles or a reviewed sensor-set override before using these profiles as deployment evidence",
            },
            {
                "id": "mixed_runtime_and_offline_tools",
                "severity": "medium",
                "fact": "drivable_bev and data_collection contain runtime nodes and offline dataset/audit tools together",
                "p4_action": "move only offline entrypoints to tools/perception after import and bag parity",
            },
            {
                "id": "duplicate_unselected_yolo_weight",
                "severity": "low",
                "fact": "yolo11m(1).pt is tracked but no audited launch selects it",
                "p4_action": "remove only after artifact provenance review",
            },
        ],
        "migration_gates": [
            "canonical asmc_perception launch reproduces audited input/output topics or supplies an explicit compatibility remap",
            "all geometric outputs keep the audited frame and source-stamp policy",
            "selected model paths and SHA-256 values are recorded in the deployment artifact manifest",
            "raw/result bag replay demonstrates topic, type, geometry, rate, staleness, and latency parity",
            "traffic-light typed replacement is additive until every consumer leaves String topics",
            "lidar tracked-object messages move to asmc_msgs without silent field/unit/frame changes",
            "nested morai_msgs and Msg are removed only after clean build and zero consumer references",
            "learning and learning_by_cheating are removed only after blocking external reference count reaches zero and BEV/map replacement parity passes",
            "vendor Velodyne packages remain external/vendor dependencies rather than being absorbed into asmc_perception",
            "oracle profiles remain deployment-blocked",
        ],
        "source_assertions": assertions,
        "external_reference_gates": external_reference_audit(repo_root),
        "next_step": "P4-B canonical asmc_perception facade and additive interface decisions; no physical source deletion",
    }
    return manifest


def dump_manifest(value: Mapping[str, Any]) -> str:
    return yaml.safe_dump(
        value,
        allow_unicode=True,
        default_flow_style=False,
        sort_keys=False,
        width=110,
    )


def render_report(manifest: Mapping[str, Any]) -> str:
    scopes = manifest["runtime_scopes"]
    topics = manifest["topic_contract"]
    models = manifest["model_artifacts"]
    bags = manifest["bag_profiles"]
    launches = manifest["launch_inventory"]
    lines = [
        "# P4-A perception runtime baseline",
        "",
        f"> **식별자:** `{manifest['baseline_id']}`",
        f"> **입력 commit:** `{manifest['input_commit']}`",
        "> **범위:** 패키지 이동·삭제·동작 변경 전 현재 인지 runtime 계약 동결",
        "",
        "## 결론",
        "",
        "현재 인지 runtime은 아직 하나의 `asmc_perception` 패키지가 아니다. camera semantic, metric BEV/lane,",
        "traffic light, LiDAR/fusion, recorder가 분산돼 있고 LiDAR는 outer catkin에서 제외된 nested workspace다.",
        "P4-A는 이 상태를 정상 목표로 승인한 것이 아니라, 이후 이동에서 frame·timestamp·topic·model 의미를",
        "잃지 않기 위한 회귀 기준으로 동결했다.",
        "",
        f"- 감사 scope: {scopes['scope_count']}개, tracked file {scopes['tracked_file_count']}개",
        f"- launch: {launches['file_count']}개, node 선언 {launches['node_declaration_count']}개, declared entrypoint {manifest['declared_entrypoints']['count']}개",
        f"- 계약 입력: {topics['input_count']}개, 출력: {topics['output_count']}개",
        f"- 모델 artifact: {len(models['items'])}개(선택 {models['selected_count']}, 미선택 중복 {models['unselected_duplicate_count']})",
        f"- bag profile: {bags['count']}개(deployment 허용 {bags['deployment_allowed_count']}, privileged GT {bags['privileged_gt_count']})",
        f"- 중복 ROS message: {manifest['duplicate_messages']['count']}개",
        "",
        "기계 판독 정본은 `config/baselines/perception_runtime_baseline_v1.yaml`, 재생성·검증기는",
        "`tools/audit/build_perception_runtime_baseline.py`다.",
        "",
        "## 현재 package 경계",
        "",
        "| ID | 현재 경로 | 분류 | 이관 목표 | tracked file |",
        "|---|---|---|---|---:|",
    ]
    for item in scopes["items"]:
        lines.append(
            f"| `{item['id']}` | `{item['path']}` | `{item['classification']}` | "
            f"{item['target']} | {item['tracked_file_count']} |"
        )
    lines.extend(
        [
            "",
            "## 좌표·시간 계약",
            "",
            "- camera metric BEV와 lane observation은 `base_link`이며 x-forward, y-left, z-up이다.",
            "- LiDAR tracked object 위치·속도는 array header의 `map` frame 절대좌표다.",
            "- HD Map context도 `map`이다. consumer는 `map↔base_link` 변환을 정확히 한 번만 적용한다.",
            "- camera semantic은 입력 image header를 보존하고, BEV/lane은 동기화된 source stamp를 이어받는다.",
            "- traffic-light의 현행 String 두 토픽에는 header/source stamp가 없다. stale timeout은 node 내부",
            "  monotonic time뿐이므로 typed additive 계약이 필요하다.",
            "- 설정·nominal 기준은 RGB `20/20/20/10 Hz`, VLP16 설정·소비 loop `15 Hz`, GPS `5 Hz`,",
            "  IMU `50 Hz`다. local 실측은 VLP16 완성 scan 약 `12 Hz`, IMU 약 `42~44 Hz`였으며",
            "  camera fused BEV 상한은 `10 Hz`다.",
            "",
            "## 선택 모델",
            "",
            "| ID | 선택 | 크기 | SHA-256 | 경로 |",
            "|---|:---:|---:|---|---|",
        ]
    )
    for item in models["items"]:
        lines.append(
            f"| `{item['id']}` | {'yes' if item['selected'] else 'no'} | {item['size_bytes']} | "
            f"`{item['sha256']}` | `{item['path']}` |"
        )
    lines.extend(
        [
            "",
            "현재 모델 파일은 모두 tracked돼 있지만 목표 정책은 실행 image 밖 immutable artifact + SHA manifest다.",
            "경로만 바꾸거나 같은 이름의 weight를 덮어쓰지 않는다.",
            "",
            "## 반드시 해결할 통합 위험",
            "",
        ]
    )
    for risk in manifest["known_integration_risks"]:
        lines.append(
            f"- **{risk['severity']} · `{risk['id']}`:** {risk['fact']}  "
            f"조치: {risk['p4_action']}"
        )
    lines.extend(
        [
            "",
            "특히 LiDAR `Main`의 구 `link_node` road mask는 완성된 `/hd_map/*`와 다른 지도 계약이다.",
            "P4-A에서는 사실을 기록만 했고, 필터 결과 parity 없이 경로를 즉시 바꾸지 않았다.",
            "",
            "## Oracle 금지선",
            "",
        ]
    )
    for item in manifest["oracle_policy"]["deployment_forbidden_inputs"]:
        lines.append(f"- {item}")
    lines.extend(
        [
            "",
            "이 입력들은 `privileged_gt=true`, `deployment_allowed=false`인 offline profile에서만 사용할 수 있다.",
            "기록 profile이 있다는 이유로 본선 runtime 입력으로 승격되지 않는다.",
            "",
            "## P4 완료 gate",
            "",
        ]
    )
    for gate in manifest["migration_gates"]:
        lines.append(f"- {gate}")
    lines.extend(
        [
            "",
            "## 재생성·검증",
            "",
            "```bash",
            "python3 tools/audit/build_perception_runtime_baseline.py --check",
            "python3 -m unittest tools/audit/test_perception_runtime_baseline.py",
            "```",
            "",
            "동일 source에서 두 번 생성한 결과는 byte-identical해야 한다. package/topic/model/bag/message 기준이",
            "누락되거나 선택 모델 SHA가 달라지면 검증은 실패한다.",
            "",
            "## 다음 작업",
            "",
            "P4-B에서 먼저 canonical `asmc_perception` façade와 compatibility launch를 만들고, traffic-light typed",
            "출력 및 LiDAR tracked-object의 `asmc_msgs` 이관 계약을 additive하게 결정한다. 기존 source의 물리 이동과",
            "삭제는 그 뒤 bag replay parity를 통과한 단위별 PR에서 수행한다.",
            "",
        ]
    )
    return "\n".join(lines)


def _diff(expected: str, actual: str, path: Path) -> str:
    return "".join(
        difflib.unified_diff(
            actual.splitlines(True),
            expected.splitlines(True),
            fromfile=str(path),
            tofile=f"generated:{path}",
        )
    )


def write_or_check(repo_root: Path, write: bool, check: bool) -> None:
    manifest = build_manifest(repo_root)
    outputs = {
        repo_root / MANIFEST_REL: dump_manifest(manifest),
        repo_root / REPORT_REL: render_report(manifest),
    }
    if write:
        for path, content in outputs.items():
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content, encoding="utf-8")
            print(f"wrote {path.relative_to(repo_root)}")
    if check:
        differences = []
        for path, expected in outputs.items():
            actual = path.read_text(encoding="utf-8") if path.is_file() else ""
            if actual != expected:
                differences.append(_diff(expected, actual, path.relative_to(repo_root)))
        if differences:
            raise BaselineError("P4-A baseline drift:\n" + "\n".join(differences))
        print(f"PASS: {BASELINE_ID} manifest and report are deterministic")
    if not write and not check:
        sys.stdout.write(outputs[repo_root / MANIFEST_REL])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path(__file__).resolve().parents[2],
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--write", action="store_true")
    mode.add_argument("--check", action="store_true")
    args = parser.parse_args()
    try:
        legacy = args.repo_root / "src/lidar+sensorfusion/src/Main/src/Camera/YOLO/YOLODetection.py"
        if not legacy.is_file():
            raise BaselineError("P4-A is a frozen historical inventory. Use the source/generator revision recorded in its baseline report; current models and commands are documented in asmc_camera/README.md.")
        write_or_check(args.repo_root, args.write, args.check)
    except (BaselineError, OSError, ValueError, yaml.YAMLError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
