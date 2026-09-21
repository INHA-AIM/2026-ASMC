#!/usr/bin/env python3
"""Build the deterministic P1-A runtime baseline manifest and report.

The audit is deliberately read-only with respect to ROS packages, sensor/network
configuration, and HD-map geometry.  It uses only the Python standard library
plus PyYAML and contains a small ROS1 bag-v2 index reader so the baseline can be
regenerated on a host where the ROS CLI is not installed.
"""

from __future__ import annotations

import argparse
import difflib
import hashlib
import json
import math
import os
import re
import struct
import sys
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, MutableMapping, Optional, Sequence, Tuple

import yaml


SCHEMA_VERSION = 1
BASELINE_ID = "runtime_baseline_v1"

HD_MAP_REL = Path(
    "src/asmc_hd_map/reports/kcity2025_route_curation_20260917/"
    "route_curated_candidate_map.json"
)
HD_MAP_SUMMARY_REL = Path(
    "src/asmc_hd_map/reports/kcity2025_route_curation_20260917/summary.json"
)
SENSOR_REL = Path("config/ASMC_sensor_set_v2.json")
CHECKPOINT_REL = Path("R_KR_PG_K-city_2025/2026_molit_comp_checkpoints.json")
GLOBAL_PATH_REL = Path("R_KR_PG_K-city_2025/2026_molit_comp_global_path.txt")
COMPETITION_METADATA_REL = Path(
    "src/asmc_hd_map/config/kcity2025_competition_metadata.yaml"
)
AUTONAV_SENSOR_REL = Path(
    "On-site inspection/MoraiLauncher_Win_S/MoraiLauncher_Win_Data/SaveFile/"
    "Sensor/25.S4.MolitComp03/Autonav_sensor.json"
)
RAW_BAG_REL = Path(
    "rosbags/raw/20260913_195606_kcity2025_shadow_static_avoid_return_sunny_1pm_wheel/"
    "bags/20260913_195606_kcity2025_shadow_static_avoid_return_sunny_1pm_wheel_0.bag"
)
RESULT_BAG_REL = Path(
    "rosbags/results/20260913_195606_kcity2025_shadow_static_avoid_return_sunny_1pm_wheel/"
    "twinlite_v6_shadow_boundary_family_candidate_v3_profile_v1_20260915/bags/"
    "twinlite_v6_shadow_boundary_family_candidate_v3_profile_v1_20260915_0.bag"
)

EXPECTED_MAP_COUNTS = {
    "lane_marking": 121,
    "stopline": 7,
    "white": 94,
    "yellow": 27,
    "solid": 90,
    "broken": 31,
}

TEXT_SUFFIXES = {
    ".bash", ".cc", ".cfg", ".cmake", ".cpp", ".cxx", ".h", ".hpp",
    ".json", ".launch", ".md", ".msg", ".py", ".sh", ".txt", ".xml",
    ".yaml", ".yml",
}

LEGACY_TARGETS = {
    "learning": "src/learning/",
    "learning_by_cheating": "src/learning_by_cheating/",
    "link_node": "src/asmc_hd_map/development/won_tae_link_node/",
    "morai_udp_bridge": "src/morai_udp_bridge/",
    "morai_sensor_bridge": "src/morai_sensor_bridge/",
    "vehicle_state": "src/vehicle_state/",
    "camera_semantic_perception": "src/camera_semantic_perception/",
    "drivable_bev": "src/perception/drivable_bev/",
    "traffic_light": "src/traffic_light/",
    "data_collection": "src/data_collection/",
    "duplicate_msg": "src/lidar+sensorfusion/src/Msg/",
    "duplicate_morai_msgs": "src/lidar+sensorfusion/src/morai_msgs/",
}


class BaselineError(RuntimeError):
    """A required baseline input or invariant is missing."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def require_file(path: Path, label: str) -> Path:
    if not path.is_file():
        raise BaselineError(f"missing {label}: {path}")
    return path


def repo_relative(path: Path, repo_root: Path) -> str:
    # Preserve the repository entry rather than following symlinks. Catkin
    # workspaces may contain a tracked CMakeLists.txt symlink into /opt/ros;
    # resolving it would both escape the repository and make the manifest
    # depend on the host ROS installation.
    return path.absolute().relative_to(repo_root.absolute()).as_posix()


def data_relative(path: Path, data_root: Path) -> str:
    return path.resolve().relative_to(data_root.resolve()).as_posix()


def read_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def stable_float(value: float, digits: int = 6) -> float:
    return round(float(value), digits)


def _package_dependencies(root: ET.Element, tag: str) -> List[str]:
    return sorted({node.text.strip() for node in root.findall(tag) if node.text})


def audit_packages(repo_root: Path) -> Dict[str, Any]:
    packages: List[Dict[str, Any]] = []
    for package_xml in sorted((repo_root / "src").rglob("package.xml")):
        root = ET.parse(package_xml).getroot()
        name = (root.findtext("name") or "").strip()
        version = (root.findtext("version") or "").strip()
        if not name:
            raise BaselineError(f"package without a name: {package_xml}")
        dependencies = {
            "buildtool": _package_dependencies(root, "buildtool_depend"),
            "build": _package_dependencies(root, "build_depend"),
            "build_export": _package_dependencies(root, "build_export_depend"),
            "exec": _package_dependencies(root, "exec_depend"),
            "test": _package_dependencies(root, "test_depend"),
            "generic": _package_dependencies(root, "depend"),
        }
        packages.append(
            {
                "name": name,
                "path": repo_relative(package_xml.parent, repo_root),
                "version": version,
                "dependencies": dependencies,
                "package_xml_sha256": sha256_file(package_xml),
            }
        )

    package_names = {item["name"] for item in packages}
    edges = []
    for item in packages:
        all_dependencies = sorted(
            {
                dependency
                for values in item["dependencies"].values()
                for dependency in values
                if dependency in package_names
            }
        )
        for dependency in all_dependencies:
            edges.append({"from": item["name"], "to": dependency})
    edges.sort(key=lambda item: (item["from"], item["to"]))
    return {"count": len(packages), "items": packages, "internal_dependency_edges": edges}


def _nearest_package(path: Path, repo_root: Path) -> Optional[Tuple[str, Path]]:
    current = path.parent
    src_root = repo_root / "src"
    while current != current.parent and src_root in (current, *current.parents):
        package_xml = current / "package.xml"
        if package_xml.is_file():
            root = ET.parse(package_xml).getroot()
            name = (root.findtext("name") or "").strip()
            return name, current
        if current == src_root:
            break
        current = current.parent
    return None


def audit_launches(repo_root: Path) -> Dict[str, Any]:
    launches: List[Dict[str, Any]] = []
    node_count = 0
    for launch_path in sorted((repo_root / "src").rglob("*.launch")):
        try:
            root = ET.parse(launch_path).getroot()
        except ET.ParseError as exc:
            raise BaselineError(f"invalid launch XML {launch_path}: {exc}") from exc
        nodes = []
        for node in root.iter("node"):
            remaps = sorted(
                [
                    {"from": remap.get("from", ""), "to": remap.get("to", "")}
                    for remap in node.findall("remap")
                ],
                key=lambda item: (item["from"], item["to"]),
            )
            nodes.append(
                {
                    "pkg": node.get("pkg", ""),
                    "type": node.get("type", ""),
                    "name": node.get("name", ""),
                    "remaps": remaps,
                }
            )
        nodes.sort(key=lambda item: (item["pkg"], item["type"], item["name"]))
        includes = sorted(
            include.get("file", "") for include in root.iter("include") if include.get("file")
        )
        node_count += len(nodes)
        launches.append(
            {
                "path": repo_relative(launch_path, repo_root),
                "nodes": nodes,
                "includes": includes,
            }
        )
    return {"file_count": len(launches), "node_declaration_count": node_count, "files": launches}


def audit_executables(repo_root: Path, launches: Mapping[str, Any]) -> Dict[str, Any]:
    targets: List[Dict[str, str]] = []
    for cmake_path in sorted((repo_root / "src").rglob("CMakeLists.txt")):
        package = _nearest_package(cmake_path, repo_root)
        if package is None:
            continue
        package_name, _ = package
        text = cmake_path.read_text(encoding="utf-8", errors="replace")
        for match in re.finditer(r"\badd_executable\s*\(\s*([^\s\)]+)", text, re.MULTILINE):
            targets.append(
                {
                    "package": package_name,
                    "kind": "cmake_add_executable",
                    "target": match.group(1),
                    "source": repo_relative(cmake_path, repo_root),
                }
            )
        for match in re.finditer(
            r"\bcatkin_install_python\s*\(\s*PROGRAMS(.*?)DESTINATION",
            text,
            re.DOTALL,
        ):
            block = re.sub(r"#[^\n]*", "", match.group(1))
            for token in re.split(r"\s+", block.strip()):
                if token and not token.startswith("${"):
                    targets.append(
                        {
                            "package": package_name,
                            "kind": "catkin_install_python",
                            "target": token,
                            "source": repo_relative(cmake_path, repo_root),
                        }
                    )
    for launch in launches["files"]:
        for node in launch["nodes"]:
            targets.append(
                {
                    "package": node["pkg"],
                    "kind": "launch_node",
                    "target": node["type"],
                    "source": launch["path"],
                }
            )
    unique = {
        (item["package"], item["kind"], item["target"], item["source"]): item
        for item in targets
    }
    items = sorted(
        unique.values(),
        key=lambda item: (item["package"], item["target"], item["kind"], item["source"]),
    )
    return {"declaration_count": len(items), "items": items}


TOPIC_PATTERNS = (
    ("publish", "cpp", re.compile(r"\badvertise\s*<\s*([^>]+)>\s*\(\s*[\"']([^\"']+)[\"']")),
    ("subscribe", "cpp", re.compile(r"\bsubscribe\s*<\s*([^>]+)>\s*\(\s*[\"']([^\"']+)[\"']")),
    ("publish", "python", re.compile(r"rospy\.Publisher\s*\(\s*[\"']([^\"']+)[\"']\s*,\s*([A-Za-z0-9_\.]+)")),
    ("subscribe", "python", re.compile(r"rospy\.Subscriber\s*\(\s*[\"']([^\"']+)[\"']\s*,\s*([A-Za-z0-9_\.]+)")),
)


def audit_static_topics(repo_root: Path, launches: Mapping[str, Any]) -> Dict[str, Any]:
    references: List[Dict[str, Any]] = []
    source_suffixes = {".cc", ".cpp", ".cxx", ".h", ".hpp", ".py"}
    for path in sorted((repo_root / "src").rglob("*")):
        if not path.is_file() or path.suffix not in source_suffixes:
            continue
        if any(part in {"build", "devel"} for part in path.parts):
            continue
        text = path.read_text(encoding="utf-8", errors="replace")
        for direction, language, pattern in TOPIC_PATTERNS:
            for match in pattern.finditer(text):
                if language == "python":
                    topic, message_type = match.group(1), match.group(2)
                else:
                    message_type, topic = match.group(1), match.group(2)
                references.append(
                    {
                        "direction": direction,
                        "topic": topic,
                        "message_type_literal": message_type.strip(),
                        "source": repo_relative(path, repo_root),
                        "line": text.count("\n", 0, match.start()) + 1,
                    }
                )
    for launch in launches["files"]:
        for node in launch["nodes"]:
            for remap in node["remaps"]:
                references.append(
                    {
                        "direction": "remap",
                        "topic": remap["to"],
                        "message_type_literal": "",
                        "source": launch["path"],
                        "line": None,
                        "from": remap["from"],
                    }
                )
    references.sort(
        key=lambda item: (
            item["topic"], item["direction"], item["source"], item.get("line") or 0
        )
    )
    return {
        "method": "literal advertise/subscribe calls and launch remaps; runtime topics come from bags",
        "reference_count": len(references),
        "references": references,
    }


def _find_generated_header(repo_root: Path, package: str, message: str) -> Optional[Path]:
    preferred = repo_root / "devel" / "include" / package / f"{message}.h"
    if preferred.is_file():
        return preferred
    candidates = sorted(
        path
        for path in (repo_root / "src").glob(f"**/devel/include/{package}/{message}.h")
        if path.is_file()
    )
    return candidates[0] if candidates else None


def _extract_generated_md5(header: Path, package: str, message: str) -> str:
    text = header.read_text(encoding="utf-8", errors="replace")
    marker = re.search(
        rf"struct\s+MD5Sum<\s*::{re.escape(package)}::{re.escape(message)}_.*?"
        rf"return\s+\"([0-9a-f]{{32}})\";",
        text,
        re.DOTALL,
    )
    if not marker:
        raise BaselineError(f"cannot extract ROS MD5 for {package}/{message} from {header}")
    return marker.group(1)


def audit_messages(repo_root: Path) -> Dict[str, Any]:
    excluded = ("src/MORAI-ROS_morai_msgs/", "src/lidar+sensorfusion/src/morai_msgs/")
    messages: List[Dict[str, Any]] = []
    asmc_specs = None
    asmc_md5_cache: Dict[str, str] = {}
    for msg_path in sorted((repo_root / "src").rglob("*.msg")):
        relative = repo_relative(msg_path, repo_root)
        if relative.startswith(excluded):
            continue
        package = _nearest_package(msg_path, repo_root)
        if package is None:
            raise BaselineError(f"message has no containing package: {relative}")
        package_name, _ = package
        header = _find_generated_header(repo_root, package_name, msg_path.stem)
        if header is not None:
            ros_md5 = _extract_generated_md5(header, package_name, msg_path.stem)
        elif package_name == "asmc_msgs":
            # P1-B must be auditable before a Noetic workspace is generated.
            # The source calculator is independently regression-tested against
            # every P1-A generated-header MD5 in validate_p1b_contract.py.
            from validate_p1b_contract import load_message_specs, ros1_md5

            if asmc_specs is None:
                asmc_specs = load_message_specs(repo_root / "src/asmc_msgs/msg")
            ros_md5 = ros1_md5(msg_path.stem, asmc_specs, asmc_md5_cache)
        else:
            raise BaselineError(
                f"generated header missing for {package_name}/{msg_path.stem}; "
                "build the workspace before regenerating the baseline"
            )
        messages.append(
            {
                "type": f"{package_name}/{msg_path.stem}",
                "source": relative,
                "source_sha256": sha256_file(msg_path),
                "ros_md5": ros_md5,
            }
        )
    messages.sort(key=lambda item: item["type"])
    return {
        "scope": "team/runtime messages; pinned and duplicate morai_msgs trees excluded",
        "md5_method": "generated headers; ROS1 source calculation fallback for unbuilt asmc_msgs",
        "count": len(messages),
        "items": messages,
    }


def _read_u32(stream: Any) -> int:
    payload = stream.read(4)
    if len(payload) != 4:
        raise BaselineError("truncated ROS bag record")
    return struct.unpack("<I", payload)[0]


def _parse_bag_fields(payload: bytes) -> Dict[str, bytes]:
    fields: Dict[str, bytes] = {}
    offset = 0
    while offset < len(payload):
        if offset + 4 > len(payload):
            raise BaselineError("truncated ROS bag field length")
        size = struct.unpack_from("<I", payload, offset)[0]
        offset += 4
        field = payload[offset : offset + size]
        if len(field) != size:
            raise BaselineError("truncated ROS bag field")
        offset += size
        key, separator, value = field.partition(b"=")
        if not separator:
            raise BaselineError("invalid ROS bag field")
        fields[key.decode("ascii")] = value
    return fields


def _bag_u32(value: bytes) -> int:
    if len(value) != 4:
        raise BaselineError("invalid ROS bag uint32")
    return struct.unpack("<I", value)[0]


def _bag_u64(value: bytes) -> int:
    if len(value) != 8:
        raise BaselineError("invalid ROS bag uint64")
    return struct.unpack("<Q", value)[0]


def _bag_time(value: bytes) -> float:
    if len(value) != 8:
        raise BaselineError("invalid ROS bag time")
    seconds, nanoseconds = struct.unpack("<II", value)
    return seconds + nanoseconds / 1_000_000_000.0


def read_ros1_bag_index(path: Path, data_root: Path) -> Dict[str, Any]:
    connections: Dict[int, Dict[str, str]] = {}
    counts: MutableMapping[int, int] = defaultdict(int)
    starts: List[float] = []
    ends: List[float] = []
    with path.open("rb") as stream:
        if stream.read(13) != b"#ROSBAG V2.0\n":
            raise BaselineError(f"not a ROS1 bag v2 file: {path}")
        header_length = _read_u32(stream)
        bag_header = _parse_bag_fields(stream.read(header_length))
        data_length = _read_u32(stream)
        stream.seek(data_length, os.SEEK_CUR)
        if bag_header.get("op") != b"\x03":
            raise BaselineError(f"missing ROS bag header record: {path}")
        index_position = _bag_u64(bag_header["index_pos"])
        if index_position <= 0:
            raise BaselineError(f"unindexed ROS bag: {path}")
        stream.seek(index_position)
        while True:
            length_payload = stream.read(4)
            if not length_payload:
                break
            if len(length_payload) != 4:
                raise BaselineError(f"truncated ROS bag index: {path}")
            header_length = struct.unpack("<I", length_payload)[0]
            header = _parse_bag_fields(stream.read(header_length))
            record_data_length = _read_u32(stream)
            record_data = stream.read(record_data_length)
            if len(record_data) != record_data_length:
                raise BaselineError(f"truncated ROS bag index data: {path}")
            operation = header.get("op", b"\x00")[0]
            if operation == 0x07:  # CONNECTION
                connection_id = _bag_u32(header["conn"])
                connection_header = _parse_bag_fields(record_data)
                connections[connection_id] = {
                    "topic": header["topic"].decode("utf-8"),
                    "type": connection_header.get("type", b"").decode("utf-8"),
                    "md5": connection_header.get("md5sum", b"").decode("ascii"),
                }
            elif operation == 0x06:  # CHUNK_INFO
                starts.append(_bag_time(header["start_time"]))
                ends.append(_bag_time(header["end_time"]))
                if len(record_data) % 8:
                    raise BaselineError(f"invalid ROS bag chunk-info counts: {path}")
                for offset in range(0, len(record_data), 8):
                    connection_id, count = struct.unpack_from("<II", record_data, offset)
                    counts[connection_id] += count
    if not connections or not starts or not ends:
        raise BaselineError(f"ROS bag has no indexed connections/chunks: {path}")
    duration = max(ends) - min(starts)
    if duration <= 0:
        raise BaselineError(f"ROS bag duration is not positive: {path}")

    grouped: MutableMapping[Tuple[str, str, str], int] = defaultdict(int)
    for connection_id, metadata in connections.items():
        grouped[(metadata["topic"], metadata["type"], metadata["md5"])] += counts.get(
            connection_id, 0
        )
    topics = []
    for (topic, message_type, md5), message_count in sorted(grouped.items()):
        topics.append(
            {
                "topic": topic,
                "type": message_type,
                "md5": md5,
                "messages": message_count,
                "average_hz": stable_float(message_count / duration),
            }
        )
    return {
        "path": data_relative(path, data_root),
        "bytes": path.stat().st_size,
        "sha256": sha256_file(path),
        "duration_s": stable_float(duration),
        "message_count": sum(item["messages"] for item in topics),
        "topics": topics,
    }


def audit_bags(data_root: Path) -> Dict[str, Any]:
    raw_path = require_file(data_root / RAW_BAG_REL, "representative raw bag")
    result_path = require_file(data_root / RESULT_BAG_REL, "representative result bag")
    return {
        "scenario_pairing": "same 20260913_195606 shadow_static_avoid_return run",
        "reader": "built-in ROS1 bag-v2 index reader",
        "raw": read_ros1_bag_index(raw_path, data_root),
        "result": read_ros1_bag_index(result_path, data_root),
    }


def _semantic_differences(left: Any, right: Any, prefix: str = "$") -> List[Dict[str, Any]]:
    differences: List[Dict[str, Any]] = []
    if type(left) is not type(right):
        return [{"path": prefix, "v2": left, "submitted_autonav": right}]
    if isinstance(left, dict):
        for key in sorted(set(left) | set(right)):
            child = f"{prefix}.{key}"
            if key not in left:
                differences.append({"path": child, "v2": None, "submitted_autonav": right[key]})
            elif key not in right:
                differences.append({"path": child, "v2": left[key], "submitted_autonav": None})
            else:
                differences.extend(_semantic_differences(left[key], right[key], child))
    elif isinstance(left, list):
        if len(left) != len(right):
            differences.append(
                {"path": f"{prefix}.length", "v2": len(left), "submitted_autonav": len(right)}
            )
        for index, (left_item, right_item) in enumerate(zip(left, right)):
            differences.extend(
                _semantic_differences(left_item, right_item, f"{prefix}[{index}]")
            )
    elif left != right:
        differences.append({"path": prefix, "v2": left, "submitted_autonav": right})
    return differences


def _sensor_entry(
    item: Mapping[str, Any], config_key: str, label: str, ordinal: int
) -> Dict[str, Any]:
    config = item[config_key]
    udp = config["udpConfig"]
    period = float(config["sensorPeriod"])
    entry = {
        "id": int(item["m_SensorUniqueID"]),
        "label": f"{label}_{ordinal}",
        "position_m": {axis: float(item["pos"][axis]) for axis in ("x", "y", "z")},
        "rotation_deg": {
            axis: float(item["rot"][axis]) for axis in ("roll", "pitch", "yaw")
        },
        "period_s": stable_float(period, 9),
        "nominal_hz": stable_float(1.0 / period),
        "udp": {
            "host_ip": udp["hostIP"],
            "host_port": int(udp["hostPort"]),
            "destination_ip": udp["destinationIP"],
            "destination_port": int(udp["destinationPort"]),
        },
    }
    if label == "lidar3d":
        entry["model"] = "VLP16" if int(config.get("lidar3DType", 0)) == 0 else str(
            config.get("lidar3DType")
        )
        entry["frequency_index"] = int(config.get("frequencyIndex", -1))
    return entry


def audit_sensors(repo_root: Path, data_root: Path) -> Dict[str, Any]:
    sensor_path = require_file(repo_root / SENSOR_REL, "Autonav v2 sensor set")
    submitted_path = require_file(
        data_root / AUTONAV_SENSOR_REL, "submitted Autonav sensor set"
    )
    sensor_json = read_json(sensor_path)
    submitted_json = read_json(submitted_path)
    inventory: List[Dict[str, Any]] = []
    groups = (
        ("cameraList", "cc", "camera"),
        ("Lidar3DList", "lc", "lidar3d"),
        ("GPSList", "gc", "gps"),
        ("IMUList", "ic", "imu"),
    )
    for list_key, config_key, label in groups:
        for ordinal, item in enumerate(sensor_json.get(list_key, []), start=1):
            inventory.append(_sensor_entry(item, config_key, label, ordinal))
    inventory.sort(key=lambda item: item["id"])
    lidar_entries = [item for item in inventory if item["label"].startswith("lidar3d_")]
    if len(lidar_entries) != 1:
        raise BaselineError(f"expected exactly one v2 3D LiDAR, found {len(lidar_entries)}")
    current_lidar_hz = lidar_entries[0]["nominal_hz"]
    if not math.isclose(current_lidar_hz, 10.0, abs_tol=0.001):
        raise BaselineError(f"expected current v2 LiDAR baseline near 10 Hz, got {current_lidar_hz}")
    differences = _semantic_differences(sensor_json, submitted_json)
    expected_difference_path = "$.cameraList[3].rot.pitch"
    if [item["path"] for item in differences] != [expected_difference_path]:
        raise BaselineError(
            "v2/submitted Autonav semantic differences changed: "
            + ", ".join(item["path"] for item in differences)
        )
    return {
        "canonical_v2": {
            "path": SENSOR_REL.as_posix(),
            "bytes": sensor_path.stat().st_size,
            "sha256": sha256_file(sensor_path),
            "sensor_count": len(inventory),
            "inventory": inventory,
        },
        "submitted_autonav_reference": {
            "path": AUTONAV_SENSOR_REL.as_posix(),
            "bytes": submitted_path.stat().st_size,
            "sha256": sha256_file(submitted_path),
            "role": "provenance_only_not_runtime_source",
            "semantic_differences_from_v2": differences,
        },
        "lidar_rate_contract": {
            "baseline_hz": current_lidar_hz,
            "approved_future_target_hz": 15.0,
            "owner_requirement": "Son Jae-ho / LiDAR",
            "change_stage": 3,
            "status": "recorded_not_applied",
            "verification": "MORAI UI export plus >=30 s observed /velodyne_points at 14-16 Hz",
        },
    }


def audit_hd_map(repo_root: Path) -> Dict[str, Any]:
    map_path = require_file(repo_root / HD_MAP_REL, "stage14 HD map")
    summary_path = require_file(repo_root / HD_MAP_SUMMARY_REL, "stage14 HD map summary")
    map_json = read_json(map_path)
    summary = read_json(summary_path)
    features = map_json.get("features", [])
    feature_types = Counter(feature.get("feature_type") for feature in features)
    colors = Counter()
    patterns = Counter()
    for feature in features:
        if feature.get("feature_type") != "lane_marking":
            continue
        attributes = feature.get("attributes", {})
        colors[attributes.get("color")] += 1
        patterns[attributes.get("pattern")] += 1
    actual = {
        "lane_marking": feature_types["lane_marking"],
        "stopline": feature_types["stopline"],
        "white": colors["white"],
        "yellow": colors["yellow"],
        "solid": patterns["solid"],
        "broken": patterns["broken"],
    }
    if actual != EXPECTED_MAP_COUNTS:
        raise BaselineError(f"stage14 HD-map counts changed: expected {EXPECTED_MAP_COUNTS}, got {actual}")
    if summary.get("selected_marking_count") != EXPECTED_MAP_COUNTS["lane_marking"]:
        raise BaselineError("stage14 summary marking count disagrees with geometry")
    if summary.get("selected_stopline_count") != EXPECTED_MAP_COUNTS["stopline"]:
        raise BaselineError("stage14 summary stopline count disagrees with geometry")

    geometry_payload = [
        {
            "id": feature.get("id"),
            "feature_type": feature.get("feature_type"),
            "frame_id": feature.get("frame_id"),
            "geometry": feature.get("geometry"),
        }
        for feature in features
    ]
    geometry_payload.sort(key=lambda item: (str(item["feature_type"]), str(item["id"])))
    build = map_json.get("build", {})
    return {
        "path": HD_MAP_REL.as_posix(),
        "summary_path": HD_MAP_SUMMARY_REL.as_posix(),
        "map_id": map_json.get("map_id"),
        "build_id": build.get("build_id"),
        "frame_id": map_json.get("frame"),
        "schema_version": map_json.get("schema_version"),
        "operational": bool(build.get("operational", False)),
        "partial_candidate": bool(build.get("partial_candidate", False)),
        "feature_counts": dict(sorted(feature_types.items())),
        "lane_marking_colors": dict(sorted(colors.items())),
        "lane_marking_patterns": dict(sorted(patterns.items())),
        "file_sha256": sha256_file(map_path),
        "geometry_sha256": canonical_sha256(geometry_payload),
        "summary_sha256": sha256_file(summary_path),
        "validation": {
            "expected": EXPECTED_MAP_COUNTS,
            "actual": actual,
            "passed": True,
        },
    }


def audit_competition_inputs(
    data_root: Path, repo_root: Optional[Path] = None
) -> Dict[str, Any]:
    checkpoint_path = require_file(data_root / CHECKPOINT_REL, "official checkpoint JSON")
    global_path = require_file(data_root / GLOBAL_PATH_REL, "official global path")
    checkpoints = read_json(checkpoint_path)
    checkpoint_items = checkpoints.get("checkpoints", [])
    if len(checkpoint_items) != 16:
        raise BaselineError(f"expected 16 official checkpoints, got {len(checkpoint_items)}")
    if float(checkpoints.get("pass_radius_m", -1.0)) != 3.0:
        raise BaselineError("official checkpoint radius is not 3 m")
    points: List[Tuple[float, float, float]] = []
    with global_path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            parts = line.split()
            if not parts:
                continue
            if len(parts) != 3:
                raise BaselineError(f"invalid global path line {line_number}")
            points.append(tuple(float(value) for value in parts))
    if len(points) < 2:
        raise BaselineError("global path has fewer than two points")
    route_length_xy = sum(
        math.hypot(right[0] - left[0], right[1] - left[1])
        for left, right in zip(points, points[1:])
    )
    result = {
        "checkpoints": {
            "path": CHECKPOINT_REL.as_posix(),
            "sha256": sha256_file(checkpoint_path),
            "count": len(checkpoint_items),
            "pass_radius_m": 3.0,
            "yaw_unit": checkpoints.get("yaw_unit"),
        },
        "global_path": {
            "path": GLOBAL_PATH_REL.as_posix(),
            "sha256": sha256_file(global_path),
            "point_count": len(points),
            "length_xy_m": stable_float(route_length_xy),
            "start_success_5_percent_station_m": stable_float(route_length_xy * 0.05),
        },
        "status": "confirmed_inputs_for_P1-C",
    }
    if repo_root is not None:
        metadata_path = require_file(
            repo_root / COMPETITION_METADATA_REL, "P1-C competition metadata bundle"
        )
        metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
        if metadata.get("schema_version") != "asmc.competition_metadata/v1":
            raise BaselineError("unexpected P1-C competition metadata schema")
        metadata_validation = metadata.get("validation") or {}
        if metadata_validation.get("passed") is not True:
            raise BaselineError("P1-C competition metadata validation did not pass")
        if len(metadata.get("checkpoints") or []) != 16:
            raise BaselineError("P1-C competition metadata must contain 16 checkpoints")
        statuses = Counter(
            str(item.get("status")) for item in (metadata.get("mission_zones") or [])
        )
        if set(statuses) - {"official", "derived_official", "provisional"}:
            raise BaselineError("P1-C mission metadata contains an invalid authority status")
        result["metadata_bundle"] = {
            "path": COMPETITION_METADATA_REL.as_posix(),
            "sha256": sha256_file(metadata_path),
            "schema_version": metadata.get("schema_version"),
            "checkpoint_count": len(metadata.get("checkpoints") or []),
            "mission_zone_count": len(metadata.get("mission_zones") or []),
            "mission_status_counts": dict(sorted(statuses.items())),
            "validation_passed": True,
            "published": False,
        }
        result["status"] = "P1-C_metadata_frozen_not_yet_published"
    return result


def _script_category(path: Path) -> str:
    name = path.name.lower()
    if "docker" in name:
        return "container"
    if any(token in name for token in ("train", "evaluate", "dataset", "bake", "curate", "analyze")):
        return "train_eval_data"
    if any(token in name for token in ("diagnose", "verify", "probe", "wait", "smoke", "audit")):
        return "diagnostic"
    return "runtime_or_build"


def audit_scripts(repo_root: Path) -> Dict[str, Any]:
    scripts = []
    for path in sorted((repo_root / "scripts").iterdir()):
        if not path.is_file() or path.suffix == ".pyc":
            continue
        scripts.append(
            {
                "path": repo_relative(path, repo_root),
                "category": _script_category(path),
                "executable": bool(path.stat().st_mode & 0o111),
                "sha256": sha256_file(path),
            }
        )
    return {
        "count": len(scripts),
        "category_counts": dict(sorted(Counter(item["category"] for item in scripts).items())),
        "items": scripts,
    }


def _candidate_reference_files(repo_root: Path) -> Iterable[Path]:
    excluded_parts = {".git", "build", "devel", "__pycache__"}
    excluded_prefixes = (
        "src/asmc_hd_map/reports/",
        "config/baselines/",
        "docs/baselines/",
        "tools/audit/",
    )
    for root_name in ("docs", "scripts", "src", "tools"):
        root = repo_root / root_name
        if not root.exists():
            continue
        for path in sorted(root.rglob("*")):
            # External/toolchain symlink contents are not repository source and
            # would make this audit host-dependent.
            if path.is_symlink():
                continue
            if not path.is_file() or path.suffix.lower() not in TEXT_SUFFIXES:
                continue
            relative = repo_relative(path, repo_root)
            if relative.startswith(excluded_prefixes):
                continue
            if excluded_parts.intersection(path.parts):
                continue
            if path.stat().st_size > 2 * 1024 * 1024:
                continue
            yield path


def audit_legacy_references(repo_root: Path) -> Dict[str, Any]:
    matches: Dict[str, List[str]] = {name: [] for name in LEGACY_TARGETS}
    for path in _candidate_reference_files(repo_root):
        text = path.read_text(encoding="utf-8", errors="replace")
        relative = repo_relative(path, repo_root)
        for name, needle in LEGACY_TARGETS.items():
            if needle in text or (repo_root / needle).resolve() == path.resolve():
                matches[name].append(relative)
    items = []
    for name, target in LEGACY_TARGETS.items():
        files = sorted(set(matches[name]))
        items.append(
            {"name": name, "target": target, "reference_file_count": len(files), "files": files}
        )
    return {"scan_scope": "repository text excluding generated reports and large files", "items": items}


def future_requirements() -> List[Dict[str, Any]]:
    return [
        {
            "id": "lidar_15hz",
            "stage": 3,
            "status": "approved_not_applied",
            "requirement": "Change the Autonav VLP16 from the 10 Hz baseline to 15 Hz.",
            "owner": "Son Jae-ho / LiDAR",
            "acceptance": "MORAI UI-exported config; >=30 s observed /velodyne_points average in 14-16 Hz plus tracking/resource comparison.",
        },
        {
            "id": "field_aligned_udp_ports",
            "stage": 3,
            "status": "approved_not_applied",
            "requirement": "Use field ports in both environments: Ctrl 9093/9094 mode 2, Competition Vehicle Status 7603/7502, Collision 9091/9092, v2 sensor ports 9290-9303.",
            "acceptance": "Only IP differs between dev_local and competition_lan; packet replay and profile validation pass.",
        },
        {
            "id": "package_consolidation",
            "stage": "2-5",
            "status": "planned",
            "requirement": "Converge team-owned runtime code on asmc_msgs, asmc_hd_map, asmc_bridge, and asmc_perception without moving planning-owned packages.",
            "acceptance": "Each legacy deletion gate has replacement parity and zero live references.",
        },
        {
            "id": "field_rehearsal",
            "stage": 6,
            "status": "planned",
            "requirement": "Run native-Linux Docker LAN2LAN rehearsal on two hosts before deployment freeze.",
            "acceptance": "15 minute full-stack run, offline recovery, packet/resource evidence, and rollback runbook.",
        },
    ]


def build_manifest(repo_root: Path, data_root: Path) -> Dict[str, Any]:
    launches = audit_launches(repo_root)
    manifest = {
        "schema_version": SCHEMA_VERSION,
        "baseline_id": BASELINE_ID,
        "purpose": "P1-A package/topic/message/sensor/bag/HD-map regression baseline",
        "determinism": {
            "timestamps_omitted": True,
            "paths": "repository-relative or ASMC_DATA-relative",
            "large_artifacts": "referenced by size and SHA-256; never copied",
        },
        "protected_scope": {
            "package_moves": False,
            "sensor_config_changes": False,
            "network_config_changes": False,
            "hd_map_geometry_changes": False,
            "planning_owned_source_changes": False,
        },
        "packages": audit_packages(repo_root),
        "launches": launches,
        "executables": audit_executables(repo_root, launches),
        "static_topic_references": audit_static_topics(repo_root, launches),
        "message_contracts": audit_messages(repo_root),
        "representative_bags": audit_bags(data_root),
        "sensors": audit_sensors(repo_root, data_root),
        "hd_map": audit_hd_map(repo_root),
        "competition_inputs": audit_competition_inputs(data_root, repo_root),
        "root_scripts": audit_scripts(repo_root),
        "legacy_references": audit_legacy_references(repo_root),
        "future_requirements": future_requirements(),
    }
    return manifest


def dump_manifest(manifest: Mapping[str, Any]) -> str:
    return yaml.safe_dump(
        dict(manifest),
        allow_unicode=True,
        sort_keys=False,
        default_flow_style=False,
        width=120,
    )


def _markdown_table(headers: Sequence[str], rows: Sequence[Sequence[Any]]) -> List[str]:
    lines = ["| " + " | ".join(headers) + " |", "|" + "|".join("---" for _ in headers) + "|"]
    for row in rows:
        rendered = [str(value).replace("|", "\\|").replace("\n", " ") for value in row]
        lines.append("| " + " | ".join(rendered) + " |")
    return lines


def render_report(manifest: Mapping[str, Any]) -> str:
    packages = manifest["packages"]
    launches = manifest["launches"]
    messages = manifest["message_contracts"]
    hd_map = manifest["hd_map"]
    sensors = manifest["sensors"]
    competition = manifest["competition_inputs"]
    competition_metadata = competition.get("metadata_bundle", {})
    lines = [
        "# P1-A Runtime Baseline v1",
        "",
        "> **상태:** 동결된 회귀 기준 · 생성 도구: `tools/audit/build_runtime_baseline.py`",
        "> **주의:** 이 문서는 센서셋·네트워크·패키지 경로·HD맵 geometry를 변경하지 않는다.",
        "",
        "## 1. 요약",
        "",
        f"- catkin package: **{packages['count']}개**",
        f"- launch file/node 선언: **{launches['file_count']}개 / {launches['node_declaration_count']}개**",
        f"- local runtime message MD5: **{messages['count']}개**",
        f"- HD Map: `{hd_map['build_id']}` · geometry SHA-256 `{hd_map['geometry_sha256']}`",
        f"- HD Map inventory: lane marking **{hd_map['feature_counts'].get('lane_marking', 0)}개**, stopline **{hd_map['feature_counts'].get('stopline', 0)}개**",
        f"- 공식 checkpoint: **{competition['checkpoints']['count']}개**, 반경 **{competition['checkpoints']['pass_radius_m']:.1f} m**",
        f"- 전역경로: **{competition['global_path']['length_xy_m']:.2f} m**, 출발 5% station **{competition['global_path']['start_success_5_percent_station_m']:.2f} m**",
        f"- P1-C metadata: checkpoint **{competition_metadata.get('checkpoint_count', 0)}개**, mission zone **{competition_metadata.get('mission_zone_count', 0)}개** (아직 미발행)",
        "",
        "현재 stage14 map은 검수 기준이지만 `partial_candidate=true`, `operational=false`다. 이 상태값을",
        "geometry 회귀 기준과 별개로 보존한다.",
        "",
        "## 2. HD Map 회귀 기준",
        "",
    ]
    lines.extend(
        _markdown_table(
            ["항목", "값"],
            [
                ["Map / build", f"`{hd_map['map_id']}` / `{hd_map['build_id']}`"],
                ["Source file SHA-256", f"`{hd_map['file_sha256']}`"],
                ["Geometry-only SHA-256", f"`{hd_map['geometry_sha256']}`"],
                ["Lane marking", hd_map["feature_counts"].get("lane_marking", 0)],
                ["White / yellow", f"{hd_map['lane_marking_colors'].get('white', 0)} / {hd_map['lane_marking_colors'].get('yellow', 0)}"],
                ["Solid / broken", f"{hd_map['lane_marking_patterns'].get('solid', 0)} / {hd_map['lane_marking_patterns'].get('broken', 0)}"],
                ["Stopline", hd_map["feature_counts"].get("stopline", 0)],
            ],
        )
    )
    lines.extend(["", "## 3. 센서 기준과 승인된 후속 변경", ""])
    sensor_rows = []
    for item in sensors["canonical_v2"]["inventory"]:
        udp = item["udp"]
        sensor_rows.append(
            [
                item["id"],
                item["label"],
                f"{item['nominal_hz']:.3f}",
                f"{udp['host_port']} → {udp['destination_port']}",
                f"({item['position_m']['x']:.2f}, {item['position_m']['y']:.2f}, {item['position_m']['z']:.2f})",
            ]
        )
    lines.extend(_markdown_table(["ID", "센서", "현재 Hz", "UDP port", "위치 (m)"], sensor_rows))
    lidar_contract = sensors["lidar_rate_contract"]
    lines.extend(
        [
            "",
            f"LiDAR는 현재 **{lidar_contract['baseline_hz']:.0f} Hz**가 회귀 기준이다. 손재호 담당 코드의",
            f"요구에 따른 **{lidar_contract['approved_future_target_hz']:.0f} Hz** 변경은 3단계의 승인된 요구지만 아직 적용하지 않았다.",
            "MORAI UI에서 15 Hz로 설정·재저장하여 `sensorPeriod`와 `frequencyIndex`를 확인한 뒤",
            "30초 이상 `/velodyne_points` 실측과 tracking/resource 비교를 통과해야 한다.",
            "",
            "현장 제출 `Autonav_sensor.json`은 provenance 전용이다. v2와의 semantic 차이는",
            "자유 카메라 pitch(`328°` 대 제출본 `340°`) 하나뿐이며 실행 정본은 v2다.",
            "",
            "## 4. 대표 bag 토픽",
            "",
        ]
    )
    for label in ("raw", "result"):
        bag = manifest["representative_bags"][label]
        lines.extend(
            [
                f"### {label}",
                "",
                f"`$ASMC_DATA/{bag['path']}` · {bag['duration_s']:.3f} s · SHA-256 `{bag['sha256']}`",
                "",
            ]
        )
        lines.extend(
            _markdown_table(
                ["토픽", "타입", "MD5", "메시지", "평균 Hz"],
                [
                    [
                        f"`{topic['topic']}`",
                        f"`{topic['type']}`",
                        f"`{topic['md5']}`",
                        topic["messages"],
                        f"{topic['average_hz']:.3f}",
                    ]
                    for topic in bag["topics"]
                ],
            )
        )
        lines.append("")
    lines.extend(["## 5. 팀 메시지 MD5", ""])
    lines.extend(
        _markdown_table(
            ["타입", "ROS MD5", "소스 SHA-256"],
            [
                [f"`{item['type']}`", f"`{item['ros_md5']}`", f"`{item['source_sha256']}`"]
                for item in messages["items"]
            ],
        )
    )
    lines.extend(["", "## 6. 이후 단계에 넘기는 요구사항", ""])
    lines.extend(
        _markdown_table(
            ["ID", "단계", "상태", "요구사항"],
            [
                [item["id"], item["stage"], item["status"], item["requirement"]]
                for item in manifest["future_requirements"]
            ],
        )
    )
    lines.extend(
        [
            "",
            "## 7. 재생성·검증",
            "",
            "```bash",
            "python3 tools/audit/build_runtime_baseline.py --write",
            "python3 tools/audit/build_runtime_baseline.py --check",
            "```",
            "",
            "`--check`는 manifest와 보고서를 메모리에서 다시 생성해 커밋된 파일과 byte-for-byte 비교한다.",
            "대표 bag과 현장 자료는 복사하지 않고 상대 경로·크기·SHA만 저장한다. 로컬 runtime message",
            "MD5는 generated header를 우선 사용한다. 아직 build되지 않은 `asmc_msgs`만 P1-B에서 기존",
            "generated MD5와 대조한 ROS1 source 계산기를 fallback으로 사용하며, 그 밖의 package는 clean",
            "환경에서 먼저 build해야 한다.",
            "",
        ]
    )
    return "\n".join(lines)


def _write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _check_text(path: Path, expected: str) -> bool:
    if not path.is_file():
        print(f"ERROR: missing generated file: {path}", file=sys.stderr)
        return False
    actual = path.read_text(encoding="utf-8")
    if actual == expected:
        return True
    print(f"ERROR: generated file is stale: {path}", file=sys.stderr)
    diff = difflib.unified_diff(
        actual.splitlines(), expected.splitlines(), fromfile=str(path), tofile="regenerated"
    )
    for line in list(diff)[:200]:
        print(line, file=sys.stderr)
    return False


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--data-root", type=Path, default=None)
    parser.add_argument(
        "--manifest", type=Path, default=Path("config/baselines/runtime_baseline_v1.yaml")
    )
    parser.add_argument(
        "--report", type=Path, default=Path("docs/baselines/runtime-baseline-v1.md")
    )
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--write", action="store_true", help="write the baseline outputs")
    mode.add_argument("--check", action="store_true", help="fail when committed outputs are stale")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    repo_root = args.repo_root.resolve()
    if not (repo_root / ".git").exists():
        raise BaselineError(f"not a repository root: {repo_root}")
    data_root = (
        args.data_root
        or (Path(os.environ["ASMC_DATA"]) if os.environ.get("ASMC_DATA") else repo_root.parent / "data")
    ).resolve()
    manifest_path = args.manifest if args.manifest.is_absolute() else repo_root / args.manifest
    report_path = args.report if args.report.is_absolute() else repo_root / args.report
    manifest = build_manifest(repo_root, data_root)
    manifest_text = dump_manifest(manifest)
    report_text = render_report(manifest)
    if args.write:
        _write_text(manifest_path, manifest_text)
        _write_text(report_path, report_text)
        print(f"wrote {manifest_path}")
        print(f"wrote {report_path}")
        return 0
    valid = _check_text(manifest_path, manifest_text) and _check_text(report_path, report_text)
    if valid:
        print(f"baseline is current: {manifest_path}")
        print(f"report is current: {report_path}")
        return 0
    return 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except BaselineError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2)
