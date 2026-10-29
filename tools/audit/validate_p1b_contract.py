#!/usr/bin/env python3
"""Validate the frozen P1-B ROS1 message and HD Map topic contract.

The validator intentionally does not import ROS.  It implements the ROS1
message MD5 text rules and checks the result against both the P1-A baseline and
the P1-B contract, so it can run in CI before a Noetic workspace is available.
An actual catkin build remains a separate release gate.
"""

from __future__ import annotations

import argparse
import hashlib
import re
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Mapping, MutableMapping, Optional, Sequence, Tuple

import yaml


class ContractError(RuntimeError):
    """Raised when a frozen contract invariant is violated."""


BUILTIN_TYPES = {
    "bool",
    "int8",
    "uint8",
    "int16",
    "uint16",
    "int32",
    "uint32",
    "int64",
    "uint64",
    "float32",
    "float64",
    "string",
    "time",
    "duration",
    "char",
    "byte",
}

# Dependencies used by asmc_msgs v1. These are the canonical ROS1 MD5 sums.
EXTERNAL_MESSAGE_MD5 = {
    "std_msgs/Header": "2176decaecbce78abc3b96ef049fabed",
    "nav_msgs/Odometry": "cd5e73d190d741a2f92e81eda573aca7",
    "geometry_msgs/Point": "4a842b65f413084dc2b10fb484ea7f17",
    "geometry_msgs/Point32": "cc153912f1453b708d221682bc23d9ac",
    "geometry_msgs/Pose": "e45d45a5a1ce597b249e23fb30fc871f",
    "geometry_msgs/Vector3": "4a842b65f413084dc2b10fb484ea7f17",
}

EXTERNAL_TOPIC_TYPES = {
    "nav_msgs/OccupancyGrid": "3381f2d731d4076ec5c71b0759edbe4e",
    "visualization_msgs/MarkerArray": "d155b9ce5188fbaf89745847fd5882d7",
}

REQUIRED_TOPICS = {
    "/hd_map/meta",
    "/hd_map/global/lane_markings",
    "/hd_map/global/lanes",
    "/hd_map/global/drivable_areas",
    "/hd_map/global/drivable_grid",
    "/hd_map/global/checkpoints",
    "/hd_map/global/mission_zones",
    "/hd_map/local/lanes",
    "/hd_map/local/drivable_grid",
    "/hd_map/local/current_lane",
    "/hd_map/debug/markers",
}


@dataclass(frozen=True)
class Constant:
    type_name: str
    name: str
    value: str


@dataclass(frozen=True)
class Field:
    type_name: str
    name: str


@dataclass(frozen=True)
class MessageSpec:
    name: str
    path: Path
    constants: Tuple[Constant, ...]
    fields: Tuple[Field, ...]


def _load_yaml(path: Path) -> Mapping[str, object]:
    if not path.is_file():
        raise ContractError(f"missing YAML: {path}")
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ContractError(f"YAML root must be a mapping: {path}")
    return data


def _strip_array(type_name: str) -> str:
    return re.sub(r"\[[0-9]*\]$", "", type_name)


def _parse_message(path: Path) -> MessageSpec:
    constants: List[Constant] = []
    fields: List[Field] = []
    for line_number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        parts = line.split(None, 1)
        if len(parts) != 2:
            raise ContractError(f"invalid message line {path}:{line_number}: {raw}")
        type_name, declaration = parts
        if "=" in declaration:
            name, value = declaration.split("=", 1)
            constants.append(Constant(type_name, name.strip(), value.strip()))
        else:
            fields.append(Field(type_name, declaration.strip()))
    return MessageSpec(path.stem, path, tuple(constants), tuple(fields))


def load_message_specs(message_dir: Path) -> Dict[str, MessageSpec]:
    specs = {_path.stem: _parse_message(_path) for _path in sorted(message_dir.glob("*.msg"))}
    if not specs:
        raise ContractError(f"no message definitions found under {message_dir}")
    return specs


def ros1_md5(
    message_name: str,
    specs: Mapping[str, MessageSpec],
    cache: Optional[MutableMapping[str, str]] = None,
    stack: Tuple[str, ...] = (),
) -> str:
    """Return a ROS1 message MD5 using genmsg's canonical MD5-text rules."""

    if cache is None:
        cache = {}
    if message_name in cache:
        return cache[message_name]
    if message_name in stack:
        raise ContractError(f"recursive message dependency: {' -> '.join(stack + (message_name,))}")
    if message_name not in specs:
        raise ContractError(f"unknown local message type: {message_name}")

    spec = specs[message_name]
    lines = [
        f"{constant.type_name} {constant.name}={constant.value}"
        for constant in spec.constants
    ]
    for field in spec.fields:
        base_type = _strip_array(field.type_name)
        if base_type == "Header":
            base_type = "std_msgs/Header"
        if base_type in BUILTIN_TYPES:
            md5_type = field.type_name
        elif base_type in EXTERNAL_MESSAGE_MD5:
            # ROS1 replaces a compound field type, including compound arrays,
            # with the dependency MD5 and does not retain the array suffix.
            md5_type = EXTERNAL_MESSAGE_MD5[base_type]
        else:
            local_name = base_type.split("/")[-1]
            md5_type = ros1_md5(local_name, specs, cache, stack + (message_name,))
        lines.append(f"{md5_type} {field.name}")

    digest = hashlib.md5("\n".join(lines).encode("utf-8")).hexdigest()
    cache[message_name] = digest
    return digest


def all_message_md5(specs: Mapping[str, MessageSpec]) -> Dict[str, str]:
    cache: Dict[str, str] = {}
    return {name: ros1_md5(name, specs, cache) for name in sorted(specs)}


def _as_mapping(value: object, label: str) -> Mapping[str, object]:
    if not isinstance(value, dict):
        raise ContractError(f"{label} must be a mapping")
    return value


def _as_list(value: object, label: str) -> List[object]:
    if not isinstance(value, list):
        raise ContractError(f"{label} must be a list")
    return value


def _message_name(type_name: str) -> str:
    prefix = "asmc_msgs/"
    if not type_name.startswith(prefix):
        raise ContractError(f"contract message must use {prefix}: {type_name}")
    return type_name[len(prefix) :]


def _validate_cmake(message_dir: Path, specs: Mapping[str, MessageSpec]) -> None:
    cmake_path = message_dir.parent / "CMakeLists.txt"
    cmake = cmake_path.read_text(encoding="utf-8")
    registered = set(re.findall(r"^\s*([A-Za-z][A-Za-z0-9_]*\.msg)\s*$", cmake, re.MULTILINE))
    actual = {f"{name}.msg" for name in specs}
    missing = sorted(actual - registered)
    stale = sorted(registered - actual)
    if missing or stale:
        raise ContractError(f"CMake message registration mismatch: missing={missing}, stale={stale}")


def _validate_package_version(message_dir: Path, expected: str) -> None:
    root = ET.parse(message_dir.parent / "package.xml").getroot()
    actual = (root.findtext("version") or "").strip()
    if actual != expected:
        raise ContractError(f"asmc_msgs version {actual!r}, contract requires {expected!r}")


def _validate_legacy_against_p1a(
    legacy: Mapping[str, object], baseline_path: Path
) -> None:
    baseline = _load_yaml(baseline_path)
    section = _as_mapping(baseline.get("message_contracts"), "P1-A message_contracts")
    items = _as_list(section.get("items"), "P1-A message_contracts.items")
    recorded = {
        item["type"]: item["ros_md5"]
        for item in items
        if isinstance(item, dict) and str(item.get("type", "")).startswith("asmc_msgs/")
    }
    for type_name, expected in legacy.items():
        if recorded.get(type_name) != expected:
            raise ContractError(
                f"legacy MD5 differs from P1-A for {type_name}: "
                f"contract={expected}, baseline={recorded.get(type_name)}"
            )


def _validate_messages(
    contract: Mapping[str, object], specs: Mapping[str, MessageSpec], baseline_path: Path,
    registered_names: Optional[set] = None,
) -> Dict[str, str]:
    md5s = all_message_md5(specs)
    legacy = _as_mapping(contract.get("legacy_messages"), "legacy_messages")
    additions = _as_mapping(contract.get("new_messages"), "new_messages")
    _validate_legacy_against_p1a(legacy, baseline_path)

    declared_names = {_message_name(name) for name in legacy} | {
        _message_name(name) for name in additions
    }
    missing_sources = sorted(declared_names - set(specs))
    if missing_sources:
        raise ContractError(
            f"contract message sources are missing: {missing_sources}"
        )
    if registered_names is None:
        registered_names = declared_names
    if set(specs) != registered_names:
        raise ContractError(
            "message registry mismatch: "
            f"unregistered={sorted(set(specs) - registered_names)}, "
            f"missing_sources={sorted(registered_names - set(specs))}"
        )

    for type_name, expected_md5 in legacy.items():
        name = _message_name(type_name)
        if md5s[name] != expected_md5:
            raise ContractError(
                f"legacy MD5 changed for {type_name}: expected {expected_md5}, got {md5s[name]}"
            )

    for type_name, definition_value in additions.items():
        name = _message_name(type_name)
        definition = _as_mapping(definition_value, f"new_messages.{type_name}")
        expected_md5 = definition.get("md5")
        if md5s[name] != expected_md5:
            raise ContractError(
                f"new message MD5 drift for {type_name}: expected {expected_md5}, got {md5s[name]}"
            )
        required = {str(item) for item in _as_list(definition.get("required_fields"), f"{type_name}.required_fields")}
        actual_fields = {field.name for field in specs[name].fields}
        missing = sorted(required - actual_fields)
        if missing:
            raise ContractError(f"{type_name} missing required fields: {missing}")
    return md5s


def _load_registered_message_names(repo_root: Path, registry_relative: object) -> set:
    if not registry_relative:
        raise ContractError("message_registry is required")
    registry_path = repo_root / str(registry_relative)
    registry = _load_yaml(registry_path)
    if registry.get("schema_version") != 1:
        raise ContractError("message registry must use schema_version 1")
    entries = _as_list(registry.get("contracts"), "message registry contracts")
    owners: Dict[str, str] = {}
    for index, raw_entry in enumerate(entries):
        entry = _as_mapping(raw_entry, f"message registry contracts[{index}]")
        relative = str(entry.get("path", ""))
        if not relative:
            raise ContractError("message registry contract path is empty")
        child = _load_yaml(repo_root / relative)
        sections = _as_list(entry.get("message_sections"), f"{relative}.message_sections")
        for raw_section in sections:
            section_name = str(raw_section)
            section = _as_mapping(child.get(section_name), f"{relative}.{section_name}")
            for type_name in section:
                name = _message_name(str(type_name))
                if name in owners:
                    raise ContractError(
                        f"message {type_name} has multiple contracts: {owners[name]}, {relative}"
                    )
                owners[name] = relative
    return set(owners)


def _constant_map(spec: MessageSpec) -> Dict[str, str]:
    return {constant.name: constant.value for constant in spec.constants}


def _validate_semantics(specs: Mapping[str, MessageSpec]) -> None:
    mission = _constant_map(specs["MissionZone"])
    required_statuses = {
        "STATUS_UNSPECIFIED": "0",
        "STATUS_OFFICIAL": "1",
        "STATUS_DERIVED_OFFICIAL": "2",
        "STATUS_PROVISIONAL": "3",
    }
    if {name: mission.get(name) for name in required_statuses} != required_statuses:
        raise ContractError("MissionZone authority status constants changed")

    checkpoint_fields = {field.name for field in specs["CompetitionCheckpoint"].fields}
    if not {"official_yaw_deg", "pass_radius_m", "route_station_m", "link_id"} <= checkpoint_fields:
        raise ContractError("CompetitionCheckpoint lost an official/projection field")

    marking = _constant_map(specs["LaneMarking"])
    if marking.get("PATTERN_SOLID") != "1" or marking.get("PATTERN_BROKEN") != "2":
        raise ContractError("LaneMarking solid/broken constants changed")

    association = _constant_map(specs["LaneAssociation"])
    if (
        association.get("STATE_INSIDE") != "1"
        or association.get("STATE_NEARBY") != "3"
        or association.get("STATE_STALE") != "4"
        or association.get("REFERENCE_BASE_LINK_REAR_AXLE") != "1"
    ):
        raise ContractError("LaneAssociation state/reference constants changed")


def _validate_topics(contract: Mapping[str, object], message_md5: Mapping[str, str]) -> None:
    topics = _as_mapping(contract.get("topics"), "topics")
    names = set(topics)
    if names != REQUIRED_TOPICS:
        raise ContractError(
            f"topic inventory mismatch: missing={sorted(REQUIRED_TOPICS - names)}, "
            f"unexpected={sorted(names - REQUIRED_TOPICS)}"
        )
    known_types = {
        **{f"asmc_msgs/{name}": digest for name, digest in message_md5.items()},
        **EXTERNAL_TOPIC_TYPES,
    }
    for topic, definition_value in topics.items():
        if not str(topic).startswith("/"):
            raise ContractError(f"topic must be absolute: {topic}")
        definition = _as_mapping(definition_value, f"topics.{topic}")
        type_name = definition.get("type")
        if type_name not in known_types:
            raise ContractError(f"unknown topic type for {topic}: {type_name}")
        if definition.get("md5") != known_types[type_name]:
            raise ContractError(
                f"topic MD5 mismatch for {topic}: expected {known_types[type_name]}, "
                f"got {definition.get('md5')}"
            )
        scope = definition.get("scope")
        latch = definition.get("latch")
        if scope == "global" and latch is not True:
            raise ContractError(f"global topic must be latched: {topic}")
        if scope in {"local", "debug"} and latch is not False:
            raise ContractError(f"{scope} topic must not be latched: {topic}")
        if definition.get("queue_size") != 1:
            raise ContractError(f"contract queue_size must be 1: {topic}")


def validate_contract(repo_root: Path, contract_path: Path, baseline_path: Path) -> Dict[str, object]:
    contract = _load_yaml(contract_path)
    if contract.get("schema_version") != 1 or contract.get("status") != "frozen":
        raise ContractError("contract must be schema_version 1 with status=frozen")
    package = _as_mapping(contract.get("package"), "package")
    if package.get("compatibility_policy") != "additive_only":
        raise ContractError("P1-B requires compatibility_policy=additive_only")

    message_dir = repo_root / "src" / "asmc_msgs" / "msg"
    specs = load_message_specs(message_dir)
    _validate_cmake(message_dir, specs)
    _validate_package_version(message_dir, str(package.get("version", "")))
    registered_names = _load_registered_message_names(
        repo_root, contract.get("message_registry")
    )
    md5s = _validate_messages(contract, specs, baseline_path, registered_names)
    _validate_semantics(specs)
    _validate_topics(contract, md5s)
    return {
        "contract_id": contract.get("contract_id"),
        "message_count": len(specs),
        "contract_message_count": len(_as_mapping(contract.get("legacy_messages"), "legacy_messages"))
        + len(_as_mapping(contract.get("new_messages"), "new_messages")),
        "registered_message_count": len(registered_names),
        "legacy_message_count": len(_as_mapping(contract.get("legacy_messages"), "legacy_messages")),
        "new_message_count": len(_as_mapping(contract.get("new_messages"), "new_messages")),
        "topic_count": len(_as_mapping(contract.get("topics"), "topics")),
        "message_md5": {f"asmc_msgs/{name}": md5s[name] for name in sorted(md5s)},
    }


def _default_repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=_default_repo_root())
    parser.add_argument("--contract", type=Path)
    parser.add_argument("--baseline", type=Path)
    args = parser.parse_args(argv)

    repo_root = args.repo_root.resolve()
    contract = (args.contract or repo_root / "config/contracts/hd_map_topic_contract_v1.yaml").resolve()
    baseline = (args.baseline or repo_root / "config/baselines/runtime_baseline_v1.yaml").resolve()
    try:
        result = validate_contract(repo_root, contract, baseline)
    except (ContractError, OSError, ValueError, yaml.YAMLError) as exc:
        print(f"P1-B contract validation failed: {exc}", file=sys.stderr)
        return 1

    print(
        "P1-B contract valid: "
        f"{result['registered_message_count']} registered messages; "
        f"HD Map contract owns {result['contract_message_count']} "
        f"({result['legacy_message_count']} frozen + {result['new_message_count']} additive), "
        f"{result['topic_count']} topics"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
