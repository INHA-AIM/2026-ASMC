#!/usr/bin/env python3
"""Validate the additive perception message/topic contract without ROS."""

from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, Mapping, Optional, Sequence

import yaml

from validate_p1b_contract import (
    ContractError,
    all_message_md5,
    load_message_specs,
)


REQUIRED_MESSAGES = {
    "TrafficLightDetection",
    "TrackedObject",
    "TrackedObjectArray",
}

REQUIRED_CANONICAL_TOPICS = {
    "/perception/traffic_light/detection",
    "/perception/lidar/tracked_objects",
    "/perception/lidar/drivable_tracked_objects",
}

REQUIRED_COMPATIBILITY_TOPICS = {
    "/traffic_light/state",
    "/traffic_light/detail",
    "/lidar/tracked_objects_map",
    "/last_lidar",
}


def _mapping(value: object, label: str) -> Mapping[str, object]:
    if not isinstance(value, dict):
        raise ContractError(f"{label} must be a mapping")
    return value


def _message_name(type_name: str) -> str:
    prefix = "asmc_msgs/"
    if not type_name.startswith(prefix):
        raise ContractError(f"perception message must use {prefix}: {type_name}")
    return type_name[len(prefix) :]


def _constant_map(spec) -> Dict[str, str]:
    return {value.name: value.value for value in spec.constants}


def validate_contract(repo_root: Path, contract_path: Path) -> Mapping[str, object]:
    contract = yaml.safe_load(contract_path.read_text(encoding="utf-8"))
    contract = _mapping(contract, "contract")
    if contract.get("schema_version") != 1 or contract.get("status") != "frozen":
        raise ContractError("perception contract must be schema_version 1 and frozen")
    if contract.get("compatibility_policy") != "additive_only":
        raise ContractError("perception contract must be additive_only")

    message_dir = repo_root / "src" / "asmc_msgs" / "msg"
    specs = load_message_specs(message_dir)
    md5s = all_message_md5(specs)
    declared = _mapping(contract.get("messages"), "messages")
    names = {_message_name(str(type_name)) for type_name in declared}
    if names != REQUIRED_MESSAGES:
        raise ContractError(
            f"perception message inventory mismatch: {sorted(names)}"
        )
    for type_name, raw_definition in declared.items():
        name = _message_name(str(type_name))
        definition = _mapping(raw_definition, f"messages.{type_name}")
        if definition.get("md5") != md5s[name]:
            raise ContractError(
                f"MD5 mismatch for {type_name}: contract={definition.get('md5')} source={md5s[name]}"
            )
        required = {str(value) for value in definition.get("required_fields", [])}
        fields = {field.name for field in specs[name].fields}
        if not required <= fields:
            raise ContractError(
                f"{type_name} missing required fields: {sorted(required - fields)}"
            )

    traffic_constants = _constant_map(specs["TrafficLightDetection"])
    expected_states = {
        "STATE_UNKNOWN": "0",
        "STATE_NOT_DETECTED": "1",
        "STATE_RED": "2",
        "STATE_YELLOW": "3",
        "STATE_GREEN": "4",
        "STATE_RED_YELLOW": "5",
        "STATE_RED_LEFT": "6",
        "STATE_GREEN_LEFT": "7",
        "STATE_LEFT": "8",
    }
    if traffic_constants != expected_states:
        raise ContractError("TrafficLightDetection state constants changed")

    canonical = _mapping(contract.get("canonical_topics"), "canonical_topics")
    if set(canonical) != REQUIRED_CANONICAL_TOPICS:
        raise ContractError("canonical perception topic inventory changed")
    for topic, raw_definition in canonical.items():
        definition = _mapping(raw_definition, f"canonical_topics.{topic}")
        name = _message_name(str(definition.get("type", "")))
        if name not in REQUIRED_MESSAGES or definition.get("md5") != md5s[name]:
            raise ContractError(f"canonical topic type/MD5 mismatch: {topic}")
        if definition.get("latch") is not False:
            raise ContractError(f"dynamic perception topic must not be latched: {topic}")
        expected_status = (
            "active_parallel_p4c"
            if topic == "/perception/traffic_light/detection"
            else "declared_not_published_p4b"
        )
        if definition.get("producer_status") != expected_status:
            raise ContractError(
                f"unexpected producer status for {topic}: "
                f"{definition.get('producer_status')}"
            )
        if topic == "/perception/traffic_light/detection":
            if definition.get("parity_evidence") != (
                "config/baselines/perception_traffic_light_parity_v1.yaml"
            ):
                raise ContractError("traffic-light parity evidence is not pinned")
            if definition.get("consumer_cutover_status") != (
                "typed_preferred_legacy_fallback_p4c3"
            ):
                raise ContractError("traffic-light consumer cutover status is ambiguous")

    compatibility = _mapping(
        contract.get("compatibility_topics"), "compatibility_topics"
    )
    if set(compatibility) != REQUIRED_COMPATIBILITY_TOPICS:
        raise ContractError("compatibility perception topic inventory changed")
    for topic, raw_definition in compatibility.items():
        definition = _mapping(raw_definition, f"compatibility_topics.{topic}")
        if not definition.get("retire_gate"):
            raise ContractError(f"compatibility topic lacks retire gate: {topic}")

    bindings = _mapping(contract.get("consumer_bindings"), "consumer_bindings")
    behavior = _mapping(bindings.get("behavior_planner"), "behavior binding")
    expected_binding = {
        "canonical_topic": "/perception/traffic_light/detection",
        "compatibility_topic": "/traffic_light/state",
        "selection_policy": "typed_authoritative_after_first_message",
        "invalid_or_stale_policy": "unknown_without_legacy_fallback",
    }
    for key, expected in expected_binding.items():
        if behavior.get(key) != expected:
            raise ContractError(
                f"behavior traffic-light binding {key} changed: {behavior.get(key)}"
            )

    behavior_config_path = (
        repo_root / "src/behavior_planner/src/config/behavior_params.yaml"
    )
    behavior_config = _mapping(
        yaml.safe_load(behavior_config_path.read_text(encoding="utf-8")),
        "behavior config",
    )
    traffic_config = _mapping(
        behavior_config.get("traffic_light"), "behavior traffic_light config"
    )
    if traffic_config.get("typed_topic") != expected_binding["canonical_topic"]:
        raise ContractError("behavior planner canonical traffic-light topic drifted")
    if traffic_config.get("topic") != expected_binding["compatibility_topic"]:
        raise ContractError("behavior planner compatibility topic drifted")

    for filename in ("autonomy_no_bridges.launch", "autonomy_no_bridges_v2.launch"):
        behavior_launch = ET.parse(
            repo_root / "src/behavior_planner/launch" / filename
        ).getroot()
        nodes = {
            (item.get("pkg"), item.get("type"))
            for item in behavior_launch.iter("node")
        }
        if ("asmc_perception", "traffic_light_node.py") not in nodes:
            raise ContractError(f"{filename} does not start canonical producer")
        if ("traffic_light", "live_infer.py") in nodes:
            raise ContractError(f"{filename} regressed to legacy producer")

    launch_path = repo_root / "src" / "asmc_perception" / "launch" / "perception.launch"
    root = ET.parse(launch_path).getroot()
    arguments = {item.get("name"): item.get("default") for item in root.findall("arg")}
    for name in (
        "start_camera_lane",
        "start_traffic_light",
        "start_lidar_fusion",
        "start_recorder",
        "start_visualization",
    ):
        if name not in arguments:
            raise ContractError(f"facade launch is missing {name}")

    return {
        "contract_id": contract.get("contract_id"),
        "message_count": len(declared),
        "canonical_topic_count": len(canonical),
        "compatibility_topic_count": len(compatibility),
        "consumer_binding_count": len(bindings),
        "message_md5": {
            f"asmc_msgs/{name}": md5s[name] for name in sorted(REQUIRED_MESSAGES)
        },
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repo-root", type=Path, default=Path(__file__).resolve().parents[2]
    )
    parser.add_argument("--contract", type=Path)
    args = parser.parse_args(argv)
    repo_root = args.repo_root.resolve()
    contract = (
        args.contract
        or repo_root / "config/contracts/perception_runtime_topic_contract_v1.yaml"
    ).resolve()
    try:
        result = validate_contract(repo_root, contract)
    except (ContractError, OSError, ValueError, yaml.YAMLError, ET.ParseError) as exc:
        print(f"Perception contract validation failed: {exc}", file=sys.stderr)
        return 1
    print(
        "Perception contract valid: "
        f"{result['message_count']} messages, "
        f"{result['canonical_topic_count']} canonical topics, "
        f"{result['compatibility_topic_count']} compatibility topics"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
