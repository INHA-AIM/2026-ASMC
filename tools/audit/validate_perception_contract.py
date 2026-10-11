#!/usr/bin/env python3
"""Validate the additive perception message/topic contract without ROS."""

from __future__ import annotations

import argparse
import hashlib
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
    runtime_move_path = contract.get("runtime_move_evidence")
    expected_runtime_move = (
        "config/baselines/perception_runtime_move_smoke_v1.yaml"
    )
    if runtime_move_path != expected_runtime_move:
        raise ContractError("perception runtime move evidence is not pinned")
    runtime_move = _mapping(
        yaml.safe_load(
            (repo_root / expected_runtime_move).read_text(encoding="utf-8")
        ),
        "perception runtime move evidence",
    )
    if (
        runtime_move.get("acceptance_id")
        != "p4c4_camera_bev_lane_physical_move_v1"
        or runtime_move.get("status") != "passed"
    ):
        raise ContractError("perception runtime move evidence did not pass")

    support_config_path = contract.get("support_config_evidence")
    expected_support_config = (
        "config/baselines/perception_support_config_parity_v1.yaml"
    )
    if support_config_path != expected_support_config:
        raise ContractError("perception support/config evidence is not pinned")
    support_config = _mapping(
        yaml.safe_load(
            (repo_root / expected_support_config).read_text(encoding="utf-8")
        ),
        "perception support/config evidence",
    )
    if (
        support_config.get("acceptance_id")
        != "p4c6_perception_package_closeout_v1"
        or support_config.get("status") != "passed"
    ):
        raise ContractError("perception support/config evidence did not pass")
    canonical_configs = support_config.get("canonical_configs", [])
    if not isinstance(canonical_configs, list) or len(canonical_configs) != 12:
        raise ContractError("perception canonical config inventory changed")
    for index, raw_config in enumerate(canonical_configs):
        config = _mapping(raw_config, f"canonical_configs[{index}]")
        canonical_path = repo_root / str(config.get("path", ""))
        if not canonical_path.is_file():
            raise ContractError(f"canonical config file missing: {config}")
        canonical_bytes = canonical_path.read_bytes()
        digest = hashlib.sha256(canonical_bytes).hexdigest()
        if digest != config.get("sha256"):
            raise ContractError(f"canonical config SHA drifted: {canonical_path}")

    removed_package_paths = {
        "camera_semantic_perception": repo_root / "src/perception/camera_semantic_perception",
        "drivable_bev": repo_root / "src/perception/drivable_bev",
        "traffic_light": repo_root / "src/traffic_light",
    }
    for removed_package, removed_path in removed_package_paths.items():
        if removed_path.exists():
            raise ContractError(
                f"retired perception package still exists: {removed_package}"
            )

    artifacts = support_config.get("canonical_artifacts", [])
    if not isinstance(artifacts, list) or len(artifacts) != 1:
        raise ContractError("perception canonical artifact inventory changed")
    artifact = _mapping(artifacts[0], "canonical_artifacts[0]")
    artifact_path = repo_root / str(artifact.get("path", ""))
    if not artifact_path.is_file():
        raise ContractError(f"canonical perception artifact missing: {artifact_path}")
    if hashlib.sha256(artifact_path.read_bytes()).hexdigest() != artifact.get("sha256"):
        raise ContractError(f"canonical perception artifact SHA drifted: {artifact_path}")

    representative = _mapping(
        support_config.get("representative_bag_parity"),
        "representative_bag_parity",
    )
    if (
        representative.get("acceptance_id")
        != "p4c5_representative_bag_output_parity_v1"
        or representative.get("status") != "passed"
    ):
        raise ContractError("representative perception bag parity did not pass")
    parity_topics = _mapping(representative.get("topics"), "parity topics")
    expected_parity_topics = {
        "/perception/camera/front/road_marking/class_id",
        "/perception/bev/camera/road_marking/class_id",
        "/perception/lane/boundary_observations",
    }
    if set(parity_topics) != expected_parity_topics:
        raise ContractError("representative perception parity topics changed")
    for topic, raw_result in parity_topics.items():
        result = _mapping(raw_result, f"parity topic {topic}")
        if int(result.get("common_source_stamps", 0)) < 8:
            raise ContractError(f"insufficient common parity frames: {topic}")
        if int(result.get("payload_mismatches", -1)) != 0:
            raise ContractError(f"perception payload parity regressed: {topic}")

    canonical_source = repo_root / "src/asmc_camera/src/asmc_camera"
    forbidden_imports = (
        "from camera_semantic_perception",
        "import camera_semantic_perception",
        "from drivable_bev",
        "import drivable_bev",
        "from data_collection",
        "import data_collection",
    )
    for source_path in canonical_source.rglob("*.py"):
        body = source_path.read_text(encoding="utf-8")
        if any(value in body for value in forbidden_imports):
            raise ContractError(
                f"canonical perception still imports a compatibility package: {source_path}"
            )

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
            closed_loop_path = definition.get("closed_loop_evidence")
            expected_closed_loop = (
                "config/baselines/perception_traffic_light_closed_loop_v1.yaml"
            )
            if closed_loop_path != expected_closed_loop:
                raise ContractError("traffic-light closed-loop evidence is not pinned")
            closed_loop = _mapping(
                yaml.safe_load(
                    (repo_root / expected_closed_loop).read_text(encoding="utf-8")
                ),
                "traffic-light closed-loop evidence",
            )
            if (
                closed_loop.get("acceptance_id")
                != "p4c4_traffic_light_behavior_closed_loop_v1"
                or closed_loop.get("status") != "passed"
            ):
                raise ContractError("traffic-light closed-loop evidence did not pass")
            vehicle_path = definition.get("vehicle_closed_loop_evidence")
            expected_vehicle = (
                "config/baselines/perception_traffic_light_vehicle_closed_loop_v1.yaml"
            )
            if vehicle_path != expected_vehicle:
                raise ContractError("traffic-light vehicle evidence is not pinned")
            vehicle = _mapping(
                yaml.safe_load(
                    (repo_root / expected_vehicle).read_text(encoding="utf-8")
                ),
                "traffic-light vehicle closed-loop evidence",
            )
            if (
                vehicle.get("acceptance_id")
                != "p4c4_traffic_light_vehicle_closed_loop_v1"
                or vehicle.get("status") != "passed"
            ):
                raise ContractError("traffic-light vehicle evidence did not pass")

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
    behavior = _mapping(bindings.get("asmc_planner"), "behavior binding")
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
        repo_root / "src/asmc_planner/config/behavior/behavior_params.yaml"
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

    for filename in ("autonomy_no_bridges_v2.launch",):
        behavior_launch = ET.parse(
            repo_root / "src/asmc_planner/launch" / filename
        ).getroot()
        nodes = {
            (item.get("pkg"), item.get("type"))
            for item in behavior_launch.iter("node")
        }
        if ("asmc_camera", "traffic_light_node.py") not in nodes:
            raise ContractError(f"{filename} does not start canonical producer")
        if ("traffic_light", "live_infer.py") in nodes:
            raise ContractError(f"{filename} regressed to legacy producer")

    launch_path = repo_root / "src" / "asmc_camera" / "launch" / "perception.launch"
    root = ET.parse(launch_path).getroot()
    arguments = {item.get("name"): item.get("default") for item in root.findall("arg")}
    for name in (
        "start_camera_lane",
        "start_traffic_light",
        "start_visualization",
    ):
        if name not in arguments:
            raise ContractError(f"facade launch is missing {name}")

    runtime_nodes = {
        (item.get("pkg"), item.get("type")) for item in root.iter("node")
    }
    for executable in (
        "camera_bev_node.py",
        "camera_bev_fusion_node.py",
        "lane_vectorizer_node.py",
        "traffic_light_node.py",
    ):
        if ("asmc_camera", executable) not in runtime_nodes:
            raise ContractError(
                f"facade launch does not use canonical executable: {executable}"
            )
    launch_body = launch_path.read_text(encoding="utf-8")
    for package_name in (
        "camera_semantic_perception",
        "drivable_bev",
        "data_collection",
    ):
        if f"$(find {package_name})" in launch_body:
            raise ContractError(
                f"facade launch still loads compatibility package {package_name}"
            )

    return {
        "contract_id": contract.get("contract_id"),
        "message_count": len(declared),
        "canonical_topic_count": len(canonical),
        "compatibility_topic_count": len(compatibility),
        "consumer_binding_count": len(bindings),
        "guarded_config_count": len(canonical_configs),
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
        f"{result['compatibility_topic_count']} compatibility topics, "
        f"{result['guarded_config_count']} guarded canonical configs"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
