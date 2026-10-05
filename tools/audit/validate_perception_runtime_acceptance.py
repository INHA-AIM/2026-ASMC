#!/usr/bin/env python3
"""Validate the P5-C perception live-acceptance contract and source bindings."""

from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Optional, Sequence

import yaml


CONTRACT_ID = "asmc_perception_runtime_acceptance_v1"
REQUIRED_GROUPS = {"camera_lane", "traffic_light", "visualization"}
REQUIRED_CONSUMER_GATES = {"planning_traffic_light", "lane_adapter"}


class AcceptanceContractError(RuntimeError):
    """Raised when the live-acceptance contract is incomplete or stale."""


def _mapping(value: object, label: str) -> Mapping[str, object]:
    if not isinstance(value, dict):
        raise AcceptanceContractError(f"{label} must be a mapping")
    return value


def _list(value: object, label: str) -> List[object]:
    if not isinstance(value, list):
        raise AcceptanceContractError(f"{label} must be a list")
    return value


def load_contract(path: Path) -> Mapping[str, object]:
    return _mapping(yaml.safe_load(path.read_text(encoding="utf-8")), str(path))


def expand_pattern(pattern: str, views: Sequence[str]) -> List[str]:
    if "{view}" in pattern:
        return [pattern.replace("{view}", view) for view in views]
    return [pattern]


def expand_topic_specs(
    contract: Mapping[str, object], group_names: Iterable[str]
) -> List[Dict[str, object]]:
    views = [str(value) for value in contract["views"]]
    groups = _mapping(contract["topic_groups"], "topic_groups")
    result: List[Dict[str, object]] = []
    for group_name in group_names:
        group = _mapping(groups.get(group_name), f"topic_groups.{group_name}")
        for raw in _list(group.get("topics"), f"topic_groups.{group_name}.topics"):
            spec = dict(_mapping(raw, f"topic_groups.{group_name}.topic"))
            pattern = str(spec.pop("pattern", ""))
            for topic in expand_pattern(pattern, views):
                expanded = dict(spec)
                expanded["topic"] = topic
                expanded["group"] = group_name
                result.append(expanded)
    return result


def expand_forbidden_topics(contract: Mapping[str, object]) -> List[str]:
    views = [str(value) for value in contract["views"]]
    return sorted(
        {
            topic
            for raw in _list(
                contract.get("default_forbidden_topics"),
                "default_forbidden_topics",
            )
            for topic in expand_pattern(str(raw), views)
        }
    )


def _validate_topic_spec(raw: object, label: str) -> Mapping[str, object]:
    spec = _mapping(raw, label)
    pattern = str(spec.get("pattern", ""))
    type_name = str(spec.get("type", ""))
    if not pattern.startswith("/"):
        raise AcceptanceContractError(f"{label}.pattern must be absolute")
    if "/" not in type_name:
        raise AcceptanceContractError(f"{label}.type must be a ROS type")
    publisher = str(spec.get("publisher", ""))
    if not publisher.startswith("/"):
        raise AcceptanceContractError(f"{label}.publisher must be absolute")
    if "min_rate_hz" not in spec and "min_messages" not in spec:
        raise AcceptanceContractError(
            f"{label} needs min_rate_hz or min_messages"
        )
    if "min_rate_hz" in spec and float(spec["min_rate_hz"]) <= 0.0:
        raise AcceptanceContractError(f"{label}.min_rate_hz must be positive")
    if "min_messages" in spec and int(spec["min_messages"]) < 1:
        raise AcceptanceContractError(f"{label}.min_messages must be positive")
    if "max_stale_sec" in spec and float(spec["max_stale_sec"]) <= 0.0:
        raise AcceptanceContractError(f"{label}.max_stale_sec must be positive")
    for subscriber in spec.get("subscribers") or []:
        if not str(subscriber).startswith("/"):
            raise AcceptanceContractError(
                f"{label}.subscribers must be absolute node names"
            )
    return spec


def _load_repository_yaml(repo_root: Path, relative: str) -> Mapping[str, object]:
    path = repo_root / relative
    if not path.is_file():
        raise AcceptanceContractError(f"closeout evidence is missing: {relative}")
    return _mapping(yaml.safe_load(path.read_text(encoding="utf-8")), relative)


def _validate_planning_signal_launch(
    launch: ET.Element, launch_name: str, expected_default: str
) -> None:
    arguments = {item.get("name"): item for item in launch.findall("arg")}
    signal_arg = arguments.get("traffic_light")
    if signal_arg is None or signal_arg.get("default") != expected_default:
        raise AcceptanceContractError(
            f"{launch_name} traffic_light default must be {expected_default}"
        )
    if expected_default == "false":
        if "planning-owner review" not in signal_arg.get("doc", ""):
            raise AcceptanceContractError(
                f"{launch_name} lost the cross-team planning ownership note"
            )
    elif "external traffic-light producer" not in signal_arg.get("doc", ""):
        raise AcceptanceContractError(
            f"{launch_name} must document disabling duplicate signal inference"
        )

    # Planning may own standalone signal inference; the complete camera/lane
    # pipeline remains an independently launched perception graph.
    for include in launch.iter("include"):
        if "$(find asmc_perception)" in include.get("file", ""):
            raise AcceptanceContractError(
                f"{launch_name} must not include the full perception pipeline"
            )
    perception_nodes = [
        node for node in launch.iter("node")
        if node.get("pkg") == "asmc_perception"
    ]
    if len(perception_nodes) != 1 or perception_nodes[0].get("type") != "traffic_light_node.py":
        raise AcceptanceContractError(
            f"{launch_name} must contain only its explicit traffic-light producer"
        )
    signal = perception_nodes[0]
    gated_signal = any(
        group.get("if") == "$(arg traffic_light)"
        and signal in list(group.iter("node"))
        for group in launch.iter("group")
    )
    if not gated_signal:
        raise AcceptanceContractError(
            f"{launch_name} must allow disabling signal inference via traffic_light"
        )
    if expected_default == "true":
        if signal.get("required") != "true":
            raise AcceptanceContractError(
                f"{launch_name} must fail visibly when standalone signal inference dies"
            )
        camera = arguments.get("camera_topic")
        if camera is None or camera.get("default") != "/cam_extra/image_jpeg/compressed":
            raise AcceptanceContractError(
                f"{launch_name} must use the bridge traffic-light camera topic"
            )


def _validate_p5_closeout(
    repo_root: Path, contract: Mapping[str, object]
) -> Mapping[str, object]:
    expected_phases = {
        "p5c_runtime_acceptance": "complete",
        "p5d_consumer_handoff": "complete",
        "p5e_operational_closeout": "complete",
        "next_development": "lidar_fusion_baseline",
    }
    phases = _mapping(contract.get("phase_status"), "phase_status")
    if dict(phases) != expected_phases:
        raise AcceptanceContractError("P5 phase status or next development changed")

    handoff = _mapping(contract.get("consumer_handoff"), "consumer_handoff")
    traffic = _mapping(handoff.get("traffic_light"), "consumer_handoff.traffic_light")
    expected_traffic = {
        "producer_owner": "asmc_perception",
        "canonical_topic": "/perception/traffic_light/detection",
        "compatibility_topic": "/traffic_light/state",
        "consumer_node": "/behavior_planner",
        "selection_policy": "typed_authoritative_after_first_message",
        "invalid_or_stale_policy": "unknown_without_legacy_fallback",
        "planning_launch_starts_producer_by_default": False,
        "compatibility_retirement": "deferred_until_all_consumers_are_typed",
    }
    for key, expected in expected_traffic.items():
        if traffic.get(key) != expected:
            raise AcceptanceContractError(f"traffic-light handoff {key} changed")

    lane = _mapping(handoff.get("lane_family"), "consumer_handoff.lane_family")
    expected_lane = {
        "producer_owner": "asmc_perception",
        "canonical_topic": "/perception/lane/boundary_family",
        "compatibility_topic": "/perception/lane/boundary_observations",
        "reference_consumer_node": "/boundary_family_monitor",
        "planning_input": "typed_family",
        "debug_topic": "/planning/debug/lane_boundary_family/strips",
        "debug_is_planning_input": False,
        "compatibility_enabled_by_default": False,
        "adapter_retirement_owner": "planning",
    }
    for key, expected in expected_lane.items():
        if lane.get(key) != expected:
            raise AcceptanceContractError(f"lane-family handoff {key} changed")

    evidence_paths = [
        str(value)
        for section in (traffic, lane)
        for value in _list(section.get("evidence"), "consumer evidence")
    ]
    for relative in evidence_paths:
        if not (repo_root / relative).is_file():
            raise AcceptanceContractError(
                f"consumer handoff evidence is missing: {relative}"
            )

    expected_defaults = {
        "autonomy_no_bridges.launch": "false",
        "autonomy_no_bridges_v2.launch": "true",
    }
    if traffic.get("planning_launch_signal_defaults") != {
        name: default == "true" for name, default in expected_defaults.items()
    }:
        raise AcceptanceContractError("planning signal launch defaults changed")
    for launch_name, default in expected_defaults.items():
        launch = ET.parse(
            repo_root / "src/behavior_planner/launch" / launch_name
        ).getroot()
        _validate_planning_signal_launch(launch, launch_name, default)

    shadow_result = _load_repository_yaml(
        repo_root,
        "src/data_collection/config/bag_profiles/shadow_boundary_result.yaml",
    )
    required = set(shadow_result.get("required_topics") or [])
    optional = set(shadow_result.get("optional_topics") or [])
    if "/perception/lane/boundary_family" not in required:
        raise AcceptanceContractError("shadow handoff bag must require boundary_family")
    if "/perception/lane/boundary_observations" in required:
        raise AcceptanceContractError("shadow handoff bag still requires legacy observations")
    if "/perception/lane/boundary_observations" not in optional:
        raise AcceptanceContractError("legacy observations must remain optional for A/B replay")

    adapter_result = _load_repository_yaml(
        repo_root,
        "src/data_collection/config/bag_profiles/shadow_boundary_family_adapter_result.yaml",
    )
    adapter_required = set(adapter_result.get("required_topics") or [])
    if adapter_required != {
        "/perception/lane/boundary_family",
        "/planning/diagnostics/lane_boundary_family_adapter",
    }:
        raise AcceptanceContractError(
            "lane adapter result must require only canonical family and diagnostics"
        )

    baseline = _load_repository_yaml(
        repo_root, "config/baselines/runtime_baseline_v1.yaml"
    )
    result_bag = _mapping(
        _mapping(baseline.get("representative_bags"), "representative_bags").get(
            "result"
        ),
        "representative_bags.result",
    )
    family_topic = next(
        (
            item
            for item in result_bag.get("topics") or []
            if item.get("topic") == "/perception/lane/boundary_family"
        ),
        None,
    )
    if family_topic is None or int(family_topic.get("messages", 0)) < 1:
        raise AcceptanceContractError("representative result bag lacks boundary_family")

    closeout = _mapping(contract.get("operational_closeout"), "operational_closeout")
    expected_closeout = {
        "runtime_inference_container": "asmc-ros-noetic",
        "training_container": "asmc-perception-train",
        "bridge_process_policy": "separate_foreground_for_rate_monitoring",
        "web_console": "deferred_until_operational_demand",
    }
    for key, expected in expected_closeout.items():
        if closeout.get(key) != expected:
            raise AcceptanceContractError(f"operational closeout {key} changed")
    if (repo_root / "scripts/perception.sh").exists():
        raise AcceptanceContractError("removed perception wrapper was reintroduced")

    commands = _mapping(closeout.get("commands"), "operational closeout commands")
    expected_commands = {
        "bridge": "./scripts/udp_bridge.sh",
        "perception": (
            "roslaunch asmc_perception perception.launch start_traffic_light:=true"
        ),
        "lane_adapter": (
            "roslaunch lane_boundary_adapter shadow_boundary_family_candidate.launch"
        ),
        "runtime_acceptance": "./scripts/verify_perception.sh",
    }
    if dict(commands) != expected_commands:
        raise AcceptanceContractError("canonical P5 operational commands changed")
    for relative in ("scripts/udp_bridge.sh", "scripts/verify_perception.sh"):
        if not (repo_root / relative).is_file():
            raise AcceptanceContractError(f"operational entrypoint is missing: {relative}")
    for relative in (
        "src/asmc_perception/launch/perception.launch",
        "src/jamming_lane_boundary_adapter/launch/shadow_boundary_family_candidate.launch",
    ):
        if not (repo_root / relative).is_file():
            raise AcceptanceContractError(f"operational launch is missing: {relative}")

    visualization = _mapping(closeout.get("visualization"), "closeout visualization")
    if visualization.get("rqt_topics") != [
        "/perception/camera/debug/mosaic/compressed",
        "/perception/bev/debug/comparison/compressed",
    ]:
        raise AcceptanceContractError("canonical rqt topic pair changed")
    rviz_path = str(visualization.get("lane_rviz_config", ""))
    if visualization.get("lane_rviz_fixed_frame") != "base_link" or not (
        repo_root / rviz_path
    ).is_file():
        raise AcceptanceContractError("lane RViz closeout binding is invalid")

    return {
        "phase_count": 3,
        "consumer_count": len(handoff),
        "evidence_count": len(evidence_paths),
        "representative_lane_family_messages": int(family_topic["messages"]),
    }


def validate_contract(
    repo_root: Path, contract_path: Path
) -> Mapping[str, object]:
    contract = load_contract(contract_path)
    if contract.get("schema_version") != 1:
        raise AcceptanceContractError("schema_version must be 1")
    if contract.get("contract_id") != CONTRACT_ID:
        raise AcceptanceContractError("unexpected contract_id")
    if contract.get("status") != "active":
        raise AcceptanceContractError("acceptance contract must be active")
    if float(contract.get("default_duration_sec", 0.0)) <= 0.0:
        raise AcceptanceContractError("default_duration_sec must be positive")
    views = [str(value) for value in _list(contract.get("views"), "views")]
    if views != ["front", "left", "right"]:
        raise AcceptanceContractError("views must be front,left,right")

    groups = _mapping(contract.get("topic_groups"), "topic_groups")
    if set(groups) != REQUIRED_GROUPS:
        raise AcceptanceContractError(
            "topic group mismatch: missing={} extra={}".format(
                sorted(REQUIRED_GROUPS - set(groups)),
                sorted(set(groups) - REQUIRED_GROUPS),
            )
        )
    expanded_owner: Dict[str, str] = {}
    for group_name, raw_group in groups.items():
        group = _mapping(raw_group, f"topic_groups.{group_name}")
        nodes = [
            str(value)
            for value in _list(
                group.get("required_nodes"),
                f"topic_groups.{group_name}.required_nodes",
            )
        ]
        if not nodes or any(not value.startswith("/") for value in nodes):
            raise AcceptanceContractError(
                f"topic_groups.{group_name}.required_nodes must be absolute"
            )
        for index, raw_spec in enumerate(
            _list(group.get("topics"), f"topic_groups.{group_name}.topics")
        ):
            spec = _validate_topic_spec(
                raw_spec, f"topic_groups.{group_name}.topics[{index}]"
            )
            for topic in expand_pattern(str(spec["pattern"]), views):
                previous = expanded_owner.get(topic)
                if previous is not None:
                    raise AcceptanceContractError(
                        f"topic {topic} belongs to both {previous} and {group_name}"
                    )
                expanded_owner[topic] = group_name

    gates = _mapping(
        contract.get("optional_consumer_gates"), "optional_consumer_gates"
    )
    if set(gates) != REQUIRED_CONSUMER_GATES:
        raise AcceptanceContractError("optional consumer gate inventory changed")
    for gate_name, raw_gate in gates.items():
        gate = _mapping(raw_gate, f"optional_consumer_gates.{gate_name}")
        for index, raw_spec in enumerate(
            _list(gate.get("topics"), f"optional_consumer_gates.{gate_name}.topics")
        ):
            spec = _mapping(
                raw_spec, f"optional_consumer_gates.{gate_name}.topics[{index}]"
            )
            pattern = str(spec.get("pattern", ""))
            if not pattern.startswith("/"):
                raise AcceptanceContractError(
                    f"optional consumer topic in {gate_name} must be absolute"
                )
            subscribers = [str(value) for value in spec.get("subscribers") or []]
            if not subscribers or any(
                not value.startswith("/") for value in subscribers
            ):
                raise AcceptanceContractError(
                    f"optional consumer subscribers in {gate_name} are invalid"
                )

    forbidden = expand_forbidden_topics(contract)
    overlap = sorted(set(forbidden) & set(expanded_owner))
    if overlap:
        raise AcceptanceContractError(
            "required topics are also default-forbidden: {}".format(overlap)
        )

    bindings = _list(contract.get("static_bindings"), "static_bindings")
    binding_ids = set()
    for index, raw_binding in enumerate(bindings):
        binding = _mapping(raw_binding, f"static_bindings[{index}]")
        binding_id = str(binding.get("id", ""))
        if not binding_id or binding_id in binding_ids:
            raise AcceptanceContractError(f"duplicate static binding {binding_id!r}")
        binding_ids.add(binding_id)
        relative = str(binding.get("file", ""))
        path = repo_root / relative
        if not path.is_file():
            raise AcceptanceContractError(
                f"static binding file is missing: {relative}"
            )
        body = path.read_text(encoding="utf-8")
        for anchor in _list(binding.get("anchors"), f"{binding_id}.anchors"):
            text = str(anchor)
            if not text or text not in body:
                raise AcceptanceContractError(
                    f"static binding {binding_id} has stale anchor: {text!r}"
                )

    deferred = _list(contract.get("deferred_components"), "deferred_components")
    deferred_topics = set()
    for index, raw_component in enumerate(deferred):
        component = _mapping(raw_component, f"deferred_components[{index}]")
        if not str(component.get("reason", "")).strip():
            raise AcceptanceContractError("deferred component needs a reason")
        for topic in _list(
            component.get("topics"), f"deferred_components[{index}].topics"
        ):
            deferred_topics.add(str(topic))
    conflict = sorted(deferred_topics & set(expanded_owner))
    if conflict:
        raise AcceptanceContractError(
            "deferred topics cannot be runtime requirements: {}".format(conflict)
        )

    closeout_result = _validate_p5_closeout(repo_root, contract)
    return {
        "contract_id": CONTRACT_ID,
        "topic_group_count": len(groups),
        "camera_lane_topic_count": len(
            expand_topic_specs(contract, ["camera_lane"])
        ),
        "all_topic_count": len(expanded_owner),
        "default_forbidden_topic_count": len(forbidden),
        "static_binding_count": len(bindings),
        "optional_consumer_gate_count": len(gates),
        "deferred_topic_count": len(deferred_topics),
        "p5_closeout": closeout_result,
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repo-root", type=Path, default=Path(__file__).resolve().parents[2]
    )
    parser.add_argument(
        "--contract",
        type=Path,
        default=Path("config/contracts/perception_runtime_acceptance_v1.yaml"),
    )
    args = parser.parse_args(argv)
    repo_root = args.repo_root.resolve()
    contract_path = args.contract
    if not contract_path.is_absolute():
        contract_path = repo_root / contract_path
    try:
        result = validate_contract(repo_root, contract_path)
    except (AcceptanceContractError, OSError, ValueError, yaml.YAMLError) as exc:
        print(f"perception runtime acceptance validation failed: {exc}")
        return 1
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
