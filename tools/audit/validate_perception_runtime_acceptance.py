#!/usr/bin/env python3
"""Validate the P5-C perception live-acceptance contract and source bindings."""

from __future__ import annotations

import argparse
import json
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
