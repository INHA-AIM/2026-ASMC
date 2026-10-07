#!/usr/bin/env python3
"""Validate the P5 perception topic inventory against source and bag profiles."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Sequence, Set

import yaml


class PolicyError(RuntimeError):
    """Raised when the perception topic policy is incomplete or stale."""


REQUIRED_FAMILY_IDS = {
    "camera_inference_status",
    "camera_objects",
    "camera_objects_debug_raw",
    "camera_objects_diagnostics",
    "camera_objects_visualization",
    "camera_semantic_full_resolution",
    "camera_semantic_model_grid",
    "camera_diagnostics",
    "per_view_projected_semantics",
    "per_view_projected_coverage",
    "per_view_projection_diagnostics",
    "fused_bev_semantics",
    "fused_bev_source_count",
    "fused_bev_source_view",
    "fused_bev_diagnostics",
    "lane_boundary_observations",
    "lane_boundary_family_candidate",
    "lane_compatibility_markers",
    "lane_debug_geometry",
    "lane_diagnostics",
    "lane_pipeline_audit",
    "lane_shared_family_diagnostics",
    "lane_shared_family_markers",
    "semantic_visualization",
    "camera_grid_visualization",
    "bev_visualization",
    "lane_visualization",
    "traffic_light_visualization",
    "traffic_light_typed",
    "traffic_light_legacy",
}

ALLOWED_CLASSIFICATIONS = {
    "canonical_runtime",
    "canonical_quality_metadata",
    "required_internal_transport",
    "operational_diagnostics",
    "compatibility",
    "candidate_contract",
    "debug_opt_in",
    "audit_opt_in",
}

ALLOWED_CURRENT_MODES = {
    "default_on",
    "explicit_argument",
    "subscriber_lazy",
    "visualization_launch_and_subscriber_lazy",
}

ALLOWED_ACTIONS = {
    "keep",
    "preserve_until_migration",
    "default_off_p5b",
    "opt_in_p5b",
    "review_after_planner_handoff",
    "defer_owner",
    "retire_after_consumer_gate",
    "retire_after_comparison",
    "consolidate_p5b",
}


def _mapping(value: object, label: str) -> Mapping[str, object]:
    if not isinstance(value, dict):
        raise PolicyError(f"{label} must be a mapping")
    return value


def _list(value: object, label: str) -> List[object]:
    if not isinstance(value, list):
        raise PolicyError(f"{label} must be a list")
    return value


def _read_yaml(path: Path) -> Mapping[str, object]:
    return _mapping(yaml.safe_load(path.read_text(encoding="utf-8")), str(path))


def _assert_anchors(repo_root: Path, raw: object, label: str) -> None:
    value = _mapping(raw, label)
    relative = str(value.get("file", ""))
    if not relative:
        raise PolicyError(f"{label}.file is required")
    path = repo_root / relative
    if not path.is_file():
        raise PolicyError(f"{label} source is missing: {relative}")
    body = path.read_text(encoding="utf-8")
    anchors = value.get("anchors")
    if anchors is None and "anchor" in value:
        anchors = [value["anchor"]]
    for anchor in _list(anchors, f"{label}.anchors"):
        text = str(anchor)
        if not text or text not in body:
            raise PolicyError(f"{label} anchor is stale in {relative}: {text!r}")


def _expand(pattern: str, views: Sequence[str]) -> List[str]:
    if "{view}" in pattern:
        return [pattern.replace("{view}", view) for view in views]
    return [pattern]


def _bag_topic_index(profile_root: Path) -> Dict[str, Set[str]]:
    if not profile_root.is_dir():
        raise PolicyError(f"bag profile root is missing: {profile_root}")
    result: Dict[str, Set[str]] = {}
    for path in sorted(profile_root.glob("*.yaml")):
        profile = _read_yaml(path)
        topics: Iterable[object] = list(profile.get("required_topics") or [])
        topics = list(topics) + list(profile.get("optional_topics") or [])
        for raw_topic in topics:
            result.setdefault(str(raw_topic), set()).add(path.name)
    return result


def validate_policy(repo_root: Path, policy_path: Path) -> Mapping[str, object]:
    policy = _read_yaml(policy_path)
    if policy.get("schema_version") != 1:
        raise PolicyError("perception topic policy schema_version must be 1")
    if policy.get("policy_id") != "asmc_perception_topic_policy_v1":
        raise PolicyError("unexpected perception topic policy_id")
    if policy.get("audit_id") != "p5a_perception_topic_audit_v1":
        raise PolicyError("unexpected perception topic audit_id")
    if policy.get("status") != "frozen_for_p5b":
        raise PolicyError("P5-A policy must be frozen_for_p5b")

    commit = str(policy.get("audited_base_commit", ""))
    if len(commit) != 40 or any(value not in "0123456789abcdef" for value in commit):
        raise PolicyError("audited_base_commit must be a full lowercase Git SHA")

    scope = _mapping(policy.get("scope"), "scope")
    views = [str(value) for value in _list(scope.get("views"), "scope.views")]
    if views != ["front", "left", "right"]:
        raise PolicyError("the audited camera view order must be front,left,right")
    if scope.get("runtime_behavior_changed") is not False:
        raise PolicyError("P5-A must not change runtime behavior")
    exclusions = _list(scope.get("excluded_owners"), "scope.excluded_owners")
    excluded_components = {
        str(_mapping(value, "excluded owner").get("component", ""))
        for value in exclusions
    }
    if "lane_boundary_adapter" not in excluded_components:
        raise PolicyError("planning-owned lane_boundary_adapter exclusion is missing")

    launch_defaults = _mapping(policy.get("launch_defaults"), "launch_defaults")
    _assert_anchors(
        repo_root,
        {
            "file": launch_defaults.get("source"),
            "anchors": launch_defaults.get("anchors"),
        },
        "launch_defaults",
    )

    profile_root = repo_root / str(scope.get("bag_profile_root", ""))
    bag_index = _bag_topic_index(profile_root)
    families = _list(policy.get("topic_families"), "topic_families")
    by_id: Dict[str, Mapping[str, object]] = {}
    topic_owner: Dict[str, str] = {}
    classified_topics: List[str] = []

    for index, raw_family in enumerate(families):
        family = _mapping(raw_family, f"topic_families[{index}]")
        family_id = str(family.get("id", ""))
        if not family_id or family_id in by_id:
            raise PolicyError(f"duplicate or empty topic family id: {family_id!r}")
        by_id[family_id] = family

        classification = str(family.get("classification", ""))
        current_mode = str(family.get("current_mode", ""))
        action = str(family.get("p5b_action", ""))
        if classification not in ALLOWED_CLASSIFICATIONS:
            raise PolicyError(f"{family_id} has invalid classification: {classification}")
        if current_mode not in ALLOWED_CURRENT_MODES:
            raise PolicyError(f"{family_id} has invalid current_mode: {current_mode}")
        if action not in ALLOWED_ACTIONS:
            raise PolicyError(f"{family_id} has invalid P5-B action: {action}")
        if not str(family.get("rationale", "")).strip():
            raise PolicyError(f"{family_id} has no rationale")

        _assert_anchors(repo_root, family.get("producer"), f"{family_id}.producer")
        for consumer_index, consumer in enumerate(
            _list(family.get("consumers"), f"{family_id}.consumers")
        ):
            _assert_anchors(
                repo_root,
                consumer,
                f"{family_id}.consumers[{consumer_index}]",
            )

        expected_profiles = sorted(
            str(value)
            for value in _list(family.get("bag_profiles"), f"{family_id}.bag_profiles")
        )
        if len(expected_profiles) != len(set(expected_profiles)):
            raise PolicyError(f"{family_id} repeats a bag profile")
        for profile in expected_profiles:
            if not (profile_root / profile).is_file():
                raise PolicyError(f"{family_id} names missing bag profile: {profile}")

        for topic_index, raw_topic in enumerate(
            _list(family.get("topics"), f"{family_id}.topics")
        ):
            topic = _mapping(raw_topic, f"{family_id}.topics[{topic_index}]")
            pattern = str(topic.get("pattern", ""))
            type_name = str(topic.get("type", ""))
            if not pattern.startswith("/") or "/" not in type_name:
                raise PolicyError(f"{family_id} has invalid topic/type: {topic}")
            for expanded in _expand(pattern, views):
                previous = topic_owner.get(expanded)
                if previous is not None:
                    raise PolicyError(
                        f"topic {expanded} belongs to both {previous} and {family_id}"
                    )
                topic_owner[expanded] = family_id
                classified_topics.append(expanded)
                observed_profiles = sorted(bag_index.get(expanded, set()))
                if observed_profiles != expected_profiles:
                    raise PolicyError(
                        f"{family_id} bag usage drift for {expanded}: "
                        f"policy={expected_profiles} source={observed_profiles}"
                    )

    if set(by_id) != REQUIRED_FAMILY_IDS:
        raise PolicyError(
            "topic family inventory mismatch: missing={} extra={}".format(
                sorted(REQUIRED_FAMILY_IDS - set(by_id)),
                sorted(set(by_id) - REQUIRED_FAMILY_IDS),
            )
        )

    required_internal = {
        family_id
        for family_id, family in by_id.items()
        if family.get("classification") == "required_internal_transport"
    }
    if required_internal != {
        "camera_semantic_model_grid",
        "per_view_projected_semantics",
    }:
        raise PolicyError("required internal transports were misclassified")
    for family_id in required_internal:
        if by_id[family_id].get("p5b_action") != "preserve_until_migration":
            raise PolicyError(f"{family_id} cannot be disabled as debug output")

    expected_default_off = {
        "per_view_projected_coverage",
        "fused_bev_source_view",
    }
    actual_default_off = {
        family_id
        for family_id, family in by_id.items()
        if family.get("p5b_action") == "default_off_p5b"
    }
    if actual_default_off != expected_default_off:
        raise PolicyError("P5-B default-off candidate set changed")
    if by_id["lane_pipeline_audit"].get("p5b_action") != "opt_in_p5b":
        raise PolicyError("frame-level lane audit must move to explicit evaluation mode")
    if by_id["lane_boundary_family_candidate"].get("p5b_action") != "defer_owner":
        raise PolicyError("planning-owned boundary-family candidate must be deferred")

    order = _list(policy.get("p5b_order"), "p5b_order")
    if len(order) < 5 or any(not str(value).strip() for value in order):
        raise PolicyError("P5-B ordered migration gates are incomplete")

    return {
        "policy_id": policy["policy_id"],
        "audit_id": policy["audit_id"],
        "family_count": len(by_id),
        "expanded_topic_count": len(classified_topics),
        "required_internal_family_count": len(required_internal),
        "default_off_candidate_count": len(actual_default_off),
        "bag_profile_count": len(list(profile_root.glob("*.yaml"))),
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path(__file__).resolve().parents[2],
    )
    parser.add_argument(
        "--policy",
        type=Path,
        default=Path("config/contracts/perception_topic_policy_v1.yaml"),
    )
    args = parser.parse_args(argv)
    repo_root = args.repo_root.resolve()
    policy_path = args.policy
    if not policy_path.is_absolute():
        policy_path = repo_root / policy_path
    try:
        result = validate_policy(repo_root, policy_path)
    except (OSError, PolicyError, yaml.YAMLError) as exc:
        print(f"perception topic policy validation failed: {exc}")
        return 1
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
