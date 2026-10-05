# -*- coding: utf-8 -*-
"""Strict rosbag profile loading and deployment/GT separation."""

from __future__ import print_function

import copy
import os

import yaml


PRIVILEGED_TOPICS = frozenset(
    (
        "/Object_topic",
        "/velodyne_points_semantic",
        "/velodyne_points_instance",
        "/morai/gt/lidar_instance_raw",
        "/sem_front/image_jpeg/compressed",
        "/sem_left/image_jpeg/compressed",
        "/sem_right/image_jpeg/compressed",
    )
)


class BagProfileError(ValueError):
    pass


def privileged_topics_present(topics):
    listed = set(topics)
    return PRIVILEGED_TOPICS.intersection(listed).union(
        topic for topic in listed
        if topic.startswith("/sem_") or topic.startswith("/morai/gt/")
    )


def _merge(base, overlay):
    result = copy.deepcopy(base)
    for key, value in (overlay or {}).items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def _profile_path(profile_dir, name):
    safe = str(name or "").strip()
    if not safe or os.path.basename(safe) != safe:
        raise BagProfileError("profile must be a simple file stem")
    if safe.endswith(".yaml"):
        safe = safe[:-5]
    return os.path.join(profile_dir, safe + ".yaml")


def load_bag_profile(profile_dir, name, _seen=None):
    """Load one profile, resolving a local ``extends`` chain."""
    profile_dir = os.path.abspath(profile_dir)
    path = _profile_path(profile_dir, name)
    if not os.path.isfile(path):
        raise BagProfileError("bag profile not found: {}".format(path))
    seen = set(_seen or ())
    stem = os.path.splitext(os.path.basename(path))[0]
    if stem in seen:
        raise BagProfileError("cyclic bag profile extends: {}".format(stem))
    seen.add(stem)
    with open(path, "r") as stream:
        raw = yaml.safe_load(stream) or {}
    child_has_name = "name" in raw
    parent = raw.pop("extends", None)
    if parent:
        raw = _merge(load_bag_profile(profile_dir, parent, seen), raw)
    if not child_has_name:
        raw["name"] = stem
    raw["resolved_from"] = path
    validate_bag_profile(raw)
    return raw


def _topic_list(profile, key):
    value = profile.get(key) or []
    if not isinstance(value, list):
        raise BagProfileError("{} must be a list".format(key))
    topics = [str(item).strip() for item in value]
    if any(not item.startswith("/") for item in topics):
        raise BagProfileError("all {} entries must be absolute ROS topics".format(key))
    if len(topics) != len(set(topics)):
        raise BagProfileError("{} contains duplicate topics".format(key))
    return topics


def validate_bag_profile(profile):
    required = _topic_list(profile, "required_topics")
    optional = _topic_list(profile, "optional_topics")
    overlap = set(required).intersection(optional)
    if overlap:
        raise BagProfileError(
            "topics cannot be both required and optional: {}".format(sorted(overlap))
        )
    kind = str(profile.get("kind", "raw"))
    if kind not in ("raw", "result"):
        raise BagProfileError("kind must be raw or result")
    transport = str(profile.get("transport", "udp"))
    if transport not in ("udp", "rosbridge", "replay"):
        raise BagProfileError("transport must be udp, rosbridge, or replay")
    timestamp_mode = str(profile.get("timestamp_mode", "receive"))
    if timestamp_mode not in ("receive", "source", "replay"):
        raise BagProfileError("timestamp_mode must be receive, source, or replay")

    privileged = bool(profile.get("privileged_gt", False))
    deployment_allowed = bool(profile.get("deployment_allowed", not privileged))
    listed = required + optional
    present_gt = privileged_topics_present(listed)
    if present_gt and not privileged:
        raise BagProfileError(
            "non-GT profile contains privileged topics: {}".format(sorted(present_gt))
        )
    if privileged and deployment_allowed:
        raise BagProfileError("privileged_gt profile cannot be deployment_allowed")
    expected = profile.get("expected_types") or {}
    unknown_expected = set(expected).difference(required + optional)
    if unknown_expected:
        raise BagProfileError(
            "expected_types contains unlisted topics: {}".format(
                sorted(unknown_expected)
            )
        )

    profile["required_topics"] = required
    profile["optional_topics"] = optional
    profile["kind"] = kind
    profile["transport"] = transport
    profile["timestamp_mode"] = timestamp_mode
    profile["privileged_gt"] = privileged
    profile["deployment_allowed"] = deployment_allowed
    profile["compression"] = str(profile.get("compression", "none"))
    if profile["compression"] not in ("none", "lz4"):
        raise BagProfileError("compression must be none or lz4")
    return profile
