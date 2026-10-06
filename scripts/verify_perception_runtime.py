#!/usr/bin/env python3
"""Measure the live perception ROS graph against the P5-C acceptance contract."""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path
from typing import Dict, List, Mapping, Optional, Sequence

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "tools" / "audit"))

from validate_perception_runtime_acceptance import (  # noqa: E402
    AcceptanceContractError,
    expand_forbidden_topics,
    expand_topic_specs,
    load_contract,
    validate_contract,
)


def percentile(values: Sequence[float], percent: float):
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    index = (len(ordered) - 1) * float(percent) / 100.0
    lower = int(math.floor(index))
    upper = int(math.ceil(index))
    if lower == upper:
        return ordered[lower]
    return ordered[lower] * (upper - index) + ordered[upper] * (index - lower)


def interval_summary(arrivals: Sequence[float]) -> Mapping[str, object]:
    intervals = [
        (right - left) * 1000.0
        for left, right in zip(arrivals[:-1], arrivals[1:])
        if right >= left
    ]
    if not intervals:
        return {"samples": 0, "p50_ms": None, "p95_ms": None, "max_ms": None}
    return {
        "samples": len(intervals),
        "p50_ms": round(percentile(intervals, 50), 3),
        "p95_ms": round(percentile(intervals, 95), 3),
        "max_ms": round(max(intervals), 3),
    }


def snapshot_samples(samples: Mapping[str, Mapping[str, object]]):
    """Copy callback-owned values without changing scalar counter types."""
    return {
        topic: {
            key: (
                sorted(value)
                if isinstance(value, set)
                else list(value)
                if isinstance(value, list)
                else value
            )
            for key, value in values.items()
        }
        for topic, values in samples.items()
    }


def call_with_timeout(label, callback, timeout_sec: float):
    """Run a ROS-master XML-RPC call without allowing an infinite hang."""
    result = {}
    failure = {}

    def invoke():
        try:
            result["value"] = callback()
        except Exception as exc:  # ROS XML-RPC raises several exception types.
            failure["error"] = exc

    worker = threading.Thread(target=invoke, daemon=True)
    worker.start()
    worker.join(float(timeout_sec))
    if worker.is_alive():
        raise TimeoutError(f"{label} timed out after {timeout_sec:.1f}s")
    if "error" in failure:
        raise RuntimeError(f"{label} failed: {failure['error']}") from failure[
            "error"
        ]
    return result.get("value")


def parse_args(argv: Optional[Sequence[str]] = None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--contract",
        type=Path,
        default=Path("config/contracts/perception_runtime_acceptance_v1.yaml"),
    )
    parser.add_argument("--duration", type=float)
    parser.add_argument("--max-stale-sec", type=float, default=1.0)
    parser.add_argument("--master-timeout-sec", type=float, default=3.0)
    parser.add_argument("--camera-objects", action="store_true",
                        help="require the opt-in front object producer and diagnostics")
    parser.add_argument("--no-camera-lane", action="store_true",
                        help="check selected producers without requiring lane inference")
    parser.add_argument(
        "--traffic-light",
        action="store_true",
        help="require the typed and compatibility traffic-light producers",
    )
    parser.add_argument(
        "--visualization",
        action="store_true",
        help="subscribe to and require the two canonical rqt compressed topics",
    )
    parser.add_argument(
        "--require-planning",
        action="store_true",
        help="require behavior_planner subscriptions to typed and legacy traffic light",
    )
    parser.add_argument(
        "--require-lane-adapter",
        action="store_true",
        help="require the read-only shared-family adapter subscription",
    )
    parser.add_argument(
        "--allow-opt-in-debug",
        action="store_true",
        help="report but do not fail publishers that are default-off in production",
    )
    parser.add_argument("--yaml-out", type=Path)
    return parser.parse_args(argv)


def _consumer_requirements(contract, gate_names):
    gates = contract["optional_consumer_gates"]
    requirements: Dict[str, List[str]] = {}
    required_nodes = set()
    for gate_name in gate_names:
        for spec in gates[gate_name]["topics"]:
            topic = str(spec["pattern"])
            subscribers = [str(value) for value in spec["subscribers"]]
            requirements.setdefault(topic, []).extend(subscribers)
            required_nodes.update(subscribers)
    return requirements, required_nodes


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    contract_path = args.contract
    if not contract_path.is_absolute():
        contract_path = REPO_ROOT / contract_path
    try:
        static_result = validate_contract(REPO_ROOT, contract_path)
        contract = load_contract(contract_path)
    except (AcceptanceContractError, OSError, ValueError, yaml.YAMLError) as exc:
        print(f"[FAIL] static perception contract: {exc}")
        return 2

    duration = float(
        args.duration
        if args.duration is not None
        else contract["default_duration_sec"]
    )
    if (
        duration <= 0.0
        or args.max_stale_sec <= 0.0
        or args.master_timeout_sec <= 0.0
    ):
        raise SystemExit(
            "--duration, --max-stale-sec and --master-timeout-sec must be positive"
        )
    if args.require_planning and not args.traffic_light:
        raise SystemExit("--require-planning also requires --traffic-light")

    if args.require_lane_adapter and args.no_camera_lane:
        raise SystemExit("--require-lane-adapter cannot use --no-camera-lane")
    groups = [] if args.no_camera_lane else ["camera_lane"]
    if args.camera_objects:
        groups.append("camera_objects")
    if args.traffic_light:
        groups.append("traffic_light")
    if args.visualization:
        groups.append("visualization")
    if not groups:
        raise SystemExit("select a producer when using --no-camera-lane")
    topic_specs = expand_topic_specs(contract, groups)
    specs_by_topic = {str(spec["topic"]): spec for spec in topic_specs}

    consumer_gates = []
    if args.require_planning:
        consumer_gates.append("planning_traffic_light")
    if args.require_lane_adapter:
        consumer_gates.append("lane_adapter")
    extra_subscribers, consumer_nodes = _consumer_requirements(
        contract, consumer_gates
    )

    required_nodes = set(consumer_nodes)
    for group_name in groups:
        required_nodes.update(
            str(value)
            for value in contract["topic_groups"][group_name]["required_nodes"]
        )

    try:
        import rosgraph
        import roslib.message
        import rosnode
        import rospy
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
    except ImportError as exc:
        raise SystemExit(
            "ROS Python packages are required; run through scripts/verify_perception.sh"
        ) from exc

    try:
        call_with_timeout(
            "ROS master preflight",
            rosgraph.Master("/perception_runtime_acceptance_preflight").getPid,
            args.master_timeout_sec,
        )
    except (RuntimeError, TimeoutError) as exc:
        print(f"[FAIL] {exc}")
        return 2

    rospy.init_node(
        "perception_runtime_acceptance", anonymous=True, disable_signals=True
    )
    lock = threading.Lock()
    samples = {
        topic: {
            "arrivals": [],
            "source_stamps_ns": [],
            "zero_stamps": 0,
            "stamp_regressions": 0,
            "frame_ids": set(),
            "diagnostic_errors": [],
        }
        for topic in specs_by_topic
    }

    def callback(message, topic):
        now = time.monotonic()
        with lock:
            sample = samples[topic]
            sample["arrivals"].append(now)
            header = getattr(message, "header", None)
            if header is not None:
                stamp_ns = int(header.stamp.to_nsec())
                if stamp_ns <= 0:
                    sample["zero_stamps"] += 1
                else:
                    previous = sample["source_stamps_ns"]
                    if previous and stamp_ns < previous[-1]:
                        sample["stamp_regressions"] += 1
                    previous.append(stamp_ns)
                sample["frame_ids"].add(str(header.frame_id))
            if isinstance(message, DiagnosticArray):
                for status in message.status:
                    if int(status.level) >= int(DiagnosticStatus.ERROR):
                        sample["diagnostic_errors"].append(
                            {"name": status.name, "message": status.message}
                        )

    subscribers = []
    unresolved_types = []
    for topic, spec in specs_by_topic.items():
        message_class = roslib.message.get_message_class(str(spec["type"]))
        if message_class is None:
            unresolved_types.append(str(spec["type"]))
            continue
        subscribers.append(
            rospy.Subscriber(
                topic, message_class, callback, callback_args=topic, queue_size=1
            )
        )
    if unresolved_types:
        print(
            "[FAIL] unresolved ROS message types: "
            + ", ".join(sorted(set(unresolved_types)))
        )
        return 2

    started = time.monotonic()
    deadline = started + duration
    while time.monotonic() < deadline and not rospy.is_shutdown():
        time.sleep(0.05)
    finished = time.monotonic()

    master = rosgraph.Master(rospy.get_name())
    try:
        publishers_state, subscribers_state, _services = call_with_timeout(
            "ROS master system-state query",
            master.getSystemState,
            args.master_timeout_sec,
        )
        published_topics = call_with_timeout(
            "ROS master published-topic query",
            rospy.get_published_topics,
            args.master_timeout_sec,
        )
        node_names = call_with_timeout(
            "ROS master node query",
            rosnode.get_node_names,
            args.master_timeout_sec,
        )
    except (RuntimeError, TimeoutError) as exc:
        for subscriber in subscribers:
            subscriber.unregister()
        print(f"[FAIL] {exc}")
        return 2
    publishers_by_topic = {
        topic: sorted(nodes) for topic, nodes in publishers_state
    }
    subscribers_by_topic = {
        topic: sorted(nodes) for topic, nodes in subscribers_state
    }
    published_types = dict(published_topics)
    live_nodes = set(node_names)

    failures = []
    missing_nodes = sorted(required_nodes - live_nodes)
    if missing_nodes:
        failures.extend("node:" + value for value in missing_nodes)

    report = {
        "contract_id": contract["contract_id"],
        "duration_sec": round(finished - started, 3),
        "groups": groups,
        "consumer_gates": consumer_gates,
        "static_validation": static_result,
        "missing_nodes": missing_nodes,
        "topics": {},
    }

    print("TOPIC                                                       COUNT   RATE    STATUS")
    with lock:
        snapshot = snapshot_samples(samples)
    for topic in sorted(specs_by_topic):
        spec = specs_by_topic[topic]
        sample = snapshot[topic]
        arrivals = sample["arrivals"]
        count = len(arrivals)
        rate_hz = (
            (count - 1) / (arrivals[-1] - arrivals[0])
            if count >= 2 and arrivals[-1] > arrivals[0]
            else 0.0
        )
        topic_failures = []
        minimum_messages = int(spec.get("min_messages", 2))
        if count < minimum_messages:
            topic_failures.append("messages")
        minimum_rate = spec.get("min_rate_hz")
        if minimum_rate is not None and rate_hz < float(minimum_rate):
            topic_failures.append("rate")
        observed_type = published_types.get(topic)
        if observed_type != spec["type"]:
            topic_failures.append("type")
        publishers = publishers_by_topic.get(topic, [])
        if publishers != [str(spec["publisher"])]:
            topic_failures.append("publisher")
        expected_subscribers = set(str(value) for value in spec.get("subscribers") or [])
        expected_subscribers.update(extra_subscribers.get(topic, []))
        observed_subscribers = set(subscribers_by_topic.get(topic, []))
        if not expected_subscribers.issubset(observed_subscribers):
            topic_failures.append("subscriber")
        if spec.get("require_nonzero_stamp") and sample["zero_stamps"]:
            topic_failures.append("zero_stamp")
        if sample["stamp_regressions"]:
            topic_failures.append("stamp_regression")
        expected_frame = spec.get("frame_id")
        if expected_frame is not None and sample["frame_ids"] != [expected_frame]:
            topic_failures.append("frame_id")
        stale_sec = finished - arrivals[-1] if arrivals else None
        topic_max_stale_sec = float(
            spec.get("max_stale_sec", args.max_stale_sec)
        )
        if stale_sec is None or stale_sec > topic_max_stale_sec:
            topic_failures.append("stale")
        if sample["diagnostic_errors"]:
            topic_failures.append("diagnostic_error")
        failures.extend(f"{topic}:{value}" for value in topic_failures)
        report["topics"][topic] = {
            "type_expected": spec["type"],
            "type_observed": observed_type,
            "publishers": publishers,
            "subscribers": sorted(observed_subscribers),
            "expected_subscribers": sorted(expected_subscribers),
            "count": count,
            "rate_hz": round(rate_hz, 3),
            "min_rate_hz": minimum_rate,
            "arrival_intervals": interval_summary(arrivals),
            "last_arrival_age_sec": None if stale_sec is None else round(stale_sec, 3),
            "max_stale_sec": topic_max_stale_sec,
            "frame_ids": sample["frame_ids"],
            "zero_stamps": sample["zero_stamps"],
            "stamp_regressions": sample["stamp_regressions"],
            "diagnostic_errors": sample["diagnostic_errors"],
            "failures": topic_failures,
            "ok": not topic_failures,
        }
        print(
            "{:<59} {:>5} {:>7.2f}  {}".format(
                topic,
                count,
                rate_hz,
                "OK" if not topic_failures else "FAIL:" + ",".join(topic_failures),
            )
        )

    forbidden = expand_forbidden_topics(contract)
    advertised_forbidden = sorted(
        topic for topic in forbidden if topic in published_types
    )
    report["default_forbidden_publishers"] = advertised_forbidden
    report["allow_opt_in_debug"] = bool(args.allow_opt_in_debug)
    if advertised_forbidden and not args.allow_opt_in_debug:
        failures.extend("default_off:" + topic for topic in advertised_forbidden)

    report["failures"] = failures
    report["ok"] = not failures
    if args.yaml_out is not None:
        output = args.yaml_out
        if not output.is_absolute():
            output = REPO_ROOT / output
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            yaml.safe_dump(report, allow_unicode=True, sort_keys=False),
            encoding="utf-8",
        )
        print(f"report: {output}")

    for subscriber in subscribers:
        subscriber.unregister()

    if missing_nodes:
        print("[FAIL] missing nodes: " + ", ".join(missing_nodes))
    if advertised_forbidden:
        label = "INFO" if args.allow_opt_in_debug else "FAIL"
        print(
            f"[{label}] default-off publishers: "
            + ", ".join(advertised_forbidden)
        )
    print(json.dumps({"ok": report["ok"], "failures": failures}, sort_keys=True))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
