#!/usr/bin/env python3
"""Count MORAI bridge topics with wall-clock timing and fail on missing inputs."""

import argparse
import bisect
import json
import math
import sys
import threading
import time
from pathlib import Path


COMPETITION_COMMON_REQUIRED_TOPICS = (
    "/Ego_topic",
    "/image_jpeg/compressed",
    "/cam_left/image_jpeg/compressed",
    "/cam_right/image_jpeg/compressed",
    "/cam_extra/image_jpeg/compressed",
    "/velodyne_points",
    "/gps",
    "/imu",
    "/vehicle/state",
)

DEVELOPMENT_EXTRA_TOPICS = (
    "/Object_topic",
    "/sem_front/image_jpeg/compressed",
    "/sem_left/image_jpeg/compressed",
    "/sem_right/image_jpeg/compressed",
    "/velodyne_points_instance",
)

COMPETITION_FORBIDDEN_TOPICS = (
    "/Object_topic",
    "/sem_front/image_jpeg/compressed",
    "/sem_left/image_jpeg/compressed",
    "/sem_right/image_jpeg/compressed",
    "/velodyne_points_instance",
)

NOMINAL_RATES_HZ = {
    "/Ego_topic": 50.0,
    "/Competition_topic": 50.0,
    "/Object_topic": 50.0,
    "/image_jpeg/compressed": 20.0,
    "/cam_left/image_jpeg/compressed": 20.0,
    "/cam_right/image_jpeg/compressed": 20.0,
    "/cam_extra/image_jpeg/compressed": 10.0,
    "/sem_front/image_jpeg/compressed": 20.0,
    "/sem_left/image_jpeg/compressed": 20.0,
    "/sem_right/image_jpeg/compressed": 20.0,
    "/velodyne_points": 10.0,
    "/velodyne_points_instance": 10.0,
    "/gps": 5.0,
    "/imu": 50.0,
    "/vehicle/state": 50.0,
}


def default_required_topics(profile, require_competition_status=False):
    """Build a capability-based topic gate, independent of MORAI release."""
    normalized = {"runtime": "competition", "gt": "development"}.get(
        profile, profile
    )
    if normalized not in ("competition", "development"):
        raise ValueError("profile must be competition or development")
    topics = COMPETITION_COMMON_REQUIRED_TOPICS
    if require_competition_status:
        topics += ("/Competition_topic",)
    if normalized == "development":
        topics += DEVELOPMENT_EXTRA_TOPICS
    return normalized, topics


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--min-messages", type=int, default=2)
    parser.add_argument(
        "--profile",
        choices=("competition", "development", "runtime", "gt"),
        default="competition",
        help=("competition checks the deployment allowlist and absence of privileged "
              "publishers; development adds Object/Semantic/Instance. runtime/gt are aliases"),
    )
    parser.add_argument(
        "--required-topic",
        action="append",
        dest="required_topics",
        help="replace defaults by specifying one or more required topics",
    )
    parser.add_argument("--json-out")
    parser.add_argument(
        "--strict-thresholds", action="store_true",
        help="also fail the documented GT rate and source-pairing gates",
    )
    parser.add_argument(
        "--lidar-target-hz", type=float,
        help=("optional exact-arrival diagnostic; do not use this to validate the "
              "v3 configured/consumer 15 Hz requirement because complete scans may "
              "arrive at about 12 Hz"),
    )
    parser.add_argument(
        "--simulator-release",
        choices=("molitcomp03", "26r1"),
        default="molitcomp03",
        help=("provenance label only; release does not prove that optional "
              "Competition Vehicle Status is emitted"),
    )
    parser.add_argument(
        "--require-competition-status",
        action="store_true",
        help=("field-only gate: require type-787 /Competition_topic in addition "
              "to the participant runtime topics"),
    )
    return parser.parse_args()


def percentile(values, percent):
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * float(percent) / 100.0
    low, high = int(math.floor(index)), int(math.ceil(index))
    if low == high:
        return ordered[low]
    return ordered[low] * (high - index) + ordered[high] * (index - low)


def nearest_p95_ms(left, right):
    if not left or not right:
        return None
    right = sorted(right)
    gaps = []
    for stamp in left:
        index = bisect.bisect_left(right, stamp)
        candidates = []
        if index < len(right):
            candidates.append(abs(stamp - right[index]))
        if index:
            candidates.append(abs(stamp - right[index - 1]))
        gaps.append(min(candidates) * 1000.0)
    return percentile(gaps, 95)


def interval_summary(arrivals):
    """Describe subscriber arrival gaps without claiming network packet loss."""
    if len(arrivals) < 2:
        return {
            "samples": 0, "p50_ms": None, "p95_ms": None, "max_ms": None,
            "jitter_p95_ms": None, "gap_events": 0,
            "long_gap_equivalent_periods": 0,
        }
    intervals = [
        (right - left) * 1000.0
        for left, right in zip(arrivals[:-1], arrivals[1:])
        if right >= left
    ]
    median = percentile(intervals, 50)
    jitter = [abs(value - median) for value in intervals]
    threshold = median * 1.5 if median and median > 0 else float("inf")
    gap_values = [value for value in intervals if value > threshold]
    long_gap_equivalent_periods = sum(
        max(0, int(round(value / median)) - 1) for value in gap_values
    ) if median and median > 0 else 0
    return {
        "samples": len(intervals),
        "p50_ms": round(median, 3),
        "p95_ms": round(percentile(intervals, 95), 3),
        "max_ms": round(max(intervals), 3),
        "jitter_p95_ms": round(percentile(jitter, 95), 3),
        "gap_events": len(gap_values),
        "long_gap_equivalent_periods": long_gap_equivalent_periods,
    }


def main():
    args = parse_args()
    if args.duration <= 0:
        raise SystemExit("--duration must be positive")
    if args.min_messages < 1:
        raise SystemExit("--min-messages must be at least 1")
    if args.lidar_target_hz is not None and args.lidar_target_hz <= 0:
        raise SystemExit("--lidar-target-hz must be positive")

    try:
        import rospy
        from asmc_msgs.msg import VehicleState
        from morai_msgs.msg import EgoVehicleStatus, GPSMessage, ObjectStatusList
        from diagnostic_msgs.msg import DiagnosticArray
        from sensor_msgs.msg import CompressedImage, Imu, PointCloud2
    except ImportError as exc:
        raise SystemExit(
            "ROS message packages are required; run this inside asmc-ros-noetic"
        ) from exc

    profile, defaults = default_required_topics(
        args.profile, args.require_competition_status
    )
    topics = tuple(args.required_topics or defaults)
    competition_status_required = "/Competition_topic" in topics
    lock = threading.Lock()
    samples = {topic: [] for topic in topics}
    source_stamps = {topic: [] for topic in topics}
    header_ages_ms = {topic: [] for topic in topics}
    source_regressions = {topic: 0 for topic in topics}
    pointcloud_fields = {}
    valid_vehicle_states = 0
    bridge_diagnostics = {}
    started = time.monotonic()

    def callback(message, topic_name):
        nonlocal valid_vehicle_states
        now = time.monotonic()
        wall_now = time.time()
        with lock:
            samples[topic_name].append(now)
            if hasattr(message, "header") and message.header.stamp.to_nsec() > 0:
                stamp = message.header.stamp.to_sec()
                if source_stamps[topic_name] and stamp < source_stamps[topic_name][-1]:
                    source_regressions[topic_name] += 1
                source_stamps[topic_name].append(stamp)
                age_ms = (wall_now - stamp) * 1000.0
                if 0.0 <= age_ms <= 60000.0:
                    header_ages_ms[topic_name].append(age_ms)
            if topic_name == "/velodyne_points_instance" and hasattr(message, "fields"):
                pointcloud_fields[topic_name] = {
                    field.name: field.datatype for field in message.fields
                }
            if topic_name == "/vehicle/state" and getattr(message, "valid", False):
                valid_vehicle_states += 1

    def diagnostics_callback(message, topic_name):
        values = {}
        for status in message.status:
            values[status.name] = {
                "level": int(status.level),
                "message": status.message,
                "values": dict((item.key, item.value) for item in status.values),
            }
        with lock:
            bridge_diagnostics[topic_name] = values

    rospy.init_node("morai_host_network_verifier", anonymous=True, disable_signals=True)
    subscribers = []
    typed_messages = {
        "/Ego_topic": EgoVehicleStatus,
        "/Competition_topic": EgoVehicleStatus,
        "/Object_topic": ObjectStatusList,
        "/gps": GPSMessage,
        "/imu": Imu,
        "/velodyne_points": PointCloud2,
        "/velodyne_points_instance": PointCloud2,
        "/vehicle/state": VehicleState,
    }
    for topic in topics:
        message_type = (
            CompressedImage if topic.endswith("/compressed")
            else typed_messages.get(topic, rospy.AnyMsg)
        )
        subscribers.append(
            rospy.Subscriber(
                topic, message_type, callback, callback_args=topic, queue_size=1
            )
        )
    for diagnostic_topic in (
        "/morai_sensor_bridge/diagnostics",
        "/morai_udp_bridge/diagnostics",
    ):
        subscribers.append(
            rospy.Subscriber(
                diagnostic_topic, DiagnosticArray, diagnostics_callback,
                callback_args=diagnostic_topic, queue_size=1,
            )
        )

    deadline = started + args.duration
    while time.monotonic() < deadline and not rospy.is_shutdown():
        time.sleep(0.05)

    finished = time.monotonic()
    report = {
        "stream_profile": profile,
        "simulator_release": args.simulator_release,
        "competition_status_required": competition_status_required,
        "duration_seconds": round(finished - started, 6),
        "min_messages": args.min_messages,
        "topics": {},
    }
    failures = []
    missing_topics = []
    nominal_rates = dict(NOMINAL_RATES_HZ)
    if args.lidar_target_hz is not None:
        nominal_rates["/velodyne_points"] = args.lidar_target_hz
        nominal_rates["/velodyne_points_instance"] = args.lidar_target_hz
    with lock:
        snapshot = {topic: list(values) for topic, values in samples.items()}

    published_topics = dict(rospy.get_published_topics())
    forbidden_publishers = (
        sorted(
            topic for topic in COMPETITION_FORBIDDEN_TOPICS
            if topic in published_topics
        )
        if profile == "competition" else []
    )
    report["forbidden_topic_publishers"] = forbidden_publishers
    report["valid_vehicle_state_messages"] = valid_vehicle_states
    if forbidden_publishers:
        failures.append("privileged:" + ",".join(forbidden_publishers))
        print(
            "[FAIL] competition profile exposes privileged publishers: "
            + ", ".join(forbidden_publishers)
        )
    if "/vehicle/state" in topics and valid_vehicle_states == 0:
        failures.append("/vehicle/state:valid")
        print("[FAIL] /vehicle/state was never valid during the measurement window")

    print("TOPIC                                             COUNT   RATE(Hz)   STATUS")
    for topic in topics:
        values = snapshot[topic]
        count = len(values)
        rate = 0.0
        if count >= 2 and values[-1] > values[0]:
            rate = (count - 1) / (values[-1] - values[0])
        ok = count >= args.min_messages
        if not ok:
            failures.append(topic)
            missing_topics.append(topic)
        report["topics"][topic] = {
            "count": count,
            "rate_hz": round(rate, 3),
            "nominal_rate_hz": nominal_rates.get(topic),
            "nominal_rate_ratio": (
                None if not nominal_rates.get(topic)
                else round(rate / nominal_rates[topic], 4)
            ),
            "arrival_intervals": interval_summary(values),
            "header_age_ms": {
                "samples": len(header_ages_ms[topic]),
                "p50": (
                    None if not header_ages_ms[topic]
                    else round(percentile(header_ages_ms[topic], 50), 3)
                ),
                "p95": (
                    None if not header_ages_ms[topic]
                    else round(percentile(header_ages_ms[topic], 95), 3)
                ),
                "max": (
                    None if not header_ages_ms[topic]
                    else round(max(header_ages_ms[topic]), 3)
                ),
            },
            "source_stamp_regressions": source_regressions[topic],
            "ok": ok,
        }
        print("{:<49} {:>5} {:>10.3f}   {}".format(topic, count, rate, "OK" if ok else "MISSING"))

    for subscriber in subscribers:
        subscriber.unregister()

    report["ok"] = not failures
    report["missing_topics"] = missing_topics
    report["bridge_diagnostics"] = bridge_diagnostics
    report["interpretation"] = {
        "nominal_rate_shortfall": (
            "configured/nominal rate shortfall is not proof of UDP packet loss"
        ),
        "long_gap_equivalent_periods": (
            "subscriber-visible long-gap duration expressed as median periods; it "
            "is jitter/burst evidence, not a dropped-frame count"
        ),
        "header_age_ms": (
            "ROS publish-to-subscriber age in receive timestamp mode; source-to-"
            "subscriber age in source timestamp mode"
        ),
    }
    if args.lidar_target_hz is not None and "/velodyne_points" in report["topics"]:
        lidar_rate = report["topics"]["/velodyne_points"]["rate_hz"]
        lidar_target_ok = abs(lidar_rate - args.lidar_target_hz) <= 1.0
        report["lidar_target_gate"] = {
            "target_hz": args.lidar_target_hz,
            "tolerance_hz": 1.0,
            "measured_hz": lidar_rate,
            "ok": lidar_target_ok,
        }
        if args.strict_thresholds and not lidar_target_ok:
            failures.append("threshold:lidar_target_hz")
            report["ok"] = False
    if profile == "development":
        schema = pointcloud_fields.get("/velodyne_points_instance", {})
        schema_ok = schema.get("instance_id") == 6 and "intensity" not in schema
        report["instance_lidar_schema"] = {
            "fields": schema,
            "expected": {"instance_id": 6},
            "ok": schema_ok,
        }
        if not schema_ok:
            failures.append("/velodyne_points_instance:schema")
            report["ok"] = False
            print("[FAIL] Instance LiDAR must contain uint32 instance_id and no intensity")
        pairs = {
            "rgb_sem_front": ("/image_jpeg/compressed", "/sem_front/image_jpeg/compressed", 30.0),
            "rgb_sem_left": ("/cam_left/image_jpeg/compressed", "/sem_left/image_jpeg/compressed", 30.0),
            "rgb_sem_right": ("/cam_right/image_jpeg/compressed", "/sem_right/image_jpeg/compressed", 30.0),
            "intensity_instance_lidar": ("/velodyne_points", "/velodyne_points_instance", 50.0),
        }
        report["pairing"] = {}
        threshold_failures = []
        for name, (left, right, limit) in pairs.items():
            p95 = nearest_p95_ms(source_stamps[left], source_stamps[right])
            ok = p95 is not None and p95 <= limit
            report["pairing"][name] = {
                "p95_ms": None if p95 is None else round(p95, 3),
                "limit_ms": limit,
                "ok": ok,
            }
            if not ok:
                threshold_failures.append(name)
        camera_gate_topics = (
            "/image_jpeg/compressed", "/cam_left/image_jpeg/compressed",
            "/cam_right/image_jpeg/compressed", "/sem_front/image_jpeg/compressed",
            "/sem_left/image_jpeg/compressed", "/sem_right/image_jpeg/compressed",
        )
        rates = dict((topic, report["topics"][topic]["rate_hz"]) for topic in topics)
        camera_rate_ok = all(rates[topic] >= 18.0 for topic in camera_gate_topics)
        lidar_rates = [rates["/velodyne_points"], rates["/velodyne_points_instance"]]
        lidar_rate_ok = min(lidar_rates) >= 7.0
        lidar_difference_ok = (
            max(lidar_rates) > 0
            and (max(lidar_rates) - min(lidar_rates)) / max(lidar_rates) <= 0.15
        )
        report["thresholds"] = {
            "camera_rates_at_least_18hz": camera_rate_ok,
            "lidar_rates_at_least_7hz": lidar_rate_ok,
            "lidar_rate_difference_within_15pct": lidar_difference_ok,
            "source_pairing": not threshold_failures,
        }
        if args.strict_thresholds:
            for name, ok in report["thresholds"].items():
                if not ok:
                    failures.append("threshold:" + name)
            report["ok"] = not failures
    report["failures"] = failures
    report["missing_topics"] = missing_topics
    report["ok"] = not failures
    if args.json_out:
        output = Path(args.json_out)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False))
    return 0 if not failures else 2


if __name__ == "__main__":
    sys.exit(main())
