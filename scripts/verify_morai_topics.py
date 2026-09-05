#!/usr/bin/env python3
"""Count MORAI bridge topics with wall-clock timing and fail on missing inputs."""

import argparse
import json
import sys
import threading
import time
from pathlib import Path


DEFAULT_REQUIRED_TOPICS = (
    "/Ego_topic",
    "/image_jpeg/compressed",
    "/cam_left/image_jpeg/compressed",
    "/cam_right/image_jpeg/compressed",
    "/cam_extra/image_jpeg/compressed",
    "/velodyne_points",
    "/gps",
    "/imu",
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--min-messages", type=int, default=2)
    parser.add_argument(
        "--required-topic",
        action="append",
        dest="required_topics",
        help="replace defaults by specifying one or more required topics",
    )
    parser.add_argument("--json-out")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.duration <= 0:
        raise SystemExit("--duration must be positive")
    if args.min_messages < 1:
        raise SystemExit("--min-messages must be at least 1")

    try:
        import rospy
    except ImportError as exc:
        raise SystemExit("rospy is required; run this inside asmc-ros-noetic") from exc

    topics = tuple(args.required_topics or DEFAULT_REQUIRED_TOPICS)
    lock = threading.Lock()
    samples = {topic: [] for topic in topics}
    started = time.monotonic()

    def callback(_message, topic_name):
        now = time.monotonic()
        with lock:
            samples[topic_name].append(now)

    rospy.init_node("morai_host_network_verifier", anonymous=True, disable_signals=True)
    subscribers = [
        rospy.Subscriber(
            topic,
            rospy.AnyMsg,
            callback,
            callback_args=topic,
            queue_size=1,
        )
        for topic in topics
    ]

    deadline = started + args.duration
    while time.monotonic() < deadline and not rospy.is_shutdown():
        time.sleep(0.05)

    finished = time.monotonic()
    report = {
        "duration_seconds": round(finished - started, 6),
        "min_messages": args.min_messages,
        "topics": {},
    }
    failures = []
    with lock:
        snapshot = {topic: list(values) for topic, values in samples.items()}

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
        report["topics"][topic] = {
            "count": count,
            "rate_hz": round(rate, 3),
            "ok": ok,
        }
        print("{:<49} {:>5} {:>10.3f}   {}".format(topic, count, rate, "OK" if ok else "MISSING"))

    for subscriber in subscribers:
        subscriber.unregister()

    report["ok"] = not failures
    report["missing_topics"] = failures
    if args.json_out:
        output = Path(args.json_out)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False))
    return 0 if not failures else 2


if __name__ == "__main__":
    sys.exit(main())

