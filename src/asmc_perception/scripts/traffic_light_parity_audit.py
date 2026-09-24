#!/usr/bin/env python3
"""Replay a representative bag through legacy and canonical traffic-light nodes."""

from __future__ import annotations

import hashlib
import json
import statistics
import sys
import threading
import time
from collections import Counter, deque
from pathlib import Path

import rosbag
import rospy
import yaml
from asmc_msgs.msg import TrafficLightDetection
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from asmc_perception.traffic_light_parity import (
    ParityError,
    compare_details,
    parse_detail,
    validate_typed_result,
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _percentile(values, fraction):
    if not values:
        return 0.0
    ordered = sorted(float(value) for value in values)
    index = int(round((len(ordered) - 1) * float(fraction)))
    return ordered[max(0, min(index, len(ordered) - 1))]


class AuditNode:
    def __init__(self):
        self.bag_path = Path(rospy.get_param("~bag_path")).expanduser().resolve()
        self.bag_topic = rospy.get_param(
            "~bag_topic", "/cam_extra/image_jpeg/compressed"
        )
        self.reference_input_topic = rospy.get_param(
            "~reference_input_topic",
            "/asmc_perception/audit/reference/input/compressed",
        )
        self.canonical_input_topic = rospy.get_param(
            "~canonical_input_topic",
            "/asmc_perception/audit/canonical/input/compressed",
        )
        self.sample_count = int(rospy.get_param("~sample_count", 24))
        self.startup_timeout_sec = float(rospy.get_param("~startup_timeout_sec", 45.0))
        self.output_timeout_sec = float(rospy.get_param("~output_timeout_sec", 5.0))
        self.stale_timeout_sec = float(rospy.get_param("~stale_timeout_sec", 1.0))
        self.report_path = rospy.get_param("~report_path", "")
        self.checkpoint_path = Path(
            rospy.get_param("~checkpoint_path")
        ).expanduser().resolve()

        self.condition = threading.Condition()
        self.events = {
            "reference_state": deque(),
            "reference_detail": deque(),
            "canonical_state": deque(),
            "canonical_detail": deque(),
            "typed": deque(),
        }
        self.reference_publisher = rospy.Publisher(
            self.reference_input_topic, CompressedImage, queue_size=1
        )
        self.canonical_publisher = rospy.Publisher(
            self.canonical_input_topic, CompressedImage, queue_size=1
        )
        rospy.Subscriber(
            rospy.get_param("~reference_state_topic"),
            String,
            lambda message: self._append("reference_state", message.data),
            queue_size=20,
        )
        rospy.Subscriber(
            rospy.get_param("~reference_detail_topic"),
            String,
            lambda message: self._append("reference_detail", message.data),
            queue_size=20,
        )
        rospy.Subscriber(
            rospy.get_param("~canonical_state_topic"),
            String,
            lambda message: self._append("canonical_state", message.data),
            queue_size=20,
        )
        rospy.Subscriber(
            rospy.get_param("~canonical_detail_topic"),
            String,
            lambda message: self._append("canonical_detail", message.data),
            queue_size=20,
        )
        rospy.Subscriber(
            rospy.get_param("~typed_topic"),
            TrafficLightDetection,
            lambda message: self._append("typed", message),
            queue_size=20,
        )

        self.maximum_probability_delta = 0.0
        self.maximum_candidate_delta = 0.0
        self.reference_latency_ms = []
        self.canonical_latency_ms = []
        self.labels = Counter()
        self.errors = []
        self.bag_topic_message_count = 0

    def _append(self, key, value):
        with self.condition:
            self.events[key].append(value)
            self.condition.notify_all()

    def _clear_events(self):
        with self.condition:
            for queue in self.events.values():
                queue.clear()

    def _wait_for_connections(self):
        deadline = time.monotonic() + self.startup_timeout_sec
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if (
                self.reference_publisher.get_num_connections() >= 1
                and self.canonical_publisher.get_num_connections() >= 1
            ):
                return
            rospy.sleep(0.05)
        raise ParityError("legacy or canonical input subscriber did not connect")

    def _wait_for_reference(self, stale):
        deadline = time.monotonic() + self.output_timeout_sec
        with self.condition:
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                reference_details = [
                    value
                    for value in self.events["reference_detail"]
                    if parse_detail(value)["stale"] == bool(stale)
                ]
                if self.events["reference_state"] and reference_details:
                    return {
                        "reference_state": self.events["reference_state"][-1],
                        "reference_detail": reference_details[-1],
                    }
                self.condition.wait(timeout=0.02)
        phase = "stale" if stale else "fresh"
        raise ParityError("timed out waiting for reference %s outputs" % phase)

    def _wait_for_canonical(self, stale):
        deadline = time.monotonic() + self.output_timeout_sec
        with self.condition:
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                canonical_details = [
                    value
                    for value in self.events["canonical_detail"]
                    if parse_detail(value)["stale"] == bool(stale)
                ]
                typed = [
                    value
                    for value in self.events["typed"]
                    if bool(value.stale) == bool(stale)
                ]
                if (
                    self.events["canonical_state"]
                    and canonical_details
                    and typed
                ):
                    return {
                        "canonical_state": self.events["canonical_state"][-1],
                        "canonical_detail": canonical_details[-1],
                        "typed": typed[-1],
                    }
                self.condition.wait(timeout=0.02)
        phase = "stale" if stale else "fresh"
        raise ParityError("timed out waiting for canonical %s outputs" % phase)

    def _compare(self, outputs, source, stale):
        reference = parse_detail(outputs["reference_detail"])
        canonical = parse_detail(outputs["canonical_detail"])
        if bool(reference["stale"]) != bool(stale):
            raise ParityError("reference stale phase mismatch")
        if outputs["reference_state"] != reference["label"]:
            raise ParityError("reference state/detail mismatch")
        if outputs["canonical_state"] != canonical["label"]:
            raise ParityError("canonical state/detail mismatch")
        detail_metrics = compare_details(reference, canonical)
        typed_metrics = validate_typed_result(
            outputs["typed"],
            canonical,
            expected_stamp_nsec=source.header.stamp.to_nsec(),
            expected_frame_id=source.header.frame_id,
        )
        self.maximum_probability_delta = max(
            self.maximum_probability_delta,
            detail_metrics["probability_delta"],
            typed_metrics["probability_delta"],
        )
        self.maximum_candidate_delta = max(
            self.maximum_candidate_delta,
            detail_metrics["candidate_confidence_delta"],
            typed_metrics["candidate_confidence_delta"],
        )
        if not stale:
            self.labels[canonical["label"]] += 1
            try:
                self.reference_latency_ms.append(
                    float(json.loads(outputs["reference_detail"])["ms"])
                )
            except (KeyError, TypeError, ValueError):
                pass
            self.canonical_latency_ms.append(
                float(outputs["typed"].inference_latency_ms)
            )

    def _bag_messages(self):
        with rosbag.Bag(str(self.bag_path), "r") as bag:
            topics = bag.get_type_and_topic_info()[1]
            topic_info = topics.get(self.bag_topic)
            if topic_info is None:
                raise ParityError("bag does not contain topic %s" % self.bag_topic)
            self.bag_topic_message_count = int(topic_info.message_count)
            required = self.sample_count + 1
            if self.bag_topic_message_count < required:
                raise ParityError(
                    "bag topic has %d frames, needs at least %d"
                    % (self.bag_topic_message_count, required)
                )
            if required == 1:
                selected = {0}
            else:
                selected = {
                    int(
                        round(
                            index
                            * (self.bag_topic_message_count - 1)
                            / float(required - 1)
                        )
                    )
                    for index in range(required)
                }
            for index, (_, message, _) in enumerate(
                bag.read_messages(topics=[self.bag_topic])
            ):
                if index not in selected:
                    continue
                # rosbag may reconstruct a dynamic class even when the stored
                # datatype/MD5 exactly matches the installed ROS message.
                # Rebuild the generated class expected by rospy.Publisher.
                if getattr(message, "_type", "") != "sensor_msgs/CompressedImage":
                    continue
                output = CompressedImage()
                output.header.seq = int(message.header.seq)
                output.header.stamp = message.header.stamp
                output.header.frame_id = str(message.header.frame_id)
                output.format = str(message.format)
                output.data = bytes(message.data)
                yield output

    def _publish_and_compare(self, source):
        self._clear_events()
        self.reference_publisher.publish(source)
        outputs = self._wait_for_reference(stale=False)
        self.canonical_publisher.publish(source)
        outputs.update(self._wait_for_canonical(stale=False))
        self._compare(outputs, source, stale=False)

    def _data_reference(self):
        value = str(self.bag_path)
        if value == "/data":
            return "$ASMC_DATA"
        if value.startswith("/data/"):
            return "$ASMC_DATA/" + value[len("/data/") :]
        return value

    @staticmethod
    def _workspace_reference(path):
        value = str(path)
        if value.startswith("/root/ws/"):
            return value[len("/root/ws/") :]
        return value

    def _report(self, status, fresh_frames, stale_passed, recovery_passed):
        return {
            "schema_version": 1,
            "audit_id": "p4c2_traffic_light_parity_v1",
            "status": status,
            "source": {
                "bag": self._data_reference(),
                "topic": self.bag_topic,
                "sampling": "uniform_topic_extent",
                "topic_message_count": self.bag_topic_message_count,
                "size_bytes": self.bag_path.stat().st_size
                if self.bag_path.is_file()
                else 0,
                "sha256": _sha256(self.bag_path)
                if self.bag_path.is_file()
                else "",
            },
            "checkpoint": {
                "path": self._workspace_reference(self.checkpoint_path),
                "sha256": _sha256(self.checkpoint_path)
                if self.checkpoint_path.is_file()
                else "",
            },
            "acceptance": {
                "requested_fresh_frames": self.sample_count,
                "initial_fresh_frames_compared": min(
                    fresh_frames, self.sample_count
                ),
                "recovery_fresh_frames_compared": 1
                if recovery_passed
                else 0,
                "total_fresh_frames_compared": fresh_frames,
                "configured_stale_sec": self.stale_timeout_sec,
                "stale_transition_passed": bool(stale_passed),
                "recovery_transition_passed": bool(recovery_passed),
                "probability_tolerance": 1.0e-4,
                "maximum_probability_delta": round(
                    self.maximum_probability_delta, 8
                ),
                "maximum_candidate_confidence_delta": round(
                    self.maximum_candidate_delta, 8
                ),
                "source_stamp_and_frame_mismatches": 0 if status == "pass" else None,
            },
            "observed": {
                "labels": dict(sorted(self.labels.items())),
                "reference_latency_ms": {
                    "median": round(statistics.median(self.reference_latency_ms), 3)
                    if self.reference_latency_ms
                    else 0.0,
                    "p95": round(_percentile(self.reference_latency_ms, 0.95), 3),
                },
                "canonical_latency_ms": {
                    "median": round(statistics.median(self.canonical_latency_ms), 3)
                    if self.canonical_latency_ms
                    else 0.0,
                    "p95": round(_percentile(self.canonical_latency_ms, 0.95), 3),
                },
            },
            "errors": list(self.errors),
        }

    def _write_report(self, report):
        rendered = yaml.safe_dump(report, sort_keys=False, allow_unicode=True)
        if self.report_path:
            path = Path(self.report_path).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(rendered, encoding="utf-8")
            rospy.loginfo("traffic-light parity report: %s", path)
        print(rendered)

    def run(self):
        fresh_frames = 0
        stale_passed = False
        recovery_passed = False
        try:
            if self.sample_count <= 0:
                raise ParityError("sample_count must be positive")
            if not self.bag_path.is_file():
                raise ParityError("bag does not exist: %s" % self.bag_path)
            if not self.checkpoint_path.is_file():
                raise ParityError(
                    "checkpoint does not exist: %s" % self.checkpoint_path
                )
            self._wait_for_connections()
            messages = self._bag_messages()
            last_source = None
            for _ in range(self.sample_count):
                try:
                    source = next(messages)
                except StopIteration as exc:
                    raise ParityError("bag ended before requested sample count") from exc
                self._publish_and_compare(source)
                fresh_frames += 1
                last_source = source

            self._clear_events()
            outputs = self._wait_for_reference(stale=True)
            outputs.update(self._wait_for_canonical(stale=True))
            self._compare(outputs, last_source, stale=True)
            stale_passed = True

            try:
                recovery_source = next(messages)
            except StopIteration as exc:
                raise ParityError("bag has no frame for stale recovery") from exc
            self._publish_and_compare(recovery_source)
            fresh_frames += 1
            recovery_passed = True
        except (OSError, ParityError, rosbag.ROSBagException) as exc:
            self.errors.append(str(exc))
            rospy.logerr("traffic-light parity failed: %s", exc)
            report = self._report(
                "fail", fresh_frames, stale_passed, recovery_passed
            )
            self._write_report(report)
            return 1

        report = self._report("pass", fresh_frames, stale_passed, recovery_passed)
        self._write_report(report)
        rospy.loginfo(
            "traffic-light parity PASS fresh=%d stale=%s recovery=%s",
            fresh_frames,
            stale_passed,
            recovery_passed,
        )
        return 0


def main():
    rospy.init_node("traffic_light_parity_audit", anonymous=False)
    return AuditNode().run()


if __name__ == "__main__":
    sys.exit(main())
