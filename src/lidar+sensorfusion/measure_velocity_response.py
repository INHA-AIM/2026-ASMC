#!/usr/bin/env python3

"""Measure stopped-to-moving response against MORAI Object_topic ground truth."""

import math
import re
import time

import rospy
from morai_msgs.msg import ObjectStatusList
from visualization_msgs.msg import MarkerArray


BASELINE_SECONDS = 3.0
TEST_SECONDS = 120.0
GROUND_TRUTH_ON_KMH = 2.0
BASELINE_STATIC_KMH = 0.8
REPORT_AFTER_ONSET_SECONDS = 12.0

MARKER_TEXT = re.compile(
    r"ID:(\d+) MOVE:(\d+).*?\|v\|:([0-9.]+)", re.DOTALL
)


class VelocityResponseProbe:
    def __init__(self):
        self.started_at = time.monotonic()
        self.baseline_ends_at = self.started_at + BASELINE_SECONDS
        self.object_now = {}
        self.tracker_now = {}
        self.object_baseline_max = {}
        self.tracker_baseline_max = {}
        self.ground_truth_event = None
        self.tracker_events = {}

        rospy.Subscriber(
            "/Object_topic", ObjectStatusList, self.object_callback, queue_size=10
        )
        rospy.Subscriber(
            "/lidar_tracking_markers",
            MarkerArray,
            self.tracker_callback,
            queue_size=10,
        )

    @staticmethod
    def object_speed_kmh(obj):
        # MORAI ObjectStatus velocity.x is the object's forward speed in m/s.
        return abs(float(obj.velocity.x)) * 3.6

    def object_callback(self, message):
        now = time.monotonic()
        objects = (
            list(message.npc_list)
            + list(message.pedestrian_list)
            + list(message.obstacle_list)
        )
        for obj in objects:
            speed = self.object_speed_kmh(obj)
            unique_id = int(obj.unique_id)
            self.object_now[unique_id] = (speed, obj.name)

            if now <= self.baseline_ends_at:
                self.object_baseline_max[unique_id] = max(
                    self.object_baseline_max.get(unique_id, 0.0), speed
                )
                continue

            was_static = (
                self.object_baseline_max.get(unique_id, 0.0)
                < BASELINE_STATIC_KMH
            )
            if (
                self.ground_truth_event is None
                and was_static
                and speed >= GROUND_TRUTH_ON_KMH
            ):
                self.ground_truth_event = (now, unique_id, obj.name)
                print(
                    "GT_ONSET uid={} name={} speed={:.2f}km/h".format(
                        unique_id, obj.name, speed
                    ),
                    flush=True,
                )

    def tracker_callback(self, message):
        now = time.monotonic()
        for marker in message.markers:
            if marker.ns != "id":
                continue
            match = MARKER_TEXT.search(marker.text)
            if not match:
                continue

            tracker_id = int(match.group(1))
            move = int(match.group(2))
            speed = float(match.group(3))
            self.tracker_now[tracker_id] = (speed, move)

            if now <= self.baseline_ends_at:
                self.tracker_baseline_max[tracker_id] = max(
                    self.tracker_baseline_max.get(tracker_id, 0.0), speed
                )
                continue

            was_static = (
                self.tracker_baseline_max.get(tracker_id, 0.0)
                < BASELINE_STATIC_KMH
            )
            if (
                was_static
                and speed >= GROUND_TRUTH_ON_KMH
                and tracker_id not in self.tracker_events
            ):
                self.tracker_events[tracker_id] = now
                print(
                    "TRACK_ONSET id={} speed={:.2f}km/h move={}".format(
                        tracker_id, speed, move
                    ),
                    flush=True,
                )

    def report_state(self, now):
        eligible_ground_truth = [
            (unique_id, round(values[0], 2), values[1])
            for unique_id, values in sorted(self.object_now.items())
            if self.object_baseline_max.get(unique_id, 0.0)
            < BASELINE_STATIC_KMH
        ]
        candidate_tracks = [
            (tracker_id, round(values[0], 2), values[1])
            for tracker_id, values in sorted(self.tracker_now.items())
            if self.tracker_baseline_max.get(tracker_id, 0.0)
            < BASELINE_STATIC_KMH
        ]
        print(
            "STATE t={:.1f} eligible_gt={} candidate_tracks={}".format(
                now - self.started_at,
                eligible_ground_truth,
                candidate_tracks,
            ),
            flush=True,
        )

    def run(self):
        print(
            "BASELINE {:.0f}s: keep the test vehicle stopped".format(
                BASELINE_SECONDS
            ),
            flush=True,
        )
        next_report = self.started_at
        while not rospy.is_shutdown():
            now = time.monotonic()
            if now - self.started_at >= TEST_SECONDS:
                break
            if now >= next_report:
                next_report = now + 1.0
                self.report_state(now)
            if (
                self.ground_truth_event
                and now - self.ground_truth_event[0]
                >= REPORT_AFTER_ONSET_SECONDS
            ):
                break
            time.sleep(0.02)

        if not self.ground_truth_event:
            print("RESULT no stopped-to-moving GT transition", flush=True)
            return 1

        ground_truth_time, unique_id, _ = self.ground_truth_event
        candidates = [
            (event_time, tracker_id)
            for tracker_id, event_time in self.tracker_events.items()
            if event_time >= ground_truth_time - 0.3
        ]
        if not candidates:
            print("RESULT GT moved but no new tracker onset", flush=True)
            return 2

        tracker_time, tracker_id = min(candidates)
        print(
            "RESULT gt_uid={} tracker_id={} latency={:.3f}s".format(
                unique_id, tracker_id, tracker_time - ground_truth_time
            ),
            flush=True,
        )
        return 0


def main():
    rospy.init_node("velocity_transition_probe", anonymous=True)
    raise SystemExit(VelocityResponseProbe().run())


if __name__ == "__main__":
    main()
