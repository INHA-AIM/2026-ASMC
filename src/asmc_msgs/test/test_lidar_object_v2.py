#!/usr/bin/env python3
"""LiDAR object contract v2 (B0): frozen v1 MD5, v2 constants and lossless round trip."""

import io
import unittest

import genpy

import asmc_msgs.msg as generated_messages
from asmc_msgs.msg import LidarObject, LidarObjectArray, LidarObjectArrayV2, LidarObjectV2


FROZEN_MD5 = {
    "LidarObject": "b94cf82e0f325e9ce0364bb143df9f06",
    "LidarObjectArray": "5f774d07dffd4c7dfa93a8d4b62c60b4",
    "TrackedObject": "6c96135eb03a3ee93bba474fa835f57d",
    "TrackedObjectArray": "e127b2aabd7494ad01a5a56b005bf07e",
}

V2_MD5 = {
    "LidarObjectV2": "c22c9b52da848aeab0e2b4e73343d9f0",
    "LidarObjectArrayV2": "d728e7fd984c24641abe10dddfda6dd5",
}

MOTION_CONSTANTS = {
    "MOTION_UNKNOWN": 0,
    "MOTION_LOW_THREAT": 1,
    "MOTION_DYNAMIC_PENDING": 2,
    "MOTION_DYNAMIC": 3,
}

ARRAY_STATUS_CONSTANTS = ("OBSERVED", "OBSERVED_EMPTY", "INPUT_INVALID", "POSE_UNAVAILABLE")


def _round_trip(message):
    buffer = io.BytesIO()
    message.serialize(buffer)
    restored = type(message)()
    restored.deserialize(buffer.getvalue())
    return restored


def _object(state, index):
    """A v2 object whose every field differs from its default value."""
    item = LidarObjectV2()
    item.track_id = 100 + index
    item.detection_index = 7 + index
    item.observation_state = LidarObjectV2.OBSERVED if index % 2 == 0 else LidarObjectV2.PREDICTED
    item.age = 3 + index
    item.miss_count = index
    item.classification = "car"
    item.camera_class = "truck"
    item.center.x, item.center.y, item.center.z = 1.5 + index, -2.25, 0.75
    for corner, point in enumerate(item.obb_corners):
        point.x, point.y, point.z = corner + 0.125, -corner - 0.5, 0.25 * corner
    item.yaw = 0.5 - index
    item.yaw_rate_valid = True
    item.yaw_rate_rad_s = 0.125 * index
    item.length, item.width, item.height = 4.5, 1.75, 1.5
    item.relative_velocity_valid = True
    item.relative_velocity_mps.x = 1.0 + index
    item.absolute_velocity_valid = index % 2 == 1
    item.absolute_velocity_mps.y = -0.5 * index
    item.source_point_count = 50 + index
    item.motion_state = state
    item.motion_valid = state != LidarObjectV2.MOTION_UNKNOWN
    item.dynamic_valid = item.motion_valid
    item.is_dynamic = state == LidarObjectV2.MOTION_DYNAMIC
    return item


class LidarObjectV2ContractTest(unittest.TestCase):
    def test_frozen_message_md5_is_unchanged(self):
        for name, expected in FROZEN_MD5.items():
            with self.subTest(message=name):
                self.assertEqual(getattr(generated_messages, name)._md5sum, expected)

    def test_v2_md5_and_type_are_pinned(self):
        for name, expected in V2_MD5.items():
            with self.subTest(message=name):
                message_class = getattr(generated_messages, name)
                self.assertEqual(message_class._md5sum, expected)
                self.assertEqual(message_class._type, "asmc_msgs/" + name)

    def test_constant_values(self):
        for name, value in MOTION_CONSTANTS.items():
            with self.subTest(constant=name):
                self.assertEqual(getattr(LidarObjectV2, name), value)
        self.assertEqual(LidarObjectV2.OBSERVED, LidarObject.OBSERVED)
        self.assertEqual(LidarObjectV2.PREDICTED, LidarObject.PREDICTED)
        for name in ARRAY_STATUS_CONSTANTS:
            with self.subTest(constant=name):
                self.assertEqual(getattr(LidarObjectArrayV2, name), getattr(LidarObjectArray, name))

    def test_v2_repeats_every_v1_field_then_appends_motion(self):
        count = len(LidarObject.__slots__)
        self.assertEqual(LidarObjectV2.__slots__[:count], LidarObject.__slots__)
        self.assertEqual(LidarObjectV2._slot_types[:count], LidarObject._slot_types)
        self.assertEqual(LidarObjectV2.__slots__[count:], ["motion_valid", "motion_state"])
        self.assertEqual(LidarObjectV2._slot_types[count:], ["bool", "uint8"])

    def test_array_v2_keeps_v1_header_status_reason_provenance(self):
        self.assertEqual(LidarObjectArrayV2.__slots__, LidarObjectArray.__slots__)
        self.assertEqual(LidarObjectArrayV2._slot_types[:-1], LidarObjectArray._slot_types[:-1])
        self.assertEqual(LidarObjectArray._slot_types[-1], "asmc_msgs/LidarObject[]")
        self.assertEqual(LidarObjectArrayV2._slot_types[-1], "asmc_msgs/LidarObjectV2[]")

    def test_default_object_is_unknown_and_invalid(self):
        item = LidarObjectV2()
        self.assertFalse(item.motion_valid)
        self.assertEqual(item.motion_state, LidarObjectV2.MOTION_UNKNOWN)
        self.assertFalse(item.dynamic_valid)
        self.assertFalse(item.is_dynamic)

    def test_round_trip_preserves_all_four_motion_states(self):
        array = LidarObjectArrayV2()
        array.header.seq = 11
        array.header.stamp = genpy.Time(1760000000, 123456789)
        array.header.frame_id = "base_link"
        array.status = LidarObjectArrayV2.OBSERVED
        array.reason = "observed"
        array.stamp_provenance = "bridge_receive_ns"
        states = sorted(MOTION_CONSTANTS.values())
        array.objects = [_object(state, index) for index, state in enumerate(states)]

        restored = _round_trip(array)

        self.assertEqual(restored, array)
        self.assertEqual([item.motion_state for item in restored.objects], states)
        self.assertEqual([item.motion_valid for item in restored.objects], [False, True, True, True])
        self.assertEqual([item.dynamic_valid for item in restored.objects], [False, True, True, True])
        self.assertEqual([item.is_dynamic for item in restored.objects], [False, False, False, True])
        self.assertEqual(restored.header.stamp, genpy.Time(1760000000, 123456789))
        self.assertEqual(restored.stamp_provenance, "bridge_receive_ns")

    def test_round_trip_each_state_alone(self):
        for state in MOTION_CONSTANTS.values():
            with self.subTest(motion_state=state):
                original = _object(state, 0)
                self.assertEqual(_round_trip(original), original)

    def test_observed_empty_array_round_trip(self):
        # Wire property only: an empty array keeps its status, reason and stamp.
        # Which topic may publish which status is a topic-contract rule, not a type rule.
        array = LidarObjectArrayV2()
        array.header.stamp = genpy.Time(1760000001, 5)
        array.header.frame_id = "base_link"
        array.status = LidarObjectArrayV2.OBSERVED_EMPTY
        array.reason = "observed_empty"
        array.stamp_provenance = "bridge_receive_ns"
        restored = _round_trip(array)
        self.assertEqual(restored, array)
        self.assertEqual(restored.status, LidarObjectArrayV2.OBSERVED_EMPTY)
        self.assertEqual(restored.objects, [])


if __name__ == "__main__":
    unittest.main()
