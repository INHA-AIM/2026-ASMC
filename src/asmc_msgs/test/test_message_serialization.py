#!/usr/bin/env python3
"""Generated-message MD5 and serialization regression for contract v1."""

import io
import unittest

import asmc_msgs.msg as generated_messages


EXPECTED_MD5 = {
    "HdMapMeta": "adb0d98c5dbd41055ebb929fcdf3af19",
    "LaneMarking": "65d4f9e4cbb4f60a65b5d7c3702e49fa",
    "LaneMarkingArray": "2eae16c9d4bf3385ac35eba4b3d1b823",
    "Lane": "380340d72bbc206438d6cf73629c5fa4",
    "LaneArray": "95ad93ef58c8ff22f4069b8ca2388395",
    "LaneAssociation": "fac864677f1248f439252de3e651536d",
    "DrivableArea": "3f988da146dba2b6bd2348b32ba3fb7d",
    "DrivableAreaArray": "672c8c1123790ada26596de6aab5edcb",
    "CompetitionCheckpoint": "794b79e3d48ac7fef6493abadad2fc47",
    "CompetitionCheckpointArray": "2a9f1f28734c005b499ce4993b20931e",
    "MissionZone": "902a8d5532787290e6471aa52a8a0d9c",
    "MissionZoneArray": "04325427bb7aedffd9a6ec4881ed46ce",
    "TrafficLightDetection": "86055e23eb1cd7ce661d0c315d66a05e",
    "TrackedObject": "6c96135eb03a3ee93bba474fa835f57d",
    "TrackedObjectArray": "e127b2aabd7494ad01a5a56b005bf07e",
}


class GeneratedSerializationTest(unittest.TestCase):
    def test_default_instances_round_trip_and_match_frozen_md5(self):
        for name, expected_md5 in EXPECTED_MD5.items():
            with self.subTest(message=name):
                message_class = getattr(generated_messages, name)
                self.assertEqual(message_class._md5sum, expected_md5)
                original = message_class()
                buffer = io.BytesIO()
                original.serialize(buffer)
                restored = message_class()
                restored.deserialize(buffer.getvalue())
                self.assertEqual(str(original), str(restored))


if __name__ == "__main__":
    unittest.main()
