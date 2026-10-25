#!/usr/bin/env python3
"""Generated-message MD5 and serialization regression for contract v1."""

import io
import unittest

import asmc_msgs.msg as generated_messages


EXPECTED_MD5 = {
    "CameraModelGridTiming": "3d9d2bf10966b2aac416bb6a1f66ea99",
    "SensorFrameTiming": "07dce68802e02b5b3f4cd89724a7cad7",
    "InferenceStatus": "8b2caf96bbf9a9eaea848fcb9551b79f",
    "CameraObjectObservation": "fbddb8fe0071685b60811037c0302106",
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
    "LidarObject": "b94cf82e0f325e9ce0364bb143df9f06",
    "LidarObjectArray": "5f774d07dffd4c7dfa93a8d4b62c60b4",
    "LidarObservation": "d11a2a270112231e4f79aaebff621d36",
    "LidarPlanningObservation": "670b15bd7bb1bc58e0640c2a35054936",
    "CameraObjectBox": "34412c90553e06734f1acade77c8e1e3",
    "CameraObjectBoxArray": "310c671afb3693e7909db6453d115b79",
}


class GeneratedSerializationTest(unittest.TestCase):
    def test_sensor_frame_timing_keeps_raw_and_selected_clocks_separate(self):
        from asmc_msgs.msg import SensorFrameTiming
        import genpy
        original = SensorFrameTiming(bridge_session_id='session', stream_id='cam1', frame_sequence=3,
            published_stamp=genpy.Time(100, 20), sensor_wire_sequence=17, sensor_wire_sequence_valid=True,
            source_sec=99, source_nsec=123, source_valid=True, source_duplicate=True,
            rx_first_unix_ns=100000000001, rx_last_unix_ns=100000000010,
            rx_first_monotonic_ns=50, rx_last_monotonic_ns=60, packet_count=2,
            header_timestamp_policy='receive')
        buffer = io.BytesIO(); original.serialize(buffer)
        restored = SensorFrameTiming(); restored.deserialize(buffer.getvalue())
        self.assertEqual(original, restored)
        self.assertEqual(restored.source_sec, 99)
        self.assertEqual(restored.published_stamp.secs, 100)
        self.assertEqual(restored.sensor_wire_sequence, 17)

    def test_lidar_observation_preserves_scan_identity_and_array(self):
        from asmc_msgs.msg import LidarObservation, LidarObjectArray
        import genpy
        m = LidarObservation(producer_session_id='producer-uuid',generation=2,scan_sequence=93,
                             config_id='config-sha',calibration_id='calibration-sha')
        m.objects.header.stamp=genpy.Time(100,17)
        m.objects.header.frame_id='base_link'
        m.objects.status=LidarObjectArray.OBSERVED_EMPTY
        raw=io.BytesIO(); m.serialize(raw)
        restored=LidarObservation(); restored.deserialize(raw.getvalue())
        self.assertEqual(m,restored)
        self.assertEqual(restored.objects.header.stamp.nsecs,17)

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


    def test_inference_session_and_service_roundtrip(self):
        from asmc_msgs.msg import InferenceStatus
        from asmc_msgs.srv import SetInferenceMode, SetInferenceModeRequest
        self.assertEqual(SetInferenceMode._md5sum, "94a789aab65be7cc8d09e8f777708430")
        status = InferenceStatus(producer_session_id="restart-uuid", generation=7, mode=2)
        raw = io.BytesIO(); status.serialize(raw)
        copy = InferenceStatus(); copy.deserialize(raw.getvalue())
        self.assertEqual(copy.producer_session_id, "restart-uuid")
        self.assertEqual(copy.generation, 7)
        request = SetInferenceModeRequest(mode=1, target_hz=15.)
        raw = io.BytesIO(); request.serialize(raw)
        copy = SetInferenceModeRequest(); copy.deserialize(raw.getvalue())
        self.assertEqual(copy, request)


if __name__ == "__main__":
    unittest.main()
