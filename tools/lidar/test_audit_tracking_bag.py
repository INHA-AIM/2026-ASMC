import math
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from audit_tracking_bag import TrackingAudit, cloud_summary, distribution


def array(topic, stamp_ns, ids=(), frame=None, bag_ns=None):
    return {"topic": topic, "stamp_ns": stamp_ns,
            "bag_ns": stamp_ns if bag_ns is None else bag_ns,
            "frame": frame or ("map" if topic == "/last_lidar" else "velodyne"),
            "objects": [{"id": value, "geometry": [1., 2., 3.],
                         "velocity_mps": [0., 0., 0.], "speed_kph": 0.} for value in ids]}


def gps(time, valid=True):
    return {"topic": "/gps", "stamp_ns": time, "bag_ns": time,
            "frame": "gps", "gps_valid": valid}


class TrackingAuditTest(unittest.TestCase):
    def test_empty_arrays_are_unknown_not_free_space(self):
        audit = TrackingAudit()
        audit.add(array("/last_lidar", 1_000_000_000))
        audit.add(array("/lidar_nogps", 1_000_000_000))
        report = audit.report()
        self.assertEqual(report["handoff"]["both_empty_pairs"], 1)
        self.assertEqual(report["handoff"]["observed_nonempty_stream_switches"], [])
        self.assertTrue(any("do not prove free space" in item for item in report["interpretation"]))

    def test_stream_overlap_and_frame_mismatch_are_visible(self):
        audit = TrackingAudit(local_frame="base_link")
        audit.add(array("/last_lidar", 1_000_000_000, [2]))
        audit.add(array("/lidar_nogps", 1_000_000_000, [2]))
        report = audit.report()
        self.assertEqual(report["handoff"]["both_nonempty_pairs"], 1)
        self.assertEqual(report["topics"]["/lidar_nogps"]["unexpected_frame_messages"], 1)

    def test_recovery_hysteresis_is_an_observation_not_false_failure(self):
        audit = TrackingAudit()
        audit.add(gps(900_000_000, True))
        audit.add(gps(1_100_000_000, False))
        audit.add(gps(1_200_000_000, True))
        for time, map_ids, local_ids in [(1_000_000_000, [7], []),
                                         (1_150_000_000, [], [7]),
                                         (1_250_000_000, [], [7]),
                                         (2_000_000_000, [7], [])]:
            audit.add(array("/last_lidar", time, map_ids))
            audit.add(array("/lidar_nogps", time, local_ids))
        handoff = audit.report()["handoff"]
        self.assertEqual(len(handoff["gps_sample_transitions"]), 2)
        self.assertEqual(len(handoff["observed_nonempty_stream_switches"]), 2)
        self.assertEqual(handoff["observed_nonempty_stream_switches"][0]["shared_ids"], [7])
        self.assertEqual(handoff["gps_vs_output_observations"]["raw_valid/local_nonempty"], 1)
        self.assertEqual(handoff["gps_vs_output_observations"]["unknown_or_stale/map_nonempty"], 1)

    def test_timing_does_not_hide_regressions_or_claim_latency(self):
        audit = TrackingAudit()
        for stamp in [1_000_000_000, 1_000_000_000, 900_000_000, 0]:
            audit.add(array("/last_lidar", stamp, bag_ns=2_000_000_000))
        item = audit.report()["topics"]["/last_lidar"]
        self.assertEqual(item["stamp_regressions"], 1)
        self.assertEqual(item["duplicate_stamp_count"], 1)
        self.assertEqual(item["zero_or_missing_stamp_count"], 1)

    def test_duplicate_and_reappearing_ids_are_separate(self):
        audit = TrackingAudit()
        for time, ids in [(1, [2, 2]), (2, []), (3, [2])]:
            audit.add(array("/last_lidar", time, ids))
        item = audit.report()["topics"]["/last_lidar"]["tracking"]
        self.assertEqual(item["duplicate_ids_within_array"], 1)
        self.assertEqual(item["id_reappearances_after_absence"], 1)

    def test_unmatched_stamps_do_not_pair_reused_output(self):
        audit = TrackingAudit()
        audit.add(array("/last_lidar", 1_000_000_000, [1]))
        audit.add(array("/last_lidar", 1_000_000_000, [1]))
        audit.add(array("/lidar_nogps", 1_000_000_000))
        report = audit.report()["handoff"]
        self.assertEqual(report["paired_arrays"], 1)
        self.assertEqual(report["unpaired_map_arrays"], 1)

    def test_cloud_layout_respects_padding_and_endianness(self):
        fields = [SimpleNamespace(name=name, offset=index * 4, datatype=7, count=1)
                  for index, name in enumerate(("x", "y", "z"))]
        message = SimpleNamespace(width=1, height=2, fields=fields, point_step=12,
                                  row_step=16, is_bigendian=True,
                                  data=struct.pack(">fff", 2., 0., 0.) + b"PAD!" +
                                  struct.pack(">fff", float("nan"), 0., 0.) + b"PAD!")
        result = cloud_summary(message, 100)
        self.assertEqual(result["sampled_points"], 2)
        self.assertEqual(result["sampled_nonfinite_xyz"], 1)
        self.assertFalse(result["malformed_layout"])
        message.data = message.data[:20]
        self.assertTrue(cloud_summary(message, 100)["malformed_layout"])

    def test_distribution_rejects_nonfinite_values(self):
        self.assertEqual(distribution([float("nan"), float("inf")])["count"], 0)
        self.assertEqual(distribution([1, 2, 3])["p50"], 2)

    def test_diagnostics_keep_inactive_and_missing_protection_separate(self):
        audit = TrackingAudit()
        for index, values in enumerate([
                {"tunnel_active": "false"},
                {"tunnel_active": "true", "protection_available": "false", "time_frame_total_ms": "17.2"},
                {"tunnel_active": "true", "protection_available": "true", "time_frame_total_ms": "18.2"}]):
            audit.add({"topic": "/lidar/tunnel/diagnostics", "stamp_ns": index + 1,
                       "bag_ns": index + 1, "frame": "velodyne",
                       "diagnostics": [{"name": "tunnel", "level": 0, "message": "status", "values": values}]})
        result = audit.report()["topics"]["/lidar/tunnel/diagnostics"]["diagnostics"]["tunnel"]
        self.assertEqual(result["numeric"]["time_frame_total_ms"]["count"], 2)
        self.assertEqual(result["categorical"]["tunnel_active"], {"false": 1, "true": 2})
        self.assertEqual(result["categorical"]["protection_available"], {"false": 1, "true": 1})


if __name__ == "__main__":
    unittest.main()
