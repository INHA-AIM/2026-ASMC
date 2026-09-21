#!/usr/bin/env python3
"""Focused tests for the P1-A baseline builder."""

from __future__ import annotations

import json
import os
import struct
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import build_runtime_baseline as baseline


REPO_ROOT = Path(__file__).resolve().parents[2]
DATA_ROOT = Path(os.environ.get("ASMC_DATA", str(REPO_ROOT.parent / "data")))


def _field(name: str, value: bytes) -> bytes:
    payload = name.encode("ascii") + b"=" + value
    return struct.pack("<I", len(payload)) + payload


def _header(fields: list[tuple[str, bytes]]) -> bytes:
    return b"".join(_field(name, value) for name, value in fields)


def _record(fields: list[tuple[str, bytes]], data: bytes = b"") -> bytes:
    header = _header(fields)
    return struct.pack("<I", len(header)) + header + struct.pack("<I", len(data)) + data


def _time(seconds: int, nanoseconds: int = 0) -> bytes:
    return struct.pack("<II", seconds, nanoseconds)


def _make_bag(path: Path) -> None:
    magic = b"#ROSBAG V2.0\n"
    placeholder = _record(
        [
            ("op", b"\x03"),
            ("index_pos", struct.pack("<Q", 0)),
            ("conn_count", struct.pack("<I", 1)),
            ("chunk_count", struct.pack("<I", 1)),
        ],
        b"",
    )
    index_position = len(magic) + len(placeholder)
    bag_header = _record(
        [
            ("op", b"\x03"),
            ("index_pos", struct.pack("<Q", index_position)),
            ("conn_count", struct.pack("<I", 1)),
            ("chunk_count", struct.pack("<I", 1)),
        ],
        b"",
    )
    connection_data = _header(
        [
            ("type", b"std_msgs/String"),
            ("md5sum", b"992ce8a1687cec8c8bd883ec73ca41d1"),
        ]
    )
    connection = _record(
        [
            ("op", b"\x07"),
            ("conn", struct.pack("<I", 0)),
            ("topic", b"/fixture"),
        ],
        connection_data,
    )
    chunk_info = _record(
        [
            ("op", b"\x06"),
            ("ver", struct.pack("<I", 1)),
            ("chunk_pos", struct.pack("<Q", 0)),
            ("start_time", _time(10)),
            ("end_time", _time(12)),
            ("count", struct.pack("<I", 1)),
        ],
        struct.pack("<II", 0, 20),
    )
    path.write_bytes(magic + bag_header + connection + chunk_info)


class RuntimeBaselineTest(unittest.TestCase):
    def test_ros1_bag_index_reader_without_ros_cli(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            data_root = Path(temporary)
            bag_path = data_root / "fixture.bag"
            _make_bag(bag_path)
            result = baseline.read_ros1_bag_index(bag_path, data_root)
        self.assertEqual("fixture.bag", result["path"])
        self.assertEqual(2.0, result["duration_s"])
        self.assertEqual(20, result["message_count"])
        self.assertEqual(10.0, result["topics"][0]["average_hz"])
        self.assertEqual("992ce8a1687cec8c8bd883ec73ca41d1", result["topics"][0]["md5"])

    def test_stage14_geometry_inventory_is_frozen(self) -> None:
        result = baseline.audit_hd_map(REPO_ROOT)
        self.assertEqual(baseline.EXPECTED_MAP_COUNTS, result["validation"]["actual"])
        self.assertEqual(64, len(result["geometry_sha256"]))
        self.assertFalse(result["operational"])
        self.assertTrue(result["partial_candidate"])

    def test_v2_sensor_baseline_and_future_lidar_target_are_distinct(self) -> None:
        result = baseline.audit_sensors(REPO_ROOT, DATA_ROOT)
        contract = result["lidar_rate_contract"]
        self.assertEqual(10.0, contract["baseline_hz"])
        self.assertEqual(15.0, contract["approved_future_target_hz"])
        self.assertEqual("recorded_not_applied", contract["status"])
        differences = result["submitted_autonav_reference"][
            "semantic_differences_from_v2"
        ]
        self.assertEqual(["$.cameraList[3].rot.pitch"], [item["path"] for item in differences])

    def test_checkpoint_and_route_facts(self) -> None:
        result = baseline.audit_competition_inputs(DATA_ROOT, REPO_ROOT)
        self.assertEqual(16, result["checkpoints"]["count"])
        self.assertEqual(3.0, result["checkpoints"]["pass_radius_m"])
        self.assertAlmostEqual(2184.61, result["global_path"]["length_xy_m"], places=2)
        self.assertAlmostEqual(
            109.23,
            result["global_path"]["start_success_5_percent_station_m"],
            places=2,
        )
        self.assertEqual(15, result["metadata_bundle"]["mission_zone_count"])
        self.assertEqual(
            {"derived_official": 4, "provisional": 11},
            result["metadata_bundle"]["mission_status_counts"],
        )
        self.assertFalse(result["metadata_bundle"]["published"])

    def test_python_static_topic_audit_keeps_topic_and_type_in_right_columns(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "src/example/node.py"
            source.parent.mkdir(parents=True)
            source.write_text(
                'pub = rospy.Publisher("/example/out", ExampleMessage, queue_size=1)\n'
                'sub = rospy.Subscriber("/example/in", InputMessage, callback)\n',
                encoding="utf-8",
            )
            result = baseline.audit_static_topics(root, {"files": []})
        references = {(item["direction"], item["topic"]): item for item in result["references"]}
        self.assertEqual("ExampleMessage", references[("publish", "/example/out")]["message_type_literal"])
        self.assertEqual("InputMessage", references[("subscribe", "/example/in")]["message_type_literal"])

    def test_yaml_render_is_deterministic(self) -> None:
        value = {"schema_version": 1, "unicode": "체크", "items": [3, 2, 1]}
        self.assertEqual(baseline.dump_manifest(value), baseline.dump_manifest(value))
        self.assertEqual(value, json.loads(json.dumps(value, ensure_ascii=False)))

    def test_repository_symlink_does_not_escape_or_enter_reference_scan(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            workspace = Path(temporary)
            root = workspace / "repository"
            source = root / "src"
            source.mkdir(parents=True)
            external = workspace / "external.txt"
            external.write_text("src/link_node/", encoding="utf-8")
            link = source / "CMakeLists.txt"
            link.symlink_to(external)
            self.assertEqual("src/CMakeLists.txt", baseline.repo_relative(link, root))
            self.assertNotIn(link, list(baseline._candidate_reference_files(root)))


if __name__ == "__main__":
    unittest.main()
