import copy
import json
import math
from pathlib import Path
import tempfile
import unittest

from asmc_hd_map.competition_metadata import (
    CompetitionMetadataError,
    build_competition_metadata,
    default_data_root,
    dump_metadata_yaml,
    project_point_to_polyline,
    validate_competition_metadata,
)


class CompetitionMetadataGeometryTest(unittest.TestCase):
    def test_xy_projection_returns_continuous_station(self):
        points = [(0.0, 0.0, 1.0), (10.0, 0.0, 2.0), (10.0, 10.0, 3.0)]
        projected = project_point_to_polyline((9.0, 4.0, 99.0), points)
        self.assertAlmostEqual(projected["station_m"], 14.0)
        self.assertAlmostEqual(projected["error_xy_m"], 1.0)
        self.assertEqual(projected["segment_index"], 1)
        self.assertEqual(projected["projected_xyz"], [10.0, 4.0, 2.4])

    def test_closed_route_duplicate_prefers_start_station(self):
        projected = project_point_to_polyline(
            (0.0, 0.0, 0.0),
            [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 0.0)],
        )
        self.assertEqual(projected["station_m"], 0.0)


class CompetitionMetadataWorkspaceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.repo_root = Path(__file__).resolve().parents[3]
        cls.data_root = default_data_root(cls.repo_root)
        cls.config = (
            cls.repo_root
            / "src/asmc_hd_map/config/kcity2025_competition_metadata_source.yaml"
        )
        cls.document = build_competition_metadata(cls.config, cls.repo_root, cls.data_root)

    def test_official_checkpoint_contract_and_reviewed_link_assignment(self):
        document = self.document
        self.assertEqual(len(document["checkpoints"]), 15)
        self.assertEqual(document["route"]["length_xy_m"], 2184.611723336)
        self.assertEqual(document["route"]["start_success_5_percent_station_m"], 109.230586167)
        self.assertEqual(document["route"]["closure_error_xy_m"], 0.0)

        expected_links = {
            "START_END": "A2256W000748",
            "1": "A2256W000748",
            "2": "A2256W000236",
            "3": "A2256W000215",
            "4": "A2256W000222",
            "5": "A2256W000205",
            "6": "A2256W000318",
            "7": "A2256W000083",
            "8": "A2256W000304",
            "9": "A2256W000866",
            "10": "A2256W000846",
            "11": "A2256W000445",
            "12": "A2256W000153",
            "13": "A2256W000446",
            "14": "A2256W000126",
        }
        self.assertEqual(
            {item["id"]: item["link_id"] for item in document["checkpoints"]},
            expected_links,
        )
        self.assertTrue(document["checkpoints"][0]["is_start"])
        self.assertTrue(document["checkpoints"][0]["is_finish"])
        self.assertEqual(document["checkpoints"][0]["route_station_m"], 0.0)
        self.assertEqual(document["validation"]["max_checkpoint_route_projection_error_m"], 0.0)
        self.assertEqual(document["validation"]["max_checkpoint_link_projection_error_m"], 0.0)

    def test_rule_v1_2_removed_the_a411_checkpoint(self):
        # Rule book v1.2 dropped the 2026-08-03 checkpoint 11 at (66.885, 89.198)
        # on A411 and renumbered the rest without moving them.
        for item in self.document["checkpoints"]:
            position = item["pose"]["position"]
            self.assertGreater(math.hypot(position["x"] - 66.88496106746607,
                                          position["y"] - 89.19790052389726), 3.0)
        by_id = {item["id"]: item for item in self.document["checkpoints"]}
        self.assertEqual(by_id["11"]["pose"]["position"]["x"], 75.41420370276319)
        self.assertEqual(by_id["14"]["pose"]["position"]["x"], -74.86074459552765)

    def test_official_yaw_is_preserved_and_quaternion_is_derived(self):
        source = json.loads(
            (self.data_root / "R_KR_PG_K-city_2025/2026_molit_comp_checkpoints.json").read_text(
                encoding="utf-8"
            )
        )
        for raw, built in zip(source["checkpoints"], self.document["checkpoints"]):
            self.assertEqual(built["official_yaw_deg"], raw["yaw"])
            orientation = built["pose"]["orientation"]
            self.assertAlmostEqual(
                math.sqrt(sum(float(orientation[key]) ** 2 for key in ("x", "y", "z", "w"))),
                1.0,
                places=10,
            )

    def test_mission_authority_and_bounds_are_separated(self):
        missions = {item["id"]: item for item in self.document["mission_zones"]}
        self.assertEqual(len(missions), 15)
        self.assertEqual(self.document["validation"]["mission_status_counts"], {
            "derived_official": 4,
            "provisional": 11,
        })
        speed_profile = sorted(
            (
                item
                for item in missions.values()
                if item["mission_type"] in ("speed_limit", "high_speed_exception")
            ),
            key=lambda item: item["route_station_start_m"],
        )
        self.assertEqual(
            [item["id"] for item in speed_profile],
            [
                "speed_limit_60_before_high_speed",
                "high_speed_exception",
                "speed_limit_60_after_high_speed",
            ],
        )
        self.assertEqual(
            [item["route_station_start_m"] for item in speed_profile],
            [0.0, 1118.741751169, 1741.720988511],
        )
        self.assertEqual(
            [item["route_station_end_m"] for item in speed_profile],
            [1118.741751169, 1741.720988511, 2184.611723336],
        )
        self.assertEqual(
            [item["speed_limit_kph"] if item["has_speed_limit"] else None for item in speed_profile],
            [60.0, None, 60.0],
        )
        self.assertTrue(self.document["validation"]["speed_profile_full_route_coverage"])
        self.assertEqual(self.document["validation"]["speed_profile_segment_count"], 3)
        self.assertEqual(missions["start_success_5_percent"]["route_station_end_m"], 109.230586167)
        self.assertEqual(missions["high_speed_exception"]["route_station_start_m"], 1118.741751169)
        self.assertEqual(missions["high_speed_exception"]["route_station_end_m"], 1741.720988511)
        self.assertNotIn("highway_merge_candidate", missions)
        non_policy_missions = [
            item
            for item in missions.values()
            if item["mission_type"] not in ("speed_limit", "high_speed_exception", "traffic_light")
        ]
        self.assertTrue(
            all(
                link_id not in item["link_ids"]
                for item in non_policy_missions
                for link_id in ("A2256W000054", "A2256W000846")
            )
        )
        self.assertEqual(missions["gps_shadow_candidate"]["start_link_id"], "A2256W000126")
        self.assertEqual(
            missions["static_obstacle_candidate"]["link_ids"],
            ["A2256W000222"],
        )
        self.assertEqual(
            missions["dynamic_obstacle_candidate"]["link_ids"],
            ["A2256W000728", "A2256W000599", "A2256W000207", "A2256W000205"],
        )
        monitoring = missions["traffic_light_activation_A236_to_A308"]
        self.assertEqual(monitoring["route_station_start_m"], 160.009167271)
        self.assertEqual(monitoring["route_station_end_m"], 609.876838441)
        self.assertEqual(monitoring["start_link_id"], "A2256W000236")
        self.assertEqual(monitoring["end_link_id"], "A2256W000308")
        self.assertFalse(monitoring["planning_enforcement_allowed"])
        monitoring_payload = json.loads(monitoring["rule_payload_json"])
        self.assertFalse(monitoring_payload["spatial_mission"])
        self.assertEqual(monitoring_payload["policy_scope"], "link_span")
        self.assertEqual(
            monitoring_payload["runtime_activation_hint"], "enable_within_zone_only"
        )
        self.assertTrue(monitoring_payload["disable_outside_zone"])
        traffic = [item for item in missions.values() if item["mission_type"] == "traffic_light"]
        self.assertEqual(len(traffic), 6)
        self.assertTrue(all(item["status"] == "provisional" for item in traffic))
        self.assertTrue(all(item["planning_enforcement_allowed"] is False for item in traffic))
        traffic_boundaries = [
            item
            for item in traffic
            if item["route_station_start_m"] == item["route_station_end_m"]
        ]
        self.assertEqual(len(traffic_boundaries), 5)
        self.assertTrue(
            all(
                monitoring["route_station_start_m"]
                <= item["route_station_start_m"]
                <= monitoring["route_station_end_m"]
                for item in traffic_boundaries
            )
        )

    def test_provisional_mission_cannot_enable_planning(self):
        changed = copy.deepcopy(self.document)
        candidate = next(
            item for item in changed["mission_zones"] if item["status"] == "provisional"
        )
        candidate["planning_enforcement_allowed"] = True
        with self.assertRaisesRegex(CompetitionMetadataError, "provisional mission"):
            validate_competition_metadata(changed)

    def test_missing_authority_status_fails(self):
        changed = copy.deepcopy(self.document)
        changed["mission_zones"][0]["status"] = ""
        with self.assertRaisesRegex(CompetitionMetadataError, "mission status"):
            validate_competition_metadata(changed)

    def test_generated_bundle_is_deterministic_and_current(self):
        rebuilt = build_competition_metadata(self.config, self.repo_root, self.data_root)
        self.assertEqual(dump_metadata_yaml(self.document), dump_metadata_yaml(rebuilt))
        committed = (
            self.repo_root / "src/asmc_hd_map/config/kcity2025_competition_metadata.yaml"
        ).read_text(encoding="utf-8")
        self.assertEqual(committed, dump_metadata_yaml(rebuilt))

    def test_frozen_source_sha_fails_closed(self):
        source_text = self.config.read_text(encoding="utf-8")
        pinned = self.document["sources"]["official_checkpoints"]["sha256"]
        self.assertEqual(source_text.count(pinned), 1)
        with tempfile.TemporaryDirectory() as raw:
            changed_config = Path(raw) / "source.yaml"
            changed_config.write_text(
                source_text.replace(pinned, "0" * 64),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(CompetitionMetadataError, "frozen input SHA changed"):
                build_competition_metadata(changed_config, self.repo_root, self.data_root)


if __name__ == "__main__":
    unittest.main()
