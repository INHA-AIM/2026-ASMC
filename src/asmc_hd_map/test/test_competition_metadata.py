import copy
import hashlib
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

    def test_route_speed_limits_cap_mgeo_by_the_competition_profile(self):
        limits = {item["link_id"]: item for item in self.document["route_speed_limits"]}
        self.assertEqual(list(limits), self.document["route"]["link_ids"])
        for link_id in ("A2256W000202", "A2256W000315"):
            self.assertEqual(limits[link_id]["mgeo_max_speed_kph"], 70.0)
            self.assertEqual(limits[link_id]["effective_limit_kph"], 60.0)
        for link_id in ("A2256W000411", "A2256W000420", "A2256W000408", "A2256W000445", "A2256W000153"):
            self.assertIsNone(limits[link_id]["competition_cap_kph"])
            self.assertEqual(limits[link_id]["speed_zone_id"], "high_speed_exception")
        self.assertEqual(limits["A2256W000153"]["effective_limit_kph"], 50.0)
        self.assertTrue(all(
            item["effective_limit_kph"] <= 60.0
            for item in limits.values() if item["competition_cap_kph"] is not None
        ))
        self.assertEqual(self.document["validation"]["route_speed_links_capped_below_mgeo"], 2)

    def test_route_speed_limit_tampering_fails_closed(self):
        changed = copy.deepcopy(self.document)
        changed["route_speed_limits"][9]["effective_limit_kph"] = 70.0
        with self.assertRaisesRegex(CompetitionMetadataError, "min\\(MGeo, cap\\)"):
            validate_competition_metadata(changed)

    def test_rule_v1_2_payloads(self):
        missions = {item["id"]: item for item in self.document["mission_zones"]}
        shadow = json.loads(missions["gps_shadow_candidate"]["rule_payload_json"])
        self.assertEqual(shadow["random_mission_types"], ["obstacle_avoidance"])
        self.assertIs(shadow["lane_keeping_penalty"], False)
        boundaries = [item for item in self.document["mission_zones"]
                      if item["mission_type"] == "traffic_light"
                      and item["route_station_start_m"] == item["route_station_end_m"]]
        self.assertEqual(len(boundaries), 5)
        for item in boundaries:
            payload = json.loads(item["rule_payload_json"])
            self.assertEqual(payload["judgement"], "signal_when_front_wheels_cross_stopline")
            self.assertEqual(payload["violation_penalty_s"], 30)
            self.assertEqual(payload["judged_intersection_count"], 4)
            self.assertEqual(payload["run_failure_condition"], "all_judged_intersections_violated")
            self.assertTrue(payload["count_same_intersection_once"])
            self.assertFalse(payload["reset_count_on_checkpoint_return"])
            self.assertIsNone(payload["judged_intersection_id"])
            self.assertFalse(payload["intersection_failure_evaluation_available"])
            self.assertNotIn("red_without_deceleration", payload)

    def test_rule_update_does_not_move_route_checkpoints_or_missions(self):
        # Spatial fingerprint of main c971d6b, before the scoring-only update.
        fields = ("id", "mission_type", "route_station_start_m", "route_station_end_m",
                  "start_link_id", "end_link_id", "link_ids", "polygon")
        spatial = {"route": self.document["route"], "checkpoints": self.document["checkpoints"],
                   "missions": [{k: m[k] for k in fields} for m in self.document["mission_zones"]]}
        digest = hashlib.sha256(json.dumps(spatial, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
        self.assertEqual(digest, "e8f98cfcb406be98202913642f599bf85605eb0e64f2e77ba8ffcb872d3795f0")

    def test_whole_route_rules_are_exposed_even_in_uncapped_speed_zone(self):
        rules = self.document["route_rules"]
        self.assertEqual(rules["run"]["time_limit_s"], 900)
        self.assertEqual(rules["lane_keeping"]["penalty_s"], 5)
        self.assertEqual(rules["lane_keeping"]["sustained_contact_repeat_s"], 3)
        self.assertEqual(rules["lane_keeping"]["wheel_contact_count_min"], 1)
        self.assertEqual(rules["lane_keeping"]["exempt_zone_ids"], ["gps_shadow_candidate"])
        self.assertFalse(rules["collision"]["continuous_contact_additional_penalty"])
        self.assertTrue(rules["collision"]["recollision_additional_penalty"])
        carriers = [m for m in self.document["mission_zones"]
                    if m["mission_type"] in ("speed_limit", "high_speed_exception")]
        self.assertEqual(len(carriers), 3)
        for mission in carriers:
            self.assertEqual(json.loads(mission["rule_payload_json"])["route_rules"], rules)

    def test_scoring_tampering_is_rejected(self):
        mutations = [
            ("traffic_light_C1256W000077", "boundary_only", False),
            ("traffic_light_C1256W000077", "violation_penalty_s", 15),
            ("traffic_light_C1256W000077", "red_without_deceleration", "run_failure"),
            ("traffic_light_C1256W000077", "judged_intersection_count", 5),
            ("traffic_light_C1256W000077", "reset_count_on_checkpoint_return", True),
            ("traffic_light_C1256W000077", "violation_count_unit", "event"),
            ("traffic_light_C1256W000077", "count_same_intersection_once", False),
            ("traffic_light_C1256W000077", "judged_intersection_id", "guessed_A219"),
            ("traffic_light_C1256W000077", "intersection_failure_evaluation_available", True),
            ("gps_shadow_candidate", "random_mission_types", ["static_obstacle"]),
            ("gps_shadow_candidate", "lane_keeping_penalty", True),
            ("roundabout_candidate", "collision_penalty_s", 0),
            ("static_obstacle_candidate", "continuous_contact_additional_penalty", True),
            ("speed_limit_60_before_high_speed", "overspeed_penalty_s", 30),
            ("speed_limit_60_after_high_speed", "sustained_overspeed_repeat_s", 5),
            ("start_success_5_percent", "time_limit_s", 900),
            ("high_speed_exception", "route_rules", {}),
        ]
        for mission_id, field, value in mutations:
            with self.subTest(mission=mission_id, field=field):
                changed = copy.deepcopy(self.document)
                mission = next(m for m in changed["mission_zones"] if m["id"] == mission_id)
                payload = json.loads(mission["rule_payload_json"])
                payload[field] = value
                mission["rule_payload_json"] = json.dumps(payload)
                with self.assertRaises(CompetitionMetadataError):
                    validate_competition_metadata(changed)

    def test_missing_or_changed_global_rules_are_rejected(self):
        for section, field, value in (("run", "time_limit_s", 600),
                                      ("lane_keeping", "penalty_s", 15),
                                      ("lane_keeping", "wheel_contact_count_min", 4),
                                      ("checkpoints", "approved_recovery_penalty_s", 30)):
            with self.subTest(section=section, field=field):
                changed = copy.deepcopy(self.document)
                changed["route_rules"][section][field] = value
                with self.assertRaises(CompetitionMetadataError):
                    validate_competition_metadata(changed)
        changed = copy.deepcopy(self.document)
        del changed["route_rules"]
        with self.assertRaises(CompetitionMetadataError):
            validate_competition_metadata(changed)

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
            "team_final": 11,
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
        # Obstacle zones include approach and recovery distance around their
        # junctions; the obstacle itself is expected on the junction links.
        static = missions["static_obstacle_candidate"]
        self.assertEqual(static["link_ids"], ["A2256W000213", "A2256W000222", "A2256W000220"])
        self.assertAlmostEqual(static["route_station_start_m"], 0.5 * (237.422 + 315.906), places=2)
        self.assertAlmostEqual(static["route_station_end_m"], 365.775, places=2)
        self.assertEqual(
            json.loads(static["rule_payload_json"])["obstacle_expected_link_ids"], ["A2256W000222"]
        )
        dynamic = missions["dynamic_obstacle_candidate"]
        self.assertEqual(
            json.loads(dynamic["rule_payload_json"])["obstacle_expected_link_ids"],
            ["A2256W000728", "A2256W000599", "A2256W000207", "A2256W000205"],
        )
        self.assertLess(dynamic["route_station_start_m"], 427.851)
        self.assertGreater(dynamic["route_station_end_m"], 482.819)
        monitoring = missions["traffic_light_activation_A236_to_A308"]
        self.assertEqual(monitoring["route_station_start_m"], 160.009167271)
        self.assertEqual(monitoring["route_station_end_m"], 609.876838441)
        self.assertEqual(monitoring["start_link_id"], "A2256W000236")
        self.assertEqual(monitoring["end_link_id"], "A2256W000308")
        self.assertTrue(monitoring["planning_enforcement_allowed"])
        monitoring_payload = json.loads(monitoring["rule_payload_json"])
        self.assertFalse(monitoring_payload["spatial_mission"])
        self.assertEqual(monitoring_payload["policy_scope"], "link_span")
        self.assertEqual(
            monitoring_payload["runtime_activation_hint"], "enable_within_zone_only"
        )
        self.assertTrue(monitoring_payload["disable_outside_zone"])
        traffic = [item for item in missions.values() if item["mission_type"] == "traffic_light"]
        self.assertEqual(len(traffic), 6)
        self.assertTrue(all(item["status"] == "team_final" for item in traffic))
        self.assertTrue(all(item["planning_enforcement_allowed"] is True for item in traffic))
        for item in missions.values():
            if item["status"] == "team_final":
                payload = json.loads(item["rule_payload_json"])
                self.assertEqual(payload["authority"], "team_final")
                self.assertTrue(item["planning_enforcement_allowed"])
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
            item for item in changed["mission_zones"] if item["status"] == "team_final"
        )
        candidate["status"] = "provisional"
        self.assertTrue(candidate["planning_enforcement_allowed"])
        with self.assertRaisesRegex(CompetitionMetadataError, "provisional mission"):
            validate_competition_metadata(changed)

    def test_team_final_mission_keeps_its_authority_tag(self):
        changed = copy.deepcopy(self.document)
        candidate = next(
            item for item in changed["mission_zones"] if item["status"] == "team_final"
        )
        payload = json.loads(candidate["rule_payload_json"])
        payload.pop("authority")
        candidate["rule_payload_json"] = json.dumps(payload, sort_keys=True)
        with self.assertRaisesRegex(CompetitionMetadataError, "authority tag"):
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
