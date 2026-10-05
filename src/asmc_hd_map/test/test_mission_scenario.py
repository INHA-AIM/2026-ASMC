#!/usr/bin/env python3
"""Mission scenario spec properties.

These tests derive every expectation from the current metadata and scenario
source, so editing a zone boundary or an actor link list does not require
editing this file.
"""

import math
from pathlib import Path
import random
import tempfile
import unittest

import yaml

from asmc_hd_map.mission_scenario import (
    ACTOR_ROLES,
    DEFAULT_SOURCE_REL,
    MissionScenarioSpec,
    build_mission_scenario_spec,
)
from asmc_hd_map.runtime_bundle import polyline_stations

REPO_ROOT = Path(__file__).resolve().parents[3]
METADATA_PATH = REPO_ROOT / "src/asmc_hd_map/config/kcity2025_competition_metadata.yaml"


def _distance_to_polyline(point, polyline):
    best = math.inf
    for start, end in zip(polyline, polyline[1:]):
        dx, dy = end[0] - start[0], end[1] - start[1]
        length_sq = dx * dx + dy * dy
        fraction = 0.0 if length_sq == 0.0 else max(
            0.0, min(1.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_sq)
        )
        best = min(best, math.hypot(point[0] - start[0] - fraction * dx, point[1] - start[1] - fraction * dy))
    return best


class MissionScenarioSpecTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.spec = MissionScenarioSpec.load(REPO_ROOT)
        cls.metadata = yaml.safe_load(METADATA_PATH.read_text(encoding="utf-8"))
        cls.source = yaml.safe_load((REPO_ROOT / DEFAULT_SOURCE_REL).read_text(encoding="utf-8"))

    def test_every_metadata_zone_is_in_the_spec_with_its_span(self):
        by_id = {zone["id"]: zone for zone in self.spec.zones()}
        for mission in self.metadata["mission_zones"]:
            zone = by_id[mission["id"]]
            self.assertEqual(zone["route_station_start_m"], mission["route_station_start_m"])
            self.assertEqual(zone["route_station_end_m"], mission["route_station_end_m"])
            self.assertEqual(zone["route_link_ids"], mission["link_ids"])

    def test_committed_inputs_build_without_reference_warnings(self):
        self.assertEqual(self.spec.warnings, [])

    def test_configured_zones_resolve_actor_sources(self):
        for zone_id, entry in (self.source.get("zones") or {}).items():
            configured_roles = {item["role"] for item in entry.get("actor_sources") or []}
            self.assertTrue(configured_roles.issubset(ACTOR_ROLES), zone_id)
            self.assertEqual(set(self.spec.actor_roles(zone_id)), configured_roles, zone_id)

    def test_sampled_poses_lie_on_their_actor_link_with_link_heading(self):
        rng = random.Random(7)
        for zone in self.spec.zones():
            for role in self.spec.actor_roles(zone["id"]):
                for _ in range(5):
                    pose = self.spec.sample_actor_pose(zone["id"], role, rng, lateral_jitter_m=0.4)
                    source = next(
                        item for item in zone["actor_sources"]
                        if item["role"] == role and item["link_id"] == pose["link_id"]
                    )
                    distance = _distance_to_polyline((pose["x"], pose["y"]), source["centerline"])
                    self.assertLessEqual(distance, 0.4 + 0.05, pose)
                    centerline = source["centerline"]
                    stations = polyline_stations(centerline)
                    self.assertLessEqual(pose["link_station_m"], stations[-1])
                    index = max(
                        i for i in range(len(stations) - 1)
                        if stations[i] <= pose["link_station_m"] + 1e-9
                    )
                    start, end = centerline[index], centerline[min(index + 1, len(centerline) - 1)]
                    heading = math.degrees(math.atan2(end[1] - start[1], end[0] - start[0]))
                    delta = (pose["yaw_deg"] - heading + 180.0) % 360.0 - 180.0
                    self.assertLess(abs(delta), 30.0, pose)
                    if source["in_compact_product"] and abs(pose["lateral_offset_m"]) < 1.0:
                        self.assertTrue(pose["in_compact_drivable"], pose)

    def test_sampling_is_reproducible_for_a_seed(self):
        zone = next(item for item in self.spec.zones() if item["actor_sources"])
        role = zone["actor_sources"][0]["role"]
        first = [self.spec.sample_actor_pose(zone["id"], role, random.Random(3)) for _ in range(3)]
        second = [self.spec.sample_actor_pose(zone["id"], role, random.Random(3)) for _ in range(3)]
        self.assertEqual(first, second)

    def test_surface_zones_have_one_connected_outline_set(self):
        for zone in self.spec.zones():
            if zone["surface"] is None:
                continue
            self.assertGreater(zone["surface"]["area_m2"], 0.0, zone["id"])
            self.assertTrue(zone["surface"]["outlines"], zone["id"])

    def test_route_pose_round_trips_through_route_station(self):
        for station in (10.0, 900.0, 1950.0):
            pose = self.spec.route_pose(station)
            projected = self.spec.route_station(pose["x"], pose["y"])
            self.assertAlmostEqual(projected["station_m"], station, delta=0.05)
            self.assertLess(projected["lateral_error_m"], 0.05)
            left = self.spec.route_pose(station, 1.5)
            self.assertAlmostEqual(math.hypot(left["x"] - pose["x"], left["y"] - pose["y"]), 1.5, delta=1e-6)

    def test_reference_errors_are_warnings_not_failures(self):
        source = yaml.safe_load((REPO_ROOT / DEFAULT_SOURCE_REL).read_text(encoding="utf-8"))
        source["zones"]["roundabout_candidate"]["actor_sources"].append(
            {"role": "npc_vehicle", "link_ids": ["A2256W999999"]}
        )
        source["zones"]["no_such_zone"] = {"actor_sources": []}
        with tempfile.TemporaryDirectory() as raw:
            path = Path(raw) / "source.yaml"
            path.write_text(yaml.safe_dump(source, allow_unicode=True), encoding="utf-8")
            document = build_mission_scenario_spec(REPO_ROOT, path)
        self.assertTrue(any("A2256W999999" in value for value in document["warnings"]))
        self.assertTrue(any("no_such_zone" in value for value in document["warnings"]))


if __name__ == "__main__":
    unittest.main()
