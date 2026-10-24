#!/usr/bin/env python3

import json
import copy
from pathlib import Path
import unittest

import cv2
import numpy as np

from asmc_hd_map.compact_drivable import (
    build_compact_drivable,
    canonical_sha256,
    sha256_file,
)


class CompactDrivableWorkspaceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = Path(__file__).resolve().parents[3]
        cls.source = cls.root / "src/asmc_hd_map/config/kcity2025_compact_drivable_source.yaml"
        cls.artifact_path = cls.root / "src/asmc_hd_map/config/kcity2025_compact_drivable_map.json"
        cls.grid_path = cls.root / "src/asmc_hd_map/config/kcity2025_compact_drivable_grid.npz"
        cls.artifact = json.loads(cls.artifact_path.read_text(encoding="utf-8"))
        cls.generated, cls.arrays = build_compact_drivable(cls.root, cls.source)

    def test_frozen_geometry_and_selection(self):
        stats = self.artifact["statistics"]
        self.assertEqual(self.artifact["reviewed_geometry_sha256"],
                         "e836f762dbbad4f02aaa6b37f486439aa02d3d5f961fa72b162cde4ebd24a3b3")
        self.assertEqual(stats["route_lane_count"], 39)
        self.assertEqual(stats["selected_lane_count"], 73)
        self.assertEqual(stats["drivable_area_count"], 115)
        self.assertEqual(stats["checkpoint_centers_inside"], 15)
        self.assertLess(stats["compact_fraction_of_legacy"], 0.20)
        self.assertEqual(self.artifact["grid"]["resolution_m"], 0.2)
        self.assertEqual(self.artifact["selection"]["endpoint_cap_style"], "node_normal_flat")
        self.assertIn("A2256W000427", self.artifact["selection"]["explicit_excluded_link_ids"])
        self.assertNotIn("A2256W000427", self.artifact["selection"]["selected_lane_link_ids"])
        self.assertIn("A2256W000179", self.artifact["selection"]["explicit_excluded_link_ids"])
        self.assertNotIn("A2256W000179", self.artifact["selection"]["selected_lane_link_ids"])
        self.assertEqual(
            self.artifact["selection"]["explicit_lane_groups"]["gps_shadow_opposite_lane"],
            ["A2256W000131"],
        )
        self.assertIn("A2256W000131", self.artifact["selection"]["selected_lane_link_ids"])
        self.assertEqual(stats["boundary_refined_lane_count"], 44)
        self.assertEqual(stats["boundary_refined_side_count"], 80)
        self.assertEqual(stats["corridor_fill_area_count"], 20)
        self.assertEqual(stats["intersection_surface_patch_count"], 14)
        self.assertEqual(
            self.artifact["selection"]["intersection_groups"]["roundabout_maneuver"],
            ["A2256W000134"],
        )
        self.assertIn("A2256W000134", self.artifact["selection"]["selected_lane_link_ids"])

    def test_intersections_are_exactly_route_plus_explicit_allowlist(self):
        selection = self.artifact["selection"]
        route = set(selection["route_link_ids"])
        explicit = {
            link_id
            for values in selection["intersection_groups"].values()
            for link_id in values
        }
        intersection_lanes = {
            lane["link_id"] for lane in self.artifact["lanes"]
            if lane["lane_type"] == "intersection"
        }
        self.assertEqual(intersection_lanes - route, explicit - route)
        self.assertEqual(len(selection["intersection_support_link_ids"]), 22)
        closure = selection["intersection_node_pair_closures"]["dynamic_obstacle_maneuver"]
        self.assertEqual(
            closure["derived_link_ids"],
            ["A2256W000066", "A2256W000087", "A2256W000211", "A2256W001125"],
        )
        self.assertEqual(
            selection["intersection_groups"]["dynamic_obstacle_maneuver"],
            [
                "A2256W000066", "A2256W000087", "A2256W000206", "A2256W000211",
                "A2256W000599", "A2256W000636", "A2256W001124", "A2256W001125",
            ],
        )

    def test_dynamic_junction_vector_union_is_one_connected_surface(self):
        dynamic_ids = set(
            self.artifact["selection"]["intersection_groups"]["dynamic_obstacle_maneuver"]
        )
        polygons = [
            np.asarray(area["polygon"], dtype=np.float64)[:, :2]
            for area in self.artifact["drivable_areas"]
            if set(area["link_ids"]).issubset(dynamic_ids)
            and area.get("surface_group") == "dynamic_obstacle_maneuver"
        ]
        self.assertGreater(len(polygons), 8)
        all_points = np.vstack(polygons)
        resolution = float(self.artifact["grid"]["resolution_m"])
        origin = all_points.min(axis=0) - 1.0
        extent = all_points.max(axis=0) - origin + 1.0
        shape = tuple(np.ceil(extent[::-1] / resolution).astype(int))
        mask = np.zeros(shape, dtype=np.uint8)
        for polygon in polygons:
            pixels = np.rint((polygon - origin[None, :]) / resolution).astype(np.int32)
            cv2.fillPoly(mask, [pixels.reshape((-1, 1, 2))], 1, cv2.LINE_8)
        component_count, _ = cv2.connectedComponents(mask, connectivity=8)
        self.assertEqual(component_count - 1, 1)

    def test_shadow_opposite_lane_has_explicit_surface_bridge(self):
        pairs = self.artifact["selection"]["surface_bridge_pairs"]
        shadow = next(value for value in pairs if value["id"] == "gps_shadow_A126_A131")
        self.assertEqual(shadow["link_ids"], ["A2256W000126", "A2256W000131"])
        self.assertEqual(shadow["source"], "explicit_reviewed_pair")

    def test_roundabout_a134_a151_surface_is_connected_and_center_island_is_preserved(self):
        areas = [
            area for area in self.artifact["drivable_areas"]
            if area.get("surface_group") == "roundabout_maneuver"
        ]
        patches = [area for area in areas if area["id"].startswith("drivable:surface:")]
        self.assertEqual(len(patches), 4)
        self.assertGreater(sum(area["surface_area_m2"] for area in patches), 55.0)
        self.assertTrue(all(area["surface_preserves_holes"] for area in patches))
        self.assertTrue(all(area["surface_envelope_buffer_m"] == 2.0 for area in patches))
        self.assertTrue(all(
            area["surface_supplemental_link_ids"] == ["A2256W000151"]
            for area in patches
        ))
        self.assertTrue(all(area["validity"] == "validated" for area in areas))

        polygons = [np.asarray(area["polygon"], dtype=np.float64)[:, :2] for area in areas]
        points = np.vstack(polygons)
        resolution = float(self.artifact["grid"]["resolution_m"])
        origin = points.min(axis=0) - 1.0
        extent = points.max(axis=0) - origin + 1.0
        shape = tuple(np.ceil(extent[::-1] / resolution).astype(int))
        mask = np.zeros(shape, dtype=np.uint8)
        for polygon in polygons:
            pixels = np.rint((polygon - origin[None, :]) / resolution).astype(np.int32)
            cv2.fillPoly(mask, [pixels.reshape((-1, 1, 2))], 1, cv2.LINE_8)
        component_count, _ = cv2.connectedComponents(mask, connectivity=8)
        self.assertEqual(component_count - 1, 1)

        grid = self.arrays["occupancy"]
        grid_origin = np.asarray([
            self.artifact["grid"]["origin_x"], self.artifact["grid"]["origin_y"]
        ])

        def occupancy(x, y):
            pixel = np.rint((np.asarray([x, y]) - grid_origin) / resolution).astype(int)
            return int(grid[pixel[1], pixel[0]])

        self.assertEqual(occupancy(-102.0, 340.0), 100)
        self.assertEqual(occupancy(-102.97, 325.54), 0)
        self.assertEqual(occupancy(-114.11, 330.54), 0)
        self.assertEqual(occupancy(-89.43, 330.98), 0)

    def test_static_junction_right_turn_corners_are_widened(self):
        lanes = {lane["link_id"]: lane for lane in self.artifact["lanes"]}
        for link_id, extra in (("A2256W000226", 1.0), ("A2256W000111", 1.5)):
            refinement = lanes[link_id]["polygon_refinement"]
            self.assertEqual(refinement["mode"], "mgeo_link_width_reviewed_widening")
            self.assertEqual((refinement["side"], refinement["maximum_extra_m"]), ("right", extra))
            self.assertEqual(refinement["profile"], "half_sine")
        # The outermost southbound A1212 is not a maneuver lane for the static obstacle.
        self.assertNotIn("A2256W001212", lanes)

        grid = self.arrays["occupancy"]
        resolution = float(self.artifact["grid"]["resolution_m"])
        origin = np.asarray([self.artifact["grid"]["origin_x"], self.artifact["grid"]["origin_y"]])

        def occupancy(x, y):
            pixel = np.rint((np.asarray([x, y]) - origin) / resolution).astype(int)
            return int(grid[pixel[1], pixel[0]])

        # Inner-corner pavement past the nominal 3.5 m turn edge is now drivable,
        # while the curb side beyond the reviewed widening stays non-drivable.
        self.assertEqual(occupancy(-54.38, -131.38), 0)
        self.assertEqual(occupancy(-53.72, -130.50), 100)
        self.assertEqual(occupancy(-55.07, -149.22), 0)
        self.assertEqual(occupancy(-54.21, -150.27), 100)
        # The A1212 strip and opposite lanes past the node line stay out.
        self.assertEqual(occupancy(-71.0, -140.0), 100)
        self.assertEqual(occupancy(-67.5, -121.0), 100)

    def test_obstacle_surfaces_are_connected_and_have_no_internal_holes(self):
        for group in ("static_obstacle_maneuver", "dynamic_obstacle_maneuver"):
            link_ids = {
                lane["link_id"] for lane in self.artifact["lanes"]
                if lane["drivable_group"] == group
            }
            polygons = [
                np.asarray(area["polygon"], dtype=np.float64)[:, :2]
                for area in self.artifact["drivable_areas"]
                if area["link_ids"] and set(area["link_ids"]).issubset(link_ids)
            ]
            points = np.vstack(polygons)
            resolution = float(self.artifact["grid"]["resolution_m"])
            origin = points.min(axis=0) - 1.0
            extent = points.max(axis=0) - origin + 1.0
            shape = tuple(np.ceil(extent[::-1] / resolution).astype(int))
            mask = np.zeros(shape, dtype=np.uint8)
            for polygon in polygons:
                pixels = np.rint((polygon - origin[None, :]) / resolution).astype(np.int32)
                cv2.fillPoly(mask, [pixels.reshape((-1, 1, 2))], 1, cv2.LINE_8)
            component_count, _ = cv2.connectedComponents(mask, connectivity=8)
            self.assertEqual(component_count - 1, 1, group)
            _, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
            hole_count = 0 if hierarchy is None else sum(value[3] >= 0 for value in hierarchy[0])
            self.assertEqual(hole_count, 0, group)

    def test_generated_geometry_matches_committed_artifact(self):
        committed = copy.deepcopy(self.artifact)
        generated = copy.deepcopy(self.generated)
        committed.pop("product_sha256", None)
        generated.pop("product_sha256", None)
        committed["grid"].pop("sha256", None)
        self.assertEqual(generated, committed)
        with np.load(str(self.grid_path), allow_pickle=False) as archive:
            self.assertTrue(np.array_equal(self.arrays["occupancy"], archive["occupancy"]))

    def test_hashes_are_self_consistent(self):
        expected_product = canonical_sha256(
            {key: value for key, value in self.artifact.items() if key != "product_sha256"}
        )
        self.assertEqual(self.artifact["product_sha256"], expected_product)
        self.assertEqual(self.artifact["grid"]["sha256"], sha256_file(self.grid_path))

    def test_lane_boundary_ids_reference_reviewed_markings(self):
        reviewed = json.loads((self.root / self.artifact["sources"]["reviewed_lane_markings"]["path"])
                              .read_text(encoding="utf-8"))
        valid = {str(item["id"]) for item in reviewed["features"] if item["feature_type"] == "lane_marking"}
        for lane in self.artifact["lanes"]:
            for side in ("left_boundary_id", "right_boundary_id"):
                if lane[side]:
                    self.assertIn(lane[side], valid)


if __name__ == "__main__":
    unittest.main()
