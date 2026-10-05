import unittest
import numpy as np
from metrics import topology, actor_shape, frame_metrics, regions


class ShapeMetricsTest(unittest.TestCase):
    def test_intact_wall_vs_scattered_equal_count(self):
        points = np.column_stack((np.arange(100)*.1, np.zeros(100), np.zeros(100)))
        contiguous = np.ones(100, np.uint8); contiguous[:50] = 2
        fragmented = np.ones(100, np.uint8); fragmented[::2] = 2
        r = topology(points, {"intact": contiguous, "fragmented": fragmented})
        a, b = r["intact"]["0.2"], r["fragmented"]["0.2"]
        self.assertEqual(a["retained_points"], b["retained_points"])
        self.assertLess(a["deletion_preservation_edges"], b["deletion_preservation_edges"])
        self.assertLessEqual(a["added_split_parts"], b["added_split_parts"])

    def test_no_claim_that_full_deletion_is_good(self):
        points = np.column_stack((np.zeros(12), np.zeros(12), np.arange(12)*.1))
        mask = np.full(12, 2); mask[:4] = 1
        r = actor_shape(points, mask)
        self.assertEqual(r["observed_height_bands"]["lower"]["retained_points"], 0)
        self.assertFalse(r["complete_ground_loss"])
        self.assertTrue(actor_shape(points, np.ones(12))["complete_ground_loss"])

    def test_preexisting_sparse_connections_are_not_filter_splits(self):
        points = np.array([[0,0,0], [5,0,0], [10,0,0]])
        r = topology(points, {"all": np.full(3,2)})["all"]["0.8"]
        self.assertEqual(r["input_components"], 3)
        self.assertEqual(r["added_split_parts"], 0)
        self.assertEqual(r["newly_isolated_points"], 0)

    def test_instance_zero_is_not_an_object(self):
        xyz = np.column_stack((np.arange(12)*.1, np.zeros(12), np.zeros(12), np.ones(12)))
        r = frame_metrics(xyz, np.full(12,109), np.zeros(12), {"x": np.full(12,2)}, 0)
        self.assertEqual(r["x"]["objects"], [])
        self.assertIn("standing_object", r["x"]["classes"])
        self.assertNotIn("wall", r["x"]["classes"])

    def test_regions_and_unclassified(self):
        p = np.column_stack((np.arange(12)*.1, np.zeros(12), np.zeros(12)))
        self.assertEqual(regions(p, np.array([1]*6+[2]*6))["partial_regions"], 1)
        self.assertEqual(actor_shape(p, np.full(12,3))["unclassified_points"], 12)
        self.assertFalse(actor_shape(p, np.full(12,3))["complete_ground_loss"])
        self.assertEqual(regions(p,np.full(12,3))["entirely_removed_regions"],0)
        self.assertEqual(regions(p,np.full(12,3))["regions_with_unclassified_points"],1)

    def test_roll_changes_observed_lower_band_not_lineage(self):
        cloud = np.column_stack((np.zeros(12), np.arange(12)*.1,
                                 np.zeros(12), np.ones(12))).astype(np.float32)
        masks = {"candidate": np.array([1]*6+[2]*6, np.uint8)}
        semantic = np.full(12, 125, np.uint16)
        instance = np.ones(12, np.uint16)
        original = frame_metrics(cloud, semantic, instance, masks, 0.0)
        pitch_only = frame_metrics(cloud, semantic, instance, masks, 0.0, 0.0)
        corrected = frame_metrics(cloud, semantic, instance, masks, 0.0, np.pi/2)
        self.assertEqual(original, pitch_only)
        old = original["candidate"]["objects"][0]
        new = corrected["candidate"]["objects"][0]
        self.assertEqual(old["retained_points"], new["retained_points"])
        self.assertGreater(old["observed_height_bands"]["lower"]["retained_points"], 0)
        self.assertEqual(new["observed_height_bands"]["lower"]["retained_points"], 0)

if __name__ == "__main__":
    unittest.main()
