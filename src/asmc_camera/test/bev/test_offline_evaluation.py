#!/usr/bin/env python3

import unittest
from pathlib import Path

import cv2
import numpy as np

from asmc_camera.camera.geometry import compute_letterbox_geometry
from asmc_camera.bev.calibration import load_calibration_snapshot
from asmc_camera.bev.grid import BevGridSpec
from asmc_camera.offline.bev.offline_evaluation import (
    ProjectedProbabilities,
    bev_from_model_homography,
    fuse_probabilities,
    letterbox_valid_mask,
    project_model_grid_hard,
    project_model_grid_probabilities,
)
from asmc_camera.bev.projector import CameraBevProjector


PACKAGE_ROOT = Path(__file__).resolve().parents[2]


class OfflineEvaluationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        _, cameras = load_calibration_snapshot(
            PACKAGE_ROOT / "config" / "bev" / "cameras_v2.yaml"
        )
        cls.camera = cameras["front"]
        cls.grid = BevGridSpec(-5.0, 20.0, -8.0, 8.0, 0.1)
        cls.projector = CameraBevProjector(cls.camera, cls.grid, 22.0, 0.1)
        cls.geometry = compute_letterbox_geometry(
            (cls.camera.height, cls.camera.width), (384, 640)
        )

    def test_composed_homography_matches_native_mapping(self):
        model_points = np.asarray(
            [[[100.0, 100.0]], [[320.0, 300.0]], [[600.0, 350.0]]],
            dtype=np.float64,
        )
        direct = cv2.perspectiveTransform(
            model_points, bev_from_model_homography(self.projector, self.geometry)
        )
        valid = letterbox_valid_mask(self.geometry)
        self.assertEqual(255, int(valid[12, 0]))
        self.assertEqual(0, int(valid[0, 0]))

        left, top, _, _ = self.geometry.pad_ltrb
        scale_x, scale_y = self.geometry.scale_xy
        native = model_points.copy()
        native[:, 0, 0] = (native[:, 0, 0] - left) / scale_x
        native[:, 0, 1] = (native[:, 0, 1] - top) / scale_y
        expected = cv2.perspectiveTransform(
            native, self.projector.homography.bev_from_image
        )
        np.testing.assert_allclose(direct, expected, atol=1e-6)

    def test_soft_projection_is_normalized_and_deterministic(self):
        height, width = self.geometry.target_hw
        drivable = np.full((height, width), 0.75, dtype=np.float32)
        marking = np.zeros((4, height, width), dtype=np.float32)
        marking[0] = 0.1
        marking[1] = 0.6
        marking[2] = 0.2
        marking[3] = 0.1
        first = project_model_grid_probabilities(
            self.projector, self.geometry, drivable, marking
        )
        second = project_model_grid_probabilities(
            self.projector, self.geometry, drivable, marking
        )
        covered = first.coverage > 0
        self.assertTrue(np.any(covered))
        np.testing.assert_allclose(
            first.road_marking_probabilities[:, covered].sum(axis=0),
            1.0,
            atol=1e-6,
        )
        np.testing.assert_array_equal(
            first.road_marking_probabilities, second.road_marking_probabilities
        )
        self.assertTrue(np.all(first.drivable_probability[~covered] == 0.0))

    def test_hard_projection_preserves_class_domain(self):
        height, width = self.geometry.target_hw
        drivable = np.ones((height, width), dtype=np.float32)
        classes = np.zeros((height, width), dtype=np.uint8)
        classes[:, width // 2 :] = 3
        result = project_model_grid_hard(
            self.projector, self.geometry, drivable, classes
        )
        self.assertTrue(set(np.unique(result.road_marking_class_id)).issubset({0, 3}))
        self.assertTrue(np.all(result.road_marking_class_id[result.coverage == 0] == 0))

    def test_soft_fusion_uses_all_class_evidence(self):
        shape = (2, 2)
        front_prob = np.zeros((4, *shape), dtype=np.float32)
        left_prob = np.zeros((4, *shape), dtype=np.float32)
        front_prob[1] = 0.6
        front_prob[2] = 0.4
        left_prob[1] = 0.2
        left_prob[2] = 0.8
        projected = {
            "front": ProjectedProbabilities(
                np.full(shape, 0.2, dtype=np.float32),
                front_prob,
                np.full(shape, 255, dtype=np.uint8),
            ),
            "left": ProjectedProbabilities(
                np.full(shape, 0.8, dtype=np.float32),
                left_prob,
                np.full(shape, 255, dtype=np.uint8),
            ),
        }
        quality = {
            "front": np.ones(shape, dtype=np.float32),
            "left": np.ones(shape, dtype=np.float32),
        }
        result = fuse_probabilities(projected, quality)
        np.testing.assert_allclose(result.drivable_probability, 0.5, atol=1e-6)
        np.testing.assert_array_equal(result.road_marking_class_id, 2)
        np.testing.assert_allclose(
            result.road_marking_probabilities.sum(axis=0), 1.0, atol=1e-6
        )
        np.testing.assert_array_equal(result.source_count, 2)


if __name__ == "__main__":
    unittest.main()
