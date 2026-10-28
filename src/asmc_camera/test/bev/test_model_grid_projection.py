#!/usr/bin/env python3

import unittest
from pathlib import Path

import numpy as np

from asmc_camera.bev.calibration import load_calibration_snapshot
from asmc_camera.bev.model_grid_projection import ModelGridGroundProjector


PACKAGE_ROOT = Path(__file__).resolve().parents[2]


class ModelGridProjectionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        _, cls.cameras = load_calibration_snapshot(
            PACKAGE_ROOT / "config" / "bev" / "cameras_v2.yaml"
        )

    def _single_point(self, view, ground_xy, source_mask, model_hw=(384,640)):
        camera = self.cameras[view]
        # Include the x=20 m boundary explicitly for the quantization audit;
        # runtime ROI membership otherwise uses the usual half-open interval.
        projector = ModelGridGroundProjector(
            camera, model_hw=model_hw, source_mask=source_mask, x_bounds_m=(-5.0, 20.5)
        )
        pixel, depth = camera.project_ground([ground_xy])
        self.assertGreater(depth[0], 0.0)
        scale_x, scale_y = projector.geometry.scale_xy
        left, top, _, _ = projector.geometry.pad_ltrb
        column = int(round(pixel[0, 0] * scale_x + left))
        row = int(round(pixel[0, 1] * scale_y + top))
        classes = np.zeros(projector.model_hw, dtype=np.uint8)
        confidence = np.zeros(projector.model_hw, dtype=np.uint8)
        classes[row, column] = 1
        confidence[row, column] = 255
        return projector.extract(classes, confidence, {1: 0.5})

    def test_uint8_sparse_extraction_matches_full_float_path_at_every_confidence(self):
        projector = ModelGridGroundProjector(self.cameras['front'], model_hw=(96, 160))
        indices = np.flatnonzero(projector.valid)
        self.assertGreaterEqual(len(indices), 512)
        classes = np.zeros(projector.model_hw, dtype=np.uint8)
        confidence = np.zeros_like(classes)
        for i, label in enumerate((1, 2)):
            pixels = indices[i * 256:(i + 1) * 256]
            classes.ravel()[pixels] = label
            confidence.ravel()[pixels] = np.arange(256, dtype=np.uint8)
        thresholds = list((np.arange(256, dtype=np.float32) / 255.).astype(float))
        thresholds += [np.nextafter(.5, 0.), .5, np.nextafter(.5, 1.)]
        for threshold in thresholds:
            values = {1: threshold, 2: 1. - threshold}
            full_classes, normalized, masks = projector._selection_masks(classes, confidence, values)
            selected = masks['selected']
            result = projector.extract(classes, confidence, values)
            np.testing.assert_array_equal(result.xy, projector.ground_xy[selected])
            np.testing.assert_array_equal(result.class_id, full_classes[selected])
            np.testing.assert_array_equal(result.confidence, normalized[selected])
            np.testing.assert_array_equal(result.geometric_quality, projector.geometric_quality[selected])
            np.testing.assert_array_equal(result.source_mask, np.full(np.count_nonzero(selected), projector.source_mask, np.uint8))

    def test_sparse_path_preserves_strided_inputs_empty_classes_and_float_validation(self):
        projector = ModelGridGroundProjector(self.cameras['front'], model_hw=(96, 160))
        rng = np.random.RandomState(28)
        classes = rng.randint(0, 4, projector.model_hw).astype(np.uint8)[:, ::-1]
        confidence = rng.randint(0, 256, projector.model_hw).astype(np.uint8)[:, ::-1]
        expected = projector.extract(classes, confidence.astype(np.float32) / 255., {1: .3, 2: .8})
        actual = projector.extract(classes, confidence, {1: .3, 2: .8})
        for key in ('xy', 'class_id', 'confidence', 'geometric_quality', 'source_mask'):
            np.testing.assert_array_equal(getattr(actual, key), getattr(expected, key))
        self.assertEqual(len(projector.extract(classes, confidence, {}, classes=()).xy), 0)
        for threshold in (-.1, 1.1, float('nan')):
            with self.assertRaises(ValueError):
                projector.extract(classes, confidence, {1: threshold})
        outside = tuple(np.argwhere(~projector.valid)[0])
        for value in (float('nan'), -.1, 1.1):
            invalid = confidence.astype(np.float32) / 255.
            invalid[outside] = value
            with self.assertRaises(ValueError):
                projector.extract(classes, invalid, {})
        with self.assertRaises(ValueError):
            projector.extract(classes.astype(np.int16), confidence, {})
        with self.assertRaises(ValueError):
            projector.extract(classes, confidence[:-1], {})

    def test_front_pixel_maps_directly_to_metric_ground(self):
        result = self._single_point("front", (10.0, 1.5), 1)
        self.assertEqual((1, 2), result.xy.shape)
        np.testing.assert_allclose(result.xy[0], [10.0, 1.5], atol=0.15)
        self.assertEqual(1, int(result.source_mask[0]))

    def test_front_960_grid_and_side_640_keep_metric_coordinate_contract(self):
        for view, point, source, shape in [('front',(10.0,1.5),1,(544,960)),
                                            ('left',(2.0,5.0),2,(384,640)),
                                            ('right',(2.0,-5.0),4,(384,640))]:
            result=self._single_point(view,point,source,shape)
            np.testing.assert_allclose(result.xy[0],point,atol=.15)
            self.assertEqual(int(result.source_mask[0]),source)

    def test_side480_native_grid_keeps_both_camera_metric_coordinates(self):
        for view, point, source in [('left',(2.0,5.0),2),('right',(2.0,-5.0),4)]:
            result=self._single_point(view,point,source,(480,640))
            np.testing.assert_allclose(result.xy[0],point,atol=.15)
            self.assertEqual(int(result.source_mask[0]),source)

    def test_front_model_grid_quantization_for_variable_lane_widths(self):
        for width in (2.5, 3.0, 3.5, 4.0):
            for distance in (5.0, 10.0, 15.0, 20.0):
                left_y = 0.35 + 0.57 * width
                right_y = left_y - width
                left = self._single_point("front", (distance, left_y), 1)
                right = self._single_point("front", (distance, right_y), 1)
                self.assertLessEqual(
                    abs(float(left.xy[0, 1] - right.xy[0, 1]) - width), 0.15
                )
                self.assertLessEqual(abs(float(left.xy[0, 0]) - distance), 0.15)
                self.assertLessEqual(abs(float(right.xy[0, 0]) - distance), 0.15)

    def test_side_views_keep_rep103_lateral_sign(self):
        left = self._single_point("left", (2.0, 5.0), 2)
        right = self._single_point("right", (2.0, -5.0), 4)
        self.assertGreater(float(left.xy[0, 1]), 0.0)
        self.assertLess(float(right.xy[0, 1]), 0.0)

    def test_letterbox_padding_is_never_evidence(self):
        projector = ModelGridGroundProjector(self.cameras["left"], source_mask=2)
        classes = np.ones(projector.model_hw, dtype=np.uint8)
        confidence = np.full(projector.model_hw, 255, dtype=np.uint8)
        result = projector.extract(classes, confidence, {1: 0.5})
        self.assertGreater(len(result.xy), 0)
        self.assertTrue(np.all(result.xy[:, 0] >= -5.0))
        self.assertTrue(np.all(result.xy[:, 0] <= 20.0))
        self.assertTrue(np.all(result.xy[:, 1] >= -8.0))
        self.assertTrue(np.all(result.xy[:, 1] <= 8.0))

    def test_selection_audit_separates_confidence_and_projection_loss(self):
        projector = ModelGridGroundProjector(self.cameras["front"], source_mask=1)
        classes = np.zeros(projector.model_hw, dtype=np.uint8)
        confidence = np.zeros(projector.model_hw, dtype=np.uint8)
        valid_row, valid_column = np.argwhere(projector.valid)[0]
        invalid_row, invalid_column = np.argwhere(~projector.valid)[0]
        classes[valid_row, valid_column] = 1
        confidence[valid_row, valid_column] = 100
        classes[invalid_row, invalid_column] = 2
        confidence[invalid_row, invalid_column] = 255
        audit = projector.selection_audit(classes, confidence, {1: 0.5, 2: 0.5})
        self.assertEqual(2, audit["model_marking_pixels"])
        self.assertEqual(1, audit["projection_invalid"])
        self.assertEqual(1, audit["below_confidence"])
        self.assertEqual(0, audit["accepted"])


if __name__ == "__main__":
    unittest.main()
