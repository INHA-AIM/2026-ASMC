#!/usr/bin/env python3

import unittest

from asmc_perception.offline.bev.invalid_support_review import (
    HEALTHY_REVIEW_CATEGORY,
    InvalidSupportReviewSampler,
    invalid_support_review_categories,
)


def classification(**flags):
    defaults = {
        "vehicle_near_leading_tail": False,
        "preview_trailing_tail": False,
        "whole_support_shrink": False,
        "internal_support_gap": False,
        "preview_entry_missing": False,
    }
    defaults.update(flags)
    return {"flags": defaults}


class InvalidSupportReviewTest(unittest.TestCase):
    def test_categories_preserve_multi_label_frames(self):
        value = invalid_support_review_categories(
            classification(
                vehicle_near_leading_tail=True,
                whole_support_shrink=True,
            )
        )
        self.assertEqual(
            ("vehicle_near_leading_tail", "whole_support_shrink"), value
        )

    def test_no_material_frame_is_an_explicit_control_sample(self):
        self.assertEqual(
            (HEALTHY_REVIEW_CATEGORY,),
            invalid_support_review_categories(classification()),
        )

    def test_sampler_applies_quota_and_spacing_per_category(self):
        sampler = InvalidSupportReviewSampler(limit_per_category=2, min_frame_gap=10)
        value = classification(preview_trailing_tail=True)
        self.assertEqual(("preview_trailing_tail",), sampler.select(0, value))
        self.assertEqual((), sampler.select(5, value))
        self.assertEqual(("preview_trailing_tail",), sampler.select(10, value))
        self.assertEqual((), sampler.select(30, value))
        self.assertEqual(2, sampler.summary()["preview_trailing_tail"])

    def test_zero_limit_disables_review_without_side_effect(self):
        sampler = InvalidSupportReviewSampler(limit_per_category=0)
        self.assertEqual(
            (), sampler.select(0, classification(internal_support_gap=True))
        )
        self.assertEqual(0, sampler.summary()["internal_support_gap"])

    def test_invalid_sampler_configuration_is_rejected(self):
        with self.assertRaises(ValueError):
            InvalidSupportReviewSampler(limit_per_category=-1)
        with self.assertRaises(ValueError):
            InvalidSupportReviewSampler(min_frame_gap=-1)


if __name__ == "__main__":
    unittest.main()
