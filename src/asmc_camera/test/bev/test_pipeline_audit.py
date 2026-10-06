#!/usr/bin/env python3

import unittest

from asmc_camera.bev.pipeline_audit import (
    PipelineAuditAccumulator,
    primary_outcome,
    rejection_counts,
    summarize_pipeline_frames,
    summarize_pipeline_run_groups,
)


def frame(**counts):
    defaults = {
        "model_marking_pixels": 100,
        "evidence": 80,
        "nodes": 12,
        "tracks": 3,
        "prefit_tracks": 3,
        "lanes": 3,
        "observations": 3,
        "boundaries": 3,
        "yellow_boundaries": 1,
        "complete": True,
    }
    defaults.update(counts)
    return {"counts": defaults}


class PipelineAuditTest(unittest.TestCase):
    def test_primary_outcome_reports_first_empty_stage(self):
        self.assertEqual(primary_outcome(frame(evidence=0)), "no_metric_evidence")
        self.assertEqual(primary_outcome(frame(tracks=0)), "no_supported_track")
        self.assertEqual(
            primary_outcome(frame(yellow_boundaries=0, complete=False)),
            "boundaries_without_yellow",
        )
        self.assertEqual(primary_outcome(frame()), "complete_boundary_family")

    def test_accumulator_tracks_outage_streaks(self):
        accumulator = PipelineAuditAccumulator()
        accumulator.update(frame(boundaries=0, yellow_boundaries=0, complete=False))
        value = accumulator.update(
            frame(boundaries=2, yellow_boundaries=0, complete=False)
        )
        self.assertEqual(value["longest_streak_frames"]["no_boundary"], 1)
        self.assertEqual(value["longest_streak_frames"]["no_yellow"], 2)
        value = accumulator.update(frame())
        self.assertEqual(value["current_streak_frames"]["no_yellow"], 0)
        self.assertEqual(value["outcomes"]["complete_boundary_family"], 1)

    def test_rejection_counts_are_stage_qualified(self):
        value = frame()
        value["projection"] = {
            "front": {"projection_invalid": 3, "below_confidence": 2}
        }
        value["association"] = {"rejected": {"short_span": 4}}
        counts = rejection_counts(value)
        self.assertEqual(counts["projection.front.projection_invalid"], 3)
        self.assertEqual(counts["association.short_span"], 4)

    def test_association_direct_rejections_are_not_omitted(self):
        value = frame()
        value["association"] = {
            "candidate_pairs": 30,
            "rejected_lateral_residual": 12,
            "rejected_heading_change": 3,
            "assignment_conflicts": 2,
        }
        counts = rejection_counts(value)
        self.assertEqual(12, counts["association.lateral_residual"])
        self.assertEqual(3, counts["association.heading_change"])
        self.assertEqual(2, counts["association.assignment_conflict"])

    def test_summary_reports_frame_incidence_and_rates(self):
        first = frame()
        first["association"] = {"rejected_lateral_residual": 8}
        second = frame(complete=False)
        second["association"] = {"rejected_lateral_residual": 2}
        value = summarize_pipeline_frames([first, second])
        self.assertEqual(10, value["rejections"]["association.lateral_residual"])
        self.assertEqual(
            2, value["rejection_frame_counts"]["association.lateral_residual"]
        )
        self.assertAlmostEqual(
            1.0, value["rejection_frame_rates"]["association.lateral_residual"]
        )
        self.assertAlmostEqual(
            0.5, value["outcome_rates"]["complete_boundary_family"]
        )
        self.assertEqual(
            1,
            value["rejection_frame_counts_by_outcome"]
            ["complete_boundary_family"]["association.lateral_residual"],
        )
        self.assertEqual(
            1,
            value["rejection_frame_counts_by_outcome"]
            ["partial_boundary_family"]["association.lateral_residual"],
        )
        self.assertAlmostEqual(
            1.0,
            value["rejection_frame_rates_by_outcome"]
            ["partial_boundary_family"]["association.lateral_residual"],
        )

    def test_recorded_frames_are_reaggregated_from_zero(self):
        value = summarize_pipeline_frames(
            [
                frame(boundaries=0, yellow_boundaries=0, complete=False),
                frame(),
            ]
        )
        self.assertTrue(value["available"])
        self.assertEqual(2, value["frames"])
        self.assertEqual(2, value["frame_messages"])
        self.assertEqual(1, value["outcomes"]["no_tracked_boundary"])
        self.assertEqual(1, value["outcomes"]["complete_boundary_family"])

    def test_no_recorded_pipeline_audit_is_explicit(self):
        value = summarize_pipeline_frames([])
        self.assertFalse(value["available"])
        self.assertEqual(0, value["frames"])

    def test_runtime_stage_timings_are_aggregated(self):
        first = frame()
        first["runtime_profile_schema"] = "asmc-lane-runtime-profile-1.0.0"
        first["timing_ms"] = {"projection_audit": 2.0, "callback_total": 10.0}
        second = frame()
        second["runtime_profile_schema"] = "asmc-lane-runtime-profile-1.0.0"
        second["timing_ms"] = {"projection_audit": 4.0, "callback_total": 14.0}
        value = summarize_pipeline_frames([first, second])
        profile = value["runtime_profile"]
        self.assertEqual(
            {"asmc-lane-runtime-profile-1.0.0": 2}, profile["schemas"]
        )
        self.assertEqual(2, profile["stages_ms"]["projection_audit"]["count"])
        self.assertAlmostEqual(
            3.0, profile["stages_ms"]["projection_audit"]["p50"]
        )
        self.assertAlmostEqual(
            13.8, profile["stages_ms"]["callback_total"]["p95"]
        )

    def test_run_group_summary_does_not_join_outage_streaks(self):
        outage = frame(boundaries=0, yellow_boundaries=0, complete=False)
        value = summarize_pipeline_run_groups([[outage], [outage]])
        self.assertEqual(2, value["runs"])
        self.assertEqual(2, value["frames"])
        self.assertEqual(1, value["longest_streak_frames"]["no_boundary"])


if __name__ == "__main__":
    unittest.main()
