#!/usr/bin/env python3

import tempfile
import unittest
from pathlib import Path

import yaml

from asmc_perception.offline.bev.bag_batch import (
    diagnostic_values,
    load_bag_selection,
    result_quality_gates,
    topic_message_count,
)


class BagBatchTest(unittest.TestCase):
    def make_selection(self, root, missing=None):
        run_id = "20260912_010000_kcity2025_shadow_clear_sunny_11am_wheel"
        raw = root / "raw" / run_id
        (raw / "bags").mkdir(parents=True)
        (raw / "bags" / "run_0.bag").write_bytes(b"bag")
        (raw / "metadata.yaml").write_text(
            yaml.safe_dump(
                {
                    "status": "complete",
                    "scenario": "shadow_clear",
                    "environment_initial": {"weather": "sunny", "sim_hour": 11},
                }
            )
        )
        selection = root / "selection.yaml"
        selection.write_text(
            yaml.safe_dump(
                {
                    "schema_version": "asmc-rosbag-selection-1.0.0",
                    "missing_combinations": missing or [],
                    "selected_runs": [
                        {
                            "run_id": run_id,
                            "scenario": "shadow_clear",
                            "weather": "sunny",
                            "sim_hour": 11,
                            "recorded_controller": "morai_auto",
                            "effective_controller": "wheel",
                            "duration_s": 20.0,
                        }
                    ],
                }
            )
        )
        return selection

    def test_selection_preserves_operator_controller_correction(self):
        with tempfile.TemporaryDirectory() as value:
            root = Path(value)
            _, runs = load_bag_selection(self.make_selection(root), root)
            self.assertEqual(1, len(runs))
            self.assertEqual("morai_auto", runs[0].recorded_controller)
            self.assertEqual("wheel", runs[0].effective_controller)

    def test_selection_rejects_missing_matrix_entries(self):
        with tempfile.TemporaryDirectory() as value:
            root = Path(value)
            path = self.make_selection(root, [{"weather": "foggy"}])
            with self.assertRaisesRegex(ValueError, "missing combinations"):
                load_bag_selection(path, root)

    def test_selection_scope_detects_an_unreported_missing_condition(self):
        with tempfile.TemporaryDirectory() as value:
            root = Path(value)
            path = self.make_selection(root)
            document = yaml.safe_load(path.read_text())
            document["scope"] = {
                "scenarios": ["shadow_clear"],
                "allowed_weather": ["sunny", "foggy"],
                "allowed_sim_hours": [11],
            }
            path.write_text(yaml.safe_dump(document))
            with self.assertRaisesRegex(ValueError, "matrix differs"):
                load_bag_selection(path, root)

    def test_diagnostic_values_finds_status_across_topics(self):
        value = diagnostic_values(
            {
                "diagnostics_final": {
                    "/perception/lane/diagnostics": {
                        "drivable_bev/lane_vectorizer": {
                            "values": {"outputs": "42"}
                        }
                    }
                }
            },
            "drivable_bev/lane_vectorizer",
        )
        self.assertEqual({"outputs": "42"}, value)

    def test_result_quality_gates_separate_empty_candidates_from_pipeline_failure(self):
        topics = (
            "/perception/camera/diagnostics",
            "/perception/bev/diagnostics",
            "/perception/lane/diagnostics",
            "/perception/lane/boundaries",
        )
        metadata = {
            "status": "complete",
            "topic_statistics": {topic: {"messages": 4} for topic in topics},
            "diagnostics_final": {
                "/perception/lane/diagnostics": {
                    "drivable_bev/lane_vectorizer": {
                        "values": {
                            "outputs": "42",
                            "errors": "0",
                            "last_counts": '{"shadow_boundaries": 0}',
                        }
                    }
                }
            },
        }
        gates = result_quality_gates(metadata)
        self.assertTrue(all(gates.values()))
        self.assertEqual(4, topic_message_count(metadata, topics[0]))

    def test_result_quality_gates_reject_missing_lane_output(self):
        metadata = {
            "status": "complete",
            "topic_statistics": {},
            "diagnostics_final": {},
        }
        gates = result_quality_gates(metadata)
        self.assertFalse(gates["lane_outputs_positive"])
        self.assertFalse(gates["lane_boundaries_recorded"])

    def test_candidate_quality_gate_is_opt_in_and_checks_typed_topic(self):
        metadata = {
            "status": "complete",
            "topic_statistics": {},
            "diagnostics_final": {},
        }
        ordinary = result_quality_gates(metadata)
        self.assertNotIn("boundary_family_recorded", ordinary)
        required = result_quality_gates(metadata, require_boundary_family=True)
        self.assertFalse(required["boundary_family_recorded"])
        metadata["topic_statistics"]["/perception/lane/boundary_family"] = {
            "messages": "7"
        }
        required = result_quality_gates(metadata, require_boundary_family=True)
        self.assertTrue(required["boundary_family_recorded"])
        self.assertTrue(required["lane_boundaries_recorded"])


if __name__ == "__main__":
    unittest.main()
