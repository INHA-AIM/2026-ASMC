#!/usr/bin/env python3

import importlib.util
import os
import unittest


SCRIPT = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__), "..", "..", "..", "scripts",
        "verify_morai_topics.py",
    )
)
SPEC = importlib.util.spec_from_file_location("verify_morai_topics", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class RequiredTopicsTest(unittest.TestCase):
    def test_participant_runtime_does_not_require_competition_status(self):
        profile, topics = MODULE.default_required_topics("competition")
        self.assertEqual("competition", profile)
        self.assertNotIn("/Competition_topic", topics)
        self.assertIn("/vehicle/state", topics)

    def test_field_gate_explicitly_requires_competition_status(self):
        _, topics = MODULE.default_required_topics(
            "competition", require_competition_status=True
        )
        self.assertIn("/Competition_topic", topics)

    def test_development_alias_adds_oracles_without_field_status(self):
        profile, topics = MODULE.default_required_topics("gt")
        self.assertEqual("development", profile)
        self.assertIn("/Object_topic", topics)
        self.assertIn("/velodyne_points_instance", topics)
        self.assertNotIn("/Competition_topic", topics)


if __name__ == "__main__":
    unittest.main()
