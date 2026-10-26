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
        self.assertIn("/Ego_topic", topics)
        self.assertNotIn("/Competition_topic", topics)

    def test_development_alias_adds_oracles_without_field_status(self):
        profile, topics = MODULE.default_required_topics("gt")
        self.assertEqual("development", profile)
        self.assertIn("/Object_topic", topics)
        self.assertIn("/velodyne_points_instance", topics)
        self.assertNotIn("/Competition_topic", topics)


    def test_lan_requires_9099_and_forbids_detail_and_gt(self):
        _, topics = MODULE.default_required_topics('competition', True, 'lan2lan')
        self.assertIn('/Ego_topic', topics)
        self.assertNotIn('/Competition_topic', topics)
        self.assertIn('/Competition_topic', MODULE.forbidden_topics('competition', 'lan2lan'))
        self.assertNotIn('/Ego_topic', MODULE.forbidden_topics('competition', 'lan2lan'))
        with self.assertRaises(ValueError):
            MODULE.default_required_topics('gt', transport_profile='lan2lan')

    def test_overrides_cannot_replace_9099_with_detail(self):
        topics = MODULE.resolve_required_topics((), ['/gps'], 'lan2lan', True)
        self.assertIn('/Ego_topic', topics)
        with self.assertRaises(ValueError):
            MODULE.resolve_required_topics((), ['/Competition_topic'], 'lan2lan', True)


if __name__ == "__main__":
    unittest.main()
