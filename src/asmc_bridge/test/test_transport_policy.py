#!/usr/bin/env python3
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from asmc_bridge.transport_policy import ego_status_enabled, detail_status_enabled


class TransportPolicyTest(unittest.TestCase):
    def test_local_and_lan_keep_9099_enabled(self):
        for transport in ('local', 'lan2lan'):
            self.assertTrue(ego_status_enabled(transport))
            self.assertTrue(ego_status_enabled(transport, True))
        self.assertFalse(ego_status_enabled('local', False))

    def test_lan_cannot_disable_competition_or_enable_detail(self):
        with self.assertRaises(ValueError):
            ego_status_enabled('lan2lan', False)
        with self.assertRaises(ValueError):
            detail_status_enabled('lan2lan', True)
        with self.assertRaises(ValueError):
            ego_status_enabled('lan2lan', True, 9798)
        self.assertTrue(ego_status_enabled('local', True, 8000))
        self.assertFalse(detail_status_enabled('lan2lan'))
        self.assertTrue(detail_status_enabled('local'))
        self.assertFalse(detail_status_enabled('local', False))

    def test_unknown_transport_is_rejected(self):
        for function in (ego_status_enabled, detail_status_enabled):
            with self.assertRaises(ValueError):
                function('unknown')


if __name__ == '__main__':
    unittest.main()
