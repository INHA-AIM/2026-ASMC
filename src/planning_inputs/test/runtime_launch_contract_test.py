#!/usr/bin/env python3
"""Check executable launch contracts and real map artifacts after package moves."""
import pathlib
import unittest
import xml.etree.ElementTree as ET

ROOT = pathlib.Path(__file__).resolve().parents[3]

class LaunchContracts(unittest.TestCase):
    def test_planning_never_starts_a_camera_signal_producer(self):
        for name in ('autonomy_no_bridges_v2.launch', 'mode_control_integration.launch'):
            tree = ET.parse(ROOT / 'src/asmc_planner/launch' / name)
            args = {arg.get('name') for arg in tree.iter('arg')}
            self.assertTrue({'traffic_light', 'traffic_light_checkpoint',
                             'traffic_light_device', 'camera_topic'}.isdisjoint(args), name)
            self.assertFalse(any(node.get('pkg') in ('asmc_camera', 'traffic_light')
                                 for node in tree.iter('node')), name)
        import yaml
        behavior = yaml.safe_load((ROOT / 'src/asmc_planner/config/behavior/behavior_params.yaml').read_text())
        self.assertEqual(behavior['traffic_light']['typed_topic'], '/perception/traffic_light/detection')
        self.assertEqual(behavior['traffic_light']['topic'], '/traffic_light/state')

    def test_canonical_hd_map_bundle(self):
        import json, hashlib, yaml
        package=ROOT/'src/asmc_lidar'
        config=yaml.safe_load((package/'config/lidar_pipeline.yaml').read_text())['hd_map_audit']
        manifest=package/config['manifest']
        self.assertEqual(hashlib.sha256(manifest.read_bytes()).hexdigest(),config['manifest_sha256'])
        self.assertTrue(json.loads(manifest.read_text()))

if __name__ == '__main__':
    unittest.main()
