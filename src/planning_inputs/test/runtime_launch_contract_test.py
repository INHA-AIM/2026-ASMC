#!/usr/bin/env python3
"""Check executable launch contracts and real map artifacts after package moves."""
import pathlib
import unittest
import xml.etree.ElementTree as ET

ROOT = pathlib.Path(__file__).resolve().parents[3]

class LaunchContracts(unittest.TestCase):
    def test_standalone_signal_producer(self):
        tree = ET.parse(ROOT / 'src/behavior_planner/launch/autonomy_no_bridges_v2.launch')
        args = {arg.get('name'): arg.get('default') for arg in tree.findall('arg')}
        self.assertEqual(args['traffic_light'], 'true')
        group = tree.find("group[@if='$(arg traffic_light)']")
        node = group.find("node[@name='traffic_light_infer']")
        self.assertEqual(node.get('pkg'), 'asmc_camera')
        self.assertEqual(node.get('required'), 'true')
        self.assertIn('--image-topic $(arg camera_topic)', node.get('args'))
        self.assertIn('--typed-topic /perception/traffic_light/detection', node.get('args'))
        self.assertTrue((ROOT / 'src/asmc_camera/scripts' / node.get('type')).is_file())

    def test_nested_lidar_mask_artifacts(self):
        package = ROOT / 'src/lidar+sensorfusion/src/Main'
        tree = ET.parse(package / 'main.launch')
        params = {p.get('name'): p.get('value') for p in tree.iter('param')}
        for name in ('drivable_mask_image', 'drivable_mask_metadata'):
            resolved = pathlib.Path(params[name].replace('$(find Main)', str(package))).resolve()
            resolved.relative_to(ROOT.resolve())
            self.assertTrue(resolved.is_file(), str(resolved))
        self.assertEqual(pathlib.Path(params['drivable_mask_image']).suffix, '.png')
        self.assertEqual(pathlib.Path(params['drivable_mask_metadata']).suffix, '.yaml')

if __name__ == '__main__':
    unittest.main()
