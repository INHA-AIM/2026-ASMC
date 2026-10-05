#!/usr/bin/env python3
"""Verify default source, pinned assets and unchanged validated filter policy."""
import pathlib
import unittest
import xml.etree.ElementTree as ET
import yaml
ROOT = pathlib.Path(__file__).resolve().parents[3]
PKG = ROOT / 'src/planning_inputs'
class FilterContract(unittest.TestCase):
    def test_raw_source_and_required_adapter(self):
        root = ET.parse(PKG / 'launch/objects.launch').getroot()
        self.assertEqual(root.find("arg[@name='input_topic']").get('default'),
                         '/lidar/tracked_objects_map')
        node = root.find("node[@name='planning_objects_adapter']")
        self.assertEqual(node.get('required'), 'true')
        launch = ET.parse(ROOT / 'src/behavior_planner/launch/autonomy_no_bridges_v2.launch').getroot()
        self.assertEqual(launch.find("arg[@name='lidar_input_topic']").get('default'),
                         '/lidar/tracked_objects_map')
        for name in ('drivable_mask_image', 'drivable_mask_metadata',
                     'known_static_catalog', 'static_structure_exclusion_config'):
            relative = node.find("param[@name='%s']" % name).get('value')
            path = pathlib.Path(relative.replace('$(find planning_inputs)', str(PKG)))
            self.assertTrue(path.is_file(), str(path))
    def test_validated_catalog_and_policy(self):
        params = yaml.safe_load((PKG / 'config/object_filter_params.yaml').read_text())
        self.assertEqual(params['drivable_margin_m'], 0.0)
        self.assertEqual(params['drivable_ratio_threshold'], 0.15)
        self.assertEqual(params['known_static_center_tolerance_m'], 2.5)
        for name in ('drivable_filter_enabled','known_static_filter_enabled',
                     'static_structure_exclusion_enabled'):
            self.assertTrue(params[name])
        folder = PKG / 'config/validated_0781402'
        metadata = yaml.safe_load((folder / 'road_mask.yaml').read_text())
        catalog = yaml.safe_load((folder / 'known_static_objects.yaml').read_text())
        self.assertEqual(metadata['map_id'], params['drivable_map_id'])
        self.assertEqual(metadata['local_origin_in_global'],
                         catalog['local_origin_in_global'])
        self.assertTrue(catalog['observations'])
        self.assertEqual((folder / 'road_mask.png').read_bytes()[:8], b'\x89PNG\r\n\x1a\n')
if __name__ == '__main__':
    unittest.main()
