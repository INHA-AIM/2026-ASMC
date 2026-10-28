#!/usr/bin/env python3
"""Check executable launch contracts and real map artifacts after package moves."""
import pathlib
import unittest
import xml.etree.ElementTree as ET

ROOT = pathlib.Path(__file__).resolve().parents[3]

class LaunchContracts(unittest.TestCase):
    def test_web_console_defaults_are_canonical_and_distinct(self):
        dashboard=ET.parse(ROOT/'src/asmc_planner/launch/dashboard.launch').getroot()
        lidar=ET.parse(ROOT/'src/asmc_lidar/launch/view.launch').getroot()
        args=lambda tree:{arg.get('name'):arg.get('default') for arg in tree.findall('arg')}
        self.assertEqual(args(dashboard)['state_topic'],'/planning/vehicle/state')
        self.assertEqual(args(dashboard)['dashboard_lidar_objects_topic'],'/planning/objects')
        self.assertEqual(args(dashboard)['dashboard_lidar_motion_debug_topic'],'')
        self.assertEqual(args(dashboard)['dashboard_port'],'8088')
        self.assertEqual(args(lidar)['dashboard_port'],'8091')
        self.assertNotEqual(args(dashboard)['dashboard_port'],args(lidar)['dashboard_port'])
        self.assertTrue(all(n.get('type') not in ('lidar_pipeline_node','asmc_planner_node')
                            for tree in (dashboard,lidar) for n in tree.findall('node')))

    def test_integrated_planning_includes_dashboard_and_opener_by_default(self):
        launch=ET.parse(ROOT/'src/asmc_planner/launch/autonomy_no_bridges_v2.launch').getroot()
        args={a.get('name'):a.get('default') for a in launch.findall('arg')}
        self.assertEqual(args['dashboard'],'true')
        self.assertEqual(args['open_dashboard'],'true')
        includes=[node for node in launch.findall('include')
                  if node.get('file','').endswith('/dashboard.launch')]
        self.assertEqual(len(includes),1)
        self.assertEqual(includes[0].get('if'),'$(arg dashboard)')
        forwarding={a.get('name'):a.get('value') for a in includes[0].findall('arg')}
        self.assertEqual(forwarding['open_dashboard'],'$(arg open_dashboard)')
        self.assertEqual(forwarding['dashboard_port'],'$(arg dashboard_port)')

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
