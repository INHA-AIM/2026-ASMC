#!/usr/bin/env python3
"""Canonical perception ownership and direct typed planning input contract."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import yaml
ROOT=Path(__file__).resolve().parents[3]
class InputContract(unittest.TestCase):
    def test_direct_common_input_without_source_selection_or_producer(self):
        root=ET.parse(ROOT/'src/planning_inputs/launch/objects.launch').getroot()
        self.assertEqual(root.find("arg[@name='input_topic']").get('default'), '/perception/lidar/tracks_map')
        self.assertEqual(len(root.findall('node')),1)
        self.assertEqual(root.find('node').get('required'),'true')
        self.assertEqual(root.findall('.//rosparam'),[])
        self.assertEqual({p.get('name') for p in root.findall('.//param')},{'input_topic','output_topic'})
        launch=ET.parse(ROOT/'src/asmc_planner/launch/autonomy_no_bridges_v2.launch').getroot()
        self.assertEqual(launch.find("arg[@name='lidar_input_topic']").get('default'),'/perception/lidar/tracks_map')
        args={a.get('name') for a in launch.findall('arg')}
        self.assertTrue({'lidar_source_policy','objects_source'}.isdisjoint(args))
        self.assertFalse(any(n.get('pkg')=='asmc_lidar' for n in launch.iter('node')))
    def test_current_filters_stay_in_perception(self):
        config=yaml.safe_load((ROOT/'src/asmc_lidar/config/lidar_pipeline.yaml').read_text())
        self.assertEqual(config['hd_map_audit']['mode'],'filter')
        preview=config['hd_map_audit']['delivery_preview']
        for k,v in {'minimum_road_points':3,'minimum_road_ratio':.1,'minimum_obb_road_ratio':.1,'road_inset_m':.1}.items():
            self.assertEqual(preview[k],v)
        self.assertEqual(config['delivery']['confirmation_frames'],3)
        self.assertEqual(config['delivery']['release_frames'],5)
        source=(ROOT/'src/planning_inputs/src/objects_node.cpp').read_text()
        self.assertNotIn('MotionClassifier',source)
        self.assertNotIn('PlanningObjectFilter',source)
        self.assertNotIn('source_policy',source)
if __name__=='__main__':unittest.main()
