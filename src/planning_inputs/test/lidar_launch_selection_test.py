#!/usr/bin/env python3
"""Resolve the canonical stack and scenario producer/consumer without a master."""
from pathlib import Path
import unittest
import roslaunch.config
ROOT=Path(__file__).resolve().parents[3]
def config(path,*args):
    return roslaunch.config.load_config_default([(str(ROOT/path),list(args))],None)
class LidarLaunchSelection(unittest.TestCase):
    def test_one_adapter_without_implicit_producer(self):
        c=config('src/asmc_planner/launch/autonomy_no_bridges_v2.launch')
        adapters=[n for n in c.nodes if n.type=='planning_objects_adapter']
        self.assertEqual(len(adapters),1)
        self.assertEqual(c.params['/planning_objects_adapter/input_topic'].value,
                         '/perception/lidar/planning_observations_map')
        self.assertFalse(any(n.type in ('tracked_wire_bridge.py','lidar_pipeline_node') for n in c.nodes))
        self.assertTrue(c.params['/camera_bbox_lane_change/return_to_entry_lane'].value)
    def test_scenario_producer_and_consumer(self):
        p=config('tools/scen_runner/launch/perception.launch')
        pipelines=[n for n in p.nodes if n.type=='lidar_pipeline_node']
        self.assertEqual(len(pipelines),1)
        self.assertEqual(pipelines[0].namespace,'/perception/lidar/')
        c=config('tools/scen_runner/launch/planning.launch')
        self.assertEqual(c.params['/planning_objects_adapter/input_topic'].value,
                         '/perception/lidar/planning_observations_map')
        self.assertTrue(c.params['/camera_bbox_lane_change/return_to_entry_lane'].value)
    def test_explicit_test_namespace_input(self):
        p=config('src/asmc_lidar/launch/lidar.launch','namespace:=/migration/lidar')
        self.assertEqual([n.namespace for n in p.nodes if n.type=='lidar_pipeline_node'],['/migration/lidar/'])
        c=config('src/asmc_planner/launch/autonomy_no_bridges_v2.launch',
                 'lidar_input_topic:=/migration/lidar/planning_observations_map')
        self.assertEqual(c.params['/planning_objects_adapter/input_topic'].value,
                         '/migration/lidar/planning_observations_map')
    def test_collection_uses_the_same_typed_contract(self):
        c=config('src/data_collection/launch/capture_collection_planner.launch')
        self.assertEqual(len([n for n in c.nodes if n.type=='lidar_pipeline_node']),1)
        self.assertEqual(len([n for n in c.nodes if n.type=='planning_objects_adapter']),1)
        self.assertEqual(c.params['/planning_objects_adapter/input_topic'].value,
                         '/perception/lidar/planning_observations_map')
        self.assertEqual(c.params['/planning_objects_adapter/output_topic'].value,'/planning/objects')
if __name__=='__main__':unittest.main()
