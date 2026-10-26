#!/usr/bin/env python3
"""Resolve launch configurations without starting nodes or contacting MORAI."""
from pathlib import Path
import unittest
import roslaunch.config

ROOT = Path(__file__).resolve().parents[3]


def config(path, *args):
    return roslaunch.config.load_config_default([(str(ROOT / path), list(args))], None)


class LidarLaunchSelection(unittest.TestCase):
    def test_each_policy_has_exactly_one_object_adapter(self):
        for policy, executable, topic in (
                ('asmc_lidar', 'planning_objects_adapter', '/perception/lidar/tracks_map'),
                ('asmc_lidar_companion', 'asmc_lidar_planning_adapter', '/migration/lidar/planning_observations_map'),
                ('validated_lidar', 'planning_objects_adapter', '/planning/input/tracked_objects')):
            with self.subTest(policy=policy):
                resolved = config('src/asmc_planner/launch/autonomy_no_bridges_v2.launch',
                                  'lidar_source_policy:=' + policy)
                adapters = [n for n in resolved.nodes if n.type in
                            ('planning_objects_adapter', 'asmc_lidar_planning_adapter')]
                self.assertEqual([n.type for n in adapters], [executable])
                self.assertEqual(resolved.params['/' + adapters[0].name + '/input_topic'].value, topic)
                wires = [n for n in resolved.nodes if n.type == 'tracked_wire_bridge.py']
                self.assertEqual(len(wires), int(policy == 'validated_lidar'))
                self.assertTrue(resolved.params['/camera_bbox_lane_change/return_to_entry_lane'].value)

    def test_companion_wrappers_preserve_producer_consumer_pair(self):
        resolved = config('src/asmc_planner/launch/autonomy_asmc_lidar.launch')
        pipeline = [n for n in resolved.nodes if n.type == 'lidar_pipeline_node']
        self.assertEqual(len(pipeline), 1)
        self.assertEqual(pipeline[0].namespace, '/migration/lidar/')
        self.assertTrue(resolved.params['/migration/lidar/lidar_pipeline/planning_output'].value)
        self.assertEqual(resolved.params['/asmc_lidar_planning_adapter/input_topic'].value,
                         '/migration/lidar/planning_observations_map')
        perception = config('tools/scen_runner/launch/perception.launch', 'lidar_backend:=asmc_lidar')
        planning = config('tools/scen_runner/launch/planning.launch', 'lidar_backend:=asmc_lidar')
        self.assertTrue(perception.params['/migration/lidar/lidar_pipeline/planning_output'].value)
        self.assertEqual(planning.params['/asmc_lidar_planning_adapter/input_topic'].value,
                         '/migration/lidar/planning_observations_map')
        self.assertTrue(planning.params['/camera_bbox_lane_change/return_to_entry_lane'].value)


if __name__ == '__main__':
    unittest.main()
