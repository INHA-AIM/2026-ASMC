#!/usr/bin/env python3
"""Runtime asset gate failures must happen before vehicle control starts."""
import json
from pathlib import Path
import tempfile
import unittest
import yaml
from check_highway_merge import check_assets


class HighwayPreflightTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.package = self.root / 'src/asmc_planner'
        self.config = self.package / 'config/frenet/params.yaml'
        self.config.parent.mkdir(parents=True)
        self.map_dir = self.config.parent / 'hd_map'
        self.map_dir.mkdir()
        self.behavior = self.package / 'config/behavior'
        self.behavior.mkdir()
        (self.package / 'src').mkdir()
        (self.package / 'launch').mkdir()
        (self.package / 'src/route.txt').write_text('0 0\n1 0\n')
        self.policy = {'enabled': True, 'zones': [
            {'source_link': 'source', 'target_lanes': ['lane:target']} ]}
        self.settings = {'waypoint_file': 'route.txt', 'avoid_hd_map': {'enabled': True},
                         'planner': {'highway_merge': self.policy}}
        self.write_config()
        (self.map_dir / 'drivable.json').write_text(json.dumps({
            'frame_id': 'map', 'build_id': 'fixture', 'reviewed_geometry_sha256': 'fixture',
            'lanes': [{'id': 'lane:' + name, 'link_id': name, 'centerline': [[0, 0], [10, 0]]}
                      for name in ('source', 'target')], 'drivable_areas': [{}]}))
        (self.map_dir / 'lines.json').write_text('{}')
        (self.map_dir / 'drivable_grid.rle').write_text('fixture')
        behavior = {'map': {'link_set_path': 'src/asmc_planner/config/behavior/links.json',
                            'node_set_path': 'src/asmc_planner/config/behavior/nodes.json'},
                    'route': {'route_links_path': 'src/asmc_planner/config/behavior/route.yaml'}}
        (self.behavior / 'behavior_params.yaml').write_text(yaml.safe_dump(behavior))
        (self.behavior / 'links.json').write_text(json.dumps([{'idx': name, 'points': [[0, 0], [10, 0]]}
            for name in ('source', 'target')]))
        (self.behavior / 'nodes.json').write_text('[{"idx": "node"}]')
        (self.behavior / 'route.yaml').write_text('route:\n  links: [source, target]\n')
        for name in ('planner_v2.launch', 'autonomy_no_bridges_v2.launch'):
            (self.package / 'launch' / name).write_text(
                '<launch><arg name="highway_merge" default="%s"/>' %
                ('true' if name == 'autonomy_no_bridges_v2.launch' else 'false') +
                ('<param name="/frenet_planner/highway_merge_enabled" value="$(arg highway_merge)"/>'
                 if name == 'planner_v2.launch' else
                 '<include file="$(find asmc_planner)/launch/planner_v2.launch">'
                 '<arg name="highway_merge" value="$(arg highway_merge)"/></include>') + '</launch>')

    def write_config(self):
        self.config.write_text(yaml.safe_dump(self.settings))

    def check(self):
        return check_assets(self.root, self.config)

    def test_complete_assets(self):
        self.assertEqual(1, self.check()['zone_count'])

    def test_behavior_map_origin_mismatch(self):
        (self.behavior / 'links.json').write_text(json.dumps([
            {'idx': name, 'points': [[100, 0], [110, 0]]} for name in ('source', 'target')]))
        with self.assertRaisesRegex(ValueError, 'centerline mismatch'):
            self.check()

    def test_missing_official_route(self):
        (self.package / 'src/route.txt').unlink()
        with self.assertRaisesRegex(ValueError, 'missing or empty'):
            self.check()

    def test_missing_or_empty_raster(self):
        (self.map_dir / 'drivable_grid.rle').write_text('')
        with self.assertRaisesRegex(ValueError, 'missing or empty'):
            self.check()

    def test_wrong_behavior_route(self):
        (self.behavior / 'route.yaml').write_text('route:\n  links: [foreign]\n')
        with self.assertRaisesRegex(ValueError, 'Behavior route'):
            self.check()

    def test_target_missing_from_behavior_map(self):
        self.policy['zones'][0]['target_lanes'] = ['lane:foreign']
        self.write_config()
        with self.assertRaisesRegex(ValueError, 'target link missing'):
            self.check()

    def test_lane_change_missing_behavior_link(self):
        self.policy['lane_change'] = {'enabled': True, 'lanes': [
            ['lane:source', 'lane:target'], ['lane:foreign', 'lane:target']]}
        self.write_config()
        with self.assertRaisesRegex(ValueError, 'lane-change link missing'):
            self.check()

    def test_lane_change_cannot_choose_dead_end_successor(self):
        self.policy['lane_change'] = {'enabled': True, 'lanes': [
            ['lane:source', 'lane:target'], ['lane:source', 'lane:dead_end']]}
        self.write_config()
        with self.assertRaisesRegex(ValueError, 'explicit common lane'):
            self.check()

    def test_disabled_road_map(self):
        self.settings['avoid_hd_map']['enabled'] = False
        self.write_config()
        with self.assertRaisesRegex(ValueError, 'avoid_hd_map'):
            self.check()

    def test_missing_wrapper_forwarding(self):
        (self.package / 'launch/autonomy_no_bridges_v2.launch').write_text(
            '<launch><arg name="highway_merge" default="true"/></launch>')
        with self.assertRaisesRegex(ValueError, 'does not forward'):
            self.check()

    def test_standalone_planner_stays_opt_in(self):
        (self.package / 'launch/planner_v2.launch').write_text(
            '<launch><arg name="highway_merge" default="true"/></launch>')
        with self.assertRaisesRegex(ValueError, 'default=false'):
            self.check()

    def test_autonomy_defaults_on(self):
        path = self.package / 'launch/autonomy_no_bridges_v2.launch'
        path.write_text(path.read_text().replace('default="true"', 'default="false"'))
        with self.assertRaisesRegex(ValueError, 'default=true'):
            self.check()


if __name__ == '__main__':
    unittest.main()
